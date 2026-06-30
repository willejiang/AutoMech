#!/usr/bin/env python3
"""The cadam SCAD worker — ONE worker builds the WHOLE model's geometry.

Replaces makerv2's per-link FreeCAD worker (worker.py + freecad_runner.py). One
LLM call emits a single .scad with one top-level `module <link>()` per link; we
render each module to meshes/<link>.stl via the native OpenSCAD CLI, validate
with check_stl, and (bounded) retry the modules that failed.

Shares the same LLM client/gateway the manager uses (settings.worker_client()).
"""
from __future__ import annotations

import os
from pathlib import Path

from .llm.client import LLMError
from .llm.conversation import Conversation
from .model import KinematicModel, RunContext, WorkerResult
from .prompts.scad_worker_prompt import (SCAD_WORKER_SYSTEM,
                                         build_scad_worker_user,
                                         build_scad_worker_retry)
from .scad_render import find_openscad, parse_modules, render_module_err
from .validation import check_stl


def _strip_fences(text: str) -> str:
    t = (text or "").strip()
    if t.startswith("```"):
        # drop first fence line and trailing fence
        body = t.split("\n", 1)[1] if "\n" in t else ""
        if body.rstrip().endswith("```"):
            body = body.rstrip()[:-3]
        return body.strip()
    return t


def _render_and_check(oscad, scad_path, model, ctx):
    """Render every link's module, validate each. Returns
    (results: list[WorkerResult], failed: list[(name, err)])."""
    results, failed = [], []
    present = set(parse_modules(scad_path.read_text(encoding="utf-8", errors="replace")))
    for link in model.links:
        stl = Path(ctx.meshes_dir) / f"{link.name}.stl"
        if link.name not in present:
            err = f"module {link.name}() not defined in the .scad"
            results.append(WorkerResult(link_name=link.name, success=False,
                                        abs_mesh_path=str(stl), error=err))
            failed.append((link.name, err))
            continue
        ok, rerr = render_module_err(oscad, scad_path, link.name, stl)
        rep = check_stl(str(stl)) if ok else None
        good = bool(ok and rep and rep.exists and rep.loadable and not rep.degenerate)
        err = "" if good else (rerr or (rep.error if rep else "render failed"))
        results.append(WorkerResult(link_name=link.name, success=good,
                                    abs_mesh_path=str(stl), error=err,
                                    stl_report=rep))
        if not good:
            failed.append((link.name, err))
    return results, failed


def build_all(model: KinematicModel, ctx: RunContext, settings,
              log_fn=print) -> list[WorkerResult]:
    """Generate the whole-model .scad via cadam, render per-link STLs, validate,
    and retry failures. Returns one WorkerResult per link."""
    oscad = find_openscad()
    if not oscad:
        return [WorkerResult(link_name=l.name, success=False,
                             error="OpenSCAD CLI not found (set OPENSCAD_BIN)")
                for l in model.links]

    scad_path = Path(ctx.run_dir) / "model.scad"
    # Worker model: defaults to the manager's model (claude-opus-4.8). Override
    # with MAKER2_WORKER_MODEL if a faster model is wanted for the heavy
    # whole-model generation.
    import copy
    wsettings = copy.copy(settings)
    wsettings.model = os.environ.get("MAKER2_WORKER_MODEL", settings.model)
    client = wsettings.worker_client()
    log_fn(f"[worker] model = {wsettings.model}")
    conv = Conversation()
    conv.add_user_message(build_scad_worker_user(model))

    attempts = getattr(settings, "worker_retries", 1) + 1
    results: list[WorkerResult] = []
    for attempt in range(1, attempts + 1):
        messages = conv.get_messages_for_api(api_style=client.api_style)
        try:
            text = client.send(messages, system=SCAD_WORKER_SYSTEM)
        except LLMError as e:
            log_fn(f"[worker] LLM error (attempt {attempt}): {e}")
            if attempt < attempts:
                continue
            return [WorkerResult(link_name=l.name, success=False,
                                 error=f"LLM error: {e}") for l in model.links]

        scad = _strip_fences(text)
        scad_path.write_text(scad, encoding="utf-8")
        conv.add_assistant_message(text)

        results, failed = _render_and_check(oscad, scad_path, model, ctx)
        built = sum(1 for r in results if r.success)
        log_fn(f"[worker] attempt {attempt}: {built}/{len(results)} links built")

        if not failed:
            return results
        if attempt < attempts:
            conv.add_user_message(build_scad_worker_retry(failed))

    return results
