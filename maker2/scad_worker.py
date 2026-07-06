#!/usr/bin/env python3
"""The cadam SCAD worker — build a model's geometry in PARALLEL BATCHES.

Each link needs one top-level `module <link>()` in OpenSCAD; a downstream tool
renders each module in isolation and assembles them via the joints. The OLD design
asked for the WHOLE model in ONE LLM call, which overran the gateway's ~16000
output cap on a 6-7 link subassembly (→ discarded + retried ~3x). This version
splits the links into small BATCHES (~3) and generates them CONCURRENTLY, in WAVES:
- each batch = one small LLM call (well under the cap) that defines only its links,
- batches in a wave run in parallel (ThreadPool, up to settings.max_workers),
- waves run in order so a later batch's prompt can name the parts EARLIER waves
  already built (coherence) plus the peers it's built alongside.
Per built STL we emit an ARTIFACT_JSON `mesh_progress` line so the UI can render the
model piece-by-piece as parts land. All batches' modules are concatenated into
model.scad at the end (the GLB/render path does `use <model.scad>`).

Shares the manager's LLM client/gateway (settings.worker_client()).
"""
from __future__ import annotations

import json
import os
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from .llm.client import LLMError
from .llm.conversation import Conversation
from .model import KinematicModel, RunContext, WorkerResult
from .prompts.scad_worker_prompt import (SCAD_WORKER_SYSTEM,
                                         build_scad_worker_batch,
                                         build_scad_worker_batch_retry,
                                         build_scad_worker_continue)
from .scad_render import (find_openscad, parse_modules, render_module_err,
                          trim_to_complete_modules)
from .validation import check_stl


# ONE worker per subassembly: a single call generates EVERY module for the model, so
# one mind holds all the parts at once (parallel workers reason independently and put
# parts in each other's space -> interpenetration). Overflow is handled by
# completion-continuation (send_collect + trim_to_complete_modules), not by splitting
# the work across parallel workers. With a large-output model (gpt-5.5, 128K) a whole
# subassembly fits comfortably. Set very high so all links land in ONE batch.
_BATCH_SIZE = 100000
_MAX_CONTINUE = 8          # continuation rounds per generation (all links in one call)


def _strip_fences(text: str) -> str:
    t = (text or "").strip()
    if t.startswith("```"):
        body = t.split("\n", 1)[1] if "\n" in t else ""
        if body.rstrip().endswith("```"):
            body = body.rstrip()[:-3]
        return body.strip()
    return t


def _render_link(oscad, scad_path, link, ctx) -> WorkerResult:
    """Render + validate one link's module from the given .scad file."""
    stl = Path(ctx.meshes_dir) / f"{link.name}.stl"
    ok, rerr = render_module_err(oscad, scad_path, link.name, stl)
    rep = check_stl(str(stl)) if ok else None
    good = bool(ok and rep and rep.exists and rep.loadable and not rep.degenerate)
    err = "" if good else (rerr or (rep.error if rep else "render failed"))
    return WorkerResult(link_name=link.name, success=good,
                        abs_mesh_path=str(stl), error=err, stl_report=rep)


def _emit_progress(link_name: str, built: int, total: int, run_dir: str, lock) -> None:
    """Emit a bare ARTIFACT_JSON mesh_progress line (NOT via log_fn — the stream
    route matches lines that START with ARTIFACT_JSON:, so no [sub:id] prefix).
    Serialized so parallel threads don't interleave the line."""
    with lock:
        print("ARTIFACT_JSON:" + json.dumps({
            "kind": "mesh_progress", "link": link_name,
            "built": built, "total": total, "run_dir": run_dir}), flush=True)


def _generate_batch_scad(client, conv, want_names, log_fn, tag) -> str:
    """Generate a batch's .scad with COMPLETION-CONTINUATION. Streams via
    send_collect; on a cap truncation (finish=='length') keep the COMPLETE modules,
    drop the half-written tail, and ask the model to continue with the modules still
    missing — concatenating until all wanted modules are present, the model stops,
    or the continuation budget runs out. Returns the accumulated .scad text."""
    accum = ""
    for round_i in range(_MAX_CONTINUE + 1):
        messages = conv.get_messages_for_api(api_style=client.api_style)
        text, finish = client.send_collect(messages, system=SCAD_WORKER_SYSTEM)
        piece = _strip_fences(text)
        complete, partial = trim_to_complete_modules(piece)
        # Keep only whole modules; a truncated tail is discarded and re-requested.
        accum = (accum + "\n" + complete).strip() if accum else complete.strip()
        conv.add_assistant_message(text)                 # model sees its own partial
        present = set(parse_modules(accum))
        missing = [n for n in want_names if n not in present]
        if not missing:
            return accum
        if finish != "length":
            # Model stopped on its own but didn't define everything — one nudge to
            # finish the rest (counts against the budget) then give up if still short.
            if round_i >= _MAX_CONTINUE:
                return accum
        else:
            log_fn(f"[worker] {tag}: output capped mid-generation; continuing "
                   f"({len(present)} module(s) so far, need {missing})")
        conv.add_user_message(build_scad_worker_continue(missing))
    return accum


def _build_batch(idx, batch, model, done, peers, oscad, ctx, settings, client,
                 counter, total, emit_lock, log_fn, research_note: str = ""):
    """Generate + render ONE batch (its own small .scad), retrying its failed
    modules. Returns list[WorkerResult] for the batch's links. Emits mesh_progress
    per built STL. Thread-safe (own conversation + own batch .scad file)."""
    names = [l.name for l in batch]
    batch_scad = Path(ctx.run_dir) / f"_batch_{idx}.scad"
    conv = Conversation()
    if research_note:
        conv.add_user_message(
            "Reference facts from a web search (use where relevant for real "
            f"dimensions/specs):\n{research_note[:4000]}")
    conv.add_user_message(build_scad_worker_batch(model, batch, done, peers))
    attempts = getattr(settings, "worker_retries", 1) + 1
    results: dict = {}

    for attempt in range(1, attempts + 1):
        try:
            scad = _generate_batch_scad(client, conv, names, log_fn, f"batch {idx}")
        except LLMError as e:
            log_fn(f"[worker] batch {idx} LLM error (attempt {attempt}): {e}")
            if attempt < attempts:
                continue
            for l in batch:
                results.setdefault(l.name, WorkerResult(
                    link_name=l.name, success=False, error=f"LLM error: {e}"))
            break
        batch_scad.write_text(scad, encoding="utf-8")
        present = set(parse_modules(scad))
        failed: list = []
        for l in batch:
            if l.name not in present:
                r = WorkerResult(link_name=l.name, success=False,
                                 abs_mesh_path=str(Path(ctx.meshes_dir)/f"{l.name}.stl"),
                                 error=f"module {l.name}() not defined")
            else:
                r = _render_link(oscad, batch_scad, l, ctx)
            prev = results.get(l.name)
            results[l.name] = r
            if r.success and (prev is None or not prev.success):
                with emit_lock:
                    counter[0] += 1
                    n = counter[0]
                _emit_progress(l.name, n, total, ctx.run_dir, emit_lock)
            if not r.success:
                failed.append((l.name, r.error))
        if not failed:
            break
        if attempt < attempts:
            conv.add_user_message(build_scad_worker_batch_retry(failed))

    return [results[l.name] for l in batch]


def build_all(model: KinematicModel, ctx: RunContext, settings,
              log_fn=print) -> list[WorkerResult]:
    """Generate the model's geometry in parallel batches, render + validate each
    link, and combine into model.scad. Returns one WorkerResult per link (in the
    model's link order)."""
    oscad = find_openscad()
    if not oscad:
        return [WorkerResult(link_name=l.name, success=False,
                             error="OpenSCAD CLI not found (set OPENSCAD_BIN)")
                for l in model.links]

    import copy
    wsettings = copy.copy(settings)
    wsettings.model = os.environ.get("MAKER2_WORKER_MODEL", settings.model)
    client = wsettings.worker_client()
    total = len(model.links)
    log_fn(f"[worker] model = {wsettings.model} | {total} links in "
           f"batches of {_BATCH_SIZE}")

    # Optional web-search research (gated by settings.enable_reference_tools): look up
    # standard part dimensions ONCE for this subassembly and thread the findings into
    # each batch's prompt as context.
    from .tools import research_findings
    research_note = research_findings(client, wsettings, model.name, log_fn=log_fn)

    # Split links into batches, then group batches into waves of `max_workers`.
    batches = [model.links[i:i + _BATCH_SIZE]
               for i in range(0, total, _BATCH_SIZE)]
    wave_size = max(1, min(len(batches), getattr(settings, "max_workers", 4)))

    counter = [0]                     # built-so-far (shared, lock-guarded)
    emit_lock = threading.Lock()
    all_results: dict = {}
    done_names: list = []             # parts finished by prior waves (context)

    for w in range(0, len(batches), wave_size):
        wave = batches[w:w + wave_size]
        wave_names = [l.name for b in wave for l in b]

        def _work(bi_batch):
            bi, batch = bi_batch
            peers = [n for n in wave_names if n not in {l.name for l in batch}]
            return bi, _build_batch(bi, batch, model, list(done_names), peers,
                                    oscad, ctx, wsettings, client, counter, total,
                                    emit_lock, log_fn, research_note=research_note)

        indexed = list(enumerate(wave, start=w))
        if len(wave) == 1:
            bi, res = _work(indexed[0])
            for r in res:
                all_results[r.link_name] = r
        else:
            with ThreadPoolExecutor(max_workers=len(wave)) as pool:
                futs = [pool.submit(_work, ib) for ib in indexed]
                for f in as_completed(futs):
                    _, res = f.result()
                    for r in res:
                        all_results[r.link_name] = r
        done_names.extend(wave_names)
        built = sum(1 for n in wave_names if all_results.get(n) and all_results[n].success)
        log_fn(f"[worker] wave {w // wave_size + 1}: {built}/{len(wave_names)} "
               f"links built ({counter[0]}/{total} total)")

    # Combine every batch's .scad into model.scad so the GLB/render path resolves.
    combined = []
    for i in range(len(batches)):
        bp = Path(ctx.run_dir) / f"_batch_{i}.scad"
        if bp.exists():
            combined.append(f"// ---- batch {i} ----\n"
                            + bp.read_text(encoding="utf-8", errors="replace"))
    (Path(ctx.run_dir) / "model.scad").write_text("\n\n".join(combined),
                                                  encoding="utf-8")

    results = [all_results.get(l.name) or WorkerResult(
        link_name=l.name, success=False, error="not built") for l in model.links]
    built = sum(1 for r in results if r.success)
    log_fn(f"[worker] done: {built}/{total} links built")
    return results
