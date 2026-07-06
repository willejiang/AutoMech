#!/usr/bin/env python3
"""The CadQuery worker — build a model's geometry as Python, one function per link.

Drop-in replacement for scad_worker.build_all: same (model, ctx, settings) ->
list[WorkerResult] contract, same mesh_progress ARTIFACT emission, same
completion-continuation on an output-cap truncation. The difference is the backend:
an LLM writes a CadQuery Python script with one `build_<link>()` per link (each
returns a `cq.Workplane` solid in MM at its local origin), and each function is
EXECUTED IN A SUBPROCESS (sandboxed: clean env, cwd = run dir, a timeout, no
network) that exports `meshes/<link>.stl`. Validation is the SAME validation.check_stl
gate the SCAD path used, so the URDF/scale contract is unchanged.

Why CadQuery: curved geometry (fillets/lofts/sweeps/splines) that OpenSCAD only
approximated. Why a subprocess: executing model-authored Python is arbitrary code
(same trust boundary as OpenSCAD running generated .scad) and cadquery/OCCT can hard-
crash the interpreter on bad input — a subprocess isolates that and enforces a timeout.

Each part's generating script is persisted to `<run>/cq/<link>.py` so a later
localized fault can EDIT that part's script line-by-line (build_cq_worker_edit)
instead of regenerating the whole subassembly.
"""
from __future__ import annotations

import ast
import copy
import json
import os
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from .llm.client import LLMError
from .llm.conversation import Conversation
from .model import KinematicModel, RunContext, WorkerResult
from .prompts.cq_worker_prompt import (CQ_WORKER_SYSTEM,
                                       build_cq_worker_batch,
                                       build_cq_worker_batch_retry,
                                       build_cq_worker_continue)
from .validation import check_stl


# ONE worker per subassembly (see scad_worker): a single call holds every part in
# mind so parts don't collide. Overflow -> completion-continuation, not parallel
# splitting. Set very high so all links land in ONE batch.
_BATCH_SIZE = 100000
_MAX_CONTINUE = 8          # continuation rounds per generation
_EXEC_TIMEOUT = 180        # seconds per part-export subprocess


def _strip_fences(text: str) -> str:
    t = (text or "").strip()
    if t.startswith("```"):
        body = t.split("\n", 1)[1] if "\n" in t else ""
        if body.rstrip().endswith("```"):
            body = body.rstrip()[:-3]
        return body.strip()
    return t


def parse_functions(script: str) -> list[str]:
    """The link names a script defines: top-level `def build_<name>()` -> <name>.
    Uses ast; on a syntax error (a truncated tail) falls back to a line scan so the
    continuation loop can still tell what's present."""
    names: list[str] = []
    try:
        tree = ast.parse(script)
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name.startswith("build_"):
                names.append(node.name[len("build_"):])
        return names
    except SyntaxError:
        import re
        for m in re.finditer(r"^def\s+build_([A-Za-z_]\w*)\s*\(", script, re.MULTILINE):
            names.append(m.group(1))
        return names


def trim_to_complete_functions(script: str) -> tuple[str, str]:
    """Split a (possibly truncated) script into (complete_prefix, trailing_partial).

    Parses with ast and keeps every top-level statement that fully parses; the tail
    the model was mid-writing when the output cap cut it off is dropped and
    re-requested. The Python analogue of scad_render.trim_to_complete_modules. If the
    whole thing parses, the tail is empty; if nothing parses, progressively drop
    trailing lines until the prefix parses."""
    try:
        ast.parse(script)
        return script, ""
    except SyntaxError:
        pass
    lines = script.splitlines(keepends=True)
    # Find the largest line-prefix that parses cleanly.
    lo, hi, best = 0, len(lines), 0
    while lo <= hi:
        mid = (lo + hi) // 2
        prefix = "".join(lines[:mid])
        try:
            ast.parse(prefix)
            best = mid
            lo = mid + 1
        except SyntaxError:
            hi = mid - 1
    complete = "".join(lines[:best])
    partial = "".join(lines[best:])
    return complete, partial


# The subprocess body that executes ONE part function and exports its STL. Kept as a
# string template so we never import cadquery in THIS process (heavy OCCT load) — the
# child imports it. The child runs with a clean cwd = run dir and no network use.
_EXPORT_RUNNER = r'''
import sys, json
src_path, func_name, out_stl = sys.argv[1], sys.argv[2], sys.argv[3]
ns = {}
try:
    with open(src_path, "r", encoding="utf-8") as f:
        code = f.read()
    exec(compile(code, src_path, "exec"), ns)
    fn = ns.get("build_" + func_name)
    if fn is None:
        print(json.dumps({"ok": False, "error": "build_%s() not defined" % func_name}))
        sys.exit(0)
    solid = fn()
    # Accept a Workplane, an Assembly, or a raw Shape.
    if hasattr(solid, "val"):
        shape = solid.val()
    elif hasattr(solid, "toCompound"):
        shape = solid.toCompound()
    else:
        shape = solid
    shape.exportStl(out_stl)
    print(json.dumps({"ok": True, "error": ""}))
except Exception as e:
    import traceback
    print(json.dumps({"ok": False,
                      "error": (type(e).__name__ + ": " + str(e))[:400],
                      "trace": traceback.format_exc()[-600:]}))
'''


def _export_link(script_path: Path, link_name: str, stl_path: Path,
                 run_dir: str) -> tuple[bool, str]:
    """Execute build_<link>() from script_path in a subprocess and export its STL.
    Returns (ok, error). Sandboxed: clean-ish env, cwd = run dir, timeout, no net."""
    stl_path.parent.mkdir(parents=True, exist_ok=True)
    runner = Path(run_dir) / "_cq_export_runner.py"
    if not runner.exists():
        runner.write_text(_EXPORT_RUNNER, encoding="utf-8")
    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    try:
        r = subprocess.run(
            [sys.executable, str(runner), str(script_path), link_name, str(stl_path)],
            capture_output=True, text=True, timeout=_EXEC_TIMEOUT,
            cwd=run_dir, env=env)
    except subprocess.TimeoutExpired:
        return False, f"cadquery export timed out after {_EXEC_TIMEOUT}s"
    except Exception as e:
        return False, f"subprocess failed: {type(e).__name__}: {e}"
    out = (r.stdout or "").strip().splitlines()
    payload = None
    for line in reversed(out):
        line = line.strip()
        if line.startswith("{"):
            try:
                payload = json.loads(line)
                break
            except Exception:
                continue
    if payload is None:
        tail = (r.stderr or r.stdout or "").strip()[-300:]
        return False, f"no export result (rc={r.returncode}); {tail}"
    return bool(payload.get("ok")), str(payload.get("error", ""))


def _render_link(script_path, link, ctx, run_dir) -> WorkerResult:
    """Export + validate one link from the given per-batch script."""
    stl = Path(ctx.meshes_dir) / f"{link.name}.stl"
    ok, rerr = _export_link(script_path, link.name, stl, run_dir)
    rep = check_stl(str(stl)) if ok else None
    good = bool(ok and rep and rep.exists and rep.loadable and not rep.degenerate)
    err = "" if good else (rerr or (rep.error if rep else "export failed"))
    return WorkerResult(link_name=link.name, success=good,
                        abs_mesh_path=str(stl), error=err, stl_report=rep)


def _emit_progress(link_name: str, built: int, total: int, run_dir: str, lock) -> None:
    with lock:
        print("ARTIFACT_JSON:" + json.dumps({
            "kind": "mesh_progress", "link": link_name,
            "built": built, "total": total, "run_dir": run_dir}), flush=True)


def _generate_batch_script(client, conv, want_names, log_fn, tag) -> str:
    """Generate a batch's CadQuery script with COMPLETION-CONTINUATION. Streams via
    send_collect; on a cap truncation keep the COMPLETE functions, drop the half-written
    tail, and ask for the functions still missing — concatenating until all wanted
    functions are present, the model stops, or the budget runs out."""
    accum = ""
    for round_i in range(_MAX_CONTINUE + 1):
        messages = conv.get_messages_for_api(api_style=client.api_style)
        text, finish = client.send_collect(messages, system=CQ_WORKER_SYSTEM)
        piece = _strip_fences(text)
        complete, _partial = trim_to_complete_functions(piece)
        accum = (accum + "\n" + complete).strip() if accum else complete.strip()
        conv.add_assistant_message(text)
        present = set(parse_functions(accum))
        missing = [n for n in want_names if n not in present]
        if not missing:
            return accum
        if finish != "length":
            if round_i >= _MAX_CONTINUE:
                return accum
        else:
            log_fn(f"[cq-worker] {tag}: output capped mid-generation; continuing "
                   f"({len(present)} function(s) so far, need {missing})")
        conv.add_user_message(build_cq_worker_continue(missing))
    return accum


def _write_part_scripts(batch_script: str, batch, cq_dir: Path) -> None:
    """Persist EACH link's own function to <run>/cq/<link>.py (prereq for 2b line-edits).

    Splits the batch script into per-function sources via ast so a later fault can edit
    ONE part in isolation. The shared import header is prepended to each so the file is
    runnable standalone. Best-effort: a parse failure just skips (the batch file remains
    the source of truth for the current export)."""
    cq_dir.mkdir(parents=True, exist_ok=True)
    try:
        tree = ast.parse(batch_script)
    except SyntaxError:
        return
    header_nodes = [n for n in tree.body
                    if isinstance(n, (ast.Import, ast.ImportFrom))]
    header = "\n".join(ast.get_source_segment(batch_script, n) or "" for n in header_nodes)
    if "import cadquery" not in header:
        header = ("import cadquery as cq\nimport math\n" + header).strip()
    want = {l.name for l in batch}
    for node in tree.body:
        if (isinstance(node, ast.FunctionDef) and node.name.startswith("build_")
                and node.name[len("build_"):] in want):
            body = ast.get_source_segment(batch_script, node)
            if not body:
                continue
            src = (header + "\n\n\n" + body + "\n").lstrip()
            (cq_dir / f"{node.name[len('build_'):]}.py").write_text(src, encoding="utf-8")


def _build_batch(idx, batch, model, done, peers, ctx, settings, client,
                 counter, total, emit_lock, log_fn, research_note: str = ""):
    """Generate + export ONE batch (its own script), retrying its failed functions.
    Returns list[WorkerResult]. Emits mesh_progress per built STL. Thread-safe (own
    conversation + own batch file)."""
    names = [l.name for l in batch]
    batch_script = Path(ctx.run_dir) / f"_batch_{idx}.py"
    cq_dir = Path(ctx.run_dir) / "cq"
    conv = Conversation()
    if research_note:
        conv.add_user_message(
            "Reference facts from a web search (use where relevant for real "
            f"dimensions/specs):\n{research_note[:4000]}")
    conv.add_user_message(build_cq_worker_batch(model, batch, done, peers))
    attempts = getattr(settings, "worker_retries", 1) + 1
    results: dict = {}

    for attempt in range(1, attempts + 1):
        try:
            script = _generate_batch_script(client, conv, names, log_fn, f"batch {idx}")
        except LLMError as e:
            log_fn(f"[cq-worker] batch {idx} LLM error (attempt {attempt}): {e}")
            if attempt < attempts:
                continue
            for l in batch:
                results.setdefault(l.name, WorkerResult(
                    link_name=l.name, success=False, error=f"LLM error: {e}"))
            break
        batch_script.write_text(script, encoding="utf-8")
        _write_part_scripts(script, batch, cq_dir)
        present = set(parse_functions(script))
        failed: list = []
        for l in batch:
            if l.name not in present:
                r = WorkerResult(link_name=l.name, success=False,
                                 abs_mesh_path=str(Path(ctx.meshes_dir)/f"{l.name}.stl"),
                                 error=f"function build_{l.name}() not defined")
            else:
                r = _render_link(batch_script, l, ctx, ctx.run_dir)
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
            conv.add_user_message(build_cq_worker_batch_retry(failed))

    return [results[l.name] for l in batch]


def build_all(model: KinematicModel, ctx: RunContext, settings,
              log_fn=print) -> list[WorkerResult]:
    """Generate the model's geometry as CadQuery Python, export + validate each link,
    and combine into model.py. Returns one WorkerResult per link (in model link order).
    Mirrors scad_worker.build_all's contract exactly."""
    wsettings = copy.copy(settings)
    wsettings.model = os.environ.get("MAKER2_WORKER_MODEL", settings.model)
    client = wsettings.worker_client()
    total = len(model.links)
    log_fn(f"[cq-worker] model = {wsettings.model} | {total} links in "
           f"batches of {_BATCH_SIZE} (CadQuery)")

    from .tools import research_findings
    research_note = research_findings(client, wsettings, model.name, log_fn=log_fn)

    batches = [model.links[i:i + _BATCH_SIZE]
               for i in range(0, total, _BATCH_SIZE)]
    wave_size = max(1, min(len(batches), getattr(settings, "max_workers", 4)))

    counter = [0]
    emit_lock = threading.Lock()
    all_results: dict = {}
    done_names: list = []

    for w in range(0, len(batches), wave_size):
        wave = batches[w:w + wave_size]
        wave_names = [l.name for b in wave for l in b]

        def _work(bi_batch):
            bi, batch = bi_batch
            peers = [n for n in wave_names if n not in {l.name for l in batch}]
            return bi, _build_batch(bi, batch, model, list(done_names), peers,
                                    ctx, wsettings, client, counter, total,
                                    emit_lock, log_fn, research_note=research_note)

        indexed = list(enumerate(wave, start=w))
        if len(wave) == 1:
            _bi, res = _work(indexed[0])
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
        log_fn(f"[cq-worker] wave {w // wave_size + 1}: {built}/{len(wave_names)} "
               f"links built ({counter[0]}/{total} total)")

    # Combine every batch's script into model.py (a record of the whole build).
    combined = []
    for i in range(len(batches)):
        bp = Path(ctx.run_dir) / f"_batch_{i}.py"
        if bp.exists():
            combined.append(f"# ---- batch {i} ----\n"
                            + bp.read_text(encoding="utf-8", errors="replace"))
    (Path(ctx.run_dir) / "model.py").write_text("\n\n".join(combined), encoding="utf-8")

    results = [all_results.get(l.name) or WorkerResult(
        link_name=l.name, success=False, error="not built") for l in model.links]
    built = sum(1 for r in results if r.success)
    log_fn(f"[cq-worker] done: {built}/{total} links built")
    return results


def rebuild_link(link, script_text: str, ctx: RunContext, run_dir: str,
                 log_fn=print) -> WorkerResult:
    """2b hook: (re-)export ONE link from an already-edited per-part script. Writes the
    script to <run>/cq/<link>.py and exports meshes/<link>.stl. Used by the line-edit
    path (build_cq_worker_edit) so only the changed part re-renders; unchanged parts'
    STLs are copied forward by the orchestrator. Returns its WorkerResult."""
    cq_dir = Path(run_dir) / "cq"
    cq_dir.mkdir(parents=True, exist_ok=True)
    script_path = cq_dir / f"{link.name}.py"
    script_path.write_text(script_text, encoding="utf-8")
    r = _render_link(script_path, link, ctx, run_dir)
    log_fn(f"[cq-worker] rebuilt {link.name}: {'OK' if r.success else 'FAIL ' + r.error[:80]}")
    return r
