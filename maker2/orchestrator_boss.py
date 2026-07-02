"""Boss orchestrator: build the subassemblies of a SubassemblyPlan (Stage B).

Given a validated SubassemblyPlan (from maker2.boss), build EACH subassembly with
the existing single-subassembly pipeline — manager.decompose (under the boss's
frame contract) -> build_urdf -> scaffold_meshes -> scad_worker.build_all — and run
them IN PARALLEL across subassemblies, reusing the orchestrator's continuous
ThreadPoolExecutor pattern. Each subassembly gets its own run dir
<session_root>/sub_<id>/ (model.urdf + meshes/ + kinematic_model.json +
sub_frames.json). Subs listed in `reuse` are loaded from disk instead of rebuilt,
so the boss loop (Stage F) can surgically re-run only the blamed subassembly.

This module grows across Stages B/C/D/F; Stage B provides the build layer only.
See .claude/plans/precious-humming-wand.md.
"""

from __future__ import annotations

import json
import os
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

from .boss import frame_contract_for
from .manager import decompose, load_model
from .model import SubResult
from .orchestrator import make_run_context
from .scad_worker import build_all
from .urdf_builder import build_urdf, scaffold_meshes, validate_urdf


def _sub_frames_to_dict(model) -> list:
    """The manager's realized interface frames, JSON-ready (from model.frames_realized)."""
    out = []
    for e in getattr(model, "frames_realized", []) or []:
        out.append({
            "frame": e.get("frame", ""),
            "link": e.get("link", ""),
            "local_xyz_m": list(e.get("local_xyz_m", (0.0, 0.0, 0.0))),
            "local_rpy_rad": list(e.get("local_rpy_rad", (0.0, 0.0, 0.0))),
        })
    return out


def _load_sub_from_disk(sub_id: str, session_root: str, log_fn=print) -> SubResult:
    """Reload an already-built subassembly (for surgical re-runs that skip it)."""
    run_dir = os.path.abspath(os.path.join(session_root, f"sub_{sub_id}"))
    ctx = make_run_context(sub_id, session_root, run_dir=run_dir)
    model_json = os.path.join(run_dir, "kinematic_model.json")
    if not os.path.exists(model_json):
        return SubResult(id=sub_id, ok=False,
                         error=f"reuse requested but {model_json} is missing")
    try:
        model = load_model(model_json)
    except Exception as e:
        return SubResult(id=sub_id, ok=False, error=f"reuse load failed: {e}")
    frames = []
    fp = os.path.join(run_dir, "sub_frames.json")
    if os.path.exists(fp):
        try:
            frames = json.loads(open(fp, encoding="utf-8").read())
        except Exception:
            frames = []
    ok, _ = validate_urdf(ctx.urdf_path, require_meshes=True)
    log_fn(f"[sub:{sub_id}] reused from disk ({run_dir})")
    return SubResult(id=sub_id, ctx=ctx, model=model, results=[],
                     sub_frames=frames, ok=ok, error="")


def build_subassembly(spec, plan, settings, session_root, *,
                      feedback: str | None = None, log_fn=print) -> SubResult:
    """Build ONE subassembly under the boss's frame contract. Returns a SubResult.

    Runs the full single-subassembly pipeline into <session_root>/sub_<id>/:
    manager.decompose(frame_contract=...) -> build_urdf -> validate topology ->
    scaffold_meshes -> scad_worker.build_all -> validate with meshes. Writes the
    manager's realized interface frames to sub_frames.json for the assembler.
    """
    sub_id = spec.id
    run_dir = os.path.abspath(os.path.join(session_root, f"sub_{sub_id}"))
    ctx = make_run_context(spec.brief or sub_id, session_root, run_dir=run_dir)
    os.makedirs(ctx.logs_dir, exist_ok=True)
    fc = frame_contract_for(plan, sub_id)

    def slog(msg: str) -> None:
        log_fn(f"[sub:{sub_id}] {msg}")

    try:
        slog("manager: decomposing this subassembly under the frame contract ...")
        model = decompose(spec.brief, settings, model_json_path=ctx.model_json_path,
                          frame_contract=fc, evaluator_feedback=feedback, log_fn=slog)
    except Exception as e:
        slog(f"manager FAILED: {e}")
        return SubResult(id=sub_id, ctx=ctx, ok=False, error=f"manager: {e}")

    build_urdf(model, ctx)
    ok, err = validate_urdf(ctx.urdf_path, require_meshes=False)
    if not ok:
        slog(f"URDF topology invalid: {err}")
        return SubResult(id=sub_id, ctx=ctx, model=model, ok=False,
                         error=f"urdf topology: {err}")
    scaffold_meshes(model, ctx)

    # Record the realized interface frames for the assembler (Stage C).
    sub_frames = _sub_frames_to_dict(model)
    try:
        with open(os.path.join(run_dir, "sub_frames.json"), "w", encoding="utf-8") as f:
            json.dump(sub_frames, f, indent=2)
    except Exception as e:
        slog(f"could not write sub_frames.json: {e}")
    # A missing/short realized-frame set is not fatal here (Stage D precheck will
    # catch it) but is worth flagging: the assembler needs one per contract frame.
    want = {fr.name for fr in fc.frames}
    got = {e["frame"] for e in sub_frames}
    missing = sorted(want - got)
    if missing:
        slog(f"WARNING: manager did not realize interface frame(s): {missing}")

    slog("cadam SCAD worker: generating .scad + rendering per-link STLs ...")
    results = build_all(model, ctx, settings, log_fn=slog)
    built = sum(1 for r in results if r.success)
    ok2, err2 = validate_urdf(ctx.urdf_path, require_meshes=True)
    success = ((built == len(results)) or (getattr(settings, "allow_partial", False)
               and built > 0)) and ok2
    slog(f"built {built}/{len(results)} links; URDF(with meshes) ok={ok2}")
    return SubResult(id=sub_id, ctx=ctx, model=model, results=results,
                     sub_frames=sub_frames, ok=bool(success),
                     error="" if success else (err2 or f"{built}/{len(results)} links built"))


def build_all_subassemblies(plan, settings, session_root, *,
                            feedback_by_sub: dict | None = None,
                            reuse: set = frozenset(), log_fn=print) -> dict:
    """Build every subassembly of `plan` IN PARALLEL. Returns {sub_id: SubResult}.

    Mirrors Orchestrator._run_workers: one ThreadPoolExecutor over all subs keeps
    up to settings.subassembly_max_managers builds in flight (each sub's own worker
    pipeline blocks on LLM/subprocess calls, so the GIL is moot). Subs in `reuse`
    are loaded from disk instead of rebuilt (surgical re-runs). Logging is
    serialized so parallel [sub:*] lines don't interleave mid-line.
    """
    feedback_by_sub = feedback_by_sub or {}
    lock = threading.Lock()

    def log(msg: str) -> None:
        with lock:
            log_fn(msg)

    to_build = [s for s in plan.subassemblies if s.id not in reuse]
    n = max(1, min(len(to_build) or 1, getattr(settings, "subassembly_max_managers", 4)))
    log(f"[boss] building {len(to_build)} subassembly(ies) with up to {n} in "
        f"parallel; reusing {len(reuse)} from disk")

    results: dict = {}
    # Reused subs first (cheap, synchronous disk loads).
    for s in plan.subassemblies:
        if s.id in reuse:
            results[s.id] = _load_sub_from_disk(s.id, session_root, log_fn=log)

    def work(spec) -> SubResult:
        return build_subassembly(spec, plan, settings, session_root,
                                 feedback=feedback_by_sub.get(spec.id), log_fn=log)

    if to_build:
        with ThreadPoolExecutor(max_workers=n) as pool:
            futures = {pool.submit(work, s): s for s in to_build}
            done = 0
            for fut in as_completed(futures):
                spec = futures[fut]
                try:
                    results[spec.id] = fut.result()
                except Exception as e:   # build_subassembly shouldn't raise, but be safe
                    results[spec.id] = SubResult(id=spec.id, ok=False,
                                                 error=f"raised: {type(e).__name__}: {e}")
                done += 1
                r = results[spec.id]
                # ARTIFACT so the UI (Stage H) can show/reload this sub immediately.
                render_dir = r.ctx.run_dir if r.ctx else ""
                log("ARTIFACT_JSON:" + json.dumps({
                    "kind": "subassembly", "sub_id": spec.id,
                    "run_dir": render_dir, "render_dir": render_dir, "ok": r.ok}))
                log(f"[boss] subassembly progress {done}/{len(to_build)} "
                    f"({spec.id}: {'OK' if r.ok else 'FAIL'})")

    return {s.id: results[s.id] for s in plan.subassemblies if s.id in results}


# --------------------------------------------------------------------------- #
# CLI (Stage B verification): build a plan's subassemblies in parallel.
# --------------------------------------------------------------------------- #

def main() -> int:
    import argparse
    import sys
    from .boss import load_plan, plan_machine
    from .config import Settings

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            pass

    ap = argparse.ArgumentParser(
        description="maker2 boss orchestrator: build a plan's subassemblies (parallel)")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--prompt", help="machine prompt: plan it, then build the subs")
    src.add_argument("--plan", help="path to an existing subassembly_plan.json")
    ap.add_argument("--out", default="output")
    ap.add_argument("--model", default=None)
    a = ap.parse_args()

    settings = Settings.load()
    if a.model:
        settings.model = a.model.split("/", 1)[-1]

    if a.plan:
        plan = load_plan(a.plan)
        session_root = os.path.dirname(os.path.abspath(a.plan))
    else:
        ctx = make_run_context(a.prompt, a.out)
        os.makedirs(ctx.run_dir, exist_ok=True)
        session_root = ctx.run_dir
        plan = plan_machine(a.prompt, settings,
                            plan_json_path=os.path.join(session_root,
                                                        "subassembly_plan.json"),
                            log_fn=print)

    print(f"[boss] session root: {session_root}")
    subs = build_all_subassemblies(plan, settings, session_root, log_fn=print)
    print("-" * 56)
    ok = sum(1 for r in subs.values() if r.ok)
    print(f"RESULT: {ok}/{len(subs)} subassemblies built")
    for sid, r in subs.items():
        print(f"  {sid:<22} {'OK' if r.ok else 'FAIL'}  "
              f"frames={len(r.sub_frames)}  {r.error[:50]}")
    return 0 if ok == len(subs) else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
