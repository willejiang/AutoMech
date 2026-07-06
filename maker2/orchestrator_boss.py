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
from pathlib import Path

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
# Stage F: the surgical boss loop.
# --------------------------------------------------------------------------- #

def _sub_physics(sub, prompt, *, log_fn=print) -> dict:
    """Drive ONE subassembly on its own URDF (each sub is a valid mechanism), so a
    fault localizes to this sub_id BEFORE assembly. Returns run_physics' dict, or a
    'no test' pass if the sub has no movable joints / physics is unavailable."""
    from .physics import run_physics
    movable = [j for j in sub.model.joints if j.type in ("revolute", "prismatic", "continuous")]
    if not movable:
        return {"passed": True, "verdict": "PASS", "summary": "no movable joints",
                "blamed_kind": None}
    try:
        return run_physics(sub.ctx.urdf_path, f"{prompt} :: subassembly {sub.id}",
                           sub.ctx.run_dir)
    except Exception as e:
        log_fn(f"[sub:{sub.id}] physics unavailable ({e}); skipping pre-check sim")
        return {"passed": True, "verdict": "PASS", "summary": f"physics skipped: {e}",
                "blamed_kind": None}


def run_boss(prompt: str, out_dir: str = "output", settings=None, *,
             do_physics: bool = True, per_sub_physics: bool = False,
             max_boss_iters: int = 0, thread: str | None = None,
             log_fn=print) -> dict:
    """The hierarchical pipeline end-to-end, with SURGICAL fault routing.

    Infinite MAIN loop (like run.run's physics-driven loop): boss plan -> parallel
    subassembly build -> [optional per-sub physics] -> assemble -> precheck ->
    assembled physics -> aggregate. A failure re-runs the SMALLEST thing:
      - a sub that didn't build / failed its own physics / a precheck 'sub' fault
        -> re-run ONLY that manager (others reused from disk).
      - a precheck 'interface' fault or an aggregated 'interface' physics fault
        -> re-plan via the boss.
    `max_boss_iters<=0` = infinite (stop by killing the process). Emits the stage
    markers + ARTIFACTs the UI (Stage H) reads.
    """
    from . import assembler, boss, precheck as precheck_mod
    from .config import Settings

    settings = settings or Settings.load()
    infinite = max_boss_iters <= 0

    def log(m):
        log_fn(m)

    slug = _slug_for(prompt)
    from datetime import datetime, timezone
    session_root = os.path.abspath(os.path.join(out_dir, f"{slug}_boss"))
    os.makedirs(session_root, exist_ok=True)
    plan_path = os.path.join(session_root, "subassembly_plan.json")

    result = {"ok": False, "run_dir": session_root, "render_dir": "",
              "iterations": 0, "error": "", "hierarchy": True,
              "subassemblies": [], "physics": None}

    plan = None
    feedback = None                       # boss re-plan feedback (interface faults)
    feedback_by_sub: dict = {}            # per-sub manager feedback (sub faults)
    reuse: set = set()                    # subs to load from disk (unchanged ones)
    it = 0
    while True:
        log(f"\n===== ITERATION {it} (boss{' re-plan' if feedback else ''}) =====")

        # 1. Boss plan (re-plan only when an interface fault set `feedback`).
        if plan is None or feedback is not None:
            try:
                plan = boss.plan_machine(prompt, settings, plan_json_path=plan_path,
                                        feedback=feedback, log_fn=log)
            except boss.BossError as e:
                result["error"] = f"boss failed: {e}"
                log(f"[boss] FAILED: {e}")
                break
            feedback = None
            reuse = set()                 # a fresh plan invalidates all built subs
            feedback_by_sub = {}
        result["iterations"] = it + 1

        # 2. Build subassemblies in parallel (reusing unchanged ones from disk).
        subs = build_all_subassemblies(plan, settings, session_root,
                                       feedback_by_sub=feedback_by_sub, reuse=reuse,
                                       log_fn=log)
        result["subassemblies"] = [{"id": s.id, "ok": subs[s.id].ok,
                                    "run_dir": subs[s.id].ctx.run_dir if subs[s.id].ctx else ""}
                                   for s in plan.subassemblies]
        failed = [sid for sid, r in subs.items() if not r.ok]
        if failed:
            for sid in failed:
                feedback_by_sub[sid] = (subs[sid].error or "build failed") + \
                    " — rebuild this subassembly."
            reuse = {s.id for s in plan.subassemblies} - set(failed)
            log(f"[boss] {len(failed)} subassembly(ies) failed to build "
                f"{failed}; re-running only those.")
            if not infinite and it >= max_boss_iters - 1:
                result["error"] = f"subassemblies failed: {failed}"; break
            it += 1; continue

        # 3. (optional) Per-sub physics: localize a drivetrain fault to its sub_id
        #    BEFORE stitching, so we never blame the assembly for a bad part.
        if do_physics and per_sub_physics:
            bad = {}
            for s in plan.subassemblies:
                pr = _sub_physics(subs[s.id], prompt, log_fn=log)
                if pr.get("passed") is False:
                    bad[s.id] = (pr.get("reason") or pr.get("summary") or "sub physics FAIL")
            if bad:
                feedback_by_sub.update({k: v + " — fix this subassembly's mechanism."
                                        for k, v in bad.items()})
                reuse = {s.id for s in plan.subassemblies} - set(bad)
                log(f"[boss] per-sub physics FAILED for {list(bad)}; re-running those.")
                if not infinite and it >= max_boss_iters - 1:
                    result["error"] = f"sub physics failed: {list(bad)}"; break
                it += 1; continue

        # 4. Assemble the subassemblies into one URDF. Each iteration writes its OWN
        #    dir (assembly_iter_<it>) so previous versions are RETAINED on disk and the
        #    UI can scrub back to them (canvas + physics recording per version).
        assembly_ctx = make_run_context(
            plan.name, session_root,
            run_dir=os.path.join(session_root, f"assembly_iter_{it}"))
        try:
            final = assembler.assemble(plan, subs, assembly_ctx, log_fn=log)
        except assembler.AssemblerError as e:
            # A stitch failure is an interface/plan fault -> re-plan.
            feedback = f"assembly failed: {e}"
            log(f"[assembler] FAILED -> boss re-plan: {e}")
            if not infinite and it >= max_boss_iters - 1:
                result["error"] = f"assembly failed: {e}"; break
            it += 1; continue
        result["render_dir"] = assembly_ctx.run_dir
        result["ok"] = True
        log("ARTIFACT_JSON:" + json.dumps({
            "kind": "assembled_model", "iter": it, "run_dir": assembly_ctx.run_dir,
            "render_dir": assembly_ctx.run_dir}))

        # 5. Geometric pre-check BEFORE physics.
        rep = precheck_mod.precheck(plan, subs, assembly_ctx.urdf_path, log_fn=log)
        log("ARTIFACT_JSON:" + json.dumps({
            "kind": "precheck", "iter": it, "ok": rep.ok,
            "violations": [{"kind": v.kind, "severity": v.severity,
                            "sub_id": v.sub_id, "detail": v.detail} for v in rep.violations]}))
        if not rep.ok:
            iface = [v for v in rep.violations if v.severity == "interface"]
            if iface:
                feedback = "geometry pre-check failed (interface): " + \
                    "; ".join(v.detail for v in iface)
                log("[precheck] interface fault -> boss re-plan")
            else:
                for v in rep.violations:
                    if v.sub_id:
                        feedback_by_sub[v.sub_id] = f"geometry: {v.detail} — fix this subassembly."
                blamed = {v.sub_id for v in rep.violations if v.sub_id}
                reuse = {s.id for s in plan.subassemblies} - blamed
                log(f"[precheck] sub fault -> re-running {sorted(blamed)}")
            if not infinite and it >= max_boss_iters - 1:
                result["error"] = f"precheck failed: {rep.summary()}"; break
            it += 1; continue

        # 6. Physics on the ASSEMBLED machine (multi-test + aggregate from Stage E).
        if not do_physics:
            log("[boss] assembled + precheck OK (physics not requested) -> done.")
            break
        from .physics import run_physics
        log("[physics] evaluating the assembled machine ...")
        try:
            phys = run_physics(assembly_ctx.urdf_path, prompt, assembly_ctx.run_dir)
        except Exception as e:
            log(f"[physics] failed: {e}")
            phys = {"passed": None, "blamed_kind": None, "summary": f"physics error: {e}"}
        result["physics"] = phys
        log("ARTIFACT_JSON:" + json.dumps({
            "kind": "physics", "iter": it, "run_dir": assembly_ctx.run_dir,
            "render_dir": assembly_ctx.run_dir, "passed": phys.get("passed"),
            "physics": phys}))
        if phys.get("passed"):
            log("[boss] assembled physics PASS -> done.")
            break
        if phys.get("passed") is None:
            log("[boss] physics errored/unavailable -> stop with current assembly.")
            break

        # 7. Route the physics failure. An 'interface' fault (motion didn't cross a
        #    seam) -> boss re-plan; otherwise re-run the blamed subassembly(ies).
        if not infinite and it >= max_boss_iters - 1:
            result["error"] = f"physics failed: {phys.get('summary')}"; break
        if phys.get("blamed_kind") == "interface":
            feedback = f"the assembled machine failed physics at a SEAM: {phys.get('summary')}"
            log("[boss] physics interface fault -> boss re-plan")
        else:
            blamed = _map_blamed_to_subs(phys.get("blamed_subs") or [], plan)
            if blamed:
                cm = phys.get("cause_map") or {}
                for sid in blamed:
                    feedback_by_sub[sid] = ("assembled physics blamed this subassembly: "
                                            + str(cm.get(sid, phys.get("summary", ""))))
                reuse = {s.id for s in plan.subassemblies} - set(blamed)
                log(f"[boss] physics blamed subs {sorted(blamed)} -> re-running those.")
            else:
                # Couldn't localize -> re-plan (safest).
                feedback = f"the assembled machine failed physics: {phys.get('summary')}"
                log("[boss] physics fault not localized -> boss re-plan")
        it += 1
        continue

    # ---- sidecar + result.json for the UI ----
    try:
        Path(session_root, "result.json").write_text(json.dumps(result, indent=2))
        Path(session_root, "run.json").write_text(json.dumps({
            "prompt": prompt, "model": settings.model, "hierarchy": True,
            "thread": thread,
            "created_at": datetime.now(timezone.utc).isoformat()}, indent=2))
    except Exception as e:
        log(f"[boss] could not write result.json: {e}")

    log("-" * 56)
    n_ok = sum(1 for s in result["subassemblies"] if s["ok"])
    phys_ok = (result["physics"] or {}).get("passed")
    overall = result["ok"] and (phys_ok is not False)   # None (not run) counts as ok
    log(f"RESULT: {'PASS' if overall else 'FAIL'} "
        f"— {n_ok}/{len(result['subassemblies'])} subassemblies over "
        f"{result['iterations']} boss iteration(s). Bundle: {session_root}")
    return result


def _slug_for(prompt: str) -> str:
    from .orchestrator import _slug
    return _slug(prompt)


def _map_blamed_to_subs(blamed, plan) -> set:
    """Map physics-blamed subsystem ids to boss subassembly ids. The evaluator names
    subsystems after the assembled model's namespaced links (e.g. 'sub_output_...'),
    so a boss sub_id is blamed when a blamed subsystem string starts with it."""
    sub_ids = [s.id for s in plan.subassemblies]
    out = set()
    for b in blamed:
        bs = str(b)
        for sid in sub_ids:
            if bs == sid or bs.startswith(f"{sid}_") or sid in bs:
                out.add(sid)
    return out


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
