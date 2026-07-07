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
from .urdf_builder import build_urdf, scaffold_meshes, validate_urdf


def _worker_build_all(model, ctx, settings, log_fn):
    """Dispatch geometry building to the configured backend (settings.worker_backend):
    CadQuery by default (curved geometry), OpenSCAD as the legacy fallback. Both share
    the build_all(model, ctx, settings, log_fn) -> list[WorkerResult] contract."""
    backend = getattr(settings, "worker_backend", "cadquery")
    if backend == "openscad":
        from .scad_worker import build_all as _ba
    else:
        from .cq_worker import build_all as _ba
    return _ba(model, ctx, settings, log_fn=log_fn)


def _sub_frames_to_dict(model, contract_frames=None) -> list:
    """The manager's realized interface frames, JSON-ready.

    Primary source is ``model.frames_realized`` (the manager's own frame->link mapping).
    FALLBACK: for any contract frame the manager DECLARED no realization for, but whose
    name EXACTLY matches a link in the model (the manager's own convention — it names a
    dedicated marker link after the frame), auto-realize it at that link's origin. This
    keeps the assembler from crashing on "frame not realized" when the manager built the
    frame link but forgot to emit the frames_realized entry — a common LLM lapse that
    otherwise loops the whole boss pipeline forever."""
    out = []
    seen: set = set()
    for e in getattr(model, "frames_realized", []) or []:
        name = e.get("frame", "")
        out.append({
            "frame": name,
            "link": e.get("link", ""),
            "local_xyz_m": list(e.get("local_xyz_m", (0.0, 0.0, 0.0))),
            "local_rpy_rad": list(e.get("local_rpy_rad", (0.0, 0.0, 0.0))),
        })
        if name:
            seen.add(name)

    if contract_frames:
        link_names = {l.name for l in model.links}
        for fr in contract_frames:
            fname = getattr(fr, "name", "") or ""
            if fname and fname not in seen and fname in link_names:
                out.append({
                    "frame": fname, "link": fname,
                    "local_xyz_m": [0.0, 0.0, 0.0],
                    "local_rpy_rad": [0.0, 0.0, 0.0],
                })
                seen.add(fname)
    return out


def _load_sub_from_disk(sub_id: str, session_root: str, log_fn=print,
                        plan=None) -> SubResult:
    """Reload an already-built subassembly (for surgical re-runs that skip it).

    When ``plan`` is given, ALSO verify the sub realized every interface frame its
    plan spec requires — a sub whose sub_frames.json is missing a contract frame
    cannot be welded (the assembler would crash), so it is marked ok=False here and
    gets REBUILT instead of reused. This is what stops a broken sub from being reused
    forever in a crash loop."""
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
    # Frame-completeness: a reused sub MUST expose every interface frame its plan spec
    # requires, or it can't be assembled. If any is unrealized, don't reuse — rebuild.
    if ok and plan is not None:
        spec = plan.sub_by_id(sub_id)
        want = {fr.name for fr in (spec.frames if spec else [])}
        got = {e.get("frame") for e in (frames or [])}
        missing = sorted(want - got)
        if missing:
            log_fn(f"[sub:{sub_id}] prior build is missing interface frame(s) "
                   f"{missing}; will REBUILD instead of reuse")
            return SubResult(id=sub_id, ctx=ctx, model=model, results=[],
                             sub_frames=frames, ok=False,
                             error=f"unrealized interface frame(s): {missing}")
    log_fn(f"[sub:{sub_id}] reused from disk ({run_dir})")
    return SubResult(id=sub_id, ctx=ctx, model=model, results=[],
                     sub_frames=frames, ok=ok, error="")


def build_subassembly(spec, plan, settings, session_root, *,
                      feedback: str | None = None, user_prompt: str = "",
                      log_fn=print) -> SubResult:
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

    # Item 4b: if this sub was built before AND we're rebuilding it because of a fault,
    # try a MINIMAL edit instead of a from-scratch rebuild: (1) let the manager KEEP it
    # if the fault isn't about this sub, else (2) apply a structured PATCH and rebuild
    # ONLY the changed links (unchanged links keep their prior STLs).
    prior_model_json = None
    prior_model_path = os.path.join(run_dir, "kinematic_model.json")
    if feedback and os.path.exists(prior_model_path):
        try:
            prior_model_json = open(prior_model_path, encoding="utf-8").read()
        except Exception:
            prior_model_json = None
    if prior_model_json:
        from .manager import should_rebuild, decompose_patch
        if not should_rebuild(prior_model_json, feedback, settings,
                              frame_contract=fc, log_fn=slog):
            slog("keeping prior build (fault is elsewhere) — reused from disk")
            return _load_sub_from_disk(sub_id, session_root, log_fn=log_fn, plan=plan)
        try:
            model, changed, patch_meta = decompose_patch(
                prior_model_json, feedback, settings,
                frame_contract=fc, model_json_path=ctx.model_json_path,
                log_fn=slog)
            return _finish_subassembly(spec, plan, ctx, run_dir, fc, model, settings,
                                       slog, only_links=changed, patch_meta=patch_meta,
                                       user_prompt=user_prompt, log_fn=log_fn)
        except Exception as e:
            slog(f"patch path failed ({e}); falling back to full rebuild")

    try:
        slog("manager: decomposing this subassembly under the frame contract ...")
        model = decompose(spec.brief, settings, model_json_path=ctx.model_json_path,
                          frame_contract=fc, evaluator_feedback=feedback, log_fn=slog)
    except Exception as e:
        slog(f"manager FAILED: {e}")
        return SubResult(id=sub_id, ctx=ctx, ok=False, error=f"manager: {e}")

    return _finish_subassembly(spec, plan, ctx, run_dir, fc, model, settings, slog,
                               only_links=None, user_prompt=user_prompt, log_fn=log_fn)


def _edit_changed_links(model, ctx, run_dir, only_links, patch_meta, settings, slog):
    """Item 2b: line-EDIT each MODIFIED link whose CadQuery script was persisted, rather
    than regenerate it. Returns (edit_results, edited_names). A link is edited only if it
    is in patch_meta['modify'], is in only_links, and has a prior <run>/cq/<link>.py; the
    worker gets that script + the fault and returns the smallest change. On any failure
    the link is left for the normal build (NOT added to edited_names)."""
    from .cq_worker import rebuild_link
    from .prompts.cq_worker_prompt import build_cq_worker_edit

    modify = set((patch_meta or {}).get("modify") or set())
    fault_by_link = (patch_meta or {}).get("fault_by_link") or {}
    cq_dir = Path(run_dir) / "cq"
    results: list = []
    edited: set = set()
    for l in model.links:
        if l.name not in only_links or l.name not in modify:
            continue
        script_path = cq_dir / f"{l.name}.py"
        if not script_path.exists():
            continue                              # no prior script -> fresh build
        try:
            prior_script = script_path.read_text(encoding="utf-8")
            client = settings.worker_client()
            from .llm.conversation import Conversation
            conv = Conversation()
            conv.add_user_message(build_cq_worker_edit(
                prior_script, l.name, fault_by_link.get(l.name, "fix this part")))
            text, _ = client.send_collect(
                conv.get_messages_for_api(api_style=client.api_style),
                system="")
            edited_script = _strip_cq_fences(text)
            if "def build_" not in edited_script:
                slog(f"2b: edit for '{l.name}' had no function; leaving for full build")
                continue
            r = rebuild_link(l, edited_script, ctx, run_dir, log_fn=slog)
            if r.success:
                results.append(r)
                edited.add(l.name)
                slog(f"2b: line-edited '{l.name}' (minimal change) — OK")
            else:
                slog(f"2b: edited '{l.name}' failed to export ({r.error[:80]}); "
                     "leaving for full build")
        except Exception as e:
            slog(f"2b: edit of '{l.name}' errored ({e}); leaving for full build")
    return results, edited


def _strip_cq_fences(text: str) -> str:
    t = (text or "").strip()
    if t.startswith("```"):
        body = t.split("\n", 1)[1] if "\n" in t else ""
        if body.rstrip().endswith("```"):
            body = body.rstrip()[:-3]
        return body.strip()
    return t


def _finish_subassembly(spec, plan, ctx, run_dir, fc, model, settings, slog,
                        *, only_links=None, patch_meta=None, user_prompt: str = "",
                        log_fn=print) -> SubResult:
    """Shared tail of build_subassembly: URDF + frames + worker build + validate.
    ``only_links`` (a set) limits the worker to just the changed links (patch path);
    None builds every link. Unchanged links keep their prior STLs (copied forward).
    ``patch_meta`` (item 2b) = {"modify": {names}, "add": {names}, "fault_by_link": {..}}
    so a MODIFIED part's existing CadQuery script is line-EDITED (minimal change) instead
    of regenerated; ADDED parts are freshly generated."""
    sub_id = spec.id
    build_urdf(model, ctx)
    ok, err = validate_urdf(ctx.urdf_path, require_meshes=False)
    if not ok:
        slog(f"URDF topology invalid: {err}")
        return SubResult(id=sub_id, ctx=ctx, model=model, ok=False,
                         error=f"urdf topology: {err}")
    scaffold_meshes(model, ctx)

    # Record the realized interface frames for the assembler (Stage C).
    sub_frames = _sub_frames_to_dict(model, fc.frames)
    try:
        with open(os.path.join(run_dir, "sub_frames.json"), "w", encoding="utf-8") as f:
            json.dump(sub_frames, f, indent=2)
    except Exception as e:
        slog(f"could not write sub_frames.json: {e}")
    want = {fr.name for fr in fc.frames}
    got = {e["frame"] for e in sub_frames}
    missing = sorted(want - got)
    if missing:
        slog(f"WARNING: manager did not realize interface frame(s): {missing}")

    # Item 2b: for a MODIFIED link whose CadQuery script was persisted last build, do a
    # minimal line-EDIT of that one part (keep the good work, change as few lines as
    # possible) instead of regenerating it. Only when the backend is CadQuery and the
    # script exists; everything else (added links, un-editable modifies) still goes
    # through the normal worker build below.
    edit_results: list = []
    edited_names: set = set()
    if (only_links is not None and patch_meta
            and getattr(settings, "worker_backend", "cadquery") != "openscad"):
        edit_results, edited_names = _edit_changed_links(
            model, ctx, run_dir, only_links, patch_meta, settings, slog)

    # On a patch, build ONLY the changed links NOT already handled by an edit; the
    # unchanged ones already have valid STLs on disk that the URDF still references.
    to_build_model = model
    if only_links is not None:
        import copy
        keep = [l for l in model.links
                if l.name in only_links and l.name not in edited_names]
        to_build_model = copy.copy(model)
        to_build_model.links = keep
        slog(f"patch build: {len(edited_names)} link(s) line-edited, "
             f"(re)building {len(keep)} link(s), keeping "
             f"{len(model.links) - len(keep) - len(edited_names)} prior STL(s)")

    if to_build_model.links:
        slog(f"worker ({getattr(settings, 'worker_backend', 'cadquery')}): generating "
             "geometry + exporting per-link STLs ...")
        part_results = _worker_build_all(to_build_model, ctx, settings, log_fn=slog)
    else:
        part_results = []
    # Assemble the full per-link result list: edited + built parts + assumed-ok unchanged.
    if only_links is not None:
        handled = {r.link_name for r in part_results} | edited_names
        from .model import WorkerResult
        results = list(edit_results) + list(part_results) + [
            WorkerResult(link_name=l.name, success=True,
                         abs_mesh_path=os.path.join(ctx.meshes_dir, f"{l.name}.stl"))
            for l in model.links if l.name not in handled]
    else:
        results = part_results
    built = sum(1 for r in results if r.success)
    ok2, err2 = validate_urdf(ctx.urdf_path, require_meshes=True)
    success = ((built == len(results)) or (getattr(settings, "allow_partial", False)
               and built > 0)) and ok2
    slog(f"built {built}/{len(results)} links; URDF(with meshes) ok={ok2}")

    # Conflict gate: right after the worker built this sub's STLs, check for rigid parts
    # that interpenetrate (the manager places parts blind; the worker owns the geometry —
    # when they disagree, parts overlap). On a conflict, a debugger sees the WHOLE sub
    # (prompt, brief, URDF, every part's CadQuery script) and moves/reshapes the offenders,
    # then we recheck. If still stuck after the cap, FAIL UP so the boss re-plans this sub.
    if (success and getattr(settings, "enable_sub_conflict_gate", True)
            and getattr(settings, "worker_backend", "cadquery") != "openscad"):
        from . import subcheck, subdebugger
        max_tries = max(1, getattr(settings, "sub_conflict_max_tries", 3))
        conflicts = subcheck.sub_conflicts(model, ctx.urdf_path, log_fn=slog)
        attempt = 0
        while conflicts and attempt < max_tries:
            attempt += 1
            worst = conflicts[0]
            slog(f"[conflict] {worst.part_a} <-> {worst.part_b} ({worst.frac:.0%}); "
                 f"debugger attempt {attempt}/{max_tries}")
            log_fn("ARTIFACT_JSON:" + json.dumps({
                "kind": "conflict", "sub_id": sub_id, "attempt": attempt,
                "pairs": [{"a": c.part_a, "b": c.part_b, "frac": round(c.frac, 3)}
                          for c in conflicts]}))
            try:
                model, _changed, moved = subdebugger.debug_sub(
                    model, ctx, run_dir, spec, plan, user_prompt, conflicts, settings,
                    frame_contract=fc, log_fn=slog)
            except subdebugger.SubDebuggerError as e:
                slog(f"[conflict] debugger could not patch this pass ({e})")
                break
            if moved:
                build_urdf(model, ctx)          # a pose changed -> regenerate the URDF
            validate_urdf(ctx.urdf_path, require_meshes=True)
            conflicts = subcheck.sub_conflicts(model, ctx.urdf_path, log_fn=slog)
        if conflicts:
            worst = conflicts[0]
            slog(f"[conflict] UNRESOLVED after {attempt} pass(es): "
                 f"{worst.part_a} vs {worst.part_b} ({worst.frac:.0%}) -> failing sub up")
            return SubResult(id=sub_id, ctx=ctx, model=model, results=results,
                             sub_frames=_sub_frames_to_dict(model, fc.frames), ok=False,
                             error=(f"unresolved rigid conflict: {worst.part_a} and "
                                    f"{worst.part_b} interpenetrate ({worst.frac:.0%}) — "
                                    "fix their placement or geometry."))
        if attempt:
            # Cleared after >=1 debugger pass: the model/geometry moved, so persist the
            # corrected model + refresh the realized interface frames (a pose edit can
            # move them) so disk (which the assembler + any reuse read) matches.
            slog(f"[conflict] cleared after {attempt} debugger pass(es)")
            try:
                from .manager import save_model
                save_model(model, ctx.model_json_path)
            except Exception as e:
                slog(f"[conflict] could not persist debugged model: {e}")
            sub_frames = _sub_frames_to_dict(model, fc.frames)
            try:
                with open(os.path.join(run_dir, "sub_frames.json"), "w",
                          encoding="utf-8") as f:
                    json.dump(sub_frames, f, indent=2)
            except Exception as e:
                slog(f"[conflict] could not rewrite sub_frames.json: {e}")

    return SubResult(id=sub_id, ctx=ctx, model=model, results=results,
                     sub_frames=sub_frames, ok=bool(success),
                     error="" if success else (err2 or f"{built}/{len(results)} links built"))


def build_all_subassemblies(plan, settings, session_root, *,
                            feedback_by_sub: dict | None = None,
                            reuse: set = frozenset(), user_prompt: str = "",
                            log_fn=print) -> dict:
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
    # Reused subs first (cheap, synchronous disk loads). A reused sub that fails
    # frame-completeness (its prior build can't be assembled) is promoted to the build
    # list right now, so it's REBUILT this iteration instead of crashing the assembler.
    reuse_targets = [s for s in plan.subassemblies if s.id in reuse]
    promoted: list = []
    for s in reuse_targets:
        r = _load_sub_from_disk(s.id, session_root, log_fn=log, plan=plan)
        if r.ok:
            results[s.id] = r
        else:
            promoted.append(s)
            feedback_by_sub.setdefault(
                s.id, (r.error or "prior build unusable") + " — rebuild this subassembly.")
    if promoted:
        log(f"[boss] {len(promoted)} reused sub(s) had an unusable prior build "
            f"{[s.id for s in promoted]}; rebuilding them.")
        to_build = to_build + promoted

    def work(spec) -> SubResult:
        return build_subassembly(spec, plan, settings, session_root,
                                 feedback=feedback_by_sub.get(spec.id),
                                 user_prompt=user_prompt, log_fn=log)

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

def _sub_physics(sub, prompt, *, settings=None, log_fn=print) -> dict:
    """Drive ONE subassembly on its own URDF (each sub is a valid mechanism), so a
    fault localizes to this sub_id BEFORE assembly. Returns run_physics' dict, or a
    'no test' pass if the sub has no movable parts / physics is unavailable."""
    from .physics import run_physics
    movable = [l for l in sub.model.links
               if getattr(l, "dof", "fixed") in ("spin", "free")]
    if not movable:
        return {"passed": True, "verdict": "PASS", "summary": "no movable parts",
                "blamed_kind": None}
    try:
        return run_physics(sub.ctx.urdf_path, f"{prompt} :: subassembly {sub.id}",
                           sub.ctx.run_dir, settings)
    except Exception as e:
        log_fn(f"[sub:{sub.id}] physics unavailable ({e}); skipping pre-check sim")
        return {"passed": True, "verdict": "PASS", "summary": f"physics skipped: {e}",
                "blamed_kind": None}


def run_boss(prompt: str, out_dir: str = "output", settings=None, *,
             do_physics: bool = True, per_sub_physics: bool = False,
             max_boss_iters: int = 0, thread: str | None = None,
             refine_message: str | None = None,
             log_fn=print) -> dict:
    """The hierarchical pipeline end-to-end, with SURGICAL fault routing.

    Infinite MAIN loop (like run.run's physics-driven loop): boss plan -> parallel
    subassembly build -> [optional per-sub physics] -> assemble -> precheck ->
    assembled physics -> aggregate. A failure re-runs the SMALLEST thing:
      - a sub that didn't build / failed its own physics / a precheck 'sub' fault
        -> re-run ONLY that manager (others reused from disk).
      - a precheck 'interface' fault or an aggregated 'interface' physics fault
        -> re-plan via the boss.
    ``refine_message`` (multi-turn) re-plans the SAME machine with a user change,
    loading the prior plan from this session and REUSING every subassembly whose id
    the boss keeps (only changed subs rebuild). `max_boss_iters<=0` = infinite (stop
    by killing the process). Emits the stage markers + ARTIFACTs the UI reads.
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
    last_plan_json = None                 # last successfully-built plan (reuse baseline
                                          # for the NEXT fault re-plan)
    replan_blamed: set = set()            # sub ids a fault blamed (must rebuild on re-plan)

    # Multi-turn refine: load the prior plan from this session (same slug -> same
    # session_root) so the boss updates it, and the ids it keeps can be REUSED from
    # disk (only changed subs rebuild).
    prior_plan_json = None
    prior_sub_ids: set = set()
    if refine_message and os.path.exists(plan_path):
        try:
            prior_plan_json = open(plan_path, encoding="utf-8").read()
            prior_sub_ids = {s.id for s in boss.load_plan(plan_path).subassemblies}
            log(f"[boss] refine: loaded prior plan ({len(prior_sub_ids)} subassemblies)")
        except Exception as e:
            log(f"[boss] refine: could not load prior plan ({e}); planning fresh")
            refine_message = None
    elif refine_message:
        log("[boss] refine requested but no prior plan on disk; planning fresh")
        refine_message = None

    it = 0
    while True:
        log(f"\n===== ITERATION {it} (boss{' re-plan' if feedback else ''}) =====")

        # 1. Boss plan (re-plan only when an interface fault set `feedback`; the FIRST
        #    iteration also carries a refine change if this is a multi-turn refine).
        if plan is None or feedback is not None:
            # Carry the prior plan on ANY re-plan (refine OR a fault) so the boss keeps
            # unchanged subassembly ids and they can be reused. `last_plan_json` is the
            # last successfully-built plan; on iteration 0 it's the refine's prior plan.
            replan_prior = None
            if refine_message and it == 0:
                replan_prior = prior_plan_json
            elif feedback is not None:
                replan_prior = last_plan_json          # fault re-plan keeps the prior
            try:
                plan = boss.plan_machine(
                    prompt, settings, plan_json_path=plan_path, feedback=feedback,
                    refine_message=refine_message if it == 0 else None,
                    prior_plan_json=replan_prior,
                    log_fn=log)
            except boss.BossError as e:
                result["error"] = f"boss failed: {e}"
                log(f"[boss] FAILED: {e}")
                break
            # Reuse every sub whose id the boss KEPT from the prior plan AND that is
            # built on disk, MINUS the ids the fault blamed (those must rebuild). This
            # preserves good work on both refines and fault re-plans.
            if replan_prior:
                try:
                    prior_ids = {s.id for s in
                                 boss.plan_from_dict(json.loads(replan_prior)).subassemblies}
                except Exception:
                    prior_ids = prior_sub_ids
                kept = {s.id for s in plan.subassemblies} & prior_ids
                on_disk = {sid for sid in kept if os.path.exists(
                    os.path.join(session_root, f"sub_{sid}", "kinematic_model.json"))}
                reuse = on_disk - replan_blamed
                log(f"[boss] re-plan: reusing {len(reuse)} unchanged subassemblies, "
                    f"rebuilding {len(plan.subassemblies) - len(reuse)}")
            else:
                reuse = set()             # a truly-fresh plan invalidates built subs
            feedback = None
            feedback_by_sub = {}
            replan_blamed = set()
        result["iterations"] = it + 1
        # Remember this plan as the reuse baseline for the NEXT re-plan.
        last_plan_json = json.dumps(boss.plan_to_dict(plan))

        # 1b/1c. Coarse appearance proxy (flag-gated): a low-poly whole-machine base
        #        look + a per-sub proportion summary threaded to every manager via the
        #        frame contract. Stashed on the plan so frame_contract_for picks it up
        #        without changing build_subassembly's call site.
        if getattr(settings, "enable_appearance_proxy", False):
            try:
                from . import appearance
                ap = appearance.build_appearance_proxy(plan, session_root, log_fn=log)
                plan.appearance_summary = ap.get("summary_text", "")
            except Exception as e:
                log(f"[boss] appearance proxy skipped ({e})")

        # 2. Build subassemblies in parallel (reusing unchanged ones from disk).
        subs = build_all_subassemblies(plan, settings, session_root,
                                       feedback_by_sub=feedback_by_sub, reuse=reuse,
                                       user_prompt=prompt, log_fn=log)
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
                pr = _sub_physics(subs[s.id], prompt, settings=settings, log_fn=log)
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

        # 4b. Silent overlap auto-nudge (Session B item 1b): separate any subassemblies
        #     that interpenetrate but share NO seam so THIS assembly closes, and tell the
        #     blamed managers to fix their placement next iteration (a nudge is a hint,
        #     not a rebuild trigger — it never sets `feedback`).
        try:
            nudges = assembler.auto_nudge_overlaps(final, plan, subs, assembly_ctx,
                                                   log_fn=log)
        except Exception as e:
            log(f"[assembler] auto-nudge skipped ({e})")
            nudges = {}
        for sid, dxyz in (nudges or {}).items():
            mm = ", ".join(f"{v*1000:+.1f}" for v in dxyz)
            feedback_by_sub[sid] = ((feedback_by_sub.get(sid, "") + " ") if
                                    feedback_by_sub.get(sid) else "") + (
                f"your subassembly was auto-moved [{mm}] mm to clear an overlapping "
                "neighbor — fix its global placement so it does not interpenetrate.")

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
                # Only the subs named in interface violations rebuild on the re-plan;
                # the rest keep their ids and reuse from disk (prior plan is carried).
                replan_blamed = {v.sub_id for v in iface if v.sub_id}
                log(f"[precheck] interface fault -> boss re-plan (rebuild only "
                    f"{sorted(replan_blamed) or 'affected subs'}; others reused)")
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

        # 5b. APPEARANCE JUDGE on the ASSEMBLED machine (boss mode had none — this is
        #     the gate that catches parts floating in space / disconnected / wrong
        #     proportions, which precheck (geometry-only) and physics (transmission-
        #     only) miss). Render views + VLM verdict; on FAIL, feed the suggestions to
        #     a boss re-plan (prior plan kept -> unchanged subs reuse).
        if getattr(settings, "enable_hierarchy_judge", True):
            try:
                from .viz import render_six_views
                from .judger import judge as _judge, JudgeError
                views_dir = os.path.join(assembly_ctx.run_dir, "views")
                view_pngs = {}
                try:
                    view_pngs = render_six_views(assembly_ctx.urdf_path, views_dir)
                    log(f"[judge] rendered {len(view_pngs)} view(s) of the assembled machine")
                except Exception as e:
                    log(f"[judge] view render unavailable ({type(e).__name__}); judging text-only")
                verdict = _judge(prompt, final, [], view_pngs, settings,
                                 out_json_path=os.path.join(assembly_ctx.run_dir, "judge.json"),
                                 log_fn=log)
                log("ARTIFACT_JSON:" + json.dumps({
                    "kind": "judge", "iter": it, "passed": verdict.passed,
                    "reasons": verdict.reasons[:400], "suggestions": verdict.suggestions[:400]}))
                if not verdict.passed:
                    fb = (verdict.suggestions or verdict.reasons
                          or "the assembled machine looks wrong")
                    feedback = ("the assembled machine FAILED the appearance review "
                                f"(parts floating/disconnected/mis-proportioned): {fb}")
                    log(f"[judge] FAIL -> boss re-plan: {fb[:120]}")
                    if not infinite and it >= max_boss_iters - 1:
                        result["error"] = f"judge failed: {fb}"; break
                    it += 1; continue
                log("[judge] assembled appearance PASS")
            except JudgeError as e:
                log(f"[judge] verdict unavailable ({e}); continuing to physics")
            except Exception as e:
                log(f"[judge] skipped ({type(e).__name__}: {str(e)[:100]})")

        # 6. Physics on the ASSEMBLED machine (multi-test + aggregate from Stage E).
        if not do_physics:
            log("[boss] assembled + precheck OK (physics not requested) -> done.")
            break
        from .physics import run_physics
        log("[physics] evaluating the assembled machine ...")
        try:
            phys = run_physics(assembly_ctx.urdf_path, prompt, assembly_ctx.run_dir,
                               settings)
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

        # 7. Route the physics failure PRECISELY on the diagnoser's blame:
        #    - structure with a named culprit sub/part -> re-run ONLY that sub (others
        #      reuse from disk), handing the manager the exact culprit part.
        #    - interface (motion didn't cross a seam) -> boss re-plan, but carry the
        #      prior plan so unchanged subs keep their ids and reuse; only the two
        #      seam subs rebuild.
        #    - non-localized -> re-run the subs whose tests FAILED; re-plan (with prior
        #      plan) only if NOTHING maps. We never wholesale-wipe good work.
        if not infinite and it >= max_boss_iters - 1:
            result["error"] = f"physics failed: {phys.get('summary')}"; break
        kind = phys.get("blamed_kind")
        culprit_subs = _map_blamed_to_subs(phys.get("culprit_subs") or [], plan)
        blamed = _map_blamed_to_subs(phys.get("blamed_subs") or [], plan)
        cm = phys.get("cause_map") or {}
        cparts = phys.get("culprit_parts") or []

        if kind == "interface":
            # Seam fault: re-plan minimally, keeping every non-seam sub (they reuse).
            feedback = f"the assembled machine failed physics at a SEAM: {phys.get('summary')}"
            replan_blamed = culprit_subs or blamed   # only these rebuild after re-plan
            log(f"[boss] physics interface fault -> boss re-plan (rebuild only "
                f"{sorted(replan_blamed) or 'seam subs'}; others reused)")
        elif (culprit_subs or blamed):
            # Localized structure fault -> surgically re-run the blamed sub(s) only.
            target = culprit_subs or blamed
            for sid in target:
                part = next((p for p in cparts
                             if _map_blamed_to_subs([p], plan) == {sid}), "")
                detail = str((cm.get(sid) or {}).get("reason", "")
                             or phys.get("summary", ""))
                feedback_by_sub[sid] = (
                    "assembled physics blamed this subassembly"
                    + (f"; the exact broken part is '{part}'" if part else "")
                    + f": {detail}")
            reuse = {s.id for s in plan.subassemblies} - set(target)
            log(f"[boss] physics blamed subs {sorted(target)}"
                + (f" (parts {cparts})" if cparts else "")
                + " -> re-running only those.")
        else:
            # Couldn't localize. Re-run the subs whose tests failed (from cause_map)
            # rather than wiping everything; fall back to a prior-plan re-plan only if
            # even that is empty.
            failed_keys = _map_blamed_to_subs(list(cm.keys()), plan) if cm else set()
            failed_keys = {k for k in failed_keys
                           if (cm.get(k) or {}).get("verdict") != "PASS"} or failed_keys
            if failed_keys:
                for sid in failed_keys:
                    feedback_by_sub[sid] = ("physics failed; this subassembly's test did "
                                            "not pass: " + str(phys.get("summary", "")))
                reuse = {s.id for s in plan.subassemblies} - failed_keys
                log(f"[boss] physics not part-localized -> re-running failed subs "
                    f"{sorted(failed_keys)} (keeping the rest).")
            else:
                feedback = f"the assembled machine failed physics: {phys.get('summary')}"
                log("[boss] physics fault fully unlocalized -> boss re-plan (prior plan "
                    "kept so unchanged subs still reuse).")
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
    subs = build_all_subassemblies(plan, settings, session_root,
                                   user_prompt=a.prompt, log_fn=print)
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
