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


def _try_contract_repair(kind, detail, plan, subs, settings, log_fn) -> bool:
    """Gate-fault debugger: fix a CONTRACT (naming/realization) fault IN PLACE instead of a full
    boss re-plan. Deterministic repair first (fault_repair), then an LLM contract-debugger
    fallback (gated by settings.enable_contract_debugger, default on). Returns True if anything
    was repaired (caller should RETRY the failed gate/assemble); False -> fall back to re-plan."""
    from . import fault_repair
    rr = fault_repair.repair_contract_fault(kind, plan, subs, detail, log_fn=log_fn)
    if not rr.repaired and getattr(settings, "enable_contract_debugger", True):
        try:
            from . import contract_debugger
            rr = contract_debugger.debug_contract_fault(kind, detail, plan, subs, settings,
                                                        log_fn=log_fn)
        except Exception as e:
            log_fn(f"[repair] contract-debugger errored ({type(e).__name__}: {e}); re-planning")
            return False
    if rr.repaired:
        log_fn(f"[repair] contract fault fixed in place ({rr.note.strip() or 'see above'}) — "
               f"retrying without a boss re-plan")
        log_fn("ARTIFACT_JSON:" + json.dumps({
            "kind": "gate", "layer": "repair", "code": "CONTRACT_REPAIRED",
            "detail": rr.note.strip(), "ok": True}))
    return rr.repaired


def _sub_frames_to_dict(model, contract_frames=None) -> list:
    """The manager's realized interface frames, JSON-ready.

    PRIORITY 0 — BUILDER-OWNED BINDING: a `mount` frame whose `mounts_part` names a link that
    ACTUALLY EXISTS in this sub is realized on that link. The boss no longer authors
    `mounts_part` (it cannot know the builder's part names — that guess was the seating bug);
    this only fires when the MANAGER itself named one of its OWN parts for the frame, which is a
    valid builder-owned binding (no cross-agent name guess). A `mounts_part` that names a
    non-existent link is ignored (the manager's frames_realized / fallbacks handle the frame).

    Then ``model.frames_realized`` (the manager's own frame->link+offset, which supports a
    positioned local offset so a seat lands at a bore's position) supplies the rest, with
    FALLBACKS for a frame the manager declared nothing for:
      1. name-match: a link named EXACTLY like the frame (the marker-link convention);
      2. mount-role -> root link: ONLY a `mount` frame with NO mounts_part (a genuine
         structural mounting face) is realized by the ROOT link at its origin.
    Frames still unrealized after these are reported by the manager gate (fail-fast)."""
    out = []
    seen: set = set()

    # PRIORITY 0: a mount frame bound (by the MANAGER, to its own real part) -> that link.
    link_names0 = {l.name for l in model.links}
    for fr in (contract_frames or []):
        fname = getattr(fr, "name", "") or ""
        mp = (getattr(fr, "mounts_part", "") or "").strip()
        if not fname or fname in seen or not mp:
            continue
        if mp in link_names0:
            out.append({"frame": fname, "link": mp,
                        "local_xyz_m": [0.0, 0.0, 0.0], "local_rpy_rad": [0.0, 0.0, 0.0]})
            seen.add(fname)
        # a mounts_part naming a NON-existent link -> ignore; frames_realized/fallbacks handle it.

    for e in getattr(model, "frames_realized", []) or []:
        name = e.get("frame", "")
        if name in seen:
            continue                          # a bound frame already realized above
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
        root = getattr(model, "root_link", "") or ""
        for fr in contract_frames:
            fname = getattr(fr, "name", "") or ""
            if not fname or fname in seen:
                continue
            if fname in link_names:                       # 1. exact name-match
                out.append({"frame": fname, "link": fname,
                            "local_xyz_m": [0.0, 0.0, 0.0],
                            "local_rpy_rad": [0.0, 0.0, 0.0]})
                seen.add(fname)
                continue
            # 2. semantic hardware match. Through-shaft frames are commonly named
            # input_rear_bearing_datum while the manager part is input_rear_bearing.
            # Prefer that physical datum over collapsing every mount onto the root.
            noise={"datum","frame","seat","mount","interface"}
            ft=set(fname.lower().split("_"))-noise
            ranked=[]
            for lname in link_names:
                lt=set(lname.lower().split("_"))-noise;score=len(ft & lt)
                if score:ranked.append((score,lname))
            ranked.sort(reverse=True)
            if ranked and ranked[0][0]>=2 and (len(ranked)==1 or ranked[0][0]>ranked[1][0]):
                out.append({"frame":fname,"link":ranked[0][1],
                            "local_xyz_m":[0.0,0.0,0.0],"local_rpy_rad":[0.0,0.0,0.0]})
                seen.add(fname)
            elif (getattr(fr, "role", "mount") == "mount"
                  and not (getattr(fr, "mounts_part", "") or "").strip()
                  and root in link_names):
                # 2. mount -> root link, ONLY for a seat with NO mounts_part (a genuine
                # structural mounting FACE on the root body). A bound seat is handled by
                # PRIORITY 0 above; if its part is missing it stays unrealized (gate catches).
                out.append({"frame": fname, "link": root,
                            "local_xyz_m": [0.0, 0.0, 0.0],
                            "local_rpy_rad": [0.0, 0.0, 0.0]})
                seen.add(fname)
    return out


def _snapshot_best_subs(session_root: str, sub_ids, log_fn=print) -> str:
    """Copy the current sub_<id>/ dirs into best_snapshot/ so a later REVERT can restore
    the best-scoring geometry byte-for-byte (sub dirs are otherwise overwritten in place
    on every rebuild). Returns the snapshot dir."""
    import shutil
    snap = os.path.join(session_root, "best_snapshot")
    try:
        if os.path.isdir(snap):
            shutil.rmtree(snap, ignore_errors=True)
        os.makedirs(snap, exist_ok=True)
        for sid in sub_ids:
            src = os.path.join(session_root, f"sub_{sid}")
            if os.path.isdir(src):
                shutil.copytree(src, os.path.join(snap, f"sub_{sid}"),
                                dirs_exist_ok=True)
    except Exception as e:
        log_fn(f"[boss] best-snapshot failed ({e}); revert will reuse live disk state")
    return snap


def _restore_best_subs(session_root: str, sub_ids, log_fn=print) -> bool:
    """Restore sub_<id>/ dirs from best_snapshot/ (the inverse of _snapshot_best_subs),
    so a REVERT reuses the best geometry. Returns True if anything was restored."""
    import shutil
    snap = os.path.join(session_root, "best_snapshot")
    if not os.path.isdir(snap):
        return False
    restored = False
    for sid in sub_ids:
        src = os.path.join(snap, f"sub_{sid}")
        dst = os.path.join(session_root, f"sub_{sid}")
        if os.path.isdir(src):
            try:
                if os.path.isdir(dst):
                    shutil.rmtree(dst, ignore_errors=True)
                shutil.copytree(src, dst, dirs_exist_ok=True)
                restored = True
            except Exception as e:
                log_fn(f"[boss] restore of sub_{sid} failed ({e})")
    return restored


def _load_sub_from_disk(sub_id: str, session_root: str, log_fn=print,
                        plan=None, settings=None) -> SubResult:
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
    if ok and plan is not None and not getattr(settings, "manager_py", False):
        # Require the SAME frame set the manager was asked to realize — i.e. the frame
        # CONTRACT (compiler-named frames when a hardpoint contract exists, boss-plan names
        # otherwise), not the raw boss-plan sub.frames. frame_contract_for is the single
        # function the build/write path uses, so routing `want` through it keeps the reuse
        # baseline in the same naming as what was persisted to sub_frames.json; otherwise a
        # compiled sub always looks "missing" its frames and is needlessly rebuilt.
        try:
            want = {fr.name for fr in (frame_contract_for(plan, sub_id).frames or [])}
        except Exception:
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
            return _load_sub_from_disk(sub_id, session_root, log_fn=log_fn, plan=plan, settings=settings)
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
        model = _decompose_sub(spec, fc, settings, ctx, feedback=feedback, log_fn=slog)
    except Exception as e:
        slog(f"manager FAILED: {e}")
        return SubResult(id=sub_id, ctx=ctx, ok=False, error=f"manager: {e}")

    return _finish_subassembly(spec, plan, ctx, run_dir, fc, model, settings, slog,
                               only_links=None, user_prompt=user_prompt, log_fn=log_fn)


def _decompose_sub(spec, fc, settings, ctx, *, feedback=None, log_fn=print):
    """Front door for building ONE sub's model: split it first if it is expected to be
    oversized (C8), else best-of-N decompose (C7). Both keep-best on pre-render badness."""
    threshold = int(getattr(settings, "sub_split_threshold", 12))
    if (getattr(settings, "enable_sub_split", True)
            and int(getattr(spec, "est_link_budget", 0) or 0) > threshold):
        split = _split_decompose(spec, fc, settings, ctx, feedback=feedback, log_fn=log_fn)
        if split is not None:
            return split
        log_fn("[split] halving did not improve on a single decomposition; using it")
    return _decompose_best_of_n(spec, fc, settings, ctx, feedback=feedback, log_fn=log_fn)


def _split_decompose(spec, fc, settings, ctx, *, feedback=None, log_fn=print):
    """C8 — split an oversized subassembly. Rather than overload ONE manager call with a
    10-26-link sub (where it forgets parts and drifts), run the manager TWICE on the same
    brief + frame contract but each time instructed to author only HALF the parts — the
    STRUCTURE/FRAMES half (mounts, plates, the interface frames) and the MECHANISM half
    (gears, shafts, moving parts) — then MERGE the two KinematicModels.

    SAFE BY CONSTRUCTION: on any problem (a half fails to decompose, or the merged model
    doesn't validate) this returns None and the caller falls back to a single best-of-N
    decomposition. The merged model's badness is logged so a poor split is visible; the
    downstream manager gate + compile gate still judge it like any other model."""
    from .manager import decompose, _validate_model, _manager_gate_errors, _NONBLOCKING_CODES
    from .badness import badness
    from .model import KinematicModel

    halves = [
        ("structure", "For THIS response author ONLY the STRUCTURAL parts of this "
                      "subassembly: the mounting plates/frames/housings and EVERY interface "
                      "frame the contract requires (place a real link at each). Do NOT author "
                      "gears, shafts, or moving internals — another pass covers those."),
        ("mechanism", "For THIS response author ONLY the MECHANISM parts of this "
                      "subassembly: the gears, shafts, wheels and moving internals. Do NOT "
                      "author the mounting plates/housings — another pass covers those. Still "
                      "set each part's dof and place them with poses relative to the parts you "
                      "author."),
    ]
    partials = []
    for name, directive in halves:
        fb = (feedback + "\n\n" + directive) if feedback else directive
        try:
            m = decompose(spec.brief, settings, model_json_path=None,
                          frame_contract=fc, evaluator_feedback=fb, log_fn=log_fn)
        except Exception as e:
            log_fn(f"[split] {name} half failed ({e}); abandoning split")
            return None
        log_fn(f"[split] {name} half: {len(m.links)} links")
        partials.append(m)

    # Merge: concat links (dedup by name, first wins), concat poses + mesh_pairs, union
    # frames_realized, keep the structure half's root. Then re-validate/normalize.
    seen: set = set()
    links = []
    for m in partials:
        for l in m.links:
            if l.name not in seen:
                seen.add(l.name)
                links.append(l)
    merged = KinematicModel(
        name=partials[0].name,
        root_link=partials[0].root_link,
        links=links,
        poses=[p for m in partials for p in m.poses],
        mesh_pairs=[mp for m in partials for mp in m.mesh_pairs],
    )
    fr: dict = {}
    for m in partials:
        for e in getattr(m, "frames_realized", []) or []:
            fr.setdefault(e.get("frame"), e)
    merged.frames_realized = list(fr.values())
    try:
        _validate_model(merged)            # normalize + weak forest validation (may drop refs)
    except Exception as e:
        log_fn(f"[split] merged model failed validation ({e}); abandoning split")
        return None

    merged_b = badness(merged, _manager_gate_errors(
        merged, fc, manager_py=getattr(settings, "manager_py", False)), context={"fc": fc})
    log_fn(f"[split] merged {len(merged.links)} links, badness={merged_b:.2f}")
    return merged


def _decompose_best_of_n(spec, fc, settings, ctx, *, feedback=None, log_fn=print):
    """C7 — best-of-N sub by gate badness. Generate up to settings.sub_best_of independent
    manager decompositions of this subassembly and KEEP the one with the lowest pre-render
    badness (pure-Python gates + badness(), no render). Each decompose() already keep-bests
    its own retries; best-of-N adds independent SAMPLES on top so a single unlucky
    decomposition doesn't decide the sub. N<=1 (or a single clean candidate) is just one
    decompose. The winning model is (re)persisted to ctx.model_json_path.

    NOTE the cost: N decompositions = N x (manager_retries+1) LLM calls, so keep N small
    (default 2). Only used on the from-scratch path (the patch path stays a single edit)."""
    from .manager import decompose, save_model, _manager_gate_errors, _NONBLOCKING_CODES
    from .badness import badness

    n = max(1, int(getattr(settings, "sub_best_of", 2)))
    best = None                            # (badness, model)
    for i in range(n):
        try:
            model = decompose(spec.brief, settings, model_json_path=ctx.model_json_path,
                              frame_contract=fc, evaluator_feedback=feedback, log_fn=log_fn)
        except Exception as e:
            if n == 1 or best is None:
                if i == n - 1 and best is None:
                    raise                  # all candidates failed -> propagate
                log_fn(f"[best-of-{n}] candidate {i+1} failed ({e}); trying another")
                continue
            log_fn(f"[best-of-{n}] candidate {i+1} failed ({e}); keeping earlier best")
            continue
        errs = _manager_gate_errors(model, fc, manager_py=getattr(settings, "manager_py", False))
        blocking = [e for e in errs if e.code not in _NONBLOCKING_CODES]
        b = badness(model, errs, context={"fc": fc})
        log_fn(f"[best-of-{n}] candidate {i+1}/{n}: badness={b:.2f} "
               f"({len(blocking)} blocking)")
        if best is None or b < best[0]:
            best = (b, model)
        if not blocking:                   # a CLEAN candidate — no need to sample more
            log_fn(f"[best-of-{n}] candidate {i+1} is clean; stop sampling")
            break
    b, model = best
    save_model(model, ctx.model_json_path)
    if n > 1:
        log_fn(f"[best-of-{n}] kept the lowest-badness decomposition (badness {b:.2f})")
    return model


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

    # DETERMINISTIC MANAGER GATES (no LLM): schema validity (Phase 0) + per-sub overlap
    # and pose-graph connectivity (Phase 1), on the manager's model BEFORE any URDF build
    # or worker render. A failure fails the sub UP with the specific codes so the boss
    # re-decomposes only this sub — routine geometry/schema faults skip the slow debugger.
    # (Skipped on the patch path: only_links means a targeted re-render, not a fresh model.)
    if only_links is None and not getattr(settings, "manager_py", False):
        from .benchmarks import format_errors
        from .benchmarks.schema_gate import manager_schema_gate
        from .benchmarks.manager_gate import manager_gate, frame_drift_errors
        _pre_frames = _sub_frames_to_dict(model, fc.frames)
        all_errs = manager_schema_gate(model) + manager_gate(model, _pre_frames, fc)
        # FRAMES-REALIZED gate: every interface frame the boss contract requires must be
        # realized by a real link (after the auto-realize fallbacks). An unrealized frame
        # makes the assembler crash on "frame not realized", which — with no gate — loops
        # the whole pipeline for hours. Fail the sub FAST with the exact missing names so
        # the manager re-runs knowing precisely what to place. This is BLOCKING.
        from .benchmarks import GateError as _GateError
        _realized = {e["frame"] for e in _pre_frames}
        _missing = [fr.name for fr in fc.frames if fr.name not in _realized]
        for fn in _missing:
            all_errs.append(_GateError(
                "manager", "ERR_FRAME_UNREALIZED",
                f"interface frame '{fn}' is not realized by any link — the assembler "
                "cannot weld this subassembly. Place a real link at that frame's GLOBAL "
                "position and declare it in frames_realized (frame->link + local offset).",
                fn))
        # MOUNTS_PART enforcement: when the boss pinned a specific part to a seat frame
        # (mounts_part), the manager MUST realize that frame with THAT part — otherwise the
        # part isn't anchored to its hole and it drifts/overlaps other parts. A prompt asks
        # for this; this gate makes it binding. BLOCKING.
        _frame_link = {e["frame"]: e.get("link", "") for e in _pre_frames}
        for fr in fc.frames:
            want_part = (getattr(fr, "mounts_part", "") or "").strip()
            if not want_part or fr.name not in _frame_link:
                continue                       # no pin, or unrealized (owned by the check above)
            got = _frame_link[fr.name]
            if got != want_part:
                all_errs.append(_GateError(
                    "manager", "ERR_SEAT_PART",
                    f"seat frame '{fr.name}' must be realized by part '{want_part}' (the boss "
                    f"pinned that part to this hole), but it is realized by '{got}'. Make "
                    f"'{want_part}' the part at frame '{fr.name}' and mate it so it lands at "
                    f"the frame's position — this fixes each part to its own seat so they "
                    f"don't stack at the origin and overlap.",
                    fr.name))
        # C6 — FRAME DRIFT (self-consistency): a frame can be 'realized' but on the wrong
        # link / at the wrong offset. For the ROOT sub (pinned at the global origin) the
        # frames must hit the contract's ABSOLUTE coords; for a WELDED CHILD (rigidly
        # transformed onto its weld) only the RELATIVE layout of its frames must match the
        # contract — the sub's own origin is not the global origin. BLOCKING.
        _is_root = (sub_id == getattr(plan, "root_sub", None))
        all_errs.extend(frame_drift_errors(model, fc, is_root=_is_root,
                                           realized_frames=_pre_frames))
        # ERR_OVL from the PRE-render AABB check stays a WARNING (verified: it cannot be
        # made zero-false-positive). A DECLARED-box AABB grossly over-approximates non-boxy
        # parts: a radial cage of 8 thin ribs (each a 2 mm x 330 mm cylinder) rotated about
        # a common hub produces 44 near-total (95%+) box overlaps whose real 2 mm meshes only
        # kiss at the center — and even an OBB/SAT check still reports 26/28 rib pairs
        # overlapping, because thin rods through one hub genuinely share that space in the
        # envelope. So blocking here would break every legitimate cage/spoke design. The
        # AUTHORITATIVE overlap gate is the post-render real-mesh subcheck (the conflict gate
        # below, which DOES fail the sub up) plus the C5 dry-run compile. Schema codes +
        # ERR_CONNECT + the frame gates (all reliable on declared data) still BLOCK.
        mgr_errs = [e for e in all_errs if e.code != "ERR_OVL"]
        # ERR_OVL warnings come sorted worst-overlap-first from the gate. A dense coaxial
        # assembly (a tourbillon cage: 20+ thin discs stacked on one axis at different
        # heights) produces DOZENS of declared-box AABB overlaps that are mostly false
        # positives — the real-mesh subcheck is authoritative. So emit the structured
        # ARTIFACT_JSON for every one (the UI can show them all), but cap the human log to
        # the worst few + a summary so a single sub can't flood the console with 175 lines.
        _WARN_LOG_CAP = 8
        _warn_shown = 0
        _warn_total = sum(1 for e in all_errs if e.code == "ERR_OVL")
        for e in all_errs:
            blocking = e.code != "ERR_OVL"
            log_fn("ARTIFACT_JSON:" + json.dumps({
                "kind": "gate", "layer": "manager", "sub_id": sub_id, "code": e.code,
                "detail": e.detail, "culprit": e.culprit, "ok": (not blocking)}))
            if not blocking and _warn_shown < _WARN_LOG_CAP:
                slog(f"manager gate WARN {e}")
                _warn_shown += 1
        if _warn_total > _warn_shown:
            slog(f"manager gate WARN … +{_warn_total - _warn_shown} more overlap warning(s) "
                 "(declared-box AABB; the post-render real-mesh check is authoritative)")
        if mgr_errs:
            slog(f"manager gate FAILED with {len(mgr_errs)} blocking issue(s):")
            for e in mgr_errs:
                slog(f"  {e}")
            return SubResult(id=sub_id, ctx=ctx, model=model, ok=False,
                             error=("this subassembly failed automated checks; fix exactly "
                                    "these and rebuild:\n" + format_errors(mgr_errs)))
        slog("manager gate PASSED (schema + connectivity + frames; overlap warned only)")

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

    if getattr(settings, "manager_py", False):
        # 方案B: the manager authored CadQuery that ALREADY built + exported every part's STL
        # during parse (py_manager.evaluate_manager_python). The worker step is absorbed —
        # skip geometry generation entirely; the meshes/ dir is already populated.
        slog("worker absorbed into manager (manager_py): parts already built + exported")
        part_results = []
    elif to_build_model.links:
        slog(f"worker ({getattr(settings, 'worker_backend', 'cadquery')}): generating "
             "geometry + exporting per-link STLs ...")
        part_results = _worker_build_all(to_build_model, ctx, settings, log_fn=slog)
    else:
        part_results = []

    # DETERMINISTIC WORKER GATE (no LLM): for each part the worker just built, check its
    # CadQuery script honors the manager's declared dims and the STL is manifold. Only a
    # HARD mesh failure (ERR_MANIFOLD) fails the sub up for a rebuild; ERR_DIM is a WARNING
    # (the manager's size_mm is approximate — measured ~29% of valid parts legitimately
    # deviate for clearances/pitch-vs-outer radius — so it is surfaced but does not block).
    # CadQuery backend only (scripts live at <run>/cq/<link>.py).
    if (part_results and getattr(settings, "worker_backend", "cadquery") != "openscad"):
        from .benchmarks import format_errors
        from .benchmarks.worker_gate import worker_gate
        by_name = {l.name: l for l in to_build_model.links}
        hard_errs: list = []
        for r in part_results:
            if not getattr(r, "success", False):
                continue
            link = by_name.get(r.link_name)
            if link is None:
                continue
            script_path = Path(run_dir) / "cq" / f"{r.link_name}.py"
            script_text = ""
            if script_path.exists():
                try:
                    script_text = script_path.read_text(encoding="utf-8")
                except OSError:
                    script_text = ""
            stl_path = os.path.join(ctx.meshes_dir, f"{r.link_name}.stl")
            for e in worker_gate(script_text, link, stl_path):
                blocking = e.code != "ERR_DIM"
                log_fn("ARTIFACT_JSON:" + json.dumps({
                    "kind": "gate", "layer": "worker", "sub_id": sub_id,
                    "code": e.code, "detail": e.detail, "culprit": e.culprit,
                    "ok": (not blocking)}))
                if blocking:
                    hard_errs.append(e)
                else:
                    slog(f"worker gate WARN {e}")
        if hard_errs:
            slog(f"worker gate FAILED with {len(hard_errs)} blocking issue(s):")
            for e in hard_errs:
                slog(f"  {e}")
            return SubResult(id=sub_id, ctx=ctx, model=model, results=part_results,
                             ok=False,
                             error=("this subassembly's parts failed automated checks; fix "
                                    "exactly these and rebuild:\n" + format_errors(hard_errs)))
        slog("worker gate PASSED (manifold; dim divergences warned only)")

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
    slog(f"built {built}/{len(results)} links; URDF(with meshes) ok={ok2}"
         + (f" ({err2})" if not ok2 and err2 else ""))

    # Conflict gate: right after the worker built this sub's STLs, check for rigid parts
    # that interpenetrate (the manager places parts blind; the worker owns the geometry —
    # when they disagree, parts overlap). On a conflict, the debugger moves/reshapes the
    # offenders, then we recheck. Deep-think (Phase 6) picks the depth: FULL (whole sub,
    # extended thinking, sub_conflict_max_tries passes) vs SLIM (the 2 conflicting parts,
    # thinking off, 1 pass). If still stuck after the cap, FAIL UP so the boss re-plans.
    if success and getattr(settings, "enable_sub_conflict_gate", True):
        from . import subcheck, subdebugger
        dbg_mode = settings.debugger_mode() if hasattr(settings, "debugger_mode") else "full"
        dbg_backend = (settings.effective_worker_backend()
                       if hasattr(settings, "effective_worker_backend")
                       else getattr(settings, "worker_backend", "cadquery"))
        max_tries = (settings.debugger_max_tries()
                     if hasattr(settings, "debugger_max_tries")
                     else max(1, getattr(settings, "sub_conflict_max_tries", 3)))
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
                    frame_contract=fc, mode=dbg_mode, backend=dbg_backend, log_fn=slog)
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

    # POST-DEBUGGER FRAME RE-VALIDATION: the pre-render frame gates (ERR_FRAME_UNREALIZED +
    # C6 drift) ran on the manager's PRE-debug model. The conflict-debugger above can move
    # poses, re-save the model, and rewrite sub_frames.json — so a frame that was realized
    # at the right offset can drift, or a fallback-realized frame can vanish, with no gate
    # re-check. Re-run those exact frame checks on the FINAL model so the assembler never
    # welds a sub whose frames silently moved (the "3 subs float 24 mm away" failure). Same
    # guard as the pre-debug gate (fresh full build only); BLOCKING — fail the sub UP.
    if success and only_links is None and not getattr(settings, "manager_py", False):
        from .benchmarks import GateError as _GateError
        from .benchmarks import format_errors as _fmt_frames
        from .benchmarks.manager_gate import frame_drift_errors as _frame_drift
        _final_frames = _sub_frames_to_dict(model, fc.frames)
        _realized_now = {e["frame"] for e in _final_frames}
        _frame_errs = [
            _GateError("manager", "ERR_FRAME_UNREALIZED",
                       f"interface frame '{fr.name}' is not realized by any link after the "
                       "debugger pass — the assembler cannot weld this subassembly. Place a "
                       "real link at that frame's position and declare it in frames_realized.",
                       fr.name)
            for fr in fc.frames if fr.name not in _realized_now]
        _is_root = (sub_id == getattr(plan, "root_sub", None))
        _frame_errs.extend(_frame_drift(model, fc, is_root=_is_root,
                                        realized_frames=_final_frames))
        for e in _frame_errs:
            log_fn("ARTIFACT_JSON:" + json.dumps({
                "kind": "gate", "layer": "manager", "sub_id": sub_id, "code": e.code,
                "detail": e.detail, "culprit": e.culprit, "ok": False}))
        if _frame_errs:
            slog(f"post-debugger frame gate FAILED with {len(_frame_errs)} issue(s):")
            for e in _frame_errs:
                slog(f"  {e}")
            return SubResult(id=sub_id, ctx=ctx, model=model, results=results,
                             sub_frames=_final_frames, ok=False,
                             error=("this subassembly's interface frames drifted or went "
                                    "unrealized after the geometry fix; correct exactly "
                                    "these and rebuild:\n" + _fmt_frames(_frame_errs)))
        slog("post-debugger frame gate PASSED (all contract frames realized + in place)")

    # C5 — DRY-RUN COMPILE GATE (the final sub gate): the real-mesh conflict gate above
    # confirmed the parts don't interpenetrate; this confirms the sub actually LOADS in the
    # simulator that will score it. build_mjcf + MjModel.from_xml_path is the ultimate
    # zero-false-positive check — it IS the sim. A sub that won't compile fails UP so the
    # boss/manager fixes the offending part instead of the assembler crashing later or a
    # physics run wasting time on an unloadable machine. Full-build path only (the meshes
    # must be on disk); guarded + degrade-safe inside compile_gate.
    if success and only_links is None:
        from .benchmarks import format_errors as _fmt_compile
        from .benchmarks.compile_gate import compile_gate
        comp_errs = compile_gate(model, ctx, settings, log_fn=slog)
        for e in comp_errs:
            log_fn("ARTIFACT_JSON:" + json.dumps({
                "kind": "gate", "layer": "manager", "sub_id": sub_id, "code": e.code,
                "detail": e.detail, "culprit": e.culprit, "ok": False}))
        if comp_errs:
            slog(f"compile gate FAILED: {comp_errs[0]}")
            return SubResult(id=sub_id, ctx=ctx, model=model, results=results,
                             sub_frames=_sub_frames_to_dict(model, fc.frames), ok=False,
                             error=("this subassembly does not load in the simulator; fix "
                                    "exactly this and rebuild:\n" + _fmt_compile(comp_errs)))
        slog("compile gate PASSED (MJCF loads in MuJoCo)")

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
        r = _load_sub_from_disk(s.id, session_root, log_fn=log, plan=plan, settings=settings)
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
    # SAFETY: even "infinite" gets a hard ceiling so a fundamentally-unbuildable plan
    # (e.g. subs that never realize their interface frames) cannot loop for hours. And a
    # no-progress counter bails if we keep failing WITHOUT ever reaching the physics score
    # — repeating the same fault N times is not progress, it's a stuck loop.
    hard_ceiling = getattr(settings, "max_total_iters", 40)
    max_no_progress = getattr(settings, "max_no_progress_iters", 8)
    no_progress = 0                       # consecutive iters that never scored

    # Deep-think toggle (Phase 6): derive the geometry backend from deep_think so the
    # whole build path (worker + debugger) is driven by the one switch.
    settings.worker_backend = settings.effective_worker_backend()
    log_fn(f"[boss] deep_think={settings.deep_think} -> worker_backend="
           f"{settings.worker_backend}, debugger={settings.debugger_mode()}")

    slug = _slug_for(prompt)
    from datetime import datetime
    # Each run gets its OWN directory (slug + start timestamp). Without the stamp every run of
    # the same prompt reused ONE dir, so successive runs layered their sub_*/plan/run.log on top
    # of each other and the data read back was a mix of several runs. The stamp is computed once
    # per run_boss call, so all iterations of THIS run still share the one directory.
    _stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_root = os.path.abspath(os.path.join(out_dir, f"{slug}_boss_{_stamp}"))
    os.makedirs(session_root, exist_ok=True)

    # Tee every log line to <session_root>/run.log so the backend has the SAME full
    # transcript the UI's RAW LOG shows (fresh file per run — truncated on open).
    _run_log_path = os.path.join(session_root, "run.log")
    try:
        _run_log_fh = open(_run_log_path, "w", encoding="utf-8", buffering=1)
    except Exception:
        _run_log_fh = None

    def log(m):
        log_fn(m)
        if _run_log_fh is not None:
            try:
                _run_log_fh.write(str(m) + "\n")
            except Exception:
                pass
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

    # Keep-best / score-gated iteration (maker2-mujoco-contact Phase 5): keep an
    # iteration's change ONLY if the numeric design score improves; else revert to the
    # best-so-far subs and steer the next change differently. best_iter_reuse is the set
    # of ALL sub ids to reload from disk to reconstruct the best assembly.
    best_score = float("-inf")
    best_iter_reuse: set = set()
    iters_since_accept = 0
    score_target = float(getattr(settings, "score_target", 0.9))
    score_plateau = int(getattr(settings, "score_plateau", 3))

    # C15 — pre-physics whole-machine keep-best: track the lowest ASSEMBLED badness so a
    # re-plan that does not get the machine closer to buildable can be recognized (and the
    # boss steered to carve differently) BEFORE physics is ever reached. This is the missing
    # link that made every pre-physics loop a blind retry; the existing physics score-loop
    # below remains the FINAL keep-best once a score exists.
    best_assembled_badness = float("inf")
    assembled_stall = 0
    plateau_k = int(getattr(settings, "loop_plateau_k", 2))

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
        # SAFETY ceiling: absolute cap even in "infinite" mode, plus a no-progress bail.
        if it >= hard_ceiling:
            result["error"] = (f"stopped after {it} iterations (hard ceiling {hard_ceiling}) "
                               "without a passing design — the plan is likely unbuildable "
                               "(check that subassemblies realize their interface frames)")
            log(f"[boss] HARD CEILING {hard_ceiling} reached without success; giving up.")
            break
        if no_progress >= max_no_progress:
            result["error"] = (f"stopped after {no_progress} consecutive iterations with no "
                               "progress (never reached a physics score) — the pipeline is "
                               "stuck re-running the same failing stage; check the last "
                               "feedback for the blocking fault")
            log(f"[boss] NO PROGRESS for {no_progress} iterations; giving up to avoid a "
                "runaway loop.")
            break
        log(f"\n===== ITERATION {it} (boss{' re-plan' if feedback else ''}) =====")
        no_progress += 1                  # reset to 0 below once physics is reached
        judge_verdict = None            # set by the appearance judge below; fed to score

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

        # 1a. DETERMINISTIC BOSS GATES (no LLM): schema validity (Phase 0) then plan-level
        #     support + gear-mesh distance (Phase 2). A failure bounces the specific error
        #     codes back to the BOSS for a re-plan BEFORE any manager runs — routine plan
        #     faults never reach the slow debugger.
        from .benchmarks import format_errors
        from .benchmarks.schema_gate import boss_schema_gate
        from .benchmarks.boss_gate import boss_gate
        gate_errs = boss_schema_gate(plan) + boss_gate(plan)
        for e in gate_errs:
            log("ARTIFACT_JSON:" + json.dumps({
                "kind": "gate", "layer": "boss", "code": e.code,
                "detail": e.detail, "culprit": e.culprit, "ok": False}))
        if gate_errs:
            log(f"[boss] gate FAILED with {len(gate_errs)} issue(s); re-planning:")
            for e in gate_errs:
                log(f"[boss]   {e}")
            feedback = ("the plan failed automated checks; fix exactly these and re-plan:\n"
                        + format_errors(gate_errs))
            # Blame the named subs so unchanged ones can still reuse on the re-plan.
            replan_blamed = {e.culprit.split(":")[0] for e in gate_errs
                             if e.culprit and e.culprit.split(":")[0]
                             in {s.id for s in plan.subassemblies}}
            if not infinite and it >= max_boss_iters - 1:
                result["error"] = f"boss gate failed: {format_errors(gate_errs)}"; break
            it += 1; continue
        log("[boss] gate PASSED (schema + support + mesh distance)")

        # 1a. Authoritative geometry compile (flag-gated): for recognized topology,
        #     freeze a coherent zero-DOF hardpoint contract BEFORE any manager runs.
        #     The compiler and its gates are authoritative over every derived number;
        #     unrecognized topology falls back to the legacy hierarchy. Stashed on the
        #     plan so downstream stages can consume the frozen contract.
        plan.hardpoint_contract = None
        compiler_mode = getattr(settings, "geometry_compiler_mode", "auto")
        if compiler_mode != "legacy":
            try:
                from .design.bridge import compile_from_plan
                _compiled, _contract = compile_from_plan(
                    plan, prompt, mode=compiler_mode,
                    out_dir=os.path.join(session_root, "design"), log_fn=log)
                if _contract is not None:
                    plan.hardpoint_contract = _contract
                    # Unify the frame-name vocabulary: rewrite the boss plan's frame names +
                    # seam references to the AUTHORITATIVE compiler-contract names, so every
                    # name-keyed consumer (manager gate frame_drift, assembler mount lookup,
                    # slvs solve) compares like against like. Without this the boss uses
                    # 'seat_input_front' while the manager realizes 'housing_input_stage_front_
                    # bore', and every by-name gate silently no-ops -> collapsed seats slip
                    # through to the final solve. Best-effort; skips cleanly on any mismatch.
                    try:
                        from .boss import unify_plan_frame_names
                        n = unify_plan_frame_names(plan, _contract, log_fn=log)
                        if n:
                            log(f"[boss] unified {n} frame name(s) to the compiled contract")
                    except Exception as e:
                        log(f"[boss] frame-name unification skipped ({e})")
            except Exception as e:
                if compiler_mode == "required":
                    result["error"] = f"geometry compile failed: {e}"
                    break
                log(f"[boss] geometry compile skipped ({e}); using legacy_hierarchy")

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
        # Managers run as an AgentTeamRunner team (team_managers), collaborating over a
        # shared revisioned state seeded from the compiled hardpoint contract, instead of
        # the old one-way fan-out. Same {sub_id: SubResult} contract as before.
        from .team_managers import run_subassembly_team
        subs = run_subassembly_team(plan, settings, session_root,
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

        # 2b. DISJOINT-PARTS gate: every physical part belongs to exactly ONE subassembly. If
        #     the same part NAME was built in two subs, the boss duplicated it across sub
        #     briefs — at assembly the two copies overlap 100% and the debugger can't fix it
        #     (each copy is pinned to an interface frame, immovable, and no stage can delete a
        #     duplicate). So catch it HERE and re-plan (a plan fault, not a per-sub build
        #     fault) with the exact duplicated names, so the boss re-partitions parts disjointly.
        part_subs: dict = {}
        for s in plan.subassemblies:
            sr = subs.get(s.id)
            if sr is None or getattr(sr, "model", None) is None:
                continue
            for l in sr.model.links:
                part_subs.setdefault(l.name, set()).add(s.id)
        dup_parts = {name: sorted(ss) for name, ss in part_subs.items() if len(ss) > 1}
        if dup_parts:
            # NOTE: this is WARN-only, not a re-plan trigger. The assembler namespaces every
            # link as "<sub_id>_<name>", so two subs each owning a part called 'bearing_lower'
            # become 'input_stage_bearing_lower' vs 'output_stage_bearing_lower' — DISTINCT in
            # the final model, no collision. Rejecting this raw-name overlap was a FALSE
            # POSITIVE that forced a boss re-plan; the boss then re-partitioned + RENAMED its
            # subs, which defeated the id-based reuse and made the SAME fault recur under new
            # names every iteration -> the run looped to the no-progress cap. The only real
            # risk (the SAME shared physical part built twice, landing two copies at one
            # interface frame) is a geometry overlap, which the overlap/precheck gates own.
            dup_list = "; ".join(f"'{n}' in {ss}" for n, ss in sorted(dup_parts.items()))
            log_fn("ARTIFACT_JSON:" + json.dumps({
                "kind": "gate", "layer": "boss", "iter": it, "code": "ERR_DUP_PARTS",
                "detail": dup_list, "culprit": ",".join(sorted(dup_parts)), "ok": True}))
            log(f"[boss] disjoint-parts WARN (namespaced, not blocking): {len(dup_parts)} "
                f"part name(s) reused across subs ({dup_list})")

        # 2c. CROSS-SUB FRAME-AGREEMENT gate: the two subs a seam joins are built in isolation;
        #     verify both realized their shared interface frame on the SAME feature (each landed
        #     where the boss declared it). Catches a manager collapsing a seat onto its root
        #     origin (the shaft then buries itself in the housing). Deterministic, no LLM ->
        #     boss re-plan (rebuild the blamed sub).
        from .benchmarks.manager_gate import seam_frame_agreement_errors as _seam_agree
        _agree_errs = _seam_agree(plan, subs)
        if _agree_errs:
            for e in _agree_errs:
                log_fn("ARTIFACT_JSON:" + json.dumps({
                    "kind": "gate", "layer": "manager", "sub_id": e.culprit,
                    "code": e.code, "detail": e.detail, "ok": False}))
            # Gate-fault debugger first: a collapsed seat is a realization fault fixable IN PLACE
            # (re-point the frame onto its real bore) — no rebuild, no re-plan.
            if _try_contract_repair("frame_agree", _agree_errs[0].detail, plan, subs, settings, log):
                _agree_errs2 = _seam_agree(plan, subs)
                if not _agree_errs2:
                    log("[boss] frame-agreement repaired in place (no rebuild)")
                    _agree_errs = []
                else:
                    _agree_errs = _agree_errs2
        if _agree_errs:
            blamed_subs = {e.culprit for e in _agree_errs}   # culprit = the misrealizing sub
            for sid in blamed_subs:
                feedback_by_sub[sid] = (_agree_errs[0].detail + " — rebuild this subassembly, "
                                        "realizing its interface frame on the real part.")
            reuse = {s.id for s in plan.subassemblies} - blamed_subs
            log(f"[boss] frame-agreement FAILED -> rebuild {sorted(blamed_subs)}: "
                f"{_agree_errs[0].detail[:100]}")
            if not infinite and it >= max_boss_iters - 1:
                result["error"] = f"frame disagreement: {_agree_errs[0].detail}"; break
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
            final = assembler.assemble(plan, subs, assembly_ctx, settings=settings,
                                       log_fn=log)
        except assembler.AssemblerError as e:
            repaired = False
            if (getattr(e, "kind", "") == "authoritative_solver"
                    and getattr(settings, "enable_solver_failure_analyzer", True)
                    and getattr(e, "failure_report", None)):
                try:
                    from .assembly_repair import (LocalRepairTransaction,apply_candidate,
                        generate_repair_candidates,solver_repair_acceptance)
                    from .assembly_analyzer import analyze_failure
                    configured=max(1,int(getattr(settings,"solver_local_repair_max_attempts",2)))
                    # One candidate per currently failing key, plus the configured floor. The
                    # report may expose several independent rear/gear failures sequentially.
                    max_local=max(configured,len(str(e.failure_report.get('error','')).split(';'))+2)
                    current_report=e.failure_report
                    current_report_path=e.report_path
                    initial=generate_repair_candidates(current_report,plan,subs,settings)
                    affected=sorted({sid for c in initial
                                     for sid in c.rebuild_scope.get("subassemblies",[])})
                    with LocalRepairTransaction(subs,affected) as tx:
                        for local_attempt in range(1,max_local+1):
                            candidates=(initial if local_attempt==1 else
                                generate_repair_candidates(current_report,plan,subs,settings))
                            decision=analyze_failure(session_root,current_report,candidates,
                                settings,log,report_path=current_report_path,
                                source="slvs",attempt=local_attempt)
                            log("ARTIFACT_JSON:"+json.dumps({
                                "kind":"assembly_analysis","iter":it,
                                "local_attempt":local_attempt,
                                "failure_id":current_report.get("failure_id"),
                                "decision":decision}))
                            cid=(decision.get("selected_candidate_id")
                                 if decision.get("decision")=="repair" else None)
                            cand=next((c for c in candidates
                                       if c.candidate_id==cid and c.allowed),None)
                            if cand is None:
                                log(f"[analyzer] escalation: {decision.get('escalation_reason') or decision.get('root_cause')}")
                                break
                            escaped=[sid for sid in cand.rebuild_scope.get("subassemblies",[])
                                     if sid not in affected]
                            if escaped:
                                tx.ids.update(escaped);tx.commit();affected.extend(escaped)
                            log(f"[analyzer] attempt {local_attempt}/{max_local} selected "
                                f"{cand.candidate_id}: {cand.rationale}")
                            apply_candidate(cand,subs,settings,log)
                            repair_ctx=make_run_context(plan.name,session_root,
                                run_dir=os.path.join(assembly_ctx.run_dir,
                                                     f"local_repair_{local_attempt}"))
                            try:
                                final=assembler.assemble(plan,subs,repair_ctx,
                                                         settings=settings,log_fn=log)
                            except assembler.AssemblerError as next_error:
                                if (getattr(next_error,"kind","")!="authoritative_solver"
                                        or not getattr(next_error,"failure_report",None)):
                                    raise
                                acceptance=solver_repair_acceptance(current_report,
                                    next_error.failure_report,cand)
                                if not acceptance['accepted']:
                                    tx.rollback()
                                    log(f"[analyzer] attempt {local_attempt} rejected and rolled back: {acceptance}")
                                    break
                                tx.commit()
                                current_report=next_error.failure_report
                                current_report_path=next_error.report_path
                                log(f"[analyzer] attempt {local_attempt} accepted as strict improvement; "
                                    f"checkpointed with remaining failures: {current_report.get('error')}")
                                continue
                            assembly_ctx=repair_ctx
                            repaired=True
                            log(f"[analyzer] localized repair solved in iteration {it} "
                                f"after {local_attempt} attempt(s)")
                            break
                        if not repaired:
                            # Keep the last checkpointed strict improvements. Only the most
                            # recent rejected step is rolled back at its rejection site.
                            log("[analyzer] localized sequence stopped with accepted checkpoints preserved")
                except Exception as ae:
                    log(f"[analyzer] local repair failed safely ({type(ae).__name__}: {ae})")
            if not repaired:
                # Structural/name faults retain the existing deterministic-then-LLM repair.
                if _try_contract_repair("assembler", str(e), plan, subs, settings, log):
                    try:
                        final = assembler.assemble(plan, subs, assembly_ctx,
                                                   settings=settings, log_fn=log)
                        log("[assembler] re-assembled after contract repair (no re-plan)")
                    except assembler.AssemblerError as e2:
                        feedback = f"assembly failed: {e2}"
                        log(f"[assembler] FAILED after repair -> boss re-plan: {e2}")
                        if not infinite and it >= max_boss_iters - 1:
                            result["error"] = f"assembly failed: {e2}"; break
                        it += 1; continue
                else:
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

        # 4a. POST-ASSEMBLE gear-mesh distance gate: the boss no longer authors gear-center
        #     coordinates, so mesh spacing is validated on the COMPILER'S solved world frames
        #     (final.assembly_frames_world), not the plan. If two meshing gears didn't land
        #     one pitch-center-distance apart, the weld geometry is wrong -> re-plan (same
        #     path as an AssemblerError). Deterministic, no LLM.

        # 4a-0. MODULE-CONSISTENCY gate: two gears mesh ONLY if they share the same module
        #       (tooth size). A pair with mismatched modules cannot mesh at ANY spacing, so
        #       catch it here (read module off the BUILT gear parts, resolved role-based via the
        #       mesh FRAME, not the boss's guessed mesh_pair name) and route to a re-plan before
        #       the spacing check. Deterministic, no LLM.
        from .assembler import _gear_link as _mesh_gear_link
        _mod_errs = []
        for _seam in plan.seams:
            if _seam.kind != "power":
                continue
            _psub = subs.get(_seam.parent_sub); _csub = subs.get(_seam.child_sub)
            if not _psub or not _csub or _psub.model is None or _csub.model is None:
                continue
            _gp = _mesh_gear_link(_psub, _seam, 0); _gc = _mesh_gear_link(_csub, _seam, 1)
            if _gp is None or _gc is None:
                continue                          # unresolved -> the assembler gate owns it
            def _mod_of(_lk):
                try:
                    return float((_lk.size_mm or {}).get("module"))
                except (TypeError, ValueError):
                    return None
            _mp0 = _mod_of(_gp); _mp1 = _mod_of(_gc)
            if _mp0 and _mp1 and abs(_mp0 - _mp1) > 1e-6:
                _mod_errs.append((_seam, _gp.name, _mp0, _gc.name, _mp1))
        if _mod_errs:
            _s, _a, _ma, _b, _mb = _mod_errs[0]
            _detail = (f"gears '{_a}' (module {_ma:g}) and '{_b}' (module {_mb:g}) are meshed "
                       f"by seam '{_s.id}' but have DIFFERENT modules — different tooth sizes "
                       f"cannot mesh at any spacing. Give both gears the same module.")
            log_fn("ARTIFACT_JSON:" + json.dumps({
                "kind": "gate", "layer": "boss", "code": "ERR_MESH_MODULE",
                "detail": _detail, "culprit": f"{_a}~{_b}", "ok": False}))
            feedback = "assembly mesh check failed: " + _detail
            log(f"[assembler] mesh MODULE mismatch -> boss re-plan: {_detail}")
            if not infinite and it >= max_boss_iters - 1:
                result["error"] = feedback; break
            it += 1; continue

        from .benchmarks.boss_gate import mesh_distance_errors as _mesh_dist
        _mesh_errs = _mesh_dist(plan, getattr(final, "assembly_frames_world", []))
        for e in _mesh_errs:
            log_fn("ARTIFACT_JSON:" + json.dumps({
                "kind": "gate", "layer": "boss", "code": e.code, "detail": e.detail,
                "culprit": e.culprit, "ok": False}))
        if _mesh_errs:
            feedback = "assembly mesh check failed: " + "; ".join(e.detail for e in _mesh_errs)
            log(f"[assembler] mesh distance FAILED -> boss re-plan: {_mesh_errs[0].detail}")
            if not infinite and it >= max_boss_iters - 1:
                result["error"] = feedback; break
            it += 1; continue

        # 4b. Silent overlap auto-nudge (Session B item 1b): separate any subassemblies
        #     that interpenetrate but share NO seam so THIS assembly closes, and tell the
        #     blamed managers to fix their placement next iteration (a nudge is a hint,
        #     not a rebuild trigger — it never sets `feedback`).
        try:
            nudges = assembler.auto_nudge_overlaps(final, plan, subs, assembly_ctx,
                                                   settings=settings, log_fn=log)
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
        # 5a. DETERMINISTIC ASSEMBLED-SUPPORT GATE (no LLM): the ONLY grounding/support
        #     check — subs aren't free-standing, so support is a whole-machine property.
        #     Weld chain to root_sub + the base must be the lowest structural part. A
        #     failure is an interface fault -> boss re-plan.
        from .benchmarks import format_errors as _fmt_gate
        from .benchmarks.assembled_gate import assembled_gate
        asm_all = assembled_gate(plan, assembly_ctx.urdf_path, log_fn=log)
        # ERR_SUP_FLOAT (weld chain) is orientation-INDEPENDENT and blocks. ERR_SUP_GROUND
        # (which sub is lowest) is orientation-DEPENDENT — a watch laid dial-down
        # legitimately puts the motion works below the mainplate — so it only WARNS, it
        # does not re-plan (validated: it false-fired on a real tourbillon).
        asm_errs = [e for e in asm_all if e.code == "ERR_SUP_FLOAT"]
        for e in asm_all:
            blocking = e.code == "ERR_SUP_FLOAT"
            log("ARTIFACT_JSON:" + json.dumps({
                "kind": "gate", "layer": "assembled", "iter": it, "code": e.code,
                "detail": e.detail, "culprit": e.culprit, "ok": (not blocking)}))
            if not blocking:
                log(f"[assembled] support gate WARN {e}")
        if asm_errs:
            feedback = ("the assembled machine failed automated support checks; fix "
                        "exactly these and re-plan:\n" + _fmt_gate(asm_errs))
            replan_blamed = {e.culprit for e in asm_errs if e.culprit
                             and e.culprit in {s.id for s in plan.subassemblies}}
            log(f"[assembled] support gate FAILED -> boss re-plan (rebuild "
                f"{sorted(replan_blamed) or 'affected subs'}; others reused)")
            if not infinite and it >= max_boss_iters - 1:
                result["error"] = f"assembled support failed: {_fmt_gate(asm_errs)}"; break
            it += 1; continue
        log("[assembled] support gate PASSED (weld chain; grounding warned only)")

        rep = precheck_mod.precheck(plan, subs, assembly_ctx.urdf_path, log_fn=log)
        precheck_path = os.path.join(assembly_ctx.run_dir, "precheck_report.json")
        os.makedirs(assembly_ctx.run_dir, exist_ok=True)
        with open(precheck_path, "w", encoding="utf-8") as f:
            json.dump(rep.to_dict(), f, indent=2)
        log("ARTIFACT_JSON:" + json.dumps({
            "kind": "precheck", "iter": it, **rep.to_dict()}))
        if not rep.ok and getattr(settings, "enable_precheck_failure_analyzer", True):
            repaired_precheck = False
            try:
                from .assembly_repair import (LocalRepairTransaction, apply_candidate,
                    generate_precheck_repair_candidates, precheck_repair_acceptance)
                from .assembly_analyzer import analyze_failure
                baseline = rep.to_dict()
                candidates = generate_precheck_repair_candidates(
                    baseline, plan, subs, assembly_ctx, settings)
                if not candidates:
                    raise RuntimeError("no proven precheck repair candidate")
                affected = sorted({sid for c in candidates if c.allowed
                                   for sid in c.rebuild_scope.get("subassemblies", [])})
                max_local = max(1, int(getattr(settings, "precheck_local_repair_max_attempts", 2)))
                with LocalRepairTransaction(subs, affected) as tx:
                    for local_attempt in range(1, max_local + 1):
                        attempt_ctx = make_run_context(plan.name, session_root,
                            run_dir=os.path.join(assembly_ctx.run_dir,
                                                 f"precheck_local_repair_{local_attempt}"))
                        os.makedirs(attempt_ctx.run_dir, exist_ok=True)
                        with open(os.path.join(attempt_ctx.run_dir, "candidate_set.json"),
                                  "w", encoding="utf-8") as f:
                            json.dump([c.to_dict() for c in candidates], f, indent=2)
                        decision = analyze_failure(session_root, baseline, candidates, settings,
                            log, report_path=precheck_path, source="precheck",
                            attempt=local_attempt)
                        with open(os.path.join(attempt_ctx.run_dir, "analyzer_decision.json"),
                                  "w", encoding="utf-8") as f:
                            json.dump(decision, f, indent=2)
                        cid = (decision.get("selected_candidate_id")
                               if decision.get("decision") == "repair" else None)
                        cand = next((c for c in candidates
                                     if c.candidate_id == cid and c.allowed), None)
                        if cand is None:
                            log(f"[precheck-analyzer] escalation: "
                                f"{decision.get('escalation_reason') or decision.get('root_cause')}")
                            break
                        if any(sid not in affected for sid in
                               cand.rebuild_scope.get("subassemblies", [])):
                            raise RuntimeError("candidate escaped the precheck transaction scope")
                        apply_candidate(cand, subs, settings, log)
                        changed_links=set(cand.rebuild_scope.get("links", []))
                        declared_links=set(cand.target.get("physical_links", []))
                        if changed_links != declared_links:
                            raise RuntimeError("candidate rebuild scope does not cover every changed link")
                        repaired_final = assembler.assemble(plan, subs, attempt_ctx,
                                                             settings=settings, log_fn=log)
                        next_rep = precheck_mod.precheck(
                            plan, subs, attempt_ctx.urdf_path, log_fn=log)
                        next_report = next_rep.to_dict()
                        with open(os.path.join(attempt_ctx.run_dir, "precheck_report.json"),
                                  "w", encoding="utf-8") as f:
                            json.dump(next_report, f, indent=2)
                        acceptance = precheck_repair_acceptance(baseline, next_report, cand)
                        with open(os.path.join(attempt_ctx.run_dir, "acceptance.json"),
                                  "w", encoding="utf-8") as f:
                            json.dump(acceptance, f, indent=2)
                        if not acceptance["accepted"]:
                            log(f"[precheck-analyzer] attempt {local_attempt} rejected: "
                                f"{acceptance}")
                            tx.rollback()
                            break
                        final = repaired_final
                        assembly_ctx = attempt_ctx
                        rep = next_rep
                        precheck_path = os.path.join(attempt_ctx.run_dir,
                                                     "precheck_report.json")
                        repaired_precheck = True
                        log(f"[precheck-analyzer] localized physical repair accepted in "
                            f"iteration {it}, attempt {local_attempt}")
                        break
                    if not repaired_precheck:
                        tx.rollback()
            except Exception as ae:
                log(f"[precheck-analyzer] local repair failed safely "
                    f"({type(ae).__name__}: {ae})")
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

        # C15/C16 — WHOLE-MACHINE keep-best (pre-physics). The machine assembled AND passed
        # the geometry pre-check, so it is buildable enough to score deterministically. Track
        # the lowest assembled badness; if this iteration did not get the machine closer to
        # buildable for plateau_k iterations running, ESCALATE — steer the next re-plan to
        # CARVE THE MACHINE DIFFERENTLY rather than re-tuning the same split. This makes the
        # pre-physics loop monotonic; the physics score-loop below is the final keep-best.
        asm_badness = _assembled_badness(plan, subs, rep, asm_errs)
        escalate_carve = False
        if asm_badness < best_assembled_badness - 1e-3:
            best_assembled_badness = asm_badness
            assembled_stall = 0
            log(f"[boss] assembled badness {asm_badness:.2f} (new pre-physics best)")
        else:
            assembled_stall += 1
            log(f"[boss] assembled badness {asm_badness:.2f} did not beat best "
                f"{best_assembled_badness:.2f} (pre-physics stall "
                f"{assembled_stall}/{plateau_k})")
            if assembled_stall >= plateau_k:
                assembled_stall = 0
                escalate_carve = True
                log("[boss] pre-physics plateau -> escalate: the next re-plan should CARVE "
                    "THE MACHINE DIFFERENTLY (different subassembly boundaries), not re-tune "
                    "the same split.")

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
                judge_verdict = verdict           # fed into the keep-best score below
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
        no_progress = 0                   # reached a physics score -> real progress

        # ---- Keep-best / score-gated iteration (Phase 5) ----
        # Compute a numeric design score from physics + precheck + the appearance judge.
        # Keep this iteration's change ONLY if the score beats best-so-far; else REVERT
        # to the best subs and steer the next change differently. Stop on target/plateau.
        from .score import score as _score
        eps = 1e-3
        try:
            s_val, s_break = _score(phys, rep, judge_verdict, settings)
        except Exception as e:
            log(f"[score] scoring failed ({e}); treating as neutral 0.0")
            s_val, s_break = 0.0, {"score": 0.0, "terms": {}}
        result["score"] = s_val
        result["score_breakdown"] = s_break
        log(f"[score] iteration {it}: score={s_val:.3f} "
            f"(best={best_score if best_score > float('-inf') else 'none'}) "
            f"terms={s_break.get('terms')}")
        log("ARTIFACT_JSON:" + json.dumps({
            "kind": "physics", "iter": it, "run_dir": assembly_ctx.run_dir,
            "render_dir": assembly_ctx.run_dir, "passed": phys.get("passed"),
            "score": round(s_val, 4), "score_breakdown": s_break, "physics": phys}))

        if phys.get("passed") is None:
            log("[boss] physics errored/unavailable -> stop with current assembly.")
            break

        accepted = s_val > best_score + eps
        if accepted:
            best_score = s_val
            best_iter_reuse = {s.id for s in plan.subassemblies}
            iters_since_accept = 0
            # Snapshot the winning subs so a later revert restores THIS geometry (sub
            # dirs are overwritten in place on rebuild, so reuse alone isn't enough).
            _snapshot_best_subs(session_root, best_iter_reuse, log_fn=log)
            log(f"[boss] ACCEPT iteration {it}: new best score {s_val:.3f}")
            # TRACK1: memory append hook — on a newly-accepted best score, remember each
            # PASSING sub's skeleton + parts so future runs can retrieve a worked example
            # (kb.search merges curated + memory). Best-effort, gated by enable_kb; a
            # missing index / absent RAG deps is a silent no-op, never fails the run.
            if getattr(settings, "enable_kb", False):
                try:
                    from . import kb
                    from .benchmarks.manager_gate import frame_drift_errors as _fd
                    note = ""
                    if judge_verdict is not None:
                        note = (getattr(judge_verdict, "memory_note", "")
                                or (judge_verdict.reasons or "")[:200])
                    # A sub good enough to SHIP is not automatically good enough to TEACH:
                    # a remembered design is retrieved as a worked example, so promote only
                    # subs that ALSO realize every contract frame in place (zero drift /
                    # unrealized on the hardened, fallback-resolved check). This stops a
                    # marginal sub that squeaked past the ship bar (e.g. collapsed mount
                    # frames) from becoming a poisoned exemplar that steers future runs wrong.
                    passing_models = {}
                    for sid, sr in subs.items():
                        if not (getattr(sr, "ok", False) and getattr(sr, "model", None)):
                            continue
                        _fc = frame_contract_for(plan, sid)
                        _is_root = (sid == getattr(plan, "root_sub", None))
                        _drift = _fd(sr.model, _fc, is_root=_is_root,
                                     realized_frames=getattr(sr, "sub_frames", None) or None)
                        if _drift:
                            log(f"[kb] NOT teaching '{sid}': {len(_drift)} frame issue(s) "
                                "(ships, but not a clean exemplar)")
                            continue
                        passing_models[sid] = sr.model
                    kb.remember_passing_subs(passing_models, collection="manager",
                                             score=s_val, note=note, log_fn=log)
                except Exception as e:
                    log(f"[kb] memory append skipped: {e}")
        else:
            iters_since_accept += 1
            log(f"[boss] REJECT iteration {it}: score {s_val:.3f} did not beat best "
                f"{best_score:.3f} -> revert to best + steer differently "
                f"({iters_since_accept}/{score_plateau} since last improvement)")

        # Stop conditions: target reached (must also PASS), max iters, or plateau.
        if s_val >= score_target and phys.get("passed"):
            log(f"[boss] score {s_val:.3f} >= target {score_target} and physics PASS "
                f"-> done.")
            break
        if not infinite and it >= max_boss_iters - 1:
            result["error"] = f"physics failed: {phys.get('summary')}"; break
        if iters_since_accept >= score_plateau:
            log(f"[boss] plateau: {score_plateau} iterations with no score improvement "
                f"-> stop with best (score {best_score:.3f}).")
            break

        # On REVERT, restore the best iteration's subs from the snapshot and reuse them
        # from disk; steer the NEXT change differently (do NOT follow this worse
        # iteration's fault blame).
        if not accepted and best_iter_reuse:
            _restore_best_subs(session_root, best_iter_reuse, log_fn=log)
            reuse = set(best_iter_reuse)
            feedback = None
            feedback_by_sub = {sid: (
                f"the previous change LOWERED the design score to {s_val:.3f} (best is "
                f"{best_score:.3f}); revert it and try a DIFFERENT fix for: "
                f"{phys.get('summary', '')}") for sid in best_iter_reuse}
            it += 1
            continue


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
        # C16 — if the pre-physics loop plateaued this iteration, steer the boss to carve
        # differently on a re-plan instead of re-tuning the same split (escalate, don't
        # repeat). Only meaningful when this iteration set a boss-level `feedback`.
        if escalate_carve and feedback:
            feedback += ("\n\nESCALATION: recent iterations have NOT gotten the machine "
                         "closer to buildable with this subassembly split. CHANGE THE "
                         "DECOMPOSITION — draw the subassembly boundaries differently (merge "
                         "or re-partition subs), do not just re-tune the current split.")
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


def _assembled_badness(plan, subs, rep, asm_errs) -> float:
    """C15 — a WHOLE-MACHINE pre-physics badness so a boss re-plan can be kept only if the
    assembled machine got CLOSER to buildable, bridging the gap until physics is reachable
    (the physics score-loop then takes over as the final keep-best). Lower = closer. It is
    built ONLY from RELIABLE assembled-stage signals:
      * precheck violations, weighted by severity (interface faults dominate — the pieces
        don't fit and the boss must re-plan; a 'sub' fault re-runs one manager),
      * assembled_gate errors (weld-chain / grounding),
      * the count of subs that failed to build (each a hard hole in the machine).
    Deliberately EXCLUDES the per-sub declared-AABB overlap term (badness()'s overlap_vol):
    on cage/spoke subs it is a huge unreliable artifact (see the ERR_OVL note) that would
    swamp the real fit signals here. No LLM, no physics; reuses the precheck/assembled
    reports already computed in the loop."""
    total = 0.0
    for v in getattr(rep, "violations", []) or []:
        total += 6.0 if getattr(v, "severity", "") == "interface" else 3.0
    total += 2.0 * len(asm_errs or [])
    total += 4.0 * sum(1 for s in plan.subassemblies
                       if not getattr(subs.get(s.id), "ok", False))
    return round(total, 3)


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
    from .team_managers import run_subassembly_team
    subs = run_subassembly_team(plan, settings, session_root,
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
