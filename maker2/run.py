#!/usr/bin/env python3
"""maker2 driver: prompt -> manager -> URDF contract -> cadam SCAD worker -> URDF
-> (optional) appearance judge -> (optional) PyBullet physics.

Mirrors makerv2's Orchestrator phases 1 & 3, but phase 2 is the single cadam
SCAD worker (scad_worker.build_all). A machine-readable result.json is written to
the run dir so the cadam UI bridge (worker/src/routes/api/run-maker2.ts) can parse
the outcome.

  python -m maker2.run "a 2-DOF pan-tilt camera mount"
  python -m maker2.run "..." --model anthropic/claude-opus-4.8 --json
  python -m maker2.run "..." --physics            # also run a rigid stability test
  python -m maker2.run "..." --manager-only
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from maker2.config import Settings
from maker2.manager import decompose
from maker2.orchestrator import make_run_context
from maker2.urdf_builder import build_urdf, scaffold_meshes, validate_urdf


def _worker_build_all(model, ctx, settings, log_fn=print):
    """Build geometry via the configured backend (settings.worker_backend): CadQuery
    by default, OpenSCAD as the legacy fallback. Same build_all(...) contract."""
    backend = getattr(settings, "worker_backend", "cadquery")
    if backend == "openscad":
        from maker2.scad_worker import build_all as _ba
    else:
        from maker2.cq_worker import build_all as _ba
    return _ba(model, ctx, settings, log_fn=log_fn)


def _load_dotenv():
    """Load orchestrator/.env so OPENSCAD_BIN/OPENSCADPATH + gateway are set."""
    for p in (Path(__file__).resolve().parents[1] / "orchestrator" / ".env",
              Path(__file__).resolve().parent / ".env"):
        if p.exists():
            for line in p.read_text().splitlines():
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    os.environ.setdefault(k, v.strip())


def _judge(prompt, model, results, ctx, settings):
    """Render 6 views of the assembled URDF and ask the appearance judge.
    Degrades to a text-only verdict if headless rendering can't produce images."""
    from maker2 import viz, judger
    view_pngs = {}
    try:
        view_pngs = viz.render_six_views(ctx.urdf_path,
                                         os.path.join(ctx.run_dir, "views"))
        print(f"[judge] rendered {len(view_pngs)} views")
    except Exception as e:
        print(f"[judge] render failed ({e}); judging text-only")
    try:
        v = judger.judge(prompt, model, results, view_pngs, settings,
                         out_json_path=os.path.join(ctx.run_dir, "judge.json"),
                         log_fn=print)
        return {"passed": bool(v.passed), "reasons": v.reasons,
                "suggestions": v.suggestions, "views": len(view_pngs)}
    except Exception as e:
        print(f"[judge] judge failed: {e}")
        return {"passed": None, "reasons": f"judge error: {e}",
                "suggestions": "", "views": len(view_pngs)}


def _append_thread(out_dir, thread, message, result, model_id):
    """Append this run as a turn to output/threads/<thread>/thread.json."""
    try:
        from datetime import datetime, timezone
        tdir = Path(out_dir, "threads", thread)
        tdir.mkdir(parents=True, exist_ok=True)
        tpath = tdir / "thread.json"
        doc = json.loads(tpath.read_text()) if tpath.exists() else {
            "id": thread, "created_at": datetime.now(timezone.utc).isoformat(),
            "model": model_id, "turns": [],
        }
        judge = result.get("judge") or {}
        doc["turns"].append({
            "message": message,
            "run_dir": result.get("run_dir", ""),
            "render_dir": result.get("render_dir", ""),
            "ok": bool(result.get("ok")),
            "hard_failed": bool(result.get("hard_failed")),
            "judge_passed": judge.get("passed"),
            "entry": result.get("entry", "rebuild"),
            "ts": datetime.now(timezone.utc).isoformat(),
        })
        tpath.write_text(json.dumps(doc, indent=2))
        print(f"[thread] appended turn -> {tpath}")
    except Exception as e:
        print(f"[thread] could not update thread.json: {e}")


def _run_skip(entry: str, prompt: str, message: str | None, prior_model: str,
              settings, out_dir: str, do_physics: bool, thread: str | None) -> dict:
    """Re-run ONLY the evaluator on the prior turn's already-built model — no
    manager, no worker. `entry` is retest | reframe | revise_scenario. The prior
    model's run_dir holds model.urdf + meshes/; we copy them into a fresh run dir so
    this turn has its own result/video, then run physics."""
    import shutil
    from datetime import datetime, timezone
    prior_dir = os.path.dirname(os.path.abspath(prior_model))
    prior_urdf = os.path.join(prior_dir, "model.urdf")
    result = {"ok": False, "run_dir": "", "urdf_path": "", "render_dir": "",
              "hard_failed": False, "links": 0, "joints": 0, "movable_joints": 0,
              "built": 0, "judge": None, "physics": None, "iterations": 1,
              "error": "", "entry": entry}
    if not os.path.exists(prior_urdf):
        result["error"] = f"skip '{entry}': prior model has no URDF ({prior_urdf})"
        print(f"[skip] {result['error']}")
        return result

    ctx = make_run_context(prompt, out_dir)
    os.makedirs(ctx.meshes_dir, exist_ok=True)
    shutil.copy2(prior_urdf, ctx.urdf_path)
    if os.path.exists(os.path.join(prior_dir, "kinematic_model.json")):
        shutil.copy2(os.path.join(prior_dir, "kinematic_model.json"),
                     ctx.model_json_path)
    prior_meshes = os.path.join(prior_dir, "meshes")
    if os.path.isdir(prior_meshes):
        for f in os.listdir(prior_meshes):
            shutil.copy2(os.path.join(prior_meshes, f),
                         os.path.join(ctx.meshes_dir, f))
    result.update(run_dir=ctx.run_dir, urdf_path=ctx.urdf_path,
                  render_dir=ctx.run_dir, ok=True)
    print(f"[skip] entry={entry}: reusing prior model in {ctx.run_dir} "
          f"(no manager/worker)")
    # The reused model is renderable now -> show it on the canvas immediately.
    print("ARTIFACT_JSON:" + json.dumps({
        "kind": "model", "iter": 0, "run_dir": ctx.run_dir,
        "render_dir": ctx.run_dir, "judge_passed": None}))

    # The evaluator instruction: for reframe/revise_scenario, steer physics via the
    # task text; run_physics' own VLM diagnosis + retries then handle the specifics.
    task = prompt
    if entry == "reframe" and message:
        task = f"{prompt} [reframe the camera: {message}]"
    elif entry == "revise_scenario" and message:
        task = f"{prompt} [change the test: {message}]"

    if do_physics:
        try:
            from maker2.physics import run_physics
            result["physics"] = run_physics(ctx.urdf_path, task, ctx.run_dir, settings)
        except Exception as e:
            print(f"[skip] physics failed: {e}")
            result["physics"] = {"passed": None, "cause": "none",
                                 "summary": f"physics error: {e}"}
        phys = result["physics"] or {}
        print("ARTIFACT_JSON:" + json.dumps({
            "kind": "physics", "iter": 0, "run_dir": ctx.run_dir,
            "render_dir": ctx.run_dir, "passed": phys.get("passed"),
            "physics": phys}))
    Path(ctx.run_dir, "result.json").write_text(json.dumps(result, indent=2))
    try:
        Path(ctx.run_dir, "run.json").write_text(json.dumps({
            "prompt": prompt, "model": settings.model, "max_iters": 0,
            "refine_message": message, "thread": thread, "entry": entry,
            "created_at": datetime.now(timezone.utc).isoformat()}, indent=2))
    except Exception:
        pass
    if thread:
        _append_thread(out_dir, thread, message or prompt, result, settings.model)
    print("-" * 56)
    print(f"RESULT (skip:{entry}): physics="
          f"{(result.get('physics') or {}).get('verdict')} Bundle: {ctx.run_dir}")
    return result


def run(prompt: str, out_dir: str = "output", manager_only: bool = False,
        allow_partial: bool = False, model: str | None = None,
        do_judge: bool = True, do_physics: bool = False,
        max_iters: int = 0, refine_message: str | None = None,
        prior_model: str | None = None, thread: str | None = None,
        entry: str = "rebuild", engine: str | None = None) -> dict:
    settings = Settings()
    settings.allow_partial = allow_partial
    if engine:
        settings.engine = engine
    if model:
        # The cadam UI passes provider-prefixed ids (e.g. "anthropic/claude-opus-4.8"),
        # but this gateway wants the bare name ("claude-opus-4.8") — strip the prefix.
        settings.model = model.split("/", 1)[-1]
    print(f"[run] model: {settings.model}")
    print(f"[run] prompt: {prompt}")
    print(f"[run] max_iters: {max_iters}")

    # Multi-turn refine: load the prior turn's model JSON (a file path) once so
    # iteration 0 can start from it. A missing/unreadable file just falls back to
    # a cold decompose (no crash).
    prior_model_json = None
    if prior_model:
        try:
            prior_model_json = Path(prior_model).read_text(encoding="utf-8")
            print(f"[run] refine from prior model: {prior_model}")
        except Exception as e:
            print(f"[run] could not read prior model ({e}); cold decompose")
    if refine_message:
        print(f"[run] refine message: {refine_message}")

    # SKIP: a follow-up that isn't a redesign shouldn't restart the slow manager.
    # If the caller didn't force --entry, classify the message to pick the cheapest
    # correct entry (rebuild / retest / reframe / revise_scenario). The three
    # non-rebuild entries re-run ONLY the evaluator on the prior model.
    if refine_message and entry == "rebuild" and prior_model:
        try:
            from maker2.diagnose import classify_followup
            gw = {"base_url": settings.base_url, "api_key": settings.api_key,
                  "model": settings.model}
            summary = ""
            if prior_model_json:
                try:
                    pm = json.loads(prior_model_json)
                    summary = (f"{pm.get('name','model')}: "
                               f"{len(pm.get('links', []))} links, "
                               f"{len(pm.get('joints', []))} joints")
                except Exception:
                    pass
            c = classify_followup(refine_message, summary, None, gw)
            entry = c["entry"]
            print(f"[run] follow-up classified as '{entry}': {c['reason']}")
        except Exception as e:
            print(f"[run] follow-up classify failed ({e}); rebuilding")
            entry = "rebuild"

    if entry != "rebuild" and prior_model:
        return _run_skip(entry, prompt, refine_message, prior_model, settings,
                         out_dir, do_physics, thread)

    # The result reflects the BEST/LAST iteration. Each iteration gets its own dir.
    # `render_dir` tracks the last iteration that actually BUILT a renderable model
    # (urdf + meshes); the canvas shows THAT, so a judge-FAIL (or a later crashed
    # iteration) still displays the last good render instead of nothing.
    result = {"ok": False, "run_dir": "", "urdf_path": "", "render_dir": "",
              "hard_failed": False,
              "links": 0, "joints": 0, "movable_joints": 0, "built": 0,
              "judge": None, "physics": None, "iterations": 0, "error": ""}

    def _one_iteration(it: int, feedback: str | None) -> tuple[dict, object, list, object]:
        """Run manager(+feedback) -> URDF -> worker -> judge for one pass.
        Returns (iter_result_patch, ctx, worker_results, model_obj)."""
        ctx = make_run_context(prompt, out_dir,
                               run_dir=os.path.join(out_dir, f"iter_{it}_{int.from_bytes(os.urandom(2),'big')}")
                               if it > 0 else None)
        os.makedirs(ctx.logs_dir, exist_ok=True)
        patch = {"run_dir": ctx.run_dir, "urdf_path": ctx.urdf_path,
                 "links": 0, "joints": 0, "movable_joints": 0, "built": 0,
                 "judge": None, "ok": False, "error": ""}
        print(f"\n===== ITERATION {it} (feedback: {'yes' if feedback else 'none'}) =====")

        # Phase 1: manager -> URDF contract. On a re-pass, judge suggestions go IN.
        # On a multi-turn refine, iteration 0 also gets the prior model + the
        # user's change request (later iterations refine via judge feedback).
        print("[1/3] manager: decomposing into links + joints ...")
        try:
            model_obj = decompose(
                prompt, settings, model_json_path=ctx.model_json_path,
                evaluator_feedback=feedback,
                refine_message=refine_message if it == 0 else None,
                prior_model_json=prior_model_json if it == 0 else None,
                log_fn=print)
        except Exception as e:
            patch["error"] = f"manager failed: {e}"
            print(f"[1/3] FAIL: {patch['error']}")
            return patch, ctx, [], None
        build_urdf(model_obj, ctx)
        ok, err = validate_urdf(ctx.urdf_path, require_meshes=False)
        if not ok:
            patch["error"] = f"URDF topology invalid: {err}"
            return patch, ctx, [], model_obj
        scaffold_meshes(model_obj, ctx)
        njoint = sum(1 for j in model_obj.joints if j.type != "fixed")
        patch.update(links=len(model_obj.links), joints=len(model_obj.joints),
                     movable_joints=njoint)
        print(f"[1/3] model: {len(model_obj.links)} links, {len(model_obj.joints)} "
              f"joints ({njoint} movable) -> {ctx.urdf_path}")

        if manager_only:
            patch["ok"] = True
            print("[done] --manager-only: URDF contract written, geometry skipped.")
            return patch, ctx, [], model_obj

        # Phase 2: the worker fills geometry FROM the manager's URDF.
        print(f"[2/3] worker ({getattr(settings, 'worker_backend', 'cadquery')}): "
              "generating geometry + exporting per-link STLs ...")
        results = _worker_build_all(model_obj, ctx, settings, log_fn=print)
        built = sum(1 for r in results if r.success)
        patch["built"] = built
        for r in results:
            print(f"      [{'OK ' if r.success else 'FAIL'}] {r.link_name}"
                  + (f" :: {r.error[:120]}" if not r.success else ""))

        # Phase 3: re-validate with meshes loaded.
        ok2, err2 = validate_urdf(ctx.urdf_path, require_meshes=True)
        print(f"[3/3] URDF valid (with meshes): {ok2}" + (f" :: {err2}" if not ok2 else ""))
        success = (built == len(results)) or (allow_partial and built > 0)
        patch["ok"] = bool(success and ok2)

        # Phase 4: appearance judge on the assembled URDF.
        if do_judge and patch["ok"]:
            print("[judge] appearance judge on the assembled URDF ...")
            patch["judge"] = _judge(prompt, model_obj, results, ctx, settings)
        return patch, ctx, results, model_obj

    # ---- TWO NESTED LOOPS ----------------------------------------------------
    # MAIN loop (always infinite): maker-subloop -> physics; a physics STRUCTURE
    #   failure rebuilds. Stops only on physics PASS, a physics error, a hard build
    #   failure, or the process being killed (the browser tab closing aborts the SSE).
    #   `max_iters` does NOT cap this loop.
    # MAKER subloop (capped by `max_iters`): manager(+feedback) -> worker -> judge.
    #   Judge FAIL -> rebuild for appearance. If the judge keeps failing past
    #   `max_iters`, we DON'T end the run — we fall through to physics with the
    #   current model (an ugly-but-testable model still deserves a physics verdict).
    #   `max_iters == 0` means INFINITE judge retries (never fall through until the
    #   judge passes). The subloop counter RESETS every main iteration.
    feedback = None                      # carried into the manager each rebuild
    last_ctx = None
    judge_infinite = max_iters <= 0
    total_it = 0                         # monotonic: unique run dir + UI iteration id
    while True:                          # ===== MAIN LOOP (infinite) =====
        sub_it = 0                       # reset the maker-subloop budget each main pass
        last_judge = None                # last judge verdict this subloop (for feedback)
        while True:                      # ----- MAKER SUBLOOP (capped) -----
            patch, ctx, results, model_obj = _one_iteration(total_it, feedback)
            result.update(patch)
            result["iterations"] = total_it + 1
            last_ctx = ctx
            # Remember the last iteration that produced a renderable model, so the
            # canvas can fall back to it if a later pass regresses or the judge fails.
            if patch.get("ok"):
                result["render_dir"] = ctx.run_dir
            result["hard_failed"] = not result["render_dir"]
            Path(ctx.run_dir, "result.json").write_text(json.dumps(result, indent=2))

            # Tell the UI a renderable model exists NOW (regardless of judge verdict),
            # so the canvas can show THIS iteration's build immediately.
            if patch.get("ok"):
                jp = (patch.get("judge") or {}).get("passed")
                print("ARTIFACT_JSON:" + json.dumps({
                    "kind": "model", "iter": total_it, "run_dir": ctx.run_dir,
                    "render_dir": result["render_dir"], "judge_passed": jp}))

            judge = patch.get("judge")
            if not patch["ok"]:
                break                    # build failed — leave the subloop (main breaks too)
            if manager_only:
                break

            last_judge = judge
            # Judge PASS -> leave the subloop and go test physics.
            if judge is None or judge.get("passed"):
                break
            # Judge FAIL -> rebuild for appearance, UNLESS the cap says fall through.
            if not judge_infinite and sub_it >= max_iters - 1:
                print(f"[loop] judge still FAIL after {max_iters} maker attempt(s); "
                      f"passing the current model to physics anyway.")
                break
            feedback = (judge.get("suggestions") or judge.get("reasons") or "").strip() or None
            print(f"[loop] judge FAIL -> re-decomposing (maker attempt {sub_it+2}).")
            sub_it += 1
            total_it += 1
            continue                     # ----- end MAKER SUBLOOP iteration -----

        # A hard build failure or --manager-only ends the whole run.
        if not patch["ok"] or manager_only:
            break

        # Gate 2: physics on the resulting model. FAIL(structure) rebuilds via the
        # MAIN loop (which resets the maker subloop). scenario/framing were already
        # retried inside run_physics, so anything failing here is treated structural.
        if not do_physics:
            break                        # judge phase done, physics not requested
        render_urdf = os.path.join(result["render_dir"], "model.urdf")
        print("[physics] evaluating the assembled URDF ...")
        try:
            from maker2.physics import run_physics
            result["physics"] = run_physics(render_urdf, prompt, result["render_dir"],
                                            settings)
        except Exception as e:
            print(f"[physics] failed: {e}")
            result["physics"] = {"passed": None, "cause": "none",
                                 "summary": f"physics error: {e}"}
        Path(ctx.run_dir, "result.json").write_text(json.dumps(result, indent=2))

        # The recording(s) exist NOW — surface them regardless of pass/fail.
        phys = result["physics"] or {}
        print("ARTIFACT_JSON:" + json.dumps({
            "kind": "physics", "iter": total_it,
            "run_dir": ctx.run_dir, "render_dir": result["render_dir"],
            "passed": phys.get("passed"), "physics": phys}))
        if phys.get("passed"):
            print("[loop] physics PASS -> done.")
            break
        if phys.get("passed") is None:
            print("[loop] physics errored/unavailable -> stop with current model.")
            break
        # physics FAIL -> rebuild via the MAIN loop. Feed the manager the physics
        # reason AND the last judge suggestions (structure is the driver, but the
        # appearance notes still help the next decomposition).
        reason = (phys.get("reason") or phys.get("summary") or "").strip()
        jsug = ""
        if last_judge is not None:
            jsug = (last_judge.get("suggestions") or last_judge.get("reasons") or "").strip()
        feedback = (f"The physics test FAILED: {reason}. The mechanism must actually "
                    f"work when driven — fix the structure so it does.")
        if jsug:
            feedback += f" Also address the appearance issues: {jsug}"
        print(f"[loop] physics FAIL (cause={phys.get('cause')}) -> "
              f"rebuilding (main iteration {total_it+2}).")
        total_it += 1
        continue                         # ===== end MAIN LOOP iteration =====


    # ---- history sidecar: make the run self-describing for the UI sidebar ----
    # result.json has no prompt/model, so the disk-scan lister can't title or
    # reopen a run from it alone. Drop a tiny run.json next to it.
    if result.get("run_dir"):
        try:
            from datetime import datetime, timezone
            now_iso = datetime.now(timezone.utc).isoformat()
            Path(result["run_dir"], "run.json").write_text(json.dumps({
                "prompt": prompt,
                "model": settings.model,
                "max_iters": max_iters,
                "refine_message": refine_message,
                "thread": thread,
                "created_at": now_iso,
            }, indent=2))
        except Exception as e:
            print(f"[run] could not write run.json: {e}")

    # ---- thread: append this run as a turn so the UI can show a conversation ----
    if thread and result.get("run_dir"):
        _append_thread(out_dir, thread, refine_message or prompt, result, settings.model)

    print("-" * 56)
    print(f"RESULT: {'PASS' if result['ok'] else 'FAIL'} — "
          f"{result['built']}/{result['links']} links built over "
          f"{result['iterations']} iteration(s). Bundle: {result['run_dir']}")
    return result


def main() -> int:
    # Windows consoles default to cp1252 and crash on non-Latin-1 output (the
    # manager's part names, ·/→ glyphs in logs). Force UTF-8 like stl_to_urdf.py.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            pass
    _load_dotenv()
    ap = argparse.ArgumentParser(description="maker2: manager + cadam SCAD worker -> URDF")
    ap.add_argument("prompt", help="natural-language product description")
    ap.add_argument("--out", default="output")
    ap.add_argument("--model", default=None, help="LLM for manager + worker (e.g. anthropic/claude-opus-4.8)")
    ap.add_argument("--manager-only", action="store_true",
                    help="stop after the URDF contract (no geometry)")
    ap.add_argument("--allow-partial", action="store_true",
                    help="succeed even if some links fail to build")
    ap.add_argument("--no-judge", action="store_true", help="skip the appearance judge")
    ap.add_argument("--physics", action="store_true", help="also run a PyBullet stability test")
    ap.add_argument("--json", action="store_true", help="print result.json as the LAST line")
    ap.add_argument("--max-iters", type=int, default=0,
                    help="MAKER-SUBLOOP cap: judge-FAIL rebuilds for appearance up to "
                         "this many attempts, then falls through to physics anyway. "
                         "0 = infinite judge retries. The MAIN loop (physics-driven "
                         "rebuild) is always infinite regardless of this value.")
    ap.add_argument("--refine-message", default=None,
                    help="multi-turn: the user's change request for the prior model")
    ap.add_argument("--prior-model", default=None,
                    help="multi-turn: path to the prior turn's kinematic_model.json")
    ap.add_argument("--thread", default=None,
                    help="thread id: append this run as a turn to output/threads/<id>/thread.json")
    ap.add_argument("--entry", default="rebuild",
                    choices=["rebuild", "retest", "reframe", "revise_scenario"],
                    help="force the entry stage; default 'rebuild' lets a follow-up be "
                         "auto-classified (skip the manager when it's not a redesign)")
    ap.add_argument("--hierarchy", action="store_true",
                    help="BOSS mode: split the machine into subassemblies (boss), build "
                         "them in parallel under an interface contract, assemble, "
                         "pre-check, and physics-test — with surgical per-sub re-runs. "
                         "For machines too big for one manager (TBM, car).")
    ap.add_argument("--web", action="store_true",
                    help="enable web-search reference lookup for the boss/manager/worker "
                         "(they research standard dims / reference designs before "
                         "building). Keyless (DuckDuckGo/Bing). Hierarchy mode.")
    ap.add_argument("--per-sub-physics", action="store_true",
                    help="(hierarchy) drive each subassembly on its own URDF before "
                         "assembly, so a drivetrain fault localizes to that sub.")
    ap.add_argument("--engine", default=None, choices=["pybullet", "mujoco"],
                    help="physics engine: 'pybullet' (default, legacy joint motors) or "
                         "'mujoco' (pure contact under gravity — transmission by tooth "
                         "contact, no motors). Requires --physics to take effect.")
    ap.add_argument("--deep-think", dest="deep_think", action="store_true", default=None,
                    help="deep-think ON: CadQuery worker + FULL debugger (thorough, slow).")
    ap.add_argument("--no-deep-think", dest="deep_think", action="store_false",
                    help="deep-think OFF: OpenSCAD worker + SLIM debugger (fast, shallow).")
    a = ap.parse_args()
    if a.hierarchy:
        from maker2.orchestrator_boss import run_boss
        settings = None
        if a.web or a.engine or a.deep_think is not None:
            from maker2.config import Settings
            settings = Settings.load()
            if a.web:
                settings.enable_reference_tools = True
            if a.engine:
                settings.engine = a.engine
            if a.deep_think is not None:
                settings.deep_think = a.deep_think
        res = run_boss(a.prompt, a.out, settings=settings, do_physics=a.physics,
                       per_sub_physics=a.per_sub_physics, thread=a.thread,
                       refine_message=a.refine_message, log_fn=print)
        if a.json:
            print("RESULT_JSON:" + json.dumps(res))
        return 0 if res.get("ok") else 1
    res = run(a.prompt, a.out, a.manager_only, a.allow_partial, a.model,
              do_judge=not a.no_judge, do_physics=a.physics, max_iters=a.max_iters,
              refine_message=a.refine_message, prior_model=a.prior_model,
              thread=a.thread, entry=a.entry, engine=a.engine)
    if a.json:
        print("RESULT_JSON:" + json.dumps(res))
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
