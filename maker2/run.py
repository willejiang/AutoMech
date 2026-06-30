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
from maker2.scad_worker import build_all
from maker2.urdf_builder import build_urdf, scaffold_meshes, validate_urdf


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


def run(prompt: str, out_dir: str = "output", manager_only: bool = False,
        allow_partial: bool = False, model: str | None = None,
        do_judge: bool = True, do_physics: bool = False,
        max_iters: int = 2) -> dict:
    settings = Settings()
    settings.allow_partial = allow_partial
    if model:
        # The cadam UI passes provider-prefixed ids (e.g. "anthropic/claude-opus-4.8"),
        # but this gateway wants the bare name ("claude-opus-4.8") — strip the prefix.
        settings.model = model.split("/", 1)[-1]
    print(f"[run] model: {settings.model}")
    print(f"[run] prompt: {prompt}")
    print(f"[run] max_iters: {max_iters}")

    # The result reflects the BEST/LAST iteration. Each iteration gets its own dir.
    result = {"ok": False, "run_dir": "", "urdf_path": "",
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
        print("[1/3] manager: decomposing into links + joints ...")
        try:
            model_obj = decompose(prompt, settings, model_json_path=ctx.model_json_path,
                                  evaluator_feedback=feedback, log_fn=print)
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
        print("[2/3] cadam SCAD worker: generating .scad + rendering per-link STLs ...")
        results = build_all(model_obj, ctx, settings, log_fn=print)
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

    # ---- refine loop: re-decompose (manager) on a judge FAIL, up to max_iters ----
    feedback = None
    last_ctx = None
    for it in range(max(1, max_iters)):
        patch, ctx, results, model_obj = _one_iteration(it, feedback)
        result.update(patch)
        result["iterations"] = it + 1
        last_ctx = ctx
        Path(ctx.run_dir, "result.json").write_text(json.dumps(result, indent=2))

        judge = patch.get("judge")
        if not patch["ok"]:
            break                                 # build failed — don't keep looping
        if manager_only:
            break
        if judge is None or judge.get("passed"):
            break                                 # judged PASS (or no judge) — done
        if it == max(1, max_iters) - 1:
            print(f"[loop] reached max_iters ({max_iters}) without a PASS verdict.")
            break
        # judge FAIL -> feed its suggestions to the manager for the next pass.
        feedback = (judge.get("suggestions") or judge.get("reasons") or "").strip() or None
        print(f"[loop] judge FAIL -> re-decomposing with feedback for iteration {it+1}.")

    # ---- physics ONCE on the final (best) URDF ----
    if do_physics and result["ok"] and last_ctx is not None:
        print("[physics] PyBullet stability test on maker2's URDF ...")
        try:
            from maker2.physics import run_physics
            result["physics"] = run_physics(result["urdf_path"], prompt, last_ctx.run_dir)
        except Exception as e:
            print(f"[physics] failed: {e}")
            result["physics"] = {"passed": None, "summary": f"physics error: {e}"}
        Path(last_ctx.run_dir, "result.json").write_text(json.dumps(result, indent=2))

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
    ap.add_argument("--max-iters", type=int, default=2,
                    help="refine passes: judge FAIL -> re-decompose -> rebuild (default 2)")
    a = ap.parse_args()
    res = run(a.prompt, a.out, a.manager_only, a.allow_partial, a.model,
              do_judge=not a.no_judge, do_physics=a.physics, max_iters=a.max_iters)
    if a.json:
        print("RESULT_JSON:" + json.dumps(res))
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
