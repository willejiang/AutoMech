#!/usr/bin/env python3
"""
automech_loop.py — the AutoMech outer automation loop.

Ties cadam (the maker) to the Isaac Sim evaluator into one closed loop:

  prompt -> cadam(.scad) -> render 6 views -> VISUAL GATE (VLM, no physics)
         -> author manifest/URDF (VLM) -> Isaac Sim evaluator -> verdict
         -> PASS? done : fix_hint -> back to cadam

Two feedback cadences (per DESIGN_LOOP.md):
  FAST  — compile error or visual-gate suggestions: short-circuits BEFORE any
          sim, so a broken or wrong-looking model never burns a GPU run.
  SLOW  — evaluator fix hints, after a full Isaac pass.

Each iteration writes a self-contained design dir under runs/<id>/iter_<N>/:
  model.scad, model.stl, views/*.png, gate.json, manifest.json, out/result.json
The manifest's source.file -> model.scad, so evaluator/evaluate.sh consumes the
dir unchanged.

OFFLINE: the Aliyun box is down -> use --dry-run to stub the sim verdict and
exercise the whole wiring without a GPU.

  python orchestrator/automech_loop.py \
     --task "quarter-car suspension that clears a 10cm curb" \
     --dry-run --max-iters 3
"""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cadam_bridge
import render_views
import visual_gate
import urdf_author
import evaluator_bridge

RUNS = Path(__file__).resolve().parent / "runs"


def _slug(task):
    s = "".join(c if c.isalnum() else "-" for c in task.lower()).strip("-")
    return s[:40] or "design"


def run(task, max_iters=3, dry_run=False, run_id=None):
    run_id = run_id or f"{_slug(task)}-{int(time.time())}"
    wd = RUNS / run_id
    wd.mkdir(parents=True, exist_ok=True)
    print(f"[loop] run dir: {wd}")
    print(f"[loop] task: {task}")
    if dry_run:
        print("[loop] DRY RUN — Isaac Sim stubbed (box offline).")

    feedback = None       # carries either fast (gate/compile) or slow (sim) feedback
    history = []

    for it in range(max_iters):
        print(f"\n===== ITERATION {it} =====", flush=True)
        idir = wd / f"iter_{it}"
        idir.mkdir(parents=True, exist_ok=True)

        # 1) MAKER: cadam generates .scad (revision if we have feedback)
        print("[loop] cadam generating .scad ...", flush=True)
        scad = cadam_bridge.generate(task, feedback)
        scad_path = idir / "model.scad"
        scad_path.write_text(scad, encoding="utf-8")

        # 2) RENDER: native OpenSCAD CLI -> STL + 6 views + module names
        r = render_views.render(scad_path, idir)
        print(f"[loop] render: compile_ok={r['compile_ok']} "
              f"views={len(r['view_pngs'])} modules={r['module_names']}", flush=True)

        if not r["compile_ok"]:
            # FAST feedback: didn't compile — back to cadam, no gate/sim.
            feedback = f"The OpenSCAD code failed to compile:\n{r['compile_err']}"
            history.append({"iter": it, "stage": "compile", "ok": False})
            print(f"[loop] FAST feedback (compile): {r['compile_err'][:200]}", flush=True)
            continue

        # 3) VISUAL GATE: VLM judges geometry/prompt-match (no physics)
        gate = visual_gate.judge(task, r["view_pngs"], r["view_names"])
        (idir / "gate.json").write_text(json.dumps(gate, indent=2))
        print(f"[loop] visual gate: passed={gate['passed']} :: {gate['reason'][:160]}",
              flush=True)

        if not gate["passed"]:
            # FAST feedback: looks wrong — back to cadam, still no sim.
            feedback = f"Visual review rejected the model: {gate['suggestions'] or gate['reason']}"
            history.append({"iter": it, "stage": "visual_gate", "ok": False})
            print(f"[loop] FAST feedback (gate): {feedback[:200]}", flush=True)
            continue

        # 4) AUTHOR: VLM writes the manifest/URDF for the articulated assembly
        print("[loop] authoring manifest/URDF ...", flush=True)
        manifest = urdf_author.author(task, scad_path, r["module_names"],
                                      r["view_pngs"], feedback,
                                      stl_path=r.get("stl_path"))
        manifest.setdefault("design_id", f"{run_id}_iter{it}")
        manifest.setdefault("user_prompt", task)
        (idir / "manifest.json").write_text(json.dumps(manifest, indent=2))
        print(f"[loop] manifest: {len(manifest.get('parts', []))} parts, "
              f"{len(manifest.get('joints', []))} joints", flush=True)

        # 5) EVALUATOR: design dir -> Isaac Sim -> VLM verdict (or stub)
        result = evaluator_bridge.evaluate(idir, dry_run=dry_run)
        passed = bool(result.get("passed"))
        print(f"[loop] evaluator: passed={passed}"
              f"{' (stub)' if result.get('stubbed') else ''} :: "
              f"{result.get('summary','')[:200]}", flush=True)

        history.append({"iter": it, "stage": "evaluate",
                        "passed": passed, "stubbed": result.get("stubbed", False)})

        if passed:
            print(f"\n[loop] *** SUCCESS at iteration {it} ***", flush=True)
            break

        # SLOW feedback: real (or stubbed) physics failure -> back to cadam.
        fixes = "; ".join(f.get("fix_hint", "") for f in result.get("failures", [])
                          if f.get("fix_hint"))
        feedback = f"Physics evaluation failed: {result.get('summary','')}\nFixes: {fixes}"
        print(f"[loop] SLOW feedback (sim): {feedback[:240]}", flush=True)

        if it == max_iters - 1:
            print(f"\n[loop] reached max iters ({max_iters}) without PASS.", flush=True)

    (wd / "history.json").write_text(json.dumps(history, indent=2))
    print(f"\n[loop] done. history: {json.dumps(history)}")
    print(f"[loop] artifacts under {wd}")
    return history


def main():
    ap = argparse.ArgumentParser(description="AutoMech: cadam + Isaac Sim automation loop")
    ap.add_argument("--task", required=True, help="natural-language design task")
    ap.add_argument("--max-iters", type=int, default=3)
    ap.add_argument("--dry-run", action="store_true",
                    help="stub the Isaac Sim step (box offline / no GPU)")
    ap.add_argument("--run-id", default=None)
    a = ap.parse_args()
    run(a.task, a.max_iters, a.dry_run, a.run_id)


if __name__ == "__main__":
    main()
