#!/usr/bin/env python3
"""maker2 driver: prompt -> manager -> URDF contract -> cadam SCAD worker -> URDF.

Mirrors makerv2's Orchestrator phases 1 & 3, but phase 2 is the single cadam
SCAD worker (scad_worker.build_all) instead of N parallel FreeCAD workers.

  python -m maker2.run "a 2-DOF pan-tilt camera mount"
  python -m maker2.run "a desk lamp with an articulated arm" --manager-only
  python -m maker2.run "..." --out output --allow-partial
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


def run(prompt: str, out_dir: str = "output", manager_only: bool = False,
        allow_partial: bool = False) -> int:
    settings = Settings()
    settings.allow_partial = allow_partial
    ctx = make_run_context(prompt, out_dir)
    os.makedirs(ctx.logs_dir, exist_ok=True)
    print(f"[run] dir: {ctx.run_dir}")
    print(f"[run] prompt: {prompt}")

    # Phase 1: manager -> KinematicModel + URDF contract + empty scaffolds.
    print("[1/3] manager: decomposing into links + joints ...")
    model = decompose(prompt, settings, model_json_path=ctx.model_json_path,
                      log_fn=print)
    build_urdf(model, ctx)
    ok, err = validate_urdf(ctx.urdf_path, require_meshes=False)
    if not ok:
        print(f"[1/3] FAIL: URDF topology invalid: {err}")
        return 1
    scaffold_meshes(model, ctx)
    njoint = sum(1 for j in model.joints if j.type != "fixed")
    print(f"[1/3] model: {len(model.links)} links, {len(model.joints)} joints "
          f"({njoint} movable) -> {ctx.urdf_path}")

    if manager_only:
        print("[done] --manager-only: URDF contract written, geometry skipped.")
        return 0

    # Phase 2: ONE cadam SCAD worker fills every link's mesh.
    print("[2/3] cadam SCAD worker: generating .scad + rendering per-link STLs ...")
    results = build_all(model, ctx, settings, log_fn=print)
    built = sum(1 for r in results if r.success)
    for r in results:
        tag = "OK " if r.success else "FAIL"
        print(f"      [{tag}] {r.link_name}"
              + (f" :: {r.error[:120]}" if not r.success else ""))

    # Phase 3: re-validate with meshes loaded.
    ok2, err2 = validate_urdf(ctx.urdf_path, require_meshes=True)
    print(f"[3/3] URDF valid (with meshes): {ok2}" + (f" :: {err2}" if not ok2 else ""))

    success = (built == len(results)) or (allow_partial and built > 0)
    print("-" * 56)
    print(f"RESULT: {'PASS' if success and ok2 else 'FAIL'} — "
          f"{built}/{len(results)} links built. Bundle: {ctx.run_dir}")
    return 0 if (success and ok2) else 1


def main() -> int:
    _load_dotenv()
    ap = argparse.ArgumentParser(description="maker2: manager + cadam SCAD worker -> URDF")
    ap.add_argument("prompt", help="natural-language product description")
    ap.add_argument("--out", default="output")
    ap.add_argument("--manager-only", action="store_true",
                    help="stop after the URDF contract (no geometry)")
    ap.add_argument("--allow-partial", action="store_true",
                    help="succeed even if some links fail to build")
    a = ap.parse_args()
    return run(a.prompt, a.out, a.manager_only, a.allow_partial)


if __name__ == "__main__":
    sys.exit(main())
