#!/usr/bin/env python3
"""Export an assembled maker2 URDF to a single binary glTF (.glb).

The cadam-style canvas wants ONE colored, orbitable solid, not the per-link STLs.
``viz.load_robot`` already assembles the URDF (meshes positioned by the joint
transforms) with per-link <material> colors, so its trimesh scene exports straight
to GLB — colors and all.

  python -m maker2.export_glb output/<run>/model.urdf            # writes <run>/_assembled.glb
  python -m maker2.export_glb output/<run>/model.urdf --stdout   # raw GLB bytes to stdout

The HTTP route (worker/src/routes/api/run-maker2-glb.ts) shells the --stdout form
and streams the bytes back to the browser.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from maker2 import viz


def export_glb(urdf_path: str) -> bytes:
    """Assemble the URDF and return GLB bytes (binary glTF, colors preserved)."""
    robot = viz.load_robot(urdf_path)
    # Same as the render paths (render_png/six_views): rescue any link that loaded
    # WITHOUT a real material color (trimesh's default near-gray/white). Skipping
    # this made the canvas GLB pale/washed-out with parts barely distinguishable.
    viz._colorize_fallback(robot)
    data = robot.scene.export(file_type="glb")
    return data if isinstance(data, (bytes, bytearray)) else bytes(data)


def main() -> int:
    ap = argparse.ArgumentParser(description="maker2: assembled URDF -> .glb")
    ap.add_argument("urdf", help="path to the assembled model.urdf")
    ap.add_argument("--out", default=None,
                    help="output .glb path (default: _assembled.glb next to the URDF)")
    ap.add_argument("--stdout", action="store_true",
                    help="write raw GLB bytes to stdout instead of a file")
    a = ap.parse_args()

    if not os.path.exists(a.urdf):
        print(f"urdf not found: {a.urdf}", file=sys.stderr)
        return 1

    data = export_glb(a.urdf)

    if a.stdout:
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
        return 0

    out = a.out or os.path.join(os.path.dirname(a.urdf), "_assembled.glb")
    with open(out, "wb") as f:
        f.write(data)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
