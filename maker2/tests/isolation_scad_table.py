#!/usr/bin/env python3
"""No-LLM isolation test for the maker2 + cadam-SCAD contract.

Mirrors makerv2's tests/isolation_urdf_table.py, but proves the SCAD-worker seam
(not the FreeCAD one): a hand-built 2-link table KinematicModel -> a fixed,
hand-written 2-module .scad -> render each module to meshes/<link>.stl via the
native OpenSCAD CLI -> check_stl each -> build + validate the assembled URDF.

No LLM, no cadam — proves manager->worker->assemble works before wiring the LLM.

  python -m maker2.tests.isolation_scad_table
"""
from __future__ import annotations

import os
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from maker2.model import KinematicModel, LinkSpec, JointSpec, RunContext
from maker2.urdf_builder import build_urdf, scaffold_meshes, validate_urdf
from maker2.validation import check_stl
from maker2.scad_render import render_module, find_openscad


# A hand-written .scad: one top-level module per link, each at its LOCAL origin.
# table_top: a flat slab centered at origin. table_leg: a cylinder rising +Z from
# origin (its top — the attach point — sits at z=0, per the joint origin).
TABLE_SCAD = """\
// units: millimeters. each module builds ONE part at its own local origin.
module table_top() {
    translate([0,0,-10]) cube([400,400,20], center=true);
}
module table_leg() {
    // attach point (top of leg) at origin; leg extends downward
    translate([0,0,-200]) cylinder(h=400, r=15, center=true, $fn=32);
}
"""


def hand_model() -> KinematicModel:
    links = [
        LinkSpec(name="table_top", description="flat square table top",
                 shape_hint="box", size_mm={"x": 400, "y": 400, "z": 20},
                 origin_note="centered at origin", mesh_filename="meshes/table_top.stl"),
        LinkSpec(name="table_leg", description="single cylindrical leg",
                 shape_hint="cylinder", size_mm={"radius": 15, "height": 400},
                 origin_note="attach point at top, extends -Z",
                 mesh_filename="meshes/table_leg.stl"),
    ]
    joints = [
        JointSpec(name="top_to_leg", type="fixed", parent="table_top",
                  child="table_leg", xyz_m=(0.18, 0.18, -0.01)),
    ]
    return KinematicModel(name="table", root_link="table_top",
                          links=links, joints=joints)


def main() -> int:
    oscad = find_openscad()
    if not oscad:
        print("[FAIL] OpenSCAD CLI not found (set OPENSCAD_BIN).")
        return 1
    print(f"[0] OpenSCAD: {oscad}")

    run_dir = Path(tempfile.gettempdir()) / f"maker2_table_{int(time.time())}"
    ctx = RunContext(
        project_slug="table", run_dir=str(run_dir),
        urdf_path=str(run_dir / "model.urdf"),
        meshes_dir=str(run_dir / "meshes"),
        logs_dir=str(run_dir / "logs"),
        model_json_path=str(run_dir / "kinematic_model.json"),
    )
    os.makedirs(ctx.logs_dir, exist_ok=True)

    model = hand_model()

    # Phase 1: URDF contract up front + empty scaffolds.
    build_urdf(model, ctx)
    ok, err = validate_urdf(ctx.urdf_path, require_meshes=False)
    print(f"[1] URDF topology valid (pre-fill): {ok}" + (f" :: {err}" if not ok else ""))
    if not ok:
        return 1
    scaffold_meshes(model, ctx)

    # Phase 2 (SCAD worker, but hand-written here): write the .scad, render each
    # module to its link's STL.
    scad_path = run_dir / "model.scad"
    scad_path.write_text(TABLE_SCAD, encoding="utf-8")
    all_ok = True
    for link in model.links:
        stl = run_dir / link.mesh_filename
        r = render_module(oscad, scad_path, link.name, stl)
        rep = check_stl(str(stl))
        status = "OK" if (r and rep.exists and rep.loadable and not rep.degenerate) else "FAIL"
        if status == "FAIL":
            all_ok = False
        print(f"[2] {link.name}: render={r} faces={rep.num_faces} "
              f"bbox={tuple(round(b,1) for b in rep.bbox_mm)} -> {status}"
              + (f" :: {rep.error}" if rep.error else ""))

    # Phase 3: re-validate with meshes loaded.
    ok2, err2 = validate_urdf(ctx.urdf_path, require_meshes=True)
    print(f"[3] URDF valid (with meshes): {ok2}" + (f" :: {err2}" if not ok2 else ""))

    print("-" * 50)
    if all_ok and ok2:
        print(f"RESULT: PASS — contract works end-to-end. Bundle: {run_dir}")
        return 0
    print("RESULT: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
