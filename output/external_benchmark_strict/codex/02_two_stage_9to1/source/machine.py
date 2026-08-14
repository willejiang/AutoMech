"""Open-frame two-stage 9:1 spur reducer; CAD units are millimetres."""
from __future__ import annotations

import math
from pathlib import Path
from build123d import Align, Axis, Box, Compound, Cylinder, Location, Plane, Polygon, export_stl, extrude

AXIS_X = {"input_shaft": -72.0, "compound_intermediate_shaft": 0.0, "output_shaft": 72.0}
PART_POSES_MM = {
    "base": (0.0, 0.0, 4.0),
    **{f"{stem}_{side}_bearing": (x, y, 90.0)
       for stem, x in (("input", -72.0), ("intermediate", 0.0), ("output", 72.0))
       for side, y in (("front", -36.0), ("rear", 36.0))},
    "input_shaft": (-72.0, 0.0, 90.0),
    "compound_intermediate_shaft": (0.0, 0.0, 90.0),
    "output_shaft": (72.0, 0.0, 90.0),
    "stage1_input_pinion": (-72.0, 0.0, 90.0),
    "stage1_driven_gear": (0.0, 0.0, 90.0),
    "stage2_intermediate_pinion": (0.0, 0.0, 90.0),
    "stage2_output_gear": (72.0, 0.0, 90.0),
    "hand_crank": (-72.0, 0.0, 90.0),
}

MECHANISM = {
    "name": "open_frame_two_stage_9to1",
    "links": [
        {"name": name, "dof": "fixed" if name == "base" or "bearing" in name else "spin_or_rigid"}
        for name in PART_POSES_MM
    ],
    "ports": {
        "input_axis": {"link": "input_shaft", "axis": [0, 1, 0]},
        "intermediate_axis": {"link": "compound_intermediate_shaft", "axis": [0, 1, 0]},
        "output_axis": {"link": "output_shaft", "axis": [0, 1, 0]},
    },
    "relations": [
        {"type": "press_fit", "outer": "stage1_input_pinion", "inner": "input_shaft"},
        {"type": "press_fit", "outer": "stage1_driven_gear", "inner": "compound_intermediate_shaft"},
        {"type": "press_fit", "outer": "stage2_intermediate_pinion", "inner": "compound_intermediate_shaft"},
        {"type": "press_fit", "outer": "stage2_output_gear", "inner": "output_shaft"},
        {"type": "press_fit", "outer": "hand_crank", "inner": "input_shaft"},
        {"type": "ideal_external_gear_mesh", "driving": "stage1_input_pinion", "driven": "stage1_driven_gear", "driving_teeth": 12, "driven_teeth": 36},
        {"type": "ideal_external_gear_mesh", "driving": "stage2_intermediate_pinion", "driven": "stage2_output_gear", "driving_teeth": 12, "driven_teeth": 36},
    ],
    "motion_joints": [
        {"name": "input_shaft_hinge", "child": "input_shaft", "axis": [0, 1, 0], "driver": True},
        {"name": "compound_intermediate_shaft_hinge", "child": "compound_intermediate_shaft", "axis": [0, 1, 0], "driver": False},
        {"name": "output_shaft_hinge", "child": "output_shaft", "axis": [0, 1, 0], "driver": False},
    ],
    "transmissions": [
        {"name": "stage1_3to1", "driving_joint": "input_shaft_hinge", "driven_joint": "compound_intermediate_shaft_hinge", "ratio": -1.0 / 3.0, "convention": "driven_over_driving"},
        {"name": "stage2_3to1", "driving_joint": "compound_intermediate_shaft_hinge", "driven_joint": "output_shaft_hinge", "ratio": -1.0 / 3.0, "convention": "driven_over_driving"},
    ],
    "driver": {"joint": "input_shaft_hinge", "source": "hand_crank"},
    "output": {"joint": "output_shaft_hinge", "link": "output_shaft"},
    "watch_links": ["stage1_input_pinion", "stage1_driven_gear", "stage2_intermediate_pinion", "stage2_output_gear", "output_shaft"],
}

def cylinder_y(radius, length):
    return Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.X, 90)

def gear(teeth, module, thickness, y, phase=0.0):
    pitch = module * teeth / 2.0
    root, outer = pitch - 1.25 * module, pitch + module
    body = cylinder_y(root, thickness)
    tip_w = 0.36 * math.pi * module
    root_w = tip_w * root / outer
    tooth_shapes = []
    for i in range(teeth):
        profile = Polygon((root - 0.7, -root_w/2), (outer, -tip_w/2), (outer, tip_w/2), (root - 0.7, root_w/2), align=None)
        tooth = extrude(Plane.XZ * profile, amount=thickness/2, both=True).rotate(Axis.Y, phase + 360*i/teeth)
        tooth_shapes.append(tooth)
    return body.fuse(*tooth_shapes).translate((0, y, 0))

def bearing():
    ring = cylinder_y(14, 10).cut(cylinder_y(6, 12))
    column = Box(20, 10, 69, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((0, 0, -47.5))
    return ring.fuse(column)

def crank():
    hub = cylinder_y(10, 8).translate((0, -54, 0))
    arm = Box(9, 6, 38, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((0, -58, 19))
    handle = cylinder_y(6, 22).translate((0, -69, 38))
    return hub.fuse(arm, handle)

def build_local_parts():
    parts = {
        "base": Box(250, 110, 8, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "input_front_bearing": bearing(), "input_rear_bearing": bearing(),
        "intermediate_front_bearing": bearing(), "intermediate_rear_bearing": bearing(),
        "output_front_bearing": bearing(), "output_rear_bearing": bearing(),
        "input_shaft": cylinder_y(5, 112),
        "compound_intermediate_shaft": cylinder_y(5, 92),
        "output_shaft": cylinder_y(5, 92),
        "stage1_input_pinion": gear(12, 3, 10, -11, 0),
        "stage1_driven_gear": gear(36, 3, 10, -11, 5),
        "stage2_intermediate_pinion": gear(12, 3, 10, 11, 0),
        "stage2_output_gear": gear(36, 3, 10, 11, 5),
        "hand_crank": crank(),
    }
    for name, shape in parts.items(): shape.label = name
    return parts

def build_machine():
    children=[]
    for name, shape in build_local_parts().items():
        placed=shape.moved(Location(PART_POSES_MM[name])); placed.label=name; children.append(placed)
    result=Compound(children=children); result.label="open_frame_two_stage_9to1"; return result

def gen_step(): return build_machine()

def export_named_stls(directory: str | Path):
    directory=Path(directory); directory.mkdir(parents=True, exist_ok=True); written=[]
    for name, shape in build_local_parts().items():
        path=directory/f"{name}.stl"; export_stl(shape,path,tolerance=0.12,angular_tolerance=0.12); written.append(path)
    return written

if __name__ == "__main__":
    import argparse
    p=argparse.ArgumentParser(); p.add_argument("--export-stls",type=Path); a=p.parse_args()
    if not build_machine().solids(): raise RuntimeError("empty assembly")
    if a.export_stls:
        for path in export_named_stls(a.export_stls): print(path)
