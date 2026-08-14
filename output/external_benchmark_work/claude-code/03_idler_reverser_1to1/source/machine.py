"""Parametric open-frame 1:1 idler reversing train.

CAD units are millimetres.  World convention: X spans the three gear
centres, Y is the common shaft axis, and Z is vertical.  All independently
moving shaft/gear groups remain separate solids and semantic links.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from math import cos, pi, sin

from build123d import Align, Box, Compound, Cylinder, Location, Rotation, export_step, export_stl

TASK_ID = "03_idler_reverser_1to1"
MODULE = 2.0
TOOTH_COUNT = 30
PITCH_RADIUS = MODULE * TOOTH_COUNT / 2.0
ROOT_RADIUS = 27.5
TIP_RADIUS = 32.0
GEAR_THICKNESS = 10.0
SHAFT_RADIUS = 4.0
SHAFT_LENGTH = 72.0
AXIS_Z = 70.0
CENTERS_X = {"input": -60.0, "idler": 0.0, "output": 60.0}

MECHANISM = {
    "name": "open_idler_reverser_1to1",
    "units": {"cad_length": "mm", "assembly_length": "m", "angle": "rad"},
    "root_link": "base",
    "links": [
        {"name": "base", "dof": "fixed", "mesh": "base.stl", "rigid_mount": "world"},
        {"name": "input_support", "dof": "fixed", "mesh": "input_support.stl", "rigid_mount": "base"},
        {"name": "idler_support", "dof": "fixed", "mesh": "idler_support.stl", "rigid_mount": "base"},
        {"name": "output_support", "dof": "fixed", "mesh": "output_support.stl", "rigid_mount": "base"},
        {"name": "input_shaft", "dof": "revolute", "axis": [0, 1, 0], "mesh": "input_shaft.stl", "driver": True},
        {"name": "input_gear", "dof": "rigid_carried", "mesh": "input_gear.stl", "rigid_mount": "input_shaft"},
        {"name": "hand_crank", "dof": "rigid_carried", "mesh": "hand_crank.stl", "rigid_mount": "input_shaft"},
        {"name": "idler_shaft", "dof": "revolute", "axis": [0, 1, 0], "mesh": "idler_shaft.stl", "driver": False},
        {"name": "idler_gear", "dof": "rigid_carried", "mesh": "idler_gear.stl", "rigid_mount": "idler_shaft"},
        {"name": "output_shaft", "dof": "revolute", "axis": [0, 1, 0], "mesh": "output_shaft.stl", "driver": False},
        {"name": "output_gear", "dof": "rigid_carried", "mesh": "output_gear.stl", "rigid_mount": "output_shaft"},
    ],
    "ports": [
        {"name": "input_axis", "link": "input_shaft", "kind": "revolute", "origin_mm": [-60, 0, 70], "axis": [0, 1, 0]},
        {"name": "idler_axis", "link": "idler_shaft", "kind": "revolute", "origin_mm": [0, 0, 70], "axis": [0, 1, 0]},
        {"name": "output_axis", "link": "output_shaft", "kind": "revolute", "origin_mm": [60, 0, 70], "axis": [0, 1, 0]},
    ],
    "relations": [
        {"name": "base_world_mount", "kind": "rigid_mount", "a": "world", "b": "base"},
        {"name": "input_front_bearing", "kind": "running_bearing", "a": "input_support", "b": "input_shaft", "clearance_mm": 0.30},
        {"name": "input_rear_bearing", "kind": "running_bearing", "a": "input_support", "b": "input_shaft", "clearance_mm": 0.30},
        {"name": "idler_front_bearing", "kind": "running_bearing", "a": "idler_support", "b": "idler_shaft", "clearance_mm": 0.30},
        {"name": "idler_rear_bearing", "kind": "running_bearing", "a": "idler_support", "b": "idler_shaft", "clearance_mm": 0.30},
        {"name": "output_front_bearing", "kind": "running_bearing", "a": "output_support", "b": "output_shaft", "clearance_mm": 0.30},
        {"name": "output_rear_bearing", "kind": "running_bearing", "a": "output_support", "b": "output_shaft", "clearance_mm": 0.30},
        {"name": "input_gear_press_fit", "kind": "press_fit", "a": "input_shaft", "b": "input_gear"},
        {"name": "idler_gear_press_fit", "kind": "press_fit", "a": "idler_shaft", "b": "idler_gear"},
        {"name": "output_gear_press_fit", "kind": "press_fit", "a": "output_shaft", "b": "output_gear"},
        {"name": "crank_press_fit", "kind": "press_fit", "a": "input_shaft", "b": "hand_crank"},
        {"name": "input_idler_mesh", "kind": "ideal_external_gear_mesh", "a": "input_gear", "b": "idler_gear"},
        {"name": "idler_output_mesh", "kind": "ideal_external_gear_mesh", "a": "idler_gear", "b": "output_gear"},
    ],
    "motion_joints": [
        {"name": "input_shaft_hinge", "parent": "base", "child": "input_shaft", "kind": "revolute", "axis": [0, 1, 0], "origin_mm": [-60, 0, 70]},
        {"name": "idler_shaft_hinge", "parent": "base", "child": "idler_shaft", "kind": "revolute", "axis": [0, 1, 0], "origin_mm": [0, 0, 70]},
        {"name": "output_shaft_hinge", "parent": "base", "child": "output_shaft", "kind": "revolute", "axis": [0, 1, 0], "origin_mm": [60, 0, 70]},
    ],
    "transmissions": [
        {"name": "input_to_idler", "kind": "external_spur", "driving": "input_shaft_hinge", "driven": "idler_shaft_hinge", "driving_teeth": 30, "driven_teeth": 30, "driven_per_driving": -1.0},
        {"name": "idler_to_output", "kind": "external_spur", "driving": "idler_shaft_hinge", "driven": "output_shaft_hinge", "driving_teeth": 30, "driven_teeth": 30, "driven_per_driving": -1.0},
    ],
    "mesh_pairs": [
        {"name": "input_idler_mesh", "a": "input_gear", "b": "idler_gear", "ratio": -1.0},
        {"name": "idler_output_mesh", "a": "idler_gear", "b": "output_gear", "ratio": -1.0},
    ],
    "driver": {"joint": "input_shaft_hinge", "actuator": "hand_crank_motor", "direct_output_actuation": False},
    "output": {"link": "output_shaft", "joint": "output_shaft_hinge"},
    "watch_links": ["input_shaft", "input_gear", "idler_shaft", "idler_gear", "output_shaft", "output_gear", "hand_crank"],
}


def _at(shape, xyz=(0, 0, 0), rot=(0, 0, 0)):
    return shape.moved(Location(xyz, rot))


def _y_cylinder(radius, length, center):
    primitive = Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return primitive.moved(Location(center, (90, 0, 0)))


def _gear(center_x, phase_deg=0.0):
    # Deliberately open, visibly toothed demonstration gear. The bore gives the
    # shaft a modeled press-fit interface instead of a concealed fused overlap.
    disk = Cylinder(ROOT_RADIUS, GEAR_THICKNESS, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    bore = Cylinder(SHAFT_RADIUS, GEAR_THICKNESS + 2.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    solids = [disk - bore]
    tooth_radial = TIP_RADIUS - ROOT_RADIUS
    tooth_tangent = 4.0
    tooth_center_radius = ROOT_RADIUS + tooth_radial / 2.0 - 0.25
    for i in range(TOOTH_COUNT):
        a = phase_deg + 360.0 * i / TOOTH_COUNT
        ar = a * pi / 180.0
        # Explicit polar placement avoids transform-order ambiguity and keeps
        # all 30 teeth distributed around the full circumference.
        tooth = Box(tooth_radial, tooth_tangent, GEAR_THICKNESS,
                    align=(Align.CENTER, Align.CENTER, Align.CENTER))
        tooth = tooth.moved(Location((tooth_center_radius * cos(ar),
                                      tooth_center_radius * sin(ar), 0),
                                     (0, 0, a)))
        solids.append(tooth)
    local = Compound(children=solids)
    return local.moved(Location((center_x, 0, AXIS_Z), (90, 0, 0)))


def _support(center_x):
    # Two narrow pedestals and annular bearing cartridges leave each mesh open.
    parts = []
    for y in (-29.0, 29.0):
        post = Box(12.0, 10.0, 63.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        post = post.moved(Location((center_x, y, 6.0)))
        outer = _y_cylinder(10.0, 8.0, (center_x, y, AXIS_Z))
        inner = _y_cylinder(SHAFT_RADIUS + 0.30, 10.0, (center_x, y, AXIS_Z))
        parts.extend([post, outer - inner])
    return Compound(children=parts)


def _hand_crank():
    # Front-mounted offset arm and rotating grip, attached only to input shaft.
    arm = Box(34.0, 6.0, 6.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    arm = arm.moved(Location((-77.0, -39.0, AXIS_Z)))
    hub = _y_cylinder(7.0, 8.0, (-60.0, -39.0, AXIS_Z))
    hub_bore = _y_cylinder(SHAFT_RADIUS, 10.0, (-60.0, -39.0, AXIS_Z))
    grip = _y_cylinder(5.0, 24.0, (-94.0, -51.0, AXIS_Z))
    grip_cap = _y_cylinder(6.0, 3.0, (-94.0, -64.5, AXIS_Z))
    return Compound(children=[arm, hub - hub_bore, grip, grip_cap])


def build_parts():
    base_plate = Box(200.0, 86.0, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    feet = [
        _y_cylinder(5.5, 4.0, (x, y, 3.0))
        for x in (-90.0, 90.0) for y in (-35.0, 35.0)
    ]
    parts = {
        "base": Compound(children=[base_plate, *feet]),
        "input_support": _support(CENTERS_X["input"]),
        "idler_support": _support(CENTERS_X["idler"]),
        "output_support": _support(CENTERS_X["output"]),
        "input_shaft": _y_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, (CENTERS_X["input"], 0, AXIS_Z)),
        "idler_shaft": _y_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, (CENTERS_X["idler"], 0, AXIS_Z)),
        "output_shaft": _y_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, (CENTERS_X["output"], 0, AXIS_Z)),
        "input_gear": _gear(CENTERS_X["input"], 0.0),
        "idler_gear": _gear(CENTERS_X["idler"], 6.0),
        "output_gear": _gear(CENTERS_X["output"], 0.0),
        "hand_crank": _hand_crank(),
    }
    for name, shape in parts.items():
        shape.label = name
    return parts


def build_machine():
    parts = build_parts()
    machine = Compound(children=list(parts.values()))
    machine.label = MECHANISM["name"]
    return machine


def gen_step():
    return build_machine()


def export_artifacts(root: Path):
    root = root.resolve()
    mesh_dir = root / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    parts = build_parts()
    for name, shape in parts.items():
        export_stl(shape, mesh_dir / f"{name}.stl", tolerance=0.08, angular_tolerance=0.08)
    export_step(build_machine(), root / "model.step")
    inventory = {
        "task_id": TASK_ID,
        "part_count": len(parts),
        "parts": sorted(parts),
        "step": "model.step",
        "meshes": [f"meshes/{n}.stl" for n in sorted(parts)],
    }
    (root / "raw" / "source_inventory.json").write_text(json.dumps(inventory, indent=2), encoding="utf-8")
    print(json.dumps(inventory, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--export-root", type=Path)
    args = parser.parse_args()
    if args.export_root:
        export_artifacts(args.export_root)
    else:
        result = build_machine()
        print(f"{MECHANISM['name']}: {len(build_parts())} named parts; valid={result.is_valid()}")
