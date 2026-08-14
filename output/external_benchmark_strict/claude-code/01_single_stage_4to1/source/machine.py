"""Open-frame single-stage 4:1 spur reducer benchmark model.

Units are millimetres.  World convention: X spans the two shafts, Y is the
parallel shaft axis (camera looks generally from -Y), and Z is up.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from build123d import (
    Align, Axis, Box, BuildSketch, Circle, Compound, Cylinder, Location,
    Mode, Polygon, Torus, export_stl, extrude,
)

MODULE = 1.5
PINION_TEETH = 16
GEAR_TEETH = 64
PINION_PITCH_R = MODULE * PINION_TEETH / 2
GEAR_PITCH_R = MODULE * GEAR_TEETH / 2
CENTER_DISTANCE = PINION_PITCH_R + GEAR_PITCH_R
FACE_WIDTH = 12.0
SHAFT_RADIUS = 4.0
INPUT_X = -CENTER_DISTANCE / 2
OUTPUT_X = CENTER_DISTANCE / 2
SHAFT_Z = 72.0

MECHANISM = {
    "name": "open_single_stage_4to1",
    "units": "mm",
    "coordinate_system": {"shaft_axis": [0, 1, 0], "up": [0, 0, 1]},
    "links": [
        {"name": "base", "dof": "fixed"},
        {"name": "input_shaft", "dof": "revolute", "axis": [0, 1, 0], "driver": True},
        {"name": "output_shaft", "dof": "revolute", "axis": [0, 1, 0], "driver": False},
    ],
    "ports": {
        "input_axis": {"link": "base", "origin": [INPUT_X, 0, SHAFT_Z], "axis": [0, 1, 0]},
        "output_axis": {"link": "base", "origin": [OUTPUT_X, 0, SHAFT_Z], "axis": [0, 1, 0]},
    },
    "relations": [
        {"kind": "running_bearing", "a": "input_shaft", "b": "input_bearing_front", "clearance_mm": 0.35},
        {"kind": "running_bearing", "a": "input_shaft", "b": "input_bearing_rear", "clearance_mm": 0.35},
        {"kind": "running_bearing", "a": "output_shaft", "b": "output_bearing_front", "clearance_mm": 0.35},
        {"kind": "running_bearing", "a": "output_shaft", "b": "output_bearing_rear", "clearance_mm": 0.35},
        {"kind": "press_fit", "a": "input_pinion", "b": "input_shaft"},
        {"kind": "press_fit", "a": "output_gear", "b": "output_shaft"},
        {"kind": "rigid_carry", "a": "hand_crank_arm", "b": "input_shaft"},
        {"kind": "rigid_carry", "a": "hand_crank_handle", "b": "hand_crank_arm"},
        {"kind": "rigid_mount", "a": "bearing_supports", "b": "base"},
        {"kind": "ideal_gear_mesh", "a": "input_pinion", "b": "output_gear", "mesh": "stage_1_mesh"},
    ],
    "motion_joints": [
        {"name": "input_shaft_hinge", "kind": "revolute", "parent": "base", "child": "input_shaft", "axis": [0, 1, 0]},
        {"name": "output_shaft_hinge", "kind": "revolute", "parent": "base", "child": "output_shaft", "axis": [0, 1, 0]},
    ],
    "transmissions": [
        {"name": "stage_1", "kind": "external_spur", "driving": "input_shaft_hinge", "driven": "output_shaft_hinge", "ratio_driven_over_driving": -0.25, "driving_teeth": 16, "driven_teeth": 64},
    ],
    "mesh_pairs": [
        {"name": "stage_1_mesh", "a": "input_pinion", "b": "output_gear", "type": "external", "center_distance_mm": CENTER_DISTANCE, "module_mm": MODULE},
    ],
    "driver": {"joint": "input_shaft_hinge", "kind": "hand_crank"},
    "output": {"joint": "output_shaft_hinge", "link": "output_shaft"},
    "watch_links": ["input_shaft", "input_pinion", "output_shaft", "output_gear", "hand_crank_arm", "hand_crank_handle"],
}


def _label(shape, name):
    shape.label = name
    return shape


def _axis_y_cylinder(radius: float, length: float, x: float, y: float, z: float):
    solid = Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    solid = solid.rotate(Axis.X, 90).move(Location((x, y, z)))
    return solid


def _spur_gear(teeth: int, module: float, face_width: float, bore_radius: float, x: float, z: float, phase_deg: float):
    pitch_r = module * teeth / 2
    root_r = pitch_r - 1.25 * module
    outer_r = pitch_r + module
    half_root = math.pi / teeth * 0.72
    half_tip = math.pi / teeth * 0.34
    with BuildSketch() as profile:
        Circle(root_r)
        for i in range(teeth):
            a = math.radians(phase_deg) + 2 * math.pi * i / teeth
            pts = [
                (root_r * math.cos(a - half_root), root_r * math.sin(a - half_root)),
                (outer_r * math.cos(a - half_tip), outer_r * math.sin(a - half_tip)),
                (outer_r * math.cos(a + half_tip), outer_r * math.sin(a + half_tip)),
                (root_r * math.cos(a + half_root), root_r * math.sin(a + half_root)),
            ]
            Polygon(*pts, mode=Mode.ADD)
        Circle(bore_radius, mode=Mode.SUBTRACT)
    gear = extrude(profile.sketch, amount=face_width)
    gear = gear.move(Location((0, 0, -face_width / 2))).rotate(Axis.X, 90).move(Location((x, 0, z)))
    return gear


def _bearing(x: float, y: float):
    # Open rolling-bearing envelope: 4.35 mm bore gives 0.35 mm radial clearance.
    ring = Torus(7.1, 2.75).rotate(Axis.X, 90).move(Location((x, y, SHAFT_Z)))
    return ring


def _support(x: float, y: float):
    # Narrow pedestal stops below the visible bearing ring.
    return Box(18, 8, 60, align=(Align.CENTER, Align.CENTER, Align.MIN)).move(Location((x, y, 6)))


def make_parts():
    parts = {}
    parts["base"] = _label(Box(150, 92, 6, align=(Align.CENTER, Align.CENTER, Align.MIN)), "base")
    parts["input_support_front"] = _label(_support(INPUT_X, -24), "input_support_front")
    parts["input_support_rear"] = _label(_support(INPUT_X, 24), "input_support_rear")
    parts["output_support_front"] = _label(_support(OUTPUT_X, -24), "output_support_front")
    parts["output_support_rear"] = _label(_support(OUTPUT_X, 24), "output_support_rear")
    parts["input_bearing_front"] = _label(_bearing(INPUT_X, -24), "input_bearing_front")
    parts["input_bearing_rear"] = _label(_bearing(INPUT_X, 24), "input_bearing_rear")
    parts["output_bearing_front"] = _label(_bearing(OUTPUT_X, -24), "output_bearing_front")
    parts["output_bearing_rear"] = _label(_bearing(OUTPUT_X, 24), "output_bearing_rear")
    parts["input_shaft"] = _label(_axis_y_cylinder(SHAFT_RADIUS, 82, INPUT_X, 0, SHAFT_Z), "input_shaft")
    parts["output_shaft"] = _label(_axis_y_cylinder(SHAFT_RADIUS, 72, OUTPUT_X, 0, SHAFT_Z), "output_shaft")
    parts["input_pinion"] = _label(_spur_gear(PINION_TEETH, MODULE, FACE_WIDTH, 3.85, INPUT_X, SHAFT_Z, 0), "input_pinion")
    # Half-tooth phase visually centers a gap opposite a pinion tooth at the pitch point.
    parts["output_gear"] = _label(_spur_gear(GEAR_TEETH, MODULE, FACE_WIDTH, 3.85, OUTPUT_X, SHAFT_Z, 180 / GEAR_TEETH), "output_gear")
    parts["hand_crank_arm"] = _label(Box(34, 6, 7, align=(Align.MAX, Align.CENTER, Align.CENTER)).move(Location((INPUT_X, -44, SHAFT_Z))), "hand_crank_arm")
    parts["hand_crank_handle"] = _label(_axis_y_cylinder(4.5, 24, INPUT_X - 34, -55, SHAFT_Z), "hand_crank_handle")
    return parts


def build_machine():
    """Return a deterministic labeled compound with independently named parts."""
    parts = make_parts()
    assembly = Compound(children=list(parts.values()))
    assembly.label = "open_single_stage_4to1"
    return assembly


def gen_step():
    return build_machine()


def export_named_meshes(output_dir: Path | None = None):
    output_dir = output_dir or Path(__file__).resolve().parents[1] / "meshes"
    output_dir.mkdir(parents=True, exist_ok=True)
    parts = make_parts()
    for name, part in parts.items():
        export_stl(part, output_dir / f"{name}.stl", tolerance=0.05, angular_tolerance=0.1)
    manifest = {name: f"meshes/{name}.stl" for name in parts}
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


if __name__ == "__main__":
    manifest = export_named_meshes()
    machine = build_machine()
    print(json.dumps({"assembly": machine.label, "parts": sorted(manifest), "part_count": len(manifest)}, indent=2))
