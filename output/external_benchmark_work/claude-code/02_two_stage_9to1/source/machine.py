"""Parametric open-frame two-stage 9:1 spur reducer benchmark source.

CAD units are millimetres. Shaft axes are world +Y.  The model uses two
external 12:36 tooth meshes; each stage is -3:1, so the output follows the
input at +1/9 speed.  Every independently named manufactured/moving item is
kept as a labeled child shape.
"""
from __future__ import annotations

import math
from pathlib import Path

from build123d import Align, Box, Compound, Cylinder, Location, Torus, export_stl

# Explicit mechanism semantics consumed by the benchmark lowering.
MECHANISM = {
    "name": "open_two_stage_9to1_reducer",
    "units": {"cad_length": "mm", "assembly_pose_length": "m", "angle": "rad"},
    "links": [
        {"name": "base", "dof": "fixed", "rigid_mount": "world"},
        {"name": "input_shaft", "dof": "revolute", "axis": [0, 1, 0], "driver": True},
        {"name": "input_pinion", "dof": "fixed", "rigid_mount": "input_shaft"},
        {"name": "hand_crank", "dof": "fixed", "rigid_mount": "input_shaft"},
        {"name": "intermediate_shaft", "dof": "revolute", "axis": [0, 1, 0]},
        {"name": "stage1_driven_gear", "dof": "fixed", "rigid_mount": "intermediate_shaft"},
        {"name": "stage2_pinion", "dof": "fixed", "rigid_mount": "intermediate_shaft"},
        {"name": "output_shaft", "dof": "revolute", "axis": [0, 1, 0], "output": True},
        {"name": "output_gear", "dof": "fixed", "rigid_mount": "output_shaft"},
    ],
    "ports": {
        "input_shaft": {"axis": "input_axis", "gear_mount": "input_pinion_press_fit", "crank_mount": "crank_press_fit"},
        "intermediate_shaft": {"axis": "intermediate_axis", "stage1_mount": "stage1_press_fit", "stage2_mount": "stage2_press_fit"},
        "output_shaft": {"axis": "output_axis", "gear_mount": "output_press_fit"},
    },
    "relations": [
        {"name": "input_bearing_front", "kind": "running_bearing", "parent": "base", "child": "input_shaft", "clearance_mm": 0.5},
        {"name": "input_bearing_rear", "kind": "running_bearing", "parent": "base", "child": "input_shaft", "clearance_mm": 0.5},
        {"name": "intermediate_bearing_front", "kind": "running_bearing", "parent": "base", "child": "intermediate_shaft", "clearance_mm": 0.5},
        {"name": "intermediate_bearing_rear", "kind": "running_bearing", "parent": "base", "child": "intermediate_shaft", "clearance_mm": 0.5},
        {"name": "output_bearing_front", "kind": "running_bearing", "parent": "base", "child": "output_shaft", "clearance_mm": 0.5},
        {"name": "output_bearing_rear", "kind": "running_bearing", "parent": "base", "child": "output_shaft", "clearance_mm": 0.5},
        {"name": "input_pinion_press_fit", "kind": "press_fit", "parent": "input_shaft", "child": "input_pinion"},
        {"name": "crank_press_fit", "kind": "press_fit", "parent": "input_shaft", "child": "hand_crank"},
        {"name": "stage1_press_fit", "kind": "press_fit", "parent": "intermediate_shaft", "child": "stage1_driven_gear"},
        {"name": "stage2_press_fit", "kind": "press_fit", "parent": "intermediate_shaft", "child": "stage2_pinion"},
        {"name": "compound_rigid_carrying", "kind": "rigid_carrying", "parent": "intermediate_shaft", "children": ["stage1_driven_gear", "stage2_pinion"]},
        {"name": "output_press_fit", "kind": "press_fit", "parent": "output_shaft", "child": "output_gear"},
        {"name": "stage1_mesh", "kind": "ideal_external_gear_mesh", "a": "input_pinion", "b": "stage1_driven_gear"},
        {"name": "stage2_mesh", "kind": "ideal_external_gear_mesh", "a": "stage2_pinion", "b": "output_gear"},
    ],
    "motion_joints": [
        {"name": "input_shaft_hinge", "kind": "revolute", "parent": "base", "child": "input_shaft", "axis": [0, 1, 0]},
        {"name": "intermediate_shaft_hinge", "kind": "revolute", "parent": "base", "child": "intermediate_shaft", "axis": [0, 1, 0]},
        {"name": "output_shaft_hinge", "kind": "revolute", "parent": "base", "child": "output_shaft", "axis": [0, 1, 0]},
    ],
    "transmissions": [
        {"name": "stage1_3to1", "kind": "external_spur", "driving": "input_shaft_hinge", "driven": "intermediate_shaft_hinge", "driving_teeth": 12, "driven_teeth": 36, "ratio_driven_per_driving": -1.0 / 3.0, "mesh": "stage1_mesh"},
        {"name": "stage2_3to1", "kind": "external_spur", "driving": "intermediate_shaft_hinge", "driven": "output_shaft_hinge", "driving_teeth": 12, "driven_teeth": 36, "ratio_driven_per_driving": -1.0 / 3.0, "mesh": "stage2_mesh"},
    ],
    "driver": {"joint": "input_shaft_hinge", "kind": "velocity", "direct_output_actuation": False},
    "output": {"joint": "output_shaft_hinge", "link": "output_shaft"},
    "watch_links": ["input_shaft", "input_pinion", "intermediate_shaft", "stage1_driven_gear", "stage2_pinion", "output_shaft", "output_gear", "hand_crank"],
}

SHAFT_X = (-72.0, 0.0, 72.0)
SHAFT_Z = 78.0
SHAFT_LENGTH = 94.0
SHAFT_RADIUS = 5.0
PINION_TEETH = 12
GEAR_TEETH = 36
MODULE = 3.0
GEAR_THICKNESS = 8.0
STAGE_Y = (-13.0, 13.0)


def _placed(shape, xyz=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0), label=""):
    result = shape.moved(Location(xyz, rot))
    result.label = label
    return result


def _shaft(x: float, label: str):
    # Cylinder defaults to +Z; rotate 90 deg about X for a +Y/-Y axis.
    return _placed(Cylinder(SHAFT_RADIUS, SHAFT_LENGTH, align=(Align.CENTER, Align.CENTER, Align.CENTER)), (x, 0, SHAFT_Z), (90, 0, 0), label)


def _gear(teeth: int, x: float, y: float, z: float, label: str):
    pitch_radius = MODULE * teeth / 2.0
    root_radius = pitch_radius - 1.25 * MODULE
    outer_radius = pitch_radius + MODULE
    tooth_depth = outer_radius - root_radius
    tooth_width = max(2.2, math.pi * MODULE * 0.46)
    children = []
    hub = Cylinder(root_radius, GEAR_THICKNESS, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    hub = hub.moved(Location((0, 0, 0), (90, 0, 0)))
    children.append(hub)
    for i in range(teeth):
        a = 360.0 * i / teeth
        tooth = Box(tooth_depth, GEAR_THICKNESS, tooth_width, align=(Align.MIN, Align.CENTER, Align.CENTER))
        tooth = tooth.moved(Location((root_radius, 0, 0)))
        tooth = tooth.moved(Location((0, 0, 0), (0, a, 0)))
        children.append(tooth)
    gear = Compound(children=children)
    return _placed(gear, (x, y, z), label=label)


def _bearing_ring(x: float, y: float, label: str):
    ring = Torus(7.2, 2.2)
    return _placed(ring, (x, y, SHAFT_Z), (90, 0, 0), label)


def _pedestal(x: float, y: float, label: str):
    block = Box(18.0, 8.0, 55.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return _placed(block, (x, y, 8.0), label=label)


def build_parts() -> dict[str, object]:
    parts: dict[str, object] = {}
    parts["base"] = _placed(Box(190.0, 110.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN)), (0, 0, 0), label="base")
    for shaft_name, x in zip(("input", "intermediate", "output"), SHAFT_X):
        for side, y in (("front", -38.0), ("rear", 38.0)):
            parts[f"{shaft_name}_pedestal_{side}"] = _pedestal(x, y, f"{shaft_name}_pedestal_{side}")
            parts[f"{shaft_name}_bearing_{side}"] = _bearing_ring(x, y, f"{shaft_name}_bearing_{side}")
    parts["input_shaft"] = _shaft(SHAFT_X[0], "input_shaft")
    parts["intermediate_shaft"] = _shaft(SHAFT_X[1], "intermediate_shaft")
    parts["output_shaft"] = _shaft(SHAFT_X[2], "output_shaft")
    parts["input_pinion"] = _gear(PINION_TEETH, SHAFT_X[0], STAGE_Y[0], SHAFT_Z, "input_pinion")
    parts["stage1_driven_gear"] = _gear(GEAR_TEETH, SHAFT_X[1], STAGE_Y[0], SHAFT_Z, "stage1_driven_gear")
    parts["stage2_pinion"] = _gear(PINION_TEETH, SHAFT_X[1], STAGE_Y[1], SHAFT_Z, "stage2_pinion")
    parts["output_gear"] = _gear(GEAR_TEETH, SHAFT_X[2], STAGE_Y[1], SHAFT_Z, "output_gear")
    crank_arm = Box(9.0, 8.0, 34.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    crank_arm = _placed(crank_arm, (SHAFT_X[0], -51.0, SHAFT_Z), label="hand_crank")
    grip = Cylinder(5.0, 22.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    grip = _placed(grip, (SHAFT_X[0], -62.0, SHAFT_Z + 34.0), (90, 0, 0), "hand_crank_grip")
    parts["hand_crank"] = Compound(children=[crank_arm, grip])
    parts["hand_crank"].label = "hand_crank"
    return parts


def build_machine():
    parts = build_parts()
    machine = Compound(children=list(parts.values()))
    machine.label = MECHANISM["name"]
    return machine


def gen_step():
    return build_machine()


def export_named_meshes(mesh_dir: Path | None = None) -> list[str]:
    mesh_dir = Path(mesh_dir) if mesh_dir else Path(__file__).resolve().parents[1] / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    outputs = []
    for name, part in build_parts().items():
        path = mesh_dir / f"{name}.stl"
        export_stl(part, path, tolerance=0.15, angular_tolerance=0.15)
        outputs.append(str(path))
    return outputs


if __name__ == "__main__":
    built = build_machine()
    children = list(built.children)
    if not children:
        raise RuntimeError("build_machine produced no named parts")
    outputs = export_named_meshes()
    print(f"mechanism={MECHANISM['name']}")
    print(f"named_parts={len(children)}")
    print(f"meshes_exported={len(outputs)}")
