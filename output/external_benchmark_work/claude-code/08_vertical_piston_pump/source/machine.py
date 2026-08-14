"""Open-frame vertical reciprocating piston-pump benchmark mechanism.

CAD units are millimetres.  World convention: X is crankshaft depth/axis,
Y is horizontal across the bench, and Z is vertical.  ``build_machine``
returns a deterministic labeled build123d Compound; running this file exports
one stable, named STL per physical part plus a STEP assembly.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from build123d import Box, Compound, Cylinder, Location, Torus, export_step, export_stl

MECHANISM = {
    "name": "open_vertical_piston_pump",
    "units": {"cad_length": "mm", "poses_length": "m", "angles": "rad"},
    "root_link": "base",
    "links": [
        {"name": "base", "dof": "fixed", "rigid_mount": "world"},
        {"name": "left_bearing", "dof": "fixed", "rigid_mount": "base"},
        {"name": "right_bearing", "dof": "fixed", "rigid_mount": "base"},
        {"name": "guide_left", "dof": "fixed", "rigid_mount": "base"},
        {"name": "guide_right", "dof": "fixed", "rigid_mount": "base"},
        {"name": "cylinder_left", "dof": "fixed", "rigid_mount": "base"},
        {"name": "cylinder_right", "dof": "fixed", "rigid_mount": "base"},
        {"name": "crankshaft", "dof": "revolute", "axis": [1, 0, 0], "driver": True},
        {"name": "crank_disk", "dof": "rigid", "rigid_mount": "crankshaft"},
        {"name": "hand_crank", "dof": "rigid", "rigid_mount": "crankshaft"},
        {"name": "eccentric_pin", "dof": "rigid", "rigid_mount": "crank_disk"},
        {"name": "connecting_rod", "dof": "planar_closure"},
        {"name": "vertical_crosshead", "dof": "slide", "axis": [0, 0, 1]},
        {"name": "pump_rod", "dof": "rigid", "rigid_mount": "vertical_crosshead"},
        {"name": "piston", "dof": "rigid", "rigid_mount": "pump_rod"},
    ],
    "ports": [
        {"name": "crank_axis", "link": "base", "kind": "revolute", "origin_mm": [0, 0, 75], "axis": [1, 0, 0]},
        {"name": "eccentric_axis", "link": "crank_disk", "kind": "pin", "origin_mm": [0, 0, 18], "axis": [1, 0, 0]},
        {"name": "rod_crank_eye", "link": "connecting_rod", "kind": "pin", "axis": [1, 0, 0]},
        {"name": "rod_crosshead_eye", "link": "connecting_rod", "kind": "pin", "axis": [1, 0, 0]},
        {"name": "crosshead_pin", "link": "vertical_crosshead", "kind": "pin", "origin_mm": [0, 0, 120], "axis": [1, 0, 0]},
        {"name": "crosshead_slide_axis", "link": "vertical_crosshead", "kind": "slide", "axis": [0, 0, 1]},
        {"name": "pump_rod_mount", "link": "vertical_crosshead", "kind": "rigid", "origin_mm": [0, 0, 128]},
        {"name": "piston_mount", "link": "pump_rod", "kind": "rigid", "origin_mm": [0, 0, 50]},
    ],
    "relations": [
        {"name": "base_world_mount", "type": "rigid_mount", "parent": "world", "child": "base"},
        {"name": "left_bearing_mount", "type": "rigid_mount", "parent": "base", "child": "left_bearing"},
        {"name": "right_bearing_mount", "type": "rigid_mount", "parent": "base", "child": "right_bearing"},
        {"name": "guide_left_mount", "type": "rigid_mount", "parent": "base", "child": "guide_left"},
        {"name": "guide_right_mount", "type": "rigid_mount", "parent": "base", "child": "guide_right"},
        {"name": "cylinder_left_mount", "type": "rigid_mount", "parent": "base", "child": "cylinder_left"},
        {"name": "cylinder_right_mount", "type": "rigid_mount", "parent": "base", "child": "cylinder_right"},
        {"name": "crankshaft_running_bearing", "type": "running_bearing", "outer_links": ["left_bearing", "right_bearing"], "inner_link": "crankshaft", "clearance_mm": 0.5},
        {"name": "disk_shaft_press_fit", "type": "press_fit", "parent": "crankshaft", "child": "crank_disk"},
        {"name": "handle_shaft_press_fit", "type": "press_fit", "parent": "crankshaft", "child": "hand_crank"},
        {"name": "eccentric_pin_press_fit", "type": "press_fit", "parent": "crank_disk", "child": "eccentric_pin"},
        {"name": "crank_pin_fit", "type": "dedicated_pin_fit", "pin": "eccentric_pin", "rod": "connecting_rod"},
        {"name": "rod_crosshead_closure", "type": "pin_closure", "a": "connecting_rod", "a_port": "rod_crosshead_eye", "b": "vertical_crosshead", "b_port": "crosshead_pin"},
        {"name": "crosshead_guide", "type": "linear_running_fit", "slider": "vertical_crosshead", "guides": ["guide_left", "guide_right"], "axis": [0, 0, 1], "clearance_mm": 1.0},
        {"name": "pump_rod_rigid_carrying", "type": "rigid_carrying", "parent": "vertical_crosshead", "child": "pump_rod"},
        {"name": "piston_rigid_carrying", "type": "rigid_carrying", "parent": "pump_rod", "child": "piston"},
    ],
    "motion_joints": [
        {"name": "crankshaft_hinge", "kind": "revolute", "parent": "base", "child": "crankshaft", "axis": [1, 0, 0], "range": [-1000.0, 1000.0]},
        {"name": "crosshead_slide", "kind": "slide", "parent": "base", "child": "vertical_crosshead", "axis": [0, 0, 1], "range_mm": [101.0, 139.0]},
        {"name": "rod_crank_pin_hinge", "kind": "revolute", "parent": "eccentric_pin", "child": "connecting_rod", "axis": [1, 0, 0]},
        {"name": "rod_crosshead_pin_hinge", "kind": "revolute", "parent": "vertical_crosshead", "child": "connecting_rod", "axis": [1, 0, 0]},
    ],
    "transmissions": [],
    "driver": {"joint": "crankshaft_hinge", "link": "crankshaft", "mode": "position", "direct_output_drive": False},
    "output": {"joint": "crosshead_slide", "link": "piston", "kind": "vertical_translation"},
    "watch_links": ["crankshaft", "eccentric_pin", "connecting_rod", "vertical_crosshead", "pump_rod", "piston"],
}

PART_ORDER = [link["name"] for link in MECHANISM["links"]]


def _at(shape, xyz=(0.0, 0.0, 0.0), rxyz=(0.0, 0.0, 0.0)):
    return shape.moved(Location(xyz, rxyz))


def _rod_between_yz(y1: float, z1: float, y2: float, z2: float, x: float = 0.0):
    """Create a slender connecting rod in the YZ plane with axis along its length."""
    dy, dz = y2 - y1, z2 - z1
    length = math.hypot(dy, dz)
    angle_x = -math.degrees(math.atan2(dy, dz))
    bar = _at(Box(7.0, 9.0, length), (x - 3.5, (y1 + y2) / 2 - 4.5, (z1 + z2) / 2), (angle_x, 0, 0))
    eye1 = _at(Cylinder(9.0, 7.0), (x - 3.5, y1, z1), (0, 90, 0))
    eye2 = _at(Cylinder(9.0, 7.0), (x - 3.5, y2, z2), (0, 90, 0))
    return bar.fuse(eye1).fuse(eye2)


def make_parts():
    """Return stable named solids at a representative mid-stroke pose."""
    parts = {}
    parts["base"] = Box(130, 90, 8).moved(Location((-65, -45, 0)))
    parts["left_bearing"] = _at(Box(12, 18, 38), (-31, -9, 42)).cut(_at(Cylinder(6.5, 14), (-32, 0, 75), (0, 90, 0)))
    parts["right_bearing"] = _at(Box(12, 18, 38), (19, -9, 42)).cut(_at(Cylinder(6.5, 14), (18, 0, 75), (0, 90, 0)))
    parts["guide_left"] = _at(Box(8, 12, 110), (-28, -6, 92))
    parts["guide_right"] = _at(Box(8, 12, 110), (20, -6, 92))
    parts["cylinder_left"] = _at(Box(6, 10, 62), (-22, -5, 198))
    parts["cylinder_right"] = _at(Box(6, 10, 62), (16, -5, 198))
    parts["crankshaft"] = _at(Cylinder(6, 72), (-36, 0, 75), (0, 90, 0))
    parts["crank_disk"] = _at(Cylinder(32, 8), (-12, 0, 75), (0, 90, 0))
    # bent hand crank: radial arm and grip, both rigidly one labeled part
    crank_arm = _at(Box(8, 48, 8), (-48, -4, 71))
    grip = _at(Cylinder(5, 30), (-48, 44, 75), (0, 90, 0))
    parts["hand_crank"] = crank_arm.fuse(grip)
    parts["eccentric_pin"] = _at(Cylinder(5, 18), (-17, 0, 93), (0, 90, 0))
    parts["connecting_rod"] = _rod_between_yz(0, 93, 0, 120, x=-4)
    parts["vertical_crosshead"] = _at(Box(38, 18, 22), (-19, -9, 109))
    parts["pump_rod"] = _at(Cylinder(5, 78), (0, 0, 127))
    parts["piston"] = _at(Cylinder(16, 14), (0, 0, 210))
    for name, shape in parts.items():
        shape.label = name
    return parts


def build_machine():
    parts = make_parts()
    root = Compound(children=[parts[name] for name in PART_ORDER])
    root.label = MECHANISM["name"]
    return root


def export_artifacts(output_root: str | Path | None = None):
    root = Path(output_root) if output_root else Path(__file__).resolve().parents[1]
    mesh_dir = root / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    parts = make_parts()
    inventory = []
    for name in PART_ORDER:
        path = mesh_dir / f"{name}.stl"
        export_stl(parts[name], path, tolerance=0.08, angular_tolerance=0.12)
        inventory.append({"name": name, "mesh": f"meshes/{name}.stl", "volume_mm3": parts[name].volume})
    export_step(build_machine(), root / "machine.step")
    (root / "raw" / "part_inventory.json").write_text(json.dumps(inventory, indent=2), encoding="utf-8")
    return inventory


if __name__ == "__main__":
    exported = export_artifacts()
    print(json.dumps({"mechanism": MECHANISM["name"], "part_count": len(exported), "parts": [p["name"] for p in exported]}))
