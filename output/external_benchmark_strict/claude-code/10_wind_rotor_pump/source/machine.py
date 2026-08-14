"""Parametric open-frame wind-rotor reciprocating pump benchmark model.

CAD units are millimetres.  The rotor/crank axis is world +Y and the pump
output translates on world +Z.  The zero-pose crank pin is at +X.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

from build123d import Box, Compound, Cylinder, Location, Pos, Rot, Sphere, export_step, export_stl

ROTOR_AXIS_Z = 150.0
CRANK_RADIUS = 20.0
ROD_LENGTH = 100.0
CROSSHEAD_Z = ROTOR_AXIS_Z - math.sqrt(ROD_LENGTH**2 - CRANK_RADIUS**2)

MECHANISM = {
    "name": "open_wind_rotor_reciprocating_pump",
    "units": "mm",
    "coordinate_system": {
        "origin": "centre of base footprint at ground mounting plane",
        "rotor_axis": [0.0, 1.0, 0.0],
        "vertical_output_axis": [0.0, 0.0, 1.0],
    },
    "links": [
        {"name": "base", "dof": "fixed", "rigid_mount": "world"},
        {"name": "front_bearing", "dof": "fixed", "rigid_mount": "base"},
        {"name": "rear_bearing", "dof": "fixed", "rigid_mount": "base"},
        {"name": "vertical_guide", "dof": "fixed", "rigid_mount": "base"},
        {"name": "rotor_shaft", "dof": "revolute", "axis": [0, 1, 0], "driver": True},
        {"name": "wind_rotor", "dof": "rigid", "rigid_mount": "rotor_shaft"},
        {"name": "crank_disk", "dof": "rigid", "rigid_mount": "rotor_shaft"},
        {"name": "crank_pin", "dof": "rigid", "rigid_mount": "crank_disk"},
        {"name": "connecting_rod", "dof": "revolute", "axis": [0, 1, 0]},
        {"name": "vertical_crosshead", "dof": "slide", "axis": [0, 0, 1]},
        {"name": "pump_rod", "dof": "rigid", "rigid_mount": "vertical_crosshead"},
        {"name": "piston_output", "dof": "rigid", "rigid_mount": "pump_rod"},
    ],
    "ports": [
        {"name": "world_rotor_axis", "link": "base", "kind": "revolute", "axis": [0, 1, 0], "point_mm": [0, 0, ROTOR_AXIS_Z]},
        {"name": "shaft_front_fit", "link": "rotor_shaft", "kind": "running_bearing", "point_mm": [0, -19, ROTOR_AXIS_Z]},
        {"name": "shaft_rear_fit", "link": "rotor_shaft", "kind": "running_bearing", "point_mm": [0, 19, ROTOR_AXIS_Z]},
        {"name": "crank_pin_axis", "link": "crank_pin", "kind": "pin", "axis": [0, 1, 0], "point_mm": [CRANK_RADIUS, 24, ROTOR_AXIS_Z]},
        {"name": "rod_big_end", "link": "connecting_rod", "kind": "pin_hinge", "point_mm": [CRANK_RADIUS, 24, ROTOR_AXIS_Z]},
        {"name": "rod_small_end", "link": "connecting_rod", "kind": "closure", "point_mm": [0, 24, CROSSHEAD_Z]},
        {"name": "crosshead_pin", "link": "vertical_crosshead", "kind": "closure", "point_mm": [0, 24, CROSSHEAD_Z]},
        {"name": "guide_axis", "link": "vertical_guide", "kind": "linear", "axis": [0, 0, 1], "point_mm": [0, 24, CROSSHEAD_Z]},
    ],
    "relations": [
        {"name": "base_to_world", "kind": "rigid_mount", "a": "base", "b": "world"},
        {"name": "front_running_bearing", "kind": "running_bearing", "a": "front_bearing", "b": "rotor_shaft", "clearance_mm": 0.8},
        {"name": "rear_running_bearing", "kind": "running_bearing", "a": "rear_bearing", "b": "rotor_shaft", "clearance_mm": 0.8},
        {"name": "rotor_press_fit", "kind": "press_fit", "a": "wind_rotor", "b": "rotor_shaft"},
        {"name": "disk_press_fit", "kind": "press_fit", "a": "crank_disk", "b": "rotor_shaft"},
        {"name": "pin_press_fit", "kind": "press_fit", "a": "crank_pin", "b": "crank_disk"},
        {"name": "big_end_pin_hinge", "kind": "pin_hinge", "a": "connecting_rod", "b": "crank_pin"},
        {"name": "small_end_closure", "kind": "point_closure", "a": "connecting_rod", "b": "vertical_crosshead"},
        {"name": "crosshead_running_slide", "kind": "running_guide", "a": "vertical_crosshead", "b": "vertical_guide", "clearance_mm": 1.0},
        {"name": "crosshead_carries_rod", "kind": "rigid_carrying", "a": "vertical_crosshead", "b": "pump_rod"},
        {"name": "rod_carries_piston", "kind": "rigid_carrying", "a": "pump_rod", "b": "piston_output"},
    ],
    "motion_joints": [
        {"name": "rotor_shaft_hinge", "kind": "revolute", "parent": "base", "child": "rotor_shaft", "axis": [0, 1, 0], "driver": True},
        {"name": "connecting_rod_big_end_hinge", "kind": "revolute", "parent": "crank_pin", "child": "connecting_rod", "axis": [0, 1, 0]},
        {"name": "vertical_crosshead_slide", "kind": "slide", "parent": "vertical_guide", "child": "vertical_crosshead", "axis": [0, 0, 1], "output": True},
    ],
    "closures": [
        {"name": "rod_crosshead_closure", "kind": "point_coincident", "port_a": "rod_small_end", "port_b": "crosshead_pin", "scale_mm": ROD_LENGTH}
    ],
    "transmissions": [],
    "driver": {"joint": "rotor_shaft_hinge", "link": "rotor_shaft", "mode": "imposed_position", "direct_output_actuation": False},
    "output": {"joint": "vertical_crosshead_slide", "link": "piston_output", "kind": "vertical_translation"},
    "watch_links": ["wind_rotor", "crank_disk", "crank_pin", "connecting_rod", "vertical_crosshead", "pump_rod", "piston_output"],
}


def _along_y(radius: float, length: float, center_y: float, center_x: float = 0.0, center_z: float = 0.0):
    return Cylinder(radius, length).moved(Pos(center_x, center_y - length / 2.0, center_z) * Rot(90, 0, 0))


def _rod_between(p1, p2, radius: float):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # Cylinder local +Z is rotated onto the endpoint vector.
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.acos(dz / length))
    body = Cylinder(radius, length).moved(Pos(x1, y1, z1) * Rot(0, pitch, yaw))
    return Compound(children=[body, Sphere(radius).moved(Pos(*p1)), Sphere(radius).moved(Pos(*p2))])


def make_parts() -> dict[str, object]:
    parts: dict[str, object] = {}

    base_plate = Box(180, 110, 10).moved(Pos(-90, -55, 0))
    left_post = Box(12, 14, 150).moved(Pos(-55, -7, 10))
    right_post = Box(12, 14, 150).moved(Pos(43, -7, 10))
    top_beam = Box(110, 14, 12).moved(Pos(-55, -7, 148))
    braces = [
        _rod_between((-72, 0, 10), (-55, 0, 128), 4.0),
        _rod_between((72, 0, 10), (55, 0, 128), 4.0),
    ]
    parts["base"] = Compound(children=[base_plate, left_post, right_post, top_beam, *braces])

    parts["front_bearing"] = Compound(children=[
        _along_y(15, 10, -15, 0, ROTOR_AXIS_Z),
        Box(38, 10, 8).moved(Pos(-19, -20, ROTOR_AXIS_Z - 23)),
    ])
    parts["rear_bearing"] = Compound(children=[
        _along_y(15, 10, 15, 0, ROTOR_AXIS_Z),
        Box(38, 10, 8).moved(Pos(-19, 10, ROTOR_AXIS_Z - 23)),
    ])
    parts["rotor_shaft"] = _along_y(6.0, 92.0, 0.0, 0.0, ROTOR_AXIS_Z)

    rotor_children = [_along_y(15, 8, -50, 0, ROTOR_AXIS_Z)]
    for angle_deg in range(0, 360, 45):
        a = math.radians(angle_deg)
        r0, r1 = 18.0, 67.0
        x0, z0 = r0 * math.cos(a), ROTOR_AXIS_Z + r0 * math.sin(a)
        x1, z1 = r1 * math.cos(a), ROTOR_AXIS_Z + r1 * math.sin(a)
        rotor_children.append(_rod_between((x0, -50, z0), (x1, -50, z1), 3.2))
        bx, bz = 52 * math.cos(a), ROTOR_AXIS_Z + 52 * math.sin(a)
        rotor_children.append(Box(30, 5, 10).moved(Pos(bx - 15, -52.5, bz - 5) * Rot(0, -angle_deg, 0)))
    parts["wind_rotor"] = Compound(children=rotor_children)

    parts["crank_disk"] = _along_y(32.0, 8.0, 30.0, 0.0, ROTOR_AXIS_Z)
    parts["crank_pin"] = _along_y(5.0, 20.0, 44.0, CRANK_RADIUS, ROTOR_AXIS_Z)

    big = (CRANK_RADIUS, 54.0, ROTOR_AXIS_Z)
    small = (0.0, 54.0, CROSSHEAD_Z)
    parts["connecting_rod"] = _rod_between(big, small, 4.5)
    parts["vertical_crosshead"] = Box(28, 16, 18).moved(Pos(-14, 46, CROSSHEAD_Z - 9))

    rail1 = Box(6, 14, 90).moved(Pos(-23, 47, 25))
    rail2 = Box(6, 14, 90).moved(Pos(17, 47, 25))
    lower_bridge = Box(46, 14, 7).moved(Pos(-23, 47, 22))
    parts["vertical_guide"] = Compound(children=[rail1, rail2, lower_bridge])

    parts["pump_rod"] = Cylinder(4, 50).moved(Pos(0, 54, CROSSHEAD_Z - 50))
    parts["piston_output"] = Cylinder(12, 8).moved(Pos(0, 54, CROSSHEAD_Z - 58))

    for name, shape in parts.items():
        shape.label = name
    return parts


def build_machine():
    """Return a deterministic, nonempty labeled build123d assembly compound."""
    parts = make_parts()
    root = Compound(label=MECHANISM["name"], children=list(parts.values()))
    return root


def gen_step():
    return build_machine()


def export_named(output_root: Path) -> None:
    output_root.mkdir(parents=True, exist_ok=True)
    mesh_dir = output_root / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    parts = make_parts()
    for name, shape in parts.items():
        export_stl(shape, mesh_dir / f"{name}.stl", tolerance=0.08, angular_tolerance=0.12)
    export_step(build_machine(), output_root / "models" / "machine.step")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--export-root", type=Path, required=True)
    args = parser.parse_args()
    (args.export_root / "models").mkdir(parents=True, exist_ok=True)
    export_named(args.export_root)
    print(f"exported {len(make_parts())} named parts")
