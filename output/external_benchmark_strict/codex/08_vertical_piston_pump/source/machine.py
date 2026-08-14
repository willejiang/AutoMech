"""Open-frame vertical piston-pump demonstration mechanism.

CAD dimensions are millimetres. World X is horizontal in the mechanism plane,
Y is the crankshaft/pin axis, and Z is vertical. Each independently moving
member is an independent labeled solid; no fluid behavior is modeled.
"""
from __future__ import annotations

import math
from pathlib import Path

from build123d import Align, Axis, Box, Compound, Cylinder, Location, export_stl


CRANK_AXIS = (-45.0, 0.0, 75.0)
CRANK_RADIUS = 20.0
ROD_LENGTH = 80.0
INITIAL_CRANK_ANGLE_DEG = 30.0
CRANK_PIN = (
    CRANK_AXIS[0] + CRANK_RADIUS * math.cos(math.radians(INITIAL_CRANK_ANGLE_DEG)),
    0.0,
    CRANK_AXIS[2] + CRANK_RADIUS * math.sin(math.radians(INITIAL_CRANK_ANGLE_DEG)),
)
CROSSHEAD_Z = CRANK_PIN[2] + math.sqrt(ROD_LENGTH**2 - CRANK_PIN[0] ** 2)

PART_POSES_MM = {
    "base": (0.0, 0.0, 4.0),
    "left_crank_support": (-45.0, -29.0, 41.5),
    "right_crank_support": (-45.0, 29.0, 41.5),
    "front_crank_bearing": (-45.0, -29.0, 75.0),
    "rear_crank_bearing": (-45.0, 29.0, 75.0),
    "crankshaft_input": CRANK_AXIS,
    "crank_web": CRANK_AXIS,
    "eccentric_pin": CRANK_PIN,
    "connecting_rod": ((CRANK_PIN[0]) / 2.0, 0.0, (CRANK_PIN[2] + CROSSHEAD_Z) / 2.0),
    "vertical_crosshead": (0.0, 0.0, CROSSHEAD_Z),
    "left_vertical_guide": (0.0, -23.0, 154.0),
    "right_vertical_guide": (0.0, 23.0, 154.0),
    "pump_rod": (0.0, 0.0, CROSSHEAD_Z),
    "piston_output": (0.0, 0.0, CROSSHEAD_Z),
    "lower_cylinder_frame": (0.0, 0.0, 84.0),
    "upper_cylinder_frame": (0.0, 0.0, 210.0),
    "hand_crank": CRANK_AXIS,
}

MECHANISM = {
    "name": "open_vertical_piston_pump",
    "units": {"cad_length": "mm", "assembly_length": "m", "angle": "rad"},
    "links": [
        {"name": name, "pose_mm": list(pose)} for name, pose in PART_POSES_MM.items()
    ],
    "ports": {
        "crank_axis": {"link": "crankshaft_input", "origin_mm": list(CRANK_AXIS), "axis": [0, 1, 0]},
        "eccentric_pin_axis": {"link": "eccentric_pin", "origin_mm": list(CRANK_PIN), "axis": [0, 1, 0]},
        "rod_big_end": {"link": "connecting_rod", "origin_mm": list(CRANK_PIN), "axis": [0, 1, 0]},
        "rod_small_end": {"link": "connecting_rod", "origin_mm": [0, 0, CROSSHEAD_Z], "axis": [0, 1, 0]},
        "crosshead_pin_axis": {"link": "vertical_crosshead", "origin_mm": [0, 0, CROSSHEAD_Z], "axis": [0, 1, 0]},
        "vertical_guide_axis": {"link": "vertical_crosshead", "origin_mm": [0, 0, CROSSHEAD_Z], "axis": [0, 0, 1]},
        "piston_axis": {"link": "piston_output", "origin_mm": [0, 0, CROSSHEAD_Z], "axis": [0, 0, 1]},
    },
    "relations": [
        {"type": "rigid_mount", "parent": "base", "child": "left_crank_support"},
        {"type": "rigid_mount", "parent": "base", "child": "right_crank_support"},
        {"type": "rigid_mount", "parent": "base", "child": "left_vertical_guide"},
        {"type": "rigid_mount", "parent": "base", "child": "right_vertical_guide"},
        {"type": "rigid_mount", "parent": "base", "child": "lower_cylinder_frame"},
        {"type": "rigid_mount", "parent": "base", "child": "upper_cylinder_frame"},
        {"type": "running_bearing", "bearing": "front_crank_bearing", "shaft": "crankshaft_input", "radial_clearance_mm": 0.75},
        {"type": "running_bearing", "bearing": "rear_crank_bearing", "shaft": "crankshaft_input", "radial_clearance_mm": 0.75},
        {"type": "press_fit", "outer": "crank_web", "inner": "crankshaft_input"},
        {"type": "press_fit", "outer": "hand_crank", "inner": "crankshaft_input"},
        {"type": "dedicated_pin_fit", "pin": "eccentric_pin", "rod": "connecting_rod", "radial_clearance_mm": 0.5},
        {"type": "revolute", "name": "rod_crosshead_revolute", "parent": "vertical_crosshead", "child": "connecting_rod", "axis": [0, 1, 0]},
        {"type": "closure", "name": "rod_crosshead_closure", "a": "rod_small_end", "b": "crosshead_pin_axis", "scale_mm": 100.0},
        {"type": "linear_guide", "name": "crosshead_vertical_guide", "guide_links": ["left_vertical_guide", "right_vertical_guide"], "moving_link": "vertical_crosshead", "axis": [0, 0, 1]},
        {"type": "rigid_carry", "name": "crosshead_carries_pump_rod", "parent": "vertical_crosshead", "child": "pump_rod"},
        {"type": "rigid_carry", "name": "pump_rod_carries_piston", "parent": "pump_rod", "child": "piston_output"},
    ],
    "motion_joints": [
        {"name": "crankshaft_input_hinge", "parent": "base", "child": "crankshaft_input", "kind": "revolute", "axis": [0, 1, 0], "origin_m": [-0.045, 0, 0.075], "driver": True},
        {"name": "eccentric_pin_hinge", "parent": "eccentric_pin", "child": "connecting_rod", "kind": "revolute", "axis": [0, 1, 0], "origin_m": [0, 0, 0], "driver": False},
        {"name": "rod_crosshead_hinge", "parent": "vertical_crosshead", "child": "connecting_rod", "kind": "revolute", "axis": [0, 1, 0], "origin_m": [0, 0, 0], "driver": False},
        {"name": "vertical_crosshead_slide", "parent": "base", "child": "vertical_crosshead", "kind": "slide", "axis": [0, 0, 1], "origin_m": [0, 0, CROSSHEAD_Z / 1000.0], "driver": False},
    ],
    "transmissions": [],
    "planetary_stages": [],
    "mesh_pairs": [],
    "driver": {"joint": "crankshaft_input_hinge", "source": "hand_crank", "mode": "input_velocity"},
    "output": {"joint": "vertical_crosshead_slide", "link": "piston_output"},
    "watch_links": ["crank_web", "eccentric_pin", "connecting_rod", "vertical_crosshead", "pump_rod", "piston_output"],
}


def _cylinder_y(radius: float, length: float):
    return Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.X, 90.0)


def _rod_between_xz(a: tuple[float, float], b: tuple[float, float], width: float = 14.0, depth: float = 10.0):
    ax, az = a
    bx, bz = b
    length = math.hypot(bx - ax, bz - az)
    angle = -math.degrees(math.atan2(bz - az, bx - ax))
    bar = Box(length, depth, width, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.Y, angle)
    bar = bar.translate(((ax + bx) / 2.0, 0.0, (az + bz) / 2.0))
    return bar.fuse(
        _cylinder_y(width / 2.0, depth).translate((ax, 0.0, az)),
        _cylinder_y(width / 2.0, depth).translate((bx, 0.0, bz)),
    )


def _bearing_ring():
    return _cylinder_y(13.0, 10.0).cut(_cylinder_y(6.75, 12.0))


def build_local_parts():
    rod_dx = CRANK_PIN[0]
    rod_dz = CROSSHEAD_Z - CRANK_PIN[2]
    rod_angle = -math.degrees(math.atan2(rod_dz, -rod_dx))
    parts = {
        "base": Box(230.0, 110.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "left_crank_support": Box(18.0, 14.0, 67.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "right_crank_support": Box(18.0, 14.0, 67.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "front_crank_bearing": _bearing_ring(),
        "rear_crank_bearing": _bearing_ring(),
        "crankshaft_input": _cylinder_y(6.0, 86.0),
        "crank_web": _cylinder_y(14.0, 8.0).fuse(
            Box(CRANK_RADIUS, 8.0, 12.0, align=(Align.MIN, Align.CENTER, Align.CENTER))
                .rotate(Axis.Y, -INITIAL_CRANK_ANGLE_DEG)
        ),
        "eccentric_pin": _cylinder_y(5.0, 20.0),
        "connecting_rod": _rod_between_xz((-ROD_LENGTH / 2.0, 0.0), (ROD_LENGTH / 2.0, 0.0)),
        "vertical_crosshead": Box(30.0, 28.0, 24.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).fuse(_cylinder_y(6.0, 38.0)),
        "left_vertical_guide": Box(34.0, 8.0, 124.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "right_vertical_guide": Box(34.0, 8.0, 124.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "pump_rod": Cylinder(4.0, 58.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).translate((0.0, 0.0, -58.0)),
        "piston_output": Cylinder(13.0, 12.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((0.0, 0.0, -58.0)),
        "lower_cylinder_frame": Cylinder(24.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).cut(
            Cylinder(16.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        ),
        "upper_cylinder_frame": Cylinder(24.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).cut(
            Cylinder(16.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        ),
        "hand_crank": _cylinder_y(10.0, 8.0).translate((0.0, -48.0, 0.0)).fuse(
            Box(9.0, 7.0, 35.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).translate((0.0, -52.0, 0.0)),
            _cylinder_y(6.0, 22.0).translate((0.0, -63.0, 35.0)),
        ),
    }
    # The rod is authored horizontally in its own local frame; rotate it into
    # the selected first pose around its center.
    parts["connecting_rod"] = parts["connecting_rod"].rotate(Axis.Y, rod_angle)
    for name, shape in parts.items():
        shape.label = name
    return parts


def build_machine():
    children = []
    for name, shape in build_local_parts().items():
        placed = shape.moved(Location(PART_POSES_MM[name]))
        placed.label = name
        children.append(placed)
    result = Compound(children=children)
    result.label = MECHANISM["name"]
    return result


def gen_step():
    return build_machine()


def export_named_stls(output_directory: str | Path) -> list[Path]:
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    written = []
    for name, shape in build_local_parts().items():
        target = output / f"{name}.stl"
        export_stl(shape, target, tolerance=0.12, angular_tolerance=0.12)
        written.append(target)
    return written


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--export-stls", type=Path)
    args = parser.parse_args()
    machine = build_machine()
    if not machine.solids():
        raise RuntimeError("build_machine() produced no solids")
    if args.export_stls is not None:
        for path in export_named_stls(args.export_stls):
            print(path)
