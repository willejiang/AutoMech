"""Open-frame 4:1 single-stage spur reducer for PhysCAD Comfort task 01.

CAD geometry uses millimetres.  The shaft axes are parallel to world +Y.
Every exported mesh is authored in its link-local frame; PART_POSES_MM places it
in the selected final assembly.
"""

from __future__ import annotations

import math
from pathlib import Path

from build123d import Align, Axis, Box, Compound, Cylinder, Location, export_stl


PART_POSES_MM = {
    "base": (20.0, 0.0, 4.0),
    "input_front_bearing": (-45.0, -28.0, 90.0),
    "input_rear_bearing": (-45.0, 28.0, 90.0),
    "output_front_bearing": (45.0, -28.0, 90.0),
    "output_rear_bearing": (45.0, 28.0, 90.0),
    "input_shaft": (-45.0, 0.0, 90.0),
    "output_shaft": (45.0, 0.0, 90.0),
    "input_pinion": (-45.0, 0.0, 90.0),
    "output_gear": (45.0, 0.0, 90.0),
    "hand_crank": (-45.0, 0.0, 90.0),
}


MECHANISM = {
    "name": "open_frame_single_stage_4to1",
    "units": {"cad_length": "mm", "assembly_length": "m", "angle": "rad"},
    "links": [
        {"name": "base", "dof": "fixed"},
        {"name": "input_front_bearing", "dof": "fixed"},
        {"name": "input_rear_bearing", "dof": "fixed"},
        {"name": "output_front_bearing", "dof": "fixed"},
        {"name": "output_rear_bearing", "dof": "fixed"},
        {"name": "input_shaft", "dof": "spin", "axis": [0.0, 1.0, 0.0], "driver": True},
        {"name": "output_shaft", "dof": "spin", "axis": [0.0, 1.0, 0.0], "driver": False},
        {"name": "input_pinion", "dof": "rigid_with_input_shaft"},
        {"name": "output_gear", "dof": "rigid_with_output_shaft"},
        {"name": "hand_crank", "dof": "rigid_with_input_shaft"},
    ],
    "ports": {
        "input_shaft_axis": {"link": "input_shaft", "origin_mm": [-45.0, 0.0, 90.0], "axis": [0.0, 1.0, 0.0]},
        "output_shaft_axis": {"link": "output_shaft", "origin_mm": [45.0, 0.0, 90.0], "axis": [0.0, 1.0, 0.0]},
    },
    "relations": [
        {"type": "rigid_mount", "parent": "base", "child": "input_front_bearing"},
        {"type": "rigid_mount", "parent": "base", "child": "input_rear_bearing"},
        {"type": "rigid_mount", "parent": "base", "child": "output_front_bearing"},
        {"type": "rigid_mount", "parent": "base", "child": "output_rear_bearing"},
        {"type": "running_bearing", "bearing": "input_front_bearing", "shaft": "input_shaft", "radial_clearance_mm": 1.0},
        {"type": "running_bearing", "bearing": "input_rear_bearing", "shaft": "input_shaft", "radial_clearance_mm": 1.0},
        {"type": "running_bearing", "bearing": "output_front_bearing", "shaft": "output_shaft", "radial_clearance_mm": 1.0},
        {"type": "running_bearing", "bearing": "output_rear_bearing", "shaft": "output_shaft", "radial_clearance_mm": 1.0},
        {"type": "press_fit", "outer": "input_pinion", "inner": "input_shaft"},
        {"type": "press_fit", "outer": "output_gear", "inner": "output_shaft"},
        {"type": "press_fit", "outer": "hand_crank", "inner": "input_shaft"},
        {"type": "ideal_external_gear_mesh", "driving": "input_pinion", "driven": "output_gear", "driving_teeth": 12, "driven_teeth": 48},
    ],
    "motion_joints": [
        {"name": "input_shaft_hinge", "parent": "base", "child": "input_shaft", "kind": "revolute", "axis": [0.0, 1.0, 0.0], "origin_m": [-0.045, 0.0, 0.09], "driver": True},
        {"name": "output_shaft_hinge", "parent": "base", "child": "output_shaft", "kind": "revolute", "axis": [0.0, 1.0, 0.0], "origin_m": [0.045, 0.0, 0.09], "driver": False},
    ],
    "transmissions": [
        {
            "name": "spur_mesh_12t_to_48t",
            "kind": "ideal_external_gear",
            "driving_joint": "input_shaft_hinge",
            "driven_joint": "output_shaft_hinge",
            "ratio_convention": "driven_angular_displacement_over_driving_angular_displacement",
            "ratio": -0.25,
        }
    ],
    "driver": {"joint": "input_shaft_hinge", "source": "hand_crank"},
    "output": {"joint": "output_shaft_hinge", "link": "output_shaft"},
    "watch_links": ["hand_crank", "input_pinion", "output_gear", "output_shaft"],
}


def _cylinder_y(radius: float, length: float):
    return Cylinder(
        radius,
        length,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).rotate(Axis.X, 90.0)


def _spur_gear(teeth: int, module: float, thickness: float, phase_deg: float = 0.0):
    pitch_radius = module * teeth / 2.0
    root_radius = pitch_radius - 1.25 * module
    outer_radius = pitch_radius + module
    core = _cylinder_y(root_radius, thickness)
    radial_start = root_radius - 0.7
    radial_depth = outer_radius - radial_start
    tangential_width = 0.36 * math.pi * module
    tangential_inner_width = tangential_width * root_radius / outer_radius
    teeth_solids = []
    for index in range(teeth):
        # A tapered prism approximates an involute tooth envelope while keeping
        # each tooth a simple robust BREP.  Narrowing the tip prevents the two
        # exact pitch envelopes from interpenetrating in the selected pose.
        from build123d import Plane, Polygon, extrude

        profile = Polygon(
            (radial_start, -tangential_inner_width / 2.0),
            (outer_radius, -tangential_width / 2.0),
            (outer_radius, tangential_width / 2.0),
            (radial_start, tangential_inner_width / 2.0),
            align=None,
        )
        tooth = extrude(Plane.XZ * profile, amount=thickness / 2.0, both=True)
        tooth = tooth.rotate(Axis.Y, phase_deg + index * 360.0 / teeth)
        teeth_solids.append(tooth)
    return core.fuse(*teeth_solids)


def _bearing_pedestal():
    outer = _cylinder_y(14.0, 10.0)
    bore = _cylinder_y(6.0, 12.0)
    ring = outer.cut(bore)
    # In this link-local frame the bearing axis is the origin.  The column
    # reaches from the ring underside to the base top at world Z=8 mm.
    column = Box(
        20.0,
        10.0,
        69.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).translate((0.0, 0.0, -47.5))
    return ring.fuse(column)


def _hand_crank():
    hub = _cylinder_y(10.0, 8.0).translate((0.0, -50.0, 0.0))
    arm = Box(
        9.0,
        6.0,
        36.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).translate((0.0, -54.0, 18.0))
    handle = _cylinder_y(6.0, 22.0).translate((0.0, -65.0, 36.0))
    return hub.fuse(arm, handle)


def build_local_parts():
    """Return a fresh deterministic mapping of semantic names to local solids."""
    bearing = _bearing_pedestal()
    parts = {
        "base": Box(230.0, 100.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "input_front_bearing": bearing,
        "input_rear_bearing": _bearing_pedestal(),
        "output_front_bearing": _bearing_pedestal(),
        "output_rear_bearing": _bearing_pedestal(),
        "input_shaft": _cylinder_y(5.0, 100.0),
        "output_shaft": _cylinder_y(5.0, 82.0),
        "input_pinion": _spur_gear(teeth=12, module=3.0, thickness=12.0, phase_deg=0.0),
        "output_gear": _spur_gear(teeth=48, module=3.0, thickness=12.0, phase_deg=3.75),
        "hand_crank": _hand_crank(),
    }
    for name, shape in parts.items():
        shape.label = name
    return parts


def build_machine():
    """Build the selected final assembly as a labeled build123d Compound."""
    placed = []
    for name, shape in build_local_parts().items():
        moved = shape.moved(Location(PART_POSES_MM[name]))
        moved.label = name
        placed.append(moved)
    machine = Compound(children=placed)
    machine.label = "open_frame_single_stage_4to1"
    return machine


def gen_step():
    """CAD-skill entry point."""
    return build_machine()


def export_named_stls(output_directory: str | Path) -> list[Path]:
    """Export every semantic part as a true link-local STL mesh."""
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
    arguments = parser.parse_args()
    built = build_machine()
    if not built.solids():
        raise RuntimeError("build_machine() produced no solids")
    if arguments.export_stls is not None:
        for path in export_named_stls(arguments.export_stls):
            print(path)
