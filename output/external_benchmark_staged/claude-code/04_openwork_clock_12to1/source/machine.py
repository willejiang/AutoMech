"""Parametric openwork 12:1 clock display.

CAD units are millimetres.  The clock face is the XZ plane, shafts run on Y,
and the camera side is -Y.  Running clearances and ideal mesh interfaces are
explicit in MECHANISM.  Executing this file exports one STL per named part.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

from build123d import Align, Axis, Box, Compound, Cylinder, Location, export_step, export_stl

TASK_ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = TASK_ROOT / "meshes"

CENTER_Z = 112.0
INTERMEDIATE_X = 60.0
SHAFT_R = 4.0
SLEEVE_OUT_R = 6.4
SLEEVE_IN_R = 5.0
MODULE = 2.0

MECHANISM = {
    "name": "openwork_clock_12to1",
    "units": "mm",
    "coordinate_system": {
        "clock_plane": "XZ",
        "shaft_axis": [0.0, 1.0, 0.0],
        "camera_side": "-Y",
    },
    "links": [
        {"name": "base_frame", "dof": "fixed", "rigid_mount": "world"},
        {"name": "minute_shaft", "dof": "spin", "axis": [0, 1, 0], "driver": True},
        {"name": "minute_pinion_15t", "dof": "rigid_carried", "carrier": "minute_shaft"},
        {"name": "minute_hand", "dof": "rigid_carried", "carrier": "minute_shaft"},
        {"name": "intermediate_shaft", "dof": "spin", "axis": [0, 1, 0]},
        {"name": "intermediate_gear_45t", "dof": "rigid_carried", "carrier": "intermediate_shaft"},
        {"name": "intermediate_pinion_12t", "dof": "rigid_carried", "carrier": "intermediate_shaft"},
        {"name": "hour_sleeve", "dof": "spin", "axis": [0, 1, 0], "output": True},
        {"name": "hour_gear_48t", "dof": "rigid_carried", "carrier": "hour_sleeve"},
        {"name": "hour_hand", "dof": "rigid_carried", "carrier": "hour_sleeve"},
    ],
    "ports": {
        "base_frame": ["minute_rear_bearing", "minute_front_sleeve_bearing", "intermediate_rear_bearing", "intermediate_front_bearing"],
        "minute_shaft": ["minute_axis", "minute_pinion_seat", "minute_hand_seat"],
        "intermediate_shaft": ["intermediate_axis", "driven_gear_seat", "second_pinion_seat"],
        "hour_sleeve": ["hour_axis", "hour_gear_seat", "hour_hand_seat"],
    },
    "relations": [
        {"kind": "running_bearing", "a": "base_frame.minute_rear_bearing", "b": "minute_shaft.minute_axis", "radial_clearance_mm": 0.6},
        {"kind": "running_bearing", "a": "base_frame.minute_front_sleeve_bearing", "b": "hour_sleeve.hour_axis", "radial_clearance_mm": 0.7},
        {"kind": "running_clearance", "a": "hour_sleeve.hour_axis", "b": "minute_shaft.minute_axis", "radial_clearance_mm": 1.0},
        {"kind": "running_bearing", "a": "base_frame.intermediate_rear_bearing", "b": "intermediate_shaft.intermediate_axis", "radial_clearance_mm": 0.6},
        {"kind": "running_bearing", "a": "base_frame.intermediate_front_bearing", "b": "intermediate_shaft.intermediate_axis", "radial_clearance_mm": 0.6},
        {"kind": "press_fit", "a": "minute_shaft.minute_pinion_seat", "b": "minute_pinion_15t", "interference_mm": 0.10},
        {"kind": "press_fit", "a": "minute_shaft.minute_hand_seat", "b": "minute_hand", "interference_mm": 0.10},
        {"kind": "press_fit", "a": "intermediate_shaft.driven_gear_seat", "b": "intermediate_gear_45t", "interference_mm": 0.10},
        {"kind": "press_fit", "a": "intermediate_shaft.second_pinion_seat", "b": "intermediate_pinion_12t", "interference_mm": 0.10},
        {"kind": "press_fit", "a": "hour_sleeve.hour_gear_seat", "b": "hour_gear_48t", "interference_mm": 0.10},
        {"kind": "press_fit", "a": "hour_sleeve.hour_hand_seat", "b": "hour_hand", "interference_mm": 0.10},
        {"kind": "ideal_gear_mesh", "a": "minute_pinion_15t", "b": "intermediate_gear_45t", "center_distance_mm": 60.0},
        {"kind": "ideal_gear_mesh", "a": "intermediate_pinion_12t", "b": "hour_gear_48t", "center_distance_mm": 60.0},
    ],
    "motion_joints": [
        {"name": "minute_input_hinge", "parent": "base_frame", "child": "minute_shaft", "kind": "revolute", "axis": [0, 1, 0], "origin_mm": [0, 0, CENTER_Z]},
        {"name": "intermediate_hinge", "parent": "base_frame", "child": "intermediate_shaft", "kind": "revolute", "axis": [0, 1, 0], "origin_mm": [INTERMEDIATE_X, 0, CENTER_Z]},
        {"name": "hour_output_hinge", "parent": "base_frame", "child": "hour_sleeve", "kind": "revolute", "axis": [0, 1, 0], "origin_mm": [0, 0, CENTER_Z]},
    ],
    "transmissions": [
        {"name": "stage_1", "driving": "minute_input_hinge", "driven": "intermediate_hinge", "ratio_driven_over_driving": -1.0 / 3.0, "teeth": [15, 45]},
        {"name": "stage_2", "driving": "intermediate_hinge", "driven": "hour_output_hinge", "ratio_driven_over_driving": -1.0 / 4.0, "teeth": [12, 48]},
    ],
    "driver": {"joint": "minute_input_hinge", "kind": "velocity", "only_driver": True},
    "output": {"joint": "hour_output_hinge", "link": "hour_sleeve"},
    "watch_links": ["minute_hand", "hour_hand", "minute_shaft", "hour_sleeve", "intermediate_shaft"],
}


def _at(shape, xyz):
    return shape.moved(Location(xyz))


def _y_cylinder(radius: float, length: float, center=(0.0, 0.0, 0.0)):
    shape = Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    shape = shape.rotate(Axis.X, 90.0)
    return _at(shape, center)


def _ring_y(outer_r: float, inner_r: float, length: float, center):
    return _y_cylinder(outer_r, length, center) - _y_cylinder(inner_r, length + 2.0, center)


def _gear(name: str, teeth: int, module: float, thickness: float, center):
    pitch_r = module * teeth / 2.0
    root_r = pitch_r - 1.25 * module
    outer_r = pitch_r + module
    body = _y_cylinder(root_r, thickness, center)
    tooth_tangent = max(1.8, math.pi * module * 0.46)
    tooth_radial = outer_r - root_r + 0.7
    for i in range(teeth):
        tooth = Box(tooth_radial, thickness, tooth_tangent,
                    align=(Align.CENTER, Align.CENTER, Align.CENTER))
        tooth = _at(tooth, (center[0] + root_r + tooth_radial / 2.0 - 0.35, center[1], center[2]))
        tooth = tooth.rotate(Axis(center, (0, 1, 0)), i * 360.0 / teeth)
        body = body.fuse(tooth)
    hub = _y_cylinder(8.0, thickness + 2.0, center)
    body = body.fuse(hub)
    body.label = name
    return body


def _hand(name: str, length: float, width: float, thickness: float, angle_deg: float, y: float):
    # Local hand points along +Z from the common axis; hub overlap is a declared press fit.
    arm = Box(width, thickness, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    arm = _at(arm, (0, y, CENTER_Z + length / 2.0))
    tip = _y_cylinder(width * 0.75, thickness, (0, y, CENTER_Z + length))
    hub = _y_cylinder(8.0, thickness + 1.0, (0, y, CENTER_Z))
    hand = arm.fuse(tip).fuse(hub)
    hand = hand.rotate(Axis((0, y, CENTER_Z), (0, 1, 0)), angle_deg)
    hand.label = name
    return hand


def build_parts():
    # Stable bench and open side supports, all fused into one fixed link.
    base = Box(160, 72, 10, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    base = _at(base, (15, 7, 5))
    left_foot = Box(30, 88, 6, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    right_foot = Box(30, 88, 6, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    frame = base.fuse(_at(left_foot, (-45, 7, 3))).fuse(_at(right_foot, (75, 7, 3)))
    for x in (-45.0, 78.0):
        upright = Box(10, 10, 135, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        frame = frame.fuse(_at(upright, (x, 15, 72.5)))
    topbar = Box(133, 10, 9, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    frame = frame.fuse(_at(topbar, (16.5, 15, 140.5)))
    # Open bearing rings are conspicuous without covering the gear faces.
    frame = frame.fuse(_ring_y(12.0, 4.6, 6.0, (0, 15, CENTER_Z)))
    frame = frame.fuse(_ring_y(12.0, 4.6, 6.0, (INTERMEDIATE_X, 15, CENTER_Z)))
    frame = frame.fuse(_ring_y(11.0, 7.1, 4.0, (0, -20, CENTER_Z)))
    frame = frame.fuse(_ring_y(10.0, 4.6, 4.0, (INTERMEDIATE_X, -20, CENTER_Z)))
    # Slim diagonal supports leave the train and both hand hubs visible.
    for x0, z0, x1 in [(-45, 75, 0), (78, 75, 60)]:
        dx = x1 - x0
        dz = CENTER_Z - z0
        length = math.hypot(dx, dz)
        support = Box(7, 6, length, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        support = _at(support, ((x0 + x1) / 2, 15, (z0 + CENTER_Z) / 2))
        support = support.rotate(Axis.Y, math.degrees(math.atan2(dx, dz)))
        frame = frame.fuse(support)
    frame.label = "base_frame"

    minute_shaft = _y_cylinder(SHAFT_R, 54.0, (0, -7, CENTER_Z)); minute_shaft.label = "minute_shaft"
    intermediate_shaft = _y_cylinder(SHAFT_R, 40.0, (INTERMEDIATE_X, -3, CENTER_Z)); intermediate_shaft.label = "intermediate_shaft"
    hour_sleeve = _ring_y(SLEEVE_OUT_R, SLEEVE_IN_R, 20.0, (0, -18, CENTER_Z)); hour_sleeve.label = "hour_sleeve"

    minute_pinion = _gear("minute_pinion_15t", 15, MODULE, 6.0, (0, 0, CENTER_Z))
    intermediate_gear = _gear("intermediate_gear_45t", 45, MODULE, 6.0, (INTERMEDIATE_X, 0, CENTER_Z))
    intermediate_pinion = _gear("intermediate_pinion_12t", 12, MODULE, 6.0, (INTERMEDIATE_X, -11, CENTER_Z))
    hour_gear = _gear("hour_gear_48t", 48, MODULE, 6.0, (0, -11, CENTER_Z))
    minute_hand = _hand("minute_hand", 50.0, 4.0, 2.4, 18.0, -31.0)
    hour_hand = _hand("hour_hand", 35.0, 6.0, 3.2, -48.0, -26.0)

    return {
        "base_frame": frame,
        "minute_shaft": minute_shaft,
        "minute_pinion_15t": minute_pinion,
        "minute_hand": minute_hand,
        "intermediate_shaft": intermediate_shaft,
        "intermediate_gear_45t": intermediate_gear,
        "intermediate_pinion_12t": intermediate_pinion,
        "hour_sleeve": hour_sleeve,
        "hour_gear_48t": hour_gear,
        "hour_hand": hour_hand,
    }


def build_machine():
    parts = build_parts()
    assembly = Compound(children=list(parts.values()), label="openwork_clock_12to1")
    return assembly


def gen_step():
    return build_machine()


def export_named_parts():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    parts = build_parts()
    for name, shape in parts.items():
        export_stl(shape, str(MESH_DIR / f"{name}.stl"), tolerance=0.08, angular_tolerance=0.12)
    export_step(Compound(children=list(parts.values()), label="openwork_clock_12to1"), str(TASK_ROOT / "openwork_clock_12to1.step"))
    return sorted(parts)


if __name__ == "__main__":
    names = export_named_parts()
    print(f"exported {len(names)} named parts")
    for name in names:
        print(name)
