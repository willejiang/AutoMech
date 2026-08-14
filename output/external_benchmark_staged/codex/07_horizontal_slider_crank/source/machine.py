"""Open horizontal hand-cranked slider-crank. CAD units are millimetres.

Coordinate convention: X is slider travel, Y is crankshaft axis/depth, and Z
is vertical.  Each movable body stays a separate labeled solid.
"""
from __future__ import annotations

import math
from pathlib import Path
from build123d import Align, Axis, Box, Compound, Cylinder, Location, export_stl

CRANK_AXIS = (0.0, 0.0, 72.0)
CRANK_RADIUS = 28.0
ROD_LENGTH = 112.0
SLIDER_X = math.sqrt(ROD_LENGTH**2 - CRANK_RADIUS**2)
MOTION_PLANE_Y = 8.0
ROD_ANGLE_DEG = math.degrees(math.atan2(CRANK_RADIUS, SLIDER_X))
POSES = {
    "base": (55.0, 0.0, 4.0),
    "left_pedestal": (0.0, 0.0, 39.0),
    "right_pedestal": (0.0, 0.0, 39.0),
    "front_bearing": CRANK_AXIS,
    "rear_bearing": CRANK_AXIS,
    "crankshaft_input": CRANK_AXIS,
    "crank_web": CRANK_AXIS,
    "crank_pin": (0.0, MOTION_PLANE_Y, CRANK_AXIS[2] + CRANK_RADIUS),
    "connecting_rod": (SLIDER_X / 2.0, MOTION_PLANE_Y, CRANK_AXIS[2] + CRANK_RADIUS / 2.0),
    "horizontal_slider": (SLIDER_X, MOTION_PLANE_Y, CRANK_AXIS[2]),
    "horizontal_guide": (118.0, MOTION_PLANE_Y, CRANK_AXIS[2]),
    "hand_crank": CRANK_AXIS,
}

MECHANISM = {
    "name": "open_horizontal_slider_crank",
    "links": [{"name": n} for n in POSES],
    "ports": {
        "crank_axis": {"link": "crankshaft_input", "axis": [0, 1, 0]},
        "crank_pin_axis": {"link": "crank_pin", "axis": [0, 1, 0]},
        "rod_big_end": {"link": "connecting_rod", "axis": [0, 1, 0]},
        "rod_small_end": {"link": "connecting_rod", "axis": [0, 1, 0]},
        "slider_pin_axis": {"link": "horizontal_slider", "axis": [0, 1, 0]},
        "guide_axis": {"link": "horizontal_guide", "axis": [1, 0, 0]},
    },
    "relations": [
        {"type": "rigid_mount", "parent": "base", "child": "left_pedestal"},
        {"type": "rigid_mount", "parent": "base", "child": "right_pedestal"},
        {"type": "rigid_mount", "parent": "base", "child": "horizontal_guide"},
        {"type": "running_bearing", "outer": "front_bearing", "inner": "crankshaft_input", "radial_clearance_mm": 0.5},
        {"type": "running_bearing", "outer": "rear_bearing", "inner": "crankshaft_input", "radial_clearance_mm": 0.5},
        {"type": "press_fit", "outer": "crank_web", "inner": "crankshaft_input"},
        {"type": "press_fit", "outer": "hand_crank", "inner": "crankshaft_input"},
        {"type": "dedicated_pin_fit", "pin": "crank_pin", "rod": "connecting_rod", "radial_clearance_mm": 0.5},
        {"type": "revolute", "name": "slider_pin_revolute", "parent": "horizontal_slider", "child": "connecting_rod", "axis": [0, 1, 0]},
        {"type": "closure", "name": "rod_crank_pin_closure", "a": "rod_big_end", "b": "crank_pin_axis", "scale_mm": 8.0},
    ],
    "motion_joints": [
        {"name": "crankshaft_input_hinge", "parent": "base", "child": "crankshaft_input", "axis": [0, 1, 0], "driver": True},
        {"name": "crank_pin_hinge", "parent": "crank_pin", "child": "connecting_rod", "axis": [0, 1, 0], "driver": False},
        {"name": "horizontal_slider_slide", "parent": "horizontal_guide", "child": "horizontal_slider", "axis": [1, 0, 0], "driver": False},
    ],
    "transmissions": [],
    "planetary_stages": [],
    "mesh_pairs": [],
    "driver": {"joint": "crankshaft_input_hinge", "source": "hand_crank", "mode": "finite_effort_motor", "effort_limit_nm": 25.0},
    "output": {"joint": "horizontal_slider_slide", "link": "horizontal_slider"},
    "watch_links": ["crank_web", "crank_pin", "connecting_rod", "horizontal_slider"],
}

def cyl_y(radius: float, length: float):
    return Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.X, 90)

def rod_between_xz(a, b, width=14.0, depth=8.0):
    ax, az = a; bx, bz = b
    length = math.hypot(bx-ax, bz-az)
    ang = -math.degrees(math.atan2(bz-az, bx-ax))
    bar = Box(length, depth, width, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.Y, ang)
    return bar.translate(((ax+bx)/2, 0, (az+bz)/2)).fuse(cyl_y(width/2, depth).translate((ax,0,az)), cyl_y(width/2, depth).translate((bx,0,bz)))

def build_local_parts():
    half_dx = SLIDER_X / 2.0
    half_dz = CRANK_RADIUS / 2.0
    rod = Box(ROD_LENGTH, 8, 14, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.Y, ROD_ANGLE_DEG)
    rod = rod.fuse(cyl_y(7, 8).translate((-half_dx,0,half_dz)), cyl_y(7,8).translate((half_dx,0,-half_dz)))
    parts = {
        "base": Box(260, 110, 8, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "left_pedestal": Box(16, 14, 70, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((0, -28, 0)),
        "right_pedestal": Box(16, 14, 70, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((0, 28, 0)),
        "front_bearing": cyl_y(12, 10).translate((0, -28, 0)).cut(cyl_y(7, 12).translate((0, -28, 0))),
        "rear_bearing": cyl_y(12, 10).translate((0, 28, 0)).cut(cyl_y(7, 12).translate((0, 28, 0))),
        "crankshaft_input": cyl_y(6, 100),
        "crank_web": cyl_y(14, 8).fuse(Box(12, 8, CRANK_RADIUS, align=(Align.CENTER, Align.CENTER, Align.MIN))),
        "crank_pin": cyl_y(5, 20),
        "connecting_rod": rod,
        "horizontal_slider": Box(30, 28, 24, align=(Align.CENTER, Align.CENTER, Align.CENTER)).fuse(cyl_y(6, 38)),
        "horizontal_guide": Box(150, 10, 8, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((0,-19,-18)).fuse(Box(150,10,8,align=(Align.CENTER,Align.CENTER,Align.CENTER)).translate((0,19,-18))),
        "hand_crank": cyl_y(11, 8).translate((0, -48, 0)).fuse(Box(9, 7, 38, align=(Align.CENTER, Align.CENTER, Align.MIN)).translate((0,-52,0)), cyl_y(6, 22).translate((0,-63,38))),
    }
    for name, shape in parts.items(): shape.label = name
    return parts

def build_machine():
    children=[]
    for name, shape in build_local_parts().items():
        placed=shape.moved(Location(POSES[name])); placed.label=name; children.append(placed)
    result=Compound(children=children); result.label=MECHANISM["name"]
    return result

def gen_step(): return build_machine()

def export_named_stls(directory):
    out=Path(directory); out.mkdir(parents=True, exist_ok=True)
    for name, shape in build_local_parts().items():
        export_stl(shape, out/f"{name}.stl", tolerance=.12, angular_tolerance=.12)

if __name__ == "__main__":
    import argparse
    parser=argparse.ArgumentParser(); parser.add_argument("--export-stls"); args=parser.parse_args()
    assert build_machine().solids()
    if args.export_stls: export_named_stls(args.export_stls)
