"""Open hand-cranked pumpjack demonstration mechanism.

CAD units are millimetres. X is horizontal, Y is the shared pin/shaft axis,
and Z is vertical. Every independently moving member is a separate labeled
solid and the exposed linkage is shown in a valid connected static pose.
"""
from __future__ import annotations

import math
from pathlib import Path
from build123d import Align, Axis, Box, Compound, Cylinder, Location, export_stl

CRANK_CENTER = (-80.0, 0.0, 70.0)
CRANK_RADIUS = 20.0
BEAM_PIVOT = (0.0, 0.0, 145.0)
BEAM_REAR_ARM = 60.0
BEAM_FRONT_ARM = 72.0
PITMAN_LENGTH = 75.0
OUTPUT_X = BEAM_FRONT_ARM

POSES = {
    "base": (0.0, 0.0, 5.0),
    "crank_pedestal_front": (-80.0, -28.0, 37.5),
    "crank_pedestal_rear": (-80.0, 28.0, 37.5),
    "crank_bearing_front": CRANK_CENTER,
    "crank_bearing_rear": CRANK_CENTER,
    "beam_support_left": (0.0, -25.0, 76.0),
    "beam_support_right": (0.0, 25.0, 76.0),
    "beam_pivot": BEAM_PIVOT,
    "vertical_guide": (OUTPUT_X, 0.0, 105.0),
    "crankshaft_input": CRANK_CENTER,
    "hand_crank": CRANK_CENTER,
    "crank_disk": CRANK_CENTER,
    "crank_pin": (-60.0, 0.0, 70.0),
    "pitman_rod": (-60.0, -12.0, 107.5),
    "walking_beam": BEAM_PIVOT,
    "polished_rod_output": (OUTPUT_X, 0.0, 92.5),
}

MECHANISM = {
    "name": "open_hand_cranked_pumpjack",
    "links": [{"name": name} for name in POSES],
    "ports": {
        "crank_axis": {"link": "crankshaft_input", "axis": [0, 1, 0]},
        "crank_pin_axis": {"link": "crank_pin", "axis": [0, 1, 0]},
        "pitman_big_end": {"link": "pitman_rod", "axis": [0, 1, 0]},
        "pitman_beam_end": {"link": "pitman_rod", "axis": [0, 1, 0]},
        "beam_rear_pin": {"link": "walking_beam", "axis": [0, 1, 0]},
        "beam_pivot_axis": {"link": "walking_beam", "axis": [0, 1, 0]},
        "beam_output_pin": {"link": "walking_beam", "axis": [0, 1, 0]},
        "polished_rod_pin": {"link": "polished_rod_output", "axis": [0, 1, 0]},
        "guide_axis": {"link": "vertical_guide", "axis": [0, 0, 1]},
    },
    "relations": [
        {"type": "rigid_mount", "parent": "base", "child": "crank_pedestal_front"},
        {"type": "rigid_mount", "parent": "base", "child": "crank_pedestal_rear"},
        {"type": "rigid_mount", "parent": "base", "child": "beam_support_left"},
        {"type": "rigid_mount", "parent": "base", "child": "beam_support_right"},
        {"type": "rigid_mount", "parent": "base", "child": "vertical_guide"},
        {"type": "running_bearing", "outer": "crank_bearing_front", "inner": "crankshaft_input", "radial_clearance_mm": 0.5},
        {"type": "running_bearing", "outer": "crank_bearing_rear", "inner": "crankshaft_input", "radial_clearance_mm": 0.5},
        {"type": "press_fit", "outer": "crank_disk", "inner": "crankshaft_input"},
        {"type": "press_fit", "outer": "hand_crank", "inner": "crankshaft_input"},
        {"type": "dedicated_pin_fit", "pin": "crank_pin", "rod": "pitman_rod", "radial_clearance_mm": 0.5},
        {"type": "revolute", "name": "pitman_beam_pin", "parent": "walking_beam", "child": "pitman_rod", "axis": [0, 1, 0]},
        {"type": "revolute", "name": "beam_output_pin_revolute", "parent": "walking_beam", "child": "polished_rod_output", "axis": [0, 1, 0]},
        {"type": "closure", "name": "pitman_crank_closure", "a": "pitman_big_end", "b": "crank_pin_axis", "scale_mm": 8.0},
        {"type": "closure", "name": "pitman_beam_closure", "a": "pitman_beam_end", "b": "beam_rear_pin", "scale_mm": 8.0},
        {"type": "closure", "name": "beam_output_closure", "a": "beam_output_pin", "b": "polished_rod_pin", "scale_mm": 8.0},
    ],
    "motion_joints": [
        {"name": "crankshaft_input_hinge", "parent": "base", "child": "crankshaft_input", "kind": "spin", "axis": [0, 1, 0], "driver": True},
        {"name": "crank_pin_hinge", "parent": "crank_pin", "child": "pitman_rod", "kind": "spin", "axis": [0, 1, 0], "driver": False},
        {"name": "beam_pivot_hinge", "parent": "base", "child": "walking_beam", "kind": "spin", "axis": [0, 1, 0], "driver": False},
        {"name": "polished_rod_slide", "parent": "vertical_guide", "child": "polished_rod_output", "kind": "slide", "axis": [0, 0, 1], "driver": False},
    ],
    "transmissions": [], "planetary_stages": [], "mesh_pairs": [],
    "driver": {"joint": "crankshaft_input_hinge", "source": "hand_crank", "mode": "finite_effort_motor", "effort_limit_nm": 30.0},
    "output": {"joint": "polished_rod_slide", "link": "polished_rod_output"},
    "watch_links": ["crank_disk", "crank_pin", "pitman_rod", "walking_beam", "polished_rod_output"],
}

def cyl_y(radius: float, length: float):
    return Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.X, 90)

def rod_xz(a, b, width=12.0, depth=9.0):
    ax, az = a; bx, bz = b
    length = math.hypot(bx-ax, bz-az)
    angle = -math.degrees(math.atan2(bz-az, bx-ax))
    bar = Box(length, depth, width, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.Y, angle)
    bar = bar.translate(((ax+bx)/2, 0, (az+bz)/2))
    return bar.fuse(cyl_y(width/2, depth).translate((ax,0,az)), cyl_y(width/2, depth).translate((bx,0,bz)))

def build_local_parts():
    support = Box(14, 14, 138, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    bearing = cyl_y(12, 10).cut(cyl_y(7, 12))
    beam = rod_xz((-BEAM_REAR_ARM, 0), (BEAM_FRONT_ARM, 0), width=15, depth=12)
    # The selected static pose has the crank pin directly below the beam rear
    # pin.  Model the local pitman vertically so both authored pin centers are
    # coincident without relying on a post-export rotation.
    pitman = rod_xz((0, -PITMAN_LENGTH/2), (0, PITMAN_LENGTH/2), width=12, depth=9)
    guide = Box(12, 8, 105, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((-13,0,0)).fuse(
        Box(12, 8, 105, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((13,0,0)))
    parts = {
        "base": Box(250, 120, 10, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "crank_pedestal_front": Box(18, 14, 65, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "crank_pedestal_rear": Box(18, 14, 65, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "crank_bearing_front": bearing,
        "crank_bearing_rear": bearing,
        "beam_support_left": support,
        "beam_support_right": support,
        "beam_pivot": cyl_y(8, 64),
        "vertical_guide": guide,
        "crankshaft_input": cyl_y(6, 88),
        "hand_crank": cyl_y(11, 8).translate((0,-48,0)).fuse(Box(9,7,38,align=(Align.CENTER,Align.CENTER,Align.MIN)).translate((0,-52,0)), cyl_y(6,22).translate((0,-63,38))),
        "crank_disk": cyl_y(28, 10),
        "crank_pin": cyl_y(5, 24),
        "pitman_rod": pitman,
        "walking_beam": beam,
        "polished_rod_output": Cylinder(5, 105, align=(Align.CENTER,Align.CENTER,Align.CENTER)).fuse(cyl_y(7,14).translate((0,0,52.5))),
    }
    for name, shape in parts.items(): shape.label = name
    return parts

def build_machine():
    children = []
    for name, shape in build_local_parts().items():
        placed = shape.moved(Location(POSES[name])); placed.label = name; children.append(placed)
    out = Compound(children=children); out.label = MECHANISM["name"]; return out

def gen_step(): return build_machine()

def export_named_stls(directory):
    out = Path(directory); out.mkdir(parents=True, exist_ok=True)
    for name, shape in build_local_parts().items():
        export_stl(shape, out / f"{name}.stl", tolerance=.12, angular_tolerance=.12)

if __name__ == "__main__":
    import argparse
    p=argparse.ArgumentParser(); p.add_argument("--export-stls"); args=p.parse_args()
    assert build_machine().solids()
    if args.export_stls: export_named_stls(args.export_stls)
