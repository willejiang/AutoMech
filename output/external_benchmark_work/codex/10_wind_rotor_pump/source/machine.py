"""Independent open wind-rotor reciprocating pump; CAD units millimetres."""
from __future__ import annotations

import math
from pathlib import Path
from build123d import Align, Axis, Box, Compound, Cylinder, Location, export_stl

SHAFT = (-45.0, 0.0, 200.0)
CRANK_RADIUS = 25.0
CROSSHEAD = (0.0, 0.0, 122.540333)
ROD_LENGTH = 80.0
CRANK_PIN = (-20.0, 0.0, 200.0)
ROD_CENTER = (-10.0, 0.0, (200.0 + CROSSHEAD[2]) / 2.0)

POSES = {
    "base": (0.0, 0.0, 5.0),
    "tower_left": (-62.0, 0.0, 105.0),
    "tower_right": (-28.0, 0.0, 105.0),
    "front_bearing": SHAFT,
    "rear_bearing": SHAFT,
    "rotor_shaft_input": SHAFT,
    "wind_rotor": SHAFT,
    "crank_disk": SHAFT,
    "crank_pin": CRANK_PIN,
    "connecting_rod": ROD_CENTER,
    "vertical_crosshead": CROSSHEAD,
    "vertical_guide": (0.0, 0.0, 135.0),
    "pump_rod": (0.0, 0.0, CROSSHEAD[2] - 30.0),
    "piston_output": (0.0, 0.0, CROSSHEAD[2] - 60.0),
}

MECHANISM = {
    "name": "open_wind_rotor_pump",
    "links": [{"name": n} for n in POSES],
    "ports": {
        "shaft_axis": {"link": "rotor_shaft_input", "axis": [0, 1, 0]},
        "crank_pin_port": {"link": "crank_pin", "axis": [0, 1, 0]},
        "rod_big_end": {"link": "connecting_rod", "axis": [0, 1, 0]},
        "rod_small_end": {"link": "connecting_rod", "axis": [0, 1, 0]},
        "crosshead_pin": {"link": "vertical_crosshead", "axis": [0, 1, 0]},
        "guide_axis": {"link": "vertical_guide", "axis": [0, 0, 1]},
    },
    "relations": [
        {"type": "rigid_mount", "parent": "base", "child": "tower_left"},
        {"type": "rigid_mount", "parent": "base", "child": "tower_right"},
        {"type": "rigid_mount", "parent": "base", "child": "vertical_guide"},
        {"type": "running_bearing", "outer": "front_bearing", "inner": "rotor_shaft_input", "radial_clearance_mm": 0.5},
        {"type": "running_bearing", "outer": "rear_bearing", "inner": "rotor_shaft_input", "radial_clearance_mm": 0.5},
        {"type": "press_fit", "outer": "wind_rotor", "inner": "rotor_shaft_input"},
        {"type": "press_fit", "outer": "crank_disk", "inner": "rotor_shaft_input"},
        {"type": "dedicated_pin_fit", "pin": "crank_pin", "rod": "connecting_rod", "radial_clearance_mm": 0.5},
        {"type": "closure", "name": "rod_crank_closure", "a": "rod_big_end", "b": "crank_pin_port", "scale_mm": ROD_LENGTH},
        {"type": "closure", "name": "rod_crosshead_closure", "a": "rod_small_end", "b": "crosshead_pin", "scale_mm": ROD_LENGTH},
        {"type": "rigid_carrying", "parent": "vertical_crosshead", "child": "pump_rod"},
        {"type": "rigid_carrying", "parent": "pump_rod", "child": "piston_output"},
    ],
    "motion_joints": [
        {"name": "rotor_shaft_input_hinge", "parent": "base", "child": "rotor_shaft_input", "axis": [0, 1, 0], "driver": True},
        {"name": "crank_pin_hinge", "parent": "crank_pin", "child": "connecting_rod", "axis": [0, 1, 0], "driver": False},
        {"name": "vertical_crosshead_slide", "parent": "vertical_guide", "child": "vertical_crosshead", "axis": [0, 0, 1], "driver": False},
    ],
    "transmissions": [], "planetary_stages": [], "mesh_pairs": [],
    "driver": {"joint": "rotor_shaft_input_hinge", "source": "wind_rotor", "mode": "imposed_velocity"},
    "output": {"joint": "vertical_crosshead_slide", "link": "piston_output"},
    "watch_links": ["wind_rotor", "crank_disk", "crank_pin", "connecting_rod", "vertical_crosshead", "pump_rod", "piston_output"],
}

def cyl_y(r, length):
    return Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.X, 90)

def rod_between_local(dx, dz, width=14.0, depth=10.0):
    length = math.hypot(dx, dz)
    angle = -math.degrees(math.atan2(dz, dx))
    bar = Box(length, depth, width, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.Y, angle)
    return bar.fuse(cyl_y(width / 2, depth).translate((-dx/2, 0, -dz/2)), cyl_y(width / 2, depth).translate((dx/2, 0, dz/2)))

def rotor():
    hub = cyl_y(15, 8)
    blades = []
    for angle in (0, 90, 180, 270):
        blade = Box(65, 6, 16, align=(Align.MIN, Align.CENTER, Align.CENTER)).translate((12, 0, 0)).rotate(Axis.Y, angle)
        blades.append(blade)
    return hub.fuse(*blades)

def build_local_parts():
    parts = {
        "base": Box(190, 120, 10, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "tower_left": Box(14, 18, 190, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "tower_right": Box(14, 18, 190, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "front_bearing": cyl_y(12, 10).translate((0, -25, 0)).cut(cyl_y(7, 12).translate((0, -25, 0))),
        "rear_bearing": cyl_y(12, 10).translate((0, 25, 0)).cut(cyl_y(7, 12).translate((0, 25, 0))),
        "rotor_shaft_input": cyl_y(6, 90),
        "wind_rotor": rotor().translate((0, -38, 0)),
        "crank_disk": cyl_y(32, 8).translate((0, 36, 0)),
        "crank_pin": cyl_y(5, 18),
        "connecting_rod": rod_between_local(CROSSHEAD[0]-CRANK_PIN[0], CROSSHEAD[2]-CRANK_PIN[2]),
        "vertical_crosshead": Box(28, 28, 24, align=(Align.CENTER, Align.CENTER, Align.CENTER)).fuse(cyl_y(6, 36)),
        "vertical_guide": Box(10, 10, 170, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((-23, 0, 0)).fuse(Box(10, 10, 170, align=(Align.CENTER, Align.CENTER, Align.CENTER)).translate((23, 0, 0))),
        "pump_rod": Cylinder(5, 50, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "piston_output": Cylinder(18, 12, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
    }
    for name, shape in parts.items(): shape.label = name
    return parts

def build_machine():
    children = []
    for name, shape in build_local_parts().items():
        placed = shape.moved(Location(POSES[name])); placed.label = name; children.append(placed)
    result = Compound(children=children); result.label = MECHANISM["name"]
    return result

def gen_step(): return build_machine()

def export_named_stls(directory):
    out = Path(directory); out.mkdir(parents=True, exist_ok=True)
    for name, shape in build_local_parts().items():
        export_stl(shape, out / f"{name}.stl", tolerance=.15, angular_tolerance=.15)

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(); p.add_argument("--export-stls"); a = p.parse_args()
    assert build_machine().solids()
    if a.export_stls: export_named_stls(a.export_stls)
