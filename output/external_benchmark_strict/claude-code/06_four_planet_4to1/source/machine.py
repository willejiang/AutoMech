"""Open-frame four-planet 4:1 planetary reducer benchmark candidate.

CAD units are millimetres.  The planetary axis is +Z; the gear plane is XY.
The fixed 72-tooth internal ring and 24-tooth sun/planets give
carrier/sun = Ns/(Ns+Nr) = 24/(24+72) = 1/4.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from build123d import Align, Box, Compound, Cylinder, Location, export_stl

NS = 24
NP = 24
NR = 72
MODULE = 2.0
PLANET_RADIUS = MODULE * (NS + NP) / 2.0  # 48 mm
GEAR_THICKNESS = 8.0
CARRIER_Z = -8.0
PLANET_ANGLES_DEG = (0.0, 90.0, 180.0, 270.0)


def _compound(name: str, shapes):
    obj = Compound(children=list(shapes))
    obj.label = name
    return obj


def _spur_gear(name: str, teeth: int, pitch_radius: float, bore_radius: float = 0.0):
    root_radius = pitch_radius - 1.5
    tip_radius = pitch_radius + 1.5
    core = Cylinder(root_radius, GEAR_THICKNESS, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    if bore_radius > 0:
        core = core - Cylinder(bore_radius, GEAR_THICKNESS + 2.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    tooth_width = 0.48 * (2.0 * math.pi * pitch_radius / teeth)
    teeth_shapes = []
    for i in range(teeth):
        a = 360.0 * i / teeth
        tooth = Box(
            tip_radius - root_radius + 0.8,
            tooth_width,
            GEAR_THICKNESS,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).located(Location(((root_radius + tip_radius) / 2.0 - 0.2, 0, 0), (0, 0, a)))
        teeth_shapes.append(tooth)
    return _compound(name, [core, *teeth_shapes])


def _ring_gear():
    outer_radius = 84.0
    inner_root_radius = 74.0
    inner_tip_radius = 68.5
    annulus = (
        Cylinder(outer_radius, GEAR_THICKNESS, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        - Cylinder(inner_root_radius, GEAR_THICKNESS + 2.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    )
    tooth_width = 0.45 * (2.0 * math.pi * (inner_root_radius - 1.0) / NR)
    teeth_shapes = []
    for i in range(NR):
        a = 360.0 * i / NR
        tooth = Box(
            inner_root_radius - inner_tip_radius + 1.0,
            tooth_width,
            GEAR_THICKNESS,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).located(Location(((inner_root_radius + inner_tip_radius) / 2.0 + 0.2, 0, 0), (0, 0, a)))
        teeth_shapes.append(tooth)
    # Two exposed feet rigidly join the ring to the base without covering the gears.
    feet = [
        Box(14, 40, 8, align=(Align.CENTER, Align.CENTER, Align.CENTER)).located(Location((x, -86, 0)))
        for x in (-62.0, 62.0)
    ]
    return _compound("fixed_ring", [annulus, *teeth_shapes, *feet])


def _carrier():
    shapes = [
        Cylinder(11.0, 4.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        - Cylinder(6.5, 6.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        Cylinder(8.0, 14.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).located(Location((0, 0, -7.0))),
    ]
    for a in PLANET_ANGLES_DEG:
        arm = Box(PLANET_RADIUS, 8.0, 4.0, align=(Align.MIN, Align.CENTER, Align.CENTER)).located(
            Location((0, 0, 0), (0, 0, a))
        )
        boss = Cylinder(8.0, 4.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).located(
            Location((PLANET_RADIUS * math.cos(math.radians(a)), PLANET_RADIUS * math.sin(math.radians(a)), 0))
        )
        shapes.extend([arm, boss])
    return _compound("carrier_output", shapes)


def _pin(name: str):
    pin = Cylinder(3.0, 20.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    pin.label = name
    return pin


def _sun():
    gear = _spur_gear("sun_gear", NS, MODULE * NS / 2.0, bore_radius=0.0)
    shaft = Cylinder(5.0, 30.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).located(Location((0, 0, 7.0)))
    return _compound("sun_input", [gear, shaft])


def _hand_crank():
    hub = Cylinder(7.5, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).located(Location((0, 0, 18.5)))
    arm = Box(38.0, 6.0, 5.0, align=(Align.MIN, Align.CENTER, Align.CENTER)).located(Location((0, 0, 18.5)))
    handle = Cylinder(5.0, 22.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).located(Location((35.0, 0, 16.0)))
    return _compound("hand_crank", [hub, arm, handle])


def _base():
    shapes = [
        Box(190, 30, 12, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        Box(36, 44, 6, align=(Align.CENTER, Align.CENTER, Align.CENTER)).located(Location((-78, 0, -6))),
        Box(36, 44, 6, align=(Align.CENTER, Align.CENTER, Align.CENTER)).located(Location((78, 0, -6))),
        Cylinder(12.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).located(Location((0, 100, -8))),
    ]
    return _compound("base", shapes)


def make_parts():
    parts = {
        "base": _base(),
        "fixed_ring": _ring_gear(),
        "sun_input": _sun(),
        "carrier_output": _carrier(),
        "hand_crank": _hand_crank(),
    }
    for i in range(4):
        parts[f"planet_gear_{i}"] = _spur_gear(f"planet_gear_{i}", NP, MODULE * NP / 2.0, bore_radius=4.2)
        parts[f"planet_pin_{i}"] = _pin(f"planet_pin_{i}")
    return parts


def final_placements():
    placements = {
        "base": Location((0, -100, -8)),
        "fixed_ring": Location((0, 0, 0)),
        "sun_input": Location((0, 0, 0)),
        "carrier_output": Location((0, 0, CARRIER_Z)),
        "hand_crank": Location((0, 0, 0)),
    }
    for i, a in enumerate(PLANET_ANGLES_DEG):
        x = PLANET_RADIUS * math.cos(math.radians(a))
        y = PLANET_RADIUS * math.sin(math.radians(a))
        placements[f"planet_gear_{i}"] = Location((x, y, 0), (0, 0, a * -3.0))
        placements[f"planet_pin_{i}"] = Location((x, y, 0))
    return placements


def build_machine():
    parts = make_parts()
    placements = final_placements()
    children = []
    for name, shape in parts.items():
        placed = shape.located(placements[name])
        placed.label = name
        children.append(placed)
    assembly = Compound(children=children)
    assembly.label = "four_planet_4to1_open_reducer"
    return assembly


def gen_step():
    return build_machine()


def export_named_meshes(mesh_dir: Path | str | None = None):
    if mesh_dir is None:
        mesh_dir = Path(__file__).resolve().parents[1] / "meshes"
    mesh_dir = Path(mesh_dir)
    mesh_dir.mkdir(parents=True, exist_ok=True)
    parts = make_parts()
    outputs = {}
    for name, shape in parts.items():
        path = mesh_dir / f"{name}.stl"
        export_stl(shape, str(path), tolerance=0.08, angular_tolerance=0.15)
        outputs[name] = str(path)
    return outputs


MECHANISM = {
    "name": "four_planet_4to1_open_reducer",
    "units": {"cad_length": "mm", "assembly_pose_length": "m", "angle": "rad"},
    "parameters": {"sun_teeth": NS, "planet_teeth": NP, "ring_teeth": NR, "module_mm": MODULE,
                   "planet_center_radius_mm": PLANET_RADIUS, "exact_carrier_to_sun_ratio": 0.25},
    "links": [
        {"name": "base", "dof": "fixed", "rigid_mount": "world"},
        {"name": "fixed_ring", "dof": "fixed", "rigid_mount": "base"},
        {"name": "sun_input", "dof": "revolute", "axis": [0, 0, 1], "driver": True},
        {"name": "carrier_output", "dof": "revolute", "axis": [0, 0, 1], "driver": False},
        {"name": "hand_crank", "dof": "fixed", "rigid_mount": "sun_input"},
        *[{"name": f"planet_gear_{i}", "dof": "revolute", "axis": [0, 0, 1], "driver": False,
           "parent": "carrier_output"} for i in range(4)],
        *[{"name": f"planet_pin_{i}", "dof": "fixed", "rigid_mount": "carrier_output"} for i in range(4)],
    ],
    "ports": {
        "base": ["ring_mount", "sun_rear_bearing", "carrier_rear_bearing"],
        "fixed_ring": ["ring_mount", *[f"internal_mesh_{i}" for i in range(4)]],
        "sun_input": ["sun_hinge", "crank_mount", *[f"sun_mesh_{i}" for i in range(4)]],
        "carrier_output": ["carrier_hinge", *[f"pin_mount_{i}" for i in range(4)]],
        "hand_crank": ["crank_mount"],
        **{f"planet_gear_{i}": [f"planet_hinge_{i}", f"sun_mesh_{i}", f"internal_mesh_{i}"] for i in range(4)},
        **{f"planet_pin_{i}": [f"pin_mount_{i}", f"planet_hinge_{i}"] for i in range(4)},
    },
    "relations": [
        {"kind": "rigid_mount", "a": "base", "b": "fixed_ring", "allows_overlap": True},
        {"kind": "running_bearing", "a": "base", "b": "sun_input", "joint": "sun_hinge"},
        {"kind": "running_bearing", "a": "base", "b": "carrier_output", "joint": "carrier_hinge"},
        {"kind": "rigid_mount", "a": "sun_input", "b": "hand_crank", "allows_overlap": True},
        *[{"kind": "dedicated_pin_fit", "a": f"carrier_output", "b": f"planet_pin_{i}",
           "allows_overlap": True} for i in range(4)],
        *[{"kind": "running_bearing", "a": f"planet_pin_{i}", "b": f"planet_gear_{i}",
           "joint": f"planet_hinge_{i}"} for i in range(4)],
    ],
    "motion_joints": [
        {"name": "sun_hinge", "kind": "revolute", "parent": "base", "child": "sun_input", "axis": [0, 0, 1]},
        {"name": "carrier_hinge", "kind": "revolute", "parent": "base", "child": "carrier_output", "axis": [0, 0, 1]},
        *[{"name": f"planet_hinge_{i}", "kind": "revolute", "parent": "carrier_output",
           "child": f"planet_gear_{i}", "axis": [0, 0, 1], "pin": f"planet_pin_{i}"} for i in range(4)],
    ],
    "transmissions": [
        {"name": "ideal_carrier_from_sun", "kind": "ideal_planetary", "driven": "carrier_hinge",
         "driving": "sun_hinge", "ratio": 0.25},
        *[{"name": f"ideal_planet_spin_{i}", "kind": "ideal_planetary_local_spin",
           "driven": f"planet_hinge_{i}", "driving": "sun_hinge", "ratio": -0.75} for i in range(4)],
    ],
    "planetary_stages": [{
        "name": "main_stage", "sun": "sun_input", "ring": "fixed_ring", "carrier": "carrier_output",
        "planets": [f"planet_gear_{i}" for i in range(4)], "pins": [f"planet_pin_{i}" for i in range(4)],
        "sun_teeth": NS, "planet_teeth": NP, "ring_teeth": NR, "ring_fixed": True,
        "carrier_to_sun_ratio": 0.25, "planet_spacing_deg": 90.0,
    }],
    "mesh_pairs": [
        *[{"name": f"sun_planet_{i}", "a": "sun_input", "b": f"planet_gear_{i}",
           "kind": "external_gear_mesh", "ideal": True, "allows_overlap": True} for i in range(4)],
        *[{"name": f"ring_planet_{i}", "a": "fixed_ring", "b": f"planet_gear_{i}",
           "kind": "internal_gear_mesh", "ideal": True, "allows_overlap": True} for i in range(4)],
    ],
    "driver": {"joint": "sun_hinge", "kind": "position_servo", "minimum_travel_rad": 12.0},
    "output": {"link": "carrier_output", "joint": "carrier_hinge", "expected_ratio": 0.25},
    "watch_links": ["sun_input", "carrier_output", *[f"planet_gear_{i}" for i in range(4)]],
}


if __name__ == "__main__":
    outputs = export_named_meshes()
    machine = build_machine()
    print(json.dumps({"mechanism": MECHANISM["name"], "parts": sorted(outputs),
                      "part_count": len(outputs), "assembly_solids": len(machine.solids())}, indent=2))
