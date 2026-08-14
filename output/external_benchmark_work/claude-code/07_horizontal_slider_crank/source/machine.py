"""Parametric open horizontal slider-crank benchmark machine.

CAD units are millimetres.  The mechanism lies in the XZ plane, the
crankshaft/pin axes are +Y, and the slider travels along +X.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Dict

from build123d import Align, Axis, Box, Compound, Cylinder, Location, export_stl

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "meshes"

CRANK_CENTER = (0.0, 0.0, 70.0)
CRANK_RADIUS = 25.0
ROD_LENGTH = 105.0
SLIDER_X0 = CRANK_RADIUS + ROD_LENGTH
ROD_PLANE_Y = -18.0

MECHANISM = {
    "name": "open_horizontal_slider_crank",
    "units": {"cad_length": "mm", "assembly_pose_length": "m", "angle": "rad"},
    "links": [
        {"name": "base", "dof": "fixed", "mesh": "meshes/base.stl", "rigid_mount": "world"},
        {"name": "bearing_support_front", "dof": "fixed", "mesh": "meshes/bearing_support_front.stl", "rigid_mount": "base"},
        {"name": "bearing_support_rear", "dof": "fixed", "mesh": "meshes/bearing_support_rear.stl", "rigid_mount": "base"},
        {"name": "horizontal_guide", "dof": "fixed", "mesh": "meshes/horizontal_guide.stl", "rigid_mount": "base", "axis": [1, 0, 0]},
        {"name": "crankshaft", "dof": "revolute", "mesh": "meshes/crankshaft.stl", "axis": [0, 1, 0], "driver": True},
        {"name": "crank_web", "dof": "rigid", "mesh": "meshes/crank_web.stl", "rigid_mount": "crankshaft"},
        {"name": "crank_pin", "dof": "rigid", "mesh": "meshes/crank_pin.stl", "rigid_mount": "crankshaft"},
        {"name": "hand_crank", "dof": "rigid", "mesh": "meshes/hand_crank.stl", "rigid_mount": "crankshaft"},
        {"name": "connecting_rod", "dof": "revolute", "mesh": "meshes/connecting_rod.stl", "axis": [0, 1, 0]},
        {"name": "horizontal_slider", "dof": "slide", "mesh": "meshes/horizontal_slider.stl", "axis": [1, 0, 0], "output": True},
        {"name": "slider_pin", "dof": "rigid", "mesh": "meshes/slider_pin.stl", "rigid_mount": "horizontal_slider"},
    ],
    "ports_by_link": {
        "base": {"crank_axis": {"point_mm": [0, 0, 70], "axis": [0, 1, 0]}, "guide_axis": {"point_mm": [130, -18, 70], "axis": [1, 0, 0]}},
        "crankshaft": {"axis": {"point_mm": [0, 0, 0], "axis": [0, 1, 0]}, "eccentric": {"point_mm": [25, -18, 0], "axis": [0, 1, 0]}},
        "connecting_rod": {"big_end": {"point_mm": [0, 0, 0], "axis": [0, 1, 0]}, "small_end": {"point_mm": [105, 0, 0], "axis": [0, 1, 0]}},
        "horizontal_slider": {"pin": {"point_mm": [0, 0, 0], "axis": [0, 1, 0]}, "guide": {"point_mm": [0, 0, 0], "axis": [1, 0, 0]}},
    },
    "relations": [
        {"name": "base_support_mount", "kind": "rigid_mount", "parent": "base", "child": "bearing_support_front"},
        {"name": "base_support_mount_rear", "kind": "rigid_mount", "parent": "base", "child": "bearing_support_rear"},
        {"name": "guide_mount", "kind": "rigid_mount", "parent": "base", "child": "horizontal_guide"},
        {"name": "front_running_bearing", "kind": "running_bearing", "outer": "bearing_support_front", "inner": "crankshaft", "axis": [0, 1, 0], "clearance_mm": 0.5},
        {"name": "rear_running_bearing", "kind": "running_bearing", "outer": "bearing_support_rear", "inner": "crankshaft", "axis": [0, 1, 0], "clearance_mm": 0.5},
        {"name": "web_press_fit", "kind": "press_fit", "parent": "crankshaft", "child": "crank_web"},
        {"name": "pin_press_fit", "kind": "press_fit", "parent": "crankshaft", "child": "crank_pin"},
        {"name": "hand_crank_press_fit", "kind": "press_fit", "parent": "crankshaft", "child": "hand_crank"},
        {"name": "big_end_pin", "kind": "pin_fit", "pin": "crank_pin", "member": "connecting_rod", "port": "big_end", "clearance_mm": 0.5},
        {"name": "small_end_pin", "kind": "pin_fit", "pin": "slider_pin", "member": "connecting_rod", "port": "small_end", "clearance_mm": 0.5},
        {"name": "rod_slider_closure", "kind": "point_closure", "a": "connecting_rod.small_end", "b": "horizontal_slider.pin", "scale_mm": 105.0},
    ],
    "motion_joints": [
        {"name": "crankshaft_hinge", "kind": "revolute", "parent": "base", "child": "crankshaft", "axis": [0, 1, 0], "port": "crank_axis", "limit": None},
        {"name": "rod_big_end_hinge", "kind": "revolute", "parent": "crankshaft", "child": "connecting_rod", "axis": [0, 1, 0], "port": "eccentric"},
        {"name": "slider_slide", "kind": "slide", "parent": "base", "child": "horizontal_slider", "axis": [1, 0, 0], "range_m": [-0.060, 0.010]},
    ],
    "closures": [{"name": "rod_slider_connect", "kind": "connect", "body1": "connecting_rod", "anchor1_mm": [105, 0, 0], "body2": "horizontal_slider", "anchor2_mm": [0, 0, 0]}],
    "transmissions": [],
    "planetary_stages": [],
    "mesh_pairs": [],
    "driver": {"joint": "crankshaft_hinge", "mode": "finite_effort_motor", "max_effort_Nm": 1.2, "direct_qpos_forbidden": True},
    "output": {"joint": "slider_slide", "link": "horizontal_slider", "kind": "linear", "axis": [1, 0, 0]},
    "output_link": "horizontal_slider",
    "watch_links": ["crankshaft", "crank_web", "crank_pin", "connecting_rod", "horizontal_slider", "slider_pin"],
}


def _cylinder_y(radius: float, length: float):
    return Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.CENTER)).rotate(Axis.X, 90)


def _bearing_support():
    post = Box(22, 10, 90, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    bore = _cylinder_y(6.5, 14).moved(Location((0, 0, 25)))
    return post - bore


def _guide():
    lower = Box(120, 34, 8, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(Location((0, 0, -27)))
    upper_left = Box(120, 5, 8, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(Location((0, -14.5, 23)))
    upper_right = Box(120, 5, 8, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(Location((0, 14.5, 23)))
    return Compound(children=[lower, upper_left, upper_right], label="horizontal_guide")


def _connecting_rod():
    beam = Box(ROD_LENGTH, 6, 12, align=(Align.MIN, Align.CENTER, Align.CENTER))
    ring_a = _cylinder_y(10, 6)
    ring_b = _cylinder_y(10, 6).moved(Location((ROD_LENGTH, 0, 0)))
    rod = beam.fuse(ring_a, ring_b)
    hole_a = _cylinder_y(4.5, 10)
    hole_b = _cylinder_y(4.5, 10).moved(Location((ROD_LENGTH, 0, 0)))
    return rod - hole_a - hole_b


def _hand_crank():
    arm = Box(42, 7, 8, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(Location((-42, 31, 0)))
    hub = _cylinder_y(10, 8).moved(Location((0, 31, 0)))
    grip = _cylinder_y(6, 28).moved(Location((-42, 45, 0)))
    return Compound(children=[arm, hub, grip], label="hand_crank")


def make_local_parts() -> Dict[str, object]:
    parts = {
        "base": Box(210, 100, 10, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "bearing_support_front": _bearing_support(),
        "bearing_support_rear": _bearing_support(),
        "horizontal_guide": _guide(),
        "crankshaft": _cylinder_y(6, 50),
        "crank_web": _cylinder_y(30, 8),
        "crank_pin": _cylinder_y(4, 20),
        "hand_crank": _hand_crank(),
        "connecting_rod": _connecting_rod(),
        "horizontal_slider": Box(32, 24, 30, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "slider_pin": _cylinder_y(4, 30),
    }
    for name, shape in parts.items():
        shape.label = name
    return parts


def initial_poses_mm():
    return {
        "base": (55, 0, 5),
        "bearing_support_front": (0, 12, 45),
        "bearing_support_rear": (0, 32, 45),
        "horizontal_guide": (105, ROD_PLANE_Y, 70),
        "crankshaft": CRANK_CENTER,
        "crank_web": CRANK_CENTER,
        "crank_pin": (CRANK_RADIUS, -12, 70),
        "hand_crank": CRANK_CENTER,
        "connecting_rod": (CRANK_RADIUS, ROD_PLANE_Y, 70),
        "horizontal_slider": (SLIDER_X0, ROD_PLANE_Y, 70),
        "slider_pin": (SLIDER_X0, ROD_PLANE_Y, 70),
    }


def build_machine():
    """Return a deterministic labeled build123d compound at the initial pose."""
    local = make_local_parts()
    placed = []
    for name, xyz in initial_poses_mm().items():
        child = local[name].moved(Location(xyz))
        child.label = name
        placed.append(child)
    return Compound(children=placed, label="open_horizontal_slider_crank")


def gen_step():
    return build_machine()


def export_named_meshes():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for name, shape in make_local_parts().items():
        export_stl(shape, MESH_DIR / f"{name}.stl", tolerance=0.08, angular_tolerance=0.12)


def write_semantics():
    poses = []
    for name, xyz_mm in initial_poses_mm().items():
        poses.append({"link": name, "position_m": [v / 1000.0 for v in xyz_mm], "quaternion_wxyz": [1, 0, 0, 0]})
    assembly = {
        "name": MECHANISM["name"],
        "root_link": "base",
        "links": MECHANISM["links"],
        "poses": poses,
        "ports_by_link": MECHANISM["ports_by_link"],
        "relations": MECHANISM["relations"],
        "motion_joints": MECHANISM["motion_joints"],
        "closures": MECHANISM["closures"],
        "transmissions": [],
        "planetary_stages": [],
        "mesh_pairs": [],
        "driver": MECHANISM["driver"],
        "output_link": "horizontal_slider",
        "watch_links": MECHANISM["watch_links"],
    }
    (ROOT / "assembly.json").write_text(json.dumps(assembly, indent=2), encoding="utf-8")
    bindings = {"roles": {
        "crankshaft_input": ["crankshaft_hinge"],
        "crank_pin": ["crank_pin"],
        "connecting_rod": ["connecting_rod"],
        "horizontal_slider": ["horizontal_slider"],
        "horizontal_guide": ["horizontal_guide"],
    }}
    (ROOT / "task_bindings.json").write_text(json.dumps(bindings, indent=2), encoding="utf-8")


if __name__ == "__main__":
    export_named_meshes()
    write_semantics()
    machine = build_machine()
    print(json.dumps({"name": machine.label, "named_parts": list(make_local_parts()), "part_count": len(make_local_parts())}))
