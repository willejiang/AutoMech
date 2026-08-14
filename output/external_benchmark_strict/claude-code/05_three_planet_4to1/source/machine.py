"""Open three-planet 4:1 reducer benchmark artifact.

CAD units are millimetres.  Assembly poses are emitted in metres/radians.
The Z axis is the common rotation axis and the XY plane exposes the gear train.
"""
from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

from build123d import Align, Box, Compound, Cylinder, Location, export_step, export_stl

TASK_DIR = Path(__file__).resolve().parents[1]
MESH_DIR = TASK_DIR / "meshes"
MODEL_DIR = TASK_DIR / "models"

MODULE = 2.0
SUN_TEETH = 12
PLANET_TEETH = 12
RING_TEETH = 36
SUN_PITCH_R = MODULE * SUN_TEETH / 2.0
PLANET_PITCH_R = MODULE * PLANET_TEETH / 2.0
RING_PITCH_R = MODULE * RING_TEETH / 2.0
PLANET_ORBIT_R = SUN_PITCH_R + PLANET_PITCH_R
GEAR_THICKNESS = 8.0
GEAR_Z = 39.0
CARRIER_Z = 30.5
PLANET_ANGLES_DEG = (0.0, 120.0, 240.0)


def _centered_box(x: float, y: float, z: float):
    return Box(x, y, z, align=(Align.CENTER, Align.CENTER, Align.CENTER))


def _external_gear(teeth: int, module: float, thickness: float, bore_r: float):
    pitch_r = module * teeth / 2.0
    root_r = max(pitch_r - 1.25 * module, bore_r + 1.2)
    tip_r = pitch_r + module
    shape = Cylinder(root_r, thickness, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    tooth_radial = tip_r - root_r + 0.8
    tooth_tangent = 0.46 * math.pi * module
    tooth_center_r = root_r + tooth_radial / 2.0 - 0.4
    for i in range(teeth):
        angle = 360.0 * i / teeth
        tooth = _centered_box(tooth_radial, tooth_tangent, thickness)
        tooth = Location((tooth_center_r, 0, 0), (0, 0, angle)) * tooth
        shape = shape + tooth
    if bore_r > 0:
        shape = shape - Cylinder(bore_r, thickness + 2.0,
                                 align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return shape


def _internal_ring(teeth: int, module: float, thickness: float):
    pitch_r = module * teeth / 2.0
    inner_root_r = pitch_r - module
    outer_r = pitch_r + 5.5
    inner_clear_r = pitch_r - 2.15 * module
    outer = Cylinder(outer_r, thickness, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    opening = Cylinder(inner_root_r, thickness + 2.0,
                       align=(Align.CENTER, Align.CENTER, Align.CENTER))
    ring = outer - opening
    tooth_radial = inner_root_r - inner_clear_r + 0.7
    tooth_tangent = 0.43 * math.pi * module
    tooth_center_r = inner_root_r - tooth_radial / 2.0 + 0.35
    for i in range(teeth):
        angle = 360.0 * i / teeth + 5.0
        tooth = _centered_box(tooth_radial, tooth_tangent, thickness)
        tooth = Location((tooth_center_r, 0, 0), (0, 0, angle)) * tooth
        ring = ring + tooth
    return ring


def _annulus(outer_r: float, inner_r: float, height: float):
    return (Cylinder(outer_r, height, align=(Align.CENTER, Align.CENTER, Align.CENTER))
            - Cylinder(inner_r, height + 2.0,
                       align=(Align.CENTER, Align.CENTER, Align.CENTER)))


def _carrier():
    hub = _annulus(8.0, 3.6, 4.0)
    shape = hub
    for angle in PLANET_ANGLES_DEG:
        arm = _centered_box(PLANET_ORBIT_R - 5.2, 5.0, 4.0)
        arm = Location(((PLANET_ORBIT_R - 5.2) / 2.0 + 5.2, 0, 0),
                       (0, 0, angle)) * arm
        shape = shape + arm
    # Coaxial output sleeve extends downward; sun shaft has 0.6 mm radial clearance.
    sleeve = _annulus(5.0, 3.6, 23.0)
    sleeve = Location((0, 0, -11.5)) * sleeve
    return shape + sleeve


def _hand_crank():
    # Local to the sun body/gear center; fully above the gear face.
    arm = _centered_box(28.0, 4.0, 4.0)
    arm = Location((14.0, 0, 13.0)) * arm
    grip = Cylinder(3.2, 14.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    grip = Location((28.0, 0, 20.0)) * grip
    return arm + grip


def _base():
    plate = Box(110.0, 100.0, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # Four bench feet keep a broad stable footprint.
    for x in (-46.0, 46.0):
        for y in (-41.0, 41.0):
            plate = plate + Location((x, y, 6.0)) * Cylinder(6.0, 4.0)
    return plate


def _support(angle_deg: float):
    # A narrow post at the ring perimeter leaves the central mechanism visible.
    post = _centered_box(7.0, 7.0, 31.0)
    return Location((0, 0, 15.5)) * post


def make_local_parts():
    """Return deterministic local-coordinate solids keyed by stable semantic names."""
    parts = {
        "base": _base(),
        "ring_gear": _internal_ring(RING_TEETH, MODULE, GEAR_THICKNESS),
        "sun_gear": _external_gear(SUN_TEETH, MODULE, GEAR_THICKNESS, 3.0),
        "carrier": _carrier(),
        "sun_shaft": Cylinder(3.0, 31.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)),
        "hand_crank": _hand_crank(),
        "upper_bearing": _annulus(7.0, 3.4, 5.0),
        "lower_bearing": _annulus(8.0, 5.4, 6.0),
    }
    planet_shape = _external_gear(PLANET_TEETH, MODULE, GEAR_THICKNESS, 3.3)
    for i in range(1, 4):
        parts[f"planet_gear_{i}"] = planet_shape
        parts[f"planet_pin_{i}"] = Cylinder(
            3.0, 13.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        )
        parts[f"ring_support_{i}"] = _support(PLANET_ANGLES_DEG[i - 1])
    for name, shape in parts.items():
        shape.label = name
    return parts


def world_placements():
    poses = {
        "base": Location((0, 0, 0)),
        "ring_gear": Location((0, 0, GEAR_Z)),
        "sun_gear": Location((0, 0, GEAR_Z)),
        "carrier": Location((0, 0, CARRIER_Z)),
        "sun_shaft": Location((0, 0, 47.5)),
        "hand_crank": Location((0, 0, GEAR_Z)),
        "upper_bearing": Location((0, 0, 51.0)),
        "lower_bearing": Location((0, 0, 15.0)),
    }
    for i, angle in enumerate(PLANET_ANGLES_DEG, 1):
        a = math.radians(angle)
        x, y = PLANET_ORBIT_R * math.cos(a), PLANET_ORBIT_R * math.sin(a)
        poses[f"planet_gear_{i}"] = Location((x, y, GEAR_Z))
        poses[f"planet_pin_{i}"] = Location((x, y, CARRIER_Z + 6.5))
        support_r = RING_PITCH_R + 6.0
        poses[f"ring_support_{i}"] = Location(
            (support_r * math.cos(a), support_r * math.sin(a), 6.0),
            (0, 0, angle),
        )
    return poses


def build_machine():
    """Build a labeled, nonempty build123d assembly Compound."""
    local = make_local_parts()
    poses = world_placements()
    children = []
    for name in sorted(local):
        placed = poses[name] * local[name]
        placed.label = name
        children.append(placed)
    assembly = Compound(children=children)
    assembly.label = "open_three_planet_4to1"
    return assembly


MECHANISM = {
    "name": "open_three_planet_4to1",
    "units": {"cad_length": "mm", "assembly_length": "m", "angle": "rad"},
    "links": [
        {"name": "base", "mesh": "meshes/base.stl", "dof": "fixed", "axis": None, "driver": False},
        {"name": "ring_gear", "mesh": "meshes/ring_gear.stl", "dof": "fixed", "axis": [0, 0, 1], "driver": False, "teeth": RING_TEETH},
        {"name": "sun_gear", "mesh": "meshes/sun_gear.stl", "dof": "revolute", "axis": [0, 0, 1], "driver": True, "teeth": SUN_TEETH},
        {"name": "carrier", "mesh": "meshes/carrier.stl", "dof": "revolute", "axis": [0, 0, 1], "driver": False},
        *[
            {"name": f"planet_gear_{i}", "mesh": f"meshes/planet_gear_{i}.stl", "dof": "revolute", "axis": [0, 0, 1], "driver": False, "teeth": PLANET_TEETH}
            for i in range(1, 4)
        ],
        *[
            {"name": f"planet_pin_{i}", "mesh": f"meshes/planet_pin_{i}.stl", "dof": "fixed", "axis": [0, 0, 1], "driver": False, "rigid_mount": "carrier"}
            for i in range(1, 4)
        ],
        {"name": "sun_shaft", "mesh": "meshes/sun_shaft.stl", "dof": "fixed", "axis": [0, 0, 1], "driver": False, "rigid_mount": "sun_gear"},
        {"name": "hand_crank", "mesh": "meshes/hand_crank.stl", "dof": "fixed", "axis": [0, 0, 1], "driver": False, "rigid_mount": "sun_gear"},
        {"name": "upper_bearing", "mesh": "meshes/upper_bearing.stl", "dof": "fixed", "axis": [0, 0, 1], "driver": False, "rigid_mount": "base"},
        {"name": "lower_bearing", "mesh": "meshes/lower_bearing.stl", "dof": "fixed", "axis": [0, 0, 1], "driver": False, "rigid_mount": "base"},
        *[
            {"name": f"ring_support_{i}", "mesh": f"meshes/ring_support_{i}.stl", "dof": "fixed", "axis": None, "driver": False, "rigid_mount": "base"}
            for i in range(1, 4)
        ],
    ],
    "poses": [],
    "ports_by_link": {
        "base": ["sun_axis", "carrier_axis", "ring_mount"],
        "sun_gear": ["sun_axis", "sun_teeth", "shaft_press_fit"],
        "ring_gear": ["ring_teeth", "ring_mount"],
        "carrier": ["carrier_axis", "planet_pin_axis_1", "planet_pin_axis_2", "planet_pin_axis_3"],
        **{f"planet_gear_{i}": [f"planet_teeth_{i}", f"planet_bore_{i}"] for i in range(1, 4)},
    },
    "relations": [
        {"type": "fixed", "parent": "world", "child": "base"},
        {"type": "rigid_mount", "parent": "base", "child": "ring_gear"},
        {"type": "press_fit", "parent": "sun_gear", "child": "sun_shaft"},
        {"type": "rigid_mount", "parent": "sun_gear", "child": "hand_crank"},
        {"type": "running_bearing", "outer": "upper_bearing", "inner": "sun_shaft", "radial_clearance_mm": 0.4},
        {"type": "running_bearing", "outer": "lower_bearing", "inner": "carrier", "radial_clearance_mm": 0.4},
        {"type": "coaxial_running_clearance", "outer": "carrier", "inner": "sun_shaft", "radial_clearance_mm": 0.6},
        *[{"type": "rigid_mount", "parent": "carrier", "child": f"planet_pin_{i}"} for i in range(1, 4)],
        *[{"type": "running_bearing", "outer": f"planet_gear_{i}", "inner": f"planet_pin_{i}", "radial_clearance_mm": 0.3} for i in range(1, 4)],
        *[{"type": "rigid_mount", "parent": "base", "child": f"ring_support_{i}"} for i in range(1, 4)],
    ],
    "motion_joints": [
        {"name": "sun_input_hinge", "kind": "revolute", "parent": "base", "child": "sun_gear", "axis": [0, 0, 1], "driver": True},
        {"name": "carrier_output_hinge", "kind": "revolute", "parent": "base", "child": "carrier", "axis": [0, 0, 1], "driver": False},
        *[
            {"name": f"planet_pin_hinge_{i}", "kind": "revolute", "parent": "carrier", "child": f"planet_gear_{i}", "axis": [0, 0, 1], "driver": False}
            for i in range(1, 4)
        ],
    ],
    "transmissions": [
        {"name": "sun_to_carrier", "driving": "sun_input_hinge", "driven": "carrier_output_hinge", "driven_over_driving": 0.25, "ideal": True},
        *[
            {"name": f"sun_to_planet_local_{i}", "driving": "sun_input_hinge", "driven": f"planet_pin_hinge_{i}", "driven_over_driving": -0.75, "ideal": True}
            for i in range(1, 4)
        ],
    ],
    "planetary_stages": [{
        "name": "stage_1", "sun": "sun_gear", "ring": "ring_gear", "carrier": "carrier",
        "planets": ["planet_gear_1", "planet_gear_2", "planet_gear_3"],
        "planet_pins": ["planet_pin_1", "planet_pin_2", "planet_pin_3"],
        "sun_teeth": SUN_TEETH, "planet_teeth": PLANET_TEETH, "ring_teeth": RING_TEETH,
        "fixed_member": "ring_gear", "input_member": "sun_gear", "output_member": "carrier",
        "sun_to_carrier_reduction": 4.0, "carrier_over_sun": 0.25,
        "planet_spacing_deg": [0.0, 120.0, 240.0],
    }],
    "mesh_pairs": [
        *[{"name": f"sun_planet_{i}", "a": "sun_gear", "b": f"planet_gear_{i}", "kind": "external", "driven_over_driving": -1.0, "ideal": True} for i in range(1, 4)],
        *[{"name": f"ring_planet_{i}", "a": "ring_gear", "b": f"planet_gear_{i}", "kind": "internal", "ideal": True} for i in range(1, 4)],
    ],
    "driver": {"joint": "sun_input_hinge", "source": "hand_crank", "mode": "position", "direct_output_actuation": False},
    "output_link": "carrier",
    "watch_links": ["sun_gear", "carrier", "planet_gear_1", "planet_gear_2", "planet_gear_3", "planet_pin_1", "planet_pin_2", "planet_pin_3", "hand_crank"],
}


def _pose_records():
    records = []
    for name, loc in world_placements().items():
        p = loc.position
        records.append({"link": name, "xyz_m": [p.X / 1000.0, p.Y / 1000.0, p.Z / 1000.0], "rpy_rad": [0.0, 0.0, 0.0]})
    return records


def assembly_document():
    doc = {
        "name": MECHANISM["name"],
        "root_link": "base",
        "links": MECHANISM["links"],
        "poses": _pose_records(),
        "ports_by_link": MECHANISM["ports_by_link"],
        "relations": MECHANISM["relations"],
        "motion_joints": MECHANISM["motion_joints"],
        "transmissions": MECHANISM["transmissions"],
        "planetary_stages": MECHANISM["planetary_stages"],
        "mesh_pairs": MECHANISM["mesh_pairs"],
        "output_link": MECHANISM["output_link"],
        "watch_links": MECHANISM["watch_links"],
    }
    return doc


def task_bindings_document():
    return {"roles": {
        "fixed_ring": ["ring_gear"],
        "sun_input": ["sun_input_hinge"],
        "carrier_output": ["carrier_output_hinge"],
        "planet_gear": ["planet_gear_1", "planet_gear_2", "planet_gear_3"],
        "planet_pin_hinge": ["planet_pin_hinge_1", "planet_pin_hinge_2", "planet_pin_hinge_3"],
        "hand_crank": ["hand_crank"],
    }}


def mjcf_text():
    mesh_assets = "\n".join(
        f'    <mesh name="{name}_mesh" file="{name}.stl" scale="0.001 0.001 0.001"/>'
        for name in sorted(make_local_parts())
    )
    static_visuals = "\n".join([
        '      <geom name="base_visual" type="mesh" mesh="base_mesh" class="visual"/>',
        '      <geom name="ring_visual" type="mesh" mesh="ring_gear_mesh" class="visual" pos="0 0 0.039"/>',
        '      <geom name="upper_bearing_visual" type="mesh" mesh="upper_bearing_mesh" class="visual" pos="0 0 0.051"/>',
        '      <geom name="lower_bearing_visual" type="mesh" mesh="lower_bearing_mesh" class="visual" pos="0 0 0.015"/>',
        *[f'      <geom name="ring_support_{i}_visual" type="mesh" mesh="ring_support_{i}_mesh" class="visual" pos="{(RING_PITCH_R+6)*math.cos(math.radians(a))/1000:.9f} {(RING_PITCH_R+6)*math.sin(math.radians(a))/1000:.9f} 0.006" euler="0 0 {math.radians(a):.9f}"/>' for i, a in enumerate(PLANET_ANGLES_DEG, 1)],
    ])
    ring_collisions = "\n".join(
        f'      <geom name="ring_collision_{i}" type="box" size="0.006 0.012 0.004" pos="{RING_PITCH_R*math.cos(2*math.pi*i/12)/1000:.9f} {RING_PITCH_R*math.sin(2*math.pi*i/12)/1000:.9f} 0.039" euler="0 0 {2*math.pi*i/12:.9f}" class="collision"/>'
        for i in range(12)
    )
    planet_bodies = []
    pin_bodies = []
    excludes = []
    equalities = []
    for i, a_deg in enumerate(PLANET_ANGLES_DEG, 1):
        a = math.radians(a_deg)
        x, y = PLANET_ORBIT_R * math.cos(a) / 1000.0, PLANET_ORBIT_R * math.sin(a) / 1000.0
        pin_bodies.append(f'''      <body name="planet_pin_{i}" pos="{x:.9f} {y:.9f} 0.0065">
        <geom name="planet_pin_{i}_visual" type="mesh" mesh="planet_pin_{i}_mesh" class="visual"/>
      </body>''')
        planet_bodies.append(f'''      <body name="planet_gear_{i}" pos="{x:.9f} {y:.9f} 0.0085">
        <inertial pos="0 0 0" mass="0.09" diaginertia="0.000012 0.000012 0.000020"/>
        <joint name="planet_pin_hinge_{i}" type="hinge" axis="0 0 1" damping="0.01"/>
        <geom name="planet_gear_{i}_visual" type="mesh" mesh="planet_gear_{i}_mesh" class="visual"/>
        <geom name="planet_gear_{i}_collision" type="cylinder" size="0.0135 0.004" class="collision"/>
      </body>''')
        excludes.extend([
            f'    <exclude body1="sun_gear" body2="planet_gear_{i}"/>',
            f'    <exclude body1="ring_gear" body2="planet_gear_{i}"/>',
        ])
        equalities.append(f'    <joint name="planetary_planet_{i}" joint1="planet_pin_hinge_{i}" joint2="sun_input_hinge" polycoef="0 -0.75 0 0 0" solref="0.002 1"/>')
    return f'''<mujoco model="open_three_planet_4to1">
  <compiler angle="radian" meshdir="../meshes" autolimits="true" fusestatic="false"/>
  <option timestep="0.005" gravity="0 0 0" integrator="implicitfast" iterations="100"/>
  <default>
    <default class="visual"><geom contype="0" conaffinity="0" density="0" rgba="0.72 0.72 0.76 1"/></default>
    <default class="collision"><geom contype="1" conaffinity="1" density="0" rgba="0 0 0 0" friction="0.6 0.02 0.002"/></default>
  </default>
  <asset>
{mesh_assets}
    <material name="gear_mat" rgba="0.78 0.62 0.20 1"/>
  </asset>
  <worldbody>
    <light pos="0 -0.15 0.22" dir="0 0.6 -1"/>
    <geom name="ground" type="plane" size="0.3 0.3 0.01" pos="0 0 -0.001" rgba="0.25 0.27 0.30 1"/>
    <body name="base" pos="0 0 0">
{static_visuals}
      <geom name="base_collision" type="box" size="0.055 0.05 0.003" pos="0 0 0.003" class="collision"/>
      <body name="ring_gear" pos="0 0 0">
{ring_collisions}
      </body>
    </body>
    <body name="sun_gear" pos="0 0 0.039">
      <inertial pos="0 0 0.005" mass="0.22" diaginertia="0.00005 0.00005 0.00008"/>
      <joint name="sun_input_hinge" type="hinge" axis="0 0 1" damping="0.02"/>
      <geom name="sun_gear_visual" type="mesh" mesh="sun_gear_mesh" class="visual" material="gear_mat"/>
      <geom name="sun_gear_collision" type="cylinder" size="0.0135 0.004" class="collision"/>
      <geom name="sun_shaft_visual" type="mesh" mesh="sun_shaft_mesh" class="visual" pos="0 0 0.0085"/>
      <body name="hand_crank" pos="0 0 0">
        <geom name="hand_crank_visual" type="mesh" mesh="hand_crank_mesh" class="visual" rgba="0.25 0.55 0.82 1"/>
      </body>
    </body>
    <body name="carrier" pos="0 0 0.0305">
      <inertial pos="0 0 -0.004" mass="0.28" diaginertia="0.00012 0.00012 0.00018"/>
      <joint name="carrier_output_hinge" type="hinge" axis="0 0 1" damping="0.03"/>
      <geom name="carrier_visual" type="mesh" mesh="carrier_mesh" class="visual" rgba="0.28 0.62 0.45 1"/>
      <geom name="carrier_hub_collision" type="cylinder" size="0.008 0.002" class="collision"/>
{chr(10).join(pin_bodies)}
{chr(10).join(planet_bodies)}
    </body>
  </worldbody>
  <contact>
{chr(10).join(excludes)}
  </contact>
  <equality>
    <joint name="planetary_carrier" joint1="carrier_output_hinge" joint2="sun_input_hinge" polycoef="0 0.25 0 0 0" solref="0.002 1"/>
{chr(10).join(equalities)}
  </equality>
  <actuator>
    <position name="hand_crank_drive" joint="sun_input_hinge" kp="260" kv="28"/>
  </actuator>
</mujoco>
'''


def export_artifacts():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    parts = make_local_parts()
    for name, shape in sorted(parts.items()):
        export_stl(shape, MESH_DIR / f"{name}.stl", tolerance=0.05, angular_tolerance=0.08)
    export_step(build_machine(), TASK_DIR / "open_three_planet_4to1.step")
    (TASK_DIR / "assembly.json").write_text(json.dumps(assembly_document(), indent=2), encoding="utf-8")
    (TASK_DIR / "task_bindings.json").write_text(json.dumps(task_bindings_document(), indent=2), encoding="utf-8")
    (MODEL_DIR / "model.mjcf").write_text(mjcf_text(), encoding="utf-8")
    return {"part_count": len(parts), "parts": sorted(parts), "step": str(TASK_DIR / "open_three_planet_4to1.step")}


if __name__ == "__main__":
    print(json.dumps(export_artifacts(), indent=2))
