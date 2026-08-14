import math
import build123d as b3d
from build123d import (
    Axis, Box, BuildPart, BuildSketch, Circle, Cylinder, Location, Polygon,
    Align, Mode, extrude
)

# ---------------------------------------------------------------------------
# Drivetrain arithmetic -- all gear locations derive from these values
# ---------------------------------------------------------------------------

M = 1.2
Z_SUN = 18
Z_PLANET = 18
Z_RING = Z_SUN + 2 * Z_PLANET
PLANET_COUNT = 3

GEAR_FACE = 8.0

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

SUN_PITCH_R = pitch_r(Z_SUN)
PLANET_PITCH_R = pitch_r(Z_PLANET)
RING_PITCH_R = pitch_r(Z_RING)

SUN_PLANET_CD = center_dist(Z_SUN, Z_PLANET)
RING_PLANET_CD = RING_PITCH_R - PLANET_PITCH_R

assert abs(SUN_PLANET_CD - RING_PLANET_CD) < 1.0e-9
assert Z_RING == Z_SUN + 2 * Z_PLANET
assert (Z_SUN + Z_RING) % PLANET_COUNT == 0

FIXED_RING_REDUCTION = 1.0 + Z_RING / Z_SUN

PLANET_ANGLES = tuple(2.0 * math.pi * i / PLANET_COUNT
                      for i in range(PLANET_COUNT))
PLANET_POSITIONS = tuple(
    (
        SUN_PLANET_CD * math.cos(a),
        SUN_PLANET_CD * math.sin(a),
    )
    for a in PLANET_ANGLES
)

# With a sun tooth centered on +X, place a planet gap toward the sun.
# The chosen tooth counts make this phase repeat correctly at all 120° sites.
PLANET_PHASES_DEG = tuple(
    math.degrees(a + math.pi - math.pi / Z_PLANET)
    for a in PLANET_ANGLES
)

# ---------------------------------------------------------------------------
# Fits and axial stations
# ---------------------------------------------------------------------------

INPUT_SHAFT_R = 4.0
INPUT_PRESS_BORE_R = INPUT_SHAFT_R - 0.005
INPUT_RUNNING_BORE_R = INPUT_SHAFT_R + 0.05

PLANET_PIN_R = 2.5
PLANET_RUNNING_BORE_R = PLANET_PIN_R + 0.05
PLANET_PIN_PRESS_BORE_R = PLANET_PIN_R - 0.005

OUTPUT_SHAFT_OUTER_R = 7.0
OUTPUT_SHAFT_INNER_R = 5.0
OUTPUT_PRESS_BORE_R = OUTPUT_SHAFT_OUTER_R - 0.005
OUTPUT_RUNNING_BORE_R = OUTPUT_SHAFT_OUTER_R + 0.05

BASE_H = 5.0
BASE_TOP_Z = BASE_H

LOWER_BEARING_Z = BASE_TOP_Z
LOWER_BEARING_H = 7.0

RING_SEAT_Z = 14.0
RING_SEAT_H = 3.0

GEAR_Z = RING_SEAT_Z + RING_SEAT_H
GEAR_TOP_Z = GEAR_Z + GEAR_FACE

PLANET_WASHER_H = 1.0
PLANET_WASHER_Z = GEAR_Z - PLANET_WASHER_H

CARRIER_GAP = 1.5
CARRIER_Z = GEAR_TOP_Z + CARRIER_GAP
CARRIER_H = 4.0
CARRIER_TOP_Z = CARRIER_Z + CARRIER_H

PLANET_PIN_Z = PLANET_WASHER_Z - 0.5
PLANET_PIN_TOP_Z = CARRIER_TOP_Z
PLANET_PIN_H = PLANET_PIN_TOP_Z - PLANET_PIN_Z

OUTPUT_SHAFT_Z = CARRIER_Z
OUTPUT_SHAFT_TOP_Z = 62.0
OUTPUT_SHAFT_H = OUTPUT_SHAFT_TOP_Z - OUTPUT_SHAFT_Z

UPPER_BEARING_Z = 44.0
UPPER_BEARING_H = 8.0
UPPER_BEARING_TOP_Z = UPPER_BEARING_Z + UPPER_BEARING_H

SLEEVE_Z = 54.0
SLEEVE_H = 4.0
SLEEVE_INNER_R = INPUT_RUNNING_BORE_R
SLEEVE_OUTER_R = OUTPUT_SHAFT_INNER_R - 0.005

INPUT_SHAFT_Z = BASE_TOP_Z
INPUT_SHAFT_TOP_Z = 72.0
INPUT_SHAFT_H = INPUT_SHAFT_TOP_Z - INPUT_SHAFT_Z

CRANK_Z = 68.0
CRANK_H = 4.0
CRANK_R = 28.0

OUTPUT_POINTER_Z = 58.0
OUTPUT_POINTER_H = 3.0
OUTPUT_POINTER_LENGTH = 24.0

RING_INTERNAL_TIP_R = RING_PITCH_R - M
RING_INTERNAL_ROOT_R = RING_PITCH_R + 1.25 * M
RING_OUTER_R = RING_INTERNAL_ROOT_R + 5.0

CARRIER_R = RING_INTERNAL_TIP_R - 0.9

# ---------------------------------------------------------------------------
# Canonical mechanism semantics
# Ports are expressed in each named part's local coordinates.
# ---------------------------------------------------------------------------

MECHANISM = {
    "name": "hand_driven_fixed_ring_planetary_reducer",
    "output_link": "carrier",
    "watch_links": [
        "input_shaft",
        "sun_gear",
        "planet_gear_1",
        "planet_gear_2",
        "planet_gear_3",
        "carrier",
        "output_shaft",
        "output_pointer",
    ],
    "ports_by_link": {
        "input_shaft": [
            {
                "name": "shaft_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, INPUT_SHAFT_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": INPUT_SHAFT_H,
            },
            {
                "name": "sun_seat",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, GEAR_Z - INPUT_SHAFT_Z + GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
        ],
        "sun_gear": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_PRESS_BORE_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "pitch",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": SUN_PITCH_R,
                "depth_mm": GEAR_FACE,
            },
        ],
        "fixed_ring_gear": [
            {
                "name": "pitch",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": RING_PITCH_R,
                "depth_mm": GEAR_FACE,
            }
        ],
        "carrier": [
            {
                "name": "output_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CARRIER_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_PRESS_BORE_R,
                "depth_mm": CARRIER_H,
            },
            *[
                {
                    "name": f"pin_bore_{i + 1}",
                    "type": "bore",
                    "xyz_mm": [
                        PLANET_POSITIONS[i][0],
                        PLANET_POSITIONS[i][1],
                        CARRIER_H / 2.0,
                    ],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_PIN_PRESS_BORE_R,
                    "depth_mm": CARRIER_H,
                }
                for i in range(PLANET_COUNT)
            ],
        ],
        "output_shaft": [
            {
                "name": "outer",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, OUTPUT_SHAFT_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_SHAFT_OUTER_R,
                "depth_mm": OUTPUT_SHAFT_H,
            },
            {
                "name": "input_clearance",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, OUTPUT_SHAFT_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_SHAFT_INNER_R,
                "depth_mm": OUTPUT_SHAFT_H,
            },
        ],
        "lower_input_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, LOWER_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_RUNNING_BORE_R,
                "depth_mm": LOWER_BEARING_H,
            }
        ],
        "upper_output_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, UPPER_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_RUNNING_BORE_R,
                "depth_mm": UPPER_BEARING_H,
            }
        ],
        "output_sleeve_bearing": [
            {
                "name": "input_journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, SLEEVE_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SLEEVE_INNER_R,
                "depth_mm": SLEEVE_H,
            }
        ],
        **{
            f"planet_pin_{i + 1}": [
                {
                    "name": "pin_axis",
                    "type": "shaft",
                    "xyz_mm": [0.0, 0.0, PLANET_PIN_H / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_PIN_R,
                    "depth_mm": PLANET_PIN_H,
                }
            ]
            for i in range(PLANET_COUNT)
        },
        **{
            f"planet_gear_{i + 1}": [
                {
                    "name": "bore",
                    "type": "bore",
                    "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_RUNNING_BORE_R,
                    "depth_mm": GEAR_FACE,
                },
                {
                    "name": "pitch",
                    "type": "gear_mesh",
                    "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "pitch_radius_mm": PLANET_PITCH_R,
                    "depth_mm": GEAR_FACE,
                },
            ]
            for i in range(PLANET_COUNT)
        },
    },
    "relations": [
        {
            "name": "sun_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "sun_seat",
            "incoming_part": "sun_gear",
            "incoming_port": "bore",
        },
        {
            "name": "input_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "lower_input_bearing",
            "base_port": "journal",
            "incoming_part": "input_shaft",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "input_upper_sleeve_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_sleeve_bearing",
            "base_port": "input_journal",
            "incoming_part": "input_shaft",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "carrier_output_press_fit",
            "mate_type": "press_fit",
            "base_part": "carrier",
            "base_port": "output_bore",
            "incoming_part": "output_shaft",
            "incoming_port": "outer",
        },
        {
            "name": "output_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_output_bearing",
            "base_port": "journal",
            "incoming_part": "output_shaft",
            "incoming_port": "outer",
        },
        *[
            {
                "name": f"planet_{i + 1}_pin_journal",
                "mate_type": "journal_bearing",
                "base_part": f"planet_gear_{i + 1}",
                "base_port": "bore",
                "incoming_part": f"planet_pin_{i + 1}",
                "incoming_port": "pin_axis",
            }
            for i in range(PLANET_COUNT)
        ],
        *[
            {
                "name": f"sun_planet_mesh_{i + 1}",
                "mate_type": "gear_spur_external",
                "base_part": "sun_gear",
                "base_port": "pitch",
                "incoming_part": f"planet_gear_{i + 1}",
                "incoming_port": "pitch",
                "separation_axis": [
                    math.cos(PLANET_ANGLES[i]),
                    math.sin(PLANET_ANGLES[i]),
                    0.0,
                ],
            }
            for i in range(PLANET_COUNT)
        ],
    ],
    "motion_joints": [
        {
            "name": "input_hinge",
            "parent": "",
            "child": "input_shaft",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "carrier_hinge",
            "parent": "",
            "child": "carrier",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        *[
            {
                "name": f"planet_{i + 1}_carrier_hinge",
                "parent": "carrier",
                "child": f"planet_gear_{i + 1}",
                "type": "hinge",
                "axis": [0.0, 0.0, 1.0],
                "pos_mm": [
                    PLANET_POSITIONS[i][0],
                    PLANET_POSITIONS[i][1],
                    0.0,
                ],
            }
            for i in range(PLANET_COUNT)
        ],
    ],
    "transmissions": [
        {
            "name": "input_shaft_to_sun",
            "type": "compound_1to1",
            "driving_link": "input_shaft",
            "driven_link": "sun_gear",
            "ratio": 1.0,
        },
        {
            "name": "carrier_to_output_shaft",
            "type": "compound_1to1",
            "driving_link": "carrier",
            "driven_link": "output_shaft",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [
        {
            "name": "fixed_ring_stage",
            "sun": "sun_gear",
            "ring": "fixed_ring_gear",
            "carrier": "carrier",
            "planets": [
                {"gear": "planet_gear_1", "pin": "planet_pin_1"},
                {"gear": "planet_gear_2", "pin": "planet_pin_2"},
                {"gear": "planet_gear_3", "pin": "planet_pin_3"},
            ],
            "sun_teeth": Z_SUN,
            "planet_teeth": Z_PLANET,
            "ring_teeth": Z_RING,
            "fixed_member": "ring",
            "input_member": "sun",
            "output_member": "carrier",
        }
    ],
}


def build_machine():
    a = AssemblyHelper("hand_driven_fixed_ring_planetary_reducer")

    def cylinder_min(radius, height):
        return Cylinder(
            radius,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )

    def annulus(outer_r, inner_r, height):
        outer = cylinder_min(outer_r, height)
        cutter = Cylinder(
            inner_r,
            height + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -1.0)))
        return outer - cutter

    def make_internal_ring(module, teeth, face_width):
        pr = pitch_r(teeth)
        tip_r = pr - module
        root_r = pr + 1.25 * module
        outer_r = root_r + 5.0

        tooth_pitch_angle = 2.0 * math.pi / teeth
        root_half_angle = 0.33 * tooth_pitch_angle
        tip_half_angle = 0.18 * tooth_pitch_angle
        overlap_r = root_r + 0.15

        with BuildSketch() as ring_sketch:
            Circle(outer_r)
            Circle(root_r, mode=Mode.SUBTRACT)

            for tooth_index in range(teeth):
                angle = tooth_index * tooth_pitch_angle

                def polar(radius, angular_offset):
                    aa = angle + angular_offset
                    return (
                        radius * math.cos(aa),
                        radius * math.sin(aa),
                    )

                Polygon(
                    polar(overlap_r, -root_half_angle),
                    polar(root_r, -root_half_angle),
                    polar(tip_r, -tip_half_angle),
                    polar(tip_r, tip_half_angle),
                    polar(root_r, root_half_angle),
                    polar(overlap_r, root_half_angle),
                    mode=Mode.ADD,
                )

        return extrude(ring_sketch.sketch, amount=face_width)

    # Ground-supported base.
    base = cylinder_min(47.0, BASE_H)
    a.add(base, "baseplate|dof=fixed")

    # Lower input bearing housing and running-fit bearing.
    lower_housing = annulus(14.0, 9.6, LOWER_BEARING_H).moved(
        Location((0.0, 0.0, LOWER_BEARING_Z))
    )
    a.add(
        lower_housing,
        "lower_bearing_housing|dof=fixed|mount=baseplate",
    )

    lower_bearing = annulus(
        9.5, INPUT_RUNNING_BORE_R, LOWER_BEARING_H
    ).moved(Location((0.0, 0.0, LOWER_BEARING_Z)))
    a.add(
        lower_bearing,
        "lower_input_bearing|dof=fixed|mount=lower_bearing_housing",
    )

    # Fixed annular seat beneath the ring gear.
    ring_seat = annulus(
        RING_OUTER_R + 1.0,
        RING_INTERNAL_ROOT_R + 0.6,
        RING_SEAT_H,
    ).moved(Location((0.0, 0.0, RING_SEAT_Z)))
    a.add(ring_seat, "ring_seat|dof=fixed|mount=baseplate")

    # Three external posts carry the upper bridge without entering the gear volume.
    POST_R = 3.5
    POST_CENTER_R = RING_OUTER_R + POST_R + 1.5
    POST_Z = BASE_TOP_Z
    POST_TOP_Z = UPPER_BEARING_TOP_Z + 3.0
    POST_H = POST_TOP_Z - POST_Z

    post_positions = tuple(
        (
            POST_CENTER_R * math.cos(a0),
            POST_CENTER_R * math.sin(a0),
        )
        for a0 in PLANET_ANGLES
    )

    for i, (px, py) in enumerate(post_positions):
        post = cylinder_min(POST_R, POST_H).moved(
            Location((px, py, POST_Z))
        )
        a.add(
            post,
            f"support_post_{i + 1}|dof=fixed|mount=baseplate",
        )

    # Input shaft: the sole driven part.
    input_shaft = cylinder_min(INPUT_SHAFT_R, INPUT_SHAFT_H).moved(
        Location((0.0, 0.0, INPUT_SHAFT_Z))
    )
    a.add(
        input_shaft,
        "input_shaft|dof=spin|driver=True|spin_axis=z|"
        "mount=lower_input_bearing,output_sleeve_bearing",
    )

    # Press-fit input sun.
    sun = make_gear(
        M,
        Z_SUN,
        GEAR_FACE,
        2.0 * INPUT_PRESS_BORE_R,
    ).moved(Location((0.0, 0.0, GEAR_Z)))
    a.add(
        sun,
        "sun_gear|dof=spin|spin_axis=z|mesh_id=fixed_ring_stage|"
        "mount=input_shaft",
    )

    # Fixed internal ring gear.
    ring = make_internal_ring(M, Z_RING, GEAR_FACE).moved(
        Location((0.0, 0.0, GEAR_Z))
    )
    a.add(
        ring,
        "fixed_ring_gear|dof=fixed|mesh_id=fixed_ring_stage|"
        "mount=ring_seat",
    )

    # Carrier plate: central output press fit plus three pin press-fit holes.
    carrier = cylinder_min(CARRIER_R, CARRIER_H)

    output_bore_tool = Cylinder(
        OUTPUT_PRESS_BORE_R,
        CARRIER_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    carrier = carrier - output_bore_tool

    for px, py in PLANET_POSITIONS:
        pin_hole_tool = Cylinder(
            PLANET_PIN_PRESS_BORE_R,
            CARRIER_H + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, -1.0)))
        carrier = carrier - pin_hole_tool

    carrier = carrier.moved(Location((0.0, 0.0, CARRIER_Z)))
    a.add(
        carrier,
        "carrier|dof=spin|spin_axis=z|mount=output_shaft",
    )

    # Carrier-mounted pins, thrust washers, and exactly three planet gears.
    for i, ((px, py), phase_deg) in enumerate(
        zip(PLANET_POSITIONS, PLANET_PHASES_DEG)
    ):
        pin_name = f"planet_pin_{i + 1}"
        washer_name = f"planet_thrust_washer_{i + 1}"
        gear_name = f"planet_gear_{i + 1}"

        pin = cylinder_min(PLANET_PIN_R, PLANET_PIN_H).moved(
            Location((px, py, PLANET_PIN_Z))
        )
        a.add(
            pin,
            f"{pin_name}|dof=fixed|mount=carrier",
        )

        washer = annulus(
            PLANET_PIN_R + 2.0,
            PLANET_PIN_R + 0.05,
            PLANET_WASHER_H,
        ).moved(Location((px, py, PLANET_WASHER_Z)))
        a.add(
            washer,
            f"{washer_name}|dof=fixed|mount={pin_name}",
        )

        planet = make_gear(
            M,
            Z_PLANET,
            GEAR_FACE,
            2.0 * PLANET_RUNNING_BORE_R,
        ).moved(
            Location(
                (px, py, GEAR_Z),
                (0.0, 0.0, phase_deg),
            )
        )
        a.add(
            planet,
            f"{gear_name}|dof=spin|spin_axis=z|"
            f"mesh_id=fixed_ring_stage|mount={pin_name},{washer_name}",
        )

    # Hollow carrier output shaft. Its 10 mm through-bore clears the 8 mm input.
    output_shaft = annulus(
        OUTPUT_SHAFT_OUTER_R,
        OUTPUT_SHAFT_INNER_R,
        OUTPUT_SHAFT_H,
    ).moved(Location((0.0, 0.0, OUTPUT_SHAFT_Z)))
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=z|"
        "mount=upper_output_bearing,carrier",
    )

    # Sleeve bearing pressed into the output tube supports the upper input shaft.
    output_sleeve = annulus(
        SLEEVE_OUTER_R,
        SLEEVE_INNER_R,
        SLEEVE_H,
    ).moved(Location((0.0, 0.0, SLEEVE_Z)))
    a.add(
        output_sleeve,
        "output_sleeve_bearing|dof=fixed|mount=output_shaft",
    )

    # Upper output bearing and its fixed bridge.
    upper_bearing = annulus(
        11.0,
        OUTPUT_RUNNING_BORE_R,
        UPPER_BEARING_H,
    ).moved(Location((0.0, 0.0, UPPER_BEARING_Z)))
    a.add(
        upper_bearing,
        "upper_output_bearing|dof=fixed|mount=upper_bearing_bridge",
    )

    bridge_z = UPPER_BEARING_Z - 1.0
    bridge_h = UPPER_BEARING_H + 2.0
    bridge = annulus(
        POST_CENTER_R + POST_R,
        11.0 - 0.005,
        bridge_h,
    ).moved(Location((0.0, 0.0, bridge_z)))
    a.add(
        bridge,
        "upper_bearing_bridge|dof=fixed|"
        "mount=support_post_1,support_post_2,support_post_3",
    )

    # Visible carrier-output pointer, press-fit around the output tube.
    pointer_pad = cylinder_min(11.0, OUTPUT_POINTER_H)
    pointer_arm = Box(
        OUTPUT_POINTER_LENGTH,
        6.0,
        OUTPUT_POINTER_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    pointer = pointer_pad + pointer_arm
    pointer_bore = Cylinder(
        OUTPUT_PRESS_BORE_R,
        OUTPUT_POINTER_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    pointer = (pointer - pointer_bore).moved(
        Location((0.0, 0.0, OUTPUT_POINTER_Z))
    )
    a.add(
        pointer,
        "output_pointer|dof=fixed|mount=output_shaft",
    )

    # Hand crank arm, press-fit to the projecting input shaft.
    crank_center_pad = cylinder_min(8.0, CRANK_H)
    crank_arm_box = Box(
        CRANK_R,
        8.0,
        CRANK_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    crank_end_pad = cylinder_min(7.0, CRANK_H).moved(
        Location((CRANK_R, 0.0, 0.0))
    )
    crank_arm = crank_center_pad + crank_arm_box + crank_end_pad

    crank_bore = Cylinder(
        INPUT_PRESS_BORE_R,
        CRANK_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    crank_arm = (crank_arm - crank_bore).moved(
        Location((0.0, 0.0, CRANK_Z))
    )
    a.add(
        crank_arm,
        "hand_crank_arm|dof=fixed|mount=input_shaft",
    )

    # Upright hand grip touches the crank arm's upper face.
    grip = cylinder_min(6.0, 18.0).moved(
        Location((CRANK_R, 0.0, CRANK_Z + CRANK_H))
    )
    a.add(
        grip,
        "hand_grip|dof=fixed|mount=hand_crank_arm",
    )

    return a.build()