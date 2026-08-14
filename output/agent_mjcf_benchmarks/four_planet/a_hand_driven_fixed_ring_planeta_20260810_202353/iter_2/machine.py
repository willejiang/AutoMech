import math

# ---------------------------------------------------------------------------
# Drivetrain arithmetic -- all gear locations derive from these values.
# ---------------------------------------------------------------------------
M = 1.0
Z_SUN = 16
Z_PLANET = 16
Z_RING = Z_SUN + 2 * Z_PLANET
PLANET_COUNT = 4

assert Z_RING == 48
assert PLANET_COUNT == 4
assert Z_SUN % PLANET_COUNT == 0
assert Z_RING % PLANET_COUNT == 0


def pitch_r(z):
    return M * z / 2.0


def center_dist_external(za, zb):
    return M * (za + zb) / 2.0


def center_dist_internal(z_internal, z_external):
    return M * (z_internal - z_external) / 2.0


SUN_PITCH_R = pitch_r(Z_SUN)
PLANET_PITCH_R = pitch_r(Z_PLANET)
RING_PITCH_R = pitch_r(Z_RING)

SUN_PLANET_CD = center_dist_external(Z_SUN, Z_PLANET)
RING_PLANET_CD = center_dist_internal(Z_RING, Z_PLANET)
assert abs(SUN_PLANET_CD - RING_PLANET_CD) < 1.0e-9

CARRIER_RADIUS = SUN_PLANET_CD
PLANET_ANGLES_DEG = tuple(i * 360.0 / PLANET_COUNT for i in range(PLANET_COUNT))
PLANET_CENTERS = tuple(
    (
        CARRIER_RADIUS * math.cos(math.radians(a)),
        CARRIER_RADIUS * math.sin(math.radians(a)),
    )
    for a in PLANET_ANGLES_DEG
)

PLANET_PHASE_DEG = 180.0 / Z_PLANET

FIXED_RING_REDUCTION = 1.0 + Z_RING / Z_SUN
assert abs(FIXED_RING_REDUCTION - 4.0) < 1.0e-9

# ---------------------------------------------------------------------------
# Axial stack and fits.
# ---------------------------------------------------------------------------
BASE_X = 90.0
BASE_Y = 82.0
BASE_H = 4.0

INPUT_SHAFT_R = 3.0
INPUT_SHAFT_Z = BASE_H
INPUT_SHAFT_H = 36.0

RUNNING_CLEARANCE = 0.05
PRESS_INTERFERENCE = 0.005

INPUT_RUNNING_BORE_R = INPUT_SHAFT_R + RUNNING_CLEARANCE
SUN_PRESS_BORE_R = INPUT_SHAFT_R - PRESS_INTERFERENCE

LOWER_INPUT_BEARING_Z = BASE_H
LOWER_INPUT_BEARING_H = 3.0
LOWER_INPUT_BEARING_OUTER_R = 5.0

OUTPUT_SHAFT_INNER_R = INPUT_SHAFT_R + RUNNING_CLEARANCE
OUTPUT_SHAFT_OUTER_R = 6.0
OUTPUT_SHAFT_Z = LOWER_INPUT_BEARING_Z + LOWER_INPUT_BEARING_H
OUTPUT_SHAFT_H = 10.0

OUTPUT_PEDESTAL_Z = BASE_H
OUTPUT_PEDESTAL_H = 4.0
OUTPUT_PEDESTAL_INNER_R = OUTPUT_SHAFT_OUTER_R + 0.20
OUTPUT_PEDESTAL_OUTER_R = 11.0

OUTPUT_BEARING_Z = OUTPUT_PEDESTAL_Z + OUTPUT_PEDESTAL_H
OUTPUT_BEARING_H = 4.0
OUTPUT_BEARING_BORE_R = OUTPUT_SHAFT_OUTER_R + RUNNING_CLEARANCE
OUTPUT_BEARING_OUTER_R = 9.0

OUTPUT_DIAL_Z = OUTPUT_BEARING_Z + OUTPUT_BEARING_H
OUTPUT_DIAL_H = 2.0
OUTPUT_DIAL_BORE_R = OUTPUT_SHAFT_OUTER_R - PRESS_INTERFERENCE
OUTPUT_DIAL_OUTER_R = 10.0
OUTPUT_DIAL_TAB_R = 2.0
OUTPUT_DIAL_TAB_X = 12.0

CARRIER_Z = OUTPUT_DIAL_Z + OUTPUT_DIAL_H
CARRIER_H = 3.0
CARRIER_OUTER_R = CARRIER_RADIUS + 6.0
CARRIER_OUTPUT_BORE_R = OUTPUT_SHAFT_OUTER_R - PRESS_INTERFERENCE

GEAR_CLEARANCE_ABOVE_CARRIER = 2.0
GEAR_Z = CARRIER_Z + CARRIER_H + GEAR_CLEARANCE_ABOVE_CARRIER
GEAR_FACE_W = 6.0
GEAR_TOP_Z = GEAR_Z + GEAR_FACE_W

PLANET_PIN_R = 2.0
PLANET_PIN_PRESS_BORE_R = PLANET_PIN_R - PRESS_INTERFERENCE
PLANET_RUNNING_BORE_R = PLANET_PIN_R + RUNNING_CLEARANCE
PLANET_PIN_Z = CARRIER_Z
PLANET_PIN_H = GEAR_TOP_Z - PLANET_PIN_Z

RING_TIP_R = RING_PITCH_R - M
RING_ROOT_R = RING_PITCH_R + 1.25 * M
RING_OUTER_R = RING_ROOT_R + 3.75
RING_SUPPORT_INNER_R = RING_ROOT_R + 0.55
RING_SUPPORT_OUTER_R = RING_OUTER_R + 0.75
RING_SUPPORT_Z = CARRIER_Z + CARRIER_H
RING_SUPPORT_H = GEAR_Z - RING_SUPPORT_Z

RING_POST_R = 2.5
RING_POST_ORBIT_R = (RING_SUPPORT_INNER_R + RING_SUPPORT_OUTER_R) / 2.0
RING_POST_Z = BASE_H
RING_POST_H = RING_SUPPORT_Z - RING_POST_Z
RING_POST_CENTERS = tuple(
    (
        RING_POST_ORBIT_R * math.cos(math.radians(a)),
        RING_POST_ORBIT_R * math.sin(math.radians(a)),
    )
    for a in PLANET_ANGLES_DEG
)

UPPER_BRIDGE_Z = GEAR_TOP_Z + 1.0
UPPER_BRIDGE_H = 1.0
UPPER_BRIDGE_X = 72.0
UPPER_BRIDGE_Y = 8.0
UPPER_BRIDGE_HOLE_R = INPUT_SHAFT_R + 0.10

UPPER_COLUMN_X = RING_OUTER_R + 5.0
UPPER_COLUMN_Y = 0.0
UPPER_COLUMN_SIZE_X = 4.0
UPPER_COLUMN_SIZE_Y = UPPER_BRIDGE_Y
UPPER_COLUMN_Z = BASE_H
UPPER_COLUMN_H = UPPER_BRIDGE_Z - UPPER_COLUMN_Z

UPPER_INPUT_BEARING_Z = UPPER_BRIDGE_Z + UPPER_BRIDGE_H
UPPER_INPUT_BEARING_H = 3.0
UPPER_INPUT_BEARING_OUTER_R = 5.0
UPPER_INPUT_BEARING_BORE_R = INPUT_SHAFT_R + RUNNING_CLEARANCE

CRANK_ARM_Z = UPPER_INPUT_BEARING_Z + UPPER_INPUT_BEARING_H + 1.0
CRANK_ARM_H = 2.0
CRANK_ARM_WIDTH = 6.0
CRANK_THROW = 22.0
CRANK_ARM_BORE_R = INPUT_SHAFT_R - PRESS_INTERFERENCE

HANDLE_R = 3.0
HANDLE_Z = CRANK_ARM_Z + CRANK_ARM_H
HANDLE_H = 8.0

RING_ROOT_HALF_ANGLE = 0.42 * math.pi / Z_RING
RING_TIP_HALF_ANGLE = 0.22 * math.pi / Z_RING

# ---------------------------------------------------------------------------
# Explicit mechanism semantics.
# ---------------------------------------------------------------------------
MECHANISM = {
    "name": "hand_driven_fixed_ring_planetary_reducer",
    "output_link": "carrier",
    "watch_links": [
        "input_shaft",
        "sun_gear",
        "planet_1",
        "planet_2",
        "planet_3",
        "planet_4",
        "carrier",
        "output_shaft",
    ],
    "ports_by_link": {
        "input_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, LOWER_INPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": LOWER_INPUT_BEARING_H,
            },
            {
                "name": "sun_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    GEAR_Z + GEAR_FACE_W / 2.0 - INPUT_SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    UPPER_INPUT_BEARING_Z
                    + UPPER_INPUT_BEARING_H / 2.0
                    - INPUT_SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": UPPER_INPUT_BEARING_H,
            },
        ],
        "lower_input_bearing": [
            {
                "name": "input_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, LOWER_INPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_RUNNING_BORE_R,
                "depth_mm": LOWER_INPUT_BEARING_H,
            }
        ],
        "upper_input_bearing": [
            {
                "name": "input_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, UPPER_INPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * UPPER_INPUT_BEARING_BORE_R,
                "depth_mm": UPPER_INPUT_BEARING_H,
            }
        ],
        "sun_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SUN_PRESS_BORE_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "sun_pitch",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": SUN_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "fixed_ring": [
            {
                "name": "internal_pitch",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": RING_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            }
        ],
        "carrier": [
            {
                "name": "output_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CARRIER_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CARRIER_OUTPUT_BORE_R,
                "depth_mm": CARRIER_H,
            },
            {
                "name": "pin_1_socket",
                "type": "bore",
                "xyz_mm": [
                    PLANET_CENTERS[0][0],
                    PLANET_CENTERS[0][1],
                    CARRIER_H / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_PIN_PRESS_BORE_R,
                "depth_mm": CARRIER_H,
            },
            {
                "name": "pin_2_socket",
                "type": "bore",
                "xyz_mm": [
                    PLANET_CENTERS[1][0],
                    PLANET_CENTERS[1][1],
                    CARRIER_H / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_PIN_PRESS_BORE_R,
                "depth_mm": CARRIER_H,
            },
            {
                "name": "pin_3_socket",
                "type": "bore",
                "xyz_mm": [
                    PLANET_CENTERS[2][0],
                    PLANET_CENTERS[2][1],
                    CARRIER_H / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_PIN_PRESS_BORE_R,
                "depth_mm": CARRIER_H,
            },
            {
                "name": "pin_4_socket",
                "type": "bore",
                "xyz_mm": [
                    PLANET_CENTERS[3][0],
                    PLANET_CENTERS[3][1],
                    CARRIER_H / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_PIN_PRESS_BORE_R,
                "depth_mm": CARRIER_H,
            },
        ],
        "output_shaft": [
            {
                "name": "carrier_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    CARRIER_Z + CARRIER_H / 2.0 - OUTPUT_SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_SHAFT_OUTER_R,
                "depth_mm": CARRIER_H,
            },
            {
                "name": "output_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    OUTPUT_BEARING_Z
                    + OUTPUT_BEARING_H / 2.0
                    - OUTPUT_SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_SHAFT_OUTER_R,
                "depth_mm": OUTPUT_BEARING_H,
            },
        ],
        "output_bearing": [
            {
                "name": "output_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, OUTPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_BEARING_BORE_R,
                "depth_mm": OUTPUT_BEARING_H,
            }
        ],
        "planet_pin_1": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, PLANET_PIN_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_PIN_R,
                "depth_mm": PLANET_PIN_H,
            }
        ],
        "planet_pin_2": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, PLANET_PIN_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_PIN_R,
                "depth_mm": PLANET_PIN_H,
            }
        ],
        "planet_pin_3": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, PLANET_PIN_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_PIN_R,
                "depth_mm": PLANET_PIN_H,
            }
        ],
        "planet_pin_4": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, PLANET_PIN_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_PIN_R,
                "depth_mm": PLANET_PIN_H,
            }
        ],
        "planet_1": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_RUNNING_BORE_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "planet_pitch",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": PLANET_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "planet_2": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_RUNNING_BORE_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "planet_pitch",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": PLANET_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "planet_3": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_RUNNING_BORE_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "planet_pitch",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": PLANET_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "planet_4": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PLANET_RUNNING_BORE_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "planet_pitch",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": PLANET_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
    },
    "relations": [
        {
            "name": "lower_input_journal",
            "mate_type": "journal_bearing",
            "base_part": "lower_input_bearing",
            "base_port": "input_bore",
            "incoming_part": "input_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "upper_input_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_input_bearing",
            "base_port": "input_bore",
            "incoming_part": "input_shaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "sun_press_fit",
            "mate_type": "press_fit",
            "base_part": "sun_gear",
            "base_port": "shaft_bore",
            "incoming_part": "input_shaft",
            "incoming_port": "sun_seat",
        },
        {
            "name": "output_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_bearing",
            "base_port": "output_bore",
            "incoming_part": "output_shaft",
            "incoming_port": "output_journal",
        },
        {
            "name": "carrier_output_press_fit",
            "mate_type": "press_fit",
            "base_part": "carrier",
            "base_port": "output_bore",
            "incoming_part": "output_shaft",
            "incoming_port": "carrier_seat",
        },
        {
            "name": "carrier_pin_1_press_fit",
            "mate_type": "press_fit",
            "base_part": "carrier",
            "base_port": "pin_1_socket",
            "incoming_part": "planet_pin_1",
            "incoming_port": "pin_axis",
        },
        {
            "name": "carrier_pin_2_press_fit",
            "mate_type": "press_fit",
            "base_part": "carrier",
            "base_port": "pin_2_socket",
            "incoming_part": "planet_pin_2",
            "incoming_port": "pin_axis",
        },
        {
            "name": "carrier_pin_3_press_fit",
            "mate_type": "press_fit",
            "base_part": "carrier",
            "base_port": "pin_3_socket",
            "incoming_part": "planet_pin_3",
            "incoming_port": "pin_axis",
        },
        {
            "name": "carrier_pin_4_press_fit",
            "mate_type": "press_fit",
            "base_part": "carrier",
            "base_port": "pin_4_socket",
            "incoming_part": "planet_pin_4",
            "incoming_port": "pin_axis",
        },
        {
            "name": "planet_1_journal",
            "mate_type": "journal_bearing",
            "base_part": "planet_1",
            "base_port": "pin_bore",
            "incoming_part": "planet_pin_1",
            "incoming_port": "pin_axis",
        },
        {
            "name": "planet_2_journal",
            "mate_type": "journal_bearing",
            "base_part": "planet_2",
            "base_port": "pin_bore",
            "incoming_part": "planet_pin_2",
            "incoming_port": "pin_axis",
        },
        {
            "name": "planet_3_journal",
            "mate_type": "journal_bearing",
            "base_part": "planet_3",
            "base_port": "pin_bore",
            "incoming_part": "planet_pin_3",
            "incoming_port": "pin_axis",
        },
        {
            "name": "planet_4_journal",
            "mate_type": "journal_bearing",
            "base_part": "planet_4",
            "base_port": "pin_bore",
            "incoming_part": "planet_pin_4",
            "incoming_port": "pin_axis",
        },
    ],
    "motion_joints": [
        {
            "name": "input_shaft_hinge",
            "parent": "",
            "child": "input_shaft",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "sun_hinge",
            "parent": "",
            "child": "sun_gear",
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
        {
            "name": "output_shaft_hinge",
            "parent": "",
            "child": "output_shaft",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "planet_1_carrier_hinge",
            "parent": "carrier",
            "child": "planet_1",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "planet_2_carrier_hinge",
            "parent": "carrier",
            "child": "planet_2",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "planet_3_carrier_hinge",
            "parent": "carrier",
            "child": "planet_3",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "planet_4_carrier_hinge",
            "parent": "carrier",
            "child": "planet_4",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
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
            "ring": "fixed_ring",
            "carrier": "carrier",
            "planets": [
                {"gear": "planet_1", "pin": "planet_pin_1"},
                {"gear": "planet_2", "pin": "planet_pin_2"},
                {"gear": "planet_3", "pin": "planet_pin_3"},
                {"gear": "planet_4", "pin": "planet_pin_4"},
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


def _annulus(outer_r, inner_r, height):
    outer = Cylinder(
        outer_r,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    cutter = Cylinder(
        inner_r,
        height + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    return outer - cutter


def _make_internal_ring():
    with BuildPart() as ring_part:
        with BuildSketch() as ring_sketch:
            Circle(RING_OUTER_R)
            Circle(RING_ROOT_R, mode=Mode.SUBTRACT)

            for tooth_index in range(Z_RING):
                a = 2.0 * math.pi * tooth_index / Z_RING
                points = [
                    (
                        RING_ROOT_R * math.cos(a - RING_ROOT_HALF_ANGLE),
                        RING_ROOT_R * math.sin(a - RING_ROOT_HALF_ANGLE),
                    ),
                    (
                        RING_TIP_R * math.cos(a - RING_TIP_HALF_ANGLE),
                        RING_TIP_R * math.sin(a - RING_TIP_HALF_ANGLE),
                    ),
                    (
                        RING_TIP_R * math.cos(a + RING_TIP_HALF_ANGLE),
                        RING_TIP_R * math.sin(a + RING_TIP_HALF_ANGLE),
                    ),
                    (
                        RING_ROOT_R * math.cos(a + RING_ROOT_HALF_ANGLE),
                        RING_ROOT_R * math.sin(a + RING_ROOT_HALF_ANGLE),
                    ),
                ]
                Polygon(*points, mode=Mode.ADD)

        extrude(amount=GEAR_FACE_W)

    return ring_part.part


def _make_carrier():
    carrier = Cylinder(
        CARRIER_OUTER_R,
        CARRIER_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    cutters = Cylinder(
        CARRIER_OUTPUT_BORE_R,
        CARRIER_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))

    for x, y in PLANET_CENTERS:
        pin_cutter = Cylinder(
            PLANET_PIN_PRESS_BORE_R,
            CARRIER_H + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((x, y, -1.0)))
        cutters = cutters + pin_cutter

    return carrier - cutters


def _make_output_dial():
    dial = _annulus(
        OUTPUT_DIAL_OUTER_R,
        OUTPUT_DIAL_BORE_R,
        OUTPUT_DIAL_H,
    )
    pointer_tab = Cylinder(
        OUTPUT_DIAL_TAB_R,
        OUTPUT_DIAL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((OUTPUT_DIAL_TAB_X, 0.0, 0.0)))
    pointer_bridge = Box(
        OUTPUT_DIAL_TAB_X,
        2.0 * OUTPUT_DIAL_TAB_R,
        OUTPUT_DIAL_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    return dial + pointer_bridge + pointer_tab


def _make_crank_arm():
    arm = Box(
        CRANK_THROW,
        CRANK_ARM_WIDTH,
        CRANK_ARM_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    root_end = Cylinder(
        CRANK_ARM_WIDTH / 2.0,
        CRANK_ARM_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    handle_end = Cylinder(
        CRANK_ARM_WIDTH / 2.0,
        CRANK_ARM_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_THROW, 0.0, 0.0)))

    arm = arm + root_end + handle_end

    bore_tool = Cylinder(
        CRANK_ARM_BORE_R,
        CRANK_ARM_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    return arm - bore_tool


def build_machine():
    a = AssemblyHelper("hand_driven_fixed_ring_planetary_reducer")

    baseplate = Box(
        BASE_X,
        BASE_Y,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(baseplate, "baseplate|dof=fixed")

    lower_input_bearing = _annulus(
        LOWER_INPUT_BEARING_OUTER_R,
        INPUT_RUNNING_BORE_R,
        LOWER_INPUT_BEARING_H,
    ).moved(Location((0.0, 0.0, LOWER_INPUT_BEARING_Z)))
    a.add(
        lower_input_bearing,
        "lower_input_bearing|dof=fixed|mount=baseplate",
    )

    output_pedestal = _annulus(
        OUTPUT_PEDESTAL_OUTER_R,
        OUTPUT_PEDESTAL_INNER_R,
        OUTPUT_PEDESTAL_H,
    ).moved(Location((0.0, 0.0, OUTPUT_PEDESTAL_Z)))
    a.add(
        output_pedestal,
        "output_bearing_pedestal|dof=fixed|mount=baseplate",
    )

    output_bearing = _annulus(
        OUTPUT_BEARING_OUTER_R,
        OUTPUT_BEARING_BORE_R,
        OUTPUT_BEARING_H,
    ).moved(Location((0.0, 0.0, OUTPUT_BEARING_Z)))
    a.add(
        output_bearing,
        "output_bearing|dof=fixed|mount=output_bearing_pedestal",
    )

    input_shaft = Cylinder(
        INPUT_SHAFT_R,
        INPUT_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, INPUT_SHAFT_Z)))
    a.add(
        input_shaft,
        "input_shaft|dof=spin|driver=True|spin_axis=z|"
        "mount=lower_input_bearing,upper_input_bearing",
    )

    output_shaft = _annulus(
        OUTPUT_SHAFT_OUTER_R,
        OUTPUT_SHAFT_INNER_R,
        OUTPUT_SHAFT_H,
    ).moved(Location((0.0, 0.0, OUTPUT_SHAFT_Z)))
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=z|mount=output_bearing",
    )

    output_dial = _make_output_dial().moved(
        Location((0.0, 0.0, OUTPUT_DIAL_Z))
    )
    a.add(
        output_dial,
        "output_dial|dof=fixed|mount=output_shaft",
    )

    carrier = _make_carrier().moved(Location((0.0, 0.0, CARRIER_Z)))
    a.add(
        carrier,
        "carrier|dof=spin|spin_axis=z|mount=output_shaft",
    )

    ring_posts = []
    for index, (x, y) in enumerate(RING_POST_CENTERS, start=1):
        post = Cylinder(
            RING_POST_R,
            RING_POST_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((x, y, RING_POST_Z)))
        name = f"ring_post_{index}"
        a.add(post, f"{name}|dof=fixed|mount=baseplate")
        ring_posts.append(name)

    ring_support = _annulus(
        RING_SUPPORT_OUTER_R,
        RING_SUPPORT_INNER_R,
        RING_SUPPORT_H,
    ).moved(Location((0.0, 0.0, RING_SUPPORT_Z)))
    a.add(
        ring_support,
        "ring_support_shelf|dof=fixed|mount=" + ",".join(ring_posts),
    )

    # Only corrected part: custom extrusion starts at local z=0.
    fixed_ring = _make_internal_ring().moved(
        Location((0.0, 0.0, GEAR_Z + GEAR_FACE_W / 2.0))
    )
    a.add(
        fixed_ring,
        "fixed_ring|dof=fixed|mount=ring_support_shelf",
    )

    sun_gear = make_gear(
        M,
        Z_SUN,
        GEAR_FACE_W,
        2.0 * SUN_PRESS_BORE_R,
    ).moved(Location((0.0, 0.0, GEAR_Z)))
    a.add(
        sun_gear,
        "sun_gear|dof=spin|spin_axis=z|mount=input_shaft",
    )

    for index, ((x, y), angle_deg) in enumerate(
        zip(PLANET_CENTERS, PLANET_ANGLES_DEG),
        start=1,
    ):
        pin_name = f"planet_pin_{index}"
        planet_name = f"planet_{index}"

        pin = Cylinder(
            PLANET_PIN_R,
            PLANET_PIN_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((x, y, PLANET_PIN_Z)))
        a.add(
            pin,
            f"{pin_name}|dof=fixed|mount=carrier",
        )

        planet_rotation = PLANET_PHASE_DEG - angle_deg
        planet = make_gear(
            M,
            Z_PLANET,
            GEAR_FACE_W,
            2.0 * PLANET_RUNNING_BORE_R,
        ).moved(
            Location(
                (x, y, GEAR_Z),
                (0.0, 0.0, planet_rotation),
            )
        )
        a.add(
            planet,
            f"{planet_name}|dof=spin|spin_axis=z|mount={pin_name}",
        )

    left_upper_column = Box(
        UPPER_COLUMN_SIZE_X,
        UPPER_COLUMN_SIZE_Y,
        UPPER_COLUMN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((-UPPER_COLUMN_X, UPPER_COLUMN_Y, UPPER_COLUMN_Z))
    )
    a.add(
        left_upper_column,
        "upper_bridge_column_left|dof=fixed|mount=baseplate",
    )

    right_upper_column = Box(
        UPPER_COLUMN_SIZE_X,
        UPPER_COLUMN_SIZE_Y,
        UPPER_COLUMN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((UPPER_COLUMN_X, UPPER_COLUMN_Y, UPPER_COLUMN_Z))
    )
    a.add(
        right_upper_column,
        "upper_bridge_column_right|dof=fixed|mount=baseplate",
    )

    upper_bridge = Box(
        UPPER_BRIDGE_X,
        UPPER_BRIDGE_Y,
        UPPER_BRIDGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    bridge_bore = Cylinder(
        UPPER_BRIDGE_HOLE_R,
        UPPER_BRIDGE_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    upper_bridge = (upper_bridge - bridge_bore).moved(
        Location((0.0, 0.0, UPPER_BRIDGE_Z))
    )
    a.add(
        upper_bridge,
        "upper_bridge|dof=fixed|"
        "mount=upper_bridge_column_left,upper_bridge_column_right",
    )

    upper_input_bearing = _annulus(
        UPPER_INPUT_BEARING_OUTER_R,
        UPPER_INPUT_BEARING_BORE_R,
        UPPER_INPUT_BEARING_H,
    ).moved(Location((0.0, 0.0, UPPER_INPUT_BEARING_Z)))
    a.add(
        upper_input_bearing,
        "upper_input_bearing|dof=fixed|mount=upper_bridge",
    )

    crank_arm = _make_crank_arm().moved(
        Location((0.0, 0.0, CRANK_ARM_Z))
    )
    a.add(
        crank_arm,
        "crank_arm|dof=fixed|mount=input_shaft",
    )

    hand_handle = Cylinder(
        HANDLE_R,
        HANDLE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_THROW, 0.0, HANDLE_Z)))
    a.add(
        hand_handle,
        "hand_handle|dof=fixed|mount=crank_arm",
    )

    return a.build()