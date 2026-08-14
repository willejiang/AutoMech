import math

# ---------------------------------------------------------------------------
# Drivetrain arithmetic — all gear locations derive from these values.
# ---------------------------------------------------------------------------

M = 0.8

Z_MINUTE = 15
Z_COMPOUND_BIG = 45
Z_COMPOUND_PINION = 12
Z_HOUR = 48

GEAR_FACE = 4.0

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

CD_STAGE_1 = center_dist(Z_MINUTE, Z_COMPOUND_BIG)
CD_STAGE_2 = center_dist(Z_COMPOUND_PINION, Z_HOUR)

assert abs(CD_STAGE_1 - CD_STAGE_2) < 1e-9
SHAFT_CENTER_DISTANCE = CD_STAGE_1

STAGE_1_REDUCTION = Z_COMPOUND_BIG / Z_MINUTE
STAGE_2_REDUCTION = Z_HOUR / Z_COMPOUND_PINION
TOTAL_REDUCTION = STAGE_1_REDUCTION * STAGE_2_REDUCTION
assert abs(TOTAL_REDUCTION - 12.0) < 1e-9

MINUTE_CENTER_X = 0.0
MINUTE_CENTER_Y = 0.0
INTERMEDIATE_CENTER_X = MINUTE_CENTER_X + SHAFT_CENTER_DISTANCE
INTERMEDIATE_CENTER_Y = MINUTE_CENTER_Y

# Approximate external-gear envelope used only to size the base and supports.
MINUTE_GEAR_OUTER_R = pitch_r(Z_MINUTE) + 1.25 * M
COMPOUND_BIG_OUTER_R = pitch_r(Z_COMPOUND_BIG) + 1.25 * M
COMPOUND_PINION_OUTER_R = pitch_r(Z_COMPOUND_PINION) + 1.25 * M
HOUR_GEAR_OUTER_R = pitch_r(Z_HOUR) + 1.25 * M

# ---------------------------------------------------------------------------
# Fits and axial stations.
# All primitive solids made below use a local z-minimum at z=0.
# ---------------------------------------------------------------------------

BASE_H = 4.0

MINUTE_SHAFT_R = 2.0
INTERMEDIATE_SHAFT_R = 2.0

HOUR_SLEEVE_INNER_R = MINUTE_SHAFT_R + 0.05
HOUR_SLEEVE_OUTER_R = 3.0

MINUTE_GEAR_BORE_R = MINUTE_SHAFT_R - 0.005
COMPOUND_GEAR_BORE_R = INTERMEDIATE_SHAFT_R - 0.005
HOUR_GEAR_BORE_R = HOUR_SLEEVE_OUTER_R - 0.005

LOWER_BEARING_CLEARANCE = 0.05
MINUTE_BEARING_BORE_R = MINUTE_SHAFT_R + LOWER_BEARING_CLEARANCE
INTERMEDIATE_BEARING_BORE_R = INTERMEDIATE_SHAFT_R + LOWER_BEARING_CLEARANCE
SLEEVE_BEARING_BORE_R = HOUR_SLEEVE_OUTER_R + LOWER_BEARING_CLEARANCE

LOWER_BEARING_Z = BASE_H
LOWER_BEARING_H = 3.0
LOWER_BEARING_OUTER_R = 4.5

STAGE_1_Z = LOWER_BEARING_Z + LOWER_BEARING_H + 0.5
STAGE_1_TOP = STAGE_1_Z + GEAR_FACE

THRUST_WASHER_Z = STAGE_1_TOP + 0.6
THRUST_WASHER_H = 0.5
THRUST_WASHER_TOP = THRUST_WASHER_Z + THRUST_WASHER_H
THRUST_WASHER_INNER_R = HOUR_SLEEVE_INNER_R + 0.05
THRUST_WASHER_OUTER_R = 4.0

HOUR_SLEEVE_Z = THRUST_WASHER_TOP
HOUR_SLEEVE_TOP = 30.0
HOUR_SLEEVE_H = HOUR_SLEEVE_TOP - HOUR_SLEEVE_Z

LOWER_SLEEVE_BEARING_Z = HOUR_SLEEVE_Z + 0.5
LOWER_SLEEVE_BEARING_H = 3.0
SLEEVE_BEARING_OUTER_R = 4.8

STAGE_2_Z = LOWER_SLEEVE_BEARING_Z + LOWER_SLEEVE_BEARING_H + 0.9
STAGE_2_TOP = STAGE_2_Z + GEAR_FACE

UPPER_SLEEVE_BEARING_Z = STAGE_2_TOP + 1.0
UPPER_SLEEVE_BEARING_H = 3.0

HOUR_HAND_Z = UPPER_SLEEVE_BEARING_Z + UPPER_SLEEVE_BEARING_H + 1.0
HOUR_HAND_T = 1.0
MINUTE_HAND_Z = HOUR_SLEEVE_TOP + 0.2
MINUTE_HAND_T = 1.0

MINUTE_SHAFT_Z = BASE_H
MINUTE_SHAFT_TOP = MINUTE_HAND_Z + MINUTE_HAND_T + 1.0
MINUTE_SHAFT_H = MINUTE_SHAFT_TOP - MINUTE_SHAFT_Z

INTERMEDIATE_SHAFT_Z = BASE_H
INTERMEDIATE_SHAFT_TOP = STAGE_2_TOP + 1.0
INTERMEDIATE_SHAFT_H = INTERMEDIATE_SHAFT_TOP - INTERMEDIATE_SHAFT_Z

# Open frame dimensions derived from the rotating envelopes.
BASE_MARGIN = 4.0
BASE_X_MIN = min(
    MINUTE_CENTER_X - HOUR_GEAR_OUTER_R,
    INTERMEDIATE_CENTER_X - COMPOUND_BIG_OUTER_R,
) - BASE_MARGIN
BASE_X_MAX = max(
    MINUTE_CENTER_X + HOUR_GEAR_OUTER_R,
    INTERMEDIATE_CENTER_X + COMPOUND_BIG_OUTER_R,
) + BASE_MARGIN
BASE_Y_MIN = -HOUR_GEAR_OUTER_R - 8.0
BASE_Y_MAX = HOUR_GEAR_OUTER_R + BASE_MARGIN

BASE_L = BASE_X_MAX - BASE_X_MIN
BASE_W = BASE_Y_MAX - BASE_Y_MIN
BASE_CX = (BASE_X_MIN + BASE_X_MAX) / 2.0
BASE_CY = (BASE_Y_MIN + BASE_Y_MAX) / 2.0

REAR_MAST_Y = BASE_Y_MIN + 3.0
REAR_MAST_W = 3.0
REAR_MAST_D = 3.0
REAR_MAST_Z = BASE_H
REAR_MAST_TOP = UPPER_SLEEVE_BEARING_Z + UPPER_SLEEVE_BEARING_H + 1.0
REAR_MAST_H = REAR_MAST_TOP - REAR_MAST_Z

BRACKET_X_W = 3.0
BRACKET_Y_END = MINUTE_CENTER_Y - SLEEVE_BEARING_OUTER_R
BRACKET_Y_START = REAR_MAST_Y
BRACKET_LEN = BRACKET_Y_END - BRACKET_Y_START
BRACKET_CENTER_Y = (BRACKET_Y_START + BRACKET_Y_END) / 2.0

LOWER_BRACKET_Z = LOWER_SLEEVE_BEARING_Z
LOWER_BRACKET_H = LOWER_SLEEVE_BEARING_H
UPPER_BRACKET_Z = UPPER_SLEEVE_BEARING_Z
UPPER_BRACKET_H = UPPER_SLEEVE_BEARING_H

MINUTE_HAND_LENGTH = 18.0
MINUTE_HAND_WIDTH = 1.8
MINUTE_HAND_HUB_R = 3.2
MINUTE_HAND_BORE_R = MINUTE_SHAFT_R - 0.005

HOUR_HAND_LENGTH = 12.0
HOUR_HAND_WIDTH = 2.2
HOUR_HAND_HUB_R = 4.0
HOUR_HAND_BORE_R = HOUR_SLEEVE_OUTER_R - 0.005


MECHANISM = {
    "name": "openwork_12_to_1_clock_display",
    "output_link": "hour_hand",
    "watch_links": [
        "minute_gear",
        "compound_big_gear",
        "compound_pinion",
        "hour_gear",
        "minute_hand",
        "hour_hand",
    ],
    "ports_by_link": {
        "minute_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * MINUTE_GEAR_BORE_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage1_mesh",
                "type": "gear_mesh",
                "xyz_mm": [pitch_r(Z_MINUTE), 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_MINUTE),
                "depth_mm": GEAR_FACE,
            },
        ],
        "compound_big_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * COMPOUND_GEAR_BORE_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage1_mesh",
                "type": "gear_mesh",
                "xyz_mm": [-pitch_r(Z_COMPOUND_BIG), 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_COMPOUND_BIG),
                "depth_mm": GEAR_FACE,
            },
        ],
        "compound_pinion": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * COMPOUND_GEAR_BORE_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage2_mesh",
                "type": "gear_mesh",
                "xyz_mm": [-pitch_r(Z_COMPOUND_PINION), 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_COMPOUND_PINION),
                "depth_mm": GEAR_FACE,
            },
        ],
        "hour_gear": [
            {
                "name": "sleeve_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * HOUR_GEAR_BORE_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage2_mesh",
                "type": "gear_mesh",
                "xyz_mm": [pitch_r(Z_HOUR), 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_HOUR),
                "depth_mm": GEAR_FACE,
            },
        ],
        "minute_shaft": [
            {
                "name": "minute_gear_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    STAGE_1_Z + GEAR_FACE / 2.0 - MINUTE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * MINUTE_SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    LOWER_BEARING_Z + LOWER_BEARING_H / 2.0 - MINUTE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * MINUTE_SHAFT_R,
                "depth_mm": LOWER_BEARING_H,
            },
            {
                "name": "sleeve_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    HOUR_SLEEVE_Z + HOUR_SLEEVE_H / 2.0 - MINUTE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * MINUTE_SHAFT_R,
                "depth_mm": HOUR_SLEEVE_H,
            },
            {
                "name": "minute_hand_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    MINUTE_HAND_Z + MINUTE_HAND_T / 2.0 - MINUTE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * MINUTE_SHAFT_R,
                "depth_mm": MINUTE_HAND_T,
            },
        ],
        "intermediate_shaft": [
            {
                "name": "big_gear_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    STAGE_1_Z + GEAR_FACE / 2.0 - INTERMEDIATE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * INTERMEDIATE_SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "pinion_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    STAGE_2_Z + GEAR_FACE / 2.0 - INTERMEDIATE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * INTERMEDIATE_SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    LOWER_BEARING_Z + LOWER_BEARING_H / 2.0 - INTERMEDIATE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * INTERMEDIATE_SHAFT_R,
                "depth_mm": LOWER_BEARING_H,
            },
        ],
        "hour_sleeve": [
            {
                "name": "inner_journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, HOUR_SLEEVE_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * HOUR_SLEEVE_INNER_R,
                "depth_mm": HOUR_SLEEVE_H,
            },
            {
                "name": "lower_outer_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    LOWER_SLEEVE_BEARING_Z
                    + LOWER_SLEEVE_BEARING_H / 2.0
                    - HOUR_SLEEVE_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * HOUR_SLEEVE_OUTER_R,
                "depth_mm": LOWER_SLEEVE_BEARING_H,
            },
            {
                "name": "upper_outer_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    UPPER_SLEEVE_BEARING_Z
                    + UPPER_SLEEVE_BEARING_H / 2.0
                    - HOUR_SLEEVE_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * HOUR_SLEEVE_OUTER_R,
                "depth_mm": UPPER_SLEEVE_BEARING_H,
            },
            {
                "name": "hour_gear_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    STAGE_2_Z + GEAR_FACE / 2.0 - HOUR_SLEEVE_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * HOUR_SLEEVE_OUTER_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "hour_hand_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    HOUR_HAND_Z + HOUR_HAND_T / 2.0 - HOUR_SLEEVE_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * HOUR_SLEEVE_OUTER_R,
                "depth_mm": HOUR_HAND_T,
            },
        ],
        "minute_input_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, LOWER_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * MINUTE_BEARING_BORE_R,
                "depth_mm": LOWER_BEARING_H,
            }
        ],
        "intermediate_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, LOWER_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * INTERMEDIATE_BEARING_BORE_R,
                "depth_mm": LOWER_BEARING_H,
            }
        ],
        "lower_sleeve_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, LOWER_SLEEVE_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SLEEVE_BEARING_BORE_R,
                "depth_mm": LOWER_SLEEVE_BEARING_H,
            }
        ],
        "upper_sleeve_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, UPPER_SLEEVE_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SLEEVE_BEARING_BORE_R,
                "depth_mm": UPPER_SLEEVE_BEARING_H,
            }
        ],
        "minute_hand": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, MINUTE_HAND_T / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * MINUTE_HAND_BORE_R,
                "depth_mm": MINUTE_HAND_T,
            }
        ],
        "hour_hand": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, HOUR_HAND_T / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * HOUR_HAND_BORE_R,
                "depth_mm": HOUR_HAND_T,
            }
        ],
    },
    "relations": [
        {
            "name": "stage1_tooth_contact",
            "mate_type": "gear_spur_external",
            "base_part": "minute_gear",
            "base_port": "stage1_mesh",
            "incoming_part": "compound_big_gear",
            "incoming_port": "stage1_mesh",
            "separation_axis": "+x",
        },
        {
            "name": "stage2_tooth_contact",
            "mate_type": "gear_spur_external",
            "base_part": "hour_gear",
            "base_port": "stage2_mesh",
            "incoming_part": "compound_pinion",
            "incoming_port": "stage2_mesh",
            "separation_axis": "+x",
        },
        {
            "name": "minute_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "minute_shaft",
            "base_port": "minute_gear_seat",
            "incoming_part": "minute_gear",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "compound_big_press_fit",
            "mate_type": "press_fit",
            "base_part": "intermediate_shaft",
            "base_port": "big_gear_seat",
            "incoming_part": "compound_big_gear",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "compound_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "intermediate_shaft",
            "base_port": "pinion_seat",
            "incoming_part": "compound_pinion",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "hour_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "hour_sleeve",
            "base_port": "hour_gear_seat",
            "incoming_part": "hour_gear",
            "incoming_port": "sleeve_bore",
        },
        {
            "name": "minute_hand_press_fit",
            "mate_type": "press_fit",
            "base_part": "minute_shaft",
            "base_port": "minute_hand_seat",
            "incoming_part": "minute_hand",
            "incoming_port": "hub_bore",
        },
        {
            "name": "hour_hand_press_fit",
            "mate_type": "press_fit",
            "base_part": "hour_sleeve",
            "base_port": "hour_hand_seat",
            "incoming_part": "hour_hand",
            "incoming_port": "hub_bore",
        },
        {
            "name": "minute_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "minute_input_bearing",
            "base_port": "journal_bore",
            "incoming_part": "minute_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "intermediate_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "intermediate_bearing",
            "base_port": "journal_bore",
            "incoming_part": "intermediate_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "hour_sleeve_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "lower_sleeve_bearing",
            "base_port": "journal_bore",
            "incoming_part": "hour_sleeve",
            "incoming_port": "lower_outer_journal",
        },
        {
            "name": "hour_sleeve_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_sleeve_bearing",
            "base_port": "journal_bore",
            "incoming_part": "hour_sleeve",
            "incoming_port": "upper_outer_journal",
        },
        {
            "name": "independent_coaxial_handshaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "hour_sleeve",
            "base_port": "inner_journal",
            "incoming_part": "minute_shaft",
            "incoming_port": "sleeve_journal",
        },
    ],
    "motion_joints": [],
    "transmissions": [
        {
            "name": "minute_to_compound_reduction",
            "type": "gear_external",
            "driving_link": "minute_gear",
            "driven_link": "compound_big_gear",
            "ratio": 0,
        },
        {
            "name": "rigid_compound_pair",
            "type": "compound_1to1",
            "driving_link": "compound_big_gear",
            "driven_link": "compound_pinion",
            "ratio": 1.0,
        },
        {
            "name": "compound_to_hour_reduction",
            "type": "gear_external",
            "driving_link": "compound_pinion",
            "driven_link": "hour_gear",
            "ratio": 0,
        },
    ],
    "planetary_stages": [],
}


def build_machine():
    def annulus(outer_r, inner_r, height):
        outer = Cylinder(
            outer_r,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        cutter = Cylinder(
            inner_r,
            height + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -0.5)))
        return outer - cutter

    def clock_hand(length, width, hub_r, bore_r, thickness):
        neck_y = 0.45 * hub_r
        with BuildPart() as hand_part:
            with BuildSketch(Plane.XY):
                Circle(hub_r)
                Polygon(
                    (-width / 2.0, neck_y),
                    (-0.65 * width, 0.78 * length),
                    (0.0, length),
                    (0.65 * width, 0.78 * length),
                    (width / 2.0, neck_y),
                    mode=Mode.ADD,
                )
                Circle(bore_r, mode=Mode.SUBTRACT)
            extrude(amount=thickness)
        return hand_part.part

    a = AssemblyHelper("openwork_12_to_1_clock_display")

    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((BASE_CX, BASE_CY, 0.0)))
    a.add(base, "base|dof=fixed")

    minute_input_bearing = annulus(
        LOWER_BEARING_OUTER_R,
        MINUTE_BEARING_BORE_R,
        LOWER_BEARING_H,
    ).moved(
        Location((MINUTE_CENTER_X, MINUTE_CENTER_Y, LOWER_BEARING_Z))
    )
    a.add(
        minute_input_bearing,
        "minute_input_bearing|dof=fixed|mount=base",
    )

    intermediate_bearing = annulus(
        LOWER_BEARING_OUTER_R,
        INTERMEDIATE_BEARING_BORE_R,
        LOWER_BEARING_H,
    ).moved(
        Location(
            (
                INTERMEDIATE_CENTER_X,
                INTERMEDIATE_CENTER_Y,
                LOWER_BEARING_Z,
            )
        )
    )
    a.add(
        intermediate_bearing,
        "intermediate_bearing|dof=fixed|mount=base",
    )

    rear_mast = Box(
        REAR_MAST_W,
        REAR_MAST_D,
        REAR_MAST_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((MINUTE_CENTER_X, REAR_MAST_Y, REAR_MAST_Z))
    )
    a.add(rear_mast, "rear_mast|dof=fixed|mount=base")

    lower_sleeve_bracket = Box(
        BRACKET_X_W,
        BRACKET_LEN,
        LOWER_BRACKET_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                MINUTE_CENTER_X,
                BRACKET_CENTER_Y,
                LOWER_BRACKET_Z,
            )
        )
    )
    a.add(
        lower_sleeve_bracket,
        "lower_sleeve_bracket|dof=fixed|mount=rear_mast",
    )

    upper_sleeve_bracket = Box(
        BRACKET_X_W,
        BRACKET_LEN,
        UPPER_BRACKET_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                MINUTE_CENTER_X,
                BRACKET_CENTER_Y,
                UPPER_BRACKET_Z,
            )
        )
    )
    a.add(
        upper_sleeve_bracket,
        "upper_sleeve_bracket|dof=fixed|mount=rear_mast",
    )

    sleeve_thrust_washer = annulus(
        THRUST_WASHER_OUTER_R,
        THRUST_WASHER_INNER_R,
        THRUST_WASHER_H,
    ).moved(
        Location((MINUTE_CENTER_X, MINUTE_CENTER_Y, THRUST_WASHER_Z))
    )
    a.add(
        sleeve_thrust_washer,
        "sleeve_thrust_washer|dof=fixed|mount=lower_sleeve_bracket",
    )

    lower_sleeve_bearing = annulus(
        SLEEVE_BEARING_OUTER_R,
        SLEEVE_BEARING_BORE_R,
        LOWER_SLEEVE_BEARING_H,
    ).moved(
        Location(
            (
                MINUTE_CENTER_X,
                MINUTE_CENTER_Y,
                LOWER_SLEEVE_BEARING_Z,
            )
        )
    )
    a.add(
        lower_sleeve_bearing,
        "lower_sleeve_bearing|dof=fixed|mount=lower_sleeve_bracket",
    )

    upper_sleeve_bearing = annulus(
        SLEEVE_BEARING_OUTER_R,
        SLEEVE_BEARING_BORE_R,
        UPPER_SLEEVE_BEARING_H,
    ).moved(
        Location(
            (
                MINUTE_CENTER_X,
                MINUTE_CENTER_Y,
                UPPER_SLEEVE_BEARING_Z,
            )
        )
    )
    a.add(
        upper_sleeve_bearing,
        "upper_sleeve_bearing|dof=fixed|mount=upper_sleeve_bracket",
    )

    minute_shaft = Cylinder(
        MINUTE_SHAFT_R,
        MINUTE_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((MINUTE_CENTER_X, MINUTE_CENTER_Y, MINUTE_SHAFT_Z))
    )
    a.add(
        minute_shaft,
        "minute_shaft|dof=spin|spin_axis=z|"
        "mount=minute_input_bearing,hour_sleeve",
    )

    intermediate_shaft = Cylinder(
        INTERMEDIATE_SHAFT_R,
        INTERMEDIATE_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                INTERMEDIATE_CENTER_X,
                INTERMEDIATE_CENTER_Y,
                INTERMEDIATE_SHAFT_Z,
            )
        )
    )
    a.add(
        intermediate_shaft,
        "intermediate_shaft|dof=spin|spin_axis=z|mount=intermediate_bearing",
    )

    hour_sleeve = annulus(
        HOUR_SLEEVE_OUTER_R,
        HOUR_SLEEVE_INNER_R,
        HOUR_SLEEVE_H,
    ).moved(
        Location((MINUTE_CENTER_X, MINUTE_CENTER_Y, HOUR_SLEEVE_Z))
    )
    a.add(
        hour_sleeve,
        "hour_sleeve|dof=spin|spin_axis=z|"
        "mount=lower_sleeve_bearing,upper_sleeve_bearing,sleeve_thrust_washer",
    )

    minute_gear = make_gear(
        M,
        Z_MINUTE,
        GEAR_FACE,
        2.0 * MINUTE_GEAR_BORE_R,
    ).moved(
        Location(
            (MINUTE_CENTER_X, MINUTE_CENTER_Y, STAGE_1_Z),
            (0.0, 0.0, 0.0),
        )
    )
    a.add(
        minute_gear,
        "minute_gear|dof=spin|driver=True|spin_axis=z|"
        "mesh_id=stage1|mount=minute_shaft",
    )

    stage1_phase_deg = 180.0 / Z_COMPOUND_BIG
    compound_big_gear = make_gear(
        M,
        Z_COMPOUND_BIG,
        GEAR_FACE,
        2.0 * COMPOUND_GEAR_BORE_R,
    ).moved(
        Location(
            (
                INTERMEDIATE_CENTER_X,
                INTERMEDIATE_CENTER_Y,
                STAGE_1_Z,
            ),
            (0.0, 0.0, stage1_phase_deg),
        )
    )
    a.add(
        compound_big_gear,
        "compound_big_gear|dof=spin|spin_axis=z|"
        "mesh_id=stage1|mount=intermediate_shaft",
    )

    compound_pinion = make_gear(
        M,
        Z_COMPOUND_PINION,
        GEAR_FACE,
        2.0 * COMPOUND_GEAR_BORE_R,
    ).moved(
        Location(
            (
                INTERMEDIATE_CENTER_X,
                INTERMEDIATE_CENTER_Y,
                STAGE_2_Z,
            ),
            (0.0, 0.0, 0.0),
        )
    )
    a.add(
        compound_pinion,
        "compound_pinion|dof=spin|spin_axis=z|"
        "mesh_id=stage2|mount=intermediate_shaft",
    )

    stage2_phase_deg = 180.0 / Z_HOUR
    hour_gear = make_gear(
        M,
        Z_HOUR,
        GEAR_FACE,
        2.0 * HOUR_GEAR_BORE_R,
    ).moved(
        Location(
            (MINUTE_CENTER_X, MINUTE_CENTER_Y, STAGE_2_Z),
            (0.0, 0.0, stage2_phase_deg),
        )
    )
    a.add(
        hour_gear,
        "hour_gear|dof=spin|spin_axis=z|"
        "mesh_id=stage2|mount=hour_sleeve",
    )

    hour_hand = clock_hand(
        HOUR_HAND_LENGTH,
        HOUR_HAND_WIDTH,
        HOUR_HAND_HUB_R,
        HOUR_HAND_BORE_R,
        HOUR_HAND_T,
    ).moved(
        Location(
            (MINUTE_CENTER_X, MINUTE_CENTER_Y, HOUR_HAND_Z),
            (0.0, 0.0, -55.0),
        )
    )
    a.add(
        hour_hand,
        "hour_hand|dof=fixed|mount=hour_sleeve",
    )

    minute_hand = clock_hand(
        MINUTE_HAND_LENGTH,
        MINUTE_HAND_WIDTH,
        MINUTE_HAND_HUB_R,
        MINUTE_HAND_BORE_R,
        MINUTE_HAND_T,
    ).moved(
        Location(
            (MINUTE_CENTER_X, MINUTE_CENTER_Y, MINUTE_HAND_Z),
            (0.0, 0.0, 15.0),
        )
    )
    a.add(
        minute_hand,
        "minute_hand|dof=fixed|mount=minute_shaft",
    )

    return a.build()