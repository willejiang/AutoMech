from math import pi

# ---------------------------------------------------------------------------
# Drivetrain arithmetic -- all meshing locations derive from these values.
# ---------------------------------------------------------------------------
M = 0.8

Z_INPUT = 12
Z_COMPOUND_WHEEL = 36
Z_COMPOUND_PINION = 12
Z_OUTPUT = 48

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

def outside_r(z):
    return pitch_r(z) + M

CD_STAGE_1 = center_dist(Z_INPUT, Z_COMPOUND_WHEEL)
CD_STAGE_2 = center_dist(Z_COMPOUND_PINION, Z_OUTPUT)

STAGE_1_RATIO = Z_COMPOUND_WHEEL / Z_INPUT
STAGE_2_RATIO = Z_OUTPUT / Z_COMPOUND_PINION
TOTAL_REDUCTION = STAGE_1_RATIO * STAGE_2_RATIO

# Gear-center layout: never guessed.
X_COMPOUND = 0.0
X_INPUT = X_COMPOUND - CD_STAGE_1
X_OUTPUT = X_COMPOUND + CD_STAGE_2
Y_SHAFTS = 0.0

# ---------------------------------------------------------------------------
# Axial stack and fits.
# ---------------------------------------------------------------------------
BASE_H = 5.0
THRUST_H = 0.5

SHAFT_R = 3.0
SHAFT_Z = BASE_H + THRUST_H
SHAFT_TOP = 46.0
SHAFT_H = SHAFT_TOP - SHAFT_Z

JOURNAL_CLEARANCE = 0.05
JOURNAL_BORE_R = SHAFT_R + JOURNAL_CLEARANCE
BEARING_OUTER_R = 5.0
BEARING_H = 5.0

PRESS_INTERFERENCE = 0.005
PRESS_BORE_R = SHAFT_R - PRESS_INTERFERENCE
PRESS_BORE_D = 2.0 * PRESS_BORE_R

LOWER_BEARING_Z = BASE_H
LOWER_BEARING_TOP = LOWER_BEARING_Z + BEARING_H

GEAR_FACE = 6.0
LOWER_GEAR_Z = LOWER_BEARING_TOP + 1.0
LOWER_GEAR_TOP = LOWER_GEAR_Z + GEAR_FACE
STAGE_AXIAL_GAP = 3.0
UPPER_GEAR_Z = LOWER_GEAR_TOP + STAGE_AXIAL_GAP
UPPER_GEAR_TOP = UPPER_GEAR_Z + GEAR_FACE

UPPER_PLATE_Z = UPPER_GEAR_TOP + 2.0
UPPER_PLATE_H = 5.0
UPPER_PLATE_TOP = UPPER_PLATE_Z + UPPER_PLATE_H
UPPER_BEARING_Z = UPPER_PLATE_Z

CRANK_Z = 40.0
CRANK_H = 4.0
CRANK_THROW = 14.0
CRANK_HUB_R = 6.0
CRANK_ARM_W = 5.0
CRANK_END_R = 4.0
HANDLE_R = 3.0
HANDLE_Z = CRANK_Z + CRANK_H
HANDLE_H = 11.0

OUTPUT_COUPLING_Z = 40.0
OUTPUT_COUPLING_H = 4.0
OUTPUT_COUPLING_R = 7.0

# ---------------------------------------------------------------------------
# Frame envelope follows the actual gear envelopes.
# ---------------------------------------------------------------------------
GEAR_X_MIN = X_INPUT - outside_r(Z_INPUT)
GEAR_X_MAX = X_OUTPUT + outside_r(Z_OUTPUT)
FRAME_MARGIN_X = 8.0
BASE_X_MIN = GEAR_X_MIN - FRAME_MARGIN_X
BASE_X_MAX = GEAR_X_MAX + FRAME_MARGIN_X
BASE_L = BASE_X_MAX - BASE_X_MIN
BASE_CX = (BASE_X_MIN + BASE_X_MAX) / 2.0

MAX_GEAR_R = max(
    outside_r(Z_COMPOUND_WHEEL),
    outside_r(Z_OUTPUT),
)
FRAME_MARGIN_Y = 8.0
BASE_W = 2.0 * (MAX_GEAR_R + FRAME_MARGIN_Y)
BASE_CY = 0.0

COLUMN_R = 3.0
COLUMN_EDGE_INSET = 5.0
COLUMN_X_LEFT = BASE_X_MIN + COLUMN_EDGE_INSET
COLUMN_X_RIGHT = BASE_X_MAX - COLUMN_EDGE_INSET
COLUMN_Y_FRONT = -BASE_W / 2.0 + COLUMN_EDGE_INSET
COLUMN_Y_REAR = BASE_W / 2.0 - COLUMN_EDGE_INSET
COLUMN_Z = BASE_H
COLUMN_H = UPPER_PLATE_Z - COLUMN_Z

UPPER_BEARING_PLATE_INTERFERENCE = 0.005
UPPER_PLATE_HOLE_R = BEARING_OUTER_R - UPPER_BEARING_PLATE_INTERFERENCE

GEAR_PHASE_STAGE_1_DEG = 180.0 / Z_COMPOUND_WHEEL
GEAR_PHASE_STAGE_2_DEG = 180.0 / Z_OUTPUT

SHAFT_LOWER_SEAT_LOCAL_Z = (
    LOWER_BEARING_Z + BEARING_H / 2.0 - SHAFT_Z
)
SHAFT_UPPER_SEAT_LOCAL_Z = (
    UPPER_BEARING_Z + BEARING_H / 2.0 - SHAFT_Z
)
SHAFT_LOWER_GEAR_LOCAL_Z = (
    LOWER_GEAR_Z + GEAR_FACE / 2.0 - SHAFT_Z
)
SHAFT_UPPER_GEAR_LOCAL_Z = (
    UPPER_GEAR_Z + GEAR_FACE / 2.0 - SHAFT_Z
)
SHAFT_CRANK_SEAT_LOCAL_Z = (
    CRANK_Z + CRANK_H / 2.0 - SHAFT_Z
)
SHAFT_COUPLING_SEAT_LOCAL_Z = (
    OUTPUT_COUPLING_Z + OUTPUT_COUPLING_H / 2.0 - SHAFT_Z
)

MECHANISM = {
    "name": "hand_cranked_two_stage_spur_reducer",
    "output_link": "output_shaft",
    "watch_links": [
        "input_shaft",
        "input_pinion",
        "compound_shaft",
        "compound_wheel",
        "compound_pinion",
        "output_gear",
        "output_shaft",
    ],
    "ports_by_link": {
        "input_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_LOWER_SEAT_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_UPPER_SEAT_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_H,
            },
            {
                "name": "pinion_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_LOWER_GEAR_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "crank_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_CRANK_SEAT_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANK_H,
            },
        ],
        "compound_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_LOWER_SEAT_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_UPPER_SEAT_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_H,
            },
            {
                "name": "wheel_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_LOWER_GEAR_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "pinion_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_UPPER_GEAR_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
        ],
        "output_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_LOWER_SEAT_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_UPPER_SEAT_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_H,
            },
            {
                "name": "gear_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_UPPER_GEAR_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "coupling_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_COUPLING_SEAT_LOCAL_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": OUTPUT_COUPLING_H,
            },
        ],
        "input_pinion": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "teeth",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": pitch_r(Z_INPUT),
                "depth_mm": GEAR_FACE,
            },
        ],
        "compound_wheel": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "teeth",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": pitch_r(Z_COMPOUND_WHEEL),
                "depth_mm": GEAR_FACE,
            },
        ],
        "compound_pinion": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "teeth",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": pitch_r(Z_COMPOUND_PINION),
                "depth_mm": GEAR_FACE,
            },
        ],
        "output_gear": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "teeth",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": pitch_r(Z_OUTPUT),
                "depth_mm": GEAR_FACE,
            },
        ],
        "input_lower_bearing": [{
            "name": "journal",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": 2.0 * JOURNAL_BORE_R,
            "depth_mm": BEARING_H,
        }],
        "input_upper_bearing": [{
            "name": "journal",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": 2.0 * JOURNAL_BORE_R,
            "depth_mm": BEARING_H,
        }],
        "compound_lower_bearing": [{
            "name": "journal",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": 2.0 * JOURNAL_BORE_R,
            "depth_mm": BEARING_H,
        }],
        "compound_upper_bearing": [{
            "name": "journal",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": 2.0 * JOURNAL_BORE_R,
            "depth_mm": BEARING_H,
        }],
        "output_lower_bearing": [{
            "name": "journal",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": 2.0 * JOURNAL_BORE_R,
            "depth_mm": BEARING_H,
        }],
        "output_upper_bearing": [{
            "name": "journal",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": 2.0 * JOURNAL_BORE_R,
            "depth_mm": BEARING_H,
        }],
        "hand_crank": [{
            "name": "shaft_bore",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, CRANK_H / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": PRESS_BORE_D,
            "depth_mm": CRANK_H,
        }],
        "output_coupling": [{
            "name": "shaft_bore",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, OUTPUT_COUPLING_H / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": PRESS_BORE_D,
            "depth_mm": OUTPUT_COUPLING_H,
        }],
    },
    "relations": [
        {
            "name": "stage_1_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "input_pinion",
            "base_port": "teeth",
            "incoming_part": "compound_wheel",
            "incoming_port": "teeth",
            "separation_axis": "+x",
            "axis_angle_deg": GEAR_PHASE_STAGE_1_DEG,
        },
        {
            "name": "stage_2_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "compound_pinion",
            "base_port": "teeth",
            "incoming_part": "output_gear",
            "incoming_port": "teeth",
            "separation_axis": "+x",
            "axis_angle_deg": GEAR_PHASE_STAGE_2_DEG,
        },
        {
            "name": "input_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_pinion",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "pinion_seat",
        },
        {
            "name": "compound_wheel_press_fit",
            "mate_type": "press_fit",
            "base_part": "compound_wheel",
            "base_port": "bore",
            "incoming_part": "compound_shaft",
            "incoming_port": "wheel_seat",
        },
        {
            "name": "compound_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "compound_pinion",
            "base_port": "bore",
            "incoming_part": "compound_shaft",
            "incoming_port": "pinion_seat",
        },
        {
            "name": "output_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "output_gear",
            "base_port": "bore",
            "incoming_part": "output_shaft",
            "incoming_port": "gear_seat",
        },
        {
            "name": "crank_press_fit",
            "mate_type": "press_fit",
            "base_part": "hand_crank",
            "base_port": "shaft_bore",
            "incoming_part": "input_shaft",
            "incoming_port": "crank_seat",
        },
        {
            "name": "output_coupling_press_fit",
            "mate_type": "press_fit",
            "base_part": "output_coupling",
            "base_port": "shaft_bore",
            "incoming_part": "output_shaft",
            "incoming_port": "coupling_seat",
        },
        {
            "name": "input_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_lower_bearing",
            "base_port": "journal",
            "incoming_part": "input_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "input_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_upper_bearing",
            "base_port": "journal",
            "incoming_part": "input_shaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "compound_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "compound_lower_bearing",
            "base_port": "journal",
            "incoming_part": "compound_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "compound_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "compound_upper_bearing",
            "base_port": "journal",
            "incoming_part": "compound_shaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "output_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_lower_bearing",
            "base_port": "journal",
            "incoming_part": "output_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "output_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_upper_bearing",
            "base_port": "journal",
            "incoming_part": "output_shaft",
            "incoming_port": "upper_journal",
        },
    ],
    "motion_joints": [
        {
            "name": "input_shaft_rotation",
            "parent": "",
            "child": "input_shaft",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "compound_shaft_rotation",
            "parent": "",
            "child": "compound_shaft",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "output_shaft_rotation",
            "parent": "",
            "child": "output_shaft",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
    ],
    "transmissions": [
        {
            "name": "input_shaft_to_pinion",
            "type": "compound_1to1",
            "driving_link": "input_shaft",
            "driven_link": "input_pinion",
            "ratio": 1.0,
        },
        {
            "name": "first_reduction",
            "type": "gear_external",
            "driving_link": "input_pinion",
            "driven_link": "compound_wheel",
            "ratio": 0,
        },
        {
            "name": "compound_pair_lock",
            "type": "compound_1to1",
            "driving_link": "compound_wheel",
            "driven_link": "compound_pinion",
            "ratio": 1.0,
        },
        {
            "name": "second_reduction",
            "type": "gear_external",
            "driving_link": "compound_pinion",
            "driven_link": "output_gear",
            "ratio": 0,
        },
        {
            "name": "output_gear_to_shaft",
            "type": "compound_1to1",
            "driving_link": "output_gear",
            "driven_link": "output_shaft",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("hand_cranked_two_stage_spur_reducer")

    z_min_align = (Align.CENTER, Align.CENTER, Align.MIN)

    def annulus(outer_r, inner_r, height):
        body = Cylinder(
            outer_r,
            height,
            align=z_min_align,
        )
        cutter = Cylinder(
            inner_r,
            height + 2.0,
            align=z_min_align,
        ).moved(Location((0.0, 0.0, -1.0)))
        return body - cutter

    def placed_gear(module, teeth, width, bore_d, x, y, z, phase_deg=0.0):
        raw = make_gear(module, teeth, width, bore_d)
        raw_z_min = raw.bounding_box().min.Z
        return raw.moved(
            Location(
                (x, y, z - raw_z_min),
                (0.0, 0.0, phase_deg),
            )
        )

    # Base plate, resting directly on the ground.
    baseplate = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=z_min_align,
    ).moved(Location((BASE_CX, BASE_CY, 0.0)))
    a.add(baseplate, "baseplate|dof=fixed")

    # Four columns touch the base and the underside of the upper bridge.
    column_positions = [
        (COLUMN_X_LEFT, COLUMN_Y_FRONT),
        (COLUMN_X_LEFT, COLUMN_Y_REAR),
        (COLUMN_X_RIGHT, COLUMN_Y_FRONT),
        (COLUMN_X_RIGHT, COLUMN_Y_REAR),
    ]
    column_names = [
        "front_left_column",
        "rear_left_column",
        "front_right_column",
        "rear_right_column",
    ]
    for name, (cx, cy) in zip(column_names, column_positions):
        column = Cylinder(
            COLUMN_R,
            COLUMN_H,
            align=z_min_align,
        ).moved(Location((cx, cy, COLUMN_Z)))
        a.add(column, f"{name}|dof=fixed|mount=baseplate")

    # Upper bridge plate with three true bearing-seat openings.
    upper_plate = Box(
        BASE_L,
        BASE_W,
        UPPER_PLATE_H,
        align=z_min_align,
    ).moved(Location((BASE_CX, BASE_CY, UPPER_PLATE_Z)))

    for sx in (X_INPUT, X_COMPOUND, X_OUTPUT):
        plate_hole = Cylinder(
            UPPER_PLATE_HOLE_R,
            UPPER_PLATE_H + 2.0,
            align=z_min_align,
        ).moved(Location((sx, Y_SHAFTS, UPPER_PLATE_Z - 1.0)))
        upper_plate = upper_plate - plate_hole

    a.add(
        upper_plate,
        "upper_bridge|dof=fixed|mount="
        "front_left_column,rear_left_column,"
        "front_right_column,rear_right_column",
    )

    # Axial thrust buttons: shaft bottom faces touch these small fixed pads.
    shaft_specs = [
        ("input", X_INPUT),
        ("compound", X_COMPOUND),
        ("output", X_OUTPUT),
    ]
    for prefix, sx in shaft_specs:
        thrust = Cylinder(
            SHAFT_R * 0.50,
            THRUST_H,
            align=z_min_align,
        ).moved(Location((sx, Y_SHAFTS, BASE_H)))
        a.add(
            thrust,
            f"{prefix}_thrust_button|dof=fixed|mount=baseplate",
        )

    # Lower bronze journal bushings stand directly on the base.
    for prefix, sx in shaft_specs:
        bearing = annulus(
            BEARING_OUTER_R,
            JOURNAL_BORE_R,
            BEARING_H,
        ).moved(Location((sx, Y_SHAFTS, LOWER_BEARING_Z)))
        a.add(
            bearing,
            f"{prefix}_lower_bearing|dof=fixed|mount=baseplate",
        )

    # Upper bushings have a light OD interference with the bridge openings.
    for prefix, sx in shaft_specs:
        bearing = annulus(
            BEARING_OUTER_R,
            JOURNAL_BORE_R,
            BEARING_H,
        ).moved(Location((sx, Y_SHAFTS, UPPER_BEARING_Z)))
        a.add(
            bearing,
            f"{prefix}_upper_bearing|dof=fixed|mount=upper_bridge",
        )

    # Three full-length visible shafts.
    input_shaft = Cylinder(
        SHAFT_R,
        SHAFT_H,
        align=z_min_align,
    ).moved(Location((X_INPUT, Y_SHAFTS, SHAFT_Z)))
    a.add(
        input_shaft,
        "input_shaft|dof=spin|driver=True|spin_axis=z|"
        "mount=input_lower_bearing,input_upper_bearing,input_thrust_button",
    )

    compound_shaft = Cylinder(
        SHAFT_R,
        SHAFT_H,
        align=z_min_align,
    ).moved(Location((X_COMPOUND, Y_SHAFTS, SHAFT_Z)))
    a.add(
        compound_shaft,
        "compound_shaft|dof=spin|spin_axis=z|"
        "mount=compound_lower_bearing,compound_upper_bearing,"
        "compound_thrust_button",
    )

    output_shaft = Cylinder(
        SHAFT_R,
        SHAFT_H,
        align=z_min_align,
    ).moved(Location((X_OUTPUT, Y_SHAFTS, SHAFT_Z)))
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=z|"
        "mount=output_lower_bearing,output_upper_bearing,"
        "output_thrust_button",
    )

    # Stage 1: exact CD_STAGE_1 spacing, common module and common axial station.
    input_pinion = placed_gear(
        M,
        Z_INPUT,
        GEAR_FACE,
        PRESS_BORE_D,
        X_INPUT,
        Y_SHAFTS,
        LOWER_GEAR_Z,
        0.0,
    )
    a.add(
        input_pinion,
        "input_pinion|dof=spin|spin_axis=z|driver=False|"
        "mesh_id=stage_1|mount=input_shaft",
    )

    compound_wheel = placed_gear(
        M,
        Z_COMPOUND_WHEEL,
        GEAR_FACE,
        PRESS_BORE_D,
        X_COMPOUND,
        Y_SHAFTS,
        LOWER_GEAR_Z,
        GEAR_PHASE_STAGE_1_DEG,
    )
    a.add(
        compound_wheel,
        "compound_wheel|dof=spin|spin_axis=z|"
        "mesh_id=stage_1|mount=compound_shaft",
    )

    # Stage 2 is at a distinct station, with exact CD_STAGE_2 spacing.
    compound_pinion = placed_gear(
        M,
        Z_COMPOUND_PINION,
        GEAR_FACE,
        PRESS_BORE_D,
        X_COMPOUND,
        Y_SHAFTS,
        UPPER_GEAR_Z,
        0.0,
    )
    a.add(
        compound_pinion,
        "compound_pinion|dof=spin|spin_axis=z|"
        "mesh_id=stage_2|mount=compound_shaft",
    )

    output_gear = placed_gear(
        M,
        Z_OUTPUT,
        GEAR_FACE,
        PRESS_BORE_D,
        X_OUTPUT,
        Y_SHAFTS,
        UPPER_GEAR_Z,
        GEAR_PHASE_STAGE_2_DEG,
    )
    a.add(
        output_gear,
        "output_gear|dof=spin|spin_axis=z|"
        "mesh_id=stage_2|mount=output_shaft",
    )

    # One-piece hand crank with a press-fit shaft bore.
    crank_hub = Cylinder(
        CRANK_HUB_R,
        CRANK_H,
        align=z_min_align,
    )
    crank_arm = Box(
        CRANK_ARM_W,
        CRANK_THROW,
        CRANK_H,
        align=(Align.CENTER, Align.MAX, Align.MIN),
    )
    crank_end = Cylinder(
        CRANK_END_R,
        CRANK_H,
        align=z_min_align,
    ).moved(Location((0.0, -CRANK_THROW, 0.0)))

    hand_crank = crank_hub + crank_arm + crank_end
    crank_bore = Cylinder(
        PRESS_BORE_R,
        CRANK_H + 2.0,
        align=z_min_align,
    ).moved(Location((0.0, 0.0, -1.0)))
    hand_crank = (hand_crank - crank_bore).moved(
        Location((X_INPUT, Y_SHAFTS, CRANK_Z))
    )
    a.add(
        hand_crank,
        "hand_crank|dof=fixed|mount=input_shaft",
    )

    # Rigid hand grip touches the crank end pad at its real top face.
    crank_handle = Cylinder(
        HANDLE_R,
        HANDLE_H,
        align=z_min_align,
    ).moved(
        Location(
            (X_INPUT, Y_SHAFTS - CRANK_THROW, HANDLE_Z)
        )
    )
    a.add(
        crank_handle,
        "crank_handle|dof=fixed|mount=hand_crank",
    )

    # Visible press-fit output coupling, leaving the shaft tip exposed.
    output_coupling = annulus(
        OUTPUT_COUPLING_R,
        PRESS_BORE_R,
        OUTPUT_COUPLING_H,
    ).moved(
        Location((X_OUTPUT, Y_SHAFTS, OUTPUT_COUPLING_Z))
    )
    a.add(
        output_coupling,
        "output_coupling|dof=fixed|mount=output_shaft",
    )

    return a.build()