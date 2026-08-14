import math

# ---------------------------------------------------------------------------
# Drivetrain arithmetic — all gear-center locations derive from these values.
# ---------------------------------------------------------------------------
M = 1.5
Z_INPUT = 24
Z_IDLER = 24
Z_OUTPUT = 24

GEAR_FACE_W = 6.0

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

CD_INPUT_IDLER = center_dist(Z_INPUT, Z_IDLER)
CD_IDLER_OUTPUT = center_dist(Z_IDLER, Z_OUTPUT)

X_INPUT = -(CD_INPUT_IDLER + CD_IDLER_OUTPUT) / 2.0
X_IDLER = X_INPUT + CD_INPUT_IDLER
X_OUTPUT = X_IDLER + CD_IDLER_OUTPUT

INPUT_PITCH_R = pitch_r(Z_INPUT)
IDLER_PITCH_R = pitch_r(Z_IDLER)
OUTPUT_PITCH_R = pitch_r(Z_OUTPUT)

STAGE_1_RATIO = -float(Z_INPUT) / float(Z_IDLER)
STAGE_2_RATIO = -float(Z_IDLER) / float(Z_OUTPUT)
OVERALL_RATIO = STAGE_1_RATIO * STAGE_2_RATIO

# Approximate external addendum envelope used only for frame sizing.
INPUT_OUTER_R = M * (Z_INPUT + 2) / 2.0
IDLER_OUTER_R = M * (Z_IDLER + 2) / 2.0
OUTPUT_OUTER_R = M * (Z_OUTPUT + 2) / 2.0
MAX_GEAR_OUTER_R = max(INPUT_OUTER_R, IDLER_OUTER_R, OUTPUT_OUTER_R)

# ---------------------------------------------------------------------------
# Open-frame support dimensions.
# All rotating axes are parallel to global +Y.
# ---------------------------------------------------------------------------
BASE_H = 6.0
BASE_MARGIN_X = 10.0
BASE_MARGIN_Y = 10.0

SHAFT_R = 3.0
SHAFT_D = 2.0 * SHAFT_R

BEARING_RUNNING_CLEARANCE = 0.05
BEARING_BORE_R = SHAFT_R + BEARING_RUNNING_CLEARANCE
BEARING_OUTER_R = 7.0
BEARING_W = 5.0

PEDESTAL_W = 10.0
PEDESTAL_H = 15.0

AXIAL_GEAR_BEARING_GAP = 2.0
FRONT_BEARING_Y = -GEAR_FACE_W / 2.0 - AXIAL_GEAR_BEARING_GAP - BEARING_W
REAR_BEARING_Y = GEAR_FACE_W / 2.0 + AXIAL_GEAR_BEARING_GAP

FRONT_BEARING_CENTER_Y = FRONT_BEARING_Y + BEARING_W / 2.0
REAR_BEARING_CENTER_Y = REAR_BEARING_Y + BEARING_W / 2.0

AXIS_Z = BASE_H + PEDESTAL_H + BEARING_OUTER_R

SHAFT_FRONT_EXTENSION = 8.0
SHAFT_REAR_EXTENSION = 4.0
SHAFT_Y0 = FRONT_BEARING_Y - SHAFT_FRONT_EXTENSION
SHAFT_Y1 = REAR_BEARING_Y + BEARING_W + SHAFT_REAR_EXTENSION
SHAFT_LENGTH = SHAFT_Y1 - SHAFT_Y0
SHAFT_CENTER_Y = (SHAFT_Y0 + SHAFT_Y1) / 2.0

# Gear base station: make_gear is constructed along local +Z, then rotated
# so local +Z becomes global +Y.
GEAR_Y0 = -GEAR_FACE_W / 2.0
GEAR_CENTER_Y = GEAR_Y0 + GEAR_FACE_W / 2.0

# A half-tooth angular offset engages each external mesh.
INPUT_GEAR_PHASE_DEG = 0.0
IDLER_GEAR_PHASE_DEG = 180.0 / Z_IDLER
OUTPUT_GEAR_PHASE_DEG = 0.0

# Press fits are explicit intended interference.
PRESS_INTERFERENCE = 0.005
GEAR_BORE_R = SHAFT_R - PRESS_INTERFERENCE
GEAR_BORE_D = 2.0 * GEAR_BORE_R

# ---------------------------------------------------------------------------
# Input hand crank.
# ---------------------------------------------------------------------------
CRANK_THROW = 18.0
CRANK_ARM_W = 8.0
CRANK_ARM_T = 4.0
CRANK_HUB_R = 6.0

HANDLE_R = 3.0
HANDLE_BORE_R = HANDLE_R - PRESS_INTERFERENCE
HANDLE_GRIP_LENGTH = 18.0

CRANK_Y0 = SHAFT_Y0
HANDLE_AXIS_Z = AXIS_Z + CRANK_THROW
HANDLE_Y0 = CRANK_Y0 - HANDLE_GRIP_LENGTH
HANDLE_LENGTH = HANDLE_GRIP_LENGTH + CRANK_ARM_T

# ---------------------------------------------------------------------------
# Base envelope derived from the complete mechanism.
# ---------------------------------------------------------------------------
BASE_X_MIN = X_INPUT - INPUT_OUTER_R - BASE_MARGIN_X
BASE_X_MAX = X_OUTPUT + OUTPUT_OUTER_R + BASE_MARGIN_X
BASE_X = BASE_X_MAX - BASE_X_MIN
BASE_X_CENTER = (BASE_X_MIN + BASE_X_MAX) / 2.0

Y_EXTENT = max(
    abs(HANDLE_Y0),
    abs(SHAFT_Y1),
    abs(FRONT_BEARING_Y),
    abs(REAR_BEARING_Y + BEARING_W),
)
BASE_Y = 2.0 * (Y_EXTENT + BASE_MARGIN_Y)

SHAFT_DATA = (
    ("input", X_INPUT),
    ("idler", X_IDLER),
    ("output", X_OUTPUT),
)

MECHANISM = {
    "name": "open_frame_three_shaft_reversing_train",
    "output_link": "output_gear",
    "watch_links": [
        "input_shaft",
        "input_gear",
        "idler_shaft",
        "idler_gear",
        "output_shaft",
        "output_gear",
    ],
    "ports_by_link": {
        "base": [],

        "input_front_pedestal": [],
        "input_rear_pedestal": [],
        "idler_front_pedestal": [],
        "idler_rear_pedestal": [],
        "output_front_pedestal": [],
        "output_rear_pedestal": [],

        "input_front_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [X_INPUT, FRONT_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_W,
            }
        ],
        "input_rear_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [X_INPUT, REAR_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_W,
            }
        ],
        "idler_front_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [X_IDLER, FRONT_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_W,
            }
        ],
        "idler_rear_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [X_IDLER, REAR_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_W,
            }
        ],
        "output_front_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [X_OUTPUT, FRONT_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_W,
            }
        ],
        "output_rear_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [X_OUTPUT, REAR_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_W,
            }
        ],

        "input_shaft": [
            {
                "name": "front_journal",
                "type": "shaft",
                "xyz_mm": [X_INPUT, FRONT_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_W,
            },
            {
                "name": "rear_journal",
                "type": "shaft",
                "xyz_mm": [X_INPUT, REAR_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_W,
            },
            {
                "name": "gear_seat",
                "type": "shaft",
                "xyz_mm": [X_INPUT, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "crank_seat",
                "type": "shaft",
                "xyz_mm": [X_INPUT, CRANK_Y0 + CRANK_ARM_T / 2.0, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": CRANK_ARM_T,
            },
        ],
        "idler_shaft": [
            {
                "name": "front_journal",
                "type": "shaft",
                "xyz_mm": [X_IDLER, FRONT_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_W,
            },
            {
                "name": "rear_journal",
                "type": "shaft",
                "xyz_mm": [X_IDLER, REAR_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_W,
            },
            {
                "name": "gear_seat",
                "type": "shaft",
                "xyz_mm": [X_IDLER, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "output_shaft": [
            {
                "name": "front_journal",
                "type": "shaft",
                "xyz_mm": [X_OUTPUT, FRONT_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_W,
            },
            {
                "name": "rear_journal",
                "type": "shaft",
                "xyz_mm": [X_OUTPUT, REAR_BEARING_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_W,
            },
            {
                "name": "gear_seat",
                "type": "shaft",
                "xyz_mm": [X_OUTPUT, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE_W,
            },
        ],

        "input_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [X_INPUT, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": GEAR_BORE_D,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "mesh_to_idler",
                "type": "gear_mesh",
                "xyz_mm": [X_INPUT, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "pitch_radius_mm": INPUT_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "idler_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [X_IDLER, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": GEAR_BORE_D,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "mesh_to_input",
                "type": "gear_mesh",
                "xyz_mm": [X_IDLER, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "pitch_radius_mm": IDLER_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "mesh_to_output",
                "type": "gear_mesh",
                "xyz_mm": [X_IDLER, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "pitch_radius_mm": IDLER_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "output_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [X_OUTPUT, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": GEAR_BORE_D,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "mesh_to_idler",
                "type": "gear_mesh",
                "xyz_mm": [X_OUTPUT, GEAR_CENTER_Y, AXIS_Z],
                "axis": [0, 1, 0],
                "pitch_radius_mm": OUTPUT_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],

        "crank_arm": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [X_INPUT, CRANK_Y0 + CRANK_ARM_T / 2.0, AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * GEAR_BORE_R,
                "depth_mm": CRANK_ARM_T,
            },
            {
                "name": "handle_bore",
                "type": "bore",
                "xyz_mm": [X_INPUT, CRANK_Y0 + CRANK_ARM_T / 2.0, HANDLE_AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * HANDLE_BORE_R,
                "depth_mm": CRANK_ARM_T,
            },
        ],
        "crank_handle": [
            {
                "name": "handle_shaft",
                "type": "shaft",
                "xyz_mm": [X_INPUT, CRANK_Y0 + CRANK_ARM_T / 2.0, HANDLE_AXIS_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * HANDLE_R,
                "depth_mm": CRANK_ARM_T,
            }
        ],
    },

    "relations": [
        {
            "name": "input_front_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_front_bearing",
            "base_port": "journal",
            "incoming_part": "input_shaft",
            "incoming_port": "front_journal",
        },
        {
            "name": "input_rear_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_rear_bearing",
            "base_port": "journal",
            "incoming_part": "input_shaft",
            "incoming_port": "rear_journal",
        },
        {
            "name": "idler_front_journal",
            "mate_type": "journal_bearing",
            "base_part": "idler_front_bearing",
            "base_port": "journal",
            "incoming_part": "idler_shaft",
            "incoming_port": "front_journal",
        },
        {
            "name": "idler_rear_journal",
            "mate_type": "journal_bearing",
            "base_part": "idler_rear_bearing",
            "base_port": "journal",
            "incoming_part": "idler_shaft",
            "incoming_port": "rear_journal",
        },
        {
            "name": "output_front_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_front_bearing",
            "base_port": "journal",
            "incoming_part": "output_shaft",
            "incoming_port": "front_journal",
        },
        {
            "name": "output_rear_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_rear_bearing",
            "base_port": "journal",
            "incoming_part": "output_shaft",
            "incoming_port": "rear_journal",
        },

        {
            "name": "input_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "gear_seat",
            "incoming_part": "input_gear",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "idler_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "idler_shaft",
            "base_port": "gear_seat",
            "incoming_part": "idler_gear",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "output_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "output_shaft",
            "base_port": "gear_seat",
            "incoming_part": "output_gear",
            "incoming_port": "shaft_bore",
        },

        {
            "name": "input_idler_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "input_gear",
            "base_port": "mesh_to_idler",
            "incoming_part": "idler_gear",
            "incoming_port": "mesh_to_input",
            "separation_axis": "+x",
        },
        {
            "name": "idler_output_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "idler_gear",
            "base_port": "mesh_to_output",
            "incoming_part": "output_gear",
            "incoming_port": "mesh_to_idler",
            "separation_axis": "+x",
        },

        {
            "name": "crank_arm_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "crank_seat",
            "incoming_part": "crank_arm",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "crank_handle_press_fit",
            "mate_type": "press_fit",
            "base_part": "crank_arm",
            "base_port": "handle_bore",
            "incoming_part": "crank_handle",
            "incoming_port": "handle_shaft",
        },
    ],

    "motion_joints": [
        {
            "name": "input_shaft_hinge",
            "parent": "",
            "child": "input_shaft",
            "type": "hinge",
            "axis": [0, 1, 0],
            "pos_mm": [X_INPUT, SHAFT_CENTER_Y, AXIS_Z],
        },
        {
            "name": "idler_shaft_hinge",
            "parent": "",
            "child": "idler_shaft",
            "type": "hinge",
            "axis": [0, 1, 0],
            "pos_mm": [X_IDLER, SHAFT_CENTER_Y, AXIS_Z],
        },
        {
            "name": "output_shaft_hinge",
            "parent": "",
            "child": "output_shaft",
            "type": "hinge",
            "axis": [0, 1, 0],
            "pos_mm": [X_OUTPUT, SHAFT_CENTER_Y, AXIS_Z],
        },
    ],

    "transmissions": [
        {
            "name": "input_shaft_to_input_gear",
            "type": "compound_1to1",
            "driving_link": "input_shaft",
            "driven_link": "input_gear",
            "ratio": 1.0,
        },
        {
            "name": "input_to_idler",
            "type": "gear_external",
            "driving_link": "input_gear",
            "driven_link": "idler_gear",
            "ratio": 0,
        },
        {
            "name": "idler_shaft_lock",
            "type": "compound_1to1",
            "driving_link": "idler_gear",
            "driven_link": "idler_shaft",
            "ratio": 1.0,
        },
        {
            "name": "idler_to_output",
            "type": "gear_external",
            "driving_link": "idler_gear",
            "driven_link": "output_gear",
            "ratio": 0,
        },
        {
            "name": "output_gear_to_output_shaft",
            "type": "compound_1to1",
            "driving_link": "output_gear",
            "driven_link": "output_shaft",
            "ratio": 1.0,
        },
    ],

    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("open_frame_three_shaft_reversing_train")

    def orient_z_to_y(part, x, y, z):
        return (
            part
            .moved(Location((0, 0, 0), (-90, 0, 0)))
            .moved(Location((x, y, z)))
        )

    def orient_gear(part, phase_deg, x, y, z):
        return (
            part
            .moved(Location((0, 0, 0), (0, 0, phase_deg)))
            .moved(Location((0, 0, 0), (-90, 0, 0)))
            .moved(Location((x, y, z)))
        )

    def make_sleeve_bearing():
        with BuildSketch() as bearing_sketch:
            Circle(BEARING_OUTER_R)
            Circle(BEARING_BORE_R, mode=Mode.SUBTRACT)
        return extrude(bearing_sketch.sketch, amount=BEARING_W)

    def make_crank_arm():
        # Local XY profile extruded along local +Z. After rotation, local +Z
        # is global +Y and local -Y is global +Z.
        with BuildSketch() as arm_sketch:
            Circle(CRANK_HUB_R)
            with b3d.Locations((0.0, -CRANK_THROW)):
                Circle(CRANK_HUB_R)
            Polygon(
                (-CRANK_ARM_W / 2.0, 0.0),
                ( CRANK_ARM_W / 2.0, 0.0),
                ( CRANK_ARM_W / 2.0, -CRANK_THROW),
                (-CRANK_ARM_W / 2.0, -CRANK_THROW),
            )
            Circle(GEAR_BORE_R, mode=Mode.SUBTRACT)
            with b3d.Locations((0.0, -CRANK_THROW)):
                Circle(HANDLE_BORE_R, mode=Mode.SUBTRACT)
        return extrude(arm_sketch.sketch, amount=CRANK_ARM_T)

    # Low open bench base.
    base = Box(
        BASE_X,
        BASE_Y,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((BASE_X_CENTER, 0.0, 0.0)))
    a.add(base, "base|dof=fixed")

    # Six narrow pedestals and six exposed sleeve journal bearings.
    for shaft_key, shaft_x in SHAFT_DATA:
        for side, bearing_y, bearing_center_y in (
            ("front", FRONT_BEARING_Y, FRONT_BEARING_CENTER_Y),
            ("rear", REAR_BEARING_Y, REAR_BEARING_CENTER_Y),
        ):
            pedestal_name = f"{shaft_key}_{side}_pedestal"
            bearing_name = f"{shaft_key}_{side}_bearing"

            pedestal = Box(
                PEDESTAL_W,
                BEARING_W,
                PEDESTAL_H,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(
                Location((shaft_x, bearing_center_y, BASE_H))
            )
            a.add(
                pedestal,
                f"{pedestal_name}|dof=fixed|mount=base",
            )

            bearing = orient_z_to_y(
                make_sleeve_bearing(),
                shaft_x,
                bearing_y,
                AXIS_Z,
            )
            a.add(
                bearing,
                f"{bearing_name}|dof=fixed|mount={pedestal_name}",
            )

    # Independent shafts, each carried by both of its bearings.
    input_shaft = orient_z_to_y(
        Cylinder(
            SHAFT_R,
            SHAFT_LENGTH,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ),
        X_INPUT,
        SHAFT_Y0,
        AXIS_Z,
    )
    a.add(
        input_shaft,
        "input_shaft|dof=spin|driver=True|spin_axis=y|"
        "mount=input_front_bearing,input_rear_bearing",
    )

    idler_shaft = orient_z_to_y(
        Cylinder(
            SHAFT_R,
            SHAFT_LENGTH,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ),
        X_IDLER,
        SHAFT_Y0,
        AXIS_Z,
    )
    a.add(
        idler_shaft,
        "idler_shaft|dof=spin|spin_axis=y|"
        "mount=idler_front_bearing,idler_rear_bearing",
    )

    output_shaft = orient_z_to_y(
        Cylinder(
            SHAFT_R,
            SHAFT_LENGTH,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ),
        X_OUTPUT,
        SHAFT_Y0,
        AXIS_Z,
    )
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=y|"
        "mount=output_front_bearing,output_rear_bearing",
    )

    # Three separate spur gears. Their x coordinates are entirely derived
    # from the two exact pitch-center distances above.
    input_gear = orient_gear(
        make_gear(M, Z_INPUT, GEAR_FACE_W, GEAR_BORE_D),
        INPUT_GEAR_PHASE_DEG,
        X_INPUT,
        GEAR_Y0,
        AXIS_Z,
    )
    a.add(
        input_gear,
        "input_gear|dof=spin|spin_axis=y|mesh_id=mesh_input_idler|"
        "mount=input_shaft",
    )

    idler_gear = orient_gear(
        make_gear(M, Z_IDLER, GEAR_FACE_W, GEAR_BORE_D),
        IDLER_GEAR_PHASE_DEG,
        X_IDLER,
        GEAR_Y0,
        AXIS_Z,
    )
    a.add(
        idler_gear,
        "idler_gear|dof=spin|spin_axis=y|"
        "mesh_id=mesh_input_idler,mesh_idler_output|mount=idler_shaft",
    )

    output_gear = orient_gear(
        make_gear(M, Z_OUTPUT, GEAR_FACE_W, GEAR_BORE_D),
        OUTPUT_GEAR_PHASE_DEG,
        X_OUTPUT,
        GEAR_Y0,
        AXIS_Z,
    )
    a.add(
        output_gear,
        "output_gear|dof=spin|spin_axis=y|mesh_id=mesh_idler_output|"
        "mount=output_shaft",
    )

    # Hand crank only on the input shaft.
    crank_arm = orient_z_to_y(
        make_crank_arm(),
        X_INPUT,
        CRANK_Y0,
        AXIS_Z,
    )
    a.add(
        crank_arm,
        "crank_arm|dof=fixed|mount=input_shaft",
    )

    crank_handle = orient_z_to_y(
        Cylinder(
            HANDLE_R,
            HANDLE_LENGTH,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ),
        X_INPUT,
        HANDLE_Y0,
        HANDLE_AXIS_Z,
    )
    a.add(
        crank_handle,
        "crank_handle|dof=fixed|mount=crank_arm",
    )

    return a.build()