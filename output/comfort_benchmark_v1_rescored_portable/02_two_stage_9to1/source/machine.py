import math

# ---------------------------------------------------------------------------
# Drivetrain arithmetic: all mesh positions derive from these values.
# ---------------------------------------------------------------------------
M = 0.8
Z_PINION = 12
Z_GEAR = 36
STAGE_RATIO = Z_GEAR / Z_PINION
OVERALL_RATIO = STAGE_RATIO * STAGE_RATIO

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

RP_PINION = pitch_r(Z_PINION)
RP_GEAR = pitch_r(Z_GEAR)
CD_STAGE_1 = center_dist(Z_PINION, Z_GEAR)
CD_STAGE_2 = center_dist(Z_PINION, Z_GEAR)

X_INPUT = 0.0
X_INTERMEDIATE = X_INPUT + CD_STAGE_1
X_OUTPUT = X_INTERMEDIATE + CD_STAGE_2

FACE_W = 5.0
AXIAL_GAP = 1.0
Y_STAGE_1 = -(FACE_W + AXIAL_GAP) / 2.0
Y_STAGE_2 = +(FACE_W + AXIAL_GAP) / 2.0

SHAFT_R = 2.0
GEAR_BORE_R = SHAFT_R - 0.005
JOURNAL_BORE_R = SHAFT_R + 0.05

BASE_H = 4.0
GEAR_OUTER_R_MAX = RP_GEAR + M
GEAR_TO_BASE_CLEARANCE = 2.0
Z_SHAFT = BASE_H + GEAR_OUTER_R_MAX + GEAR_TO_BASE_CLEARANCE

BEARING_OUTER_R = 4.3
BEARING_W = 4.0
BEARING_TO_GEAR_GAP = 1.0

Y_BEARING_LOW = (
    Y_STAGE_1 - FACE_W / 2.0
    - BEARING_TO_GEAR_GAP - BEARING_W / 2.0
)
Y_BEARING_HIGH = (
    Y_STAGE_2 + FACE_W / 2.0
    + BEARING_TO_GEAR_GAP + BEARING_W / 2.0
)

# Corrected only for output_high_bearing: make_gear grows from its axial
# placement face, so the output gear's real high-Y face is Y_STAGE_2 + FACE_W.
Y_OUTPUT_HIGH_BEARING = (
    Y_STAGE_2 + FACE_W
    + BEARING_TO_GEAR_GAP + BEARING_W / 2.0
)

Y_BEARING_LOW_OUTER = Y_BEARING_LOW - BEARING_W / 2.0
Y_BEARING_HIGH_OUTER = Y_BEARING_HIGH + BEARING_W / 2.0

SHAFT_Y_MIN = Y_BEARING_LOW_OUTER
SHAFT_Y_MAX = Y_BEARING_HIGH_OUTER
SHAFT_LENGTH = SHAFT_Y_MAX - SHAFT_Y_MIN
SHAFT_Y_CENTER = (SHAFT_Y_MIN + SHAFT_Y_MAX) / 2.0

CRANK_T = 3.0
CRANK_AXIAL_GAP = 0.5
CRANK_Y_INNER = Y_BEARING_LOW_OUTER - CRANK_AXIAL_GAP
CRANK_Y_OUTER = CRANK_Y_INNER - CRANK_T
CRANK_Y = (CRANK_Y_INNER + CRANK_Y_OUTER) / 2.0

INPUT_SHAFT_Y_MIN = CRANK_Y_OUTER
INPUT_SHAFT_Y_MAX = SHAFT_Y_MAX
INPUT_SHAFT_LENGTH = INPUT_SHAFT_Y_MAX - INPUT_SHAFT_Y_MIN
INPUT_SHAFT_Y_CENTER = (INPUT_SHAFT_Y_MIN + INPUT_SHAFT_Y_MAX) / 2.0

CRANK_THROW = 16.0
CRANK_HUB_R = 5.0
CRANK_HANDLE_BOSS_R = 4.0
CRANK_ARM_HALF_W = 2.5

HANDLE_X = X_INPUT + CRANK_THROW
HANDLE_Z = Z_SHAFT
HANDLE_PIN_R = 2.0
HANDLE_PIN_LENGTH = 8.0
HANDLE_PIN_Y_INNER = CRANK_Y_INNER
HANDLE_PIN_Y_OUTER = HANDLE_PIN_Y_INNER - HANDLE_PIN_LENGTH
HANDLE_PIN_Y = (HANDLE_PIN_Y_INNER + HANDLE_PIN_Y_OUTER) / 2.0

GRIP_LENGTH = 4.5
GRIP_END_CLEARANCE = 0.5
GRIP_Y_INNER = CRANK_Y_OUTER - GRIP_END_CLEARANCE
GRIP_Y_OUTER = GRIP_Y_INNER - GRIP_LENGTH
GRIP_Y = (GRIP_Y_INNER + GRIP_Y_OUTER) / 2.0
GRIP_OUTER_R = 4.0
GRIP_BORE_R = HANDLE_PIN_R - 0.005

PEDESTAL_OVERLAP = 0.8
PEDESTAL_W = 2.0 * BEARING_OUTER_R
PEDESTAL_H = (
    Z_SHAFT - BEARING_OUTER_R + PEDESTAL_OVERLAP - BASE_H
)

BASE_MARGIN_X = 8.0
BASE_X_MIN = X_INPUT - BEARING_OUTER_R - BASE_MARGIN_X
BASE_X_MAX = X_OUTPUT + GEAR_OUTER_R_MAX + BASE_MARGIN_X
BASE_L = BASE_X_MAX - BASE_X_MIN
BASE_X_CENTER = (BASE_X_MIN + BASE_X_MAX) / 2.0

BASE_MARGIN_Y = 4.0
BASE_Y_MIN = min(GRIP_Y_OUTER, Y_BEARING_LOW_OUTER) - BASE_MARGIN_Y
BASE_Y_MAX = Y_BEARING_HIGH_OUTER + BASE_MARGIN_Y
BASE_W = BASE_Y_MAX - BASE_Y_MIN
BASE_Y_CENTER = (BASE_Y_MIN + BASE_Y_MAX) / 2.0

DRIVEN_GEAR_PHASE_DEG = 180.0 / Z_GEAR

MECHANISM = {
    "name": "open_frame_hand_cranked_9_to_1_spur_reducer",
    "output_link": "output_shaft",
    "watch_links": [
        "crank_arm",
        "input_pinion",
        "intermediate_driven_gear",
        "intermediate_stage2_pinion",
        "output_gear",
        "output_shaft",
    ],
    "ports_by_link": {
        "crank_arm": [
            {
                "name": "input_hub_bore",
                "type": "bore",
                "xyz_mm": [X_INPUT, CRANK_Y, Z_SHAFT],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * GEAR_BORE_R,
                "depth_mm": CRANK_T,
            },
            {
                "name": "handle_bore",
                "type": "bore",
                "xyz_mm": [HANDLE_X, CRANK_Y, HANDLE_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * (HANDLE_PIN_R - 0.005),
                "depth_mm": CRANK_T,
            },
        ],
        "crank_handle_spindle": [
            {
                "name": "spindle_axis",
                "type": "shaft",
                "xyz_mm": [HANDLE_X, HANDLE_PIN_Y, HANDLE_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * HANDLE_PIN_R,
                "depth_mm": HANDLE_PIN_LENGTH,
            }
        ],
        "crank_grip": [
            {
                "name": "grip_bore",
                "type": "bore",
                "xyz_mm": [HANDLE_X, GRIP_Y, HANDLE_Z],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * GRIP_BORE_R,
                "depth_mm": GRIP_LENGTH,
            }
        ],
        "input_shaft": [
            {
                "name": "shaft_axis",
                "type": "shaft",
                "xyz_mm": [X_INPUT, INPUT_SHAFT_Y_CENTER, Z_SHAFT],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": INPUT_SHAFT_LENGTH,
            }
        ],
        "intermediate_shaft": [
            {
                "name": "shaft_axis",
                "type": "shaft",
                "xyz_mm": [X_INTERMEDIATE, SHAFT_Y_CENTER, Z_SHAFT],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": SHAFT_LENGTH,
            }
        ],
        "output_shaft": [
            {
                "name": "shaft_axis",
                "type": "shaft",
                "xyz_mm": [X_OUTPUT, SHAFT_Y_CENTER, Z_SHAFT],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": SHAFT_LENGTH,
            }
        ],
        "input_pinion": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [X_INPUT, Y_STAGE_1, Z_SHAFT],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * GEAR_BORE_R,
                "depth_mm": FACE_W,
            },
            {
                "name": "stage1_mesh",
                "type": "gear_mesh",
                "xyz_mm": [X_INPUT, Y_STAGE_1, Z_SHAFT],
                "axis": [0, 1, 0],
                "pitch_radius_mm": RP_PINION,
                "depth_mm": FACE_W,
            },
        ],
        "intermediate_driven_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [X_INTERMEDIATE, Y_STAGE_1, Z_SHAFT],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * GEAR_BORE_R,
                "depth_mm": FACE_W,
            },
            {
                "name": "stage1_mesh",
                "type": "gear_mesh",
                "xyz_mm": [X_INTERMEDIATE, Y_STAGE_1, Z_SHAFT],
                "axis": [0, 1, 0],
                "pitch_radius_mm": RP_GEAR,
                "depth_mm": FACE_W,
            },
        ],
        "intermediate_stage2_pinion": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [X_INTERMEDIATE, Y_STAGE_2, Z_SHAFT],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * GEAR_BORE_R,
                "depth_mm": FACE_W,
            },
            {
                "name": "stage2_mesh",
                "type": "gear_mesh",
                "xyz_mm": [X_INTERMEDIATE, Y_STAGE_2, Z_SHAFT],
                "axis": [0, 1, 0],
                "pitch_radius_mm": RP_PINION,
                "depth_mm": FACE_W,
            },
        ],
        "output_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [X_OUTPUT, Y_STAGE_2, Z_SHAFT],
                "axis": [0, 1, 0],
                "diameter_mm": 2.0 * GEAR_BORE_R,
                "depth_mm": FACE_W,
            },
            {
                "name": "stage2_mesh",
                "type": "gear_mesh",
                "xyz_mm": [X_OUTPUT, Y_STAGE_2, Z_SHAFT],
                "axis": [0, 1, 0],
                "pitch_radius_mm": RP_GEAR,
                "depth_mm": FACE_W,
            },
        ],
    },
    "relations": [
        {
            "name": "crank_to_input_shaft_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "shaft_axis",
            "incoming_part": "crank_arm",
            "incoming_port": "input_hub_bore",
            "separation_axis": "-y",
        },
        {
            "name": "handle_spindle_to_crank_press_fit",
            "mate_type": "press_fit",
            "base_part": "crank_arm",
            "base_port": "handle_bore",
            "incoming_part": "crank_handle_spindle",
            "incoming_port": "spindle_axis",
            "separation_axis": "-y",
        },
        {
            "name": "grip_to_spindle_press_fit",
            "mate_type": "press_fit",
            "base_part": "crank_handle_spindle",
            "base_port": "spindle_axis",
            "incoming_part": "crank_grip",
            "incoming_port": "grip_bore",
            "separation_axis": "-y",
        },
        {
            "name": "input_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "shaft_axis",
            "incoming_part": "input_pinion",
            "incoming_port": "shaft_bore",
            "separation_axis": "+y",
        },
        {
            "name": "intermediate_driven_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "intermediate_shaft",
            "base_port": "shaft_axis",
            "incoming_part": "intermediate_driven_gear",
            "incoming_port": "shaft_bore",
            "separation_axis": "-y",
        },
        {
            "name": "intermediate_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "intermediate_shaft",
            "base_port": "shaft_axis",
            "incoming_part": "intermediate_stage2_pinion",
            "incoming_port": "shaft_bore",
            "separation_axis": "+y",
        },
        {
            "name": "output_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "output_shaft",
            "base_port": "shaft_axis",
            "incoming_part": "output_gear",
            "incoming_port": "shaft_bore",
            "separation_axis": "+y",
        },
        {
            "name": "stage1_physical_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "input_pinion",
            "base_port": "stage1_mesh",
            "incoming_part": "intermediate_driven_gear",
            "incoming_port": "stage1_mesh",
            "separation_axis": "+x",
            "offset_mm": CD_STAGE_1,
        },
        {
            "name": "stage2_physical_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "intermediate_stage2_pinion",
            "base_port": "stage2_mesh",
            "incoming_part": "output_gear",
            "incoming_port": "stage2_mesh",
            "separation_axis": "+x",
            "offset_mm": CD_STAGE_2,
        },
    ],
    "motion_joints": [],
    "transmissions": [
        {
            "name": "crank_rigidly_carries_input_shaft",
            "type": "compound_1to1",
            "driving_link": "crank_arm",
            "driven_link": "input_shaft",
            "ratio": 1.0,
        },
        {
            "name": "input_shaft_rigidly_carries_pinion",
            "type": "compound_1to1",
            "driving_link": "input_shaft",
            "driven_link": "input_pinion",
            "ratio": 1.0,
        },
        {
            "name": "stage1_three_to_one",
            "type": "gear_external",
            "driving_link": "input_pinion",
            "driven_link": "intermediate_driven_gear",
            "ratio": -1.0,
        },
        {
            "name": "compound_intermediate_rigid_carry",
            "type": "compound_1to1",
            "driving_link": "intermediate_driven_gear",
            "driven_link": "intermediate_stage2_pinion",
            "ratio": 1.0,
        },
        {
            "name": "stage2_three_to_one",
            "type": "gear_external",
            "driving_link": "intermediate_stage2_pinion",
            "driven_link": "output_gear",
            "ratio": -1.0,
        },
        {
            "name": "output_gear_rigidly_carries_output_shaft",
            "type": "compound_1to1",
            "driving_link": "output_gear",
            "driven_link": "output_shaft",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("open_frame_hand_cranked_9_to_1_spur_reducer")

    def horizontal_cylinder(radius, length, xyz):
        part = Cylinder(
            radius,
            length,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        return part.moved(Location(xyz, (-90, 0, 0)))

    def make_bearing_pedestal(x, y):
        post = Box(
            PEDESTAL_W,
            BEARING_W,
            PEDESTAL_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((x, y, BASE_H)))

        outer_ring = horizontal_cylinder(
            BEARING_OUTER_R,
            BEARING_W,
            (x, y, Z_SHAFT),
        )
        bore_tool = horizontal_cylinder(
            JOURNAL_BORE_R,
            BEARING_W + 2.0,
            (x, y, Z_SHAFT),
        )

        return (post + outer_ring) - bore_tool

    def make_crank_arm():
        hub = Cylinder(
            CRANK_HUB_R,
            CRANK_T,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        handle_boss = Cylinder(
            CRANK_HANDLE_BOSS_R,
            CRANK_T,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Location((CRANK_THROW, 0, 0)))

        arm_bar = Box(
            CRANK_THROW,
            2.0 * CRANK_ARM_HALF_W,
            CRANK_T,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Location((CRANK_THROW / 2.0, 0, 0)))

        shaft_bore = Cylinder(
            GEAR_BORE_R,
            CRANK_T + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        handle_bore = Cylinder(
            HANDLE_PIN_R - 0.005,
            CRANK_T + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Location((CRANK_THROW, 0, 0)))

        arm = (hub + arm_bar + handle_boss) - shaft_bore - handle_bore
        return arm.moved(Location((X_INPUT, CRANK_Y, Z_SHAFT), (-90, 0, 0)))

    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((BASE_X_CENTER, BASE_Y_CENTER, 0)))
    a.add(base, "base|dof=fixed")

    bearing_specs = [
        ("input_low_bearing", X_INPUT, Y_BEARING_LOW),
        ("input_high_bearing", X_INPUT, Y_BEARING_HIGH),
        ("intermediate_low_bearing", X_INTERMEDIATE, Y_BEARING_LOW),
        ("intermediate_high_bearing", X_INTERMEDIATE, Y_BEARING_HIGH),
        ("output_low_bearing", X_OUTPUT, Y_BEARING_LOW),
        ("output_high_bearing", X_OUTPUT, Y_OUTPUT_HIGH_BEARING),
    ]
    for name, x, y in bearing_specs:
        a.add(
            make_bearing_pedestal(x, y),
            f"{name}|dof=fixed|mount=base",
        )

    input_shaft = horizontal_cylinder(
        SHAFT_R,
        INPUT_SHAFT_LENGTH,
        (X_INPUT, INPUT_SHAFT_Y_CENTER, Z_SHAFT),
    )
    intermediate_shaft = horizontal_cylinder(
        SHAFT_R,
        SHAFT_LENGTH,
        (X_INTERMEDIATE, SHAFT_Y_CENTER, Z_SHAFT),
    )
    output_shaft = horizontal_cylinder(
        SHAFT_R,
        SHAFT_LENGTH,
        (X_OUTPUT, SHAFT_Y_CENTER, Z_SHAFT),
    )

    a.add(
        input_shaft,
        "input_shaft|dof=spin|spin_axis=z|"
        "mount=input_low_bearing,input_high_bearing",
    )
    a.add(
        intermediate_shaft,
        "intermediate_shaft|dof=spin|spin_axis=z|"
        "mount=intermediate_low_bearing,intermediate_high_bearing",
    )
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=z|"
        "mount=output_low_bearing,output_high_bearing",
    )

    input_pinion = make_gear(
        M, Z_PINION, FACE_W, 2.0 * GEAR_BORE_R
    ).moved(Location((X_INPUT, Y_STAGE_1, Z_SHAFT), (-90, 0, 0)))

    intermediate_driven_gear = make_gear(
        M, Z_GEAR, FACE_W, 2.0 * GEAR_BORE_R
    )
    intermediate_driven_gear = intermediate_driven_gear.rotate(
        b3d.Axis.Z, DRIVEN_GEAR_PHASE_DEG
    ).moved(
        Location((X_INTERMEDIATE, Y_STAGE_1, Z_SHAFT), (-90, 0, 0))
    )

    intermediate_stage2_pinion = make_gear(
        M, Z_PINION, FACE_W, 2.0 * GEAR_BORE_R
    ).moved(
        Location((X_INTERMEDIATE, Y_STAGE_2, Z_SHAFT), (-90, 0, 0))
    )

    output_gear = make_gear(
        M, Z_GEAR, FACE_W, 2.0 * GEAR_BORE_R
    )
    output_gear = output_gear.rotate(
        b3d.Axis.Z, DRIVEN_GEAR_PHASE_DEG
    ).moved(
        Location((X_OUTPUT, Y_STAGE_2, Z_SHAFT), (-90, 0, 0))
    )

    a.add(
        input_pinion,
        "input_pinion|dof=spin|spin_axis=z|mesh_id=stage1|"
        "mount=input_shaft",
    )
    a.add(
        intermediate_driven_gear,
        "intermediate_driven_gear|dof=spin|spin_axis=z|mesh_id=stage1|"
        "mount=intermediate_shaft",
    )
    a.add(
        intermediate_stage2_pinion,
        "intermediate_stage2_pinion|dof=spin|spin_axis=z|mesh_id=stage2|"
        "mount=intermediate_shaft",
    )
    a.add(
        output_gear,
        "output_gear|dof=spin|spin_axis=z|mesh_id=stage2|"
        "mount=output_shaft",
    )

    crank_arm = make_crank_arm()
    a.add(
        crank_arm,
        "crank_arm|dof=spin|driver=True|spin_axis=z|mount=input_shaft",
    )

    handle_spindle = horizontal_cylinder(
        HANDLE_PIN_R,
        HANDLE_PIN_LENGTH,
        (HANDLE_X, HANDLE_PIN_Y, HANDLE_Z),
    )
    a.add(
        handle_spindle,
        "crank_handle_spindle|dof=fixed|mount=crank_arm",
    )

    grip_outer = horizontal_cylinder(
        GRIP_OUTER_R,
        GRIP_LENGTH,
        (HANDLE_X, GRIP_Y, HANDLE_Z),
    )
    grip_bore_tool = horizontal_cylinder(
        GRIP_BORE_R,
        GRIP_LENGTH + 2.0,
        (HANDLE_X, GRIP_Y, HANDLE_Z),
    )
    crank_grip = grip_outer - grip_bore_tool
    a.add(
        crank_grip,
        "crank_grip|dof=fixed|mount=crank_handle_spindle",
    )

    return a.build()