# --------------------------- drivetrain arithmetic ---------------------------

M = 1.5

Z_INPUT_PINION = 16
Z_STAGE1_WHEEL = 48
Z_STAGE2_PINION = 16
Z_OUTPUT_WHEEL = 48

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

CD_STAGE1 = center_dist(Z_INPUT_PINION, Z_STAGE1_WHEEL)
CD_STAGE2 = center_dist(Z_STAGE2_PINION, Z_OUTPUT_WHEEL)

STAGE1_RATIO = Z_STAGE1_WHEEL / Z_INPUT_PINION
STAGE2_RATIO = Z_OUTPUT_WHEEL / Z_STAGE2_PINION
TOTAL_REDUCTION = STAGE1_RATIO * STAGE2_RATIO

# Every gear center follows from the tooth-count arithmetic.
X_INPUT = 0.0
X_LAYSHAFT = X_INPUT + CD_STAGE1
X_OUTPUT = X_LAYSHAFT + CD_STAGE2
Y_SHAFTS = 0.0

GEAR_FACE = 10.0
STAGE1_Z = 22.0
STAGE2_Z = STAGE1_Z + GEAR_FACE + 10.0

SHAFT_R = 4.0
SHAFT_D = 2.0 * SHAFT_R
PRESS_BORE_D = 2.0 * (SHAFT_R - 0.005)
RUNNING_BORE_D = 2.0 * (SHAFT_R + 0.05)

SHAFT_Z = 4.0
INPUT_SHAFT_H = 84.0
LAYSHAFT_H = 76.0
OUTPUT_SHAFT_H = 84.0

BEARING_INNER_R = SHAFT_R + 0.05
BEARING_OUTER_R = 10.0
BEARING_H = 8.0

BASE_CX = (X_INPUT + X_OUTPUT) / 2.0
BASE_L = 180.0
BASE_W = 100.0
BASE_H = 4.0

LOWER_BEARING_Z = BASE_H
TOP_PLATE_Z = 62.0
TOP_PLATE_H = 6.0
UPPER_BEARING_Z = TOP_PLATE_Z + TOP_PLATE_H

POST_SIZE = 12.0
POST_H = TOP_PLATE_Z - BASE_H
POST_X_LEFT = BASE_CX - BASE_L / 2.0 + 8.0
POST_X_RIGHT = BASE_CX + BASE_L / 2.0 - 8.0
POST_Y_FRONT = -(BASE_W / 2.0 - 7.0)
POST_Y_REAR = BASE_W / 2.0 - 7.0

CRANK_ARM_Z = SHAFT_Z + INPUT_SHAFT_H - 4.0
CRANK_ARM_H = 6.0
CRANK_THROW = 27.0
CRANK_ARM_HALF_W = 6.0
CRANK_HANDLE_R = 5.0
CRANK_HANDLE_H = 28.0

PHASE_STAGE1_WHEEL_DEG = 180.0 / Z_STAGE1_WHEEL
PHASE_OUTPUT_WHEEL_DEG = 180.0 / Z_OUTPUT_WHEEL


MECHANISM = {
    "name": "hand_cranked_two_stage_spur_reducer",
    "output_link": "output_shaft",
    "watch_links": [
        "input_crank",
        "input_shaft",
        "layshaft",
        "output_shaft",
        "stage1_input_pinion",
        "stage1_wheel",
        "stage2_pinion",
        "stage2_output_wheel",
    ],
    "ports_by_link": {
        "input_crank": [
            {
                "name": "shaft_press_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CRANK_ARM_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": CRANK_ARM_H,
            }
        ],
        "input_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    UPPER_BEARING_Z - SHAFT_Z + BEARING_H / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "stage1_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    STAGE1_Z - SHAFT_Z + GEAR_FACE / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "crank_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    CRANK_ARM_Z - SHAFT_Z + CRANK_ARM_H / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": CRANK_ARM_H,
            },
        ],
        "layshaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    UPPER_BEARING_Z - SHAFT_Z + BEARING_H / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "stage1_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    STAGE1_Z - SHAFT_Z + GEAR_FACE / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage2_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    STAGE2_Z - SHAFT_Z + GEAR_FACE / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE,
            },
        ],
        "output_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    UPPER_BEARING_Z - SHAFT_Z + BEARING_H / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "stage2_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    STAGE2_Z - SHAFT_Z + GEAR_FACE / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE,
            },
        ],
        "stage1_input_pinion": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage1_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_INPUT_PINION),
                "depth_mm": GEAR_FACE,
            },
        ],
        "stage1_wheel": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage1_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_STAGE1_WHEEL),
                "depth_mm": GEAR_FACE,
            },
        ],
        "stage2_pinion": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage2_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_STAGE2_PINION),
                "depth_mm": GEAR_FACE,
            },
        ],
        "stage2_output_wheel": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "stage2_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_OUTPUT_WHEEL),
                "depth_mm": GEAR_FACE,
            },
        ],
        "input_lower_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "input_upper_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "layshaft_lower_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "layshaft_upper_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "output_lower_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "output_upper_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
    },
    "relations": [
        {
            "name": "crank_to_input_shaft",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "crank_seat",
            "incoming_part": "input_crank",
            "incoming_port": "shaft_press_bore",
        },
        {
            "name": "input_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "stage1_seat",
            "incoming_part": "stage1_input_pinion",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "stage1_wheel_press_fit",
            "mate_type": "press_fit",
            "base_part": "layshaft",
            "base_port": "stage1_seat",
            "incoming_part": "stage1_wheel",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "stage2_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "layshaft",
            "base_port": "stage2_seat",
            "incoming_part": "stage2_pinion",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "output_wheel_press_fit",
            "mate_type": "press_fit",
            "base_part": "output_shaft",
            "base_port": "stage2_seat",
            "incoming_part": "stage2_output_wheel",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "stage1_gear_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "stage1_input_pinion",
            "base_port": "stage1_mesh",
            "incoming_part": "stage1_wheel",
            "incoming_port": "stage1_mesh",
            "separation_axis": "+x",
        },
        {
            "name": "stage2_gear_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "stage2_pinion",
            "base_port": "stage2_mesh",
            "incoming_part": "stage2_output_wheel",
            "incoming_port": "stage2_mesh",
            "separation_axis": "+x",
        },
        {
            "name": "input_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_lower_bearing",
            "base_port": "journal_bore",
            "incoming_part": "input_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "input_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_upper_bearing",
            "base_port": "journal_bore",
            "incoming_part": "input_shaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "layshaft_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "layshaft_lower_bearing",
            "base_port": "journal_bore",
            "incoming_part": "layshaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "layshaft_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "layshaft_upper_bearing",
            "base_port": "journal_bore",
            "incoming_part": "layshaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "output_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_lower_bearing",
            "base_port": "journal_bore",
            "incoming_part": "output_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "output_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_upper_bearing",
            "base_port": "journal_bore",
            "incoming_part": "output_shaft",
            "incoming_port": "upper_journal",
        },
    ],
    "motion_joints": [],
    "transmissions": [
        {
            "name": "crank_input_coupling",
            "type": "compound_1to1",
            "driving_link": "input_crank",
            "driven_link": "input_shaft",
            "ratio": 1.0,
        },
        {
            "name": "input_shaft_pinion_coupling",
            "type": "compound_1to1",
            "driving_link": "input_shaft",
            "driven_link": "stage1_input_pinion",
            "ratio": 1.0,
        },
        {
            "name": "first_reduction",
            "type": "gear_external",
            "driving_link": "stage1_input_pinion",
            "driven_link": "stage1_wheel",
            "ratio": 0,
        },
        {
            "name": "layshaft_compound_coupling",
            "type": "compound_1to1",
            "driving_link": "stage1_wheel",
            "driven_link": "stage2_pinion",
            "ratio": 1.0,
        },
        {
            "name": "second_reduction",
            "type": "gear_external",
            "driving_link": "stage2_pinion",
            "driven_link": "stage2_output_wheel",
            "ratio": 0,
        },
        {
            "name": "output_wheel_shaft_coupling",
            "type": "compound_1to1",
            "driving_link": "stage2_output_wheel",
            "driven_link": "output_shaft",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("hand_cranked_two_stage_spur_reducer")

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

    def bored_plate(length, width, height, local_hole_xs, hole_r):
        plate = Box(
            length,
            width,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        cutters = None
        for local_x in local_hole_xs:
            cut = Cylinder(
                hole_r,
                height + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Location((local_x, 0.0, -1.0)))
            cutters = cut if cutters is None else cutters + cut
        return plate - cutters

    # Grounded base.
    baseplate = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((BASE_CX, 0.0, 0.0)))
    a.add(baseplate, "baseplate|dof=fixed")

    # Four columns touch the base and the underside of the upper plate.
    post_locations = [
        (POST_X_LEFT, POST_Y_FRONT),
        (POST_X_LEFT, POST_Y_REAR),
        (POST_X_RIGHT, POST_Y_FRONT),
        (POST_X_RIGHT, POST_Y_REAR),
    ]
    post_names = [
        "front_left_post",
        "rear_left_post",
        "front_right_post",
        "rear_right_post",
    ]

    for name, (px, py) in zip(post_names, post_locations):
        post = Box(
            POST_SIZE,
            POST_SIZE,
            POST_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, BASE_H)))
        a.add(post, f"{name}|dof=fixed|mount=baseplate")

    local_shaft_xs = [
        X_INPUT - BASE_CX,
        X_LAYSHAFT - BASE_CX,
        X_OUTPUT - BASE_CX,
    ]
    upper_plate = bored_plate(
        BASE_L,
        BASE_W,
        TOP_PLATE_H,
        local_shaft_xs,
        BEARING_INNER_R + 0.30,
    ).moved(Location((BASE_CX, 0.0, TOP_PLATE_Z)))
    a.add(
        upper_plate,
        "upper_plate|dof=fixed|mount="
        "front_left_post,rear_left_post,front_right_post,rear_right_post",
    )

    # Real lower and upper annular journal bearings.
    bearing_specifications = [
        (
            "input_lower_bearing",
            X_INPUT,
            LOWER_BEARING_Z,
            "baseplate",
        ),
        (
            "layshaft_lower_bearing",
            X_LAYSHAFT,
            LOWER_BEARING_Z,
            "baseplate",
        ),
        (
            "output_lower_bearing",
            X_OUTPUT,
            LOWER_BEARING_Z,
            "baseplate",
        ),
        (
            "input_upper_bearing",
            X_INPUT,
            UPPER_BEARING_Z,
            "upper_plate",
        ),
        (
            "layshaft_upper_bearing",
            X_LAYSHAFT,
            UPPER_BEARING_Z,
            "upper_plate",
        ),
        (
            "output_upper_bearing",
            X_OUTPUT,
            UPPER_BEARING_Z,
            "upper_plate",
        ),
    ]

    for name, bx, bz, mount_name in bearing_specifications:
        bearing = annulus(
            BEARING_OUTER_R,
            BEARING_INNER_R,
            BEARING_H,
        ).moved(Location((bx, Y_SHAFTS, bz)))
        a.add(bearing, f"{name}|dof=fixed|mount={mount_name}")

    # Shafts start directly on the base top and pass through both bearings.
    input_shaft = Cylinder(
        SHAFT_R,
        INPUT_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((X_INPUT, Y_SHAFTS, SHAFT_Z)))
    a.add(
        input_shaft,
        "input_shaft|dof=spin|spin_axis=z|"
        "mount=input_lower_bearing,input_upper_bearing",
    )

    layshaft = Cylinder(
        SHAFT_R,
        LAYSHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((X_LAYSHAFT, Y_SHAFTS, SHAFT_Z)))
    a.add(
        layshaft,
        "layshaft|dof=spin|spin_axis=z|"
        "mount=layshaft_lower_bearing,layshaft_upper_bearing",
    )

    output_shaft = Cylinder(
        SHAFT_R,
        OUTPUT_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((X_OUTPUT, Y_SHAFTS, SHAFT_Z)))
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=z|"
        "mount=output_lower_bearing,output_upper_bearing",
    )

    # First stage: exact pitch-center spacing CD_STAGE1.
    stage1_input_pinion = make_gear(
        M,
        Z_INPUT_PINION,
        GEAR_FACE,
        PRESS_BORE_D,
    ).moved(
        Location(
            (X_INPUT, Y_SHAFTS, STAGE1_Z),
            (0.0, 0.0, 0.0),
        )
    )
    a.add(
        stage1_input_pinion,
        "stage1_input_pinion|dof=spin|spin_axis=z|"
        "mesh_id=stage1|mount=input_shaft",
    )

    stage1_wheel = make_gear(
        M,
        Z_STAGE1_WHEEL,
        GEAR_FACE,
        PRESS_BORE_D,
    ).moved(
        Location(
            (X_INPUT + CD_STAGE1, Y_SHAFTS, STAGE1_Z),
            (0.0, 0.0, PHASE_STAGE1_WHEEL_DEG),
        )
    )
    a.add(
        stage1_wheel,
        "stage1_wheel|dof=spin|spin_axis=z|"
        "mesh_id=stage1|mount=layshaft",
    )

    # Second stage occupies a distinct axial station.
    stage2_pinion = make_gear(
        M,
        Z_STAGE2_PINION,
        GEAR_FACE,
        PRESS_BORE_D,
    ).moved(
        Location(
            (X_LAYSHAFT, Y_SHAFTS, STAGE2_Z),
            (0.0, 0.0, 0.0),
        )
    )
    a.add(
        stage2_pinion,
        "stage2_pinion|dof=spin|spin_axis=z|"
        "mesh_id=stage2|mount=layshaft",
    )

    stage2_output_wheel = make_gear(
        M,
        Z_OUTPUT_WHEEL,
        GEAR_FACE,
        PRESS_BORE_D,
    ).moved(
        Location(
            (X_LAYSHAFT + CD_STAGE2, Y_SHAFTS, STAGE2_Z),
            (0.0, 0.0, PHASE_OUTPUT_WHEEL_DEG),
        )
    )
    a.add(
        stage2_output_wheel,
        "stage2_output_wheel|dof=spin|spin_axis=z|"
        "mesh_id=stage2|mount=output_shaft",
    )

    # Crank arm: rounded-ended plate with an explicit shaft press-fit bore.
    arm_box = Box(
        CRANK_THROW,
        2.0 * CRANK_ARM_HALF_W,
        CRANK_ARM_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_THROW / 2.0, 0.0, 0.0)))

    arm_root = Cylinder(
        CRANK_ARM_HALF_W,
        CRANK_ARM_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    arm_tip = Cylinder(
        CRANK_ARM_HALF_W,
        CRANK_ARM_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_THROW, 0.0, 0.0)))

    crank_bore = Cylinder(
        PRESS_BORE_D / 2.0,
        CRANK_ARM_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))

    input_crank = (arm_box + arm_root + arm_tip - crank_bore).moved(
        Location((X_INPUT, Y_SHAFTS, CRANK_ARM_Z))
    )
    a.add(
        input_crank,
        "input_crank|dof=spin|spin_axis=z|driver=True|mount=input_shaft",
    )

    # Fixed hand knob touches the crank arm's upper face.
    crank_handle = Cylinder(
        CRANK_HANDLE_R,
        CRANK_HANDLE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                X_INPUT + CRANK_THROW,
                Y_SHAFTS,
                CRANK_ARM_Z + CRANK_ARM_H,
            )
        )
    )
    a.add(
        crank_handle,
        "crank_handle|dof=fixed|mount=input_crank",
    )

    return a.build()