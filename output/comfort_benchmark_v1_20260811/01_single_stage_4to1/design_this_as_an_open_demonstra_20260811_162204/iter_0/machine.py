# -------------------------- drivetrain arithmetic ---------------------------

M = 1.5
Z_INPUT = 12
Z_OUTPUT = 48
REDUCTION = Z_OUTPUT / Z_INPUT

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

R_INPUT_PITCH = pitch_r(Z_INPUT)
R_OUTPUT_PITCH = pitch_r(Z_OUTPUT)
CD_STAGE1 = center_dist(Z_INPUT, Z_OUTPUT)  # exact pitch-center distance: 45 mm

X_INPUT = -CD_STAGE1 / 2.0
X_OUTPUT = X_INPUT + CD_STAGE1
Y_SHAFT = 0.0

GEAR_FACE = 6.0
SHAFT_R = 4.0
SHAFT_D = 2.0 * SHAFT_R
PRESS_BORE_R = SHAFT_R - 0.005
PRESS_BORE_D = 2.0 * PRESS_BORE_R
RUNNING_BORE_R = SHAFT_R + 0.05
RUNNING_BORE_D = 2.0 * RUNNING_BORE_R

BASE_H = 6.0
OUTPUT_OUTSIDE_R = R_OUTPUT_PITCH + M
BASE_EDGE_MARGIN = OUTPUT_OUTSIDE_R + 8.0
BASE_L = CD_STAGE1 + 2.0 * BASE_EDGE_MARGIN
BASE_W = 80.0

THRUST_Z = BASE_H
THRUST_H = 1.0
SHAFT_Z = THRUST_Z + THRUST_H

SHAFT_COLLAR_H = 0.8
SHAFT_COLLAR_R = 5.8
THRUST_OUTER_R = 6.0

PEDESTAL_Z = BASE_H
PEDESTAL_H = 2.0
PEDESTAL_INNER_R = SHAFT_COLLAR_R + 0.30
PEDESTAL_OUTER_R = 9.0

LOWER_BEARING_Z = PEDESTAL_Z + PEDESTAL_H
BEARING_H = 5.0
UPPER_BEARING_Z = LOWER_BEARING_Z + BEARING_H
BEARING_OUTER_R = 8.0

GEAR_Z = UPPER_BEARING_Z + BEARING_H + 1.0
GEAR_TOP = GEAR_Z + GEAR_FACE

CRANK_GAP = 3.0
CRANK_Z = GEAR_TOP + CRANK_GAP
CRANK_T = 5.0
CRANK_THROW = 28.0
CRANK_ARM_W = 7.0
CRANK_HUB_R = 8.0
CRANK_END_R = 6.0
CRANK_PIN_R = 3.0
GRIP_CLEARANCE = 0.05
GRIP_BORE_R = CRANK_PIN_R + GRIP_CLEARANCE
GRIP_OUTER_R = 5.5
GRIP_H = 16.0
GRIP_Z = CRANK_Z + CRANK_T

INPUT_SHAFT_H = CRANK_Z + CRANK_T - SHAFT_Z
OUTPUT_SHAFT_H = GEAR_TOP + 10.0 - SHAFT_Z

# The output gear is shifted half of one output tooth pitch to avoid tip-to-tip
# initial alignment while retaining the exact computed center distance.
INPUT_GEAR_PHASE_DEG = 0.0
OUTPUT_GEAR_PHASE_DEG = 180.0 / Z_OUTPUT

BOLT_R = 2.5
BOLT_HOLE_R = BOLT_R + 0.10
BOLT_HEAD_R = 4.5
BOLT_HEAD_H = 3.0
BOLT_MARGIN_X = 9.0
BOLT_MARGIN_Y = 9.0
BOLT_POSITIONS = [
    (-BASE_L / 2.0 + BOLT_MARGIN_X, -BASE_W / 2.0 + BOLT_MARGIN_Y),
    (-BASE_L / 2.0 + BOLT_MARGIN_X,  BASE_W / 2.0 - BOLT_MARGIN_Y),
    ( BASE_L / 2.0 - BOLT_MARGIN_X, -BASE_W / 2.0 + BOLT_MARGIN_Y),
    ( BASE_L / 2.0 - BOLT_MARGIN_X,  BASE_W / 2.0 - BOLT_MARGIN_Y),
]


MECHANISM = {
    "name": "open_frame_hand_cranked_4_to_1_spur_reducer",
    "output_link": "output_shaft",
    "watch_links": [
        "hand_crank",
        "input_shaft",
        "input_pinion",
        "output_gear",
        "output_shaft",
    ],
    "ports_by_link": {
        "base": [
            {
                "name": "input_axis",
                "type": "cylindrical",
                "xyz_mm": [X_INPUT, Y_SHAFT, BASE_H],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
            },
            {
                "name": "output_axis",
                "type": "cylindrical",
                "xyz_mm": [X_OUTPUT, Y_SHAFT, BASE_H],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
            },
        ],
        "input_lower_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0, 0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "input_upper_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0, 0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "output_lower_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0, 0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "output_upper_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0, 0, BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": RUNNING_BORE_D,
                "depth_mm": BEARING_H,
            }
        ],
        "input_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, LOWER_BEARING_Z + BEARING_H / 2.0 - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, UPPER_BEARING_Z + BEARING_H / 2.0 - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "pinion_seat",
                "type": "shaft",
                "xyz_mm": [0, 0, GEAR_Z + GEAR_FACE / 2.0 - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "crank_seat",
                "type": "shaft",
                "xyz_mm": [0, 0, CRANK_Z + CRANK_T / 2.0 - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": CRANK_T,
            },
        ],
        "output_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, LOWER_BEARING_Z + BEARING_H / 2.0 - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, UPPER_BEARING_Z + BEARING_H / 2.0 - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": BEARING_H,
            },
            {
                "name": "gear_seat",
                "type": "shaft",
                "xyz_mm": [0, 0, GEAR_Z + GEAR_FACE / 2.0 - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "output_end",
                "type": "shaft",
                "xyz_mm": [0, 0, OUTPUT_SHAFT_H],
                "axis": [0, 0, 1],
                "diameter_mm": SHAFT_D,
            },
        ],
        "input_pinion": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0, 0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "mesh",
                "type": "gear_mesh",
                "xyz_mm": [0, 0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": R_INPUT_PITCH,
                "depth_mm": GEAR_FACE,
            },
        ],
        "output_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0, 0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "mesh",
                "type": "gear_mesh",
                "xyz_mm": [0, 0, GEAR_FACE / 2.0],
                "axis": [0, 0, 1],
                "pitch_radius_mm": R_OUTPUT_PITCH,
                "depth_mm": GEAR_FACE,
            },
        ],
        "hand_crank": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0, 0, CRANK_T / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": PRESS_BORE_D,
                "depth_mm": CRANK_T,
            },
            {
                "name": "grip_pin",
                "type": "shaft",
                "xyz_mm": [0, CRANK_THROW, CRANK_T + GRIP_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": GRIP_H,
            },
        ],
        "crank_grip": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [0, 0, GRIP_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * GRIP_BORE_R,
                "depth_mm": GRIP_H,
            }
        ],
    },
    "relations": [
        {
            "name": "input_lower_running_fit",
            "mate_type": "journal_bearing",
            "base_part": "input_lower_bearing",
            "base_port": "journal",
            "incoming_part": "input_shaft",
            "incoming_port": "lower_journal",
            "offset_mm": 0.0,
        },
        {
            "name": "input_upper_running_fit",
            "mate_type": "journal_bearing",
            "base_part": "input_upper_bearing",
            "base_port": "journal",
            "incoming_part": "input_shaft",
            "incoming_port": "upper_journal",
            "offset_mm": 0.0,
        },
        {
            "name": "output_lower_running_fit",
            "mate_type": "journal_bearing",
            "base_part": "output_lower_bearing",
            "base_port": "journal",
            "incoming_part": "output_shaft",
            "incoming_port": "lower_journal",
            "offset_mm": 0.0,
        },
        {
            "name": "output_upper_running_fit",
            "mate_type": "journal_bearing",
            "base_part": "output_upper_bearing",
            "base_port": "journal",
            "incoming_part": "output_shaft",
            "incoming_port": "upper_journal",
            "offset_mm": 0.0,
        },
        {
            "name": "pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "pinion_seat",
            "incoming_part": "input_pinion",
            "incoming_port": "shaft_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "output_gear_press_fit",
            "mate_type": "press_fit",
            "base_part": "output_shaft",
            "base_port": "gear_seat",
            "incoming_part": "output_gear",
            "incoming_port": "shaft_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "crank_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "crank_seat",
            "incoming_part": "hand_crank",
            "incoming_port": "shaft_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "crank_grip_running_fit",
            "mate_type": "journal_bearing",
            "base_part": "hand_crank",
            "base_port": "grip_pin",
            "incoming_part": "crank_grip",
            "incoming_port": "pin_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "stage1_external_spur_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "input_pinion",
            "base_port": "mesh",
            "incoming_part": "output_gear",
            "incoming_port": "mesh",
            "separation_axis": "+x",
            "axis_angle_deg": OUTPUT_GEAR_PHASE_DEG,
        },
    ],
    "motion_joints": [
        {
            "name": "input_shaft_hinge",
            "parent": "",
            "child": "input_shaft",
            "type": "hinge",
            "axis": [0, 0, 1],
            "pos_mm": [0, 0, 0],
        },
        {
            "name": "output_shaft_hinge",
            "parent": "",
            "child": "output_shaft",
            "type": "hinge",
            "axis": [0, 0, 1],
            "pos_mm": [0, 0, 0],
        },
        {
            "name": "crank_grip_hinge",
            "parent": "hand_crank",
            "child": "crank_grip",
            "type": "hinge",
            "axis": [0, 0, 1],
            "pos_mm": [0, 0, 0],
        },
    ],
    "transmissions": [
        {
            "name": "crank_to_input_shaft",
            "type": "compound_1to1",
            "driving_link": "hand_crank",
            "driven_link": "input_shaft",
            "ratio": 1.0,
        },
        {
            "name": "input_shaft_to_pinion",
            "type": "compound_1to1",
            "driving_link": "input_shaft",
            "driven_link": "input_pinion",
            "ratio": 1.0,
        },
        {
            "name": "spur_reduction_stage",
            "type": "gear_external",
            "driving_link": "input_pinion",
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
    def cylinder_base(radius, height):
        return Cylinder(
            radius,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )

    def annulus(outer_r, inner_r, height):
        outer = cylinder_base(outer_r, height)
        cutter = cylinder_base(inner_r, height + 1.0).moved(
            Location((0, 0, -0.5))
        )
        return outer - cutter

    def place_at_base(shape, x, y, z_base, phase_deg=0.0):
        rotated = shape.moved(
            Location((0, 0, 0), (0, 0, phase_deg))
        )
        z_shift = z_base - rotated.bounding_box().min.Z
        return rotated.moved(Location((x, y, z_shift)))

    def make_supported_shaft(height):
        shaft_body = cylinder_base(SHAFT_R, height)
        thrust_collar = cylinder_base(SHAFT_COLLAR_R, SHAFT_COLLAR_H)
        return shaft_body + thrust_collar

    def make_bench_bolt():
        shank = cylinder_base(BOLT_R, BASE_H + 0.5)
        head = cylinder_base(BOLT_HEAD_R, BOLT_HEAD_H).moved(
            Location((0, 0, BASE_H))
        )
        return shank + head

    def make_crank():
        hub = cylinder_base(CRANK_HUB_R, CRANK_T)

        arm = Box(
            CRANK_ARM_W,
            CRANK_THROW,
            CRANK_T,
            align=(Align.CENTER, Align.MIN, Align.MIN),
        )

        end_boss = cylinder_base(CRANK_END_R, CRANK_T).moved(
            Location((0, CRANK_THROW, 0))
        )

        crank_pin = cylinder_base(CRANK_PIN_R, GRIP_H).moved(
            Location((0, CRANK_THROW, CRANK_T))
        )

        crank_blank = hub + arm + end_boss + crank_pin

        shaft_bore = cylinder_base(
            PRESS_BORE_R,
            CRANK_T + 1.0,
        ).moved(Location((0, 0, -0.5)))

        return crank_blank - shaft_bore

    a = AssemblyHelper("open_frame_hand_cranked_4_to_1_spur_reducer")

    # Stable bench base with four actual through-holes.
    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    bolt_hole_tools = None
    for bx, by in BOLT_POSITIONS:
        tool = cylinder_base(BOLT_HOLE_R, BASE_H + 1.0).moved(
            Location((bx, by, -0.5))
        )
        bolt_hole_tools = tool if bolt_hole_tools is None else bolt_hole_tools + tool
    base = base - bolt_hole_tools
    a.add(base, "base|dof=fixed")

    for index, (bx, by) in enumerate(BOLT_POSITIONS, start=1):
        bolt = make_bench_bolt().moved(Location((bx, by, 0)))
        a.add(
            bolt,
            f"bench_bolt_{index}|dof=fixed|mount=base",
        )

    # Exposed thrust washers and low annular bearing pedestals.
    thrust_shape = annulus(
        THRUST_OUTER_R,
        RUNNING_BORE_R,
        THRUST_H,
    )
    pedestal_shape = annulus(
        PEDESTAL_OUTER_R,
        PEDESTAL_INNER_R,
        PEDESTAL_H,
    )
    bearing_shape = annulus(
        BEARING_OUTER_R,
        RUNNING_BORE_R,
        BEARING_H,
    )

    input_thrust = thrust_shape.moved(
        Location((X_INPUT, Y_SHAFT, THRUST_Z))
    )
    output_thrust = thrust_shape.moved(
        Location((X_OUTPUT, Y_SHAFT, THRUST_Z))
    )
    a.add(input_thrust, "input_thrust_washer|dof=fixed|mount=base")
    a.add(output_thrust, "output_thrust_washer|dof=fixed|mount=base")

    input_pedestal = pedestal_shape.moved(
        Location((X_INPUT, Y_SHAFT, PEDESTAL_Z))
    )
    output_pedestal = pedestal_shape.moved(
        Location((X_OUTPUT, Y_SHAFT, PEDESTAL_Z))
    )
    a.add(input_pedestal, "input_bearing_pedestal|dof=fixed|mount=base")
    a.add(output_pedestal, "output_bearing_pedestal|dof=fixed|mount=base")

    input_lower_bearing = bearing_shape.moved(
        Location((X_INPUT, Y_SHAFT, LOWER_BEARING_Z))
    )
    input_upper_bearing = bearing_shape.moved(
        Location((X_INPUT, Y_SHAFT, UPPER_BEARING_Z))
    )
    output_lower_bearing = bearing_shape.moved(
        Location((X_OUTPUT, Y_SHAFT, LOWER_BEARING_Z))
    )
    output_upper_bearing = bearing_shape.moved(
        Location((X_OUTPUT, Y_SHAFT, UPPER_BEARING_Z))
    )

    a.add(
        input_lower_bearing,
        "input_lower_bearing|dof=fixed|mount=input_bearing_pedestal",
    )
    a.add(
        input_upper_bearing,
        "input_upper_bearing|dof=fixed|mount=input_lower_bearing",
    )
    a.add(
        output_lower_bearing,
        "output_lower_bearing|dof=fixed|mount=output_bearing_pedestal",
    )
    a.add(
        output_upper_bearing,
        "output_upper_bearing|dof=fixed|mount=output_lower_bearing",
    )

    # Separate rotating shafts with integral thrust collars.
    input_shaft = make_supported_shaft(INPUT_SHAFT_H).moved(
        Location((X_INPUT, Y_SHAFT, SHAFT_Z))
    )
    output_shaft = make_supported_shaft(OUTPUT_SHAFT_H).moved(
        Location((X_OUTPUT, Y_SHAFT, SHAFT_Z))
    )

    a.add(
        input_shaft,
        "input_shaft|dof=spin|spin_axis=z|"
        "mount=input_thrust_washer,input_lower_bearing,input_upper_bearing",
    )
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=z|"
        "mount=output_thrust_washer,output_lower_bearing,output_upper_bearing",
    )

    # Exact-center-distance spur pair using one common module.
    input_pinion_raw = make_gear(
        M,
        Z_INPUT,
        GEAR_FACE,
        PRESS_BORE_D,
    )
    output_gear_raw = make_gear(
        M,
        Z_OUTPUT,
        GEAR_FACE,
        PRESS_BORE_D,
    )

    input_pinion = place_at_base(
        input_pinion_raw,
        X_INPUT,
        Y_SHAFT,
        GEAR_Z,
        INPUT_GEAR_PHASE_DEG,
    )
    output_gear = place_at_base(
        output_gear_raw,
        X_OUTPUT,
        Y_SHAFT,
        GEAR_Z,
        OUTPUT_GEAR_PHASE_DEG,
    )

    a.add(
        input_pinion,
        "input_pinion|dof=spin|spin_axis=z|mesh_id=stage1|mount=input_shaft",
    )
    a.add(
        output_gear,
        "output_gear|dof=spin|spin_axis=z|mesh_id=stage1|mount=output_shaft",
    )

    # Only the hand crank is marked as the driver.
    hand_crank = make_crank().moved(
        Location((X_INPUT, Y_SHAFT, CRANK_Z))
    )
    a.add(
        hand_crank,
        "hand_crank|dof=spin|spin_axis=z|driver=True|mount=input_shaft",
    )

    # Freely rotating visible grip on the crank's integral pin.
    crank_grip = annulus(
        GRIP_OUTER_R,
        GRIP_BORE_R,
        GRIP_H,
    ).moved(
        Location((X_INPUT, Y_SHAFT + CRANK_THROW, GRIP_Z))
    )
    a.add(
        crank_grip,
        "crank_grip|dof=spin|spin_axis=z|mount=hand_crank",
    )

    return a.build()