import math

# ---------------------------------------------------------------------------
# Drivetrain arithmetic -- all gear locations derive from these values.
# ---------------------------------------------------------------------------

M = 1.5
Z_SUN = 18
Z_PLANET = 18
Z_RING = Z_SUN + 2 * Z_PLANET
PLANET_COUNT = 3

assert Z_RING == Z_SUN + 2 * Z_PLANET
assert (Z_SUN + Z_RING) % PLANET_COUNT == 0

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
PLANET_ORBIT_R = SUN_PLANET_CD

assert abs(SUN_PLANET_CD - RING_PLANET_CD) < 1.0e-9

FIXED_RING_REDUCTION = 1.0 + Z_RING / Z_SUN

PLANET_ANGLES_DEG = [
    i * 360.0 / PLANET_COUNT for i in range(PLANET_COUNT)
]
PLANET_CENTERS = [
    (
        PLANET_ORBIT_R * math.cos(math.radians(a)),
        PLANET_ORBIT_R * math.sin(math.radians(a)),
    )
    for a in PLANET_ANGLES_DEG
]
PLANET_PHASE_DEG = [
    (a + 180.0 - 180.0 / Z_PLANET) % (360.0 / Z_PLANET)
    for a in PLANET_ANGLES_DEG
]

GEAR_FACE_W = 8.0
GEAR_Z = 18.0
GEAR_MID_Z = GEAR_Z + GEAR_FACE_W / 2.0

# Corrected ring station: lower face touches the top of the lower bearing.
RING_Z = GEAR_Z + GEAR_FACE_W + 7.0

RING_TIP_R = RING_PITCH_R - M
RING_ROOT_R = RING_PITCH_R + 1.25 * M
RING_OUTER_R = RING_ROOT_R + 4.625

INPUT_SHAFT_R = 4.0
OUTPUT_SHAFT_R = 5.0
PLANET_PIN_R = 3.0

PRESS_INTERFERENCE = 0.005
RUNNING_CLEARANCE = 0.05

SUN_BORE_R = INPUT_SHAFT_R - PRESS_INTERFERENCE
PLANET_BORE_R = PLANET_PIN_R + RUNNING_CLEARANCE
CARRIER_OUTPUT_BORE_R = OUTPUT_SHAFT_R - PRESS_INTERFERENCE
CARRIER_PIN_BORE_R = PLANET_PIN_R - PRESS_INTERFERENCE

BASE_Z = 0.0
BASE_H = 4.0

OUTPUT_BEARING_Z_1 = BASE_Z + BASE_H
OUTPUT_BEARING_H = 4.0
OUTPUT_BEARING_Z_2 = OUTPUT_BEARING_Z_1 + OUTPUT_BEARING_H
OUTPUT_BEARING_TOP = OUTPUT_BEARING_Z_2 + OUTPUT_BEARING_H

CARRIER_Z = OUTPUT_BEARING_TOP
CARRIER_H = 4.0
CARRIER_TOP = CARRIER_Z + CARRIER_H
CARRIER_R = PLANET_ORBIT_R + 8.0

OUTPUT_SHAFT_Z = BASE_Z + BASE_H
OUTPUT_SHAFT_TOP = CARRIER_TOP - 0.2
OUTPUT_SHAFT_H = OUTPUT_SHAFT_TOP - OUTPUT_SHAFT_Z

PLANET_PIN_Z = CARRIER_Z
PLANET_PIN_TOP = GEAR_Z + GEAR_FACE_W + 1.0
PLANET_PIN_H = PLANET_PIN_TOP - PLANET_PIN_Z

INPUT_SHAFT_Z = CARRIER_TOP + 0.2
INPUT_SHAFT_TOP = 52.0
INPUT_SHAFT_H = INPUT_SHAFT_TOP - INPUT_SHAFT_Z

LOWER_INPUT_BEARING_Z = GEAR_Z + GEAR_FACE_W + 2.0
INPUT_BEARING_H = 5.0
UPPER_INPUT_BEARING_Z = 41.0

LOWER_BRIDGE_Z = LOWER_INPUT_BEARING_Z + 1.0
BRIDGE_H = 3.0
LOWER_BRIDGE_TOP = LOWER_BRIDGE_Z + BRIDGE_H
UPPER_BRIDGE_Z = UPPER_INPUT_BEARING_Z + 1.0
UPPER_BRIDGE_TOP = UPPER_BRIDGE_Z + BRIDGE_H

INPUT_THRUST_FLANGE_Z = LOWER_INPUT_BEARING_Z - 1.5
INPUT_THRUST_FLANGE_H = 1.5
INPUT_THRUST_FLANGE_R = INPUT_SHAFT_R + 2.5

BEARING_OUTER_R = 8.5
INPUT_BEARING_BORE_R = INPUT_SHAFT_R + RUNNING_CLEARANCE
OUTPUT_BEARING_BORE_R = OUTPUT_SHAFT_R + RUNNING_CLEARANCE
BEARING_SEAT_R = BEARING_OUTER_R - PRESS_INTERFERENCE

RING_POST_R = RING_ROOT_R + (RING_OUTER_R - RING_ROOT_R) / 2.0
RING_POST_RADIUS = 3.0
RING_POST_Z = BASE_Z + BASE_H
RING_POST_H = GEAR_Z - RING_POST_Z

STANCHION_ORBIT_R = RING_OUTER_R + 5.0
STANCHION_R = 3.0
BRIDGE_R = STANCHION_ORBIT_R + STANCHION_R + 1.0

LOWER_STANCHION_Z = BASE_Z + BASE_H
LOWER_STANCHION_H = LOWER_BRIDGE_Z - LOWER_STANCHION_Z
MID_STANCHION_Z = LOWER_BRIDGE_TOP
MID_STANCHION_H = UPPER_BRIDGE_Z - MID_STANCHION_Z

CRANK_ARM_Z = 48.0
CRANK_ARM_H = 3.0
CRANK_THROW = 25.0
CRANK_ARM_HALF_W = 5.0
GRIP_Z = CRANK_ARM_Z + CRANK_ARM_H
GRIP_H = 14.0
GRIP_R = 4.5

MECHANISM = {
    "name": "hand_driven_fixed_ring_planetary_reducer",
    "output_link": "carrier",
    "watch_links": [
        "sun_gear",
        "planet_1",
        "planet_2",
        "planet_3",
        "carrier",
        "output_shaft",
    ],
    "ports_by_link": {
        "input_shaft": [
            {
                "name": "sun_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, GEAR_MID_Z - INPUT_SHAFT_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    LOWER_INPUT_BEARING_Z
                    + INPUT_BEARING_H / 2.0
                    - INPUT_SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": INPUT_BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    UPPER_INPUT_BEARING_Z
                    + INPUT_BEARING_H / 2.0
                    - INPUT_SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": INPUT_BEARING_H,
            },
        ],
        "sun_gear": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SUN_BORE_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "planet_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": SUN_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "ring_gear": [
            {
                "name": "internal_mesh",
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
            *[
                {
                    "name": f"pin_bore_{i + 1}",
                    "type": "bore",
                    "xyz_mm": [
                        PLANET_CENTERS[i][0],
                        PLANET_CENTERS[i][1],
                        CARRIER_H / 2.0,
                    ],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * CARRIER_PIN_BORE_R,
                    "depth_mm": CARRIER_H,
                }
                for i in range(PLANET_COUNT)
            ],
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
                "diameter_mm": 2.0 * OUTPUT_SHAFT_R,
                "depth_mm": CARRIER_H,
            },
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    OUTPUT_BEARING_Z_1
                    + OUTPUT_BEARING_H / 2.0
                    - OUTPUT_SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_SHAFT_R,
                "depth_mm": OUTPUT_BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    OUTPUT_BEARING_Z_2
                    + OUTPUT_BEARING_H / 2.0
                    - OUTPUT_SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_SHAFT_R,
                "depth_mm": OUTPUT_BEARING_H,
            },
        ],
        "lower_input_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, INPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_BEARING_BORE_R,
                "depth_mm": INPUT_BEARING_H,
            }
        ],
        "upper_input_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, INPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_BEARING_BORE_R,
                "depth_mm": INPUT_BEARING_H,
            }
        ],
        "lower_output_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, OUTPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_BEARING_BORE_R,
                "depth_mm": OUTPUT_BEARING_H,
            }
        ],
        "upper_output_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, OUTPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_BEARING_BORE_R,
                "depth_mm": OUTPUT_BEARING_H,
            }
        ],
        **{
            f"planet_{i + 1}": [
                {
                    "name": "pin_bore",
                    "type": "bore",
                    "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_BORE_R,
                    "depth_mm": GEAR_FACE_W,
                },
                {
                    "name": "sun_mesh",
                    "type": "gear_mesh",
                    "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "pitch_radius_mm": PLANET_PITCH_R,
                    "depth_mm": GEAR_FACE_W,
                },
                {
                    "name": "ring_mesh",
                    "type": "gear_mesh",
                    "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "pitch_radius_mm": PLANET_PITCH_R,
                    "depth_mm": GEAR_FACE_W,
                },
            ]
            for i in range(PLANET_COUNT)
        },
        **{
            f"planet_pin_{i + 1}": [
                {
                    "name": "carrier_seat",
                    "type": "shaft",
                    "xyz_mm": [0.0, 0.0, CARRIER_H / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_PIN_R,
                    "depth_mm": CARRIER_H,
                },
                {
                    "name": "planet_journal",
                    "type": "shaft",
                    "xyz_mm": [0.0, 0.0, GEAR_MID_Z - PLANET_PIN_Z],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_PIN_R,
                    "depth_mm": GEAR_FACE_W,
                },
            ]
            for i in range(PLANET_COUNT)
        },
    },
    "relations": [
        {
            "name": "sun_press_fit",
            "mate_type": "press_fit",
            "base_part": "sun_gear",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "sun_seat",
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
            "name": "input_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "lower_input_bearing",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "input_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_input_bearing",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "output_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "lower_output_bearing",
            "base_port": "bore",
            "incoming_part": "output_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "output_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_output_bearing",
            "base_port": "bore",
            "incoming_part": "output_shaft",
            "incoming_port": "upper_journal",
        },
        *[
            {
                "name": f"pin_{i + 1}_carrier_press_fit",
                "mate_type": "press_fit",
                "base_part": "carrier",
                "base_port": f"pin_bore_{i + 1}",
                "incoming_part": f"planet_pin_{i + 1}",
                "incoming_port": "carrier_seat",
            }
            for i in range(PLANET_COUNT)
        ],
        *[
            {
                "name": f"planet_{i + 1}_journal",
                "mate_type": "journal_bearing",
                "base_part": f"planet_{i + 1}",
                "base_port": "pin_bore",
                "incoming_part": f"planet_pin_{i + 1}",
                "incoming_port": "planet_journal",
            }
            for i in range(PLANET_COUNT)
        ],
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
        *[
            {
                "name": f"planet_{i + 1}_carrier_hinge",
                "parent": "carrier",
                "child": f"planet_{i + 1}",
                "type": "hinge",
                "axis": [0.0, 0.0, 1.0],
                "pos_mm": [
                    PLANET_CENTERS[i][0],
                    PLANET_CENTERS[i][1],
                    GEAR_MID_Z - CARRIER_Z,
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
            "ring": "ring_gear",
            "carrier": "carrier",
            "planets": [
                {"gear": "planet_1", "pin": "planet_pin_1"},
                {"gear": "planet_2", "pin": "planet_pin_2"},
                {"gear": "planet_3", "pin": "planet_pin_3"},
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

    def base_align(shape):
        z_min = shape.bounding_box().min.Z
        return shape.moved(Location((0.0, 0.0, -z_min)))

    def annulus(outer_r, inner_r, height):
        with BuildPart() as bp:
            with BuildSketch():
                Circle(outer_r)
                Circle(inner_r, mode=Mode.SUBTRACT)
            extrude(amount=height)
        return bp.part

    def internal_ring_gear():
        tooth_pitch_angle = 2.0 * math.pi / Z_RING
        root_half_angle = tooth_pitch_angle * 0.23
        tip_half_angle = tooth_pitch_angle * 0.14

        with BuildPart() as bp:
            with BuildSketch():
                Circle(RING_OUTER_R)
                Circle(RING_ROOT_R, mode=Mode.SUBTRACT)

                for tooth_index in range(Z_RING):
                    angle = tooth_index * tooth_pitch_angle
                    points = [
                        (
                            RING_ROOT_R * math.cos(angle - root_half_angle),
                            RING_ROOT_R * math.sin(angle - root_half_angle),
                        ),
                        (
                            (RING_ROOT_R - 0.35 * M)
                            * math.cos(angle - root_half_angle),
                            (RING_ROOT_R - 0.35 * M)
                            * math.sin(angle - root_half_angle),
                        ),
                        (
                            RING_TIP_R * math.cos(angle - tip_half_angle),
                            RING_TIP_R * math.sin(angle - tip_half_angle),
                        ),
                        (
                            RING_TIP_R * math.cos(angle + tip_half_angle),
                            RING_TIP_R * math.sin(angle + tip_half_angle),
                        ),
                        (
                            (RING_ROOT_R - 0.35 * M)
                            * math.cos(angle + root_half_angle),
                            (RING_ROOT_R - 0.35 * M)
                            * math.sin(angle + root_half_angle),
                        ),
                        (
                            RING_ROOT_R * math.cos(angle + root_half_angle),
                            RING_ROOT_R * math.sin(angle + root_half_angle),
                        ),
                    ]
                    Polygon(*points, mode=Mode.ADD)

            extrude(amount=GEAR_FACE_W)

        return bp.part

    def carrier_plate():
        with BuildPart() as bp:
            with BuildSketch():
                Circle(CARRIER_R)
                Circle(CARRIER_OUTPUT_BORE_R, mode=Mode.SUBTRACT)
                for px, py in PLANET_CENTERS:
                    with b3d.Locations((px, py)):
                        Circle(CARRIER_PIN_BORE_R, mode=Mode.SUBTRACT)
            extrude(amount=CARRIER_H)
        return bp.part

    def bridge_plate():
        return annulus(BRIDGE_R, BEARING_SEAT_R, BRIDGE_H)

    def crank_arm():
        end_r = CRANK_ARM_HALF_W
        with BuildPart() as bp:
            with BuildSketch():
                Polygon(
                    (0.0, -CRANK_ARM_HALF_W),
                    (CRANK_THROW, -CRANK_ARM_HALF_W),
                    (CRANK_THROW, CRANK_ARM_HALF_W),
                    (0.0, CRANK_ARM_HALF_W),
                )
                Circle(end_r)
                with b3d.Locations((CRANK_THROW, 0.0)):
                    Circle(end_r)
                Circle(INPUT_SHAFT_R - PRESS_INTERFERENCE, mode=Mode.SUBTRACT)
            extrude(amount=CRANK_ARM_H)
        return bp.part

    base = Box(
        120.0,
        120.0,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, BASE_Z)))
    a.add(base, "baseplate|dof=fixed")

    output_pedestal = annulus(
        12.0, BEARING_SEAT_R, 2.0 * OUTPUT_BEARING_H
    ).moved(Location((0.0, 0.0, OUTPUT_BEARING_Z_1)))
    a.add(output_pedestal, "output_pedestal|dof=fixed|mount=baseplate")

    lower_output_bearing = annulus(
        BEARING_OUTER_R, OUTPUT_BEARING_BORE_R, OUTPUT_BEARING_H
    ).moved(Location((0.0, 0.0, OUTPUT_BEARING_Z_1)))
    a.add(
        lower_output_bearing,
        "lower_output_bearing|dof=fixed|mount=output_pedestal",
    )

    upper_output_bearing = annulus(
        BEARING_OUTER_R, OUTPUT_BEARING_BORE_R, OUTPUT_BEARING_H
    ).moved(Location((0.0, 0.0, OUTPUT_BEARING_Z_2)))
    a.add(
        upper_output_bearing,
        "upper_output_bearing|dof=fixed|mount=output_pedestal",
    )

    ring_post_names = []
    for i, angle_deg in enumerate(PLANET_ANGLES_DEG):
        angle = math.radians(angle_deg)
        px = RING_POST_R * math.cos(angle)
        py = RING_POST_R * math.sin(angle)
        name = f"ring_post_{i + 1}"
        ring_post_names.append(name)

        post = Cylinder(
            RING_POST_RADIUS,
            RING_POST_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, RING_POST_Z)))
        a.add(post, f"{name}|dof=fixed|mount=baseplate")

    lower_stanchion_names = []
    mid_stanchion_names = []

    for i, angle_deg in enumerate(PLANET_ANGLES_DEG):
        angle = math.radians(angle_deg + 60.0)
        sx = STANCHION_ORBIT_R * math.cos(angle)
        sy = STANCHION_ORBIT_R * math.sin(angle)

        lower_name = f"lower_stanchion_{i + 1}"
        lower_stanchion_names.append(lower_name)
        lower_stanchion = Cylinder(
            STANCHION_R,
            LOWER_STANCHION_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((sx, sy, LOWER_STANCHION_Z)))
        a.add(lower_stanchion, f"{lower_name}|dof=fixed|mount=baseplate")

    lower_bridge = bridge_plate().moved(
        Location((0.0, 0.0, LOWER_BRIDGE_Z))
    )
    a.add(
        lower_bridge,
        "lower_bridge|dof=fixed|mount=" + ",".join(lower_stanchion_names),
    )

    for i, angle_deg in enumerate(PLANET_ANGLES_DEG):
        angle = math.radians(angle_deg + 60.0)
        sx = STANCHION_ORBIT_R * math.cos(angle)
        sy = STANCHION_ORBIT_R * math.sin(angle)

        mid_name = f"mid_stanchion_{i + 1}"
        mid_stanchion_names.append(mid_name)
        mid_stanchion = Cylinder(
            STANCHION_R,
            MID_STANCHION_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((sx, sy, MID_STANCHION_Z)))
        a.add(mid_stanchion, f"{mid_name}|dof=fixed|mount=lower_bridge")

    upper_bridge = bridge_plate().moved(
        Location((0.0, 0.0, UPPER_BRIDGE_Z))
    )
    a.add(
        upper_bridge,
        "upper_bridge|dof=fixed|mount=" + ",".join(mid_stanchion_names),
    )

    lower_input_bearing = annulus(
        BEARING_OUTER_R, INPUT_BEARING_BORE_R, INPUT_BEARING_H
    ).moved(Location((0.0, 0.0, LOWER_INPUT_BEARING_Z)))
    a.add(
        lower_input_bearing,
        "lower_input_bearing|dof=fixed|mount=lower_bridge",
    )

    upper_input_bearing = annulus(
        BEARING_OUTER_R, INPUT_BEARING_BORE_R, INPUT_BEARING_H
    ).moved(Location((0.0, 0.0, UPPER_INPUT_BEARING_Z)))
    a.add(
        upper_input_bearing,
        "upper_input_bearing|dof=fixed|mount=upper_bridge",
    )

    ring = internal_ring_gear().moved(Location((0.0, 0.0, RING_Z)))
    a.add(
        ring,
        "ring_gear|dof=fixed|mount=" + ",".join(ring_post_names),
    )

    output_shaft = Cylinder(
        OUTPUT_SHAFT_R,
        OUTPUT_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, OUTPUT_SHAFT_Z)))
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=z|"
        "mount=lower_output_bearing,upper_output_bearing",
    )

    carrier = carrier_plate().moved(Location((0.0, 0.0, CARRIER_Z)))
    a.add(
        carrier,
        "carrier|dof=spin|spin_axis=z|mount=output_shaft",
    )

    for i, (px, py) in enumerate(PLANET_CENTERS):
        pin_name = f"planet_pin_{i + 1}"
        pin = Cylinder(
            PLANET_PIN_R,
            PLANET_PIN_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, PLANET_PIN_Z)))
        a.add(pin, f"{pin_name}|dof=fixed|mount=carrier")

    input_main = Cylinder(
        INPUT_SHAFT_R,
        INPUT_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    input_flange = Cylinder(
        INPUT_THRUST_FLANGE_R,
        INPUT_THRUST_FLANGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                0.0,
                INPUT_THRUST_FLANGE_Z - INPUT_SHAFT_Z,
            )
        )
    )
    input_shaft_local = input_main + input_flange
    input_shaft = input_shaft_local.moved(
        Location((0.0, 0.0, INPUT_SHAFT_Z))
    )
    a.add(
        input_shaft,
        "input_shaft|dof=spin|driver=True|spin_axis=z|"
        "mount=lower_input_bearing,upper_input_bearing",
    )

    sun = base_align(
        make_gear(M, Z_SUN, GEAR_FACE_W, 2.0 * SUN_BORE_R)
    ).moved(Location((0.0, 0.0, GEAR_Z)))
    a.add(
        sun,
        "sun_gear|dof=spin|spin_axis=z|mount=input_shaft",
    )

    for i, ((px, py), phase_deg) in enumerate(
        zip(PLANET_CENTERS, PLANET_PHASE_DEG)
    ):
        planet_name = f"planet_{i + 1}"
        planet = base_align(
            make_gear(M, Z_PLANET, GEAR_FACE_W, 2.0 * PLANET_BORE_R)
        ).moved(
            Location(
                (px, py, GEAR_Z),
                (0.0, 0.0, phase_deg),
            )
        )
        a.add(
            planet,
            f"{planet_name}|dof=spin|spin_axis=z|mount=planet_pin_{i + 1}",
        )

    crank = crank_arm().moved(Location((0.0, 0.0, CRANK_ARM_Z)))
    a.add(crank, "crank_arm|dof=fixed|mount=input_shaft")

    grip = Cylinder(
        GRIP_R,
        GRIP_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_THROW, 0.0, GRIP_Z)))
    a.add(grip, "hand_grip|dof=fixed|mount=crank_arm")

    return a.build()