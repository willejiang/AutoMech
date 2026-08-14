import math


# ---------------------------------------------------------------------------
# Drivetrain arithmetic -- all gear locations derive from these values.
# ---------------------------------------------------------------------------
M = 1.5
Z_SUN = 20
Z_PLANET = 20
Z_RING = Z_SUN + 2 * Z_PLANET
PLANET_COUNT = 4

assert Z_RING == 60
assert (Z_RING + Z_SUN) % PLANET_COUNT == 0


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
PLANET_ORBIT_R = SUN_PLANET_CD

FIXED_RING_CARRIER_RATIO = 1.0 + Z_RING / Z_SUN

PLANET_ANGLES_DEG = [
    i * 360.0 / PLANET_COUNT for i in range(PLANET_COUNT)
]
PLANET_XY = [
    (
        PLANET_ORBIT_R * math.cos(math.radians(a)),
        PLANET_ORBIT_R * math.sin(math.radians(a)),
    )
    for a in PLANET_ANGLES_DEG
]

# Equal tooth counts and the valid four-planet assembly condition allow the
# same planet roll phase at every 90-degree station.
PLANET_PHASE_DEG = 180.0 / Z_PLANET
RING_PHASE_DEG = 6.0

# ---------------------------------------------------------------------------
# Axial stack and fits, in millimetres.
# ---------------------------------------------------------------------------
BASE_R = 65.0
BASE_H = 4.0

SHAFT_R = 4.0
SHAFT_Z = 4.5
SHAFT_H = 43.5

RUNNING_CLEARANCE = 0.05
PRESS_INTERFERENCE = 0.005

INPUT_BEARING_INNER_R = SHAFT_R + RUNNING_CLEARANCE
INPUT_BEARING_OUTER_R = 8.5
LOWER_BEARING_Z = SHAFT_Z
LOWER_BEARING_H = 4.0

CARRIER_BORE_R = SHAFT_R + RUNNING_CLEARANCE
CARRIER_SLEEVE_R = 9.0
CARRIER_Z = 9.0
CARRIER_SLEEVE_H = 12.0
CARRIER_FLANGE_R = 16.0
CARRIER_FLANGE_LOCAL_Z = 7.0
CARRIER_FLANGE_H = 2.0
CARRIER_PLATE_R = PLANET_ORBIT_R + 8.0
CARRIER_PLATE_LOCAL_Z = 9.0
CARRIER_PLATE_H = 3.0
CARRIER_TOP_Z = CARRIER_Z + CARRIER_PLATE_LOCAL_Z + CARRIER_PLATE_H

OUTPUT_BEARING_INNER_R = CARRIER_SLEEVE_R + RUNNING_CLEARANCE
OUTPUT_BEARING_OUTER_R = 14.0
OUTPUT_BEARING_Z = 10.0
OUTPUT_BEARING_H = 6.0

PEDESTAL_INNER_R = OUTPUT_BEARING_INNER_R
PEDESTAL_OUTER_R = 18.0
PEDESTAL_Z = BASE_H
PEDESTAL_H = OUTPUT_BEARING_Z - PEDESTAL_Z

GEAR_Z = 24.0
GEAR_FACE_W = 8.0
GEAR_MID_Z = GEAR_FACE_W / 2.0

PLANET_PIN_R = 3.0
PLANET_PIN_BORE_R = PLANET_PIN_R + RUNNING_CLEARANCE
PLANET_SHOULDER_R = 5.0
PLANET_PIN_Z = CARRIER_TOP_Z
PLANET_SHOULDER_H = GEAR_Z - PLANET_PIN_Z
PLANET_JOURNAL_LOCAL_Z = PLANET_SHOULDER_H - 0.1
PLANET_JOURNAL_H = GEAR_FACE_W + 4.1

RING_TIP_R = RING_PITCH_R - M
RING_ROOT_R = RING_PITCH_R + 1.25 * M
RING_OUTER_R = RING_ROOT_R + 3.75 * M

RING_SUPPORT_INNER_R = 48.0
RING_SUPPORT_OUTER_R = 55.0
RING_SUPPORT_Z = BASE_H
RING_SUPPORT_H = GEAR_Z - RING_SUPPORT_Z

TOP_PLATE_Z = 36.0
TOP_PLATE_H = 6.0
UPPER_BEARING_Z = TOP_PLATE_Z
UPPER_BEARING_H = TOP_PLATE_H
TOP_BEARING_POCKET_R = INPUT_BEARING_OUTER_R + RUNNING_CLEARANCE

POST_ORBIT_R = 60.0
POST_R = 3.0
POST_Z = BASE_H
POST_H = TOP_PLATE_Z - POST_Z
POST_ANGLES_DEG = [0.0, 120.0, 240.0]
POST_XY = [
    (
        POST_ORBIT_R * math.cos(math.radians(a)),
        POST_ORBIT_R * math.sin(math.radians(a)),
    )
    for a in POST_ANGLES_DEG
]

CRANK_Z = TOP_PLATE_Z + TOP_PLATE_H
CRANK_H = 3.0
CRANK_HUB_R = 8.0
CRANK_THROW = 32.0
CRANK_ARM_W = 8.0
CRANK_BORE_R = SHAFT_R - PRESS_INTERFERENCE

GRIP_R = 4.0
GRIP_Z = CRANK_Z + CRANK_H
GRIP_H = 16.0


# ---------------------------------------------------------------------------
# Canonical mechanism semantics. Port coordinates are part-local.
# ---------------------------------------------------------------------------
MECHANISM = {
    "name": "hand_driven_fixed_ring_four_planet_reducer",
    "output_link": "output_carrier",
    "watch_links": [
        "input_shaft",
        "sun_gear",
        "planet_gear_1",
        "planet_gear_2",
        "planet_gear_3",
        "planet_gear_4",
        "output_carrier",
    ],
    "ports_by_link": {
        "input_shaft": [
            {
                "name": "shaft_outer",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": SHAFT_H,
            },
            {
                "name": "lower_journal",
                "type": "cylindrical",
                "xyz_mm": [
                    0.0,
                    0.0,
                    LOWER_BEARING_Z + LOWER_BEARING_H / 2.0 - SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": LOWER_BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "cylindrical",
                "xyz_mm": [
                    0.0,
                    0.0,
                    UPPER_BEARING_Z + UPPER_BEARING_H / 2.0 - SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": UPPER_BEARING_H,
            },
            {
                "name": "sun_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, GEAR_Z + GEAR_MID_Z - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "crank_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, CRANK_Z + CRANK_H / 2.0 - SHAFT_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANK_H,
            },
        ],
        "lower_input_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, LOWER_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * INPUT_BEARING_INNER_R,
                "depth_mm": LOWER_BEARING_H,
            }
        ],
        "upper_input_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, UPPER_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * INPUT_BEARING_INNER_R,
                "depth_mm": UPPER_BEARING_H,
            }
        ],
        "sun_gear": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, GEAR_MID_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * (SHAFT_R - PRESS_INTERFERENCE),
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "sun_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_MID_Z],
                "axis": [0, 0, 1],
                "pitch_radius_mm": SUN_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "fixed_ring_gear": [
            {
                "name": "ring_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_MID_Z],
                "axis": [0, 0, 1],
                "pitch_radius_mm": RING_PITCH_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "bottom_face",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0, 0, 1],
                "normal_sign": -1,
            },
        ],
        "ring_support": [
            {
                "name": "top_face",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, RING_SUPPORT_H],
                "axis": [0, 0, 1],
                "normal_sign": 1,
            }
        ],
        "output_carrier": [
            {
                "name": "sleeve_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    OUTPUT_BEARING_Z + OUTPUT_BEARING_H / 2.0 - CARRIER_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CARRIER_SLEEVE_R,
                "depth_mm": OUTPUT_BEARING_H,
            },
            {
                "name": "central_running_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CARRIER_SLEEVE_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CARRIER_BORE_R,
                "depth_mm": CARRIER_SLEEVE_H,
            },
            {
                "name": "output_flange",
                "type": "flat_face",
                "xyz_mm": [
                    0.0,
                    0.0,
                    CARRIER_FLANGE_LOCAL_Z + CARRIER_FLANGE_H / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CARRIER_FLANGE_R,
            },
        ],
        "output_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, OUTPUT_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * OUTPUT_BEARING_INNER_R,
                "depth_mm": OUTPUT_BEARING_H,
            }
        ],
        "crank_arm": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CRANK_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CRANK_BORE_R,
                "depth_mm": CRANK_H,
            },
            {
                "name": "grip_face",
                "type": "flat_face",
                "xyz_mm": [CRANK_THROW, 0.0, CRANK_H],
                "axis": [0, 0, 1],
                "normal_sign": 1,
            },
        ],
        "hand_grip": [
            {
                "name": "bottom_face",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0, 0, 1],
                "normal_sign": -1,
            }
        ],
        **{
            f"planet_pin_{i + 1}": [
                {
                    "name": "journal",
                    "type": "shaft",
                    "xyz_mm": [
                        0.0,
                        0.0,
                        PLANET_JOURNAL_LOCAL_Z + PLANET_JOURNAL_H / 2.0,
                    ],
                    "axis": [0, 0, 1],
                    "diameter_mm": 2.0 * PLANET_PIN_R,
                    "depth_mm": PLANET_JOURNAL_H,
                }
            ]
            for i in range(PLANET_COUNT)
        },
        **{
            f"planet_gear_{i + 1}": [
                {
                    "name": "bore",
                    "type": "bore",
                    "xyz_mm": [0.0, 0.0, GEAR_MID_Z],
                    "axis": [0, 0, 1],
                    "diameter_mm": 2.0 * PLANET_PIN_BORE_R,
                    "depth_mm": GEAR_FACE_W,
                },
                {
                    "name": "planet_mesh",
                    "type": "gear_mesh",
                    "xyz_mm": [0.0, 0.0, GEAR_MID_Z],
                    "axis": [0, 0, 1],
                    "pitch_radius_mm": PLANET_PITCH_R,
                    "depth_mm": GEAR_FACE_W,
                },
            ]
            for i in range(PLANET_COUNT)
        },
    },
    "relations": [
        {
            "name": "lower_input_journal",
            "mate_type": "journal_bearing",
            "base_part": "lower_input_bearing",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "upper_input_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_input_bearing",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "carrier_output_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_bearing",
            "base_port": "bore",
            "incoming_part": "output_carrier",
            "incoming_port": "sleeve_journal",
        },
        {
            "name": "sun_on_input_shaft",
            "mate_type": "press_fit",
            "base_part": "sun_gear",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "sun_seat",
        },
        {
            "name": "crank_on_input_shaft",
            "mate_type": "press_fit",
            "base_part": "crank_arm",
            "base_port": "hub_bore",
            "incoming_part": "input_shaft",
            "incoming_port": "crank_seat",
        },
        {
            "name": "grip_on_crank",
            "mate_type": "face_to_face",
            "base_part": "crank_arm",
            "base_port": "grip_face",
            "incoming_part": "hand_grip",
            "incoming_port": "bottom_face",
        },
        {
            "name": "fixed_ring_on_support",
            "mate_type": "face_to_face",
            "base_part": "ring_support",
            "base_port": "top_face",
            "incoming_part": "fixed_ring_gear",
            "incoming_port": "bottom_face",
        },
        *[
            {
                "name": f"planet_{i + 1}_journal",
                "mate_type": "journal_bearing",
                "base_part": f"planet_gear_{i + 1}",
                "base_port": "bore",
                "incoming_part": f"planet_pin_{i + 1}",
                "incoming_port": "journal",
            }
            for i in range(PLANET_COUNT)
        ],
    ],
    "motion_joints": [
        {
            "name": "input_rotation",
            "parent": "",
            "child": "input_shaft",
            "type": "hinge",
            "axis": [0, 0, 1],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "carrier_output_rotation",
            "parent": "",
            "child": "output_carrier",
            "type": "hinge",
            "axis": [0, 0, 1],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        *[
            {
                "name": f"planet_{i + 1}_spin_on_carrier",
                "parent": "output_carrier",
                "child": f"planet_gear_{i + 1}",
                "type": "hinge",
                "axis": [0, 0, 1],
                "pos_mm": [0.0, 0.0, 0.0],
            }
            for i in range(PLANET_COUNT)
        ],
    ],
    "transmissions": [
        *[
            {
                "name": f"sun_to_planet_{i + 1}",
                "type": "gear_external",
                "driving_link": "sun_gear",
                "driven_link": f"planet_gear_{i + 1}",
                "ratio": 0,
            }
            for i in range(PLANET_COUNT)
        ],
        *[
            {
                "name": f"planet_{i + 1}_to_fixed_ring",
                "type": "gear_internal",
                "driving_link": f"planet_gear_{i + 1}",
                "driven_link": "fixed_ring_gear",
                "ratio": 0,
            }
            for i in range(PLANET_COUNT)
        ],
    ],
    "planetary_stages": [
        {
            "name": "fixed_ring_stage",
            "sun": "sun_gear",
            "ring": "fixed_ring_gear",
            "carrier": "output_carrier",
            "planets": [
                {"gear": f"planet_gear_{i + 1}", "pin": f"planet_pin_{i + 1}"}
                for i in range(PLANET_COUNT)
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
    a = AssemblyHelper("hand_driven_fixed_ring_four_planet_reducer")

    def annulus(outer_r, inner_r, height):
        with BuildPart() as part:
            with BuildSketch():
                Circle(outer_r)
                Circle(inner_r, mode=Mode.SUBTRACT)
            extrude(amount=height)
        return part.part

    def make_internal_ring():
        tooth_pitch = 2.0 * math.pi / Z_RING
        root_half_angle = 0.36 * tooth_pitch
        tip_half_angle = 0.18 * tooth_pitch

        def polar(radius, angle):
            return (
                radius * math.cos(angle),
                radius * math.sin(angle),
            )

        with BuildPart() as ring_part:
            with BuildSketch():
                Circle(RING_OUTER_R)
                Circle(RING_ROOT_R, mode=Mode.SUBTRACT)

                for tooth_i in range(Z_RING):
                    angle = tooth_i * tooth_pitch
                    tooth_points = [
                        polar(RING_ROOT_R + 0.12, angle - root_half_angle),
                        polar(RING_ROOT_R + 0.12, angle + root_half_angle),
                        polar(RING_TIP_R, angle + tip_half_angle),
                        polar(RING_TIP_R, angle - tip_half_angle),
                    ]
                    Polygon(*tooth_points, mode=Mode.ADD)

            extrude(amount=GEAR_FACE_W)

        return ring_part.part

    def make_carrier():
        sleeve = annulus(
            CARRIER_SLEEVE_R,
            CARRIER_BORE_R,
            CARRIER_SLEEVE_H,
        )
        flange = annulus(
            CARRIER_FLANGE_R,
            CARRIER_BORE_R,
            CARRIER_FLANGE_H,
        ).moved(Location((0, 0, CARRIER_FLANGE_LOCAL_Z)))
        planet_plate = annulus(
            CARRIER_PLATE_R,
            CARRIER_BORE_R,
            CARRIER_PLATE_H,
        ).moved(Location((0, 0, CARRIER_PLATE_LOCAL_Z)))
        return sleeve + flange + planet_plate

    def make_planet_pin():
        shoulder = Cylinder(
            PLANET_SHOULDER_R,
            PLANET_SHOULDER_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        journal = Cylinder(
            PLANET_PIN_R,
            PLANET_JOURNAL_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0, 0, PLANET_JOURNAL_LOCAL_Z)))
        return shoulder + journal

    def make_crank_arm():
        hub = Cylinder(
            CRANK_HUB_R,
            CRANK_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        arm = Box(
            CRANK_THROW,
            CRANK_ARM_W,
            CRANK_H,
            align=(Align.MIN, Align.CENTER, Align.MIN),
        )
        bore_tool = Cylinder(
            CRANK_BORE_R,
            CRANK_H + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0, 0, -0.5)))
        return (hub + arm) - bore_tool

    # Grounded base.
    base = Cylinder(
        BASE_R,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(base, "baseplate|dof=fixed")

    # Input shaft axial thrust support.
    input_thrust_pad = Cylinder(
        SHAFT_R + 0.5,
        SHAFT_Z - BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, BASE_H)))
    a.add(input_thrust_pad, "input_thrust_pad|dof=fixed|mount=baseplate")

    lower_input_bearing = annulus(
        INPUT_BEARING_OUTER_R,
        INPUT_BEARING_INNER_R,
        LOWER_BEARING_H,
    ).moved(Location((0, 0, LOWER_BEARING_Z)))
    a.add(
        lower_input_bearing,
        "lower_input_bearing|dof=fixed|mount=baseplate",
    )

    bearing_pedestal = annulus(
        PEDESTAL_OUTER_R,
        PEDESTAL_INNER_R,
        PEDESTAL_H,
    ).moved(Location((0, 0, PEDESTAL_Z)))
    a.add(
        bearing_pedestal,
        "output_bearing_pedestal|dof=fixed|mount=baseplate",
    )

    output_bearing = annulus(
        OUTPUT_BEARING_OUTER_R,
        OUTPUT_BEARING_INNER_R,
        OUTPUT_BEARING_H,
    ).moved(Location((0, 0, OUTPUT_BEARING_Z)))
    a.add(
        output_bearing,
        "output_bearing|dof=fixed|mount=output_bearing_pedestal",
    )

    # Independently rotating carrier/output sleeve.
    carrier = make_carrier().moved(Location((0, 0, CARRIER_Z)))
    a.add(
        carrier,
        "output_carrier|dof=spin|spin_axis=z|mount=output_bearing",
    )

    # Fixed annular pedestal directly beneath the internal ring.
    ring_support = annulus(
        RING_SUPPORT_OUTER_R,
        RING_SUPPORT_INNER_R,
        RING_SUPPORT_H,
    ).moved(Location((0, 0, RING_SUPPORT_Z)))
    a.add(ring_support, "ring_support|dof=fixed|mount=baseplate")

    # Central hand-driven shaft.
    input_shaft = Cylinder(
        SHAFT_R,
        SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, SHAFT_Z)))
    a.add(
        input_shaft,
        (
            "input_shaft|dof=spin|spin_axis=z|driver=True|"
            "mount=lower_input_bearing,upper_input_bearing,input_thrust_pad"
        ),
    )

    # Sun at the common planetary gear axial station.
    sun = make_gear(
        M,
        Z_SUN,
        GEAR_FACE_W,
        2.0 * (SHAFT_R - PRESS_INTERFERENCE),
    ).moved(Location((0, 0, GEAR_Z)))
    a.add(
        sun,
        "sun_gear|dof=spin|spin_axis=z|mount=input_shaft",
    )

    # Four stepped pins are fixed to the rotating carrier. Their shoulders end
    # exactly at the planet lower faces, providing physical gravity support.
    for i, (px, py) in enumerate(PLANET_XY, start=1):
        pin = make_planet_pin().moved(
            Location((px, py, PLANET_PIN_Z))
        )
        a.add(
            pin,
            f"planet_pin_{i}|dof=fixed|mount=output_carrier",
        )

    # Four planets, each exactly one computed center distance from the sun and
    # the fixed internal ring pitch circle.
    for i, (px, py) in enumerate(PLANET_XY, start=1):
        planet = make_gear(
            M,
            Z_PLANET,
            GEAR_FACE_W,
            2.0 * PLANET_PIN_BORE_R,
        ).moved(
            Location(
                (px, py, GEAR_Z),
                (0.0, 0.0, PLANET_PHASE_DEG),
            )
        )
        a.add(
            planet,
            (
                f"planet_gear_{i}|dof=spin|spin_axis=z|"
                f"mount=planet_pin_{i}"
            ),
        )

    fixed_ring = make_internal_ring().moved(
        Location(
            (0, 0, GEAR_Z),
            (0.0, 0.0, RING_PHASE_DEG),
        )
    )
    a.add(
        fixed_ring,
        "fixed_ring_gear|dof=fixed|mount=ring_support",
    )

    # Three structural posts and a bridge support the upper input bearing.
    for i, (post_x, post_y) in enumerate(POST_XY, start=1):
        post = Cylinder(
            POST_R,
            POST_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((post_x, post_y, POST_Z)))
        a.add(
            post,
            f"top_post_{i}|dof=fixed|mount=baseplate",
        )

    top_plate_blank = Cylinder(
        BASE_R,
        TOP_PLATE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    top_plate_bore = Cylinder(
        TOP_BEARING_POCKET_R,
        TOP_PLATE_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))
    top_plate = (top_plate_blank - top_plate_bore).moved(
        Location((0, 0, TOP_PLATE_Z))
    )
    a.add(
        top_plate,
        "top_bridge|dof=fixed|mount=top_post_1,top_post_2,top_post_3",
    )

    upper_input_bearing = annulus(
        INPUT_BEARING_OUTER_R,
        INPUT_BEARING_INNER_R,
        UPPER_BEARING_H,
    ).moved(Location((0, 0, UPPER_BEARING_Z)))
    a.add(
        upper_input_bearing,
        "upper_input_bearing|dof=fixed|mount=top_bridge",
    )

    # Hand input hardware is press-fit/welded to the driven shaft rather than
    # receiving an independent rotational DOF.
    crank_arm = make_crank_arm().moved(Location((0, 0, CRANK_Z)))
    a.add(
        crank_arm,
        "crank_arm|dof=fixed|mount=input_shaft",
    )

    hand_grip = Cylinder(
        GRIP_R,
        GRIP_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_THROW, 0, GRIP_Z)))
    a.add(
        hand_grip,
        "hand_grip|dof=fixed|mount=crank_arm",
    )

    return a.build()