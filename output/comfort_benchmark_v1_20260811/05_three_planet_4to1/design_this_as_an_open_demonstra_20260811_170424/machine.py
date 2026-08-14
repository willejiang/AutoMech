import math


# ---------------------------------------------------------------------------
# Drivetrain arithmetic -- all gear geometry and locations derive from this.
# ---------------------------------------------------------------------------

M = 1.0
Z_SUN = 18
Z_PLANET = 18
Z_RING = 54
PLANET_COUNT = 3

assert Z_RING == Z_SUN + 2 * Z_PLANET
assert Z_RING == 3 * Z_SUN
assert (Z_SUN + Z_RING) % PLANET_COUNT == 0

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

PLANET_ORBIT_R = SUN_PLANET_CD
PLANET_ANGLES_DEG = tuple(i * 360.0 / PLANET_COUNT for i in range(PLANET_COUNT))
PLANET_CENTERS = tuple(
    (
        PLANET_ORBIT_R * math.cos(math.radians(a)),
        PLANET_ORBIT_R * math.sin(math.radians(a)),
    )
    for a in PLANET_ANGLES_DEG
)

FIXED_RING_REDUCTION = 1.0 + Z_RING / Z_SUN
assert FIXED_RING_REDUCTION == 4.0

# ---------------------------------------------------------------------------
# Axial stack and fits.
# All extrusion-built parts use their lower face as local z=0.
# ---------------------------------------------------------------------------

BASE_L = 78.0
BASE_W = 78.0
BASE_H = 5.0

INPUT_SHAFT_R = 3.0
INPUT_SHAFT_Z = BASE_H
INPUT_SHAFT_TOP_Z = 31.0
INPUT_SHAFT_H = INPUT_SHAFT_TOP_Z - INPUT_SHAFT_Z

SHAFT_RUNNING_BORE_R = INPUT_SHAFT_R + 0.05
SHAFT_PRESS_BORE_R = INPUT_SHAFT_R - 0.005

INPUT_BEARING_OUTER_R = 5.0
LOWER_BEARING_Z = BASE_H
LOWER_BEARING_H = 3.0
UPPER_BEARING_Z = LOWER_BEARING_Z + LOWER_BEARING_H
UPPER_BEARING_H = 3.0

CARRIER_BEARING_INNER_R = INPUT_BEARING_OUTER_R + 0.10
CARRIER_BEARING_OUTER_R = 9.0
CARRIER_BEARING_Z = UPPER_BEARING_Z
CARRIER_BEARING_H = 4.0

CARRIER_Z = CARRIER_BEARING_Z + CARRIER_BEARING_H
CARRIER_H = 3.0
CARRIER_TOP_Z = CARRIER_Z + CARRIER_H
CARRIER_HUB_R = 9.5
CARRIER_BORE_R = CARRIER_BEARING_INNER_R
CARRIER_ARM_HALF_W = 3.0
CARRIER_PIN_BOSS_R = 5.0

GEAR_CLEARANCE_Z = 0.20
GEAR_Z = CARRIER_TOP_Z + GEAR_CLEARANCE_Z
GEAR_FACE_W = 6.0
GEAR_TOP_Z = GEAR_Z + GEAR_FACE_W

PLANET_PIN_R = 2.5
PLANET_PIN_PRESS_R = PLANET_PIN_R - 0.005
PLANET_BORE_R = PLANET_PIN_R + 0.05
PLANET_PIN_Z = CARRIER_Z + 0.75
PLANET_PIN_TOP_Z = GEAR_TOP_Z + 1.0
PLANET_PIN_H = PLANET_PIN_TOP_Z - PLANET_PIN_Z

RING_TIP_R = RING_PITCH_R - M
RING_ROOT_R = RING_PITCH_R + 1.25 * M
RING_OUTER_R = RING_ROOT_R + 4.0

RING_SUPPORT_R = (RING_ROOT_R + RING_OUTER_R) / 2.0
RING_POST_R = 1.6
RING_POST_Z = BASE_H
RING_POST_H = GEAR_Z - RING_POST_Z
RING_POST_ANGLES_DEG = (60.0, 180.0, 300.0)
RING_POST_CENTERS = tuple(
    (
        RING_SUPPORT_R * math.cos(math.radians(a)),
        RING_SUPPORT_R * math.sin(math.radians(a)),
    )
    for a in RING_POST_ANGLES_DEG
)

CRANK_Z = GEAR_TOP_Z + 2.0
CRANK_H = 3.0
CRANK_HUB_R = 5.0
CRANK_THROW = 15.0
CRANK_ARM_HALF_W = 2.5
CRANK_GRIP_R = 3.0
CRANK_GRIP_Z = CRANK_Z + CRANK_H
CRANK_GRIP_H = 10.0


def _axis_port(name, port_type, x, y, z, diameter=None, pitch_radius=None, depth=None):
    p = {
        "name": name,
        "type": port_type,
        "xyz_mm": [x, y, z],
        "axis": [0.0, 0.0, 1.0],
    }
    if diameter is not None:
        p["diameter_mm"] = diameter
    if pitch_radius is not None:
        p["pitch_radius_mm"] = pitch_radius
    if depth is not None:
        p["depth_mm"] = depth
    return p


_ports = {
    "base": [
        {
            "name": "top",
            "type": "flat_face",
            "xyz_mm": [0.0, 0.0, BASE_H],
            "axis": [0.0, 0.0, 1.0],
            "normal_sign": 1,
        }
    ],
    "lower_input_bearing": [
        _axis_port(
            "journal",
            "bore",
            0.0,
            0.0,
            LOWER_BEARING_Z + LOWER_BEARING_H / 2.0,
            2.0 * SHAFT_RUNNING_BORE_R,
            depth=LOWER_BEARING_H,
        )
    ],
    "upper_input_bearing": [
        _axis_port(
            "journal",
            "bore",
            0.0,
            0.0,
            UPPER_BEARING_Z + UPPER_BEARING_H / 2.0,
            2.0 * SHAFT_RUNNING_BORE_R,
            depth=UPPER_BEARING_H,
        )
    ],
    "carrier_bearing": [
        _axis_port(
            "carrier_journal",
            "cylindrical",
            0.0,
            0.0,
            CARRIER_BEARING_Z + CARRIER_BEARING_H / 2.0,
            2.0 * CARRIER_BEARING_OUTER_R,
            depth=CARRIER_BEARING_H,
        )
    ],
    "input_shaft": [
        _axis_port(
            "shaft_axis",
            "shaft",
            0.0,
            0.0,
            INPUT_SHAFT_Z + INPUT_SHAFT_H / 2.0,
            2.0 * INPUT_SHAFT_R,
            depth=INPUT_SHAFT_H,
        ),
        _axis_port(
            "sun_seat",
            "shaft",
            0.0,
            0.0,
            GEAR_Z + GEAR_FACE_W / 2.0,
            2.0 * INPUT_SHAFT_R,
            depth=GEAR_FACE_W,
        ),
        _axis_port(
            "crank_seat",
            "shaft",
            0.0,
            0.0,
            CRANK_Z + CRANK_H / 2.0,
            2.0 * INPUT_SHAFT_R,
            depth=CRANK_H,
        ),
    ],
    "sun_gear": [
        _axis_port(
            "shaft_bore",
            "bore",
            0.0,
            0.0,
            GEAR_Z + GEAR_FACE_W / 2.0,
            2.0 * SHAFT_PRESS_BORE_R,
            depth=GEAR_FACE_W,
        ),
        _axis_port(
            "sun_mesh",
            "gear_mesh",
            0.0,
            0.0,
            GEAR_Z + GEAR_FACE_W / 2.0,
            pitch_radius=SUN_PITCH_R,
            depth=GEAR_FACE_W,
        ),
    ],
    "carrier": [
        _axis_port(
            "output_axis",
            "bore",
            0.0,
            0.0,
            CARRIER_Z + CARRIER_H / 2.0,
            2.0 * CARRIER_BORE_R,
            depth=CARRIER_H,
        )
    ],
    "fixed_ring": [
        _axis_port(
            "internal_mesh",
            "gear_mesh",
            0.0,
            0.0,
            GEAR_Z + GEAR_FACE_W / 2.0,
            pitch_radius=RING_PITCH_R,
            depth=GEAR_FACE_W,
        )
    ],
    "input_crank": [
        _axis_port(
            "hub_bore",
            "bore",
            0.0,
            0.0,
            CRANK_Z + CRANK_H / 2.0,
            2.0 * SHAFT_PRESS_BORE_R,
            depth=CRANK_H,
        )
    ],
    "crank_grip": [
        _axis_port(
            "grip_axis",
            "cylindrical",
            CRANK_THROW,
            0.0,
            CRANK_GRIP_Z + CRANK_GRIP_H / 2.0,
            2.0 * CRANK_GRIP_R,
            depth=CRANK_GRIP_H,
        )
    ],
}

_relations = [
    {
        "name": "shaft_in_lower_bearing",
        "mate_type": "journal_bearing",
        "base_part": "lower_input_bearing",
        "base_port": "journal",
        "incoming_part": "input_shaft",
        "incoming_port": "shaft_axis",
    },
    {
        "name": "shaft_in_upper_bearing",
        "mate_type": "journal_bearing",
        "base_part": "upper_input_bearing",
        "base_port": "journal",
        "incoming_part": "input_shaft",
        "incoming_port": "shaft_axis",
    },
    {
        "name": "sun_press_fit",
        "mate_type": "press_fit",
        "base_part": "input_shaft",
        "base_port": "sun_seat",
        "incoming_part": "sun_gear",
        "incoming_port": "shaft_bore",
    },
    {
        "name": "crank_press_fit",
        "mate_type": "press_fit",
        "base_part": "input_shaft",
        "base_port": "crank_seat",
        "incoming_part": "input_crank",
        "incoming_port": "hub_bore",
    },
    {
        "name": "carrier_on_support_bearing",
        "mate_type": "journal_bearing",
        "base_part": "carrier_bearing",
        "base_port": "carrier_journal",
        "incoming_part": "carrier",
        "incoming_port": "output_axis",
    },
]

_transmissions = []

for i, (px, py) in enumerate(PLANET_CENTERS, start=1):
    planet_name = f"planet_{i}"
    pin_name = f"planet_pin_{i}"

    _ports[planet_name] = [
        _axis_port(
            "pin_bore",
            "bore",
            px,
            py,
            GEAR_Z + GEAR_FACE_W / 2.0,
            2.0 * PLANET_BORE_R,
            depth=GEAR_FACE_W,
        ),
        _axis_port(
            "planet_mesh",
            "gear_mesh",
            px,
            py,
            GEAR_Z + GEAR_FACE_W / 2.0,
            pitch_radius=PLANET_PITCH_R,
            depth=GEAR_FACE_W,
        ),
    ]
    _ports[pin_name] = [
        _axis_port(
            "pin_axis",
            "shaft",
            px,
            py,
            PLANET_PIN_Z + PLANET_PIN_H / 2.0,
            2.0 * PLANET_PIN_R,
            depth=PLANET_PIN_H,
        )
    ]
    _ports["carrier"].append(
        _axis_port(
            f"pin_seat_{i}",
            "bore",
            px,
            py,
            CARRIER_Z + CARRIER_H / 2.0,
            2.0 * PLANET_PIN_PRESS_R,
            depth=CARRIER_H,
        )
    )

    radial = [px / PLANET_ORBIT_R, py / PLANET_ORBIT_R, 0.0]

    _relations.extend(
        [
            {
                "name": f"carrier_pin_press_fit_{i}",
                "mate_type": "press_fit",
                "base_part": "carrier",
                "base_port": f"pin_seat_{i}",
                "incoming_part": pin_name,
                "incoming_port": "pin_axis",
            },
            {
                "name": f"planet_pin_journal_{i}",
                "mate_type": "journal_bearing",
                "base_part": pin_name,
                "base_port": "pin_axis",
                "incoming_part": planet_name,
                "incoming_port": "pin_bore",
            },
            {
                "name": f"sun_planet_mesh_{i}",
                "mate_type": "gear_spur_external",
                "base_part": "sun_gear",
                "base_port": "sun_mesh",
                "incoming_part": planet_name,
                "incoming_port": "planet_mesh",
                "separation_axis": radial,
            },
            {
                "name": f"ring_planet_mesh_{i}",
                "mate_type": "gear_spur_external",
                "base_part": "fixed_ring",
                "base_port": "internal_mesh",
                "incoming_part": planet_name,
                "incoming_port": "planet_mesh",
                "separation_axis": [-radial[0], -radial[1], 0.0],
            },
        ]
    )

    _transmissions.extend(
        [
            {
                "name": f"sun_to_planet_{i}",
                "type": "gear_external",
                "driving_link": "sun_gear",
                "driven_link": planet_name,
                "ratio": 0,
            },
            {
                "name": f"planet_to_internal_ring_{i}",
                "type": "gear_internal",
                "driving_link": planet_name,
                "driven_link": "fixed_ring",
                "ratio": 0,
            },
        ]
    )

for i, (px, py) in enumerate(RING_POST_CENTERS, start=1):
    _ports[f"ring_post_{i}"] = [
        _axis_port(
            "support_axis",
            "cylindrical",
            px,
            py,
            RING_POST_Z + RING_POST_H / 2.0,
            2.0 * RING_POST_R,
            depth=RING_POST_H,
        )
    ]


MECHANISM = {
    "name": "open_frame_hand_driven_planetary_reducer",
    "output_link": "carrier",
    "watch_links": [
        "sun_gear",
        "planet_1",
        "planet_2",
        "planet_3",
        "carrier",
        "input_crank",
    ],
    "ports_by_link": _ports,
    "relations": _relations,
    "motion_joints": [
        {
            "name": "sun_input_hinge",
            "parent": "",
            "child": "sun_gear",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, GEAR_Z + GEAR_FACE_W / 2.0],
        },
        {
            "name": "carrier_output_hinge",
            "parent": "",
            "child": "carrier",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, CARRIER_Z + CARRIER_H / 2.0],
        },
        *[
            {
                "name": f"planet_{i}_carrier_hinge",
                "parent": "carrier",
                "child": f"planet_{i}",
                "type": "hinge",
                "axis": [0.0, 0.0, 1.0],
                "pos_mm": [
                    PLANET_CENTERS[i - 1][0],
                    PLANET_CENTERS[i - 1][1],
                    GEAR_Z + GEAR_FACE_W / 2.0,
                ],
            }
            for i in range(1, PLANET_COUNT + 1)
        ],
    ],
    "transmissions": _transmissions,
    "planetary_stages": [
        {
            "name": "four_to_one_planetary_stage",
            "sun": "sun_gear",
            "ring": "fixed_ring",
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
    def annulus(outer_r, inner_r, height):
        with BuildPart() as part:
            with BuildSketch():
                Circle(outer_r)
                Circle(inner_r, mode=Mode.SUBTRACT)
            extrude(amount=height)
        return part.part

    def internal_ring_gear(module, teeth, face_width):
        pitch_radius = pitch_r(teeth)
        tip_radius = pitch_radius - module
        root_radius = pitch_radius + 1.25 * module
        outer_radius = root_radius + 4.0

        root_half_angle = math.pi * 0.30 / teeth
        shoulder_half_angle = math.pi * 0.38 / teeth
        tip_half_angle = math.pi * 0.22 / teeth
        shoulder_radius = pitch_radius + 0.35 * module

        def polar(radius, angle):
            return (radius * math.cos(angle), radius * math.sin(angle))

        with BuildPart() as ring_part:
            with BuildSketch():
                Circle(outer_radius)
                Circle(root_radius, mode=Mode.SUBTRACT)

                for tooth_index in range(teeth):
                    center_angle = 2.0 * math.pi * tooth_index / teeth
                    tooth_points = (
                        polar(root_radius + 0.02, center_angle - root_half_angle),
                        polar(shoulder_radius, center_angle - shoulder_half_angle),
                        polar(tip_radius, center_angle - tip_half_angle),
                        polar(tip_radius, center_angle + tip_half_angle),
                        polar(shoulder_radius, center_angle + shoulder_half_angle),
                        polar(root_radius + 0.02, center_angle + root_half_angle),
                    )
                    Polygon(*tooth_points)
            extrude(amount=face_width)

        return ring_part.part

    def carrier_part():
        with BuildPart() as part:
            with BuildSketch():
                Circle(CARRIER_HUB_R)

                for px, py in PLANET_CENTERS:
                    ux = px / PLANET_ORBIT_R
                    uy = py / PLANET_ORBIT_R
                    nx = -uy
                    ny = ux

                    start_r = CARRIER_HUB_R - 1.0
                    end_r = PLANET_ORBIT_R
                    arm_points = (
                        (
                            ux * start_r + nx * CARRIER_ARM_HALF_W,
                            uy * start_r + ny * CARRIER_ARM_HALF_W,
                        ),
                        (
                            ux * end_r + nx * CARRIER_ARM_HALF_W,
                            uy * end_r + ny * CARRIER_ARM_HALF_W,
                        ),
                        (
                            ux * end_r - nx * CARRIER_ARM_HALF_W,
                            uy * end_r - ny * CARRIER_ARM_HALF_W,
                        ),
                        (
                            ux * start_r - nx * CARRIER_ARM_HALF_W,
                            uy * start_r - ny * CARRIER_ARM_HALF_W,
                        ),
                    )
                    Polygon(*arm_points)
                    with b3d.Locations((px, py)):
                        Circle(CARRIER_PIN_BOSS_R)

                Circle(CARRIER_BORE_R, mode=Mode.SUBTRACT)

                for px, py in PLANET_CENTERS:
                    with b3d.Locations((px, py)):
                        Circle(PLANET_PIN_PRESS_R, mode=Mode.SUBTRACT)

            extrude(amount=CARRIER_H)

        return part.part

    def crank_part():
        distal_center_x = CRANK_THROW
        with BuildPart() as part:
            with BuildSketch():
                Circle(CRANK_HUB_R)
                with b3d.Locations((distal_center_x, 0.0)):
                    Circle(CRANK_GRIP_R + 1.5)

                Polygon(
                    (0.0, CRANK_ARM_HALF_W),
                    (distal_center_x, CRANK_ARM_HALF_W),
                    (distal_center_x, -CRANK_ARM_HALF_W),
                    (0.0, -CRANK_ARM_HALF_W),
                )
                Circle(SHAFT_PRESS_BORE_R, mode=Mode.SUBTRACT)
            extrude(amount=CRANK_H)
        return part.part

    a = AssemblyHelper("open_frame_hand_driven_planetary_reducer")

    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(base, "base|dof=fixed")

    lower_bearing = annulus(
        INPUT_BEARING_OUTER_R,
        SHAFT_RUNNING_BORE_R,
        LOWER_BEARING_H,
    ).moved(Location((0.0, 0.0, LOWER_BEARING_Z)))
    a.add(lower_bearing, "lower_input_bearing|dof=fixed|mount=base")

    upper_bearing = annulus(
        INPUT_BEARING_OUTER_R,
        SHAFT_RUNNING_BORE_R,
        UPPER_BEARING_H,
    ).moved(Location((0.0, 0.0, UPPER_BEARING_Z)))
    a.add(upper_bearing, "upper_input_bearing|dof=fixed|mount=base")

    carrier_bearing = annulus(
        CARRIER_BEARING_OUTER_R,
        CARRIER_BEARING_INNER_R,
        CARRIER_BEARING_H,
    ).moved(Location((0.0, 0.0, CARRIER_BEARING_Z)))
    a.add(carrier_bearing, "carrier_bearing|dof=fixed|mount=base")

    input_shaft = Cylinder(
        INPUT_SHAFT_R,
        INPUT_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, INPUT_SHAFT_Z)))
    a.add(
        input_shaft,
        "input_shaft|dof=spin|spin_axis=z"
        "|mount=lower_input_bearing,upper_input_bearing",
    )

    for i, (px, py) in enumerate(RING_POST_CENTERS, start=1):
        post = Cylinder(
            RING_POST_R,
            RING_POST_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, RING_POST_Z)))
        a.add(post, f"ring_post_{i}|dof=fixed|mount=base")

    carrier = carrier_part().moved(Location((0.0, 0.0, CARRIER_Z)))
    a.add(
        carrier,
        "carrier|dof=spin|spin_axis=z|mount=carrier_bearing",
    )

    for i, (px, py) in enumerate(PLANET_CENTERS, start=1):
        pin = Cylinder(
            PLANET_PIN_R,
            PLANET_PIN_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, PLANET_PIN_Z)))
        a.add(pin, f"planet_pin_{i}|dof=fixed|mount=carrier")

    fixed_ring = internal_ring_gear(M, Z_RING, GEAR_FACE_W).moved(
        Location((0.0, 0.0, GEAR_Z))
    )
    a.add(
        fixed_ring,
        "fixed_ring|dof=fixed|mesh_id=planetary_stage"
        "|mount=ring_post_1,ring_post_2,ring_post_3",
    )

    sun = make_gear(
        M,
        Z_SUN,
        GEAR_FACE_W,
        2.0 * SHAFT_PRESS_BORE_R,
    ).moved(Location((0.0, 0.0, GEAR_Z)))
    a.add(
        sun,
        "sun_gear|dof=spin|driver=True|spin_axis=z"
        "|mesh_id=planetary_stage|mount=input_shaft",
    )

    for i, ((px, py), angle_deg) in enumerate(
        zip(PLANET_CENTERS, PLANET_ANGLES_DEG),
        start=1,
    ):
        # Rotation phase varies equally with orbital angle while all centers remain
        # exactly PLANET_ORBIT_R from the sun and ring axes.
        planet_phase_deg = angle_deg * Z_SUN / Z_PLANET + 180.0 / Z_PLANET
        planet = make_gear(
            M,
            Z_PLANET,
            GEAR_FACE_W,
            2.0 * PLANET_BORE_R,
        ).moved(
            Location(
                (px, py, GEAR_Z),
                (0.0, 0.0, planet_phase_deg),
            )
        )
        a.add(
            planet,
            f"planet_{i}|dof=spin|spin_axis=z"
            f"|mesh_id=planetary_stage|mount=planet_pin_{i}",
        )

    crank = crank_part().moved(Location((0.0, 0.0, CRANK_Z)))
    a.add(crank, "input_crank|dof=fixed|mount=input_shaft")

    crank_grip = Cylinder(
        CRANK_GRIP_R,
        CRANK_GRIP_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_THROW, 0.0, CRANK_GRIP_Z)))
    a.add(crank_grip, "crank_grip|dof=fixed|mount=input_crank")

    return a.build()