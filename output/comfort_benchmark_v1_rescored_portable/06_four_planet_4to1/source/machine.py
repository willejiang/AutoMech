import math
import build123d as b3d
from build123d import Axis, Pos, Rotation, add

# ---------------------------------------------------------------------------
# Drivetrain arithmetic: all gear placement derives from these values.
# Fixed-ring planetary:
#   carrier reduction = 1 + Z_RING / Z_SUN
# Exact 4:1 therefore requires Z_RING = 3 * Z_SUN.
# Planetary geometry requires Z_RING = Z_SUN + 2 * Z_PLANET.
# ---------------------------------------------------------------------------

M = 1.0
Z_SUN = 16
Z_PLANET = 16
Z_RING = 48
PLANET_COUNT = 4

assert Z_RING == 3 * Z_SUN
assert Z_RING == Z_SUN + 2 * Z_PLANET
assert (Z_SUN + Z_RING) % PLANET_COUNT == 0

def pitch_r(z):
    return M * z / 2.0

def center_dist_external(za, zb):
    return M * (za + zb) / 2.0

def center_dist_internal(z_internal, z_external):
    return M * (z_internal - z_external) / 2.0

R_SUN_PITCH = pitch_r(Z_SUN)
R_PLANET_PITCH = pitch_r(Z_PLANET)
R_RING_PITCH = pitch_r(Z_RING)

SUN_PLANET_CD = center_dist_external(Z_SUN, Z_PLANET)
RING_PLANET_CD = center_dist_internal(Z_RING, Z_PLANET)
PLANET_ORBIT_R = SUN_PLANET_CD

assert abs(SUN_PLANET_CD - RING_PLANET_CD) < 1.0e-9

SUN_TO_CARRIER_REDUCTION = 1.0 + Z_RING / Z_SUN
assert abs(SUN_TO_CARRIER_REDUCTION - 4.0) < 1.0e-9

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

# Compatible four-planet indexing condition:
# (Z_SUN + Z_RING) / PLANET_COUNT must be integral.
PLANET_INDEX_STEP = (Z_SUN + Z_RING) // PLANET_COUNT

# Approximate initial tooth phasing. Exact centers remain controlled solely by
# the pitch geometry above.
SUN_PHASE_DEG = 0.0
PLANET_PHASES_DEG = [
    180.0 / Z_PLANET - (Z_SUN / Z_PLANET) * a
    for a in PLANET_ANGLES_DEG
]
RING_PHASE_DEG = 180.0 / Z_RING

# ---------------------------------------------------------------------------
# Axial stack and fits
# All custom cylinders/boxes use Align.MIN in Z and therefore span [z, z+h].
# ---------------------------------------------------------------------------

BASE_X = 76.0
BASE_Y = 76.0
BASE_H = 4.0
BASE_TOP_Z = BASE_H

GEAR_FACE_W = 6.0

INPUT_SHAFT_R = 2.0
INPUT_SHAFT_Z = BASE_TOP_Z
INPUT_SHAFT_H = 29.0

OUTPUT_SLEEVE_OUTER_R = 4.0
OUTPUT_SLEEVE_INNER_R = 3.0
OUTPUT_SLEEVE_Z = BASE_TOP_Z
OUTPUT_SLEEVE_H = 10.0

INPUT_RUNNING_BORE_R = INPUT_SHAFT_R + 0.05
OUTPUT_RUNNING_BORE_R = OUTPUT_SLEEVE_OUTER_R + 0.05

INPUT_BEARING_OUTER_R = OUTPUT_SLEEVE_INNER_R - 0.05
INPUT_BEARING_Z = BASE_TOP_Z
INPUT_BEARING_H = 6.0

OUTPUT_BEARING_OUTER_R = 6.5
OUTPUT_BEARING_Z = BASE_TOP_Z
OUTPUT_BEARING_H = 5.0

CARRIER_Z = 10.0
CARRIER_H = 3.0
CARRIER_TOP_Z = CARRIER_Z + CARRIER_H
CARRIER_CENTER_R = 7.0
CARRIER_ARM_HALF_W = 3.25
CARRIER_PIN_BOSS_R = 3.0
CARRIER_OUTER_REACH = PLANET_ORBIT_R + CARRIER_PIN_BOSS_R

OUTPUT_PRESS_BORE_R = OUTPUT_SLEEVE_OUTER_R - 0.005

GEAR_CLEARANCE_ABOVE_CARRIER = 2.0
GEAR_Z = CARRIER_TOP_Z + GEAR_CLEARANCE_ABOVE_CARRIER
GEAR_TOP_Z = GEAR_Z + GEAR_FACE_W

PLANET_PIN_R = 2.0
PLANET_PIN_PRESS_BORE_R = PLANET_PIN_R - 0.005
PLANET_GEAR_RUNNING_BORE_R = PLANET_PIN_R + 0.05
PLANET_PIN_Z = CARRIER_TOP_Z - 0.5
PLANET_PIN_H = GEAR_TOP_Z - PLANET_PIN_Z + 2.0

SUN_PRESS_BORE_R = INPUT_SHAFT_R - 0.005

CRANK_Z = GEAR_TOP_Z + 6.0
CRANK_H = 3.0
CRANK_RADIUS = 18.0
CRANK_ARM_W = 5.0
CRANK_HUB_R = 4.5
CRANK_HANDLE_R = 3.0
CRANK_HANDLE_Z = CRANK_Z + CRANK_H
CRANK_HANDLE_H = 14.0
CRANK_PRESS_BORE_R = INPUT_SHAFT_R - 0.005

# Internal ring dimensions.
RING_TIP_R = R_RING_PITCH - M
RING_ROOT_R = R_RING_PITCH + 1.25 * M
RING_WALL = 3.0
RING_OUTER_R = RING_ROOT_R + RING_WALL

# Four low supports are under the ring annulus and do not obscure its teeth.
RING_POST_R = 2.0
RING_POST_RADIAL_POS = RING_ROOT_R + 0.5 * RING_WALL
RING_POST_Z = BASE_TOP_Z
RING_POST_H = GEAR_Z - RING_POST_Z
RING_POST_ANGLES_DEG = [
    45.0 + i * 360.0 / PLANET_COUNT for i in range(PLANET_COUNT)
]
RING_POST_CENTERS = [
    (
        RING_POST_RADIAL_POS * math.cos(math.radians(a)),
        RING_POST_RADIAL_POS * math.sin(math.radians(a)),
    )
    for a in RING_POST_ANGLES_DEG
]

PLANET_NAMES = [f"planet_{i + 1}" for i in range(PLANET_COUNT)]
PIN_NAMES = [f"planet_pin_{i + 1}" for i in range(PLANET_COUNT)]
POST_NAMES = [f"ring_post_{i + 1}" for i in range(PLANET_COUNT)]

# ---------------------------------------------------------------------------
# Mechanism semantics
# Port coordinates are local to each named part.
# ---------------------------------------------------------------------------

MECHANISM = {
    "name": "open_frame_hand_driven_4to1_planetary_reducer",
    "output_link": "carrier",
    "watch_links": [
        "input_shaft",
        "sun_gear",
        "carrier",
        "output_sleeve",
        "planet_1",
        "planet_2",
        "planet_3",
        "planet_4",
    ],
    "ports_by_link": {
        "base": [
            {
                "name": "top_center",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, BASE_H],
                "axis": [0.0, 0.0, 1.0],
                "normal_sign": 1,
            }
        ],
        "input_shaft": [
            {
                "name": "shaft_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, INPUT_SHAFT_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": INPUT_SHAFT_H,
            },
            {
                "name": "sun_seat",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, GEAR_Z - INPUT_SHAFT_Z + GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": GEAR_FACE_W,
            },
            {
                "name": "crank_seat",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, CRANK_Z - INPUT_SHAFT_Z + CRANK_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_SHAFT_R,
                "depth_mm": CRANK_H,
            },
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
                "name": "pitch_axis",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": R_SUN_PITCH,
                "depth_mm": GEAR_FACE_W,
            },
        ],
        "fixed_ring_gear": [
            {
                "name": "ring_pitch_axis",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": R_RING_PITCH,
                "depth_mm": GEAR_FACE_W,
            }
        ],
        "carrier": [
            {
                "name": "output_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CARRIER_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_PRESS_BORE_R,
                "depth_mm": CARRIER_H,
            },
            *[
                {
                    "name": f"pin_seat_{i + 1}",
                    "type": "bore",
                    "xyz_mm": [
                        PLANET_CENTERS[i][0],
                        PLANET_CENTERS[i][1],
                        CARRIER_H / 2.0,
                    ],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_PIN_PRESS_BORE_R,
                    "depth_mm": CARRIER_H,
                }
                for i in range(PLANET_COUNT)
            ],
        ],
        "output_sleeve": [
            {
                "name": "outer_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, OUTPUT_SLEEVE_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_SLEEVE_OUTER_R,
                "depth_mm": OUTPUT_SLEEVE_H,
            },
            {
                "name": "carrier_seat",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, CARRIER_Z - OUTPUT_SLEEVE_Z + CARRIER_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_SLEEVE_OUTER_R,
                "depth_mm": CARRIER_H,
            },
        ],
        "input_bearing": [
            {
                "name": "input_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, INPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * INPUT_RUNNING_BORE_R,
                "depth_mm": INPUT_BEARING_H,
            }
        ],
        "output_bearing": [
            {
                "name": "output_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, OUTPUT_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * OUTPUT_RUNNING_BORE_R,
                "depth_mm": OUTPUT_BEARING_H,
            }
        ],
        "crank_arm": [
            {
                "name": "drive_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CRANK_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PRESS_BORE_R,
                "depth_mm": CRANK_H,
            }
        ],
        **{
            PLANET_NAMES[i]: [
                {
                    "name": "pin_bore",
                    "type": "bore",
                    "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_GEAR_RUNNING_BORE_R,
                    "depth_mm": GEAR_FACE_W,
                },
                {
                    "name": "pitch_axis",
                    "type": "gear_mesh",
                    "xyz_mm": [0.0, 0.0, GEAR_FACE_W / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "pitch_radius_mm": R_PLANET_PITCH,
                    "depth_mm": GEAR_FACE_W,
                },
            ]
            for i in range(PLANET_COUNT)
        },
        **{
            PIN_NAMES[i]: [
                {
                    "name": "pin_shaft",
                    "type": "shaft",
                    "xyz_mm": [0.0, 0.0, PLANET_PIN_H / 2.0],
                    "axis": [0.0, 0.0, 1.0],
                    "diameter_mm": 2.0 * PLANET_PIN_R,
                    "depth_mm": PLANET_PIN_H,
                }
            ]
            for i in range(PLANET_COUNT)
        },
    },
    "relations": [
        {
            "name": "input_shaft_in_input_bearing",
            "mate_type": "journal_bearing",
            "base_part": "input_bearing",
            "base_port": "input_bore",
            "incoming_part": "input_shaft",
            "incoming_port": "shaft_axis",
            "separation_axis": "+z",
        },
        {
            "name": "output_sleeve_in_output_bearing",
            "mate_type": "journal_bearing",
            "base_part": "output_bearing",
            "base_port": "output_bore",
            "incoming_part": "output_sleeve",
            "incoming_port": "outer_journal",
            "separation_axis": "+z",
        },
        {
            "name": "sun_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "sun_seat",
            "incoming_part": "sun_gear",
            "incoming_port": "shaft_bore",
            "separation_axis": "+z",
        },
        {
            "name": "carrier_output_press_fit",
            "mate_type": "press_fit",
            "base_part": "output_sleeve",
            "base_port": "carrier_seat",
            "incoming_part": "carrier",
            "incoming_port": "output_bore",
            "separation_axis": "+z",
        },
        *[
            {
                "name": f"planet_{i + 1}_journal",
                "mate_type": "journal_bearing",
                "base_part": PIN_NAMES[i],
                "base_port": "pin_shaft",
                "incoming_part": PLANET_NAMES[i],
                "incoming_port": "pin_bore",
                "separation_axis": "+z",
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
            "name": "output_sleeve_hinge",
            "parent": "",
            "child": "output_sleeve",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        *[
            {
                "name": f"planet_{i + 1}_carrier_hinge",
                "parent": "carrier",
                "child": PLANET_NAMES[i],
                "type": "hinge",
                "axis": [0.0, 0.0, 1.0],
                "pos_mm": [
                    PLANET_CENTERS[i][0],
                    PLANET_CENTERS[i][1],
                    0.0,
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
            "name": "carrier_to_output_sleeve",
            "type": "compound_1to1",
            "driving_link": "carrier",
            "driven_link": "output_sleeve",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [
        {
            "name": "four_planet_4to1_stage",
            "sun": "sun_gear",
            "ring": "fixed_ring_gear",
            "carrier": "carrier",
            "planets": [
                {"gear": PLANET_NAMES[i], "pin": PIN_NAMES[i]}
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


def _tube(outer_r, inner_r, height):
    """Origin-at-base annular sleeve."""
    outer = Cylinder(
        outer_r,
        height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    inner = Cylinder(
        inner_r,
        height + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    return outer - inner


def _make_internal_ring():
    """Internal ring with inward-pointing trapezoidal teeth."""
    tooth_overlap = 0.10
    root_join_r = RING_ROOT_R + tooth_overlap
    shoulder_r = RING_ROOT_R - 0.45 * M

    root_half_angle = 0.44 * math.pi / Z_RING
    shoulder_half_angle = 0.37 * math.pi / Z_RING
    tip_half_angle = 0.21 * math.pi / Z_RING

    def polar(radius, angle):
        return (
            radius * math.cos(angle),
            radius * math.sin(angle),
        )

    with BuildPart() as ring_part:
        with BuildSketch(Plane.XY):
            Circle(RING_OUTER_R)
            Circle(RING_ROOT_R, mode=Mode.SUBTRACT)

            for tooth_i in range(Z_RING):
                center_angle = (
                    2.0 * math.pi * tooth_i / Z_RING
                    + math.radians(RING_PHASE_DEG)
                )
                points = [
                    polar(root_join_r, center_angle - root_half_angle),
                    polar(shoulder_r, center_angle - shoulder_half_angle),
                    polar(RING_TIP_R, center_angle - tip_half_angle),
                    polar(RING_TIP_R, center_angle + tip_half_angle),
                    polar(shoulder_r, center_angle + shoulder_half_angle),
                    polar(root_join_r, center_angle + root_half_angle),
                ]
                Polygon(*points, mode=Mode.ADD)

        extrude(amount=GEAR_FACE_W)

    return ring_part.part


def _make_carrier():
    """Rigid cross carrier with a center output bore and four pin seats."""
    carrier_core = Cylinder(
        CARRIER_CENTER_R,
        CARRIER_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    arm_x = Box(
        2.0 * CARRIER_OUTER_REACH,
        2.0 * CARRIER_ARM_HALF_W,
        CARRIER_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    arm_y = Box(
        2.0 * CARRIER_ARM_HALF_W,
        2.0 * CARRIER_OUTER_REACH,
        CARRIER_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    carrier = carrier_core + arm_x + arm_y

    center_cut = Cylinder(
        OUTPUT_PRESS_BORE_R,
        CARRIER_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    carrier = carrier - center_cut

    for px, py in PLANET_CENTERS:
        pin_cut = Cylinder(
            PLANET_PIN_PRESS_BORE_R,
            CARRIER_H + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, -1.0)))
        carrier = carrier - pin_cut

    return carrier


def _make_crank_arm():
    """Open radial hand crank with a press-fit center bore."""
    hub = Cylinder(
        CRANK_HUB_R,
        CRANK_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    end = Cylinder(
        CRANK_HANDLE_R + 1.0,
        CRANK_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_RADIUS, 0.0, 0.0)))

    beam = Box(
        CRANK_RADIUS,
        CRANK_ARM_W,
        CRANK_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_RADIUS / 2.0, 0.0, 0.0)))

    crank = hub + beam + end

    bore = Cylinder(
        CRANK_PRESS_BORE_R,
        CRANK_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))

    return crank - bore


def build_machine():
    a = AssemblyHelper("open_frame_hand_driven_planetary_reducer")

    # Broad ground-supported base.
    base = Box(
        BASE_X,
        BASE_Y,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(base, "base|dof=fixed")

    # Fixed concentric sleeve bearings, fully visible from above and the sides.
    input_bearing = _tube(
        INPUT_BEARING_OUTER_R,
        INPUT_RUNNING_BORE_R,
        INPUT_BEARING_H,
    ).moved(Location((0.0, 0.0, INPUT_BEARING_Z)))
    a.add(input_bearing, "input_bearing|dof=fixed|mount=base")

    output_bearing = _tube(
        OUTPUT_BEARING_OUTER_R,
        OUTPUT_RUNNING_BORE_R,
        OUTPUT_BEARING_H,
    ).moved(Location((0.0, 0.0, OUTPUT_BEARING_Z)))
    a.add(output_bearing, "output_bearing|dof=fixed|mount=base")

    # Coaxial output sleeve surrounds the inner input bearing.
    output_sleeve = _tube(
        OUTPUT_SLEEVE_OUTER_R,
        OUTPUT_SLEEVE_INNER_R,
        OUTPUT_SLEEVE_H,
    ).moved(Location((0.0, 0.0, OUTPUT_SLEEVE_Z)))
    a.add(
        output_sleeve,
        "output_sleeve|dof=spin|spin_axis=z|mount=output_bearing",
    )

    # Sole driven input shaft.
    input_shaft = Cylinder(
        INPUT_SHAFT_R,
        INPUT_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, INPUT_SHAFT_Z)))
    a.add(
        input_shaft,
        "input_shaft|dof=spin|spin_axis=z|driver=True|mount=input_bearing",
    )

    # Rigid carrier below the gear plane.
    carrier = _make_carrier().moved(Location((0.0, 0.0, CARRIER_Z)))
    a.add(
        carrier,
        "carrier|dof=spin|spin_axis=z|mount=output_sleeve",
    )

    # Four press-fit carrier pins, each at an exact computed orbit center.
    for i, ((px, py), pin_name) in enumerate(
        zip(PLANET_CENTERS, PIN_NAMES)
    ):
        pin = Cylinder(
            PLANET_PIN_R,
            PLANET_PIN_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, PLANET_PIN_Z)))
        a.add(pin, f"{pin_name}|dof=fixed|mount=carrier")

    # Fixed ring supports terminate exactly at the ring's lower face.
    for (px, py), post_name in zip(RING_POST_CENTERS, POST_NAMES):
        post = Cylinder(
            RING_POST_R,
            RING_POST_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, RING_POST_Z)))
        a.add(post, f"{post_name}|dof=fixed|mount=base")

    # Internal fixed ring at the common gear axial station.
    fixed_ring = _make_internal_ring().moved(
        Location(
            (0.0, 0.0, GEAR_Z),
            (0.0, 0.0, RING_PHASE_DEG),
        )
    )
    a.add(
        fixed_ring,
        "fixed_ring_gear|dof=fixed|mount="
        + ",".join(POST_NAMES),
    )

    # Input sun, press-fitted to the input shaft.
    sun_gear = make_gear(
        M,
        Z_SUN,
        GEAR_FACE_W,
        2.0 * SUN_PRESS_BORE_R,
    ).moved(
        Location(
            (0.0, 0.0, GEAR_Z),
            (0.0, 0.0, SUN_PHASE_DEG),
        )
    )
    a.add(
        sun_gear,
        "sun_gear|dof=spin|spin_axis=z|mount=input_shaft",
    )

    # Four planets. Each center is exactly SUN_PLANET_CD from the sun and
    # exactly RING_PLANET_CD inward from the ring pitch circle.
    for i, ((px, py), phase, planet_name, pin_name) in enumerate(
        zip(
            PLANET_CENTERS,
            PLANET_PHASES_DEG,
            PLANET_NAMES,
            PIN_NAMES,
        )
    ):
        planet = make_gear(
            M,
            Z_PLANET,
            GEAR_FACE_W,
            2.0 * PLANET_GEAR_RUNNING_BORE_R,
        ).moved(
            Location(
                (px, py, GEAR_Z),
                (0.0, 0.0, phase),
            )
        )
        a.add(
            planet,
            f"{planet_name}|dof=spin|spin_axis=z|mount={pin_name}",
        )

    # Visible hand crank, press-fitted at a distinct station above the gears.
    crank_arm = _make_crank_arm().moved(
        Location((0.0, 0.0, CRANK_Z))
    )
    a.add(crank_arm, "crank_arm|dof=fixed|mount=input_shaft")

    crank_handle = Cylinder(
        CRANK_HANDLE_R,
        CRANK_HANDLE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((CRANK_RADIUS, 0.0, CRANK_HANDLE_Z))
    )
    a.add(crank_handle, "crank_handle|dof=fixed|mount=crank_arm")

    return a.build()