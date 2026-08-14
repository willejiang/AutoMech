import math
from build123d import Cone


# ============================================================================
# ENGINEERING ARITHMETIC AND HARDPOINTS — computed before any geometry
# ============================================================================

BASE_L = 150.0
BASE_W = 120.0
BASE_H = 10.0

BASE_BOLT_R = 3.0
BASE_BOLT_CLEAR_R = BASE_BOLT_R + 0.20
BASE_BOLT_HEAD_R = 5.5
BASE_BOLT_HEAD_H = 4.0
BASE_BOLT_X = 55.0
BASE_BOLT_Y = 40.0
BASE_BOLT_POINTS = [
    (-BASE_BOLT_X, -BASE_BOLT_Y),
    (-BASE_BOLT_X, BASE_BOLT_Y),
    (BASE_BOLT_X, -BASE_BOLT_Y),
    (BASE_BOLT_X, BASE_BOLT_Y),
]

PIN_R = 5.0
RUNNING_CLEARANCE = 0.08
PITCH_BORE_R = PIN_R + RUNNING_CLEARANCE
PRESS_INTERFERENCE = 0.005
YAW_BASE_BORE_R = PIN_R - PRESS_INTERFERENCE
YAW_RUNNING_BORE_R = PIN_R + RUNNING_CLEARANCE

THRUST_H = 2.0
THRUST_OUTER_R = 30.0
THRUST_BORE_R = PIN_R + 0.05
THRUST_Z = BASE_H

YAW_ORIGIN_Z = THRUST_Z + THRUST_H
YAW_TURNTABLE_R = 34.0
YAW_TURNTABLE_H = 10.0
YAW_PIN_H = YAW_ORIGIN_Z + YAW_TURNTABLE_H + 4.0

EYE_W = 14.0
EYE_OUTER_R = 15.0
CLEVIS_GAP = 0.70
LUG_W = 5.0
FORK_Y = EYE_W / 2.0 + CLEVIS_GAP + LUG_W / 2.0
FORK_OUTER_Y = FORK_Y + LUG_W / 2.0
PITCH_PIN_LENGTH = 2.0 * (FORK_OUTER_Y + 1.0)

LINK_BEAM_W = 12.0
LINK_BEAM_T = 10.0
FORK_RAIL_LENGTH = 36.0
FORK_CROSSBAR_DISTANCE = 32.0
FORK_CROSSBAR_LENGTH = 12.0

SHOULDER_LOCAL_Z = 82.0
SHOULDER_WORLD = (0.0, 0.0, YAW_ORIGIN_Z + SHOULDER_LOCAL_Z)

UPPER_ARM_LENGTH = 105.0
UPPER_ARM_ANGLE_RAD = math.radians(25.0)
UPPER_ARM_DX = UPPER_ARM_LENGTH * math.cos(UPPER_ARM_ANGLE_RAD)
UPPER_ARM_DZ = UPPER_ARM_LENGTH * math.sin(UPPER_ARM_ANGLE_RAD)
ELBOW_LOCAL = (UPPER_ARM_DX, 0.0, UPPER_ARM_DZ)
ELBOW_WORLD = (
    SHOULDER_WORLD[0] + ELBOW_LOCAL[0],
    0.0,
    SHOULDER_WORLD[2] + ELBOW_LOCAL[2],
)

FOREARM_LENGTH = 90.0
FOREARM_ANGLE_RAD = math.radians(-10.0)
FOREARM_DX = FOREARM_LENGTH * math.cos(FOREARM_ANGLE_RAD)
FOREARM_DZ = FOREARM_LENGTH * math.sin(FOREARM_ANGLE_RAD)
WRIST_LOCAL = (FOREARM_DX, 0.0, FOREARM_DZ)
WRIST_WORLD = (
    ELBOW_WORLD[0] + WRIST_LOCAL[0],
    0.0,
    ELBOW_WORLD[2] + WRIST_LOCAL[2],
)

WRIST_BODY_LENGTH = 48.0
POINTER_ANGLE_RAD = math.radians(12.0)
POINTER_DIR = (
    math.cos(POINTER_ANGLE_RAD),
    0.0,
    math.sin(POINTER_ANGLE_RAD),
)
POINTER_BASE_WORLD = (
    WRIST_WORLD[0] + WRIST_BODY_LENGTH * POINTER_DIR[0],
    0.0,
    WRIST_WORLD[2] + WRIST_BODY_LENGTH * POINTER_DIR[2],
)
POINTER_LENGTH = 70.0
POINTER_BASE_R = 8.0
POINTER_TIP_R = 1.5
POINTER_BALL_R = 5.0
POINTER_ROT_Y_DEG = 90.0 - math.degrees(POINTER_ANGLE_RAD)

SHOULDER_DRIVER_EFFORT_NM = 8.0


# ============================================================================
# TRUTHFUL MECHANISM SEMANTICS
# ============================================================================

MECHANISM = {
    "name": "open_frame_4dof_serial_robot_arm",
    "output_link": "end_effector_pointer",
    "watch_links": [
        "shoulder_link",
        "elbow_link",
        "wrist_link",
        "end_effector_pointer",
    ],
    "ports_by_link": {
        "base_frame": [
            {
                "name": "yaw_axis",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, YAW_ORIGIN_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * YAW_BASE_BORE_R,
                "depth_mm": BASE_H,
            }
        ],
        "yaw_pin": [
            {
                "name": "yaw_shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": YAW_PIN_H,
            }
        ],
        "yaw_carriage": [
            {
                "name": "yaw_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * YAW_RUNNING_BORE_R,
                "depth_mm": YAW_TURNTABLE_H,
            },
            {
                "name": "shoulder_clevis",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, SHOULDER_LOCAL_Z],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PITCH_BORE_R,
                "depth_mm": PITCH_PIN_LENGTH,
            },
        ],
        "shoulder_pin": [
            {
                "name": "shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": PITCH_PIN_LENGTH,
            }
        ],
        "shoulder_link": [
            {
                "name": "shoulder_eye",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PITCH_BORE_R,
                "depth_mm": EYE_W,
            },
            {
                "name": "elbow_clevis",
                "type": "cylindrical",
                "xyz_mm": [ELBOW_LOCAL[0], 0.0, ELBOW_LOCAL[2]],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PITCH_BORE_R,
                "depth_mm": PITCH_PIN_LENGTH,
            },
        ],
        "elbow_pin": [
            {
                "name": "shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": PITCH_PIN_LENGTH,
            }
        ],
        "elbow_link": [
            {
                "name": "elbow_eye",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PITCH_BORE_R,
                "depth_mm": EYE_W,
            },
            {
                "name": "wrist_clevis",
                "type": "cylindrical",
                "xyz_mm": [WRIST_LOCAL[0], 0.0, WRIST_LOCAL[2]],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PITCH_BORE_R,
                "depth_mm": PITCH_PIN_LENGTH,
            },
        ],
        "wrist_pin": [
            {
                "name": "shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": PITCH_PIN_LENGTH,
            }
        ],
        "wrist_link": [
            {
                "name": "wrist_eye",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PITCH_BORE_R,
                "depth_mm": EYE_W,
            },
            {
                "name": "tool_mount",
                "type": "flat_face",
                "xyz_mm": [
                    WRIST_BODY_LENGTH * POINTER_DIR[0],
                    0.0,
                    WRIST_BODY_LENGTH * POINTER_DIR[2],
                ],
                "axis": [POINTER_DIR[0], 0.0, POINTER_DIR[2]],
                "normal_sign": 1,
            },
        ],
        "end_effector_pointer": [
            {
                "name": "mount_face",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "normal_sign": -1,
            },
            {
                "name": "pointer_tip",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, POINTER_LENGTH + POINTER_BALL_R - 1.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * POINTER_BALL_R,
            },
        ],
    },
    "relations": [],
    "motion_joints": [
        {
            "name": "base_yaw_revolute",
            "parent": "",
            "child": "yaw_carriage",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
            "effort_limit_nm": 6.0,
        },
        {
            "name": "shoulder_pitch_revolute",
            "parent": "yaw_carriage",
            "child": "shoulder_link",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [0.0, 0.0, 0.0],
            "effort_limit_nm": SHOULDER_DRIVER_EFFORT_NM,
            "driver": True,
        },
        {
            "name": "elbow_pitch_revolute",
            "parent": "shoulder_link",
            "child": "elbow_link",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [0.0, 0.0, 0.0],
            "effort_limit_nm": 5.0,
        },
        {
            "name": "wrist_pitch_revolute",
            "parent": "elbow_link",
            "child": "wrist_link",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [0.0, 0.0, 0.0],
            "effort_limit_nm": 3.0,
        },
    ],
    "transmissions": [],
    "planetary_stages": [],
}


# ============================================================================
# GEOMETRY HELPERS
# ============================================================================

def _axis_y_cylinder(radius, length):
    """Centered cylinder whose resulting axis is global/local +Y."""
    return Cylinder(radius, length).moved(
        Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0))
    )


def _beam_between_xz(start_x, start_z, end_x, end_z, width_y, thickness):
    """Rectangular beam between two XZ hardpoints."""
    dx = end_x - start_x
    dz = end_z - start_z
    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    mid = ((start_x + end_x) / 2.0, 0.0, (start_z + end_z) / 2.0)
    return Box(length, width_y, thickness).moved(
        Location(mid, (0.0, -math.degrees(angle), 0.0))
    )


def _fork_link(distal_x, distal_z):
    """
    Proximal moving eye, central beam, spreader, and supported distal clevis.
    All hinge bores are cut after union so no rail can fill a pin envelope.
    """
    distance = math.hypot(distal_x, distal_z)
    ux = distal_x / distance
    uz = distal_z / distance
    angle_deg = math.degrees(math.atan2(distal_z, distal_x))

    proximal_hub = _axis_y_cylinder(EYE_OUTER_R, EYE_W)

    main_start_d = EYE_OUTER_R - 1.0
    main_end_d = distance - 27.0
    main_beam = _beam_between_xz(
        ux * main_start_d,
        uz * main_start_d,
        ux * main_end_d,
        uz * main_end_d,
        LINK_BEAM_W,
        LINK_BEAM_T,
    )

    cross_d = distance - FORK_CROSSBAR_DISTANCE
    crossbar = Box(
        FORK_CROSSBAR_LENGTH,
        2.0 * FORK_OUTER_Y,
        LINK_BEAM_T,
    ).moved(
        Location(
            (ux * cross_d, 0.0, uz * cross_d),
            (0.0, -angle_deg, 0.0),
        )
    )

    rail_center_d = distance - FORK_RAIL_LENGTH / 2.0
    rail_positive = Box(
        FORK_RAIL_LENGTH,
        LUG_W,
        LINK_BEAM_T,
    ).moved(
        Location(
            (ux * rail_center_d, FORK_Y, uz * rail_center_d),
            (0.0, -angle_deg, 0.0),
        )
    )
    rail_negative = Box(
        FORK_RAIL_LENGTH,
        LUG_W,
        LINK_BEAM_T,
    ).moved(
        Location(
            (ux * rail_center_d, -FORK_Y, uz * rail_center_d),
            (0.0, -angle_deg, 0.0),
        )
    )

    lug_positive = _axis_y_cylinder(EYE_OUTER_R, LUG_W).moved(
        Location((distal_x, FORK_Y, distal_z))
    )
    lug_negative = _axis_y_cylinder(EYE_OUTER_R, LUG_W).moved(
        Location((distal_x, -FORK_Y, distal_z))
    )

    link = (
        proximal_hub
        + main_beam
        + crossbar
        + rail_positive
        + rail_negative
        + lug_positive
        + lug_negative
    )

    proximal_bore = _axis_y_cylinder(PITCH_BORE_R, EYE_W + 2.0)
    distal_bore = _axis_y_cylinder(
        PITCH_BORE_R, PITCH_PIN_LENGTH + 2.0
    ).moved(Location((distal_x, 0.0, distal_z)))

    return link - proximal_bore - distal_bore


def _simple_wrist_link():
    """Moving wrist eye and short rigid tool carrier."""
    hub = _axis_y_cylinder(EYE_OUTER_R, EYE_W)

    start_d = EYE_OUTER_R - 1.0
    beam = _beam_between_xz(
        start_d * POINTER_DIR[0],
        start_d * POINTER_DIR[2],
        WRIST_BODY_LENGTH * POINTER_DIR[0],
        WRIST_BODY_LENGTH * POINTER_DIR[2],
        LINK_BEAM_W,
        LINK_BEAM_T,
    )

    bore = _axis_y_cylinder(PITCH_BORE_R, EYE_W + 2.0)
    return hub + beam - bore


# ============================================================================
# COMPLETE MACHINE
# ============================================================================

def build_machine():
    a = AssemblyHelper("open_frame_4dof_serial_robot_arm")

    # Rigid base with true through-holes for four bolts and the yaw pin seat.
    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    for bx, by in BASE_BOLT_POINTS:
        bolt_hole = Cylinder(
            BASE_BOLT_CLEAR_R,
            BASE_H + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((bx, by, -1.0)))
        base = base - bolt_hole

    yaw_press_bore = Cylinder(
        YAW_BASE_BORE_R,
        BASE_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    base = base - yaw_press_bore

    a.add(base, "base_frame|dof=fixed")

    # Four individual base bolts; shafts clear the holes and heads seat on top.
    for index, (bx, by) in enumerate(BASE_BOLT_POINTS, start=1):
        bolt_shaft = Cylinder(
            BASE_BOLT_R,
            BASE_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        bolt_head = Cylinder(
            BASE_BOLT_HEAD_R,
            BASE_BOLT_HEAD_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, BASE_H)))
        bolt = (bolt_shaft + bolt_head).moved(Location((bx, by, 0.0)))
        a.add(
            bolt,
            f"base_bolt_{index}|dof=fixed|mount=base_frame",
        )

    # Fixed vertical yaw pin, press-seated only in the base bore.
    yaw_pin = Cylinder(
        PIN_R,
        YAW_PIN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(yaw_pin, "yaw_pin|dof=fixed|mount=base_frame")

    # Annular thrust support: its real upper face defines the carriage bottom.
    thrust_outer = Cylinder(
        THRUST_OUTER_R,
        THRUST_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    thrust_cut = Cylinder(
        THRUST_BORE_R,
        THRUST_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    thrust_bearing = (thrust_outer - thrust_cut).moved(
        Location((0.0, 0.0, THRUST_Z))
    )
    a.add(
        thrust_bearing,
        "yaw_thrust_bearing|dof=fixed|mount=base_frame",
    )

    # Yaw carriage: annular turntable plus two exposed shoulder support posts.
    turntable = Cylinder(
        YAW_TURNTABLE_R,
        YAW_TURNTABLE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    mast_height = SHOULDER_LOCAL_Z - 8.0
    mast_positive = Box(
        16.0,
        LUG_W,
        mast_height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, FORK_Y, 8.0)))
    mast_negative = Box(
        16.0,
        LUG_W,
        mast_height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, -FORK_Y, 8.0)))

    shoulder_lug_positive = _axis_y_cylinder(
        EYE_OUTER_R, LUG_W
    ).moved(Location((0.0, FORK_Y, SHOULDER_LOCAL_Z)))
    shoulder_lug_negative = _axis_y_cylinder(
        EYE_OUTER_R, LUG_W
    ).moved(Location((0.0, -FORK_Y, SHOULDER_LOCAL_Z)))

    yaw_carriage = (
        turntable
        + mast_positive
        + mast_negative
        + shoulder_lug_positive
        + shoulder_lug_negative
    )

    yaw_bore_cut = Cylinder(
        YAW_RUNNING_BORE_R,
        YAW_TURNTABLE_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))

    shoulder_bore_cut = _axis_y_cylinder(
        PITCH_BORE_R,
        PITCH_PIN_LENGTH + 2.0,
    ).moved(Location((0.0, 0.0, SHOULDER_LOCAL_Z)))

    yaw_carriage = (yaw_carriage - yaw_bore_cut - shoulder_bore_cut).moved(
        Location((0.0, 0.0, YAW_ORIGIN_Z))
    )
    a.add(
        yaw_carriage,
        "yaw_carriage|mount=yaw_pin,yaw_thrust_bearing",
    )

    # Shoulder pin is carried by the yaw clevis and follows base-yaw motion.
    shoulder_pin = Cylinder(PIN_R, PITCH_PIN_LENGTH).moved(
        Location(SHOULDER_WORLD, (90.0, 0.0, 0.0))
    )
    a.add(
        shoulder_pin,
        "shoulder_pin|dof=fixed|mount=yaw_carriage",
    )

    # Driven shoulder body, including the complete elbow support clevis.
    shoulder_link = _fork_link(
        ELBOW_LOCAL[0],
        ELBOW_LOCAL[2],
    ).moved(Location(SHOULDER_WORLD))
    a.add(
        shoulder_link,
        "shoulder_link|driver=True|mount=shoulder_pin",
    )

    # Elbow pin is fixed to and transported by the shoulder link.
    elbow_pin = Cylinder(PIN_R, PITCH_PIN_LENGTH).moved(
        Location(ELBOW_WORLD, (90.0, 0.0, 0.0))
    )
    a.add(
        elbow_pin,
        "elbow_pin|dof=fixed|mount=shoulder_link",
    )

    # Forearm body, including the complete wrist support clevis.
    elbow_link = _fork_link(
        WRIST_LOCAL[0],
        WRIST_LOCAL[2],
    ).moved(Location(ELBOW_WORLD))
    a.add(
        elbow_link,
        "elbow_link|mount=elbow_pin",
    )

    # Wrist pin follows the elbow body.
    wrist_pin = Cylinder(PIN_R, PITCH_PIN_LENGTH).moved(
        Location(WRIST_WORLD, (90.0, 0.0, 0.0))
    )
    a.add(
        wrist_pin,
        "wrist_pin|dof=fixed|mount=elbow_link",
    )

    # Final moving wrist body.
    wrist_link = _simple_wrist_link().moved(Location(WRIST_WORLD))
    a.add(
        wrist_link,
        "wrist_link|mount=wrist_pin",
    )

    # Highly visible tapered pointer with an enlarged terminal ball.
    pointer_cone = Cone(
        POINTER_BASE_R,
        POINTER_TIP_R,
        POINTER_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    pointer_ball = b3d.Sphere(POINTER_BALL_R).moved(
        Location((0.0, 0.0, POINTER_LENGTH - 1.0))
    )
    pointer = (pointer_cone + pointer_ball).moved(
        Location(
            POINTER_BASE_WORLD,
            (0.0, POINTER_ROT_Y_DEG, 0.0),
        )
    )
    a.add(
        pointer,
        "end_effector_pointer|dof=fixed|mount=wrist_link",
    )

    return a.build()