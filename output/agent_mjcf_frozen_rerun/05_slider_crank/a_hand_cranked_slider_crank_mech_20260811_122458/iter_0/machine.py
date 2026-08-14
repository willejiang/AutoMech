import math
from build123d import Axis

# ---------------------------------------------------------------------------
# KINEMATIC ARITHMETIC — all mechanism hardpoints are derived here first.
# ---------------------------------------------------------------------------

BASE_L = 140.0
BASE_W = 90.0
BASE_H = 5.0

CRANK_X = -35.0
CRANK_Y = 0.0
CRANK_R = 15.0
ROD_L = 55.0
THETA_DEG = 60.0
THETA = math.radians(THETA_DEG)

# Exact slider-crank closure:
# (wrist_x-crank_pin_x)^2 + (0-crank_pin_y)^2 = ROD_L^2
CRANK_PIN_X = CRANK_X + CRANK_R * math.cos(THETA)
CRANK_PIN_Y = CRANK_Y + CRANK_R * math.sin(THETA)
WRIST_Y = 0.0
WRIST_X = CRANK_PIN_X + math.sqrt(
    ROD_L**2 - (WRIST_Y - CRANK_PIN_Y)**2
)

ROD_DX = WRIST_X - CRANK_PIN_X
ROD_DY = WRIST_Y - CRANK_PIN_Y
ROD_ANGLE = math.atan2(ROD_DY, ROD_DX)
ROD_ANGLE_DEG = math.degrees(ROD_ANGLE)

# Dead-center output limits and exact stroke.
SLIDER_X_MIN = CRANK_X + ROD_L - CRANK_R
SLIDER_X_MAX = CRANK_X + ROD_L + CRANK_R
SLIDER_STROKE = SLIDER_X_MAX - SLIDER_X_MIN

# Axial stack, all measured from real faces.
LOWER_BUSH_Z = BASE_H
LOWER_BUSH_H = 5.0
UPPER_BRACKET_TOP_Z = 15.0
UPPER_BUSH_Z = UPPER_BRACKET_TOP_Z
UPPER_BUSH_H = 5.0

CRANK_WEB_Z = UPPER_BUSH_Z + UPPER_BUSH_H
CRANK_WEB_H = 4.0
CRANK_PIN_Z = CRANK_WEB_Z
CRANK_PIN_TOP_Z = 34.0
CRANK_PIN_H = CRANK_PIN_TOP_Z - CRANK_PIN_Z

GUIDE_TOP_Z = 22.0
SLIDER_Z = GUIDE_TOP_Z
SLIDER_LOWER_CHEEK_H = 3.0
SLIDER_POCKET_Z0 = SLIDER_Z + SLIDER_LOWER_CHEEK_H
SLIDER_POCKET_Z1 = 31.0
SLIDER_UPPER_CHEEK_H = 3.0
SLIDER_TOP_Z = SLIDER_POCKET_Z1 + SLIDER_UPPER_CHEEK_H

ROD_Z = 26.0
ROD_H = 4.0
ROD_CENTER_Z = ROD_Z + ROD_H / 2.0

WRIST_PIN_Z = SLIDER_Z
WRIST_PIN_TOP_Z = SLIDER_TOP_Z
WRIST_PIN_H = WRIST_PIN_TOP_Z - WRIST_PIN_Z

SHAFT_R = 4.0
SHAFT_RUNNING_R = SHAFT_R + 0.05
SHAFT_PRESS_R = SHAFT_R - 0.005
BUSH_OUTER_R = 7.0

PIN_R = 3.0
PIN_RUNNING_R = PIN_R + 0.05
PIN_PRESS_R = PIN_R - 0.005

ROD_BIG_END_R = 7.0
ROD_SMALL_END_R = 7.0
ROD_BAR_HALF_W = 4.0

SLIDER_L = 24.0
SLIDER_W = 28.0
SLIDER_SIDE_WALL = 3.0
SLIDER_GUIDE_CLEARANCE = 0.10

GUIDE_X0 = SLIDER_X_MIN - SLIDER_L / 2.0 - 2.0
GUIDE_X1 = SLIDER_X_MAX + SLIDER_L / 2.0 + 2.0
GUIDE_L = GUIDE_X1 - GUIDE_X0
GUIDE_XC = (GUIDE_X0 + GUIDE_X1) / 2.0
GUIDE_BED_W = SLIDER_W + 16.0
GUIDE_BED_H = GUIDE_TOP_Z - BASE_H

RAIL_W = 4.0
RAIL_H = SLIDER_TOP_Z - SLIDER_Z
RAIL_Y = SLIDER_W / 2.0 + SLIDER_GUIDE_CLEARANCE + RAIL_W / 2.0

HAND_ARM_Z = 37.0
HAND_ARM_H = 3.0
HAND_ARM_L = 34.0
HAND_RADIUS = 24.0
HAND_ANGLE_DEG = THETA_DEG + 180.0
HAND_ANGLE = math.radians(HAND_ANGLE_DEG)
HAND_GRIP_X = CRANK_X + HAND_RADIUS * math.cos(HAND_ANGLE)
HAND_GRIP_Y = CRANK_Y + HAND_RADIUS * math.sin(HAND_ANGLE)
HAND_GRIP_Z = HAND_ARM_Z + HAND_ARM_H
HAND_GRIP_H = 16.0

CRANKSHAFT_Z = BASE_H
CRANKSHAFT_TOP_Z = HAND_ARM_Z + HAND_ARM_H
CRANKSHAFT_H = CRANKSHAFT_TOP_Z - CRANKSHAFT_Z


MECHANISM = {
    "name": "hand_cranked_slider_crank",
    "output_link": "piston_slider",
    "watch_links": [
        "crankshaft",
        "connecting_rod",
        "piston_slider",
    ],
    "ports_by_link": {
        "baseplate": [
            {
                "name": "lower_bearing_seat",
                "type": "flat_face",
                "xyz_mm": [CRANK_X, CRANK_Y, BASE_H],
                "axis": [0, 0, 1],
                "normal_sign": 1,
            },
            {
                "name": "guide_mount",
                "type": "flat_face",
                "xyz_mm": [GUIDE_XC, 0, BASE_H],
                "axis": [0, 0, 1],
                "normal_sign": 1,
            },
        ],
        "lower_bushing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0, 0, LOWER_BUSH_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_RUNNING_R,
                "depth_mm": LOWER_BUSH_H,
            }
        ],
        "upper_bearing_bracket": [
            {
                "name": "bushing_seat",
                "type": "flat_face",
                "xyz_mm": [0, 0, UPPER_BRACKET_TOP_Z - BASE_H],
                "axis": [0, 0, 1],
                "normal_sign": 1,
            }
        ],
        "upper_bushing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0, 0, UPPER_BUSH_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_RUNNING_R,
                "depth_mm": UPPER_BUSH_H,
            }
        ],
        "crankshaft": [
            {
                "name": "main_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, (UPPER_BUSH_Z - CRANKSHAFT_Z) / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANKSHAFT_H,
            },
            {
                "name": "crank_pin_seat",
                "type": "bore",
                "xyz_mm": [
                    CRANK_R * math.cos(THETA),
                    CRANK_R * math.sin(THETA),
                    CRANK_WEB_Z - CRANKSHAFT_Z + CRANK_WEB_H / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * PIN_PRESS_R,
                "depth_mm": CRANK_WEB_H,
            },
            {
                "name": "hand_seat",
                "type": "shaft",
                "xyz_mm": [0, 0, HAND_ARM_Z - CRANKSHAFT_Z + HAND_ARM_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": HAND_ARM_H,
            },
        ],
        "crank_pin": [
            {
                "name": "press_seat",
                "type": "shaft",
                "xyz_mm": [0, 0, CRANK_WEB_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": CRANK_WEB_H,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, ROD_CENTER_Z - CRANK_PIN_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": ROD_H,
            },
        ],
        "connecting_rod": [
            {
                "name": "big_end_bore",
                "type": "bore",
                "xyz_mm": [0, 0, ROD_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * PIN_RUNNING_R,
                "depth_mm": ROD_H,
            },
            {
                "name": "small_end_bore",
                "type": "bore",
                "xyz_mm": [ROD_L, 0, ROD_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * PIN_RUNNING_R,
                "depth_mm": ROD_H,
            },
        ],
        "wrist_pin": [
            {
                "name": "slider_press",
                "type": "shaft",
                "xyz_mm": [0, 0, ROD_CENTER_Z - WRIST_PIN_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": WRIST_PIN_H,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, ROD_CENTER_Z - WRIST_PIN_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": ROD_H,
            },
        ],
        "piston_slider": [
            {
                "name": "wrist_bore",
                "type": "bore",
                "xyz_mm": [0, 0, ROD_CENTER_Z - SLIDER_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * PIN_PRESS_R,
                "depth_mm": WRIST_PIN_H,
            },
            {
                "name": "guide_bottom",
                "type": "flat_face",
                "xyz_mm": [0, 0, 0],
                "axis": [0, 0, 1],
                "normal_sign": -1,
            },
        ],
        "guide_bed": [
            {
                "name": "slide_surface",
                "type": "flat_face",
                "xyz_mm": [0, 0, GUIDE_BED_H],
                "axis": [0, 0, 1],
                "normal_sign": 1,
            }
        ],
        "hand_crank_arm": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0, 0, HAND_ARM_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_PRESS_R,
                "depth_mm": HAND_ARM_H,
            },
            {
                "name": "grip_mount",
                "type": "cylindrical",
                "xyz_mm": [
                    HAND_RADIUS * math.cos(HAND_ANGLE),
                    HAND_RADIUS * math.sin(HAND_ANGLE),
                    HAND_ARM_H,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 6.0,
            },
        ],
        "hand_grip": [
            {
                "name": "arm_mount",
                "type": "flat_face",
                "xyz_mm": [0, 0, 0],
                "axis": [0, 0, 1],
                "normal_sign": -1,
            }
        ],
    },
    "relations": [
        {
            "name": "lower_main_journal",
            "mate_type": "journal_bearing",
            "base_part": "lower_bushing",
            "base_port": "journal",
            "incoming_part": "crankshaft",
            "incoming_port": "main_journal",
        },
        {
            "name": "upper_main_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_bushing",
            "base_port": "journal",
            "incoming_part": "crankshaft",
            "incoming_port": "main_journal",
        },
        {
            "name": "crank_pin_press",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "crank_pin_seat",
            "incoming_part": "crank_pin",
            "incoming_port": "press_seat",
        },
        {
            "name": "big_end_revolute",
            "mate_type": "revolute",
            "base_part": "crank_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "big_end_bore",
        },
        {
            "name": "small_end_revolute",
            "mate_type": "revolute",
            "base_part": "wrist_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "small_end_bore",
        },
        {
            "name": "wrist_pin_press",
            "mate_type": "press_fit",
            "base_part": "piston_slider",
            "base_port": "wrist_bore",
            "incoming_part": "wrist_pin",
            "incoming_port": "slider_press",
        },
        {
            "name": "slider_on_bed",
            "mate_type": "face_to_face",
            "base_part": "guide_bed",
            "base_port": "slide_surface",
            "incoming_part": "piston_slider",
            "incoming_port": "guide_bottom",
            "separation_axis": "+z",
        },
        {
            "name": "hand_arm_press",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "hand_seat",
            "incoming_part": "hand_crank_arm",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "hand_grip_mount",
            "mate_type": "coaxial_face",
            "base_part": "hand_crank_arm",
            "base_port": "grip_mount",
            "incoming_part": "hand_grip",
            "incoming_port": "arm_mount",
        },
    ],
    "motion_joints": [
        {
            "name": "crankshaft_world_hinge",
            "parent": "",
            "child": "crankshaft",
            "type": "hinge",
            "axis": [0, 0, 1],
            "pos_mm": [0, 0, 0],
        },
        {
            "name": "slider_world_prismatic",
            "parent": "",
            "child": "piston_slider",
            "type": "slide",
            "axis": [1, 0, 0],
            "pos_mm": [0, 0, 0],
        },
    ],
    "transmissions": [],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("hand_cranked_slider_crank")

    # -----------------------------------------------------------------------
    # Baseplate
    # -----------------------------------------------------------------------
    baseplate = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(baseplate, "baseplate|dof=fixed")

    # -----------------------------------------------------------------------
    # Plain journal bushings and elevated second-bearing bracket
    # -----------------------------------------------------------------------
    lower_bushing_outer = Cylinder(
        BUSH_OUTER_R,
        LOWER_BUSH_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    lower_bushing_cut = Cylinder(
        SHAFT_RUNNING_R,
        LOWER_BUSH_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))
    lower_bushing = (lower_bushing_outer - lower_bushing_cut).moved(
        Location((CRANK_X, CRANK_Y, LOWER_BUSH_Z))
    )
    a.add(lower_bushing, "lower_bushing|dof=fixed|mount=baseplate")

    bracket_z = BASE_H
    bracket_column_h = UPPER_BRACKET_TOP_Z - BASE_H - 1.0

    bracket_col_1 = Box(
        5.0,
        5.0,
        bracket_column_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((-10.0, -10.0, 0)))
    bracket_col_2 = Box(
        5.0,
        5.0,
        bracket_column_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((-10.0, 10.0, 0)))
    bracket_top = Box(
        26.0,
        26.0,
        1.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, bracket_column_h)))
    bracket_hole = Cylinder(
        SHAFT_R + 0.20,
        3.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, bracket_column_h - 1.0)))

    upper_bearing_bracket = (
        bracket_col_1 + bracket_col_2 + bracket_top - bracket_hole
    ).moved(Location((CRANK_X, CRANK_Y, bracket_z)))
    a.add(
        upper_bearing_bracket,
        "upper_bearing_bracket|dof=fixed|mount=baseplate",
    )

    upper_bushing_outer = Cylinder(
        BUSH_OUTER_R,
        UPPER_BUSH_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    upper_bushing_cut = Cylinder(
        SHAFT_RUNNING_R,
        UPPER_BUSH_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))
    upper_bushing = (upper_bushing_outer - upper_bushing_cut).moved(
        Location((CRANK_X, CRANK_Y, UPPER_BUSH_Z))
    )
    a.add(
        upper_bushing,
        "upper_bushing|dof=fixed|mount=upper_bearing_bracket",
    )

    # -----------------------------------------------------------------------
    # One-piece crankshaft and crank cheek. The pin seat is a real press bore.
    # -----------------------------------------------------------------------
    crank_local_pin_x = CRANK_R * math.cos(THETA)
    crank_local_pin_y = CRANK_R * math.sin(THETA)

    main_shaft = Cylinder(
        SHAFT_R,
        CRANKSHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    crank_bar = Box(
        CRANK_R,
        10.0,
        CRANK_WEB_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (0, 0, CRANK_WEB_Z - CRANKSHAFT_Z),
            (0, 0, THETA_DEG),
        )
    )
    crank_center_boss = Cylinder(
        6.0,
        CRANK_WEB_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, CRANK_WEB_Z - CRANKSHAFT_Z)))
    crank_pin_boss = Cylinder(
        6.0,
        CRANK_WEB_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                crank_local_pin_x,
                crank_local_pin_y,
                CRANK_WEB_Z - CRANKSHAFT_Z,
            )
        )
    )
    crank_pin_seat_cut = Cylinder(
        PIN_PRESS_R,
        CRANK_WEB_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                crank_local_pin_x,
                crank_local_pin_y,
                CRANK_WEB_Z - CRANKSHAFT_Z - 1.0,
            )
        )
    )

    crankshaft = (
        main_shaft
        + crank_bar
        + crank_center_boss
        + crank_pin_boss
        - crank_pin_seat_cut
    ).moved(Location((CRANK_X, CRANK_Y, CRANKSHAFT_Z)))

    a.add(
        crankshaft,
        "crankshaft|dof=spin|driver=True|spin_axis=z|"
        "mount=lower_bushing,upper_bushing",
    )

    # Offset pin is seated through the crank cheek and projects through the rod.
    crank_pin = Cylinder(
        PIN_R,
        CRANK_PIN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_PIN_X, CRANK_PIN_Y, CRANK_PIN_Z)))
    a.add(crank_pin, "crank_pin|dof=fixed|mount=crankshaft")

    # -----------------------------------------------------------------------
    # Connecting rod: exact center length, eye bores, and pin-envelope relief.
    # It is relation-controlled, so dof=free is only a runtime representation.
    # -----------------------------------------------------------------------
    rod_bar = Box(
        ROD_L,
        2.0 * ROD_BAR_HALF_W,
        ROD_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    rod_big_eye = Cylinder(
        ROD_BIG_END_R,
        ROD_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    rod_small_eye = Cylinder(
        ROD_SMALL_END_R,
        ROD_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((ROD_L, 0, 0)))

    big_end_cut = Cylinder(
        PIN_RUNNING_R,
        ROD_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))
    small_end_cut = Cylinder(
        PIN_RUNNING_R,
        ROD_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((ROD_L, 0, -1.0)))

    connecting_rod = (
        rod_bar + rod_big_eye + rod_small_eye - big_end_cut - small_end_cut
    ).moved(
        Location(
            (CRANK_PIN_X, CRANK_PIN_Y, ROD_Z),
            (0, 0, ROD_ANGLE_DEG),
        )
    )
    a.add(
        connecting_rod,
        "connecting_rod|dof=free|mount=crank_pin,wrist_pin",
    )

    # -----------------------------------------------------------------------
    # Guide bed: its top face is the real gravity support for the slider.
    # -----------------------------------------------------------------------
    guide_bed = Box(
        GUIDE_L,
        GUIDE_BED_W,
        GUIDE_BED_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((GUIDE_XC, 0, BASE_H)))
    a.add(guide_bed, "guide_bed|dof=fixed|mount=baseplate")

    # -----------------------------------------------------------------------
    # Forked piston slider. The rod pocket is open through its complete width,
    # while upper/lower cheeks carry the wrist pin.
    # -----------------------------------------------------------------------
    slider_lower = Box(
        SLIDER_L,
        SLIDER_W,
        SLIDER_LOWER_CHEEK_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    slider_upper = Box(
        SLIDER_L,
        SLIDER_W,
        SLIDER_UPPER_CHEEK_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, SLIDER_POCKET_Z1 - SLIDER_Z)))

    slider_wall_h = SLIDER_POCKET_Z1 - SLIDER_POCKET_Z0
    slider_wall_y = SLIDER_W / 2.0 - SLIDER_SIDE_WALL / 2.0

    slider_wall_a = Box(
        SLIDER_L,
        SLIDER_SIDE_WALL,
        slider_wall_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (0, slider_wall_y, SLIDER_POCKET_Z0 - SLIDER_Z)
        )
    )
    slider_wall_b = Box(
        SLIDER_L,
        SLIDER_SIDE_WALL,
        slider_wall_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (0, -slider_wall_y, SLIDER_POCKET_Z0 - SLIDER_Z)
        )
    )

    wrist_press_cut = Cylinder(
        PIN_PRESS_R,
        WRIST_PIN_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))

    piston_slider = (
        slider_lower
        + slider_upper
        + slider_wall_a
        + slider_wall_b
        - wrist_press_cut
    ).moved(Location((WRIST_X, WRIST_Y, SLIDER_Z)))

    a.add(
        piston_slider,
        "piston_slider|dof=slide|slide_axis=x|"
        "mount=guide_bed,guide_rail_left,guide_rail_right",
    )

    # Wrist pin is press-fit only in the separated cheeks. Its middle span
    # remains exposed inside the pocket for the rod small end.
    wrist_pin = Cylinder(
        PIN_R,
        WRIST_PIN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((WRIST_X, WRIST_Y, WRIST_PIN_Z)))
    a.add(wrist_pin, "wrist_pin|dof=fixed|mount=piston_slider")

    # Side rails provide finite lateral clearance and prevent slider rotation.
    guide_rail_left = Box(
        GUIDE_L,
        RAIL_W,
        RAIL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((GUIDE_XC, RAIL_Y, SLIDER_Z)))
    a.add(
        guide_rail_left,
        "guide_rail_left|dof=fixed|mount=guide_bed",
    )

    guide_rail_right = Box(
        GUIDE_L,
        RAIL_W,
        RAIL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((GUIDE_XC, -RAIL_Y, SLIDER_Z)))
    a.add(
        guide_rail_right,
        "guide_rail_right|dof=fixed|mount=guide_bed",
    )

    # Narrow keeper strips leave 0.10 mm vertical running clearance.
    keeper_w = 6.0
    keeper_h = 2.0
    keeper_z = SLIDER_TOP_Z + SLIDER_GUIDE_CLEARANCE

    guide_keeper_left = Box(
        GUIDE_L,
        keeper_w,
        keeper_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                GUIDE_XC,
                SLIDER_W / 2.0 - keeper_w / 2.0,
                keeper_z,
            )
        )
    )
    a.add(
        guide_keeper_left,
        "guide_keeper_left|dof=fixed|mount=guide_rail_left",
    )

    guide_keeper_right = Box(
        GUIDE_L,
        keeper_w,
        keeper_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                GUIDE_XC,
                -SLIDER_W / 2.0 + keeper_w / 2.0,
                keeper_z,
            )
        )
    )
    a.add(
        guide_keeper_right,
        "guide_keeper_right|dof=fixed|mount=guide_rail_right",
    )

    # -----------------------------------------------------------------------
    # Raised hand crank and grip. The arm is opposite the crank pin so it
    # remains outside the connecting-rod working envelope.
    # -----------------------------------------------------------------------
    hand_arm_bar = Box(
        HAND_RADIUS,
        8.0,
        HAND_ARM_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, 0), (0, 0, HAND_ANGLE_DEG)))

    hand_arm_center = Cylinder(
        7.0,
        HAND_ARM_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    hand_arm_end = Cylinder(
        6.0,
        HAND_ARM_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                HAND_RADIUS * math.cos(HAND_ANGLE),
                HAND_RADIUS * math.sin(HAND_ANGLE),
                0,
            )
        )
    )
    hand_arm_bore = Cylinder(
        SHAFT_PRESS_R,
        HAND_ARM_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))

    hand_crank_arm = (
        hand_arm_bar + hand_arm_center + hand_arm_end - hand_arm_bore
    ).moved(Location((CRANK_X, CRANK_Y, HAND_ARM_Z)))

    a.add(
        hand_crank_arm,
        "hand_crank_arm|dof=fixed|mount=crankshaft",
    )

    hand_grip = Cylinder(
        5.0,
        HAND_GRIP_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((HAND_GRIP_X, HAND_GRIP_Y, HAND_GRIP_Z)))
    a.add(
        hand_grip,
        "hand_grip|dof=fixed|mount=hand_crank_arm",
    )

    return a.build()