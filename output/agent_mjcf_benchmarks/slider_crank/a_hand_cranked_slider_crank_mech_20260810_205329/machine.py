import math

# ---------------------------------------------------------------------------
# Drivetrain / kinematic arithmetic — all hardpoints are solved first.
# This mechanism has no gears; motion is transmitted by crank-pin contact and
# the two explicit revolute closure relations.
# ---------------------------------------------------------------------------

CRANK_THROW = 22.0
ROD_LENGTH = 85.0
INITIAL_CRANK_ANGLE_DEG = 45.0
INITIAL_CRANK_ANGLE = math.radians(INITIAL_CRANK_ANGLE_DEG)

CRANK_X = 0.0
CRANK_Y = 0.0

CRANK_PIN_X = CRANK_X + CRANK_THROW * math.cos(INITIAL_CRANK_ANGLE)
CRANK_PIN_Y = CRANK_Y + CRANK_THROW * math.sin(INITIAL_CRANK_ANGLE)

# Slider axis is world +X at y=0. Positive square root selects the piston side.
ROD_DY = -CRANK_PIN_Y
ROD_DX = math.sqrt(ROD_LENGTH**2 - ROD_DY**2)
WRIST_X = CRANK_PIN_X + ROD_DX
WRIST_Y = 0.0
ROD_ANGLE = math.atan2(WRIST_Y - CRANK_PIN_Y, WRIST_X - CRANK_PIN_X)
ROD_ANGLE_DEG = math.degrees(ROD_ANGLE)

# Exact slider travel limits for a zero-offset slider-crank.
WRIST_X_MIN = ROD_LENGTH - CRANK_THROW
WRIST_X_MAX = ROD_LENGTH + CRANK_THROW
SLIDER_STROKE = WRIST_X_MAX - WRIST_X_MIN

# ---------------------------------------------------------------------------
# Fits and axial stations
# ---------------------------------------------------------------------------

SHAFT_R = 5.0
JOURNAL_BORE_R = SHAFT_R + 0.05
SHAFT_PRESS_BORE_R = SHAFT_R - 0.005

CRANK_PIN_R = 3.0
CRANK_PIN_RUNNING_BORE_R = CRANK_PIN_R + 0.05
CRANK_PIN_PRESS_BORE_R = CRANK_PIN_R - 0.005

WRIST_PIN_R = 3.0
WRIST_PIN_RUNNING_BORE_R = WRIST_PIN_R + 0.05
WRIST_PIN_PRESS_BORE_R = WRIST_PIN_R - 0.005

BASE_Z = 0.0
BASE_H = 8.0
BASE_TOP_Z = BASE_Z + BASE_H

LOWER_BEARING_H = 6.0
LOWER_BEARING_Z = BASE_TOP_Z
LOWER_BEARING_TOP_Z = LOWER_BEARING_Z + LOWER_BEARING_H

GUIDE_FLOOR_Z = BASE_TOP_Z
GUIDE_FLOOR_H = 8.0
SLIDER_FLOOR_Z = GUIDE_FLOOR_Z + GUIDE_FLOOR_H

CRANK_WEB_Z = 16.0
CRANK_WEB_H = 6.0
CRANK_WEB_TOP_Z = CRANK_WEB_Z + CRANK_WEB_H

ROD_Z = 24.0
ROD_H = 8.0
ROD_CENTER_Z = ROD_Z + ROD_H / 2.0
ROD_TOP_Z = ROD_Z + ROD_H

CRANK_PIN_Z = CRANK_WEB_Z
CRANK_PIN_H = 20.0
CRANK_PIN_TOP_Z = CRANK_PIN_Z + CRANK_PIN_H

WRIST_PIN_Z = 20.0
WRIST_PIN_H = 16.0
WRIST_PIN_CENTER_Z = WRIST_PIN_Z + WRIST_PIN_H / 2.0

LOWER_CHEEK_Z = 20.0
CHEEK_H = 3.0
UPPER_CHEEK_Z = 33.0
CHEEK_AXIAL_CLEARANCE = ROD_Z - (LOWER_CHEEK_Z + CHEEK_H)

BRIDGE_Z = 42.0
BRIDGE_H = 6.0
BRIDGE_TOP_Z = BRIDGE_Z + BRIDGE_H

UPPER_BEARING_Z = BRIDGE_Z
UPPER_BEARING_H = BRIDGE_H
UPPER_BEARING_CENTER_Z = UPPER_BEARING_Z + UPPER_BEARING_H / 2.0

SHAFT_Z = BASE_TOP_Z
SHAFT_TOP_Z = 56.0
SHAFT_H = SHAFT_TOP_Z - SHAFT_Z

HAND_ARM_Z = 50.0
HAND_ARM_H = 4.0
HANDLE_OFFSET = 32.0
HANDLE_PIN_Z = HAND_ARM_Z
HANDLE_PIN_H = 26.0
HANDLE_PIN_R = 3.5
HANDLE_PRESS_BORE_R = HANDLE_PIN_R - 0.005
GRIP_Z = HAND_ARM_Z + HAND_ARM_H + 2.0
GRIP_H = 18.0
GRIP_OUTER_R = 7.0

# ---------------------------------------------------------------------------
# Hardpoint-driven connecting-rod and guide envelope
# ---------------------------------------------------------------------------

ROD_BIG_OUTER_R = CRANK_PIN_R + 8.0
ROD_SMALL_OUTER_R = WRIST_PIN_R + 7.0
ROD_WEB_HALF_W = 5.0

SLIDER_CHEEK_R = ROD_SMALL_OUTER_R + 1.0
SLIDER_REAR_CLEARANCE = 0.5
SLIDER_REAR_X0 = SLIDER_CHEEK_R + SLIDER_REAR_CLEARANCE
SLIDER_REAR_X1 = SLIDER_REAR_X0 + 15.0

# Full rod sweep fits between the guide rails.
ROD_SWEEP_HALF_WIDTH = CRANK_THROW + ROD_BIG_OUTER_R
GUIDE_ENVELOPE_CLEARANCE = 1.0
GUIDE_INNER_HALF_W = ROD_SWEEP_HALF_WIDTH + GUIDE_ENVELOPE_CLEARANCE
GUIDE_RUNNING_CLEARANCE = 0.05
SLIDER_SHOE_HALF_W = GUIDE_INNER_HALF_W - GUIDE_RUNNING_CLEARANCE

SLIDER_FRONT_EXTENT = SLIDER_CHEEK_R
SLIDER_REAR_EXTENT = SLIDER_REAR_X1
GUIDE_END_CLEARANCE = 5.0
GUIDE_X_MIN = WRIST_X_MIN - SLIDER_FRONT_EXTENT - GUIDE_END_CLEARANCE
GUIDE_X_MAX = WRIST_X_MAX + SLIDER_REAR_EXTENT + GUIDE_END_CLEARANCE
GUIDE_LENGTH = GUIDE_X_MAX - GUIDE_X_MIN
GUIDE_CENTER_X = (GUIDE_X_MIN + GUIDE_X_MAX) / 2.0

GUIDE_RAIL_T = 6.0
GUIDE_RAIL_Z = SLIDER_FLOOR_Z
GUIDE_RAIL_H = 24.0
GUIDE_RAIL_CENTER_Y = GUIDE_INNER_HALF_W + GUIDE_RAIL_T / 2.0
GUIDE_TOP_UNDERSIDE_Z = GUIDE_RAIL_Z + GUIDE_RAIL_H + 0.05
GUIDE_TOP_H = 4.0

SLIDER_LOWER_SHOE_H = 4.0
SLIDER_UPPER_SHOE_Z = 20.0
SLIDER_UPPER_SHOE_H = 4.0
SLIDER_SHOE_X0 = SLIDER_REAR_X0
SLIDER_SHOE_X1 = SLIDER_REAR_X1

BEARING_OUTER_R = 11.0
BEARING_SEAT_R = BEARING_OUTER_R - 0.005

COLUMN_CENTER_Y = GUIDE_INNER_HALF_W + GUIDE_RAIL_T + 12.0
COLUMN_W = 16.0
COLUMN_H = BRIDGE_Z - BASE_TOP_Z
BRIDGE_X_SIZE = 55.0
BRIDGE_Y_SIZE = 2.0 * (COLUMN_CENTER_Y + COLUMN_W / 2.0)

BASE_X_MIN = -42.0
BASE_X_MAX = GUIDE_X_MAX + 10.0
BASE_LENGTH = BASE_X_MAX - BASE_X_MIN
BASE_CENTER_X = (BASE_X_MIN + BASE_X_MAX) / 2.0
BASE_WIDTH = BRIDGE_Y_SIZE + 12.0

# Local port coordinates are obtained by subtracting each part placement.
ROD_BIG_PORT_LOCAL = [0.0, 0.0, ROD_H / 2.0]
ROD_SMALL_PORT_LOCAL = [ROD_LENGTH, 0.0, ROD_H / 2.0]
SLIDER_WRIST_PORT_LOCAL = [
    0.0,
    0.0,
    WRIST_PIN_CENTER_Z - SLIDER_FLOOR_Z,
]

MECHANISM = {
    "name": "hand_cranked_slider_crank",
    "output_link": "piston_slider",
    "watch_links": [
        "crankshaft",
        "crank_web",
        "connecting_rod",
        "piston_slider",
        "output_marker",
    ],
    "ports_by_link": {
        "lower_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, LOWER_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * JOURNAL_BORE_R,
                "depth_mm": LOWER_BEARING_H,
            }
        ],
        "upper_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, UPPER_BEARING_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * JOURNAL_BORE_R,
                "depth_mm": UPPER_BEARING_H,
            }
        ],
        "crankshaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    LOWER_BEARING_Z + LOWER_BEARING_H / 2.0 - SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": LOWER_BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    UPPER_BEARING_CENTER_Z - SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": UPPER_BEARING_H,
            },
            {
                "name": "web_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    CRANK_WEB_Z + CRANK_WEB_H / 2.0 - SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANK_WEB_H,
            },
            {
                "name": "hand_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    HAND_ARM_Z + HAND_ARM_H / 2.0 - SHAFT_Z,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": HAND_ARM_H,
            },
        ],
        "crank_web": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CRANK_WEB_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_PRESS_BORE_R,
                "depth_mm": CRANK_WEB_H,
            },
            {
                "name": "crankpin_bore",
                "type": "bore",
                "xyz_mm": [CRANK_THROW, 0.0, CRANK_WEB_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_PRESS_BORE_R,
                "depth_mm": CRANK_WEB_H,
            },
        ],
        "crank_pin": [
            {
                "name": "web_shank",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, CRANK_WEB_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": CRANK_WEB_H,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, ROD_CENTER_Z - CRANK_PIN_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": ROD_H,
            },
        ],
        "connecting_rod": [
            {
                "name": "big_end_bore",
                "type": "bore",
                "xyz_mm": ROD_BIG_PORT_LOCAL,
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_RUNNING_BORE_R,
                "depth_mm": ROD_H,
            },
            {
                "name": "small_end_bore",
                "type": "bore",
                "xyz_mm": ROD_SMALL_PORT_LOCAL,
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * WRIST_PIN_RUNNING_BORE_R,
                "depth_mm": ROD_H,
            },
        ],
        "piston_slider": [
            {
                "name": "wrist_axis",
                "type": "bore",
                "xyz_mm": SLIDER_WRIST_PORT_LOCAL,
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * WRIST_PIN_PRESS_BORE_R,
                "depth_mm": WRIST_PIN_H,
            },
            {
                "name": "guide_axis",
                "type": "flat_face",
                "xyz_mm": [SLIDER_REAR_X0, 0.0, 0.0],
                "axis": [1.0, 0.0, 0.0],
                "normal_sign": 1,
            },
        ],
        "wrist_pin": [
            {
                "name": "slider_shank",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, WRIST_PIN_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * WRIST_PIN_R,
                "depth_mm": WRIST_PIN_H,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, ROD_CENTER_Z - WRIST_PIN_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * WRIST_PIN_R,
                "depth_mm": ROD_H,
            },
        ],
        "hand_arm": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, HAND_ARM_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_PRESS_BORE_R,
                "depth_mm": HAND_ARM_H,
            },
            {
                "name": "handle_bore",
                "type": "bore",
                "xyz_mm": [HANDLE_OFFSET, 0.0, HAND_ARM_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * HANDLE_PRESS_BORE_R,
                "depth_mm": HAND_ARM_H,
            },
        ],
        "handle_pin": [
            {
                "name": "arm_shank",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, HAND_ARM_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * HANDLE_PIN_R,
                "depth_mm": HAND_ARM_H,
            }
        ],
    },
    "relations": [
        {
            "name": "lower_crankshaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "lower_bearing",
            "base_port": "journal_bore",
            "incoming_part": "crankshaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "upper_crankshaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_bearing",
            "base_port": "journal_bore",
            "incoming_part": "crankshaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "web_on_crankshaft",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "web_seat",
            "incoming_part": "crank_web",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "crank_pin_in_web",
            "mate_type": "press_fit",
            "base_part": "crank_web",
            "base_port": "crankpin_bore",
            "incoming_part": "crank_pin",
            "incoming_port": "web_shank",
        },
        {
            "name": "rod_big_end_revolute",
            "mate_type": "revolute",
            "base_part": "crank_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "big_end_bore",
        },
        {
            "name": "wrist_pin_in_slider",
            "mate_type": "press_fit",
            "base_part": "piston_slider",
            "base_port": "wrist_axis",
            "incoming_part": "wrist_pin",
            "incoming_port": "slider_shank",
        },
        {
            "name": "rod_small_end_revolute",
            "mate_type": "revolute",
            "base_part": "wrist_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "small_end_bore",
        },
        {
            "name": "hand_arm_on_shaft",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "hand_seat",
            "incoming_part": "hand_arm",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "handle_pin_in_arm",
            "mate_type": "press_fit",
            "base_part": "hand_arm",
            "base_port": "handle_bore",
            "incoming_part": "handle_pin",
            "incoming_port": "arm_shank",
        },
    ],
    "motion_joints": [
        {
            "name": "crankshaft_rotation",
            "parent": "",
            "child": "crankshaft",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "slider_translation",
            "parent": "",
            "child": "piston_slider",
            "type": "slide",
            "axis": [1.0, 0.0, 0.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
    ],
    "transmissions": [],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("hand_cranked_slider_crank")

    def min_cylinder(radius, height):
        return Cylinder(
            radius,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )

    def min_box(length, width, height):
        return Box(
            length,
            width,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )

    def annulus(outer_r, inner_r, height):
        outer = min_cylinder(outer_r, height)
        cutter = Cylinder(
            inner_r,
            height + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -1.0)))
        return outer - cutter

    def two_eye_link(
        length,
        end0_outer_r,
        end1_outer_r,
        web_half_w,
        bore0_r,
        bore1_r,
        thickness,
    ):
        with BuildPart() as link_part:
            with BuildSketch() as link_profile:
                Circle(end0_outer_r)
                with b3d.Locations((length, 0.0)):
                    Circle(end1_outer_r)
                Polygon(
                    (0.0, web_half_w),
                    (length, web_half_w),
                    (length, -web_half_w),
                    (0.0, -web_half_w),
                )
            extrude(amount=thickness)

            with BuildSketch() as bore_profile:
                Circle(bore0_r)
                with b3d.Locations((length, 0.0)):
                    Circle(bore1_r)
            extrude(amount=thickness, mode=Mode.SUBTRACT)
        return link_part.part

    # -----------------------------------------------------------------------
    # Grounded structure
    # -----------------------------------------------------------------------

    base = min_box(BASE_LENGTH, BASE_WIDTH, BASE_H).moved(
        Location((BASE_CENTER_X, 0.0, BASE_Z))
    )
    a.add(base, "baseplate|dof=fixed")

    lower_bearing = annulus(
        BEARING_OUTER_R,
        JOURNAL_BORE_R,
        LOWER_BEARING_H,
    ).moved(Location((CRANK_X, CRANK_Y, LOWER_BEARING_Z)))
    a.add(lower_bearing, "lower_bearing|dof=fixed|mount=baseplate")

    left_column = min_box(COLUMN_W, COLUMN_W, COLUMN_H).moved(
        Location((CRANK_X, -COLUMN_CENTER_Y, BASE_TOP_Z))
    )
    right_column = min_box(COLUMN_W, COLUMN_W, COLUMN_H).moved(
        Location((CRANK_X, COLUMN_CENTER_Y, BASE_TOP_Z))
    )
    a.add(left_column, "bridge_left_column|dof=fixed|mount=baseplate")
    a.add(right_column, "bridge_right_column|dof=fixed|mount=baseplate")

    bridge_blank = min_box(BRIDGE_X_SIZE, BRIDGE_Y_SIZE, BRIDGE_H)
    bridge_seat_tool = Cylinder(
        BEARING_SEAT_R,
        BRIDGE_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    bridge = (bridge_blank - bridge_seat_tool).moved(
        Location((CRANK_X, CRANK_Y, BRIDGE_Z))
    )
    a.add(
        bridge,
        "upper_bridge|dof=fixed|mount=bridge_left_column,bridge_right_column",
    )

    upper_bearing = annulus(
        BEARING_OUTER_R,
        JOURNAL_BORE_R,
        UPPER_BEARING_H,
    ).moved(Location((CRANK_X, CRANK_Y, UPPER_BEARING_Z)))
    a.add(upper_bearing, "upper_bearing|dof=fixed|mount=upper_bridge")

    guide_floor = min_box(GUIDE_LENGTH, 2.0 * GUIDE_INNER_HALF_W, GUIDE_FLOOR_H)
    guide_floor = guide_floor.moved(
        Location((GUIDE_CENTER_X, 0.0, GUIDE_FLOOR_Z))
    )
    a.add(guide_floor, "guide_floor|dof=fixed|mount=baseplate")

    guide_left_rail = min_box(
        GUIDE_LENGTH,
        GUIDE_RAIL_T,
        GUIDE_RAIL_H,
    ).moved(
        Location(
            (
                GUIDE_CENTER_X,
                -GUIDE_RAIL_CENTER_Y,
                GUIDE_RAIL_Z,
            )
        )
    )
    guide_right_rail = min_box(
        GUIDE_LENGTH,
        GUIDE_RAIL_T,
        GUIDE_RAIL_H,
    ).moved(
        Location(
            (
                GUIDE_CENTER_X,
                GUIDE_RAIL_CENTER_Y,
                GUIDE_RAIL_Z,
            )
        )
    )
    a.add(guide_left_rail, "guide_left_rail|dof=fixed|mount=guide_floor")
    a.add(guide_right_rail, "guide_right_rail|dof=fixed|mount=guide_floor")

    top_strip_w = 12.0
    guide_top_left = min_box(
        GUIDE_LENGTH,
        top_strip_w,
        GUIDE_TOP_H,
    ).moved(
        Location(
            (
                GUIDE_CENTER_X,
                -(GUIDE_INNER_HALF_W - top_strip_w / 2.0),
                GUIDE_TOP_UNDERSIDE_Z,
            )
        )
    )
    guide_top_right = min_box(
        GUIDE_LENGTH,
        top_strip_w,
        GUIDE_TOP_H,
    ).moved(
        Location(
            (
                GUIDE_CENTER_X,
                GUIDE_INNER_HALF_W - top_strip_w / 2.0,
                GUIDE_TOP_UNDERSIDE_Z,
            )
        )
    )
    a.add(
        guide_top_left,
        "guide_top_left|dof=fixed|mount=guide_left_rail",
    )
    a.add(
        guide_top_right,
        "guide_top_right|dof=fixed|mount=guide_right_rail",
    )

    # -----------------------------------------------------------------------
    # Driver shaft and crank
    # -----------------------------------------------------------------------

    crankshaft = min_cylinder(SHAFT_R, SHAFT_H).moved(
        Location((CRANK_X, CRANK_Y, SHAFT_Z))
    )
    a.add(
        crankshaft,
        "crankshaft|dof=spin|driver=True|spin_axis=z|"
        "mount=lower_bearing,upper_bearing",
    )

    crank_web_local = two_eye_link(
        CRANK_THROW,
        12.0,
        9.0,
        7.0,
        SHAFT_PRESS_BORE_R,
        CRANK_PIN_PRESS_BORE_R,
        CRANK_WEB_H,
    )
    crank_web = crank_web_local.moved(
        Location(
            (CRANK_X, CRANK_Y, CRANK_WEB_Z),
            (0.0, 0.0, INITIAL_CRANK_ANGLE_DEG),
        )
    )
    a.add(crank_web, "crank_web|dof=fixed|mount=crankshaft")

    crank_pin = min_cylinder(CRANK_PIN_R, CRANK_PIN_H).moved(
        Location((CRANK_PIN_X, CRANK_PIN_Y, CRANK_PIN_Z))
    )
    a.add(crank_pin, "crank_pin|dof=fixed|mount=crank_web")

    # -----------------------------------------------------------------------
    # Relation-controlled connecting rod
    # -----------------------------------------------------------------------

    connecting_rod_local = two_eye_link(
        ROD_LENGTH,
        ROD_BIG_OUTER_R,
        ROD_SMALL_OUTER_R,
        ROD_WEB_HALF_W,
        CRANK_PIN_RUNNING_BORE_R,
        WRIST_PIN_RUNNING_BORE_R,
        ROD_H,
    )
    connecting_rod = connecting_rod_local.moved(
        Location(
            (CRANK_PIN_X, CRANK_PIN_Y, ROD_Z),
            (0.0, 0.0, ROD_ANGLE_DEG),
        )
    )
    a.add(
        connecting_rod,
        "connecting_rod|dof=free|mount=crank_pin,wrist_pin",
    )

    # -----------------------------------------------------------------------
    # Guided piston slider, built in a wrist-centered local frame
    # -----------------------------------------------------------------------

    lower_cheek = annulus(
        SLIDER_CHEEK_R,
        WRIST_PIN_PRESS_BORE_R,
        CHEEK_H,
    ).moved(
        Location(
            (
                0.0,
                0.0,
                LOWER_CHEEK_Z - SLIDER_FLOOR_Z,
            )
        )
    )

    upper_cheek = annulus(
        SLIDER_CHEEK_R,
        WRIST_PIN_PRESS_BORE_R,
        CHEEK_H,
    ).moved(
        Location(
            (
                0.0,
                0.0,
                UPPER_CHEEK_Z - SLIDER_FLOOR_Z,
            )
        )
    )

    shoe_length = SLIDER_SHOE_X1 - SLIDER_SHOE_X0
    shoe_center_x = (SLIDER_SHOE_X0 + SLIDER_SHOE_X1) / 2.0

    lower_shoe = min_box(
        shoe_length,
        2.0 * SLIDER_SHOE_HALF_W,
        SLIDER_LOWER_SHOE_H,
    ).moved(Location((shoe_center_x, 0.0, 0.0)))

    upper_shoe = min_box(
        shoe_length,
        2.0 * SLIDER_SHOE_HALF_W,
        SLIDER_UPPER_SHOE_H,
    ).moved(Location((shoe_center_x, 0.0, SLIDER_UPPER_SHOE_Z)))

    rear_bridge_h = (
        UPPER_CHEEK_Z + CHEEK_H - LOWER_CHEEK_Z
    )
    rear_bridge = min_box(
        shoe_length,
        18.0,
        rear_bridge_h,
    ).moved(
        Location(
            (
                shoe_center_x,
                0.0,
                LOWER_CHEEK_Z - SLIDER_FLOOR_Z,
            )
        )
    )

    lower_eye_arm = min_box(
        SLIDER_REAR_X0,
        12.0,
        CHEEK_H,
    ).moved(
        Location(
            (
                SLIDER_REAR_X0 / 2.0,
                0.0,
                LOWER_CHEEK_Z - SLIDER_FLOOR_Z,
            )
        )
    )
    upper_eye_arm = min_box(
        SLIDER_REAR_X0,
        12.0,
        CHEEK_H,
    ).moved(
        Location(
            (
                SLIDER_REAR_X0 / 2.0,
                0.0,
                UPPER_CHEEK_Z - SLIDER_FLOOR_Z,
            )
        )
    )

    piston_slider_local = (
        lower_cheek
        + upper_cheek
        + lower_shoe
        + upper_shoe
        + rear_bridge
        + lower_eye_arm
        + upper_eye_arm
    )
    piston_slider = piston_slider_local.moved(
        Location((WRIST_X, WRIST_Y, SLIDER_FLOOR_Z))
    )
    a.add(
        piston_slider,
        "piston_slider|dof=slide|slide_axis=x|"
        "mount=guide_floor,guide_left_rail,guide_right_rail,"
        "guide_top_left,guide_top_right",
    )

    wrist_pin = min_cylinder(WRIST_PIN_R, WRIST_PIN_H).moved(
        Location((WRIST_X, WRIST_Y, WRIST_PIN_Z))
    )
    a.add(wrist_pin, "wrist_pin|dof=fixed|mount=piston_slider")

    marker_length = 10.0
    marker = min_box(marker_length, 12.0, 12.0).moved(
        Location(
            (
                WRIST_X + SLIDER_REAR_X1 + marker_length / 2.0,
                0.0,
                LOWER_CHEEK_Z,
            )
        )
    )
    a.add(marker, "output_marker|dof=fixed|mount=piston_slider")

    # -----------------------------------------------------------------------
    # Manual crank arm and handle
    # -----------------------------------------------------------------------

    hand_arm_local = two_eye_link(
        HANDLE_OFFSET,
        10.0,
        8.0,
        6.0,
        SHAFT_PRESS_BORE_R,
        HANDLE_PRESS_BORE_R,
        HAND_ARM_H,
    )
    hand_arm = hand_arm_local.moved(
        Location(
            (CRANK_X, CRANK_Y, HAND_ARM_Z),
            (0.0, 0.0, 180.0),
        )
    )
    a.add(hand_arm, "hand_arm|dof=fixed|mount=crankshaft")

    handle_world_x = CRANK_X - HANDLE_OFFSET
    handle_world_y = CRANK_Y

    handle_pin = min_cylinder(HANDLE_PIN_R, HANDLE_PIN_H).moved(
        Location((handle_world_x, handle_world_y, HANDLE_PIN_Z))
    )
    a.add(handle_pin, "handle_pin|dof=fixed|mount=hand_arm")

    grip = annulus(
        GRIP_OUTER_R,
        HANDLE_PRESS_BORE_R,
        GRIP_H,
    ).moved(Location((handle_world_x, handle_world_y, GRIP_Z)))
    a.add(grip, "handle_grip|dof=fixed|mount=handle_pin")

    return a.build()