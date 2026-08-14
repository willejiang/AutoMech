import math

# ---------------------------------------------------------------------------
# Slider-crank arithmetic: derive every mechanism hardpoint before geometry.
# ---------------------------------------------------------------------------

CRANK_RADIUS = 12.0
ROD_LENGTH = 50.0
CRANK_ANGLE_DEG = 35.0
CRANK_ANGLE_RAD = math.radians(CRANK_ANGLE_DEG)

CRANK_AXIS_X = 0.0
CRANK_AXIS_Y = 0.0

CRANK_PIN_X = CRANK_AXIS_X + CRANK_RADIUS * math.cos(CRANK_ANGLE_RAD)
CRANK_PIN_Y = CRANK_AXIS_Y + CRANK_RADIUS * math.sin(CRANK_ANGLE_RAD)

# Slider axis is world +X at y=0. Select the physically outward solution.
SLIDER_WRIST_X = (
    CRANK_PIN_X
    + math.sqrt(ROD_LENGTH**2 - CRANK_PIN_Y**2)
)
SLIDER_WRIST_Y = 0.0

ROD_DX = SLIDER_WRIST_X - CRANK_PIN_X
ROD_DY = SLIDER_WRIST_Y - CRANK_PIN_Y
ROD_ANGLE_RAD = math.atan2(ROD_DY, ROD_DX)
ROD_ANGLE_DEG = math.degrees(ROD_ANGLE_RAD)

# Verify the solved closure numerically.
SOLVED_ROD_LENGTH = math.hypot(ROD_DX, ROD_DY)
assert abs(SOLVED_ROD_LENGTH - ROD_LENGTH) < 1.0e-9

# Full slider-center travel bounds for guide and base sizing.
SLIDER_CENTER_MIN_X = ROD_LENGTH - CRANK_RADIUS
SLIDER_CENTER_MAX_X = ROD_LENGTH + CRANK_RADIUS

# ---------------------------------------------------------------------------
# Fits and axial stations.
# All stacked primitives use Align.MIN in Z.
# ---------------------------------------------------------------------------

BASE_Z = 0.0
BASE_H = 4.0
BASE_TOP_Z = BASE_Z + BASE_H

SHAFT_R = 3.0
SHAFT_Z = BASE_TOP_Z
SHAFT_H = 10.2
SHAFT_TOP_Z = SHAFT_Z + SHAFT_H

BEARING_BORE_R = SHAFT_R + 0.05
BEARING_OUTER_R = 6.5
LOWER_BEARING_Z = BASE_TOP_Z
LOWER_BEARING_H = 3.0
LOWER_BEARING_TOP_Z = LOWER_BEARING_Z + LOWER_BEARING_H
UPPER_BEARING_Z = LOWER_BEARING_TOP_Z
UPPER_BEARING_H = 3.0
UPPER_BEARING_TOP_Z = UPPER_BEARING_Z + UPPER_BEARING_H

CRANK_DISK_R = CRANK_RADIUS + 5.5
CRANK_DISK_H = 3.5
CRANK_DISK_Z = UPPER_BEARING_TOP_Z + 0.30
CRANK_DISK_TOP_Z = CRANK_DISK_Z + CRANK_DISK_H
CRANK_DISK_SHAFT_BORE_R = SHAFT_R - 0.005

CRANK_PIN_R = 2.0
CRANK_PIN_DISK_BORE_R = CRANK_PIN_R - 0.005
CRANK_PIN_Z = CRANK_DISK_Z
CRANK_PIN_TOP_Z = CRANK_DISK_TOP_Z + 4.4
CRANK_PIN_H = CRANK_PIN_TOP_Z - CRANK_PIN_Z

ROD_PIN_CLEARANCE = 0.05
ROD_BORE_R = CRANK_PIN_R + ROD_PIN_CLEARANCE
ROD_END_R = 5.0
ROD_HALF_WIDTH = 3.0
ROD_T = 3.6
ROD_Z = CRANK_DISK_TOP_Z + 0.40
ROD_TOP_Z = ROD_Z + ROD_T
ROD_CENTER_Z = ROD_Z + ROD_T / 2.0

WRIST_PIN_R = CRANK_PIN_R
WRIST_PIN_BORE_R = WRIST_PIN_R - 0.005
WRIST_PIN_Z = CRANK_DISK_Z + 0.70

SLIDER_LENGTH = 18.0
SLIDER_WIDTH = 16.0
SLIDER_LOWER_H = ROD_Z - BASE_TOP_Z - 0.40
SLIDER_LOWER_TOP_Z = BASE_TOP_Z + SLIDER_LOWER_H

SLIDER_UPPER_Z = ROD_TOP_Z + 0.40
SLIDER_UPPER_H = 2.5
SLIDER_UPPER_TOP_Z = SLIDER_UPPER_Z + SLIDER_UPPER_H

WRIST_PIN_TOP_Z = SLIDER_UPPER_TOP_Z
WRIST_PIN_H = WRIST_PIN_TOP_Z - WRIST_PIN_Z

GUIDE_CLEARANCE = 0.05
GUIDE_RAIL_W = 3.0
GUIDE_RAIL_H = 5.0
GUIDE_MARGIN_X = 2.0
GUIDE_LENGTH = (
    (SLIDER_CENTER_MAX_X - SLIDER_CENTER_MIN_X)
    + SLIDER_LENGTH
    + 2.0 * GUIDE_MARGIN_X
)
GUIDE_CENTER_X = (
    SLIDER_CENTER_MIN_X + SLIDER_CENTER_MAX_X
) / 2.0
GUIDE_RAIL_CENTER_Y = (
    SLIDER_WIDTH / 2.0 + GUIDE_CLEARANCE + GUIDE_RAIL_W / 2.0
)

BASE_MIN_X = -CRANK_DISK_R - 4.0
BASE_MAX_X = (
    SLIDER_CENTER_MAX_X + SLIDER_LENGTH / 2.0 + GUIDE_MARGIN_X + 5.0
)
BASE_LENGTH = BASE_MAX_X - BASE_MIN_X
BASE_CENTER_X = (BASE_MIN_X + BASE_MAX_X) / 2.0
BASE_WIDTH = 50.0

# Slider-local Z values. The slider placement is (wrist_x, 0, BASE_TOP_Z).
SLIDER_LOCAL_LOWER_BORE_Z = SLIDER_LOWER_H / 2.0
SLIDER_LOCAL_UPPER_Z = SLIDER_UPPER_Z - BASE_TOP_Z
SLIDER_LOCAL_UPPER_BORE_Z = SLIDER_LOCAL_UPPER_Z + SLIDER_UPPER_H / 2.0

# Joint-port locations along the actual pin axes.
CRANK_PIN_DISK_SEAT_LOCAL_Z = CRANK_DISK_H / 2.0
CRANK_PIN_ROD_LOCAL_Z = ROD_CENTER_Z - CRANK_PIN_Z
WRIST_PIN_LOWER_LOCAL_Z = (
    BASE_TOP_Z + SLIDER_LOCAL_LOWER_BORE_Z - WRIST_PIN_Z
)
WRIST_PIN_ROD_LOCAL_Z = ROD_CENTER_Z - WRIST_PIN_Z
WRIST_PIN_UPPER_LOCAL_Z = (
    BASE_TOP_Z + SLIDER_LOCAL_UPPER_BORE_Z - WRIST_PIN_Z
)

MECHANISM = {
    "name": "open_horizontal_hand_cranked_slider_crank",
    "output_link": "slider",
    "watch_links": [
        "crankshaft",
        "crank_disk",
        "connecting_rod",
        "slider",
    ],
    "ports_by_link": {
        "base": [
            {
                "name": "lower_bearing_mount",
                "type": "flat_face",
                "xyz_mm": [
                    CRANK_AXIS_X - BASE_CENTER_X,
                    CRANK_AXIS_Y,
                    BASE_TOP_Z,
                ],
                "axis": [0, 0, 1],
                "normal_sign": 1,
            }
        ],
        "lower_bearing": [
            {
                "name": "shaft_journal",
                "type": "bore",
                "xyz_mm": [0, 0, LOWER_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": LOWER_BEARING_H,
            }
        ],
        "upper_bearing": [
            {
                "name": "shaft_journal",
                "type": "bore",
                "xyz_mm": [0, 0, UPPER_BEARING_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": UPPER_BEARING_H,
            }
        ],
        "crankshaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [
                    0,
                    0,
                    LOWER_BEARING_Z + LOWER_BEARING_H / 2.0 - SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": LOWER_BEARING_H,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0,
                    0,
                    UPPER_BEARING_Z + UPPER_BEARING_H / 2.0 - SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": UPPER_BEARING_H,
            },
            {
                "name": "disk_seat",
                "type": "shaft",
                "xyz_mm": [
                    0,
                    0,
                    CRANK_DISK_Z + CRANK_DISK_H / 2.0 - SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANK_DISK_H,
            },
        ],
        "crank_disk": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0, 0, CRANK_DISK_H / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CRANK_DISK_SHAFT_BORE_R,
                "depth_mm": CRANK_DISK_H,
            },
            {
                "name": "eccentric_pin_bore",
                "type": "bore",
                "xyz_mm": [
                    CRANK_PIN_X,
                    CRANK_PIN_Y,
                    CRANK_DISK_H / 2.0,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CRANK_PIN_DISK_BORE_R,
                "depth_mm": CRANK_DISK_H,
            },
        ],
        "crank_pin": [
            {
                "name": "disk_press_seat",
                "type": "shaft",
                "xyz_mm": [0, 0, CRANK_PIN_DISK_SEAT_LOCAL_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": CRANK_DISK_H,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, CRANK_PIN_ROD_LOCAL_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": ROD_T,
            },
        ],
        "connecting_rod": [
            {
                "name": "big_end_bore",
                "type": "bore",
                "xyz_mm": [0, 0, ROD_T / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * ROD_BORE_R,
                "depth_mm": ROD_T,
            },
            {
                "name": "small_end_bore",
                "type": "bore",
                "xyz_mm": [ROD_LENGTH, 0, ROD_T / 2.0],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * ROD_BORE_R,
                "depth_mm": ROD_T,
            },
        ],
        "slider": [
            {
                "name": "slide_reference",
                "type": "flat_face",
                "xyz_mm": [0, 0, 0],
                "axis": [1, 0, 0],
                "normal_sign": 1,
            },
            {
                "name": "lower_wrist_bore",
                "type": "bore",
                "xyz_mm": [0, 0, SLIDER_LOCAL_LOWER_BORE_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * WRIST_PIN_BORE_R,
                "depth_mm": SLIDER_LOWER_H,
            },
            {
                "name": "upper_wrist_bore",
                "type": "bore",
                "xyz_mm": [0, 0, SLIDER_LOCAL_UPPER_BORE_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * WRIST_PIN_BORE_R,
                "depth_mm": SLIDER_UPPER_H,
            },
        ],
        "wrist_pin": [
            {
                "name": "lower_press_seat",
                "type": "shaft",
                "xyz_mm": [0, 0, WRIST_PIN_LOWER_LOCAL_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * WRIST_PIN_R,
                "depth_mm": SLIDER_LOWER_H,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [0, 0, WRIST_PIN_ROD_LOCAL_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * WRIST_PIN_R,
                "depth_mm": ROD_T,
            },
            {
                "name": "upper_press_seat",
                "type": "shaft",
                "xyz_mm": [0, 0, WRIST_PIN_UPPER_LOCAL_Z],
                "axis": [0, 0, 1],
                "diameter_mm": 2.0 * WRIST_PIN_R,
                "depth_mm": SLIDER_UPPER_H,
            },
        ],
    },
    "relations": [
        {
            "name": "crankshaft_lower_revolute",
            "mate_type": "revolute",
            "base_part": "lower_bearing",
            "base_port": "shaft_journal",
            "incoming_part": "crankshaft",
            "incoming_port": "lower_journal",
        },
        {
            "name": "crankshaft_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "upper_bearing",
            "base_port": "shaft_journal",
            "incoming_part": "crankshaft",
            "incoming_port": "upper_journal",
        },
        {
            "name": "disk_to_crankshaft_press_fit",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "disk_seat",
            "incoming_part": "crank_disk",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "eccentric_pin_to_disk_press_fit",
            "mate_type": "press_fit",
            "base_part": "crank_disk",
            "base_port": "eccentric_pin_bore",
            "incoming_part": "crank_pin",
            "incoming_port": "disk_press_seat",
        },
        {
            "name": "crank_end_pin_closure",
            "mate_type": "pin",
            "base_part": "crank_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "big_end_bore",
        },
        {
            "name": "lower_wrist_pin_press_fit",
            "mate_type": "press_fit",
            "base_part": "slider",
            "base_port": "lower_wrist_bore",
            "incoming_part": "wrist_pin",
            "incoming_port": "lower_press_seat",
        },
        {
            "name": "upper_wrist_pin_press_fit",
            "mate_type": "press_fit",
            "base_part": "slider",
            "base_port": "upper_wrist_bore",
            "incoming_part": "wrist_pin",
            "incoming_port": "upper_press_seat",
        },
        {
            "name": "slider_end_pin_closure",
            "mate_type": "pin",
            "base_part": "wrist_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "small_end_bore",
        },
    ],
    "motion_joints": [
        {
            "name": "input_crankshaft_hinge",
            "parent": "",
            "child": "crankshaft",
            "type": "hinge",
            "axis": [0, 0, 1],
            "pos_mm": [0, 0, 0],
        },
        {
            "name": "horizontal_slider_joint",
            "parent": "",
            "child": "slider",
            "type": "slide",
            "axis": [1, 0, 0],
            "pos_mm": [0, 0, 0],
        },
    ],
    "transmissions": [
        {
            "name": "crankshaft_to_disk_rigid_drive",
            "type": "compound_1to1",
            "driving_link": "crankshaft",
            "driven_link": "crank_disk",
            "ratio": 1.0,
        }
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("open_horizontal_hand_cranked_slider_crank")

    def annulus(outer_r, inner_r, height):
        outer = Cylinder(
            outer_r,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        cutter = Cylinder(
            inner_r,
            height + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0, 0, -1.0)))
        return outer - cutter

    # Open base: the only broad structural surface, kept below the mechanism.
    base = Box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((BASE_CENTER_X, 0, BASE_Z)))
    a.add(base, "base|dof=fixed")

    # Two exposed, axially distinct journal bearings.
    lower_bearing = annulus(
        BEARING_OUTER_R,
        BEARING_BORE_R,
        LOWER_BEARING_H,
    ).moved(Location((CRANK_AXIS_X, CRANK_AXIS_Y, LOWER_BEARING_Z)))
    a.add(lower_bearing, "lower_bearing|dof=fixed|mount=base")

    upper_bearing = annulus(
        BEARING_OUTER_R,
        BEARING_BORE_R,
        UPPER_BEARING_H,
    ).moved(Location((CRANK_AXIS_X, CRANK_AXIS_Y, UPPER_BEARING_Z)))
    a.add(
        upper_bearing,
        "upper_bearing|dof=fixed|mount=base,lower_bearing",
    )

    # Only driven input.
    crankshaft = Cylinder(
        SHAFT_R,
        SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_AXIS_X, CRANK_AXIS_Y, SHAFT_Z)))
    a.add(
        crankshaft,
        "crankshaft|dof=spin|driver=True|spin_axis=z|"
        "mount=lower_bearing,upper_bearing",
    )

    # Crank disk with a shaft press-fit bore and a separate eccentric-pin bore.
    disk_blank = Cylinder(
        CRANK_DISK_R,
        CRANK_DISK_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    shaft_bore_tool = Cylinder(
        CRANK_DISK_SHAFT_BORE_R,
        CRANK_DISK_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))
    eccentric_bore_tool = Cylinder(
        CRANK_PIN_DISK_BORE_R,
        CRANK_DISK_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_PIN_X, CRANK_PIN_Y, -1.0)))
    crank_disk = (
        disk_blank - shaft_bore_tool - eccentric_bore_tool
    ).moved(Location((CRANK_AXIS_X, CRANK_AXIS_Y, CRANK_DISK_Z)))
    a.add(
        crank_disk,
        "crank_disk|dof=spin|spin_axis=z|mount=crankshaft",
    )

    # Dedicated eccentric pin; its lower segment is pressed into the disk.
    crank_pin = Cylinder(
        CRANK_PIN_R,
        CRANK_PIN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CRANK_PIN_X, CRANK_PIN_Y, CRANK_PIN_Z)))
    a.add(
        crank_pin,
        "crank_pin|dof=fixed|mount=crank_disk",
    )

    # Horizontal anti-rotation guide rails. Their inner faces clear the slider
    # by 0.05 mm per side while the slider rests on the base.
    left_guide_rail = Box(
        GUIDE_LENGTH,
        GUIDE_RAIL_W,
        GUIDE_RAIL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((
            GUIDE_CENTER_X,
            GUIDE_RAIL_CENTER_Y,
            BASE_TOP_Z,
        ))
    )
    a.add(
        left_guide_rail,
        "left_guide_rail|dof=fixed|mount=base",
    )

    right_guide_rail = Box(
        GUIDE_LENGTH,
        GUIDE_RAIL_W,
        GUIDE_RAIL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((
            GUIDE_CENTER_X,
            -GUIDE_RAIL_CENTER_Y,
            BASE_TOP_Z,
        ))
    )
    a.add(
        right_guide_rail,
        "right_guide_rail|dof=fixed|mount=base",
    )

    # Forked slider:
    # - lower cheek/carriage rests on the base,
    # - rod occupies the open axial gap,
    # - a rear post supports the upper wrist-pin cheek without crossing the rod.
    slider_lower = Box(
        SLIDER_LENGTH,
        SLIDER_WIDTH,
        SLIDER_LOWER_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    slider_post_x0 = ROD_END_R + 0.8
    slider_post_length = (
        SLIDER_LENGTH / 2.0 - slider_post_x0
    )
    slider_post = Box(
        slider_post_length,
        4.0,
        SLIDER_UPPER_TOP_Z - BASE_TOP_Z - 1.8,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((slider_post_x0, 0, 1.8)))

    upper_eye = Cylinder(
        ROD_END_R,
        SLIDER_UPPER_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, SLIDER_LOCAL_UPPER_Z)))

    upper_bridge = Box(
        slider_post_x0 + slider_post_length,
        3.5,
        SLIDER_UPPER_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, SLIDER_LOCAL_UPPER_Z)))

    slider_raw = slider_lower + slider_post + upper_eye + upper_bridge

    lower_wrist_bore_tool = Cylinder(
        WRIST_PIN_BORE_R,
        SLIDER_LOWER_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))

    upper_wrist_bore_tool = Cylinder(
        WRIST_PIN_BORE_R,
        SLIDER_UPPER_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, SLIDER_LOCAL_UPPER_Z - 1.0)))

    slider_local = (
        slider_raw
        - lower_wrist_bore_tool
        - upper_wrist_bore_tool
    )
    slider = slider_local.moved(
        Location((SLIDER_WRIST_X, SLIDER_WRIST_Y, BASE_TOP_Z))
    )
    a.add(
        slider,
        "slider|dof=slide|slide_axis=x|"
        "mount=base,left_guide_rail,right_guide_rail",
    )

    # Wrist pin is pressed into both exposed slider cheeks.
    wrist_pin = Cylinder(
        WRIST_PIN_R,
        WRIST_PIN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((SLIDER_WRIST_X, SLIDER_WRIST_Y, WRIST_PIN_Z))
    )
    a.add(
        wrist_pin,
        "wrist_pin|dof=fixed|mount=slider",
    )

    # Rigid connecting rod in its own local frame:
    # big-end center at x=0, small-end center at x=ROD_LENGTH.
    # Both bores are running fits and are overshot through the full eye depth.
    rod_big_eye = Cylinder(
        ROD_END_R,
        ROD_T,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    rod_small_eye = Cylinder(
        ROD_END_R,
        ROD_T,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((ROD_LENGTH, 0, 0)))

    rod_shank = Box(
        ROD_LENGTH,
        2.0 * ROD_HALF_WIDTH,
        ROD_T,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    rod_raw = rod_big_eye + rod_shank + rod_small_eye

    big_end_bore_tool = Cylinder(
        ROD_BORE_R,
        ROD_T + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))

    small_end_bore_tool = Cylinder(
        ROD_BORE_R,
        ROD_T + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((ROD_LENGTH, 0, -1.0)))

    connecting_rod_local = (
        rod_raw - big_end_bore_tool - small_end_bore_tool
    )
    connecting_rod = connecting_rod_local.moved(
        Location(
            (CRANK_PIN_X, CRANK_PIN_Y, ROD_Z),
            (0, 0, ROD_ANGLE_DEG),
        )
    )
    a.add(
        connecting_rod,
        "connecting_rod|dof=free|mount=crank_pin,wrist_pin",
    )

    return a.build()