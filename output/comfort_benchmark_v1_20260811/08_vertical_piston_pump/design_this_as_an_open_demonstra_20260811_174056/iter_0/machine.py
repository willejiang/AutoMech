import math

# ---------------------------------------------------------------------------
# KINEMATIC ARITHMETIC FIRST
# ---------------------------------------------------------------------------

SHAFT_AXIS = (0.0, 1.0, 0.0)
SLIDE_AXIS = (0.0, 0.0, 1.0)

BASE_L = 62.0
BASE_W = 76.0
BASE_H = 4.0

SHAFT_R = 3.0
SHAFT_BORE_RUNNING_R = SHAFT_R + 0.05
SHAFT_BORE_PRESS_R = SHAFT_R - 0.005
SHAFT_Z = 30.0
SHAFT_Y0 = -32.0
SHAFT_Y1 = 28.0
SHAFT_LENGTH = SHAFT_Y1 - SHAFT_Y0

CRANK_RADIUS = 10.0
CRANK_PIN_R = 2.0
CRANK_PIN_RUNNING_BORE_R = CRANK_PIN_R + 0.05
CRANK_PIN_PRESS_BORE_R = CRANK_PIN_R - 0.005

# Initial world-space hardpoints.
CRANK_CENTER_W = (0.0, 25.0, SHAFT_Z)
CRANK_PIN_W = (
    CRANK_CENTER_W[0] + CRANK_RADIUS,
    CRANK_CENTER_W[1],
    CRANK_CENTER_W[2],
)
WRIST_PIN_W = (0.0, 25.0, 56.0)

ROD_DX = WRIST_PIN_W[0] - CRANK_PIN_W[0]
ROD_DZ = WRIST_PIN_W[2] - CRANK_PIN_W[2]
ROD_LENGTH = math.sqrt(ROD_DX**2 + ROD_DZ**2)
ROD_ROT_Y_DEG = math.degrees(math.atan2(ROD_DX, ROD_DZ))

# Exact slider travel limits for the upper branch of the in-line crank-slider.
WRIST_Z_MIN = SHAFT_Z - CRANK_RADIUS + ROD_LENGTH
WRIST_Z_MAX = SHAFT_Z + CRANK_RADIUS + ROD_LENGTH
SLIDER_STROKE = WRIST_Z_MAX - WRIST_Z_MIN

ROD_END_OUTER_R = 5.2
ROD_BAR_HALF_W = 2.8
ROD_THICKNESS = 5.5

WRIST_PIN_R = 2.0
WRIST_PIN_RUNNING_BORE_R = WRIST_PIN_R + 0.05
WRIST_PIN_PRESS_BORE_R = WRIST_PIN_R - 0.005
WRIST_PIN_LENGTH = 14.0

CROSSHEAD_W = 18.0
CROSSHEAD_D = 16.0
CROSSHEAD_MAIN_H = 10.0
CROSSHEAD_BASE_Z = WRIST_PIN_W[2] - CROSSHEAD_MAIN_H / 2.0
CROSSHEAD_POCKET_W = 2.0 * (ROD_END_OUTER_R + 0.20)
CROSSHEAD_POCKET_D = ROD_THICKNESS + 0.40
CROSSHEAD_BRIDGE_GAP = 0.40
CROSSHEAD_BRIDGE_Z0 = CROSSHEAD_MAIN_H + CROSSHEAD_BRIDGE_GAP
CROSSHEAD_BRIDGE_H = 3.0
CROSSHEAD_TOP_Z_LOCAL = CROSSHEAD_BRIDGE_Z0 + CROSSHEAD_BRIDGE_H

GUIDE_CLEARANCE = 0.05
GUIDE_INNER_X = CROSSHEAD_W / 2.0 + GUIDE_CLEARANCE
GUIDE_THICKNESS = 4.0
GUIDE_DEPTH = 4.0
GUIDE_Y = 32.5
GUIDE_Z0 = BASE_H
GUIDE_Z1 = 78.0
GUIDE_H = GUIDE_Z1 - GUIDE_Z0

PUMP_ROD_R = 1.5
PUMP_ROD_PRESS_BORE_R = PUMP_ROD_R - 0.005
PUMP_ROD_Z0 = CROSSHEAD_BASE_Z + CROSSHEAD_BRIDGE_Z0 + 1.0
PUMP_ROD_H = 28.0
PUMP_ROD_Z1 = PUMP_ROD_Z0 + PUMP_ROD_H

PISTON_R = 6.0
PISTON_H = 4.0
PISTON_Z0 = PUMP_ROD_Z1 - 2.0

PISTON_Z0_MIN = PISTON_Z0 + (WRIST_Z_MIN - WRIST_PIN_W[2])
PISTON_Z1_MAX = PISTON_Z0 + PISTON_H + (
    WRIST_Z_MAX - WRIST_PIN_W[2]
)

FRAME_INNER_R = PISTON_R + 1.0
FRAME_OUTER_R = 11.0
FRAME_LOWER_Z = GUIDE_Z1
FRAME_RING_H = 2.0
FRAME_POST_Z0 = FRAME_LOWER_Z + FRAME_RING_H
FRAME_UPPER_Z = max(104.0, PISTON_Z1_MAX + 3.0)
FRAME_POST_H = FRAME_UPPER_Z - FRAME_POST_Z0
FRAME_POST_R = 1.5
FRAME_POST_X = 9.0

BEARING_OUTER_R = 5.5
BEARING_LENGTH = 6.0
REAR_BEARING_Y0 = -22.0
FRONT_BEARING_Y0 = 12.0
PEDESTAL_TOP_Z = SHAFT_Z - BEARING_OUTER_R
PEDESTAL_Z0 = BASE_H
PEDESTAL_H = PEDESTAL_TOP_Z - PEDESTAL_Z0

CRANK_WEB_THICKNESS = 4.0
CRANK_WEB_Y0 = 18.0
CRANK_WEB_END_R = 4.5

CRANK_PIN_Y0 = 20.0
CRANK_PIN_LENGTH = 8.0
CRANK_PIN_CENTER_Y = CRANK_PIN_Y0 + CRANK_PIN_LENGTH / 2.0

WRIST_PIN_Y0 = 18.0
WRIST_PIN_CENTER_Y = WRIST_PIN_Y0 + WRIST_PIN_LENGTH / 2.0

HAND_ARM_THICKNESS = 4.0
HAND_ARM_Y0 = SHAFT_Y0
HAND_THROW = 14.0
HAND_ARM_END_R = 4.5
HANDLE_PIN_R = 2.0
HANDLE_PIN_Y0 = -40.0
HANDLE_PIN_LENGTH = 12.0
HANDLE_GRIP_OUTER_R = 3.2
HANDLE_GRIP_BORE_R = HANDLE_PIN_R + 0.05
HANDLE_GRIP_LENGTH = 8.0
HANDLE_GRIP_Y0 = -40.0


# ---------------------------------------------------------------------------
# MECHANISM SEMANTICS
# All port coordinates and axes are in the named part's LOCAL frame.
# ---------------------------------------------------------------------------

MECHANISM = {
    "name": "open_vertical_reciprocating_piston_pump",
    "output_link": "pump_piston",
    "watch_links": [
        "crankshaft",
        "crank_web",
        "connecting_rod",
        "crosshead",
        "pump_rod",
        "pump_piston",
    ],
    "ports_by_link": {
        "rear_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_BORE_RUNNING_R,
                "depth_mm": BEARING_LENGTH,
            }
        ],
        "front_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_BORE_RUNNING_R,
                "depth_mm": BEARING_LENGTH,
            }
        ],
        "crankshaft": [
            {
                "name": "rear_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    REAR_BEARING_Y0 + BEARING_LENGTH / 2.0 - SHAFT_Y0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_LENGTH,
            },
            {
                "name": "front_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    FRONT_BEARING_Y0 + BEARING_LENGTH / 2.0 - SHAFT_Y0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_LENGTH,
            },
            {
                "name": "crank_web_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    CRANK_WEB_Y0 + CRANK_WEB_THICKNESS / 2.0 - SHAFT_Y0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANK_WEB_THICKNESS,
            },
            {
                "name": "hand_arm_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, HAND_ARM_THICKNESS / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": HAND_ARM_THICKNESS,
            },
        ],
        "crank_web": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CRANK_WEB_THICKNESS / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_BORE_PRESS_R,
                "depth_mm": CRANK_WEB_THICKNESS,
            },
            {
                "name": "eccentric_bore",
                "type": "bore",
                "xyz_mm": [
                    CRANK_RADIUS,
                    0.0,
                    CRANK_WEB_THICKNESS / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_PRESS_BORE_R,
                "depth_mm": CRANK_WEB_THICKNESS,
            },
        ],
        "crank_pin": [
            {
                "name": "web_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 1.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": 2.0,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    CRANK_PIN_CENTER_Y - CRANK_PIN_Y0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": ROD_THICKNESS,
            },
        ],
        "connecting_rod": [
            {
                "name": "big_end",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * CRANK_PIN_RUNNING_BORE_R,
                "depth_mm": ROD_THICKNESS,
            },
            {
                "name": "small_end",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, ROD_LENGTH],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * WRIST_PIN_RUNNING_BORE_R,
                "depth_mm": ROD_THICKNESS,
            },
        ],
        "crosshead": [
            {
                "name": "wrist_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CROSSHEAD_MAIN_H / 2.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * WRIST_PIN_PRESS_BORE_R,
                "depth_mm": WRIST_PIN_LENGTH,
            },
            {
                "name": "pump_rod_socket",
                "type": "bore",
                "xyz_mm": [
                    0.0,
                    0.0,
                    CROSSHEAD_BRIDGE_Z0 + CROSSHEAD_BRIDGE_H / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_PRESS_BORE_R,
                "depth_mm": CROSSHEAD_BRIDGE_H,
            },
        ],
        "wrist_pin": [
            {
                "name": "crosshead_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, WRIST_PIN_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * WRIST_PIN_R,
                "depth_mm": WRIST_PIN_LENGTH,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    WRIST_PIN_CENTER_Y - WRIST_PIN_Y0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * WRIST_PIN_R,
                "depth_mm": ROD_THICKNESS,
            },
        ],
        "pump_rod": [
            {
                "name": "crosshead_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 1.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
                "depth_mm": 2.0,
            },
            {
                "name": "piston_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, PUMP_ROD_H - 1.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
                "depth_mm": 2.0,
            },
        ],
        "pump_piston": [
            {
                "name": "rod_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, PISTON_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_PRESS_BORE_R,
                "depth_mm": PISTON_H,
            }
        ],
        "hand_crank_arm": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, HAND_ARM_THICKNESS / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_BORE_PRESS_R,
                "depth_mm": HAND_ARM_THICKNESS,
            },
            {
                "name": "handle_bore",
                "type": "bore",
                "xyz_mm": [
                    -HAND_THROW,
                    0.0,
                    HAND_ARM_THICKNESS / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (HANDLE_PIN_R - 0.005),
                "depth_mm": HAND_ARM_THICKNESS,
            },
        ],
        "handle_pin": [
            {
                "name": "arm_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, HANDLE_PIN_LENGTH - 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * HANDLE_PIN_R,
                "depth_mm": 4.0,
            },
            {
                "name": "grip_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, HANDLE_GRIP_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * HANDLE_PIN_R,
                "depth_mm": HANDLE_GRIP_LENGTH,
            },
        ],
        "hand_grip": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, HANDLE_GRIP_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * HANDLE_GRIP_BORE_R,
                "depth_mm": HANDLE_GRIP_LENGTH,
            }
        ],
    },
    "relations": [
        {
            "name": "rear_shaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "rear_bearing",
            "base_port": "journal",
            "incoming_part": "crankshaft",
            "incoming_port": "rear_journal",
            "separation_axis": "+y",
        },
        {
            "name": "front_shaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "front_bearing",
            "base_port": "journal",
            "incoming_part": "crankshaft",
            "incoming_port": "front_journal",
            "separation_axis": "+y",
        },
        {
            "name": "shaft_to_crank_web",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "crank_web_seat",
            "incoming_part": "crank_web",
            "incoming_port": "shaft_bore",
            "separation_axis": "+y",
        },
        {
            "name": "web_to_eccentric_pin",
            "mate_type": "press_fit",
            "base_part": "crank_web",
            "base_port": "eccentric_bore",
            "incoming_part": "crank_pin",
            "incoming_port": "web_seat",
            "separation_axis": "+y",
        },
        {
            "name": "crank_pin_big_end",
            "mate_type": "revolute",
            "base_part": "crank_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "big_end",
            "separation_axis": "+y",
        },
        {
            "name": "crosshead_to_wrist_pin",
            "mate_type": "press_fit",
            "base_part": "crosshead",
            "base_port": "wrist_bore",
            "incoming_part": "wrist_pin",
            "incoming_port": "crosshead_seat",
            "separation_axis": "+y",
        },
        {
            "name": "wrist_pin_small_end",
            "mate_type": "revolute",
            "base_part": "wrist_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "small_end",
            "separation_axis": "+y",
        },
        {
            "name": "crosshead_carries_pump_rod",
            "mate_type": "press_fit",
            "base_part": "crosshead",
            "base_port": "pump_rod_socket",
            "incoming_part": "pump_rod",
            "incoming_port": "crosshead_seat",
            "separation_axis": "+z",
        },
        {
            "name": "pump_rod_carries_piston",
            "mate_type": "press_fit",
            "base_part": "pump_rod",
            "base_port": "piston_seat",
            "incoming_part": "pump_piston",
            "incoming_port": "rod_bore",
            "separation_axis": "+z",
        },
        {
            "name": "shaft_to_hand_arm",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "hand_arm_seat",
            "incoming_part": "hand_crank_arm",
            "incoming_port": "shaft_bore",
            "separation_axis": "-y",
        },
        {
            "name": "arm_to_handle_pin",
            "mate_type": "press_fit",
            "base_part": "hand_crank_arm",
            "base_port": "handle_bore",
            "incoming_part": "handle_pin",
            "incoming_port": "arm_seat",
            "separation_axis": "-y",
        },
        {
            "name": "handle_pin_to_grip",
            "mate_type": "journal_bearing",
            "base_part": "handle_pin",
            "base_port": "grip_journal",
            "incoming_part": "hand_grip",
            "incoming_port": "pin_bore",
            "separation_axis": "-y",
        },
    ],
    "motion_joints": [
        {
            "name": "crankshaft_revolute",
            "parent": "",
            "child": "crankshaft",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [0.0, 0.0, SHAFT_Z],
        },
        {
            "name": "crosshead_vertical_slide",
            "parent": "",
            "child": "crosshead",
            "type": "slide",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, CROSSHEAD_MAIN_H / 2.0],
        },
    ],
    "transmissions": [
        {
            "name": "shaft_rigidly_carries_crank_web",
            "type": "compound_1to1",
            "driving_link": "crankshaft",
            "driven_link": "crank_web",
            "ratio": 1.0,
        },
        {
            "name": "crank_web_rigidly_carries_eccentric_pin",
            "type": "compound_1to1",
            "driving_link": "crank_web",
            "driven_link": "crank_pin",
            "ratio": 1.0,
        },
        {
            "name": "shaft_rigidly_carries_hand_arm",
            "type": "compound_1to1",
            "driving_link": "crankshaft",
            "driven_link": "hand_crank_arm",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("open_vertical_reciprocating_piston_pump")

    def annulus(outer_r, inner_r, height):
        with BuildPart() as bp:
            with BuildSketch():
                Circle(outer_r)
                Circle(inner_r, mode=Mode.SUBTRACT)
            extrude(amount=height)
        return bp.part

    def two_end_link(
        center_distance,
        end_outer_r,
        bar_half_w,
        thickness,
        bore0_r,
        bore1_r,
    ):
        # Local pin centers are (0,0,0) and (0,0,center_distance).
        # Plane.XZ makes both bores run along local Y.
        with BuildPart() as bp:
            with BuildSketch(Plane.XZ):
                Circle(end_outer_r)
                with b3d.Locations((0.0, center_distance)):
                    Circle(end_outer_r)
                Polygon(
                    (-bar_half_w, 0.0),
                    (bar_half_w, 0.0),
                    (bar_half_w, center_distance),
                    (-bar_half_w, center_distance),
                )
                Circle(bore0_r, mode=Mode.SUBTRACT)
                with b3d.Locations((0.0, center_distance)):
                    Circle(bore1_r, mode=Mode.SUBTRACT)
            extrude(amount=thickness / 2.0, both=True)
        return bp.part

    def crank_arm_profile(
        throw,
        end_r,
        thickness,
        center_bore_r,
        end_bore_r,
        direction=1.0,
    ):
        end_x = direction * throw
        x0 = min(0.0, end_x)
        x1 = max(0.0, end_x)
        with BuildPart() as bp:
            with BuildSketch():
                Circle(end_r)
                with b3d.Locations((end_x, 0.0)):
                    Circle(end_r)
                Polygon(
                    (x0, -end_r),
                    (x1, -end_r),
                    (x1, end_r),
                    (x0, end_r),
                )
                Circle(center_bore_r, mode=Mode.SUBTRACT)
                with b3d.Locations((end_x, 0.0)):
                    Circle(end_bore_r, mode=Mode.SUBTRACT)
            extrude(amount=thickness)
        return bp.part

    # Bench base: its lower face is exactly on the ground plane.
    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(base, "bench_base|dof=fixed")

    # Bearing pedestals touch the base at z=BASE_H and the bearings at their
    # exact lower tangent z.
    rear_pedestal = Box(
        14.0,
        10.0,
        PEDESTAL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, REAR_BEARING_Y0 + BEARING_LENGTH / 2.0, PEDESTAL_Z0)))
    a.add(rear_pedestal, "rear_pedestal|dof=fixed|mount=bench_base")

    front_pedestal = Box(
        14.0,
        10.0,
        PEDESTAL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, FRONT_BEARING_Y0 + BEARING_LENGTH / 2.0, PEDESTAL_Z0)))
    a.add(front_pedestal, "front_pedestal|dof=fixed|mount=bench_base")

    rear_bearing = annulus(
        BEARING_OUTER_R,
        SHAFT_BORE_RUNNING_R,
        BEARING_LENGTH,
    ).moved(
        Location(
            (0.0, REAR_BEARING_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(rear_bearing, "rear_bearing|dof=fixed|mount=rear_pedestal")

    front_bearing = annulus(
        BEARING_OUTER_R,
        SHAFT_BORE_RUNNING_R,
        BEARING_LENGTH,
    ).moved(
        Location(
            (0.0, FRONT_BEARING_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(front_bearing, "front_bearing|dof=fixed|mount=front_pedestal")

    crankshaft = Cylinder(
        SHAFT_R,
        SHAFT_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (0.0, SHAFT_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        crankshaft,
        "crankshaft|dof=spin|driver=True|spin_axis=z"
        "|mount=rear_bearing,front_bearing",
    )

    # Exposed output-side crank web. It is a slender link, not a camera-
    # obstructing disk.
    crank_web = crank_arm_profile(
        CRANK_RADIUS,
        CRANK_WEB_END_R,
        CRANK_WEB_THICKNESS,
        SHAFT_BORE_PRESS_R,
        CRANK_PIN_PRESS_BORE_R,
        direction=1.0,
    ).moved(
        Location(
            (0.0, CRANK_WEB_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(crank_web, "crank_web|dof=fixed|mount=crankshaft")

    crank_pin = Cylinder(
        CRANK_PIN_R,
        CRANK_PIN_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (CRANK_PIN_W[0], CRANK_PIN_Y0, CRANK_PIN_W[2]),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(crank_pin, "crank_pin|dof=fixed|mount=crank_web")

    # Closure-controlled rod. Its local +Z runs from crank pin to wrist pin;
    # local +Y is the pin axis.
    connecting_rod = two_end_link(
        ROD_LENGTH,
        ROD_END_OUTER_R,
        ROD_BAR_HALF_W,
        ROD_THICKNESS,
        CRANK_PIN_RUNNING_BORE_R,
        WRIST_PIN_RUNNING_BORE_R,
    ).moved(
        Location(
            CRANK_PIN_W,
            (0.0, ROD_ROT_Y_DEG, 0.0),
        )
    )
    a.add(
        connecting_rod,
        "connecting_rod|dof=free|mount=crank_pin,wrist_pin",
    )

    # Crosshead: front/rear wrist-pin cheeks, side rails, a rod-motion pocket,
    # and a raised pump-rod bridge. The central pocket is derived from the
    # actual rod eye and rod thickness.
    main_block = Box(
        CROSSHEAD_W,
        CROSSHEAD_D,
        CROSSHEAD_MAIN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    left_extension = Box(
        (CROSSHEAD_W - CROSSHEAD_POCKET_W) / 2.0,
        CROSSHEAD_D,
        CROSSHEAD_BRIDGE_Z0 + CROSSHEAD_BRIDGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                -(CROSSHEAD_W + CROSSHEAD_POCKET_W) / 4.0,
                0.0,
                0.0,
            )
        )
    )

    right_extension = Box(
        (CROSSHEAD_W - CROSSHEAD_POCKET_W) / 2.0,
        CROSSHEAD_D,
        CROSSHEAD_BRIDGE_Z0 + CROSSHEAD_BRIDGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                (CROSSHEAD_W + CROSSHEAD_POCKET_W) / 4.0,
                0.0,
                0.0,
            )
        )
    )

    top_bridge = Box(
        CROSSHEAD_W,
        CROSSHEAD_D,
        CROSSHEAD_BRIDGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, CROSSHEAD_BRIDGE_Z0)))

    crosshead = main_block + left_extension + right_extension + top_bridge

    rod_pocket = Box(
        CROSSHEAD_POCKET_W,
        CROSSHEAD_POCKET_D,
        CROSSHEAD_MAIN_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))
    crosshead = crosshead - rod_pocket

    wrist_bore_tool = Cylinder(
        WRIST_PIN_PRESS_BORE_R,
        CROSSHEAD_D + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (0.0, -CROSSHEAD_D / 2.0 - 1.0, CROSSHEAD_MAIN_H / 2.0),
            (-90.0, 0.0, 0.0),
        )
    )
    crosshead = crosshead - wrist_bore_tool

    pump_socket_tool = Cylinder(
        PUMP_ROD_PRESS_BORE_R,
        CROSSHEAD_BRIDGE_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (0.0, 0.0, CROSSHEAD_BRIDGE_Z0 - 1.0)
        )
    )
    crosshead = crosshead - pump_socket_tool

    crosshead = crosshead.moved(
        Location((WRIST_PIN_W[0], WRIST_PIN_W[1], CROSSHEAD_BASE_Z))
    )
    a.add(
        crosshead,
        "crosshead|dof=slide|slide_axis=z"
        "|mount=left_guide_rail,right_guide_rail",
    )

    wrist_pin = Cylinder(
        WRIST_PIN_R,
        WRIST_PIN_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (WRIST_PIN_W[0], WRIST_PIN_Y0, WRIST_PIN_W[2]),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(wrist_pin, "wrist_pin|dof=fixed|mount=crosshead")

    # The two exposed vertical rails extend from the base to the lower pump
    # frame. Their front-side placement clears the crank web, crank pin, and
    # full connecting-rod sweep.
    left_guide = Box(
        GUIDE_THICKNESS,
        GUIDE_DEPTH,
        GUIDE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                -GUIDE_INNER_X - GUIDE_THICKNESS / 2.0,
                GUIDE_Y,
                GUIDE_Z0,
            )
        )
    )
    a.add(left_guide, "left_guide_rail|dof=fixed|mount=bench_base")

    right_guide = Box(
        GUIDE_THICKNESS,
        GUIDE_DEPTH,
        GUIDE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                GUIDE_INNER_X + GUIDE_THICKNESS / 2.0,
                GUIDE_Y,
                GUIDE_Z0,
            )
        )
    )
    a.add(right_guide, "right_guide_rail|dof=fixed|mount=bench_base")

    pump_rod = Cylinder(
        PUMP_ROD_R,
        PUMP_ROD_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 25.0, PUMP_ROD_Z0)))
    a.add(pump_rod, "pump_rod|dof=fixed|mount=crosshead")

    pump_piston = annulus(
        PISTON_R,
        PUMP_ROD_PRESS_BORE_R,
        PISTON_H,
    ).moved(Location((0.0, 25.0, PISTON_Z0)))
    a.add(pump_piston, "pump_piston|dof=fixed|mount=pump_rod")

    # Open cylinder frame: two annular end hoops and two slender posts. The
    # inner radius clears the piston throughout its complete vertical stroke.
    lower_frame = annulus(
        FRAME_OUTER_R,
        FRAME_INNER_R,
        FRAME_RING_H,
    ).moved(Location((0.0, 25.0, FRAME_LOWER_Z)))
    a.add(
        lower_frame,
        "cylinder_lower_frame|dof=fixed"
        "|mount=left_guide_rail,right_guide_rail",
    )

    left_frame_post = Cylinder(
        FRAME_POST_R,
        FRAME_POST_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((-FRAME_POST_X, 25.0, FRAME_POST_Z0)))
    a.add(
        left_frame_post,
        "cylinder_left_post|dof=fixed|mount=cylinder_lower_frame",
    )

    right_frame_post = Cylinder(
        FRAME_POST_R,
        FRAME_POST_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((FRAME_POST_X, 25.0, FRAME_POST_Z0)))
    a.add(
        right_frame_post,
        "cylinder_right_post|dof=fixed|mount=cylinder_lower_frame",
    )

    upper_frame = annulus(
        FRAME_OUTER_R,
        FRAME_INNER_R,
        FRAME_RING_H,
    ).moved(Location((0.0, 25.0, FRAME_UPPER_Z)))
    a.add(
        upper_frame,
        "cylinder_upper_frame|dof=fixed"
        "|mount=cylinder_left_post,cylinder_right_post",
    )

    # Rear hand crank. The arm is rigidly press-fit to the only driven shaft.
    hand_arm = crank_arm_profile(
        HAND_THROW,
        HAND_ARM_END_R,
        HAND_ARM_THICKNESS,
        SHAFT_BORE_PRESS_R,
        HANDLE_PIN_R - 0.005,
        direction=-1.0,
    ).moved(
        Location(
            (0.0, HAND_ARM_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(hand_arm, "hand_crank_arm|dof=fixed|mount=crankshaft")

    handle_pin = Cylinder(
        HANDLE_PIN_R,
        HANDLE_PIN_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (-HAND_THROW, HANDLE_PIN_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(handle_pin, "handle_pin|dof=fixed|mount=hand_crank_arm")

    hand_grip = annulus(
        HANDLE_GRIP_OUTER_R,
        HANDLE_GRIP_BORE_R,
        HANDLE_GRIP_LENGTH,
    ).moved(
        Location(
            (-HAND_THROW, HANDLE_GRIP_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(hand_grip, "hand_grip|dof=fixed|mount=handle_pin")

    return a.build()