import math


# ---------------------------------------------------------------------------
# CONTROLLING ARITHMETIC — solve all mechanism hardpoints before geometry
# ---------------------------------------------------------------------------

SHAFT_AXIS_X = 0.0
SHAFT_AXIS_Y = 0.0
SHAFT_AXIS_Z = 110.0

SHAFT_R = 5.0
BEARING_RUNNING_CLEARANCE = 0.05
BEARING_BORE_R = SHAFT_R + BEARING_RUNNING_CLEARANCE
BEARING_OUTER_R = 10.0
BEARING_WIDTH = 8.0

CRANK_RADIUS = 18.0
CONNECTING_ROD_LENGTH = 72.0
CRANK_ANGLE_RAD = 0.0

CRANK_PIN_R = 3.0
WRIST_PIN_R = 3.0
PIN_RUNNING_CLEARANCE = 0.05
ROD_BIG_BORE_R = CRANK_PIN_R + PIN_RUNNING_CLEARANCE
ROD_SMALL_BORE_R = WRIST_PIN_R + PIN_RUNNING_CLEARANCE
SLIDER_PIN_PRESS_BORE_R = WRIST_PIN_R - 0.005

CRANK_PIN_WORLD_X = SHAFT_AXIS_X + CRANK_RADIUS * math.cos(CRANK_ANGLE_RAD)
CRANK_PIN_WORLD_Z = SHAFT_AXIS_Z + CRANK_RADIUS * math.sin(CRANK_ANGLE_RAD)

SLIDER_AXIS_X = SHAFT_AXIS_X
ROD_HORIZONTAL_OFFSET = CRANK_PIN_WORLD_X - SLIDER_AXIS_X
ROD_VERTICAL_PROJECTION = math.sqrt(
    CONNECTING_ROD_LENGTH**2 - ROD_HORIZONTAL_OFFSET**2
)
WRIST_PIN_WORLD_X = SLIDER_AXIS_X
WRIST_PIN_WORLD_Z = CRANK_PIN_WORLD_Z - ROD_VERTICAL_PROJECTION

CRANK_WEB_Y0 = 34.0
CRANK_WEB_Y1 = 41.0
ROD_CENTER_Y = 45.0
ROD_THICKNESS = 7.5
ROD_Y0 = ROD_CENTER_Y - ROD_THICKNESS / 2.0
ROD_Y1 = ROD_CENTER_Y + ROD_THICKNESS / 2.0

WRIST_PIN_Y0 = 35.0
WRIST_PIN_Y1 = 55.0
WRIST_PIN_LENGTH = WRIST_PIN_Y1 - WRIST_PIN_Y0
WRIST_PIN_CENTER_Y = (WRIST_PIN_Y0 + WRIST_PIN_Y1) / 2.0

CRANK_PIN_Y0 = ROD_Y0 - 0.75
CRANK_PIN_Y1 = ROD_Y1 + 0.75
CRANK_PIN_LENGTH = CRANK_PIN_Y1 - CRANK_PIN_Y0
CRANK_PIN_CENTER_Y = (CRANK_PIN_Y0 + CRANK_PIN_Y1) / 2.0

assert abs(CRANK_PIN_CENTER_Y - ROD_CENTER_Y) < 1.0e-9
assert abs(WRIST_PIN_CENTER_Y - ROD_CENTER_Y) < 1.0e-9
assert abs(
    math.hypot(
        CRANK_PIN_WORLD_X - WRIST_PIN_WORLD_X,
        CRANK_PIN_WORLD_Z - WRIST_PIN_WORLD_Z,
    )
    - CONNECTING_ROD_LENGTH
) < 1.0e-9

# Main shaft runs from the rotor nose to the rear face of the crank web.
MAIN_SHAFT_Y0 = -65.0
MAIN_SHAFT_Y1 = CRANK_WEB_Y1
MAIN_SHAFT_LENGTH = MAIN_SHAFT_Y1 - MAIN_SHAFT_Y0

FRONT_BEARING_Y = -28.0
REAR_BEARING_Y = 18.0
FRONT_UPRIGHT_Y = FRONT_BEARING_Y
REAR_UPRIGHT_Y = REAR_BEARING_Y

# Local +Z of all shaft-like parts is rotated onto global +Y.
SHAFT_FRAME_ROTATION = (-90.0, 0.0, 0.0)

# Rotor dimensions and press fit.
ROTOR_CENTER_Y = -55.0
ROTOR_THICKNESS = 3.0
ROTOR_HUB_R = 9.5
ROTOR_BORE_R = SHAFT_R - 0.005
ROTOR_BLADE_COUNT = 8
ROTOR_BLADE_INNER_R = 8.5
ROTOR_BLADE_OUTER_R = 48.0
ROTOR_BLADE_ROOT_HALF_W = 5.5
ROTOR_BLADE_TIP_HALF_W = 2.3

# Crank web directly joins the main shaft to the eccentric pin.
CRANK_WEB_R = CRANK_RADIUS + CRANK_PIN_R + 5.0
CRANK_WEB_THICKNESS = CRANK_WEB_Y1 - CRANK_WEB_Y0

# Rod eye and shank dimensions.
ROD_EYE_OUTER_R = 7.0
ROD_SHANK_HALF_W = 4.0
ROD_LOCAL_SMALL_X = WRIST_PIN_WORLD_X - CRANK_PIN_WORLD_X
ROD_LOCAL_SMALL_Z = WRIST_PIN_WORLD_Z - CRANK_PIN_WORLD_Z
ROD_UNIT_X = ROD_LOCAL_SMALL_X / CONNECTING_ROD_LENGTH
ROD_UNIT_Z = ROD_LOCAL_SMALL_Z / CONNECTING_ROD_LENGTH
ROD_NORMAL_X = -ROD_UNIT_Z
ROD_NORMAL_Z = ROD_UNIT_X

# Slider cheeks are separated axially so the rod eye has working clearance.
SLIDER_HALF_X = 9.0
SLIDER_Y0 = 35.0
SLIDER_Y1 = 55.0
SLIDER_LEFT_CHEEK_Y0 = SLIDER_Y0
SLIDER_LEFT_CHEEK_Y1 = ROD_Y0 - 0.35
SLIDER_RIGHT_CHEEK_Y0 = ROD_Y1 + 0.35
SLIDER_RIGHT_CHEEK_Y1 = SLIDER_Y1
SLIDER_CHEEK_Z0_LOCAL = -14.0
SLIDER_CHEEK_Z1_LOCAL = 8.0
SLIDER_BRIDGE_Z0_LOCAL = -14.0
SLIDER_BRIDGE_Z1_LOCAL = -8.0

assert SLIDER_LEFT_CHEEK_Y1 < ROD_Y0
assert SLIDER_RIGHT_CHEEK_Y0 > ROD_Y1

# Rectangular guide gives a finite 0.15 mm running clearance per side.
SLIDER_GUIDE_CLEARANCE = 0.15
GUIDE_INNER_HALF_X = SLIDER_HALF_X + SLIDER_GUIDE_CLEARANCE
GUIDE_RAIL_THICKNESS = 4.0
GUIDE_OUTER_HALF_X = GUIDE_INNER_HALF_X + GUIDE_RAIL_THICKNESS
GUIDE_Y0 = 34.0
GUIDE_Y1 = 56.0
GUIDE_Z0 = 24.0
GUIDE_Z1 = 68.0
GUIDE_REAR_WALL_Y0 = 30.0
GUIDE_REAR_WALL_Y1 = GUIDE_Y0

# Pump rod and barrel dimensions.
BASE_Z0 = 0.0
BASE_THICKNESS = 4.0
BASE_Z1 = BASE_Z0 + BASE_THICKNESS

PUMP_ROD_R = 2.5
PUMP_ROD_PRESS_BORE_R = PUMP_ROD_R - 0.005
PUMP_ROD_Z0 = BASE_Z1
PUMP_ROD_Z1 = WRIST_PIN_WORLD_Z + SLIDER_BRIDGE_Z1_LOCAL
PUMP_ROD_LENGTH = PUMP_ROD_Z1 - PUMP_ROD_Z0

PUMP_BARREL_INNER_R = 6.0
PUMP_BARREL_OUTER_R = 8.5
PUMP_BARREL_Z0 = BASE_Z1
PUMP_BARREL_HEIGHT = 20.0
PUMP_BARREL_Z1 = PUMP_BARREL_Z0 + PUMP_BARREL_HEIGHT

PUMP_PISTON_R = PUMP_BARREL_INNER_R - 0.10
PUMP_PISTON_THICKNESS = 4.0
PUMP_PISTON_CENTER_Z = 9.0
PUMP_PISTON_BORE_R = PUMP_ROD_R - 0.005

assert PUMP_ROD_LENGTH > 0.0
assert PUMP_PISTON_R < PUMP_BARREL_INNER_R
assert PUMP_BARREL_OUTER_R < GUIDE_INNER_HALF_X

# Structural dimensions.
BASE_X = 80.0
BASE_Y = 120.0
BASE_CENTER_Y = -5.0

UPRIGHT_X = 38.0
UPRIGHT_Y = BEARING_WIDTH
UPRIGHT_Z0 = BASE_Z1
UPRIGHT_Z1 = 126.0
UPRIGHT_HEIGHT = UPRIGHT_Z1 - UPRIGHT_Z0
UPRIGHT_BEARING_SEAT_R = BEARING_OUTER_R + 0.01


MECHANISM = {
    "name": "windmill_direct_drive_reciprocating_water_pump",
    "output_link": "piston_slider",
    "watch_links": [
        "wind_rotor",
        "crankshaft",
        "connecting_rod",
        "piston_slider",
        "pump_rod",
        "pump_piston",
    ],
    "ports_by_link": {
        "wind_rotor": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * ROTOR_BORE_R,
                "depth_mm": ROTOR_THICKNESS,
            }
        ],
        "crankshaft": [
            {
                "name": "rotor_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, ROTOR_CENTER_Y - MAIN_SHAFT_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": ROTOR_THICKNESS,
            },
            {
                "name": "front_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, FRONT_BEARING_Y - MAIN_SHAFT_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_WIDTH,
            },
            {
                "name": "rear_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, REAR_BEARING_Y - MAIN_SHAFT_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_WIDTH,
            },
            {
                "name": "eccentric_pin",
                "type": "shaft",
                "xyz_mm": [
                    CRANK_RADIUS,
                    0.0,
                    CRANK_PIN_CENTER_Y - MAIN_SHAFT_Y0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": CRANK_PIN_LENGTH,
            },
        ],
        "front_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_WIDTH,
            }
        ],
        "rear_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_WIDTH,
            }
        ],
        "connecting_rod": [
            {
                "name": "big_end_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * ROD_BIG_BORE_R,
                "depth_mm": ROD_THICKNESS,
            },
            {
                "name": "small_end_bore",
                "type": "bore",
                "xyz_mm": [ROD_LOCAL_SMALL_X, 0.0, ROD_LOCAL_SMALL_Z],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * ROD_SMALL_BORE_R,
                "depth_mm": ROD_THICKNESS,
            },
        ],
        "wrist_pin": [
            {
                "name": "pin_shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, WRIST_PIN_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * WRIST_PIN_R,
                "depth_mm": WRIST_PIN_LENGTH,
            }
        ],
        "piston_slider": [
            {
                "name": "wrist_pin_seat",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * SLIDER_PIN_PRESS_BORE_R,
                "depth_mm": SLIDER_Y1 - SLIDER_Y0,
            },
            {
                "name": "pump_rod_seat",
                "type": "bore",
                "xyz_mm": [
                    0.0,
                    0.0,
                    (
                        SLIDER_BRIDGE_Z0_LOCAL
                        + SLIDER_BRIDGE_Z1_LOCAL
                    ) / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_PRESS_BORE_R,
                "depth_mm": (
                    SLIDER_BRIDGE_Z1_LOCAL
                    - SLIDER_BRIDGE_Z0_LOCAL
                ),
            },
        ],
        "pump_rod": [
            {
                "name": "upper_press_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, PUMP_ROD_LENGTH - 3.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
                "depth_mm": 6.0,
            },
            {
                "name": "piston_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, PUMP_PISTON_CENTER_Z - PUMP_ROD_Z0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
                "depth_mm": PUMP_PISTON_THICKNESS,
            },
        ],
        "pump_piston": [
            {
                "name": "rod_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_PISTON_BORE_R,
                "depth_mm": PUMP_PISTON_THICKNESS,
            }
        ],
    },
    "relations": [
        {
            "name": "rotor_press_fit",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "rotor_seat",
            "incoming_part": "wind_rotor",
            "incoming_port": "hub_bore",
        },
        {
            "name": "front_shaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "front_bearing",
            "base_port": "journal_bore",
            "incoming_part": "crankshaft",
            "incoming_port": "front_journal",
        },
        {
            "name": "rear_shaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "rear_bearing",
            "base_port": "journal_bore",
            "incoming_part": "crankshaft",
            "incoming_port": "rear_journal",
        },
        {
            "name": "crank_big_end_revolute",
            "mate_type": "revolute",
            "base_part": "crankshaft",
            "base_port": "eccentric_pin",
            "incoming_part": "connecting_rod",
            "incoming_port": "big_end_bore",
        },
        {
            "name": "rod_small_end_revolute",
            "mate_type": "revolute",
            "base_part": "wrist_pin",
            "base_port": "pin_shaft",
            "incoming_part": "connecting_rod",
            "incoming_port": "small_end_bore",
        },
        {
            "name": "wrist_pin_into_slider",
            "mate_type": "press_fit",
            "base_part": "piston_slider",
            "base_port": "wrist_pin_seat",
            "incoming_part": "wrist_pin",
            "incoming_port": "pin_shaft",
        },
        {
            "name": "pump_rod_into_slider",
            "mate_type": "press_fit",
            "base_part": "piston_slider",
            "base_port": "pump_rod_seat",
            "incoming_part": "pump_rod",
            "incoming_port": "upper_press_seat",
        },
        {
            "name": "pump_piston_on_rod",
            "mate_type": "press_fit",
            "base_part": "pump_rod",
            "base_port": "piston_seat",
            "incoming_part": "pump_piston",
            "incoming_port": "rod_bore",
        },
    ],
    "motion_joints": [
        {
            "name": "supported_crankshaft_rotation",
            "parent": "",
            "child": "crankshaft",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [SHAFT_AXIS_X, SHAFT_AXIS_Y, SHAFT_AXIS_Z],
        },
        {
            "name": "vertical_crosshead_guide",
            "parent": "",
            "child": "piston_slider",
            "type": "slide",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [WRIST_PIN_WORLD_X, WRIST_PIN_CENTER_Y, WRIST_PIN_WORLD_Z],
        },
    ],
    "transmissions": [
        {
            "name": "rotor_to_crankshaft_press_drive",
            "type": "compound_1to1",
            "driving_link": "wind_rotor",
            "driven_link": "crankshaft",
            "ratio": 1.0,
        }
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("windmill_reciprocating_water_pump")

    def annulus(outer_r, inner_r, height, align_min=False):
        if align_min:
            outer = Cylinder(
                outer_r,
                height,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
            cutter = Cylinder(
                inner_r,
                height + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Location((0.0, 0.0, -1.0)))
        else:
            outer = Cylinder(outer_r, height)
            cutter = Cylinder(inner_r, height + 2.0)
        return outer - cutter

    def shaft_axis_cutter(radius, length):
        return Cylinder(radius, length).moved(
            Location((0.0, 0.0, 0.0), SHAFT_FRAME_ROTATION)
        )

    # Grounded structural base.
    base = Box(
        BASE_X,
        BASE_Y,
        BASE_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, BASE_CENTER_Y, BASE_Z0)))
    a.add(base, "base|dof=fixed")

    # Bored bearing support uprights.
    upright_blank = Box(
        UPRIGHT_X,
        UPRIGHT_Y,
        UPRIGHT_HEIGHT,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    upright_hole = shaft_axis_cutter(
        UPRIGHT_BEARING_SEAT_R,
        UPRIGHT_Y + 2.0,
    ).moved(Location((0.0, 0.0, SHAFT_AXIS_Z - UPRIGHT_Z0)))
    upright_local = upright_blank - upright_hole

    front_upright = upright_local.moved(
        Location((0.0, FRONT_UPRIGHT_Y, UPRIGHT_Z0))
    )
    rear_upright = upright_local.moved(
        Location((0.0, REAR_UPRIGHT_Y, UPRIGHT_Z0))
    )
    a.add(front_upright, "front_upright|dof=fixed|mount=base")
    a.add(rear_upright, "rear_upright|dof=fixed|mount=base")

    # Separate journal-bearing sleeves with running-clearance bores.
    bearing_local = annulus(
        BEARING_OUTER_R,
        BEARING_BORE_R,
        BEARING_WIDTH,
    )
    front_bearing = bearing_local.moved(
        Location(
            (SHAFT_AXIS_X, FRONT_BEARING_Y, SHAFT_AXIS_Z),
            SHAFT_FRAME_ROTATION,
        )
    )
    rear_bearing = bearing_local.moved(
        Location(
            (SHAFT_AXIS_X, REAR_BEARING_Y, SHAFT_AXIS_Z),
            SHAFT_FRAME_ROTATION,
        )
    )
    a.add(
        front_bearing,
        "front_bearing|dof=fixed|mount=front_upright",
    )
    a.add(
        rear_bearing,
        "rear_bearing|dof=fixed|mount=rear_upright",
    )

    # Eight tapered blades overlap a bored central hub.
    rotor_hub = annulus(ROTOR_HUB_R, ROTOR_BORE_R, ROTOR_THICKNESS)

    with BuildPart() as blade_bp:
        with BuildSketch() as blade_sk:
            Polygon(
                (ROTOR_BLADE_INNER_R, -ROTOR_BLADE_ROOT_HALF_W),
                (ROTOR_BLADE_OUTER_R, -ROTOR_BLADE_TIP_HALF_W),
                (ROTOR_BLADE_OUTER_R, ROTOR_BLADE_TIP_HALF_W),
                (ROTOR_BLADE_INNER_R, ROTOR_BLADE_ROOT_HALF_W),
            )
        extrude(amount=ROTOR_THICKNESS / 2.0, both=True)
    blade_local = blade_bp.part

    rotor_local = rotor_hub
    for blade_index in range(ROTOR_BLADE_COUNT):
        blade_angle_deg = 360.0 * blade_index / ROTOR_BLADE_COUNT
        rotor_local = rotor_local + blade_local.moved(
            Location((0.0, 0.0, 0.0), (0.0, 0.0, blade_angle_deg))
        )

    wind_rotor = rotor_local.moved(
        Location(
            (SHAFT_AXIS_X, ROTOR_CENTER_Y, SHAFT_AXIS_Z),
            SHAFT_FRAME_ROTATION,
        )
    )
    a.add(
        wind_rotor,
        "wind_rotor|dof=spin|driver=True|spin_axis=z|mount=crankshaft",
    )

    # One rigid crankshaft: journal shaft, crank web, and eccentric pin.
    main_shaft_local = Cylinder(
        SHAFT_R,
        MAIN_SHAFT_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    crank_web_local = Cylinder(
        CRANK_WEB_R,
        CRANK_WEB_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, CRANK_WEB_Y0 - MAIN_SHAFT_Y0)))

    crank_pin_local = Cylinder(
        CRANK_PIN_R,
        CRANK_PIN_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                CRANK_RADIUS,
                0.0,
                CRANK_PIN_Y0 - MAIN_SHAFT_Y0,
            )
        )
    )

    crankshaft_local = main_shaft_local + crank_web_local + crank_pin_local
    crankshaft = crankshaft_local.moved(
        Location(
            (SHAFT_AXIS_X, MAIN_SHAFT_Y0, SHAFT_AXIS_Z),
            SHAFT_FRAME_ROTATION,
        )
    )
    a.add(
        crankshaft,
        "crankshaft|dof=spin|spin_axis=z|mount=front_bearing,rear_bearing",
    )

    # Rigid connecting rod built from the solved pin-to-pin vector.
    p0_plus = (
        ROD_NORMAL_X * ROD_SHANK_HALF_W,
        ROD_NORMAL_Z * ROD_SHANK_HALF_W,
    )
    p1_plus = (
        ROD_LOCAL_SMALL_X + ROD_NORMAL_X * ROD_SHANK_HALF_W,
        ROD_LOCAL_SMALL_Z + ROD_NORMAL_Z * ROD_SHANK_HALF_W,
    )
    p1_minus = (
        ROD_LOCAL_SMALL_X - ROD_NORMAL_X * ROD_SHANK_HALF_W,
        ROD_LOCAL_SMALL_Z - ROD_NORMAL_Z * ROD_SHANK_HALF_W,
    )
    p0_minus = (
        -ROD_NORMAL_X * ROD_SHANK_HALF_W,
        -ROD_NORMAL_Z * ROD_SHANK_HALF_W,
    )

    with BuildPart() as rod_bp:
        with BuildSketch(Plane.XZ) as rod_sk:
            Polygon(p0_plus, p1_plus, p1_minus, p0_minus)
            with b3d.Locations(
                (0.0, 0.0),
                (ROD_LOCAL_SMALL_X, ROD_LOCAL_SMALL_Z),
            ):
                Circle(ROD_EYE_OUTER_R)
            with b3d.Locations((0.0, 0.0)):
                Circle(ROD_BIG_BORE_R, mode=Mode.SUBTRACT)
            with b3d.Locations(
                (ROD_LOCAL_SMALL_X, ROD_LOCAL_SMALL_Z)
            ):
                Circle(ROD_SMALL_BORE_R, mode=Mode.SUBTRACT)
        extrude(amount=ROD_THICKNESS / 2.0, both=True)
    connecting_rod = rod_bp.part.moved(
        Location(
            (
                CRANK_PIN_WORLD_X,
                ROD_CENTER_Y,
                CRANK_PIN_WORLD_Z,
            )
        )
    )
    a.add(
        connecting_rod,
        "connecting_rod|dof=free|mount=crankshaft,wrist_pin",
    )

    # Slider cheeks lie outside the rod's axial envelope.
    left_cheek_depth = SLIDER_LEFT_CHEEK_Y1 - SLIDER_LEFT_CHEEK_Y0
    right_cheek_depth = SLIDER_RIGHT_CHEEK_Y1 - SLIDER_RIGHT_CHEEK_Y0

    left_cheek = Box(
        2.0 * SLIDER_HALF_X,
        left_cheek_depth,
        SLIDER_CHEEK_Z1_LOCAL - SLIDER_CHEEK_Z0_LOCAL,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                (
                    SLIDER_LEFT_CHEEK_Y0
                    + SLIDER_LEFT_CHEEK_Y1
                ) / 2.0
                - WRIST_PIN_CENTER_Y,
                SLIDER_CHEEK_Z0_LOCAL,
            )
        )
    )

    right_cheek = Box(
        2.0 * SLIDER_HALF_X,
        right_cheek_depth,
        SLIDER_CHEEK_Z1_LOCAL - SLIDER_CHEEK_Z0_LOCAL,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                (
                    SLIDER_RIGHT_CHEEK_Y0
                    + SLIDER_RIGHT_CHEEK_Y1
                ) / 2.0
                - WRIST_PIN_CENTER_Y,
                SLIDER_CHEEK_Z0_LOCAL,
            )
        )
    )

    lower_bridge = Box(
        2.0 * SLIDER_HALF_X,
        SLIDER_Y1 - SLIDER_Y0,
        SLIDER_BRIDGE_Z1_LOCAL - SLIDER_BRIDGE_Z0_LOCAL,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                (SLIDER_Y0 + SLIDER_Y1) / 2.0 - WRIST_PIN_CENTER_Y,
                SLIDER_BRIDGE_Z0_LOCAL,
            )
        )
    )

    slider_local = left_cheek + right_cheek + lower_bridge

    wrist_bore_tool = Cylinder(
        SLIDER_PIN_PRESS_BORE_R,
        WRIST_PIN_LENGTH + 2.0,
    ).moved(Location((0.0, 0.0, 0.0), SHAFT_FRAME_ROTATION))

    pump_rod_bore_tool = Cylinder(
        PUMP_ROD_PRESS_BORE_R,
        SLIDER_BRIDGE_Z1_LOCAL - SLIDER_BRIDGE_Z0_LOCAL + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location((0.0, 0.0, SLIDER_BRIDGE_Z0_LOCAL - 1.0))
    )

    slider_local = slider_local - wrist_bore_tool - pump_rod_bore_tool
    piston_slider = slider_local.moved(
        Location(
            (
                WRIST_PIN_WORLD_X,
                WRIST_PIN_CENTER_Y,
                WRIST_PIN_WORLD_Z,
            )
        )
    )
    a.add(
        piston_slider,
        "piston_slider|dof=slide|slide_axis=z|mount=guide_frame",
    )

    # Wrist pin spans both cheeks and the relieved rod eye.
    wrist_pin = Cylinder(
        WRIST_PIN_R,
        WRIST_PIN_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                WRIST_PIN_WORLD_X,
                WRIST_PIN_Y0,
                WRIST_PIN_WORLD_Z,
            ),
            SHAFT_FRAME_ROTATION,
        )
    )
    a.add(
        wrist_pin,
        "wrist_pin|dof=fixed|mount=piston_slider",
    )

    # Open guide: side rails constrain the slider; the rear wall stays clear
    # of the connecting-rod axial envelope.
    guide_height = GUIDE_Z1 - GUIDE_Z0
    guide_y_depth = GUIDE_Y1 - GUIDE_Y0

    left_rail = Box(
        GUIDE_RAIL_THICKNESS,
        guide_y_depth,
        guide_height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                -(GUIDE_INNER_HALF_X + GUIDE_RAIL_THICKNESS / 2.0),
                (GUIDE_Y0 + GUIDE_Y1) / 2.0,
                GUIDE_Z0,
            )
        )
    )

    right_rail = Box(
        GUIDE_RAIL_THICKNESS,
        guide_y_depth,
        guide_height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                GUIDE_INNER_HALF_X + GUIDE_RAIL_THICKNESS / 2.0,
                (GUIDE_Y0 + GUIDE_Y1) / 2.0,
                GUIDE_Z0,
            )
        )
    )

    guide_rear_wall = Box(
        2.0 * GUIDE_OUTER_HALF_X,
        GUIDE_REAR_WALL_Y1 - GUIDE_REAR_WALL_Y0,
        GUIDE_Z1 - BASE_Z1,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                (GUIDE_REAR_WALL_Y0 + GUIDE_REAR_WALL_Y1) / 2.0,
                BASE_Z1,
            )
        )
    )

    guide_frame = left_rail + right_rail + guide_rear_wall
    a.add(guide_frame, "guide_frame|dof=fixed|mount=base")

    # Open-top annular pump barrel.
    pump_barrel = annulus(
        PUMP_BARREL_OUTER_R,
        PUMP_BARREL_INNER_R,
        PUMP_BARREL_HEIGHT,
        align_min=True,
    ).moved(
        Location(
            (
                SLIDER_AXIS_X,
                WRIST_PIN_CENTER_Y,
                PUMP_BARREL_Z0,
            )
        )
    )
    a.add(pump_barrel, "pump_barrel|dof=fixed|mount=base")

    # Vertical pump rod enters a true press-fit bore in the lower bridge.
    pump_rod = Cylinder(
        PUMP_ROD_R,
        PUMP_ROD_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                SLIDER_AXIS_X,
                WRIST_PIN_CENTER_Y,
                PUMP_ROD_Z0,
            )
        )
    )
    a.add(
        pump_rod,
        "pump_rod|dof=slide|slide_axis=z|mount=piston_slider,pump_barrel",
    )

    # Hydraulic piston has 0.10 mm radial barrel clearance and a press bore.
    pump_piston_local = annulus(
        PUMP_PISTON_R,
        PUMP_PISTON_BORE_R,
        PUMP_PISTON_THICKNESS,
    )
    pump_piston = pump_piston_local.moved(
        Location(
            (
                SLIDER_AXIS_X,
                WRIST_PIN_CENTER_Y,
                PUMP_PISTON_CENTER_Z,
            )
        )
    )
    a.add(
        pump_piston,
        "pump_piston|dof=fixed|mount=pump_rod,pump_barrel",
    )

    return a.build()