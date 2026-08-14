import math
from build123d import Rotation

# ---------------------------------------------------------------------------
# ENGINEERING ARITHMETIC — solve the motion hardpoints and fits before geometry
# ---------------------------------------------------------------------------

SHAFT_AXIS_X = 0.0
SHAFT_AXIS_Y = 0.0
SHAFT_AXIS_Z = 70.0

SHAFT_R = 3.0
SHAFT_START_Y = -32.0
SHAFT_END_Y = 24.0
SHAFT_LENGTH = SHAFT_END_Y - SHAFT_START_Y

CRANK_RADIUS = 12.0
CRANK_ANGLE_RAD = 0.0
CRANK_PIN_WORLD_X = SHAFT_AXIS_X + CRANK_RADIUS * math.cos(CRANK_ANGLE_RAD)
CRANK_PIN_WORLD_Z = SHAFT_AXIS_Z + CRANK_RADIUS * math.sin(CRANK_ANGLE_RAD)

SLIDER_AXIS_X = 0.0
WRIST_PIN_WORLD_Z = 120.0
WRIST_PIN_WORLD_X = SLIDER_AXIS_X

ROD_DX = WRIST_PIN_WORLD_X - CRANK_PIN_WORLD_X
ROD_DZ = WRIST_PIN_WORLD_Z - CRANK_PIN_WORLD_Z
ROD_CENTER_DISTANCE = math.hypot(ROD_DX, ROD_DZ)
ROD_NORMAL_X = -ROD_DZ / ROD_CENTER_DISTANCE
ROD_NORMAL_Z = ROD_DX / ROD_CENTER_DISTANCE

PIN_R = 2.5
PIN_RUNNING_CLEARANCE = 0.05
PIN_BORE_R = PIN_R + PIN_RUNNING_CLEARANCE
PRESS_INTERFERENCE = 0.005
SHAFT_PRESS_BORE_R = SHAFT_R - PRESS_INTERFERENCE
SHAFT_RUNNING_BORE_R = SHAFT_R + 0.05

# All crank-slider hardpoints lie in this exposed front working plane.
ROD_BACK_Y = -22.0
ROD_THICKNESS = 5.0
ROD_FRONT_Y = ROD_BACK_Y - ROD_THICKNESS
ROD_CENTER_Y = (ROD_BACK_Y + ROD_FRONT_Y) / 2.0

CRANK_DISK_Y0 = -20.0
CRANK_DISK_THICKNESS = 6.0
CRANK_DISK_Y1 = CRANK_DISK_Y0 + CRANK_DISK_THICKNESS
CRANK_DISK_R = CRANK_RADIUS + 6.0

CRANK_PIN_Y0 = -28.0
CRANK_PIN_Y1 = -19.0
CRANK_PIN_LENGTH = CRANK_PIN_Y1 - CRANK_PIN_Y0
CRANK_PIN_SEAT_DEPTH = CRANK_PIN_Y1 - CRANK_DISK_Y0

CROSSHEAD_HALF_X = 9.0
CROSSHEAD_HALF_Z = 8.0
CROSSHEAD_CENTER_Y = ROD_CENTER_Y
CROSSHEAD_CENTER_Z = WRIST_PIN_WORLD_Z

FRONT_CHEEK_Y0 = -31.0
FRONT_CHEEK_Y1 = ROD_FRONT_Y - 0.2
REAR_CHEEK_Y0 = ROD_BACK_Y + 0.2
REAR_CHEEK_Y1 = -18.0
WRIST_PIN_Y0 = FRONT_CHEEK_Y0
WRIST_PIN_Y1 = REAR_CHEEK_Y1
WRIST_PIN_LENGTH = WRIST_PIN_Y1 - WRIST_PIN_Y0

GUIDE_CLEARANCE = 0.10
GUIDE_INNER_X = CROSSHEAD_HALF_X + GUIDE_CLEARANCE
GUIDE_WIDTH = 5.0
GUIDE_DEPTH = 14.0
GUIDE_Z0 = 100.0
GUIDE_Z1 = 156.0
GUIDE_HEIGHT = GUIDE_Z1 - GUIDE_Z0
LEFT_GUIDE_X0 = -GUIDE_INNER_X - GUIDE_WIDTH
RIGHT_GUIDE_X0 = GUIDE_INNER_X

PUMP_ROD_R = 2.5
PUMP_ROD_Z0 = CROSSHEAD_CENTER_Z + CROSSHEAD_HALF_Z
PUMP_ROD_Z1 = 170.0
PUMP_ROD_HEIGHT = PUMP_ROD_Z1 - PUMP_ROD_Z0
PISTON_R = 6.0
PISTON_HEIGHT = 8.0
PISTON_Z0 = PUMP_ROD_Z1

BEARING_OUTER_R = 7.0
BEARING_WIDTH = 6.0
FRONT_BEARING_Y0 = -11.0
REAR_BEARING_Y0 = 5.0
BEARING_BOTTOM_Z = SHAFT_AXIS_Z - BEARING_OUTER_R

ROTOR_Y0 = 14.0
ROTOR_THICKNESS = 4.0
ROTOR_HUB_R = 7.0
ROTOR_OUTER_R = 30.0
ROTOR_BLADE_ROOT_R = 7.0

BASE_X0 = -40.0
BASE_Y0 = -35.0
BASE_Z0 = 0.0
BASE_L = 80.0
BASE_W = 70.0
BASE_H = 4.0

BEARING_POST_W = 10.0
BEARING_POST_D = BEARING_WIDTH
BEARING_POST_Z0 = BASE_H
BEARING_POST_H = BEARING_BOTTOM_Z - BEARING_POST_Z0

GUIDE_COLUMN_CENTER_X = 24.0
GUIDE_COLUMN_W = 6.0
GUIDE_COLUMN_D = 8.0
GUIDE_COLUMN_Z0 = BASE_H
GUIDE_COLUMN_H = 96.0 - GUIDE_COLUMN_Z0
GUIDE_ARM_Z0 = 96.0
GUIDE_ARM_H = GUIDE_Z0 - GUIDE_ARM_Z0

ROD_EYE_OUTER_R = 6.0
ROD_SHANK_HALF_W = 3.0

MECHANISM = {
    "name": "open_frame_wind_rotor_reciprocating_pump",
    "output_link": "pump_rod",
    "watch_links": [
        "rotor_shaft",
        "wind_rotor",
        "crank_disk",
        "connecting_rod",
        "crosshead",
        "pump_rod",
        "pump_piston",
    ],
    "ports_by_link": {
        "front_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_WIDTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_RUNNING_BORE_R,
                "depth_mm": BEARING_WIDTH,
            }
        ],
        "rear_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_WIDTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_RUNNING_BORE_R,
                "depth_mm": BEARING_WIDTH,
            }
        ],
        "rotor_shaft": [
            {
                "name": "shaft_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, SHAFT_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": SHAFT_LENGTH,
            },
            {
                "name": "crank_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    (CRANK_DISK_Y0 + CRANK_DISK_Y1) / 2.0 - SHAFT_START_Y,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANK_DISK_THICKNESS,
            },
            {
                "name": "rotor_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    ROTOR_Y0 + ROTOR_THICKNESS / 2.0 - SHAFT_START_Y,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": ROTOR_THICKNESS,
            },
        ],
        "crank_disk": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CRANK_DISK_THICKNESS / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_PRESS_BORE_R,
                "depth_mm": CRANK_DISK_THICKNESS,
            },
            {
                "name": "eccentric_pin_seat",
                "type": "bore",
                "xyz_mm": [CRANK_RADIUS, 0.0, CRANK_PIN_SEAT_DEPTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (PIN_R - PRESS_INTERFERENCE),
                "depth_mm": CRANK_PIN_SEAT_DEPTH,
            },
        ],
        "wind_rotor": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, ROTOR_THICKNESS / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_PRESS_BORE_R,
                "depth_mm": ROTOR_THICKNESS,
            }
        ],
        "crank_pin": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, CRANK_PIN_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": CRANK_PIN_LENGTH,
            }
        ],
        "connecting_rod": [
            {
                "name": "big_end",
                "type": "bore",
                "xyz_mm": [
                    CRANK_PIN_WORLD_X,
                    CRANK_PIN_WORLD_Z,
                    ROD_THICKNESS / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_BORE_R,
                "depth_mm": ROD_THICKNESS,
            },
            {
                "name": "small_end",
                "type": "bore",
                "xyz_mm": [
                    WRIST_PIN_WORLD_X,
                    WRIST_PIN_WORLD_Z,
                    ROD_THICKNESS / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_BORE_R,
                "depth_mm": ROD_THICKNESS,
            },
        ],
        "crosshead": [
            {
                "name": "wrist_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PIN_BORE_R,
                "depth_mm": WRIST_PIN_LENGTH,
            },
            {
                "name": "vertical_slide",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
            },
            {
                "name": "pump_rod_seat",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, CROSSHEAD_HALF_Z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
            },
        ],
        "wrist_pin": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, WRIST_PIN_LENGTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": WRIST_PIN_LENGTH,
            }
        ],
        "pump_rod": [
            {
                "name": "lower_end",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
            },
            {
                "name": "upper_end",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, PUMP_ROD_HEIGHT],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
            },
        ],
        "pump_piston": [
            {
                "name": "rod_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, PISTON_HEIGHT / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_PRESS_BORE_R,
                "depth_mm": PISTON_HEIGHT,
            }
        ],
    },
    "relations": [
        {
            "name": "front_shaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "front_bearing",
            "base_port": "journal",
            "incoming_part": "rotor_shaft",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "rear_shaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "rear_bearing",
            "base_port": "journal",
            "incoming_part": "rotor_shaft",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "shaft_carries_crank",
            "mate_type": "press_fit",
            "base_part": "rotor_shaft",
            "base_port": "crank_seat",
            "incoming_part": "crank_disk",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "shaft_carries_wind_rotor",
            "mate_type": "press_fit",
            "base_part": "rotor_shaft",
            "base_port": "rotor_seat",
            "incoming_part": "wind_rotor",
            "incoming_port": "hub_bore",
        },
        {
            "name": "crank_carries_eccentric_pin",
            "mate_type": "press_fit",
            "base_part": "crank_disk",
            "base_port": "eccentric_pin_seat",
            "incoming_part": "crank_pin",
            "incoming_port": "pin_axis",
        },
        {
            "name": "crank_big_end_pin",
            "mate_type": "pin",
            "base_part": "crank_pin",
            "base_port": "pin_axis",
            "incoming_part": "connecting_rod",
            "incoming_port": "big_end",
        },
        {
            "name": "crosshead_carries_wrist_pin",
            "mate_type": "press_fit",
            "base_part": "crosshead",
            "base_port": "wrist_bore",
            "incoming_part": "wrist_pin",
            "incoming_port": "pin_axis",
        },
        {
            "name": "rod_small_end_closure",
            "mate_type": "pin",
            "base_part": "wrist_pin",
            "base_port": "pin_axis",
            "incoming_part": "connecting_rod",
            "incoming_port": "small_end",
        },
        {
            "name": "crosshead_carries_pump_rod",
            "mate_type": "coaxial_face",
            "base_part": "crosshead",
            "base_port": "pump_rod_seat",
            "incoming_part": "pump_rod",
            "incoming_port": "lower_end",
        },
        {
            "name": "pump_rod_carries_piston",
            "mate_type": "press_fit",
            "base_part": "pump_rod",
            "base_port": "upper_end",
            "incoming_part": "pump_piston",
            "incoming_port": "rod_bore",
        },
    ],
    "motion_joints": [
        {
            "name": "world_rotor_shaft_hinge",
            "parent": "",
            "child": "rotor_shaft",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "world_crosshead_vertical_slide",
            "parent": "",
            "child": "crosshead",
            "type": "slide",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
    ],
    "transmissions": [
        {
            "name": "shaft_to_crank_rigid_carry",
            "type": "compound_1to1",
            "driving_link": "rotor_shaft",
            "driven_link": "crank_disk",
            "ratio": 1.0,
        },
        {
            "name": "shaft_to_wind_rotor_rigid_carry",
            "type": "compound_1to1",
            "driving_link": "rotor_shaft",
            "driven_link": "wind_rotor",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("open_frame_wind_rotor_reciprocating_pump")

    axis_y_rotation = Location((0.0, 0.0, 0.0), (-90.0, 0.0, 0.0))
    rod_plane_rotation = Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0))

    def annulus(outer_r, inner_r, height):
        with BuildSketch() as sk:
            Circle(outer_r)
            Circle(inner_r, mode=Mode.SUBTRACT)
        return extrude(sk.sketch, amount=height)

    # Stable open bench base.
    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.MIN, Align.MIN, Align.MIN),
    ).moved(Location((BASE_X0, BASE_Y0, BASE_Z0)))
    a.add(base, "base|dof=fixed")

    # Bearing pedestals terminate at the tangent point of each bearing ring.
    front_post = Box(
        BEARING_POST_W,
        BEARING_POST_D,
        BEARING_POST_H,
        align=(Align.MIN, Align.MIN, Align.MIN),
    ).moved(
        Location(
            (
                -BEARING_POST_W / 2.0,
                FRONT_BEARING_Y0,
                BEARING_POST_Z0,
            )
        )
    )
    a.add(front_post, "front_bearing_post|dof=fixed|mount=base")

    rear_post = Box(
        BEARING_POST_W,
        BEARING_POST_D,
        BEARING_POST_H,
        align=(Align.MIN, Align.MIN, Align.MIN),
    ).moved(
        Location(
            (
                -BEARING_POST_W / 2.0,
                REAR_BEARING_Y0,
                BEARING_POST_Z0,
            )
        )
    )
    a.add(rear_post, "rear_bearing_post|dof=fixed|mount=base")

    front_bearing = annulus(
        BEARING_OUTER_R,
        SHAFT_RUNNING_BORE_R,
        BEARING_WIDTH,
    ).moved(
        Location(
            (SHAFT_AXIS_X, FRONT_BEARING_Y0, SHAFT_AXIS_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        front_bearing,
        "front_bearing|dof=fixed|mount=front_bearing_post",
    )

    rear_bearing = annulus(
        BEARING_OUTER_R,
        SHAFT_RUNNING_BORE_R,
        BEARING_WIDTH,
    ).moved(
        Location(
            (SHAFT_AXIS_X, REAR_BEARING_Y0, SHAFT_AXIS_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        rear_bearing,
        "rear_bearing|dof=fixed|mount=rear_bearing_post",
    )

    # Only active input: horizontal shaft along global +Y.
    rotor_shaft = Cylinder(
        SHAFT_R,
        SHAFT_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (SHAFT_AXIS_X, SHAFT_START_Y, SHAFT_AXIS_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        rotor_shaft,
        "rotor_shaft|dof=spin|spin_axis=z|driver=True|"
        "mount=front_bearing,rear_bearing",
    )

    # Crank disk with shaft interference bore and a dedicated shallow pin seat.
    with BuildSketch() as crank_sk:
        Circle(CRANK_DISK_R)
        Circle(SHAFT_PRESS_BORE_R, mode=Mode.SUBTRACT)
        with b3d.Locations((CRANK_RADIUS, 0.0)):
            Circle(PIN_R - PRESS_INTERFERENCE, mode=Mode.SUBTRACT)
    crank_disk = extrude(crank_sk.sketch, amount=CRANK_DISK_THICKNESS).moved(
        Location(
            (SHAFT_AXIS_X, CRANK_DISK_Y0, SHAFT_AXIS_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        crank_disk,
        "crank_disk|dof=fixed|mount=rotor_shaft",
    )

    # Four broad, exposed wind-rotor blades in the shaft-normal plane.
    with BuildSketch() as rotor_sk:
        Circle(ROTOR_HUB_R)
        for i in range(4):
            angle = i * math.pi / 2.0
            ux, uy = math.cos(angle), math.sin(angle)
            tx, ty = -uy, ux

            root_half = 3.0
            tip_half = 7.0
            root_r = ROTOR_BLADE_ROOT_R
            tip_r = ROTOR_OUTER_R

            p1 = (ux * root_r + tx * root_half, uy * root_r + ty * root_half)
            p2 = (ux * tip_r + tx * tip_half, uy * tip_r + ty * tip_half)
            p3 = (ux * tip_r - tx * tip_half, uy * tip_r - ty * tip_half)
            p4 = (ux * root_r - tx * root_half, uy * root_r - ty * root_half)
            Polygon(p1, p2, p3, p4)

        Circle(SHAFT_PRESS_BORE_R, mode=Mode.SUBTRACT)

    wind_rotor = extrude(rotor_sk.sketch, amount=ROTOR_THICKNESS).moved(
        Location(
            (SHAFT_AXIS_X, ROTOR_Y0, SHAFT_AXIS_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        wind_rotor,
        "wind_rotor|dof=fixed|mount=rotor_shaft",
    )

    # Dedicated eccentric crank pin. Its rear 1 mm is seated in the crank disk.
    crank_pin = Cylinder(
        PIN_R,
        CRANK_PIN_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (CRANK_PIN_WORLD_X, CRANK_PIN_Y0, CRANK_PIN_WORLD_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        crank_pin,
        "crank_pin|dof=fixed|mount=crank_disk",
    )

    # Connecting rod profile is derived directly from solved pin centers.
    crank_side_a = (
        CRANK_PIN_WORLD_X + ROD_NORMAL_X * ROD_SHANK_HALF_W,
        CRANK_PIN_WORLD_Z + ROD_NORMAL_Z * ROD_SHANK_HALF_W,
    )
    crank_side_b = (
        CRANK_PIN_WORLD_X - ROD_NORMAL_X * ROD_SHANK_HALF_W,
        CRANK_PIN_WORLD_Z - ROD_NORMAL_Z * ROD_SHANK_HALF_W,
    )
    wrist_side_a = (
        WRIST_PIN_WORLD_X + ROD_NORMAL_X * ROD_SHANK_HALF_W,
        WRIST_PIN_WORLD_Z + ROD_NORMAL_Z * ROD_SHANK_HALF_W,
    )
    wrist_side_b = (
        WRIST_PIN_WORLD_X - ROD_NORMAL_X * ROD_SHANK_HALF_W,
        WRIST_PIN_WORLD_Z - ROD_NORMAL_Z * ROD_SHANK_HALF_W,
    )

    with BuildSketch() as rod_sk:
        Circle(ROD_EYE_OUTER_R, mode=Mode.ADD).moved(
            Location((CRANK_PIN_WORLD_X, CRANK_PIN_WORLD_Z))
        )
        Circle(ROD_EYE_OUTER_R, mode=Mode.ADD).moved(
            Location((WRIST_PIN_WORLD_X, WRIST_PIN_WORLD_Z))
        )
        Polygon(
            crank_side_a,
            wrist_side_a,
            wrist_side_b,
            crank_side_b,
            mode=Mode.ADD,
        )
        Circle(PIN_BORE_R, mode=Mode.SUBTRACT).moved(
            Location((CRANK_PIN_WORLD_X, CRANK_PIN_WORLD_Z))
        )
        Circle(PIN_BORE_R, mode=Mode.SUBTRACT).moved(
            Location((WRIST_PIN_WORLD_X, WRIST_PIN_WORLD_Z))
        )

    connecting_rod = extrude(rod_sk.sketch, amount=ROD_THICKNESS).moved(
        Location((0.0, ROD_BACK_Y, 0.0), (90.0, 0.0, 0.0))
    )
    a.add(
        connecting_rod,
        "connecting_rod|dof=free|mount=crank_pin,wrist_pin",
    )

    # Remote open-frame columns support only the upper guide rails.
    left_column = Box(
        GUIDE_COLUMN_W,
        GUIDE_COLUMN_D,
        GUIDE_COLUMN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                -GUIDE_COLUMN_CENTER_X,
                CROSSHEAD_CENTER_Y,
                GUIDE_COLUMN_Z0,
            )
        )
    )
    a.add(left_column, "left_guide_column|dof=fixed|mount=base")

    right_column = Box(
        GUIDE_COLUMN_W,
        GUIDE_COLUMN_D,
        GUIDE_COLUMN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                GUIDE_COLUMN_CENTER_X,
                CROSSHEAD_CENTER_Y,
                GUIDE_COLUMN_Z0,
            )
        )
    )
    a.add(right_column, "right_guide_column|dof=fixed|mount=base")

    left_arm_x0 = -GUIDE_COLUMN_CENTER_X
    left_arm_x1 = LEFT_GUIDE_X0 + GUIDE_WIDTH
    left_arm = Box(
        left_arm_x1 - left_arm_x0,
        GUIDE_DEPTH,
        GUIDE_ARM_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((left_arm_x0, CROSSHEAD_CENTER_Y, GUIDE_ARM_Z0)))
    a.add(
        left_arm,
        "left_guide_arm|dof=fixed|mount=left_guide_column",
    )

    right_arm_x0 = RIGHT_GUIDE_X0
    right_arm_x1 = GUIDE_COLUMN_CENTER_X
    right_arm = Box(
        right_arm_x1 - right_arm_x0,
        GUIDE_DEPTH,
        GUIDE_ARM_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((right_arm_x0, CROSSHEAD_CENTER_Y, GUIDE_ARM_Z0)))
    a.add(
        right_arm,
        "right_guide_arm|dof=fixed|mount=right_guide_column",
    )

    left_guide = Box(
        GUIDE_WIDTH,
        GUIDE_DEPTH,
        GUIDE_HEIGHT,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((LEFT_GUIDE_X0, CROSSHEAD_CENTER_Y, GUIDE_Z0)))
    a.add(
        left_guide,
        "left_guide|dof=fixed|mount=left_guide_arm",
    )

    right_guide = Box(
        GUIDE_WIDTH,
        GUIDE_DEPTH,
        GUIDE_HEIGHT,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((RIGHT_GUIDE_X0, CROSSHEAD_CENTER_Y, GUIDE_Z0)))
    a.add(
        right_guide,
        "right_guide|dof=fixed|mount=right_guide_arm",
    )

    # Crosshead: separated front/rear cheeks joined by side rails. The rod eye
    # occupies the central open pocket and only the wrist pin enters its bores.
    front_cheek_depth = FRONT_CHEEK_Y1 - FRONT_CHEEK_Y0
    rear_cheek_depth = REAR_CHEEK_Y1 - REAR_CHEEK_Y0
    cheek_width = 2.0 * CROSSHEAD_HALF_X
    cheek_height = 2.0 * CROSSHEAD_HALF_Z
    side_rail_width = 3.0

    front_cheek = Box(
        cheek_width,
        front_cheek_depth,
        cheek_height,
        align=(Align.CENTER, Align.MIN, Align.CENTER),
    ).moved(
        Location(
            (
                0.0,
                FRONT_CHEEK_Y0 - CROSSHEAD_CENTER_Y,
                0.0,
            )
        )
    )

    rear_cheek = Box(
        cheek_width,
        rear_cheek_depth,
        cheek_height,
        align=(Align.CENTER, Align.MIN, Align.CENTER),
    ).moved(
        Location(
            (
                0.0,
                REAR_CHEEK_Y0 - CROSSHEAD_CENTER_Y,
                0.0,
            )
        )
    )

    full_crosshead_depth = REAR_CHEEK_Y1 - FRONT_CHEEK_Y0
    left_side_rail = Box(
        side_rail_width,
        full_crosshead_depth,
        cheek_height,
        align=(Align.MIN, Align.MIN, Align.CENTER),
    ).moved(
        Location(
            (
                -CROSSHEAD_HALF_X,
                FRONT_CHEEK_Y0 - CROSSHEAD_CENTER_Y,
                0.0,
            )
        )
    )
    right_side_rail = Box(
        side_rail_width,
        full_crosshead_depth,
        cheek_height,
        align=(Align.MAX, Align.MIN, Align.CENTER),
    ).moved(
        Location(
            (
                CROSSHEAD_HALF_X,
                FRONT_CHEEK_Y0 - CROSSHEAD_CENTER_Y,
                0.0,
            )
        )
    )

    crosshead_raw = (
        front_cheek
        + rear_cheek
        + left_side_rail
        + right_side_rail
    )

    wrist_bore_tool = Cylinder(
        PIN_BORE_R,
        WRIST_PIN_LENGTH + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                FRONT_CHEEK_Y0 - CROSSHEAD_CENTER_Y - 1.0,
                0.0,
            ),
            (-90.0, 0.0, 0.0),
        )
    )
    crosshead_local = crosshead_raw - wrist_bore_tool
    crosshead = crosshead_local.moved(
        Location(
            (
                WRIST_PIN_WORLD_X,
                CROSSHEAD_CENTER_Y,
                CROSSHEAD_CENTER_Z,
            )
        )
    )
    a.add(
        crosshead,
        "crosshead|dof=slide|slide_axis=z|mount=left_guide,right_guide",
    )

    wrist_pin = Cylinder(
        PIN_R,
        WRIST_PIN_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                WRIST_PIN_WORLD_X,
                WRIST_PIN_Y0,
                WRIST_PIN_WORLD_Z,
            ),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        wrist_pin,
        "wrist_pin|dof=fixed|mount=crosshead",
    )

    pump_rod = Cylinder(
        PUMP_ROD_R,
        PUMP_ROD_HEIGHT,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                SLIDER_AXIS_X,
                CROSSHEAD_CENTER_Y,
                PUMP_ROD_Z0,
            )
        )
    )
    a.add(
        pump_rod,
        "pump_rod|dof=fixed|mount=crosshead",
    )

    with BuildSketch() as piston_sk:
        Circle(PISTON_R)
        Circle(PUMP_ROD_R - PRESS_INTERFERENCE, mode=Mode.SUBTRACT)
    pump_piston = extrude(piston_sk.sketch, amount=PISTON_HEIGHT).moved(
        Location(
            (
                SLIDER_AXIS_X,
                CROSSHEAD_CENTER_Y,
                PISTON_Z0,
            )
        )
    )
    a.add(
        pump_piston,
        "pump_piston|dof=fixed|mount=pump_rod",
    )

    return a.build()