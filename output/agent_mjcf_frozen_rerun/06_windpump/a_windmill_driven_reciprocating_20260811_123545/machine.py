import math


# ---------------------------------------------------------------------------
# ENGINEERING ARITHMETIC — all mechanism hardpoints are derived here first
# ---------------------------------------------------------------------------

# Primary layout
BASE_L = 100.0
BASE_W = 110.0
BASE_H = 4.0

SHAFT_X = 0.0
SHAFT_Z = 95.0
SHAFT_R = 3.0
SHAFT_Y0 = -46.0
SHAFT_Y1 = 30.0
SHAFT_LEN = SHAFT_Y1 - SHAFT_Y0

# Rotor
ROTOR_BLADES = 8
ROTOR_R = 48.0
ROTOR_HUB_R = 10.0
ROTOR_T = 6.0
ROTOR_Y0 = -44.0
ROTOR_YC = ROTOR_Y0 + ROTOR_T / 2.0
ROTOR_BORE_R = SHAFT_R - 0.005

# Shaft bearings
BEARING_OUTER_R = 9.0
BEARING_W = 10.0
BEARING_BORE_R = SHAFT_R + 0.05
BEARING_1_Y0 = -28.0
BEARING_2_Y0 = 2.0
BEARING_1_YC = BEARING_1_Y0 + BEARING_W / 2.0
BEARING_2_YC = BEARING_2_Y0 + BEARING_W / 2.0
PEDESTAL_TOP_Z = SHAFT_Z - BEARING_OUTER_R
PEDESTAL_H = PEDESTAL_TOP_Z - BASE_H
PEDESTAL_W = 12.0

# Crank geometry
CRANK_R = 17.0
CRANK_T = 8.0
CRANK_Y0 = 20.0
CRANK_YC = CRANK_Y0 + CRANK_T / 2.0
CRANK_BORE_R = SHAFT_R - 0.005
ECCENTRIC_R = 14.0

# Initial configuration is bottom-dead-center.
CRANK_PIN_X = SHAFT_X
CRANK_PIN_Z = SHAFT_Z - ECCENTRIC_R
PIN_R = 3.0
PIN_RUNNING_R = PIN_R + 0.05
PIN_PRESS_R = PIN_R - 0.005

CRANK_PIN_Y0 = CRANK_Y0 + CRANK_T - 2.0
CRANK_PIN_DEPTH = 14.0
CRANK_PIN_YC = CRANK_PIN_Y0 + CRANK_PIN_DEPTH / 2.0

# Slider-crank hardpoints
WRIST_X = CRANK_PIN_X
WRIST_Z = 48.0
WRIST_YC = CRANK_PIN_YC
ROD_CENTER_DISTANCE = CRANK_PIN_Z - WRIST_Z
ROD_EYE_OUTER_R = 6.5
ROD_WEB_HALF_W = 3.5
ROD_T = 6.0
ROD_Y0 = WRIST_YC - ROD_T / 2.0

# Wrist pin spans both cheeks and the rod eye.
WRIST_PIN_Y0 = 24.0
WRIST_PIN_DEPTH = 18.0
WRIST_PIN_YC = WRIST_PIN_Y0 + WRIST_PIN_DEPTH / 2.0

# Crosshead/piston slider
SLIDER_X_HALF = 12.0
SLIDER_Y_HALF = 9.0
SLIDER_YC = WRIST_YC
SLIDER_Z0 = 35.0
SLIDER_CROSSBAR_H = 5.0
SLIDER_CHEEK_Z0 = SLIDER_CROSSBAR_H
SLIDER_CHEEK_H = 16.0
SLIDER_CHEEK_T = 5.0
SLIDER_WRIST_LOCAL_Z = WRIST_Z - SLIDER_Z0
SLIDER_GAP_HALF_Y = 4.0

# Guide clearances and envelope
GUIDE_CLEARANCE = 0.05
GUIDE_T = 4.0
GUIDE_Z0 = BASE_H
GUIDE_Z1 = 61.0
GUIDE_H = GUIDE_Z1 - GUIDE_Z0
GUIDE_INNER_X = SLIDER_X_HALF + GUIDE_CLEARANCE
GUIDE_INNER_Y = SLIDER_Y_HALF + GUIDE_CLEARANCE

# Pump rod and piston
PUMP_AXIS_X = WRIST_X
PUMP_AXIS_Y = WRIST_YC
PUMP_ROD_R = 3.0
PUMP_ROD_Z0 = 11.0
PUMP_ROD_Z1 = SLIDER_Z0 + SLIDER_CROSSBAR_H - 1.0
PUMP_ROD_H = PUMP_ROD_Z1 - PUMP_ROD_Z0
PUMP_ROD_PRESS_BORE_R = PUMP_ROD_R - 0.005

PISTON_R = 9.7
PISTON_H = 6.0
PISTON_Z0 = 10.0
PISTON_BORE_R = PUMP_ROD_R - 0.005

BARREL_INNER_R = PISTON_R + 0.15
BARREL_WALL = 3.0
BARREL_OUTER_R = BARREL_INNER_R + BARREL_WALL
BARREL_Z0 = BASE_H
BARREL_H = 24.0
BARREL_Z1 = BARREL_Z0 + BARREL_H

# The foot remains on the base and touches the barrel tangentially.
CYLINDER_FOOT_X = PUMP_AXIS_X + 2.0 * BARREL_OUTER_R

PIPE_OUTER_R = 3.0
PIPE_INNER_R = 2.0
PIPE_END_X = 26.0
PIPE_LEN = PIPE_END_X - BARREL_OUTER_R
INLET_Z = 10.0
OUTLET_Z = 22.0


MECHANISM = {
    "name": "windmill_reciprocating_piston_water_pump",
    "output_link": "piston_slider",
    "watch_links": [
        "wind_rotor",
        "rotor_shaft",
        "crank_disk",
        "connecting_rod",
        "piston_slider",
        "pump_rod",
        "pump_piston",
    ],
    "ports_by_link": {
        "wind_rotor": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, ROTOR_T / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * ROTOR_BORE_R,
                "depth_mm": ROTOR_T,
            }
        ],
        "rotor_shaft": [
            {
                "name": "rotor_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, ROTOR_YC - SHAFT_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": ROTOR_T,
            },
            {
                "name": "front_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, BEARING_1_YC - SHAFT_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_W,
            },
            {
                "name": "rear_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, BEARING_2_YC - SHAFT_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": BEARING_W,
            },
            {
                "name": "crank_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, CRANK_YC - SHAFT_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANK_T,
            },
        ],
        "front_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_W,
            }
        ],
        "rear_bearing": [
            {
                "name": "journal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, BEARING_W / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * BEARING_BORE_R,
                "depth_mm": BEARING_W,
            }
        ],
        "crank_disk": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, CRANK_T / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_BORE_R,
                "depth_mm": CRANK_T,
            },
            {
                "name": "eccentric_pin_bore",
                "type": "bore",
                "xyz_mm": [0.0, ECCENTRIC_R, CRANK_T - 1.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_PRESS_R,
                "depth_mm": 2.0,
            },
        ],
        "crank_pin": [
            {
                "name": "disk_press_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 1.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": 2.0,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, CRANK_PIN_YC - CRANK_PIN_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": ROD_T,
            },
        ],
        "connecting_rod": [
            {
                "name": "crank_eye",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, ROD_T / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_RUNNING_R,
                "depth_mm": ROD_T,
            },
            {
                "name": "wrist_eye",
                "type": "bore",
                "xyz_mm": [0.0, ROD_CENTER_DISTANCE, ROD_T / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_RUNNING_R,
                "depth_mm": ROD_T,
            },
        ],
        "wrist_pin": [
            {
                "name": "slider_press_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, WRIST_PIN_DEPTH / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": WRIST_PIN_DEPTH,
            },
            {
                "name": "rod_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, WRIST_PIN_YC - WRIST_PIN_Y0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PIN_R,
                "depth_mm": ROD_T,
            },
        ],
        "piston_slider": [
            {
                "name": "wrist_pin_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, SLIDER_WRIST_LOCAL_Z],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PIN_PRESS_R,
                "depth_mm": WRIST_PIN_DEPTH,
            },
            {
                "name": "pump_rod_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, SLIDER_CROSSBAR_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_PRESS_BORE_R,
                "depth_mm": SLIDER_CROSSBAR_H,
            },
        ],
        "pump_rod": [
            {
                "name": "slider_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, PUMP_ROD_Z1 - PUMP_ROD_Z0 - 1.5],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
                "depth_mm": 3.0,
            },
            {
                "name": "piston_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    PISTON_Z0 + PISTON_H / 2.0 - PUMP_ROD_Z0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PUMP_ROD_R,
                "depth_mm": PISTON_H,
            },
        ],
        "pump_piston": [
            {
                "name": "rod_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, PISTON_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PISTON_BORE_R,
                "depth_mm": PISTON_H,
            },
            {
                "name": "barrel_running_surface",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, PISTON_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * PISTON_R,
                "depth_mm": PISTON_H,
            },
        ],
        "pump_cylinder": [
            {
                "name": "piston_bore",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, BARREL_H / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * BARREL_INNER_R,
                "depth_mm": BARREL_H,
            }
        ],
    },
    "relations": [
        {
            "name": "rotor_to_shaft_press_fit",
            "mate_type": "press_fit",
            "base_part": "rotor_shaft",
            "base_port": "rotor_seat",
            "incoming_part": "wind_rotor",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "front_shaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "front_bearing",
            "base_port": "journal_bore",
            "incoming_part": "rotor_shaft",
            "incoming_port": "front_journal",
        },
        {
            "name": "rear_shaft_journal",
            "mate_type": "journal_bearing",
            "base_part": "rear_bearing",
            "base_port": "journal_bore",
            "incoming_part": "rotor_shaft",
            "incoming_port": "rear_journal",
        },
        {
            "name": "shaft_to_crank_press_fit",
            "mate_type": "press_fit",
            "base_part": "rotor_shaft",
            "base_port": "crank_seat",
            "incoming_part": "crank_disk",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "eccentric_pin_press_fit",
            "mate_type": "press_fit",
            "base_part": "crank_disk",
            "base_port": "eccentric_pin_bore",
            "incoming_part": "crank_pin",
            "incoming_port": "disk_press_seat",
        },
        {
            "name": "crank_rod_revolute",
            "mate_type": "revolute",
            "base_part": "crank_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "crank_eye",
        },
        {
            "name": "wrist_rod_revolute",
            "mate_type": "revolute",
            "base_part": "wrist_pin",
            "base_port": "rod_journal",
            "incoming_part": "connecting_rod",
            "incoming_port": "wrist_eye",
        },
        {
            "name": "wrist_pin_to_slider",
            "mate_type": "press_fit",
            "base_part": "piston_slider",
            "base_port": "wrist_pin_bore",
            "incoming_part": "wrist_pin",
            "incoming_port": "slider_press_seat",
        },
        {
            "name": "slider_to_pump_rod",
            "mate_type": "press_fit",
            "base_part": "piston_slider",
            "base_port": "pump_rod_bore",
            "incoming_part": "pump_rod",
            "incoming_port": "slider_seat",
        },
        {
            "name": "pump_rod_to_piston",
            "mate_type": "press_fit",
            "base_part": "pump_rod",
            "base_port": "piston_seat",
            "incoming_part": "pump_piston",
            "incoming_port": "rod_bore",
        },
        {
            "name": "piston_in_barrel",
            "mate_type": "cylindrical",
            "base_part": "pump_cylinder",
            "base_port": "piston_bore",
            "incoming_part": "pump_piston",
            "incoming_port": "barrel_running_surface",
        },
    ],
    "motion_joints": [
        {
            "name": "rotor_input_hinge",
            "parent": "",
            "child": "wind_rotor",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
        },
        {
            "name": "shaft_hinge",
            "parent": "",
            "child": "rotor_shaft",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
        },
        {
            "name": "crank_hinge",
            "parent": "",
            "child": "crank_disk",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
        },
        {
            "name": "crosshead_vertical_slide",
            "parent": "",
            "child": "piston_slider",
            "type": "slide",
            "axis": [0.0, 0.0, 1.0],
        },
        {
            "name": "pump_rod_vertical_slide",
            "parent": "",
            "child": "pump_rod",
            "type": "slide",
            "axis": [0.0, 0.0, 1.0],
        },
        {
            "name": "piston_vertical_slide",
            "parent": "",
            "child": "pump_piston",
            "type": "slide",
            "axis": [0.0, 0.0, 1.0],
        },
    ],
    "transmissions": [
        {
            "name": "rotor_shaft_lock",
            "type": "compound_1to1",
            "driving_link": "wind_rotor",
            "driven_link": "rotor_shaft",
            "ratio": 1.0,
        },
        {
            "name": "shaft_crank_lock",
            "type": "compound_1to1",
            "driving_link": "rotor_shaft",
            "driven_link": "crank_disk",
            "ratio": 1.0,
        },
        {
            "name": "crosshead_pump_rod_lock",
            "type": "compound_1to1",
            "driving_link": "piston_slider",
            "driven_link": "pump_rod",
            "ratio": 1.0,
        },
        {
            "name": "pump_rod_piston_lock",
            "type": "compound_1to1",
            "driving_link": "pump_rod",
            "driven_link": "pump_piston",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [],
}


def _annulus(outer_r, inner_r, height):
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
    return outer - cutter


def _make_rotor():
    def rotated_point(radial, tangential, angle):
        ca = math.cos(angle)
        sa = math.sin(angle)
        return (
            radial * ca - tangential * sa,
            radial * sa + tangential * ca,
        )

    with BuildPart() as rotor_part:
        with BuildSketch() as rotor_profile:
            Circle(ROTOR_HUB_R)
            for blade_index in range(ROTOR_BLADES):
                angle = 2.0 * math.pi * blade_index / ROTOR_BLADES
                points = [
                    rotated_point(ROTOR_HUB_R - 1.0, -4.0, angle),
                    rotated_point(ROTOR_HUB_R - 1.0, 4.0, angle),
                    rotated_point(ROTOR_R, 7.0, angle),
                    rotated_point(ROTOR_R, -2.0, angle),
                ]
                Polygon(*points, mode=Mode.ADD)
        extrude(amount=ROTOR_T)

        bore_tool = Cylinder(
            ROTOR_BORE_R,
            ROTOR_T + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -1.0)))
        add(bore_tool, mode=Mode.SUBTRACT)

    return rotor_part.part


def _make_crank_disk():
    disk = Cylinder(
        CRANK_R,
        CRANK_T,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    shaft_bore = Cylinder(
        CRANK_BORE_R,
        CRANK_T + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))

    eccentric_bore = Cylinder(
        PIN_PRESS_R,
        CRANK_T + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, ECCENTRIC_R, -1.0)))

    return disk - shaft_bore - eccentric_bore


def _make_connecting_rod():
    with BuildPart() as rod_part:
        with BuildSketch() as rod_profile:
            Circle(ROD_EYE_OUTER_R)
            with b3d.Locations((0.0, ROD_CENTER_DISTANCE)):
                Circle(ROD_EYE_OUTER_R)
            Polygon(
                (-ROD_WEB_HALF_W, 0.0),
                (ROD_WEB_HALF_W, 0.0),
                (ROD_WEB_HALF_W, ROD_CENTER_DISTANCE),
                (-ROD_WEB_HALF_W, ROD_CENTER_DISTANCE),
                mode=Mode.ADD,
            )
            Circle(PIN_RUNNING_R, mode=Mode.SUBTRACT)
            with b3d.Locations((0.0, ROD_CENTER_DISTANCE)):
                Circle(PIN_RUNNING_R, mode=Mode.SUBTRACT)
        extrude(amount=ROD_T)
    return rod_part.part


def _make_slider():
    crossbar = Box(
        2.0 * SLIDER_X_HALF,
        2.0 * SLIDER_Y_HALF,
        SLIDER_CROSSBAR_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    front_cheek = Box(
        2.0 * SLIDER_X_HALF,
        SLIDER_CHEEK_T,
        SLIDER_CHEEK_H,
        align=(Align.CENTER, Align.MIN, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                -SLIDER_Y_HALF,
                SLIDER_CHEEK_Z0,
            )
        )
    )

    rear_cheek = Box(
        2.0 * SLIDER_X_HALF,
        SLIDER_CHEEK_T,
        SLIDER_CHEEK_H,
        align=(Align.CENTER, Align.MIN, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                SLIDER_GAP_HALF_Y,
                SLIDER_CHEEK_Z0,
            )
        )
    )

    slider = crossbar + front_cheek + rear_cheek

    wrist_bore = Cylinder(
        PIN_PRESS_R,
        2.0 * SLIDER_Y_HALF + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                0.0,
                -SLIDER_Y_HALF - 1.0,
                SLIDER_WRIST_LOCAL_Z,
            ),
            (-90.0, 0.0, 0.0),
        )
    )

    pump_rod_bore = Cylinder(
        PUMP_ROD_PRESS_BORE_R,
        SLIDER_CROSSBAR_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, -1.0)))

    return slider - wrist_bore - pump_rod_bore


def build_machine():
    a = AssemblyHelper("windmill_reciprocating_piston_water_pump")

    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(base, "baseplate|dof=fixed")

    front_pedestal = Box(
        PEDESTAL_W,
        BEARING_W,
        PEDESTAL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((SHAFT_X, BEARING_1_YC, BASE_H)))
    a.add(front_pedestal, "front_pedestal|dof=fixed|mount=baseplate")

    rear_pedestal = Box(
        PEDESTAL_W,
        BEARING_W,
        PEDESTAL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((SHAFT_X, BEARING_2_YC, BASE_H)))
    a.add(rear_pedestal, "rear_pedestal|dof=fixed|mount=baseplate")

    front_bearing = _annulus(
        BEARING_OUTER_R,
        BEARING_BORE_R,
        BEARING_W,
    ).moved(
        Location(
            (SHAFT_X, BEARING_1_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        front_bearing,
        "front_bearing|dof=fixed|mount=front_pedestal",
    )

    rear_bearing = _annulus(
        BEARING_OUTER_R,
        BEARING_BORE_R,
        BEARING_W,
    ).moved(
        Location(
            (SHAFT_X, BEARING_2_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        rear_bearing,
        "rear_bearing|dof=fixed|mount=rear_pedestal",
    )

    shaft = Cylinder(
        SHAFT_R,
        SHAFT_LEN,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (SHAFT_X, SHAFT_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        shaft,
        "rotor_shaft|dof=spin|spin_axis=z|"
        "mount=front_bearing,rear_bearing",
    )

    rotor = _make_rotor().moved(
        Location(
            (SHAFT_X, ROTOR_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        rotor,
        "wind_rotor|dof=spin|spin_axis=z|driver=True|mount=rotor_shaft",
    )

    crank_disk = _make_crank_disk().moved(
        Location(
            (SHAFT_X, CRANK_Y0, SHAFT_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        crank_disk,
        "crank_disk|dof=spin|spin_axis=z|mount=rotor_shaft",
    )

    crank_pin = Cylinder(
        PIN_R,
        CRANK_PIN_DEPTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (CRANK_PIN_X, CRANK_PIN_Y0, CRANK_PIN_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        crank_pin,
        "crank_pin|dof=fixed|mount=crank_disk",
    )

    connecting_rod = _make_connecting_rod().moved(
        Location(
            (CRANK_PIN_X, ROD_Y0, CRANK_PIN_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        connecting_rod,
        "connecting_rod|dof=free|mount=crank_pin,wrist_pin",
    )

    piston_slider = _make_slider().moved(
        Location((WRIST_X, SLIDER_YC, SLIDER_Z0))
    )
    a.add(
        piston_slider,
        "piston_slider|dof=slide|slide_axis=z|"
        "mount=left_guide,right_guide,front_guide,rear_guide",
    )

    wrist_pin = Cylinder(
        PIN_R,
        WRIST_PIN_DEPTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (WRIST_X, WRIST_PIN_Y0, WRIST_Z),
            (-90.0, 0.0, 0.0),
        )
    )
    a.add(
        wrist_pin,
        "wrist_pin|dof=fixed|mount=piston_slider",
    )

    left_guide = Box(
        GUIDE_T,
        2.0 * SLIDER_Y_HALF,
        GUIDE_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                -GUIDE_INNER_X - GUIDE_T,
                SLIDER_YC,
                GUIDE_Z0,
            )
        )
    )
    a.add(
        left_guide,
        "left_guide|dof=fixed|mount=baseplate,pump_cylinder",
    )

    right_guide = Box(
        GUIDE_T,
        2.0 * SLIDER_Y_HALF,
        GUIDE_H,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                GUIDE_INNER_X,
                SLIDER_YC,
                GUIDE_Z0,
            )
        )
    )
    a.add(
        right_guide,
        "right_guide|dof=fixed|mount=baseplate,pump_cylinder",
    )

    front_guide = Box(
        2.0 * SLIDER_X_HALF,
        GUIDE_T,
        GUIDE_H,
        align=(Align.CENTER, Align.MIN, Align.MIN),
    ).moved(
        Location(
            (
                WRIST_X,
                SLIDER_YC - GUIDE_INNER_Y - GUIDE_T,
                GUIDE_Z0,
            )
        )
    )
    a.add(
        front_guide,
        "front_guide|dof=fixed|mount=baseplate,pump_cylinder",
    )

    rear_guide = Box(
        2.0 * SLIDER_X_HALF,
        GUIDE_T,
        GUIDE_H,
        align=(Align.CENTER, Align.MIN, Align.MIN),
    ).moved(
        Location(
            (
                WRIST_X,
                SLIDER_YC + GUIDE_INNER_Y,
                GUIDE_Z0,
            )
        )
    )
    a.add(
        rear_guide,
        "rear_guide|dof=fixed|mount=baseplate,pump_cylinder",
    )

    pump_rod = Cylinder(
        PUMP_ROD_R,
        PUMP_ROD_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((PUMP_AXIS_X, PUMP_AXIS_Y, PUMP_ROD_Z0)))
    a.add(
        pump_rod,
        "pump_rod|dof=slide|slide_axis=z|mount=piston_slider,pump_piston",
    )

    pump_piston = _annulus(
        PISTON_R,
        PISTON_BORE_R,
        PISTON_H,
    ).moved(Location((PUMP_AXIS_X, PUMP_AXIS_Y, PISTON_Z0)))
    a.add(
        pump_piston,
        "pump_piston|dof=slide|slide_axis=z|mount=pump_rod,pump_cylinder",
    )

    pump_cylinder = _annulus(
        BARREL_OUTER_R,
        BARREL_INNER_R,
        BARREL_H,
    ).moved(Location((PUMP_AXIS_X, PUMP_AXIS_Y, BARREL_Z0)))
    a.add(
        pump_cylinder,
        "pump_cylinder|dof=fixed|mount=baseplate",
    )

    cylinder_foot = Cylinder(
        BARREL_OUTER_R + 3.0,
        3.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CYLINDER_FOOT_X, PUMP_AXIS_Y, BASE_H)))
    cylinder_foot_bore = Cylinder(
        BARREL_INNER_R,
        5.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CYLINDER_FOOT_X, PUMP_AXIS_Y, BASE_H - 1.0)))
    cylinder_foot = cylinder_foot - cylinder_foot_bore
    a.add(
        cylinder_foot,
        "cylinder_foot|dof=fixed|mount=baseplate,pump_cylinder",
    )

    inlet_pipe = _annulus(
        PIPE_OUTER_R,
        PIPE_INNER_R,
        PIPE_LEN,
    ).moved(
        Location(
            (
                PUMP_AXIS_X - PIPE_END_X,
                PUMP_AXIS_Y,
                INLET_Z,
            ),
            (0.0, 90.0, 0.0),
        )
    )
    a.add(
        inlet_pipe,
        "inlet_pipe|dof=fixed|mount=pump_cylinder",
    )

    outlet_pipe = _annulus(
        PIPE_OUTER_R,
        PIPE_INNER_R,
        PIPE_LEN,
    ).moved(
        Location(
            (
                PUMP_AXIS_X + BARREL_OUTER_R,
                PUMP_AXIS_Y,
                OUTLET_Z,
            ),
            (0.0, 90.0, 0.0),
        )
    )
    a.add(
        outlet_pipe,
        "outlet_pipe|dof=fixed|mount=pump_cylinder",
    )

    top_bridge_left = Box(
        10.0,
        2.0 * (GUIDE_INNER_Y + GUIDE_T),
        4.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                -GUIDE_INNER_X - GUIDE_T / 2.0,
                SLIDER_YC,
                GUIDE_Z1,
            )
        )
    )
    a.add(
        top_bridge_left,
        "left_guide_cap|dof=fixed|mount=left_guide,front_guide,rear_guide",
    )

    top_bridge_right = Box(
        10.0,
        2.0 * (GUIDE_INNER_Y + GUIDE_T),
        4.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                GUIDE_INNER_X + GUIDE_T / 2.0,
                SLIDER_YC,
                GUIDE_Z1,
            )
        )
    )
    a.add(
        top_bridge_right,
        "right_guide_cap|dof=fixed|mount=right_guide,front_guide,rear_guide",
    )

    return a.build()