import math


# ---------------------------------------------------------------------------
# Drivetrain arithmetic — all gear locations derive from these values
# ---------------------------------------------------------------------------

M = 0.6

Z_MINUTE_PINION = 10
Z_INTERMEDIATE_WHEEL = 30
Z_INTERMEDIATE_PINION = 8
Z_HOUR_WHEEL = 32

GEAR_FACE = 2.0

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

CD_MINUTE_STAGE = center_dist(Z_MINUTE_PINION, Z_INTERMEDIATE_WHEEL)
CD_HOUR_STAGE = center_dist(Z_INTERMEDIATE_PINION, Z_HOUR_WHEEL)

assert abs(CD_MINUTE_STAGE - CD_HOUR_STAGE) < 1.0e-9

INTERMEDIATE_X = CD_MINUTE_STAGE
INTERMEDIATE_Y = 0.0

STAGE_MINUTE_Z = 4.5
STAGE_HOUR_Z = 9.0

RATIO_STAGE_1 = Z_INTERMEDIATE_WHEEL / Z_MINUTE_PINION
RATIO_STAGE_2 = Z_HOUR_WHEEL / Z_INTERMEDIATE_PINION
MINUTE_TO_HOUR_RATIO = RATIO_STAGE_1 * RATIO_STAGE_2

assert abs(MINUTE_TO_HOUR_RATIO - 12.0) < 1.0e-9

# Half-tooth initial phases help place spaces opposite teeth at the line of centers.
PHASE_INTERMEDIATE_WHEEL_DEG = 180.0 / Z_INTERMEDIATE_WHEEL
PHASE_INTERMEDIATE_PINION_DEG = PHASE_INTERMEDIATE_WHEEL_DEG
PHASE_HOUR_WHEEL_DEG = (
    PHASE_INTERMEDIATE_PINION_DEG + 180.0 / Z_HOUR_WHEEL
)

# ---------------------------------------------------------------------------
# Fits and axial stack
# ---------------------------------------------------------------------------

BASE_H = 2.0
LOWER_BEARING_Z = BASE_H
LOWER_BEARING_H = 1.5
LOWER_BEARING_TOP = LOWER_BEARING_Z + LOWER_BEARING_H

UPPER_BEARING_Z = 15.5
UPPER_BEARING_H = 1.5
UPPER_BEARING_TOP = UPPER_BEARING_Z + UPPER_BEARING_H

BRIDGE_Z = UPPER_BEARING_TOP
BRIDGE_H = 2.0
BRIDGE_TOP = BRIDGE_Z + BRIDGE_H

MINUTE_SHAFT_R = 1.0
MINUTE_SHAFT_Z = LOWER_BEARING_Z
MINUTE_SHAFT_TOP = 22.4
MINUTE_SHAFT_H = MINUTE_SHAFT_TOP - MINUTE_SHAFT_Z

INTERMEDIATE_SHAFT_R = 1.0
INTERMEDIATE_SHAFT_Z = LOWER_BEARING_Z
INTERMEDIATE_SHAFT_TOP = BRIDGE_TOP
INTERMEDIATE_SHAFT_H = INTERMEDIATE_SHAFT_TOP - INTERMEDIATE_SHAFT_Z

RUNNING_CLEARANCE = 0.05
PRESS_INTERFERENCE = 0.005

MINUTE_SHAFT_RUNNING_R = MINUTE_SHAFT_R + RUNNING_CLEARANCE
MINUTE_SHAFT_PRESS_BORE_D = 2.0 * (MINUTE_SHAFT_R - PRESS_INTERFERENCE)

INTERMEDIATE_RUNNING_R = INTERMEDIATE_SHAFT_R + RUNNING_CLEARANCE
INTERMEDIATE_PRESS_BORE_D = (
    2.0 * (INTERMEDIATE_SHAFT_R - PRESS_INTERFERENCE)
)

HOUR_PIPE_INNER_R = MINUTE_SHAFT_R + RUNNING_CLEARANCE
HOUR_PIPE_OUTER_R = 1.8
HOUR_PIPE_Z = STAGE_MINUTE_Z + GEAR_FACE
HOUR_PIPE_TOP = 21.4
HOUR_PIPE_H = HOUR_PIPE_TOP - HOUR_PIPE_Z
HOUR_PIPE_PRESS_BORE_D = 2.0 * (HOUR_PIPE_OUTER_R - PRESS_INTERFERENCE)
HOUR_PIPE_RUNNING_R = HOUR_PIPE_OUTER_R + RUNNING_CLEARANCE

LOWER_BEARING_OUTER_R = 2.8
UPPER_CENTER_BEARING_OUTER_R = 3.0
INTERMEDIATE_BEARING_OUTER_R = 2.8

HOUR_HAND_Z = 21.0
HOUR_HAND_H = 0.4
MINUTE_HAND_Z = 22.0
MINUTE_HAND_H = 0.4

HOUR_HAND_LENGTH = 17.0
MINUTE_HAND_LENGTH = 24.0

MAX_GEAR_OUTER_R = max(
    pitch_r(Z_INTERMEDIATE_WHEEL) + M,
    pitch_r(Z_HOUR_WHEEL) + M,
)

MOVEMENT_OUTER_R = (
    INTERMEDIATE_X + pitch_r(Z_INTERMEDIATE_WHEEL) + M + 7.0
)
MOVEMENT_RING_W = 4.0
MOVEMENT_INNER_R = MOVEMENT_OUTER_R - MOVEMENT_RING_W

BRIDGE_SUPPORT_Y = MAX_GEAR_OUTER_R + 3.0
BRIDGE_POST_R = 1.5
BRIDGE_POST_Z = BASE_H
BRIDGE_POST_H = BRIDGE_Z - BRIDGE_POST_Z

DIAL_POST_R = MOVEMENT_OUTER_R - 2.0
DIAL_POST_OUTER_R = 1.25
DIAL_POST_Z = BASE_H
CHAPTER_RING_Z = 20.0
CHAPTER_RING_H = 0.8
DIAL_POST_H = CHAPTER_RING_Z - DIAL_POST_Z

CHAPTER_OUTER_R = MOVEMENT_OUTER_R
CHAPTER_INNER_R = MINUTE_HAND_LENGTH - 1.0

MECHANISM = {
    "name": "openwork_mechanical_watch_movement",
    "output_link": "hour_hand",
    "watch_links": [
        "minute_hand",
        "hour_hand",
        "minute_pinion",
        "intermediate_wheel",
        "intermediate_pinion",
        "hour_wheel",
    ],
    "ports_by_link": {
        "central_lower_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0, 0, LOWER_BEARING_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * MINUTE_SHAFT_RUNNING_R,
                "depth_mm": LOWER_BEARING_H,
            }
        ],
        "central_upper_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0, 0, UPPER_BEARING_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * HOUR_PIPE_RUNNING_R,
                "depth_mm": UPPER_BEARING_H,
            }
        ],
        "intermediate_lower_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0, 0, LOWER_BEARING_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * INTERMEDIATE_RUNNING_R,
                "depth_mm": LOWER_BEARING_H,
            }
        ],
        "intermediate_upper_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0, 0, UPPER_BEARING_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * INTERMEDIATE_RUNNING_R,
                "depth_mm": UPPER_BEARING_H,
            }
        ],
        "minute_shaft": [
            {
                "name": "outer",
                "type": "shaft",
                "xyz_mm": [0, 0, MINUTE_SHAFT_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * MINUTE_SHAFT_R,
                "depth_mm": MINUTE_SHAFT_H,
            },
            {
                "name": "minute_pinion_seat",
                "type": "cylindrical",
                "xyz_mm": [
                    0,
                    0,
                    STAGE_MINUTE_Z + GEAR_FACE / 2 - MINUTE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * MINUTE_SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "minute_hand_seat",
                "type": "cylindrical",
                "xyz_mm": [
                    0,
                    0,
                    MINUTE_HAND_Z + MINUTE_HAND_H / 2 - MINUTE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * MINUTE_SHAFT_R,
                "depth_mm": MINUTE_HAND_H,
            },
        ],
        "hour_pipe": [
            {
                "name": "inner_bore",
                "type": "bore",
                "xyz_mm": [0, 0, HOUR_PIPE_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * HOUR_PIPE_INNER_R,
                "depth_mm": HOUR_PIPE_H,
            },
            {
                "name": "outer",
                "type": "shaft",
                "xyz_mm": [0, 0, HOUR_PIPE_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * HOUR_PIPE_OUTER_R,
                "depth_mm": HOUR_PIPE_H,
            },
            {
                "name": "hour_wheel_seat",
                "type": "cylindrical",
                "xyz_mm": [
                    0,
                    0,
                    STAGE_HOUR_Z + GEAR_FACE / 2 - HOUR_PIPE_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * HOUR_PIPE_OUTER_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "hour_hand_seat",
                "type": "cylindrical",
                "xyz_mm": [
                    0,
                    0,
                    HOUR_HAND_Z + HOUR_HAND_H / 2 - HOUR_PIPE_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * HOUR_PIPE_OUTER_R,
                "depth_mm": HOUR_HAND_H,
            },
        ],
        "intermediate_arbor": [
            {
                "name": "outer",
                "type": "shaft",
                "xyz_mm": [0, 0, INTERMEDIATE_SHAFT_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * INTERMEDIATE_SHAFT_R,
                "depth_mm": INTERMEDIATE_SHAFT_H,
            },
            {
                "name": "wheel_seat",
                "type": "cylindrical",
                "xyz_mm": [
                    0,
                    0,
                    STAGE_MINUTE_Z + GEAR_FACE / 2 - INTERMEDIATE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * INTERMEDIATE_SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "pinion_seat",
                "type": "cylindrical",
                "xyz_mm": [
                    0,
                    0,
                    STAGE_HOUR_Z + GEAR_FACE / 2 - INTERMEDIATE_SHAFT_Z,
                ],
                "axis": [0, 0, 1],
                "diameter_mm": 2 * INTERMEDIATE_SHAFT_R,
                "depth_mm": GEAR_FACE,
            },
        ],
        "minute_pinion": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0, 0, GEAR_FACE / 2],
                "axis": [0, 0, 1],
                "diameter_mm": MINUTE_SHAFT_PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "mesh",
                "type": "gear_mesh",
                "xyz_mm": [0, 0, GEAR_FACE / 2],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_MINUTE_PINION),
                "depth_mm": GEAR_FACE,
            },
        ],
        "intermediate_wheel": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0, 0, GEAR_FACE / 2],
                "axis": [0, 0, 1],
                "diameter_mm": INTERMEDIATE_PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "mesh",
                "type": "gear_mesh",
                "xyz_mm": [0, 0, GEAR_FACE / 2],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_INTERMEDIATE_WHEEL),
                "depth_mm": GEAR_FACE,
            },
        ],
        "intermediate_pinion": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0, 0, GEAR_FACE / 2],
                "axis": [0, 0, 1],
                "diameter_mm": INTERMEDIATE_PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "mesh",
                "type": "gear_mesh",
                "xyz_mm": [0, 0, GEAR_FACE / 2],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_INTERMEDIATE_PINION),
                "depth_mm": GEAR_FACE,
            },
        ],
        "hour_wheel": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0, 0, GEAR_FACE / 2],
                "axis": [0, 0, 1],
                "diameter_mm": HOUR_PIPE_PRESS_BORE_D,
                "depth_mm": GEAR_FACE,
            },
            {
                "name": "mesh",
                "type": "gear_mesh",
                "xyz_mm": [0, 0, GEAR_FACE / 2],
                "axis": [0, 0, 1],
                "pitch_radius_mm": pitch_r(Z_HOUR_WHEEL),
                "depth_mm": GEAR_FACE,
            },
        ],
        "hour_hand": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0, 0, HOUR_HAND_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": HOUR_PIPE_PRESS_BORE_D,
                "depth_mm": HOUR_HAND_H,
            }
        ],
        "minute_hand": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0, 0, MINUTE_HAND_H / 2],
                "axis": [0, 0, 1],
                "diameter_mm": MINUTE_SHAFT_PRESS_BORE_D,
                "depth_mm": MINUTE_HAND_H,
            }
        ],
    },
    "relations": [
        {
            "name": "minute_shaft_in_lower_bearing",
            "mate_type": "journal_bearing",
            "base_part": "central_lower_bearing",
            "base_port": "bore",
            "incoming_part": "minute_shaft",
            "incoming_port": "outer",
        },
        {
            "name": "hour_pipe_in_upper_bearing",
            "mate_type": "journal_bearing",
            "base_part": "central_upper_bearing",
            "base_port": "bore",
            "incoming_part": "hour_pipe",
            "incoming_port": "outer",
        },
        {
            "name": "minute_shaft_inside_hour_pipe",
            "mate_type": "journal_bearing",
            "base_part": "hour_pipe",
            "base_port": "inner_bore",
            "incoming_part": "minute_shaft",
            "incoming_port": "outer",
        },
        {
            "name": "intermediate_arbor_in_lower_bearing",
            "mate_type": "journal_bearing",
            "base_part": "intermediate_lower_bearing",
            "base_port": "bore",
            "incoming_part": "intermediate_arbor",
            "incoming_port": "outer",
        },
        {
            "name": "intermediate_arbor_in_upper_bearing",
            "mate_type": "journal_bearing",
            "base_part": "intermediate_upper_bearing",
            "base_port": "bore",
            "incoming_part": "intermediate_arbor",
            "incoming_port": "outer",
        },
        {
            "name": "minute_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "minute_shaft",
            "base_port": "minute_pinion_seat",
            "incoming_part": "minute_pinion",
            "incoming_port": "bore",
        },
        {
            "name": "intermediate_wheel_press_fit",
            "mate_type": "press_fit",
            "base_part": "intermediate_arbor",
            "base_port": "wheel_seat",
            "incoming_part": "intermediate_wheel",
            "incoming_port": "bore",
        },
        {
            "name": "intermediate_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "intermediate_arbor",
            "base_port": "pinion_seat",
            "incoming_part": "intermediate_pinion",
            "incoming_port": "bore",
        },
        {
            "name": "hour_wheel_press_fit",
            "mate_type": "press_fit",
            "base_part": "hour_pipe",
            "base_port": "hour_wheel_seat",
            "incoming_part": "hour_wheel",
            "incoming_port": "bore",
        },
        {
            "name": "hour_hand_press_fit",
            "mate_type": "press_fit",
            "base_part": "hour_pipe",
            "base_port": "hour_hand_seat",
            "incoming_part": "hour_hand",
            "incoming_port": "hub_bore",
        },
        {
            "name": "minute_hand_press_fit",
            "mate_type": "press_fit",
            "base_part": "minute_shaft",
            "base_port": "minute_hand_seat",
            "incoming_part": "minute_hand",
            "incoming_port": "hub_bore",
        },
        {
            "name": "minute_stage_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "minute_pinion",
            "base_port": "mesh",
            "incoming_part": "intermediate_wheel",
            "incoming_port": "mesh",
            "separation_axis": "+x",
        },
        {
            "name": "hour_stage_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "hour_wheel",
            "base_port": "mesh",
            "incoming_part": "intermediate_pinion",
            "incoming_port": "mesh",
            "separation_axis": "+x",
        },
    ],
    "motion_joints": [],
    "transmissions": [
        {
            "name": "minute_to_intermediate",
            "type": "gear_external",
            "driving_link": "minute_pinion",
            "driven_link": "intermediate_wheel",
            "ratio": 0,
        },
        {
            "name": "compound_intermediate_arbor",
            "type": "compound_1to1",
            "driving_link": "intermediate_wheel",
            "driven_link": "intermediate_pinion",
            "ratio": 0,
        },
        {
            "name": "intermediate_to_hour",
            "type": "gear_external",
            "driving_link": "intermediate_pinion",
            "driven_link": "hour_wheel",
            "ratio": 0,
        },
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("openwork_mechanical_watch_movement")

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

    def make_hand(length, hub_outer_r, bore_r, width, tip_width, thickness):
        with BuildSketch(Plane.XY) as hand_sketch:
            Circle(hub_outer_r)
            Polygon(
                (-width / 2.0, hub_outer_r * 0.25),
                (width / 2.0, hub_outer_r * 0.25),
                (tip_width / 2.0, length),
                (-tip_width / 2.0, length),
            )
            Polygon(
                (-width * 0.55, -hub_outer_r * 0.15),
                (width * 0.55, -hub_outer_r * 0.15),
                (width * 0.25, -hub_outer_r * 1.8),
                (-width * 0.25, -hub_outer_r * 1.8),
            )
            Circle(bore_r, mode=Mode.SUBTRACT)
        return extrude(hand_sketch.sketch, amount=thickness)

    # Openwork lower plate: annular rim, center pads, and crossed spokes.
    base_ring = annulus(MOVEMENT_OUTER_R, MOVEMENT_INNER_R, BASE_H)

    center_pad = Cylinder(
        4.2,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    intermediate_pad = Cylinder(
        4.2,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((INTERMEDIATE_X, INTERMEDIATE_Y, 0)))

    horizontal_spoke = Box(
        2.0 * MOVEMENT_OUTER_R,
        3.0,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((INTERMEDIATE_X / 2.0, 0, 0)))

    vertical_spoke = Box(
        3.0,
        2.0 * MOVEMENT_OUTER_R,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    diagonal_length = 2.0 * MOVEMENT_OUTER_R
    diagonal_spoke_1 = Box(
        diagonal_length,
        2.2,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((INTERMEDIATE_X / 2.0, 0, 0), (0, 0, 45)))

    diagonal_spoke_2 = Box(
        diagonal_length,
        2.2,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((INTERMEDIATE_X / 2.0, 0, 0), (0, 0, -45)))

    baseplate = (
        base_ring
        + center_pad
        + intermediate_pad
        + horizontal_spoke
        + vertical_spoke
        + diagonal_spoke_1
        + diagonal_spoke_2
    )
    a.add(baseplate, "baseplate|dof=fixed")

    # Lower fixed journal bearings.
    central_lower_bearing = annulus(
        LOWER_BEARING_OUTER_R,
        MINUTE_SHAFT_RUNNING_R,
        LOWER_BEARING_H,
    ).moved(Location((0, 0, LOWER_BEARING_Z)))
    a.add(
        central_lower_bearing,
        "central_lower_bearing|dof=fixed|mount=baseplate",
    )

    intermediate_lower_bearing = annulus(
        INTERMEDIATE_BEARING_OUTER_R,
        INTERMEDIATE_RUNNING_R,
        LOWER_BEARING_H,
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                LOWER_BEARING_Z,
            )
        )
    )
    a.add(
        intermediate_lower_bearing,
        "intermediate_lower_bearing|dof=fixed|mount=baseplate",
    )

    # Posts supporting the upper open bridge.
    bridge_post_low = Cylinder(
        BRIDGE_POST_R,
        BRIDGE_POST_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                INTERMEDIATE_X / 2.0,
                -BRIDGE_SUPPORT_Y,
                BRIDGE_POST_Z,
            )
        )
    )
    a.add(
        bridge_post_low,
        "bridge_post_low|dof=fixed|mount=baseplate",
    )

    bridge_post_high = Cylinder(
        BRIDGE_POST_R,
        BRIDGE_POST_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                INTERMEDIATE_X / 2.0,
                BRIDGE_SUPPORT_Y,
                BRIDGE_POST_Z,
            )
        )
    )
    a.add(
        bridge_post_high,
        "bridge_post_high|dof=fixed|mount=baseplate",
    )

    # Upper bridge is a cross-shaped open bearing carrier.
    bridge_horizontal = Box(
        INTERMEDIATE_X + 10.0,
        4.0,
        BRIDGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((INTERMEDIATE_X / 2.0, 0, 0)))

    bridge_vertical = Box(
        4.0,
        2.0 * BRIDGE_SUPPORT_Y,
        BRIDGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((INTERMEDIATE_X / 2.0, 0, 0)))

    bridge_center_pad = Cylinder(
        4.2,
        BRIDGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    bridge_intermediate_pad = Cylinder(
        4.2,
        BRIDGE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((INTERMEDIATE_X, 0, 0)))

    upper_bridge_local = (
        bridge_horizontal
        + bridge_vertical
        + bridge_center_pad
        + bridge_intermediate_pad
    )

    bridge_center_hole = Cylinder(
        HOUR_PIPE_RUNNING_R,
        BRIDGE_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, -1.0)))

    bridge_intermediate_hole = Cylinder(
        INTERMEDIATE_RUNNING_R,
        BRIDGE_H + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((INTERMEDIATE_X, 0, -1.0)))

    upper_bridge = (
        upper_bridge_local
        - bridge_center_hole
        - bridge_intermediate_hole
    ).moved(Location((0, 0, BRIDGE_Z)))

    a.add(
        upper_bridge,
        "upper_bridge|dof=fixed|mount=bridge_post_low,bridge_post_high",
    )

    # Upper bearings touch the underside of the upper bridge.
    central_upper_bearing = annulus(
        UPPER_CENTER_BEARING_OUTER_R,
        HOUR_PIPE_RUNNING_R,
        UPPER_BEARING_H,
    ).moved(Location((0, 0, UPPER_BEARING_Z)))
    a.add(
        central_upper_bearing,
        "central_upper_bearing|dof=fixed|mount=upper_bridge",
    )

    intermediate_upper_bearing = annulus(
        INTERMEDIATE_BEARING_OUTER_R,
        INTERMEDIATE_RUNNING_R,
        UPPER_BEARING_H,
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                UPPER_BEARING_Z,
            )
        )
    )
    a.add(
        intermediate_upper_bearing,
        "intermediate_upper_bearing|dof=fixed|mount=upper_bridge",
    )

    # Independently rotating central minute shaft.
    minute_shaft = Cylinder(
        MINUTE_SHAFT_R,
        MINUTE_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0, 0, MINUTE_SHAFT_Z)))
    a.add(
        minute_shaft,
        "minute_shaft|dof=spin|spin_axis=z|driver=True|"
        "mount=central_lower_bearing,hour_pipe",
    )

    # Intermediate arbor supported by both of its bearings.
    intermediate_arbor = Cylinder(
        INTERMEDIATE_SHAFT_R,
        INTERMEDIATE_SHAFT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                INTERMEDIATE_SHAFT_Z,
            )
        )
    )
    a.add(
        intermediate_arbor,
        "intermediate_arbor|dof=spin|spin_axis=z|"
        "mount=intermediate_lower_bearing,intermediate_upper_bearing",
    )

    # Lower minute stage: 10-tooth central pinion to 30-tooth wheel.
    minute_pinion = make_gear(
        M,
        Z_MINUTE_PINION,
        GEAR_FACE,
        MINUTE_SHAFT_PRESS_BORE_D,
    ).moved(Location((0, 0, STAGE_MINUTE_Z)))
    a.add(
        minute_pinion,
        "minute_pinion|dof=spin|spin_axis=z|mesh_id=minute_stage|"
        "mount=minute_shaft",
    )

    intermediate_wheel = make_gear(
        M,
        Z_INTERMEDIATE_WHEEL,
        GEAR_FACE,
        INTERMEDIATE_PRESS_BORE_D,
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                STAGE_MINUTE_Z,
            ),
            (0, 0, PHASE_INTERMEDIATE_WHEEL_DEG),
        )
    )
    a.add(
        intermediate_wheel,
        "intermediate_wheel|dof=spin|spin_axis=z|mesh_id=minute_stage|"
        "mount=intermediate_arbor",
    )

    # Hour pipe begins immediately above the minute pinion and runs through
    # the upper center bearing.
    hour_pipe = annulus(
        HOUR_PIPE_OUTER_R,
        HOUR_PIPE_INNER_R,
        HOUR_PIPE_H,
    ).moved(Location((0, 0, HOUR_PIPE_Z)))
    a.add(
        hour_pipe,
        "hour_pipe|dof=spin|spin_axis=z|"
        "mount=central_upper_bearing,minute_shaft",
    )

    # Upper hour stage: 8-tooth compound pinion to 32-tooth hour wheel.
    intermediate_pinion = make_gear(
        M,
        Z_INTERMEDIATE_PINION,
        GEAR_FACE,
        INTERMEDIATE_PRESS_BORE_D,
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                STAGE_HOUR_Z,
            ),
            (0, 0, PHASE_INTERMEDIATE_PINION_DEG),
        )
    )
    a.add(
        intermediate_pinion,
        "intermediate_pinion|dof=spin|spin_axis=z|mesh_id=hour_stage|"
        "mount=intermediate_arbor",
    )

    hour_wheel = make_gear(
        M,
        Z_HOUR_WHEEL,
        GEAR_FACE,
        HOUR_PIPE_PRESS_BORE_D,
    ).moved(
        Location(
            (0, 0, STAGE_HOUR_Z),
            (0, 0, PHASE_HOUR_WHEEL_DEG),
        )
    )
    a.add(
        hour_wheel,
        "hour_wheel|dof=spin|spin_axis=z|mesh_id=hour_stage|"
        "mount=hour_pipe",
    )

    # Four dial posts support the fixed chapter ring.
    dial_post_positions = [
        (DIAL_POST_R, 0),
        (-DIAL_POST_R, 0),
        (0, DIAL_POST_R),
        (0, -DIAL_POST_R),
    ]
    dial_post_names = []

    for index, (px, py) in enumerate(dial_post_positions):
        post_name = f"dial_post_{index + 1}"
        dial_post_names.append(post_name)
        post = Cylinder(
            DIAL_POST_OUTER_R,
            DIAL_POST_H,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, DIAL_POST_Z)))
        a.add(post, f"{post_name}|dof=fixed|mount=baseplate")

    chapter_ring = annulus(
        CHAPTER_OUTER_R,
        CHAPTER_INNER_R,
        CHAPTER_RING_H,
    ).moved(Location((0, 0, CHAPTER_RING_Z)))
    a.add(
        chapter_ring,
        "chapter_ring|dof=fixed|mount=" + ",".join(dial_post_names),
    )

    # Raised hour indices are fixed to the chapter ring.
    marker_radial_center = (
        CHAPTER_INNER_R + CHAPTER_OUTER_R
    ) / 2.0
    marker_h = 0.45
    marker_z = CHAPTER_RING_Z + CHAPTER_RING_H

    for index in range(12):
        angle_deg = index * 30.0
        angle_rad = math.radians(angle_deg)
        mx = marker_radial_center * math.sin(angle_rad)
        my = marker_radial_center * math.cos(angle_rad)
        marker_length = 2.1 if index % 3 == 0 else 1.4
        marker_width = 0.65 if index % 3 == 0 else 0.45

        marker = Box(
            marker_width,
            marker_length,
            marker_h,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(
            Location(
                (mx, my, marker_z),
                (0, 0, -angle_deg),
            )
        )
        marker_name = f"hour_marker_{index + 1}"
        a.add(
            marker,
            f"{marker_name}|dof=fixed|mount=chapter_ring",
        )

    # Press-fit visible hands. They remain passive fixed accessories of their
    # respective independently rotating members.
    hour_hand = make_hand(
        HOUR_HAND_LENGTH,
        hub_outer_r=2.7,
        bore_r=HOUR_PIPE_OUTER_R - PRESS_INTERFERENCE,
        width=1.3,
        tip_width=0.7,
        thickness=HOUR_HAND_H,
    ).moved(Location((0, 0, HOUR_HAND_Z)))
    a.add(
        hour_hand,
        "hour_hand|dof=fixed|mount=hour_pipe",
    )

    minute_hand = make_hand(
        MINUTE_HAND_LENGTH,
        hub_outer_r=2.2,
        bore_r=MINUTE_SHAFT_R - PRESS_INTERFERENCE,
        width=1.0,
        tip_width=0.45,
        thickness=MINUTE_HAND_H,
    ).moved(Location((0, 0, MINUTE_HAND_Z)))
    a.add(
        minute_hand,
        "minute_hand|dof=fixed|mount=minute_shaft",
    )

    return a.build()