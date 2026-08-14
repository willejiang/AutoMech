import math
from build123d import Rotation

# ---------------------------------------------------------------------------
# ENGINEERING ARITHMETIC — all mechanism hardpoints are solved before geometry
# ---------------------------------------------------------------------------

# This pumpjack has no gears. Its motion path is:
# driven crankshaft -> press-fit crank disk -> crank pin -> pitman
# -> walking beam -> output pin -> vertically guided polished rod.

BASE_L = 190.0
BASE_W = 80.0
BASE_H = 8.0
BASE_X = 10.0

SHAFT_CX = -45.0
SHAFT_CY = 0.0
SHAFT_CZ = 45.0
SHAFT_R = 4.0
SHAFT_LEN = 72.0

CRANK_THROW = 18.0
CRANK_ANGLE_DEG = 35.0
CRANK_ANGLE = math.radians(CRANK_ANGLE_DEG)

# Exact crank-pin hardpoint, derived from shaft center and crank throw.
CRANK_PIN_X = SHAFT_CX + CRANK_THROW * math.cos(CRANK_ANGLE)
CRANK_PIN_Y = -12.0
CRANK_PIN_Z = SHAFT_CZ + CRANK_THROW * math.sin(CRANK_ANGLE)

BEAM_PIVOT_X = 20.0
BEAM_PIVOT_Y = -7.0
BEAM_PIVOT_Z = 90.0
BEAM_ANGLE_DEG = 8.0
BEAM_ANGLE = math.radians(BEAM_ANGLE_DEG)
BEAM_LEFT_R = 45.0
BEAM_RIGHT_R = 55.0

# Walking-beam pin hardpoints derived from pivot, arm lengths, and beam angle.
PITMAN_BEAM_X = BEAM_PIVOT_X - BEAM_LEFT_R * math.cos(BEAM_ANGLE)
PITMAN_BEAM_Y = -12.0
PITMAN_BEAM_Z = BEAM_PIVOT_Z - BEAM_LEFT_R * math.sin(BEAM_ANGLE)

OUTPUT_PIN_X = BEAM_PIVOT_X + BEAM_RIGHT_R * math.cos(BEAM_ANGLE)
OUTPUT_PIN_Y = -12.0
OUTPUT_PIN_Z = BEAM_PIVOT_Z + BEAM_RIGHT_R * math.sin(BEAM_ANGLE)

# Pitman centerline is solved directly between its two pin hardpoints.
PITMAN_DX = PITMAN_BEAM_X - CRANK_PIN_X
PITMAN_DZ = PITMAN_BEAM_Z - CRANK_PIN_Z
PITMAN_LENGTH = math.hypot(PITMAN_DX, PITMAN_DZ)
PITMAN_ANGLE = math.atan2(PITMAN_DZ, PITMAN_DX)
PITMAN_MID_X = (CRANK_PIN_X + PITMAN_BEAM_X) / 2.0
PITMAN_MID_Y = CRANK_PIN_Y
PITMAN_MID_Z = (CRANK_PIN_Z + PITMAN_BEAM_Z) / 2.0

# Pin and running-fit arithmetic.
CRANK_PIN_R = 2.50
LINK_PIN_R = 2.50
BEAM_PIVOT_PIN_R = 3.00

RUNNING_CLEARANCE = 0.05
PRESS_INTERFERENCE = 0.005

PITMAN_BORE_R = CRANK_PIN_R + RUNNING_CLEARANCE
BEAM_JOINT_BORE_R = LINK_PIN_R - PRESS_INTERFERENCE
BEAM_PIVOT_BORE_R = BEAM_PIVOT_PIN_R + RUNNING_CLEARANCE
OUTPUT_EYE_BORE_R = LINK_PIN_R + RUNNING_CLEARANCE

BEARING_BORE_R = SHAFT_R + RUNNING_CLEARANCE
DISK_SHAFT_BORE_R = SHAFT_R - PRESS_INTERFERENCE
HAND_ARM_BORE_R = SHAFT_R - PRESS_INTERFERENCE

CRANK_DISK_R = CRANK_THROW + 7.0
CRANK_DISK_T = 10.0
CRANK_DISK_Y = 0.0

PITMAN_T = 4.0
PITMAN_EYE_R = 6.5
PITMAN_WEB_H = 7.0

BEAM_T = 4.0
BEAM_EYE_R = 8.0
BEAM_WEB_H = 10.0

PIN_SPAN_MIN_Y = -15.0
PIN_SPAN_MAX_Y = -4.0
LINK_PIN_LEN = PIN_SPAN_MAX_Y - PIN_SPAN_MIN_Y
LINK_PIN_CENTER_Y = (PIN_SPAN_MIN_Y + PIN_SPAN_MAX_Y) / 2.0

# Crankshaft support stations.
CRANK_BEARING_Y_1 = -23.0
CRANK_BEARING_Y_2 = 23.0
CRANK_BEARING_T = 8.0
CRANK_BEARING_OUTER_R = 8.0

PEDESTAL_TOP_Z = SHAFT_CZ - CRANK_BEARING_OUTER_R
PEDESTAL_H = PEDESTAL_TOP_Z - BASE_H
PEDESTAL_Z = BASE_H

# Beam support and pivot geometry.
BEAM_SUPPORT_OUTER_Y_1 = -18.0
BEAM_SUPPORT_OUTER_Y_2 = 4.0
BEAM_SUPPORT_W = 12.0
BEAM_SUPPORT_T = 6.0
BEAM_SUPPORT_TOP_Z = BEAM_PIVOT_Z - 8.0
BEAM_SUPPORT_H = BEAM_SUPPORT_TOP_Z - BASE_H

BEAM_PIVOT_PIN_MIN_Y = -21.0
BEAM_PIVOT_PIN_MAX_Y = 7.0
BEAM_PIVOT_PIN_LEN = BEAM_PIVOT_PIN_MAX_Y - BEAM_PIVOT_PIN_MIN_Y
BEAM_PIVOT_PIN_CENTER_Y = (
    BEAM_PIVOT_PIN_MIN_Y + BEAM_PIVOT_PIN_MAX_Y
) / 2.0

# Output rod and its true anti-rotation square guides.
OUTPUT_ROD_W = 6.0
OUTPUT_ROD_D = 6.0
OUTPUT_ROD_BOTTOM_Z = 12.0
OUTPUT_EYE_R = 7.0
OUTPUT_ROD_BODY_TOP_Z = OUTPUT_PIN_Z
OUTPUT_ROD_BODY_H = OUTPUT_ROD_BODY_TOP_Z - OUTPUT_ROD_BOTTOM_Z

GUIDE_CLEARANCE = 0.06
GUIDE_HOLE_W = OUTPUT_ROD_W + 2.0 * GUIDE_CLEARANCE
GUIDE_HOLE_D = OUTPUT_ROD_D + 2.0 * GUIDE_CLEARANCE
GUIDE_OUTER_W = 16.0
GUIDE_OUTER_D = 16.0
GUIDE_H = 8.0
LOWER_GUIDE_Z = 24.0
UPPER_GUIDE_Z = 50.0
GUIDE_POST_Y = 10.0
GUIDE_POST_W = 10.0
GUIDE_POST_D = 10.0
GUIDE_POST_H = 66.0

# Hand crank stations, kept beyond the positive-Y shaft bearing.
HAND_ARM_Y = 31.0
HAND_ARM_LENGTH = 24.0
HAND_ARM_T = 4.0
HAND_ARM_EYE_R = 7.0
HAND_ARM_WEB_H = 7.0
HANDLE_X = SHAFT_CX
HANDLE_Z = SHAFT_CZ + HAND_ARM_LENGTH
HANDLE_SPINDLE_R = 2.0
HANDLE_SPINDLE_Y = 39.0
HANDLE_SPINDLE_LEN = 16.0
HANDLE_GRIP_R = 5.0
HANDLE_GRIP_LEN = 11.0
HANDLE_GRIP_Y = 41.5
HANDLE_GRIP_BORE_R = HANDLE_SPINDLE_R - PRESS_INTERFERENCE


MECHANISM = {
    "name": "open_frame_hand_cranked_pumpjack",
    "output_link": "polished_output_rod",
    "watch_links": [
        "crankshaft",
        "crank_disk",
        "crank_pin",
        "pitman",
        "walking_beam",
        "polished_output_rod",
    ],
    "ports_by_link": {
        "crankshaft": [
            {
                "name": "rotation_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": SHAFT_LEN,
            },
            {
                "name": "disk_seat",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * SHAFT_R,
                "depth_mm": CRANK_DISK_T,
            },
        ],
        "crank_disk": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * DISK_SHAFT_BORE_R,
                "depth_mm": CRANK_DISK_T,
            },
            {
                "name": "crank_pin_seat",
                "type": "bore",
                "xyz_mm": [
                    CRANK_THROW * math.cos(CRANK_ANGLE),
                    CRANK_THROW * math.sin(CRANK_ANGLE),
                    0.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (CRANK_PIN_R - PRESS_INTERFERENCE),
                "depth_mm": CRANK_DISK_T,
            },
        ],
        "crank_pin": [
            {
                "name": "pin_shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * CRANK_PIN_R,
                "depth_mm": LINK_PIN_LEN,
            }
        ],
        "pitman": [
            {
                "name": "crank_end_bore",
                "type": "bore",
                "xyz_mm": [-PITMAN_LENGTH / 2.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PITMAN_BORE_R,
                "depth_mm": PITMAN_T,
            },
            {
                "name": "beam_end_bore",
                "type": "bore",
                "xyz_mm": [PITMAN_LENGTH / 2.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * PITMAN_BORE_R,
                "depth_mm": PITMAN_T,
            },
        ],
        "walking_beam": [
            {
                "name": "pitman_pin_bore",
                "type": "bore",
                "xyz_mm": [-BEAM_LEFT_R, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * BEAM_JOINT_BORE_R,
                "depth_mm": BEAM_T,
            },
            {
                "name": "pivot_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * BEAM_PIVOT_BORE_R,
                "depth_mm": BEAM_T,
            },
            {
                "name": "output_pin_bore",
                "type": "bore",
                "xyz_mm": [BEAM_RIGHT_R, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * BEAM_JOINT_BORE_R,
                "depth_mm": BEAM_T,
            },
        ],
        "pitman_beam_pin": [
            {
                "name": "pin_shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * LINK_PIN_R,
                "depth_mm": LINK_PIN_LEN,
            }
        ],
        "beam_pivot_pin": [
            {
                "name": "pivot_shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * BEAM_PIVOT_PIN_R,
                "depth_mm": BEAM_PIVOT_PIN_LEN,
            }
        ],
        "beam_output_pin": [
            {
                "name": "pin_shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * LINK_PIN_R,
                "depth_mm": LINK_PIN_LEN,
            }
        ],
        "polished_output_rod": [
            {
                "name": "beam_pin_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, OUTPUT_PIN_Z],
                "axis": [0.0, 1.0, 0.0],
                "diameter_mm": 2.0 * OUTPUT_EYE_BORE_R,
                "depth_mm": PITMAN_T,
            },
            {
                "name": "guided_axis",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, 32.0],
                "axis": [0.0, 0.0, 1.0],
                "depth_mm": OUTPUT_ROD_BODY_H,
            },
        ],
    },
    "relations": [
        {
            "name": "shaft_in_negative_bearing",
            "mate_type": "journal_bearing",
            "base_part": "negative_shaft_bearing",
            "base_port": "bearing_bore",
            "incoming_part": "crankshaft",
            "incoming_port": "rotation_axis",
        },
        {
            "name": "shaft_in_positive_bearing",
            "mate_type": "journal_bearing",
            "base_part": "positive_shaft_bearing",
            "base_port": "bearing_bore",
            "incoming_part": "crankshaft",
            "incoming_port": "rotation_axis",
        },
        {
            "name": "disk_pressed_on_shaft",
            "mate_type": "press_fit",
            "base_part": "crankshaft",
            "base_port": "disk_seat",
            "incoming_part": "crank_disk",
            "incoming_port": "shaft_bore",
        },
        {
            "name": "crank_pin_pressed_in_disk",
            "mate_type": "press_fit",
            "base_part": "crank_disk",
            "base_port": "crank_pin_seat",
            "incoming_part": "crank_pin",
            "incoming_port": "pin_shaft",
        },
        {
            "name": "crank_pin_to_pitman_closure",
            "mate_type": "revolute",
            "base_part": "crank_pin",
            "base_port": "pin_shaft",
            "incoming_part": "pitman",
            "incoming_port": "crank_end_bore",
        },
        {
            "name": "pitman_to_walking_beam_closure",
            "mate_type": "revolute",
            "base_part": "pitman_beam_pin",
            "base_port": "pin_shaft",
            "incoming_part": "pitman",
            "incoming_port": "beam_end_bore",
        },
        {
            "name": "walking_beam_fixed_pivot",
            "mate_type": "revolute",
            "base_part": "beam_pivot_pin",
            "base_port": "pivot_shaft",
            "incoming_part": "walking_beam",
            "incoming_port": "pivot_bore",
        },
        {
            "name": "beam_to_output_rod_closure",
            "mate_type": "revolute",
            "base_part": "beam_output_pin",
            "base_port": "pin_shaft",
            "incoming_part": "polished_output_rod",
            "incoming_port": "beam_pin_bore",
        },
    ],
    "motion_joints": [
        {
            "name": "driven_crankshaft_hinge",
            "parent": "",
            "child": "crankshaft",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "crank_disk_hinge",
            "parent": "",
            "child": "crank_disk",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "walking_beam_hinge",
            "parent": "",
            "child": "walking_beam",
            "type": "hinge",
            "axis": [0.0, 1.0, 0.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "vertical_guided_output",
            "parent": "",
            "child": "polished_output_rod",
            "type": "slide",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
    ],
    "transmissions": [
        {
            "name": "shaft_to_crank_disk_press_drive",
            "type": "compound_1to1",
            "driving_link": "crankshaft",
            "driven_link": "crank_disk",
            "ratio": 1.0,
        }
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("open_frame_hand_cranked_pumpjack")

    def y_cylinder(radius, length):
        """Centered cylinder whose final local axis is global Y after placement."""
        return Cylinder(radius, length).moved(Location((0, 0, 0), (90, 0, 0)))

    def y_annulus(outer_r, inner_r, length):
        outer = Cylinder(outer_r, length)
        cutter = Cylinder(inner_r, length + 2.0)
        return (outer - cutter).moved(Location((0, 0, 0), (90, 0, 0)))

    def local_link(length, thickness, eye_r, web_h, bore_r_left,
                   bore_r_right, center_bore_r=None):
        """Straight two-eye link along local X with all pin bores along local Y."""
        left_eye = y_cylinder(eye_r, thickness).moved(
            Location((-length / 2.0, 0.0, 0.0))
        )
        right_eye = y_cylinder(eye_r, thickness).moved(
            Location((length / 2.0, 0.0, 0.0))
        )
        web = Box(length, thickness, web_h)
        link = left_eye + right_eye + web

        left_cut = y_cylinder(bore_r_left, thickness + 2.0).moved(
            Location((-length / 2.0, 0.0, 0.0))
        )
        right_cut = y_cylinder(bore_r_right, thickness + 2.0).moved(
            Location((length / 2.0, 0.0, 0.0))
        )
        link = link - left_cut - right_cut

        if center_bore_r is not None:
            center_eye = y_cylinder(eye_r, thickness)
            center_cut = y_cylinder(center_bore_r, thickness + 2.0)
            link = link + center_eye - center_cut

        return link

    def beam_shape():
        left_eye = y_cylinder(BEAM_EYE_R, BEAM_T).moved(
            Location((-BEAM_LEFT_R, 0.0, 0.0))
        )
        pivot_eye = y_cylinder(BEAM_EYE_R, BEAM_T)
        right_eye = y_cylinder(BEAM_EYE_R, BEAM_T).moved(
            Location((BEAM_RIGHT_R, 0.0, 0.0))
        )
        web = Box(BEAM_LEFT_R + BEAM_RIGHT_R, BEAM_T, BEAM_WEB_H).moved(
            Location(((BEAM_RIGHT_R - BEAM_LEFT_R) / 2.0, 0.0, 0.0))
        )

        beam = left_eye + pivot_eye + right_eye + web

        left_cut = y_cylinder(BEAM_JOINT_BORE_R, BEAM_T + 2.0).moved(
            Location((-BEAM_LEFT_R, 0.0, 0.0))
        )
        pivot_cut = y_cylinder(BEAM_PIVOT_BORE_R, BEAM_T + 2.0)
        right_cut = y_cylinder(BEAM_JOINT_BORE_R, BEAM_T + 2.0).moved(
            Location((BEAM_RIGHT_R, 0.0, 0.0))
        )
        return beam - left_cut - pivot_cut - right_cut

    # Stable ground-contacting base.
    base = Box(
        BASE_L,
        BASE_W,
        BASE_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((BASE_X, 0.0, 0.0)))
    a.add(base, "base|dof=fixed")

    # Open crankshaft pedestals.
    negative_pedestal = Box(
        16.0,
        CRANK_BEARING_T,
        PEDESTAL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((SHAFT_CX, CRANK_BEARING_Y_1, PEDESTAL_Z)))
    a.add(negative_pedestal, "negative_shaft_pedestal|dof=fixed|mount=base")

    positive_pedestal = Box(
        16.0,
        CRANK_BEARING_T,
        PEDESTAL_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((SHAFT_CX, CRANK_BEARING_Y_2, PEDESTAL_Z)))
    a.add(positive_pedestal, "positive_shaft_pedestal|dof=fixed|mount=base")

    negative_bearing = y_annulus(
        CRANK_BEARING_OUTER_R, BEARING_BORE_R, CRANK_BEARING_T
    ).moved(Location((SHAFT_CX, CRANK_BEARING_Y_1, SHAFT_CZ)))
    a.add(
        negative_bearing,
        "negative_shaft_bearing|dof=fixed|mount=negative_shaft_pedestal",
    )

    positive_bearing = y_annulus(
        CRANK_BEARING_OUTER_R, BEARING_BORE_R, CRANK_BEARING_T
    ).moved(Location((SHAFT_CX, CRANK_BEARING_Y_2, SHAFT_CZ)))
    a.add(
        positive_bearing,
        "positive_shaft_bearing|dof=fixed|mount=positive_shaft_pedestal",
    )

    # Only driven body.
    crankshaft = Cylinder(SHAFT_R, SHAFT_LEN).moved(
        Location((SHAFT_CX, SHAFT_CY, SHAFT_CZ), (90, 0, 0))
    )
    a.add(
        crankshaft,
        "crankshaft|dof=spin|driver=True|spin_axis=z|"
        "mount=negative_shaft_bearing,positive_shaft_bearing",
    )

    # Crank disk built locally in XY, then rotated so its axis is global Y.
    disk_blank = Cylinder(CRANK_DISK_R, CRANK_DISK_T)
    disk_shaft_cut = Cylinder(DISK_SHAFT_BORE_R, CRANK_DISK_T + 2.0)
    disk_pin_cut = Cylinder(
        CRANK_PIN_R - PRESS_INTERFERENCE, CRANK_DISK_T + 2.0
    ).moved(
        Location(
            (
                CRANK_THROW * math.cos(CRANK_ANGLE),
                CRANK_THROW * math.sin(CRANK_ANGLE),
                0.0,
            )
        )
    )
    crank_disk = (disk_blank - disk_shaft_cut - disk_pin_cut).moved(
        Location((SHAFT_CX, CRANK_DISK_Y, SHAFT_CZ), (90, 0, 0))
    )
    a.add(
        crank_disk,
        "crank_disk|dof=spin|spin_axis=z|mount=crankshaft",
    )

    crank_pin = Cylinder(CRANK_PIN_R, LINK_PIN_LEN).moved(
        Location(
            (CRANK_PIN_X, LINK_PIN_CENTER_Y, CRANK_PIN_Z),
            (90, 0, 0),
        )
    )
    a.add(crank_pin, "crank_pin|dof=fixed|mount=crank_disk")

    # Pitman: local eye coordinates are exactly +/- PITMAN_LENGTH/2.
    pitman_local = local_link(
        PITMAN_LENGTH,
        PITMAN_T,
        PITMAN_EYE_R,
        PITMAN_WEB_H,
        PITMAN_BORE_R,
        PITMAN_BORE_R,
    )
    pitman = pitman_local.moved(
        Location(
            (PITMAN_MID_X, PITMAN_MID_Y, PITMAN_MID_Z),
            (0.0, -math.degrees(PITMAN_ANGLE), 0.0),
        )
    )
    a.add(
        pitman,
        "pitman|dof=free|mount=crank_pin,pitman_beam_pin",
    )

    # Open twin-post walking-beam support.
    support_1 = Box(
        BEAM_SUPPORT_W,
        BEAM_SUPPORT_T,
        BEAM_SUPPORT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (BEAM_PIVOT_X, BEAM_SUPPORT_OUTER_Y_1, BASE_H)
        )
    )
    a.add(support_1, "negative_beam_support|dof=fixed|mount=base")

    support_2 = Box(
        BEAM_SUPPORT_W,
        BEAM_SUPPORT_T,
        BEAM_SUPPORT_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (BEAM_PIVOT_X, BEAM_SUPPORT_OUTER_Y_2, BASE_H)
        )
    )
    a.add(support_2, "positive_beam_support|dof=fixed|mount=base")

    negative_pivot_bearing = y_annulus(
        8.0, BEAM_PIVOT_PIN_R + RUNNING_CLEARANCE, BEAM_SUPPORT_T
    ).moved(
        Location(
            (BEAM_PIVOT_X, BEAM_SUPPORT_OUTER_Y_1, BEAM_PIVOT_Z)
        )
    )
    a.add(
        negative_pivot_bearing,
        "negative_pivot_bearing|dof=fixed|mount=negative_beam_support",
    )

    positive_pivot_bearing = y_annulus(
        8.0, BEAM_PIVOT_PIN_R + RUNNING_CLEARANCE, BEAM_SUPPORT_T
    ).moved(
        Location(
            (BEAM_PIVOT_X, BEAM_SUPPORT_OUTER_Y_2, BEAM_PIVOT_Z)
        )
    )
    a.add(
        positive_pivot_bearing,
        "positive_pivot_bearing|dof=fixed|mount=positive_beam_support",
    )

    beam_pivot_pin = Cylinder(
        BEAM_PIVOT_PIN_R, BEAM_PIVOT_PIN_LEN
    ).moved(
        Location(
            (
                BEAM_PIVOT_X,
                BEAM_PIVOT_PIN_CENTER_Y,
                BEAM_PIVOT_Z,
            ),
            (90, 0, 0),
        )
    )
    a.add(
        beam_pivot_pin,
        "beam_pivot_pin|dof=fixed|"
        "mount=negative_pivot_bearing,positive_pivot_bearing",
    )

    walking_beam = beam_shape().moved(
        Location(
            (BEAM_PIVOT_X, BEAM_PIVOT_Y, BEAM_PIVOT_Z),
            (0.0, -BEAM_ANGLE_DEG, 0.0),
        )
    )
    a.add(
        walking_beam,
        "walking_beam|dof=free|mount=beam_pivot_pin",
    )

    pitman_beam_pin = Cylinder(LINK_PIN_R, LINK_PIN_LEN).moved(
        Location(
            (PITMAN_BEAM_X, LINK_PIN_CENTER_Y, PITMAN_BEAM_Z),
            (90, 0, 0),
        )
    )
    a.add(
        pitman_beam_pin,
        "pitman_beam_pin|dof=fixed|mount=walking_beam",
    )

    beam_output_pin = Cylinder(LINK_PIN_R, LINK_PIN_LEN).moved(
        Location(
            (OUTPUT_PIN_X, LINK_PIN_CENTER_Y, OUTPUT_PIN_Z),
            (90, 0, 0),
        )
    )
    a.add(
        beam_output_pin,
        "beam_output_pin|dof=fixed|mount=walking_beam",
    )

    # Square polished rod: the square section gives the guides real
    # anti-rotation geometry, so the intended DOF is a true vertical slide.
    rod_body = Box(
        OUTPUT_ROD_W,
        OUTPUT_ROD_D,
        OUTPUT_ROD_BODY_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, OUTPUT_ROD_BOTTOM_Z)))

    output_eye = y_cylinder(OUTPUT_EYE_R, PITMAN_T).moved(
        Location((0.0, 0.0, OUTPUT_PIN_Z))
    )
    output_eye_cut = y_cylinder(
        OUTPUT_EYE_BORE_R, PITMAN_T + 2.0
    ).moved(Location((0.0, 0.0, OUTPUT_PIN_Z)))

    output_rod_local = rod_body + output_eye - output_eye_cut
    polished_output_rod = output_rod_local.moved(
        Location((OUTPUT_PIN_X, OUTPUT_PIN_Y, 0.0))
    )
    a.add(
        polished_output_rod,
        "polished_output_rod|dof=free|"
        "mount=lower_output_guide,upper_output_guide,beam_output_pin",
    )

    # Rear guide post leaves the rod and beam visible from the open side.
    guide_post = Box(
        GUIDE_POST_W,
        GUIDE_POST_D,
        GUIDE_POST_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((OUTPUT_PIN_X, GUIDE_POST_Y, BASE_H)))
    a.add(guide_post, "output_guide_post|dof=fixed|mount=base")

    def make_square_guide(z_center):
        frame = Box(GUIDE_OUTER_W, GUIDE_OUTER_D, GUIDE_H)
        hole = Box(
            GUIDE_HOLE_W,
            GUIDE_HOLE_D,
            GUIDE_H + 2.0,
        )
        square_frame = frame - hole

        arm_center_y = (
            OUTPUT_PIN_Y + GUIDE_OUTER_D / 2.0 + GUIDE_POST_Y
        ) / 2.0
        arm_len_y = (
            GUIDE_POST_Y
            - (OUTPUT_PIN_Y + GUIDE_OUTER_D / 2.0)
        )
        arm = Box(
            GUIDE_OUTER_W,
            arm_len_y,
            GUIDE_H,
        ).moved(
            Location(
                (
                    0.0,
                    arm_center_y - OUTPUT_PIN_Y,
                    0.0,
                )
            )
        )
        return (square_frame + arm).moved(
            Location((OUTPUT_PIN_X, OUTPUT_PIN_Y, z_center))
        )

    lower_guide = make_square_guide(LOWER_GUIDE_Z)
    a.add(
        lower_guide,
        "lower_output_guide|dof=fixed|mount=output_guide_post",
    )

    upper_guide = make_square_guide(UPPER_GUIDE_Z)
    a.add(
        upper_guide,
        "upper_output_guide|dof=fixed|mount=output_guide_post",
    )

    # Exposed hand-crank arm beyond the positive shaft bearing.
    hand_arm_local = local_link(
        HAND_ARM_LENGTH,
        HAND_ARM_T,
        HAND_ARM_EYE_R,
        HAND_ARM_WEB_H,
        HAND_ARM_BORE_R,
        HANDLE_SPINDLE_R - PRESS_INTERFERENCE,
    )
    hand_crank_arm = hand_arm_local.moved(
        Location(
            (
                SHAFT_CX,
                HAND_ARM_Y,
                SHAFT_CZ + HAND_ARM_LENGTH / 2.0,
            ),
            (0.0, -90.0, 0.0),
        )
    )
    a.add(
        hand_crank_arm,
        "hand_crank_arm|dof=fixed|mount=crankshaft",
    )

    handle_spindle = Cylinder(
        HANDLE_SPINDLE_R, HANDLE_SPINDLE_LEN
    ).moved(
        Location(
            (HANDLE_X, HANDLE_SPINDLE_Y, HANDLE_Z),
            (90, 0, 0),
        )
    )
    a.add(
        handle_spindle,
        "handle_spindle|dof=fixed|mount=hand_crank_arm",
    )

    grip_outer = Cylinder(HANDLE_GRIP_R, HANDLE_GRIP_LEN)
    grip_cut = Cylinder(HANDLE_GRIP_BORE_R, HANDLE_GRIP_LEN + 2.0)
    hand_grip = (grip_outer - grip_cut).moved(
        Location(
            (HANDLE_X, HANDLE_GRIP_Y, HANDLE_Z),
            (90, 0, 0),
        )
    )
    a.add(
        hand_grip,
        "hand_grip|dof=fixed|mount=handle_spindle",
    )

    return a.build()