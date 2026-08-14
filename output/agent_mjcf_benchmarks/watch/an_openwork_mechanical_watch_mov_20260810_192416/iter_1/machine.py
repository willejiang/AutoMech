import math

# Drivetrain arithmetic
M = 0.45

Z_MINUTE_PINION = 12
Z_INTERMEDIATE_WHEEL = 48
Z_INTERMEDIATE_PINION = 15
Z_HOUR_WHEEL = 45

def pitch_r(z):
    return M * z / 2.0

def center_dist(za, zb):
    return M * (za + zb) / 2.0

CD_STAGE_1 = center_dist(Z_MINUTE_PINION, Z_INTERMEDIATE_WHEEL)
CD_STAGE_2 = center_dist(Z_INTERMEDIATE_PINION, Z_HOUR_WHEEL)

assert abs(CD_STAGE_1 - CD_STAGE_2) < 1.0e-9

STAGE_RATIO_1 = Z_INTERMEDIATE_WHEEL / Z_MINUTE_PINION
STAGE_RATIO_2 = Z_HOUR_WHEEL / Z_INTERMEDIATE_PINION
TOTAL_HOUR_REDUCTION = STAGE_RATIO_1 * STAGE_RATIO_2

CENTER_X = 0.0
CENTER_Y = 0.0
INTERMEDIATE_X = CENTER_X + CD_STAGE_1
INTERMEDIATE_Y = CENTER_Y

plate_h = 2.0
plate_outer_r = 27.0
plate_inner_r = 24.0
plate_spoke_w = 5.0

bridge_z = 10.2
bridge_h = 1.5

pillar_r = 0.9
pillar_z = plate_h
pillar_h = bridge_z - pillar_z
pillar_positions = [
    (-20.0, -12.0),
    (-20.0, 12.0),
    (20.0, -12.0),
    (20.0, 12.0),
]

minute_r = 0.80
minute_pipe_z = plate_h
minute_pipe_h = 15.2

hour_inner_r = minute_r + 0.06
hour_outer_r = 1.30
hour_pipe_z = 7.8
hour_pipe_h = 8.4

intermediate_arbor_r = 0.75
intermediate_arbor_z = plate_h
intermediate_arbor_h = bridge_z + bridge_h - intermediate_arbor_z

bearing_outer_r = 2.0
bearing_running_clearance = 0.05
bearing_press_interference = 0.005

lower_bearing_z = 0.0
lower_bearing_h = 4.8
upper_bearing_z = bridge_z
upper_bearing_h = bridge_h

gear_face = 1.8
stage_1_z = 5.5
stage_2_z = 8.0
stage_1_center_z = stage_1_z + gear_face / 2.0
stage_2_center_z = stage_2_z + gear_face / 2.0

hour_hand_z = 15.7
minute_hand_z = 16.4
hand_h = 0.5
hour_hand_len = 14.5
minute_hand_len = 21.5

dial_marker_r = 22.5
dial_marker_z = bridge_z + bridge_h
dial_marker_h = 0.45

MECHANISM = {
    "name": "openwork_watch_movement",
    "output_link": "hour_hand",
    "watch_links": [
        "minute_pipe",
        "intermediate_arbor",
        "hour_pipe",
        "minute_hand",
        "hour_hand",
    ],
    "ports_by_link": {
        "minute_pipe": [
            {
                "name": "shaft_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * minute_r,
                "depth_mm": minute_pipe_h,
            },
            {
                "name": "hour_journal",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, hour_pipe_z - minute_pipe_z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * minute_r,
                "depth_mm": hour_pipe_h,
            },
            {
                "name": "minute_hand_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, minute_hand_z - minute_pipe_z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * minute_r,
                "depth_mm": hand_h,
            },
        ],
        "hour_pipe": [
            {
                "name": "outer_shaft",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * hour_outer_r,
                "depth_mm": hour_pipe_h,
            },
            {
                "name": "inner_journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * hour_inner_r,
                "depth_mm": hour_pipe_h,
            },
            {
                "name": "hour_hand_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, hour_hand_z - hour_pipe_z],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * hour_outer_r,
                "depth_mm": hand_h,
            },
        ],
        "intermediate_arbor": [
            {
                "name": "shaft_axis",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * intermediate_arbor_r,
                "depth_mm": intermediate_arbor_h,
            }
        ],
        "minute_pinion": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (minute_r - bearing_press_interference),
                "depth_mm": gear_face,
            },
            {
                "name": "teeth",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": pitch_r(Z_MINUTE_PINION),
                "depth_mm": gear_face,
            },
        ],
        "intermediate_wheel": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    intermediate_arbor_r - bearing_press_interference
                ),
                "depth_mm": gear_face,
            },
            {
                "name": "teeth",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": pitch_r(Z_INTERMEDIATE_WHEEL),
                "depth_mm": gear_face,
            },
        ],
        "intermediate_pinion": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    intermediate_arbor_r - bearing_press_interference
                ),
                "depth_mm": gear_face,
            },
            {
                "name": "teeth",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": pitch_r(Z_INTERMEDIATE_PINION),
                "depth_mm": gear_face,
            },
        ],
        "hour_wheel": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    hour_outer_r - bearing_press_interference
                ),
                "depth_mm": gear_face,
            },
            {
                "name": "teeth",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": pitch_r(Z_HOUR_WHEEL),
                "depth_mm": gear_face,
            },
        ],
        "central_lower_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    minute_r + bearing_running_clearance
                ),
                "depth_mm": lower_bearing_h,
            }
        ],
        "central_upper_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    hour_outer_r + bearing_running_clearance
                ),
                "depth_mm": upper_bearing_h,
            }
        ],
        "intermediate_lower_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    intermediate_arbor_r + bearing_running_clearance
                ),
                "depth_mm": lower_bearing_h,
            }
        ],
        "intermediate_upper_bearing": [
            {
                "name": "journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    intermediate_arbor_r + bearing_running_clearance
                ),
                "depth_mm": upper_bearing_h,
            }
        ],
        "hour_hand": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    hour_outer_r - bearing_press_interference
                ),
                "depth_mm": hand_h,
            }
        ],
        "minute_hand": [
            {
                "name": "hub_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 0.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * (
                    minute_r - bearing_press_interference
                ),
                "depth_mm": hand_h,
            }
        ],
    },
    "relations": [
        {
            "name": "minute_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "central_lower_bearing",
            "base_port": "journal",
            "incoming_part": "minute_pipe",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "hour_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "central_upper_bearing",
            "base_port": "journal",
            "incoming_part": "hour_pipe",
            "incoming_port": "outer_shaft",
        },
        {
            "name": "minute_inside_hour_journal",
            "mate_type": "journal_bearing",
            "base_part": "hour_pipe",
            "base_port": "inner_journal",
            "incoming_part": "minute_pipe",
            "incoming_port": "hour_journal",
        },
        {
            "name": "intermediate_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "intermediate_lower_bearing",
            "base_port": "journal",
            "incoming_part": "intermediate_arbor",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "intermediate_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "intermediate_upper_bearing",
            "base_port": "journal",
            "incoming_part": "intermediate_arbor",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "minute_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "minute_pinion",
            "base_port": "bore",
            "incoming_part": "minute_pipe",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "intermediate_wheel_press_fit",
            "mate_type": "press_fit",
            "base_part": "intermediate_wheel",
            "base_port": "bore",
            "incoming_part": "intermediate_arbor",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "intermediate_pinion_press_fit",
            "mate_type": "press_fit",
            "base_part": "intermediate_pinion",
            "base_port": "bore",
            "incoming_part": "intermediate_arbor",
            "incoming_port": "shaft_axis",
        },
        {
            "name": "hour_wheel_press_fit",
            "mate_type": "press_fit",
            "base_part": "hour_wheel",
            "base_port": "bore",
            "incoming_part": "hour_pipe",
            "incoming_port": "outer_shaft",
        },
        {
            "name": "hour_hand_press_fit",
            "mate_type": "press_fit",
            "base_part": "hour_hand",
            "base_port": "hub_bore",
            "incoming_part": "hour_pipe",
            "incoming_port": "hour_hand_seat",
        },
        {
            "name": "minute_hand_press_fit",
            "mate_type": "press_fit",
            "base_part": "minute_hand",
            "base_port": "hub_bore",
            "incoming_part": "minute_pipe",
            "incoming_port": "minute_hand_seat",
        },
        {
            "name": "motion_work_stage_1",
            "mate_type": "gear_spur_external",
            "base_part": "minute_pinion",
            "base_port": "teeth",
            "incoming_part": "intermediate_wheel",
            "incoming_port": "teeth",
            "separation_axis": "+x",
        },
        {
            "name": "motion_work_stage_2",
            "mate_type": "gear_spur_external",
            "base_part": "intermediate_pinion",
            "base_port": "teeth",
            "incoming_part": "hour_wheel",
            "incoming_port": "teeth",
            "separation_axis": "-x",
        },
    ],
    "motion_joints": [
        {
            "name": "minute_rotation",
            "parent": "",
            "child": "minute_pipe",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "intermediate_rotation",
            "parent": "",
            "child": "intermediate_arbor",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "hour_rotation",
            "parent": "",
            "child": "hour_pipe",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
    ],
    "transmissions": [
        {
            "name": "minute_pipe_to_minute_pinion",
            "type": "compound_1to1",
            "driving_link": "minute_pipe",
            "driven_link": "minute_pinion",
            "ratio": 1.0,
        },
        {
            "name": "stage_1_external_mesh",
            "type": "gear_external",
            "driving_link": "minute_pinion",
            "driven_link": "intermediate_wheel",
            "ratio": 0,
        },
        {
            "name": "intermediate_compound",
            "type": "compound_1to1",
            "driving_link": "intermediate_wheel",
            "driven_link": "intermediate_pinion",
            "ratio": 1.0,
        },
        {
            "name": "stage_2_external_mesh",
            "type": "gear_external",
            "driving_link": "intermediate_pinion",
            "driven_link": "hour_wheel",
            "ratio": 0,
        },
        {
            "name": "hour_wheel_to_hour_pipe",
            "type": "compound_1to1",
            "driving_link": "hour_wheel",
            "driven_link": "hour_pipe",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [],
}


def build_machine():
    a = AssemblyHelper("openwork_watch_movement")

    def make_annulus(outer_r, inner_r, height):
        outer = Cylinder(
            outer_r,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        cutter = Cylinder(
            inner_r,
            height + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -0.5)))
        return outer - cutter

    def make_bearing(outer_r, shaft_r, height):
        bearing = Cylinder(
            outer_r,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        bore = Cylinder(
            shaft_r + bearing_running_clearance,
            height + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -0.5)))
        return bearing - bore

    def make_pipe(outer_r, inner_r, height):
        pipe = Cylinder(
            outer_r,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        bore = Cylinder(
            inner_r,
            height + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -0.5)))
        return pipe - bore

    def make_hand(length, width, hub_r, bore_r, thickness):
        with BuildSketch() as hand_sketch:
            Circle(hub_r)
            b3d.Rectangle(
                length,
                width,
                align=(Align.MIN, Align.CENTER),
            )
            Circle(bore_r, mode=Mode.SUBTRACT)
        return extrude(hand_sketch.sketch, amount=thickness)

    plate_ring = make_annulus(plate_outer_r, plate_inner_r, plate_h)
    plate_spoke = Box(
        2.0 * plate_outer_r,
        plate_spoke_w,
        plate_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    baseplate = plate_ring + plate_spoke

    base_hole_r = bearing_outer_r - bearing_press_interference
    base_cutters = (
        Cylinder(
            base_hole_r,
            plate_h + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((CENTER_X, CENTER_Y, -0.5)))
        +
        Cylinder(
            base_hole_r,
            plate_h + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((INTERMEDIATE_X, INTERMEDIATE_Y, -0.5)))
    )
    baseplate = baseplate - base_cutters
    a.add(baseplate, "baseplate|dof=fixed")

    pillar_names = []
    for index, (px, py) in enumerate(pillar_positions):
        name = "pillar_{:02d}".format(index + 1)
        pillar_names.append(name)
        pillar = Cylinder(
            pillar_r,
            pillar_h,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((px, py, pillar_z)))
        a.add(pillar, name + "|dof=fixed|mount=baseplate")

    bridge_ring = make_annulus(
        plate_outer_r,
        plate_inner_r,
        bridge_h,
    )
    bridge_spoke = Box(
        2.0 * plate_outer_r,
        plate_spoke_w,
        bridge_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    upper_bridge = bridge_ring + bridge_spoke

    bridge_hole_r = bearing_outer_r - bearing_press_interference
    bridge_cutters = (
        Cylinder(
            bridge_hole_r,
            bridge_h + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((CENTER_X, CENTER_Y, -0.5)))
        +
        Cylinder(
            bridge_hole_r,
            bridge_h + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((INTERMEDIATE_X, INTERMEDIATE_Y, -0.5)))
    )
    upper_bridge = (upper_bridge - bridge_cutters).moved(
        Location((0.0, 0.0, bridge_z))
    )
    a.add(
        upper_bridge,
        "upper_bridge|dof=fixed|mount=" + ",".join(pillar_names),
    )

    central_lower_bearing = make_bearing(
        bearing_outer_r,
        minute_r,
        lower_bearing_h,
    ).moved(Location((CENTER_X, CENTER_Y, lower_bearing_z)))
    a.add(
        central_lower_bearing,
        "central_lower_bearing|dof=fixed|mount=baseplate",
    )

    intermediate_lower_bearing = make_bearing(
        bearing_outer_r,
        intermediate_arbor_r,
        lower_bearing_h,
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                lower_bearing_z,
            )
        )
    )
    a.add(
        intermediate_lower_bearing,
        "intermediate_lower_bearing|dof=fixed|mount=baseplate",
    )

    central_upper_bearing = make_bearing(
        bearing_outer_r,
        hour_outer_r,
        upper_bearing_h,
    ).moved(Location((CENTER_X, CENTER_Y, upper_bearing_z)))
    a.add(
        central_upper_bearing,
        "central_upper_bearing|dof=fixed|mount=upper_bridge",
    )

    intermediate_upper_bearing = make_bearing(
        bearing_outer_r,
        intermediate_arbor_r,
        upper_bearing_h,
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                upper_bearing_z,
            )
        )
    )
    a.add(
        intermediate_upper_bearing,
        "intermediate_upper_bearing|dof=fixed|mount=upper_bridge",
    )

    minute_pipe = Cylinder(
        minute_r,
        minute_pipe_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((CENTER_X, CENTER_Y, minute_pipe_z)))
    a.add(
        minute_pipe,
        "minute_pipe|dof=spin|driver=True|spin_axis=z|"
        "mount=central_lower_bearing",
    )

    hour_pipe = make_pipe(
        hour_outer_r,
        hour_inner_r,
        hour_pipe_h,
    ).moved(Location((CENTER_X, CENTER_Y, hour_pipe_z)))
    a.add(
        hour_pipe,
        "hour_pipe|dof=spin|spin_axis=z|"
        "mount=central_upper_bearing,minute_pipe",
    )

    intermediate_arbor = Cylinder(
        intermediate_arbor_r,
        intermediate_arbor_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                intermediate_arbor_z,
            )
        )
    )
    a.add(
        intermediate_arbor,
        "intermediate_arbor|dof=spin|spin_axis=z|"
        "mount=intermediate_lower_bearing,intermediate_upper_bearing",
    )

    minute_pinion = make_gear(
        M,
        Z_MINUTE_PINION,
        gear_face,
        2.0 * (minute_r - bearing_press_interference),
    ).moved(
        Location(
            (CENTER_X, CENTER_Y, stage_1_center_z),
            (0.0, 0.0, 0.0),
        )
    )
    a.add(
        minute_pinion,
        "minute_pinion|dof=fixed|mesh_id=motion_stage_1|"
        "mount=minute_pipe",
    )

    intermediate_wheel = make_gear(
        M,
        Z_INTERMEDIATE_WHEEL,
        gear_face,
        2.0 * (
            intermediate_arbor_r - bearing_press_interference
        ),
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                stage_1_center_z,
            ),
            (0.0, 0.0, 180.0 / Z_INTERMEDIATE_WHEEL),
        )
    )
    a.add(
        intermediate_wheel,
        "intermediate_wheel|dof=fixed|mesh_id=motion_stage_1|"
        "mount=intermediate_arbor",
    )

    intermediate_pinion = make_gear(
        M,
        Z_INTERMEDIATE_PINION,
        gear_face,
        2.0 * (
            intermediate_arbor_r - bearing_press_interference
        ),
    ).moved(
        Location(
            (
                INTERMEDIATE_X,
                INTERMEDIATE_Y,
                stage_2_center_z,
            ),
            (0.0, 0.0, 0.0),
        )
    )
    a.add(
        intermediate_pinion,
        "intermediate_pinion|dof=fixed|mesh_id=motion_stage_2|"
        "mount=intermediate_arbor",
    )

    hour_wheel = make_gear(
        M,
        Z_HOUR_WHEEL,
        gear_face,
        2.0 * (hour_outer_r - bearing_press_interference),
    ).moved(
        Location(
            (CENTER_X, CENTER_Y, stage_2_center_z),
            (0.0, 0.0, 180.0 / Z_HOUR_WHEEL),
        )
    )
    a.add(
        hour_wheel,
        "hour_wheel|dof=fixed|mesh_id=motion_stage_2|"
        "mount=hour_pipe",
    )

    hour_hand = make_hand(
        hour_hand_len,
        1.2,
        2.0,
        hour_outer_r - bearing_press_interference,
        hand_h,
    ).moved(Location((CENTER_X, CENTER_Y, hour_hand_z)))
    a.add(
        hour_hand,
        "hour_hand|dof=fixed|mount=hour_pipe",
    )

    minute_hand = make_hand(
        minute_hand_len,
        0.85,
        1.45,
        minute_r - bearing_press_interference,
        hand_h,
    ).moved(
        Location(
            (CENTER_X, CENTER_Y, minute_hand_z),
            (0.0, 0.0, 90.0),
        )
    )
    a.add(
        minute_hand,
        "minute_hand|dof=fixed|mount=minute_pipe",
    )

    for index in range(12):
        angle_deg = 30.0 * index
        angle_rad = math.radians(angle_deg)
        marker_x = dial_marker_r * math.sin(angle_rad)
        marker_y = dial_marker_r * math.cos(angle_rad)
        marker_length = 2.6 if index % 3 == 0 else 1.7
        marker_width = 0.75 if index % 3 == 0 else 0.5

        marker = Box(
            marker_width,
            marker_length,
            dial_marker_h,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(
            Location(
                (marker_x, marker_y, dial_marker_z),
                (0.0, 0.0, -angle_deg),
            )
        )
        marker_name = "dial_marker_{:02d}".format(index + 1)
        a.add(
            marker,
            marker_name + "|dof=fixed|mount=upper_bridge",
        )

    return a.build()