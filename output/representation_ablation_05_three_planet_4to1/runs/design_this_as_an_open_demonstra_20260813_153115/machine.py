MECHANISM = {
    "name": "open_frame_hand_driven_4_to_1_planetary_reducer",
    "output_link": "carrier",
    "watch_links": [
        "input_shaft",
        "sun_gear",
        "planet_gear_1",
        "planet_gear_2",
        "planet_gear_3",
        "carrier",
        "output_indicator",
    ],
    "ports_by_link": {
        "base": [
            {
                "name": "top_support",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, 4.0],
                "axis": [0.0, 0.0, 1.0],
                "normal_sign": 1,
            }
        ],
        "center_pedestal": [
            {
                "name": "base_face",
                "type": "flat_face",
                "xyz_mm": [0.0, 0.0, 4.0],
                "axis": [0.0, 0.0, 1.0],
                "normal_sign": -1,
            },
            {
                "name": "input_bearing_seat",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 10.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 12.2,
                "depth_mm": 16.0,
            },
            {
                "name": "carrier_bearing_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 18.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 16.0,
                "depth_mm": 16.0,
            },
        ],
        "input_bearing": [
            {
                "name": "outer_race",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, 10.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 12.0,
                "depth_mm": 8.0,
            },
            {
                "name": "shaft_journal",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 10.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.2,
                "depth_mm": 8.0,
            },
        ],
        "input_shaft": [
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 10.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.0,
                "depth_mm": 8.0,
            },
            {
                "name": "sun_press_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.0,
                "depth_mm": 8.0,
            },
            {
                "name": "crank_press_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, 42.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.0,
                "depth_mm": 4.0,
            },
        ],
        "sun_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 7.9,
                "depth_mm": 8.0,
            },
            {
                "name": "planet_mesh_1",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
            {
                "name": "planet_mesh_2",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
            {
                "name": "planet_mesh_3",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
        ],
        "carrier_bearing": [
            {
                "name": "pedestal_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 18.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 16.2,
                "depth_mm": 8.0,
            },
            {
                "name": "carrier_journal",
                "type": "cylindrical",
                "xyz_mm": [0.0, 0.0, 18.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 18.0,
                "depth_mm": 8.0,
            },
        ],
        "carrier": [
            {
                "name": "central_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, 18.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 18.3,
                "depth_mm": 11.0,
            },
            {
                "name": "planet_axis_1",
                "type": "cylindrical",
                "xyz_mm": [27.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.0,
            },
            {
                "name": "planet_axis_2",
                "type": "cylindrical",
                "xyz_mm": [-13.5, 23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.0,
            },
            {
                "name": "planet_axis_3",
                "type": "cylindrical",
                "xyz_mm": [-13.5, -23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.0,
            },
        ],
        "planet_pin_1": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [27.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.0,
                "depth_mm": 11.0,
            }
        ],
        "planet_pin_2": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [-13.5, 23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.0,
                "depth_mm": 11.0,
            }
        ],
        "planet_pin_3": [
            {
                "name": "pin_axis",
                "type": "shaft",
                "xyz_mm": [-13.5, -23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.0,
                "depth_mm": 11.0,
            }
        ],
        "planet_bushing_1": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [27.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.2,
                "depth_mm": 8.0,
            },
            {
                "name": "gear_journal",
                "type": "cylindrical",
                "xyz_mm": [27.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.0,
                "depth_mm": 8.0,
            },
        ],
        "planet_bushing_2": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [-13.5, 23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.2,
                "depth_mm": 8.0,
            },
            {
                "name": "gear_journal",
                "type": "cylindrical",
                "xyz_mm": [-13.5, 23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.0,
                "depth_mm": 8.0,
            },
        ],
        "planet_bushing_3": [
            {
                "name": "pin_bore",
                "type": "bore",
                "xyz_mm": [-13.5, -23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 6.2,
                "depth_mm": 8.0,
            },
            {
                "name": "gear_journal",
                "type": "cylindrical",
                "xyz_mm": [-13.5, -23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.0,
                "depth_mm": 8.0,
            },
        ],
        "planet_gear_1": [
            {
                "name": "bearing_bore",
                "type": "bore",
                "xyz_mm": [27.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.2,
                "depth_mm": 8.0,
            },
            {
                "name": "sun_mesh",
                "type": "gear_mesh",
                "xyz_mm": [27.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
            {
                "name": "ring_mesh",
                "type": "gear_mesh",
                "xyz_mm": [27.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
        ],
        "planet_gear_2": [
            {
                "name": "bearing_bore",
                "type": "bore",
                "xyz_mm": [-13.5, 23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.2,
                "depth_mm": 8.0,
            },
            {
                "name": "sun_mesh",
                "type": "gear_mesh",
                "xyz_mm": [-13.5, 23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
            {
                "name": "ring_mesh",
                "type": "gear_mesh",
                "xyz_mm": [-13.5, 23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
        ],
        "planet_gear_3": [
            {
                "name": "bearing_bore",
                "type": "bore",
                "xyz_mm": [-13.5, -23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 8.2,
                "depth_mm": 8.0,
            },
            {
                "name": "sun_mesh",
                "type": "gear_mesh",
                "xyz_mm": [-13.5, -23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
            {
                "name": "ring_mesh",
                "type": "gear_mesh",
                "xyz_mm": [-13.5, -23.3827, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 13.5,
                "depth_mm": 8.0,
            },
        ],
        "fixed_ring_gear": [
            {
                "name": "internal_mesh_1",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 40.5,
                "depth_mm": 8.0,
            },
            {
                "name": "internal_mesh_2",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 40.5,
                "depth_mm": 8.0,
            },
            {
                "name": "internal_mesh_3",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, 28.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": 40.5,
                "depth_mm": 8.0,
            },
        ],
    },
    "relations": [
        {
            "name": "input_bearing_in_pedestal",
            "mate_type": "press_fit",
            "base_part": "center_pedestal",
            "base_port": "input_bearing_seat",
            "incoming_part": "input_bearing",
            "incoming_port": "outer_race",
            "offset_mm": 0.0,
        },
        {
            "name": "input_shaft_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_bearing",
            "base_port": "shaft_journal",
            "incoming_part": "input_shaft",
            "incoming_port": "lower_journal",
            "offset_mm": 0.0,
        },
        {
            "name": "sun_to_input_shaft_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "sun_press_seat",
            "incoming_part": "sun_gear",
            "incoming_port": "shaft_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "carrier_bearing_on_pedestal",
            "mate_type": "journal_bearing",
            "base_part": "center_pedestal",
            "base_port": "carrier_bearing_seat",
            "incoming_part": "carrier_bearing",
            "incoming_port": "pedestal_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "carrier_on_central_bearing",
            "mate_type": "journal_bearing",
            "base_part": "carrier_bearing",
            "base_port": "carrier_journal",
            "incoming_part": "carrier",
            "incoming_port": "central_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "sun_planet_1_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "sun_gear",
            "base_port": "planet_mesh_1",
            "incoming_part": "planet_gear_1",
            "incoming_port": "sun_mesh",
            "separation_axis": "+x",
            "angle_rad": 0.1745329252,
        },
        {
            "name": "sun_planet_2_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "sun_gear",
            "base_port": "planet_mesh_2",
            "incoming_part": "planet_gear_2",
            "incoming_port": "sun_mesh",
            "separation_axis": [-0.5, 0.8660254, 0.0],
            "angle_rad": 0.1745329252,
        },
        {
            "name": "sun_planet_3_mesh",
            "mate_type": "gear_spur_external",
            "base_part": "sun_gear",
            "base_port": "planet_mesh_3",
            "incoming_part": "planet_gear_3",
            "incoming_port": "sun_mesh",
            "separation_axis": [-0.5, -0.8660254, 0.0],
            "angle_rad": 0.1745329252,
        },
        {
            "name": "planet_1_bushing_journal",
            "mate_type": "journal_bearing",
            "base_part": "planet_bushing_1",
            "base_port": "gear_journal",
            "incoming_part": "planet_gear_1",
            "incoming_port": "bearing_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "planet_2_bushing_journal",
            "mate_type": "journal_bearing",
            "base_part": "planet_bushing_2",
            "base_port": "gear_journal",
            "incoming_part": "planet_gear_2",
            "incoming_port": "bearing_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "planet_3_bushing_journal",
            "mate_type": "journal_bearing",
            "base_part": "planet_bushing_3",
            "base_port": "gear_journal",
            "incoming_part": "planet_gear_3",
            "incoming_port": "bearing_bore",
            "offset_mm": 0.0,
        },
    ],
    "motion_joints": [
        {
            "name": "input_shaft_world_hinge",
            "parent": "",
            "child": "input_shaft",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "sun_world_hinge",
            "parent": "",
            "child": "sun_gear",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "carrier_world_hinge",
            "parent": "",
            "child": "carrier",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, 0.0],
        },
        {
            "name": "planet_1_carrier_hinge",
            "parent": "carrier",
            "child": "planet_gear_1",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [27.0, 0.0, 0.0],
        },
        {
            "name": "planet_2_carrier_hinge",
            "parent": "carrier",
            "child": "planet_gear_2",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [-13.5, 23.3827, 0.0],
        },
        {
            "name": "planet_3_carrier_hinge",
            "parent": "carrier",
            "child": "planet_gear_3",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [-13.5, -23.3827, 0.0],
        },
    ],
    "transmissions": [
        {
            "name": "input_shaft_to_sun",
            "type": "compound_1to1",
            "driving_link": "input_shaft",
            "driven_link": "sun_gear",
            "ratio": 1.0,
        }
    ],
    "planetary_stages": [
        {
            "name": "four_to_one_planetary_stage",
            "sun": "sun_gear",
            "ring": "fixed_ring_gear",
            "carrier": "carrier",
            "planets": [
                {"gear": "planet_gear_1", "pin": "planet_pin_1"},
                {"gear": "planet_gear_2", "pin": "planet_pin_2"},
                {"gear": "planet_gear_3", "pin": "planet_pin_3"},
            ],
            "sun_teeth": 18,
            "planet_teeth": 18,
            "ring_teeth": 54,
            "fixed_member": "ring",
            "input_member": "sun",
            "output_member": "carrier",
        }
    ],
}


def build_machine():
    a = AssemblyHelper("open_frame_hand_driven_4_to_1_planetary_reducer")

    base = Box(
        105.0,
        105.0,
        4.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    a.add(base, "base|dof=fixed")

    pedestal_flange = Cylinder(14.0, 4.0)
    pedestal_column = Cylinder(8.0, 18.0).moved(Location((0.0, 0.0, 4.0)))
    pedestal_bore = Cylinder(6.1, 24.0).moved(Location((0.0, 0.0, 2.0)))
    center_pedestal = (
        pedestal_flange.fuse(pedestal_column).cut(pedestal_bore)
        .moved(Location((0.0, 0.0, 4.0)))
    )
    a.add(center_pedestal, "center_pedestal|dof=fixed|mount=base")

    input_bearing_outer = Cylinder(6.0, 8.0)
    input_bearing_inner = Cylinder(4.1, 10.0).moved(Location((0.0, 0.0, -1.0)))
    input_bearing = (
        input_bearing_outer.cut(input_bearing_inner)
        .moved(Location((0.0, 0.0, 6.0)))
    )
    a.add(input_bearing, "input_bearing|dof=fixed|mount=center_pedestal")

    carrier_bearing_outer = Cylinder(9.0, 8.0)
    carrier_bearing_inner = Cylinder(8.1, 10.0).moved(Location((0.0, 0.0, -1.0)))
    carrier_bearing = (
        carrier_bearing_outer.cut(carrier_bearing_inner)
        .moved(Location((0.0, 0.0, 14.0)))
    )
    a.add(carrier_bearing, "carrier_bearing|dof=fixed|mount=center_pedestal")

    input_shaft = Cylinder(4.0, 35.0).moved(Location((0.0, 0.0, 22.5)))
    a.add(
        input_shaft,
        "input_shaft|dof=spin|driver=True|spin_axis=z|mount=input_bearing",
    )

    carrier_center = Cylinder(12.0, 4.0)
    carrier_arm_1 = Box(
        28.0,
        8.0,
        4.0,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    carrier_arm_2 = Box(
        28.0,
        8.0,
        4.0,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, 0.0), (0.0, 0.0, 120.0)))
    carrier_arm_3 = Box(
        28.0,
        8.0,
        4.0,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((0.0, 0.0, 0.0), (0.0, 0.0, 240.0)))
    carrier_pad_1 = Cylinder(6.0, 4.0).moved(Location((27.0, 0.0, 0.0)))
    carrier_pad_2 = Cylinder(6.0, 4.0).moved(
        Location((-13.5, 23.3827, 0.0))
    )
    carrier_pad_3 = Cylinder(6.0, 4.0).moved(
        Location((-13.5, -23.3827, 0.0))
    )
    carrier_hub = Cylinder(12.0, 11.0).moved(Location((0.0, 0.0, -7.0)))
    carrier_bore_tool = Cylinder(9.15, 15.0).moved(
        Location((0.0, 0.0, -9.0))
    )
    carrier = (
        carrier_center
        .fuse(carrier_arm_1)
        .fuse(carrier_arm_2)
        .fuse(carrier_arm_3)
        .fuse(carrier_pad_1)
        .fuse(carrier_pad_2)
        .fuse(carrier_pad_3)
        .fuse(carrier_hub)
        .cut(carrier_bore_tool)
        .moved(Location((0.0, 0.0, 19.0)))
    )
    a.add(
        carrier,
        "carrier|dof=spin|spin_axis=z|mount=carrier_bearing",
    )

    output_indicator_arm = Box(
        38.0,
        3.0,
        2.0,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    ).moved(Location((10.0, 0.0, 0.0)))
    output_indicator_tip = Cylinder(3.0, 2.0).moved(
        Location((48.0, 0.0, 0.0))
    )
    output_indicator = (
        output_indicator_arm.fuse(output_indicator_tip)
        .moved(Location((0.0, 0.0, 16.0)))
    )
    a.add(
        output_indicator,
        "output_indicator|dof=fixed|mount=carrier",
    )

    planet_pin_1 = Cylinder(3.0, 11.0).moved(
        Location((27.0, 0.0, 22.0))
    )
    a.add(planet_pin_1, "planet_pin_1|dof=fixed|mount=carrier")

    planet_pin_2 = Cylinder(3.0, 11.0).moved(
        Location((-13.5, 23.3827, 22.0))
    )
    a.add(planet_pin_2, "planet_pin_2|dof=fixed|mount=carrier")

    planet_pin_3 = Cylinder(3.0, 11.0).moved(
        Location((-13.5, -23.3827, 22.0))
    )
    a.add(planet_pin_3, "planet_pin_3|dof=fixed|mount=carrier")

    planet_bushing_1_outer = Cylinder(4.0, 8.0)
    planet_bushing_1_inner = Cylinder(3.1, 10.0).moved(
        Location((0.0, 0.0, -1.0))
    )
    planet_bushing_1 = (
        planet_bushing_1_outer.cut(planet_bushing_1_inner)
        .moved(Location((27.0, 0.0, 24.0)))
    )
    a.add(
        planet_bushing_1,
        "planet_bushing_1|dof=fixed|mount=planet_pin_1",
    )

    planet_bushing_2_outer = Cylinder(4.0, 8.0)
    planet_bushing_2_inner = Cylinder(3.1, 10.0).moved(
        Location((0.0, 0.0, -1.0))
    )
    planet_bushing_2 = (
        planet_bushing_2_outer.cut(planet_bushing_2_inner)
        .moved(Location((-13.5, 23.3827, 24.0)))
    )
    a.add(
        planet_bushing_2,
        "planet_bushing_2|dof=fixed|mount=planet_pin_2",
    )

    planet_bushing_3_outer = Cylinder(4.0, 8.0)
    planet_bushing_3_inner = Cylinder(3.1, 10.0).moved(
        Location((0.0, 0.0, -1.0))
    )
    planet_bushing_3 = (
        planet_bushing_3_outer.cut(planet_bushing_3_inner)
        .moved(Location((-13.5, -23.3827, 24.0)))
    )
    a.add(
        planet_bushing_3,
        "planet_bushing_3|dof=fixed|mount=planet_pin_3",
    )

    sun_gear = make_gear(1.5, 18, 8.0, 7.9).moved(
        Location((0.0, 0.0, 24.0))
    )
    a.add(
        sun_gear,
        "sun_gear|dof=spin|spin_axis=z|mount=input_shaft",
    )

    planet_gear_1 = make_gear(1.5, 18, 8.0, 8.2).moved(
        Location((27.0, 0.0, 24.0), (0.0, 0.0, 10.0))
    )
    a.add(
        planet_gear_1,
        "planet_gear_1|dof=spin|spin_axis=z|mount=planet_bushing_1",
    )

    planet_gear_2 = make_gear(1.5, 18, 8.0, 8.2).moved(
        Location((-13.5, 23.3827, 24.0), (0.0, 0.0, 10.0))
    )
    a.add(
        planet_gear_2,
        "planet_gear_2|dof=spin|spin_axis=z|mount=planet_bushing_2",
    )

    planet_gear_3 = make_gear(1.5, 18, 8.0, 8.2).moved(
        Location((-13.5, -23.3827, 24.0), (0.0, 0.0, 10.0))
    )
    a.add(
        planet_gear_3,
        "planet_gear_3|dof=spin|spin_axis=z|mount=planet_bushing_3",
    )

    ring_blank = Cylinder(46.0, 8.0)
    ring_tooth_cutter = make_gear(1.5, 54, 10.0, 0.0).moved(
        Location((0.0, 0.0, -1.0))
    )
    fixed_ring_gear = ring_blank.cut(ring_tooth_cutter).moved(
        Location((0.0, 0.0, 24.0))
    )

    ring_post_1 = Cylinder(4.0, 20.0).moved(
        Location((44.0, 0.0, 4.0))
    )
    a.add(ring_post_1, "ring_post_1|dof=fixed|mount=base")

    ring_post_2 = Cylinder(4.0, 20.0).moved(
        Location((-22.0, 38.1051, 4.0))
    )
    a.add(ring_post_2, "ring_post_2|dof=fixed|mount=base")

    ring_post_3 = Cylinder(4.0, 20.0).moved(
        Location((-22.0, -38.1051, 4.0))
    )
    a.add(ring_post_3, "ring_post_3|dof=fixed|mount=base")

    a.add(
        fixed_ring_gear,
        "fixed_ring_gear|dof=fixed|mount=ring_post_1,ring_post_2,ring_post_3",
    )

    crank_hub_outer = Cylinder(7.0, 4.0)
    crank_hub_bore = Cylinder(3.95, 6.0).moved(
        Location((0.0, 0.0, -1.0))
    )
    crank_hub = crank_hub_outer.cut(crank_hub_bore)
    crank_beam = Box(
        32.0,
        5.0,
        4.0,
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    input_crank = crank_hub.fuse(crank_beam).moved(
        Location((0.0, 0.0, 40.0))
    )
    a.add(input_crank, "input_crank|dof=fixed|mount=input_shaft")

    crank_handle = Cylinder(4.0, 14.0).moved(
        Location((30.0, 0.0, 44.0))
    )
    a.add(crank_handle, "crank_handle|dof=fixed|mount=input_crank")

    return a.build()