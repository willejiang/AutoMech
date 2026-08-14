import math
from build123d import Axis

# ---------------------------------------------------------------------------
# Drivetrain arithmetic -- all gear locations and ratios derive from this.
# ---------------------------------------------------------------------------
M = 1.5
Z_SUN = 18
Z_PLANET = 18
Z_RING = Z_SUN + 2 * Z_PLANET
PLANET_COUNT = 3

assert Z_RING == Z_SUN + 2 * Z_PLANET
assert (Z_SUN + Z_RING) % PLANET_COUNT == 0

def pitch_r(z):
    return M * z / 2.0

def center_dist_external(za, zb):
    return M * (za + zb) / 2.0

def center_dist_internal(z_internal, z_external):
    return M * (z_internal - z_external) / 2.0

SUN_PITCH_R = pitch_r(Z_SUN)
PLANET_PITCH_R = pitch_r(Z_PLANET)
RING_PITCH_R = pitch_r(Z_RING)

SUN_PLANET_CD = center_dist_external(Z_SUN, Z_PLANET)
RING_PLANET_CD = center_dist_internal(Z_RING, Z_PLANET)
assert abs(SUN_PLANET_CD - RING_PLANET_CD) < 1e-9

PLANET_ORBIT_R = SUN_PLANET_CD
PLANET_ANGLES_DEG = tuple(360.0 * i / PLANET_COUNT for i in range(PLANET_COUNT))
PLANET_CENTERS = tuple(
    (
        PLANET_ORBIT_R * math.cos(math.radians(a)),
        PLANET_ORBIT_R * math.sin(math.radians(a)),
    )
    for a in PLANET_ANGLES_DEG
)

# A tooth is centered on +X on the sun and on each internal-ring tooth.
# At the three selected azimuths the 18-tooth sun presents equivalent phases.
# Rotate each planet by half a planet tooth pitch to present a gap to the sun
# tooth and a complementary gap/tooth phase at the fixed internal ring.
PLANET_TOOTH_PITCH_DEG = 360.0 / Z_PLANET
PLANET_PHASE_DEG = 0.5 * PLANET_TOOTH_PITCH_DEG

FIXED_RING_REDUCTION = 1.0 + Z_RING / Z_SUN
assert abs(FIXED_RING_REDUCTION - 4.0) < 1e-9

# ---------------------------------------------------------------------------
# Axial stack and fits.
# All manually created solids are base-aligned and span [z, z + height].
# ---------------------------------------------------------------------------
base_r = 62.0
base_h = 4.0

output_shaft_r = 5.0
output_bearing_bore_r = output_shaft_r + 0.05
output_bearing_outer_r = 9.0
output_bearing_h = 5.0

output_lower_bearing_z = base_h
output_spacer_z = output_lower_bearing_z + output_bearing_h
output_spacer_h = 9.0
output_upper_bearing_z = output_spacer_z + output_spacer_h

carrier_z = 26.0
carrier_h = 4.0
carrier_top_z = carrier_z + carrier_h
carrier_hub_r = 10.0
carrier_arm_w = 8.0
carrier_pin_pad_r = 6.0
carrier_press_bore_r = output_shaft_r - 0.005

output_shaft_z = base_h
output_shaft_h = carrier_top_z - output_shaft_z

planet_pin_r = 2.5
planet_pin_press_r = planet_pin_r
planet_running_bore_r = planet_pin_r + 0.05
planet_pin_z = carrier_z + 1.0
planet_pin_h = 13.0

washer_z = carrier_top_z
washer_h = 1.0
washer_inner_r = planet_pin_r + 0.05
washer_outer_r = planet_pin_r + 2.0

gear_z = washer_z + washer_h
gear_h = 8.0
gear_top_z = gear_z + gear_h

input_shaft_r = 4.0
sun_press_bore_r = input_shaft_r - 0.005
input_shaft_z = gear_z
input_shaft_top_z = 73.0
input_shaft_h = input_shaft_top_z - input_shaft_z

ring_tip_r = RING_PITCH_R - M
ring_root_r = RING_PITCH_R + 1.25 * M
ring_outer_r = ring_root_r + 4.0

ring_support_r = 44.0
ring_support_post_r = 3.0
ring_support_post_z = base_h
ring_support_post_h = gear_z - ring_support_post_z

input_bearing_bore_r = input_shaft_r + 0.05
input_bearing_outer_r = 9.0
input_bearing_h = 6.0

lower_bridge_z = 43.0
lower_bridge_h = input_bearing_h
upper_bridge_z = 55.0
upper_bridge_h = input_bearing_h
bridge_outer_station_r = 54.0
bridge_hub_outer_r = 13.0
bridge_bearing_seat_r = input_bearing_outer_r + 0.02
bridge_arm_w = 7.0

main_post_z = base_h
main_post_h = lower_bridge_z - main_post_z
main_post_r = 3.5
upper_spacer_z = lower_bridge_z + lower_bridge_h
upper_spacer_h = upper_bridge_z - upper_spacer_z

crank_z = 65.0
crank_h = 4.0
crank_radius = 27.0
crank_arm_w = 7.0
crank_hub_r = 8.0
crank_press_bore_r = input_shaft_r - 0.005
handle_r = 4.5
handle_z = crank_z + crank_h
handle_h = 20.0

# Local port coordinates used by mechanism semantics.
PLANET_LOCAL_PORTS = {
    f"planet_{i + 1}": [
        {
            "name": "axis_bore",
            "type": "bore",
            "xyz_mm": [0.0, 0.0, gear_h / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": 2.0 * planet_running_bore_r,
            "depth_mm": gear_h,
        },
        {
            "name": "sun_mesh",
            "type": "gear_mesh",
            "xyz_mm": [0.0, 0.0, gear_h / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "pitch_radius_mm": PLANET_PITCH_R,
        },
        {
            "name": "ring_mesh",
            "type": "gear_mesh",
            "xyz_mm": [0.0, 0.0, gear_h / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "pitch_radius_mm": PLANET_PITCH_R,
        },
    ]
    for i in range(PLANET_COUNT)
}

PIN_LOCAL_PORTS = {
    f"planet_pin_{i + 1}": [
        {
            "name": "journal",
            "type": "shaft",
            "xyz_mm": [0.0, 0.0, planet_pin_h / 2.0],
            "axis": [0.0, 0.0, 1.0],
            "diameter_mm": 2.0 * planet_pin_r,
            "depth_mm": planet_pin_h,
        }
    ]
    for i in range(PLANET_COUNT)
}

MECHANISM = {
    "name": "hand_driven_fixed_ring_planetary_reducer",
    "output_link": "carrier_plate",
    "watch_links": [
        "input_shaft",
        "sun_gear",
        "planet_1",
        "planet_2",
        "planet_3",
        "carrier_plate",
        "output_shaft",
    ],
    "ports_by_link": {
        "input_shaft": [
            {
                "name": "sun_seat",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, gear_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * input_shaft_r,
                "depth_mm": gear_h,
            },
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    lower_bridge_z - input_shaft_z + input_bearing_h / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * input_shaft_r,
                "depth_mm": input_bearing_h,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    upper_bridge_z - input_shaft_z + input_bearing_h / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * input_shaft_r,
                "depth_mm": input_bearing_h,
            },
        ],
        "sun_gear": [
            {
                "name": "shaft_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, gear_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * sun_press_bore_r,
                "depth_mm": gear_h,
            },
            {
                "name": "planet_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, gear_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": SUN_PITCH_R,
            },
        ],
        "fixed_ring_gear": [
            {
                "name": "internal_mesh",
                "type": "gear_mesh",
                "xyz_mm": [0.0, 0.0, gear_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "pitch_radius_mm": RING_PITCH_R,
            }
        ],
        "carrier_plate": [
            {
                "name": "output_bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, carrier_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * carrier_press_bore_r,
                "depth_mm": carrier_h,
            }
        ],
        "output_shaft": [
            {
                "name": "carrier_seat",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    carrier_z - output_shaft_z + carrier_h / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * output_shaft_r,
                "depth_mm": carrier_h,
            },
            {
                "name": "lower_journal",
                "type": "shaft",
                "xyz_mm": [0.0, 0.0, output_bearing_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * output_shaft_r,
                "depth_mm": output_bearing_h,
            },
            {
                "name": "upper_journal",
                "type": "shaft",
                "xyz_mm": [
                    0.0,
                    0.0,
                    output_upper_bearing_z
                    - output_shaft_z
                    + output_bearing_h / 2.0,
                ],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * output_shaft_r,
                "depth_mm": output_bearing_h,
            },
        ],
        "input_lower_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, input_bearing_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * input_bearing_bore_r,
                "depth_mm": input_bearing_h,
            }
        ],
        "input_upper_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, input_bearing_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * input_bearing_bore_r,
                "depth_mm": input_bearing_h,
            }
        ],
        "output_lower_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, output_bearing_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * output_bearing_bore_r,
                "depth_mm": output_bearing_h,
            }
        ],
        "output_upper_bearing": [
            {
                "name": "bore",
                "type": "bore",
                "xyz_mm": [0.0, 0.0, output_bearing_h / 2.0],
                "axis": [0.0, 0.0, 1.0],
                "diameter_mm": 2.0 * output_bearing_bore_r,
                "depth_mm": output_bearing_h,
            }
        ],
        **PLANET_LOCAL_PORTS,
        **PIN_LOCAL_PORTS,
    },
    "relations": [
        {
            "name": "input_shaft_to_sun_press_fit",
            "mate_type": "press_fit",
            "base_part": "input_shaft",
            "base_port": "sun_seat",
            "incoming_part": "sun_gear",
            "incoming_port": "shaft_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "carrier_to_output_shaft_press_fit",
            "mate_type": "press_fit",
            "base_part": "output_shaft",
            "base_port": "carrier_seat",
            "incoming_part": "carrier_plate",
            "incoming_port": "output_bore",
            "offset_mm": 0.0,
        },
        {
            "name": "input_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_lower_bearing",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "lower_journal",
            "offset_mm": 0.0,
        },
        {
            "name": "input_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "input_upper_bearing",
            "base_port": "bore",
            "incoming_part": "input_shaft",
            "incoming_port": "upper_journal",
            "offset_mm": 0.0,
        },
        {
            "name": "output_lower_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_lower_bearing",
            "base_port": "bore",
            "incoming_part": "output_shaft",
            "incoming_port": "lower_journal",
            "offset_mm": 0.0,
        },
        {
            "name": "output_upper_journal",
            "mate_type": "journal_bearing",
            "base_part": "output_upper_bearing",
            "base_port": "bore",
            "incoming_part": "output_shaft",
            "incoming_port": "upper_journal",
            "offset_mm": 0.0,
        },
        *[
            {
                "name": f"planet_{i + 1}_journal",
                "mate_type": "journal_bearing",
                "base_part": f"planet_pin_{i + 1}",
                "base_port": "journal",
                "incoming_part": f"planet_{i + 1}",
                "incoming_port": "axis_bore",
                "offset_mm": 0.0,
            }
            for i in range(PLANET_COUNT)
        ],
    ],
    "motion_joints": [
        {
            "name": "sun_world_hinge",
            "parent": "",
            "child": "sun_gear",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, gear_z + gear_h / 2.0],
        },
        {
            "name": "carrier_world_hinge",
            "parent": "",
            "child": "carrier_plate",
            "type": "hinge",
            "axis": [0.0, 0.0, 1.0],
            "pos_mm": [0.0, 0.0, carrier_z + carrier_h / 2.0],
        },
        *[
            {
                "name": f"planet_{i + 1}_carrier_hinge",
                "parent": "carrier_plate",
                "child": f"planet_{i + 1}",
                "type": "hinge",
                "axis": [0.0, 0.0, 1.0],
                "pos_mm": [
                    PLANET_CENTERS[i][0],
                    PLANET_CENTERS[i][1],
                    gear_z + gear_h / 2.0,
                ],
            }
            for i in range(PLANET_COUNT)
        ],
    ],
    "transmissions": [
        {
            "name": "input_shaft_to_sun",
            "type": "compound_1to1",
            "driving_link": "input_shaft",
            "driven_link": "sun_gear",
            "ratio": 1.0,
        },
        {
            "name": "carrier_to_output_shaft",
            "type": "compound_1to1",
            "driving_link": "carrier_plate",
            "driven_link": "output_shaft",
            "ratio": 1.0,
        },
    ],
    "planetary_stages": [
        {
            "name": "fixed_ring_reduction_stage",
            "sun": "sun_gear",
            "ring": "fixed_ring_gear",
            "carrier": "carrier_plate",
            "planets": [
                {"gear": "planet_1", "pin": "planet_pin_1"},
                {"gear": "planet_2", "pin": "planet_pin_2"},
                {"gear": "planet_3", "pin": "planet_pin_3"},
            ],
            "sun_teeth": Z_SUN,
            "planet_teeth": Z_PLANET,
            "ring_teeth": Z_RING,
            "fixed_member": "ring",
            "input_member": "sun",
            "output_member": "carrier",
        }
    ],
}


def build_machine():
    a = AssemblyHelper("hand_driven_fixed_ring_planetary_reducer")

    def base_aligned_cylinder(radius, height):
        return Cylinder(
            radius,
            height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )

    def annulus(outer_r, inner_r, height):
        outer = base_aligned_cylinder(outer_r, height)
        cutter = Cylinder(
            inner_r,
            height + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -1.0)))
        return outer - cutter

    def make_internal_ring_gear(module, teeth, face_width):
        pr = module * teeth / 2.0
        tip_r = pr - module
        root_r = pr + 1.25 * module
        outer_r = root_r + 4.0

        tooth_pitch = 2.0 * math.pi / teeth
        root_half_angle = 0.34 * tooth_pitch
        tip_half_angle = 0.20 * tooth_pitch
        middle_r = 0.5 * (root_r + tip_r)

        with BuildSketch() as ring_sketch:
            Circle(outer_r)
            Circle(root_r, mode=Mode.SUBTRACT)

            for i in range(teeth):
                angle = i * tooth_pitch

                def polar(radius, theta):
                    return (
                        radius * math.cos(theta),
                        radius * math.sin(theta),
                    )

                points = [
                    polar(root_r, angle - root_half_angle),
                    polar(root_r, angle + root_half_angle),
                    polar(middle_r, angle + 0.75 * root_half_angle),
                    polar(tip_r, angle + tip_half_angle),
                    polar(tip_r, angle - tip_half_angle),
                    polar(middle_r, angle - 0.75 * root_half_angle),
                ]
                Polygon(*points, mode=Mode.ADD)

        return extrude(ring_sketch.sketch, amount=face_width)

    def make_carrier():
        carrier = base_aligned_cylinder(carrier_hub_r, carrier_h)

        for angle_deg, (px, py) in zip(PLANET_ANGLES_DEG, PLANET_CENTERS):
            arm = Box(
                PLANET_ORBIT_R,
                carrier_arm_w,
                carrier_h,
                align=(Align.MIN, Align.CENTER, Align.MIN),
            ).rotate(Axis.Z, angle_deg)
            pad = base_aligned_cylinder(carrier_pin_pad_r, carrier_h).moved(
                Location((px, py, 0.0))
            )
            carrier = carrier + arm + pad

        bore_tool = Cylinder(
            carrier_press_bore_r,
            carrier_h + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -1.0)))
        return carrier - bore_tool

    def make_bridge():
        bridge = annulus(
            bridge_hub_outer_r,
            bridge_bearing_seat_r,
            lower_bridge_h,
        )
        spoke_length = bridge_outer_station_r - bridge_hub_outer_r

        for angle_deg in (60.0, 180.0, 300.0):
            spoke = Box(
                spoke_length,
                bridge_arm_w,
                lower_bridge_h,
                align=(Align.MIN, Align.CENTER, Align.MIN),
            ).moved(Location((bridge_hub_outer_r, 0.0, 0.0)))
            spoke = spoke.rotate(Axis.Z, angle_deg)

            px = bridge_outer_station_r * math.cos(math.radians(angle_deg))
            py = bridge_outer_station_r * math.sin(math.radians(angle_deg))
            end_pad = base_aligned_cylinder(
                main_post_r + 2.0,
                lower_bridge_h,
            ).moved(Location((px, py, 0.0)))

            bridge = bridge + spoke + end_pad

        return bridge

    def make_crank_arm():
        arm = Box(
            crank_radius,
            crank_arm_w,
            crank_h,
            align=(Align.MIN, Align.CENTER, Align.MIN),
        )
        hub = base_aligned_cylinder(crank_hub_r, crank_h)
        end_pad = base_aligned_cylinder(handle_r + 2.0, crank_h).moved(
            Location((crank_radius, 0.0, 0.0))
        )
        crank = hub + arm + end_pad

        bore_tool = Cylinder(
            crank_press_bore_r,
            crank_h + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).moved(Location((0.0, 0.0, -1.0)))
        return crank - bore_tool

    # Grounded base.
    baseplate = base_aligned_cylinder(base_r, base_h)
    a.add(baseplate, "baseplate|dof=fixed")

    # Output shaft bearing stack.
    output_lower_bearing = annulus(
        output_bearing_outer_r,
        output_bearing_bore_r,
        output_bearing_h,
    ).moved(Location((0.0, 0.0, output_lower_bearing_z)))
    a.add(
        output_lower_bearing,
        "output_lower_bearing|dof=fixed|mount=baseplate",
    )

    output_bearing_spacer = annulus(
        output_bearing_outer_r + 2.0,
        output_bearing_outer_r + 0.10,
        output_spacer_h,
    ).moved(Location((0.0, 0.0, output_spacer_z)))
    a.add(
        output_bearing_spacer,
        "output_bearing_spacer|dof=fixed|mount=output_lower_bearing",
    )

    output_upper_bearing = annulus(
        output_bearing_outer_r,
        output_bearing_bore_r,
        output_bearing_h,
    ).moved(Location((0.0, 0.0, output_upper_bearing_z)))
    a.add(
        output_upper_bearing,
        "output_upper_bearing|dof=fixed|mount=output_bearing_spacer",
    )

    output_shaft = base_aligned_cylinder(
        output_shaft_r,
        output_shaft_h,
    ).moved(Location((0.0, 0.0, output_shaft_z)))
    a.add(
        output_shaft,
        "output_shaft|dof=spin|spin_axis=z|"
        "mount=output_lower_bearing,output_upper_bearing",
    )

    # Visible three-arm carrier, press fitted to the output shaft.
    carrier_plate = make_carrier().moved(Location((0.0, 0.0, carrier_z)))
    a.add(
        carrier_plate,
        "carrier_plate|dof=spin|spin_axis=z|mount=output_shaft",
    )

    # Three press-fit carrier pins and three axial thrust washers.
    for i, ((px, py), _) in enumerate(
        zip(PLANET_CENTERS, PLANET_ANGLES_DEG),
        start=1,
    ):
        pin = base_aligned_cylinder(
            planet_pin_press_r,
            planet_pin_h,
        ).moved(Location((px, py, planet_pin_z)))
        a.add(
            pin,
            f"planet_pin_{i}|dof=fixed|mount=carrier_plate",
        )

        washer = annulus(
            washer_outer_r,
            washer_inner_r,
            washer_h,
        ).moved(Location((px, py, washer_z)))
        a.add(
            washer,
            f"planet_thrust_washer_{i}|dof=fixed|mount=carrier_plate",
        )

    # Sun gear and its hand-driven input shaft.
    input_shaft = base_aligned_cylinder(
        input_shaft_r,
        input_shaft_h,
    ).moved(Location((0.0, 0.0, input_shaft_z)))
    a.add(
        input_shaft,
        "input_shaft|dof=spin|driver=True|spin_axis=z|"
        "mount=input_lower_bearing,input_upper_bearing",
    )

    sun_gear = make_gear(
        M,
        Z_SUN,
        gear_h,
        2.0 * sun_press_bore_r,
    ).moved(Location((0.0, 0.0, gear_z)))
    a.add(
        sun_gear,
        "sun_gear|dof=spin|spin_axis=z|mount=input_shaft",
    )

    # Exactly three planets, all at the computed pitch-center radius.
    for i, ((px, py), angle_deg) in enumerate(
        zip(PLANET_CENTERS, PLANET_ANGLES_DEG),
        start=1,
    ):
        planet = make_gear(
            M,
            Z_PLANET,
            gear_h,
            2.0 * planet_running_bore_r,
        ).moved(
            Location(
                (px, py, gear_z),
                (0.0, 0.0, PLANET_PHASE_DEG),
            )
        )
        a.add(
            planet,
            f"planet_{i}|dof=spin|spin_axis=z|"
            f"mount=planet_pin_{i},planet_thrust_washer_{i}",
        )

    # Fixed internal ring at the same axial station as all four external gears.
    for i, angle_deg in enumerate((60.0, 180.0, 300.0), start=1):
        px = ring_support_r * math.cos(math.radians(angle_deg))
        py = ring_support_r * math.sin(math.radians(angle_deg))
        support = base_aligned_cylinder(
            ring_support_post_r,
            ring_support_post_h,
        ).moved(Location((px, py, ring_support_post_z)))
        a.add(
            support,
            f"ring_support_post_{i}|dof=fixed|mount=baseplate",
        )

    fixed_ring = make_internal_ring_gear(
        M,
        Z_RING,
        gear_h,
    ).moved(Location((0.0, 0.0, gear_z)))
    a.add(
        fixed_ring,
        "fixed_ring_gear|dof=fixed|"
        "mount=ring_support_post_1,ring_support_post_2,ring_support_post_3",
    )

    # Three external columns support two open-spoke input bearing bridges.
    for i, angle_deg in enumerate((60.0, 180.0, 300.0), start=1):
        px = bridge_outer_station_r * math.cos(math.radians(angle_deg))
        py = bridge_outer_station_r * math.sin(math.radians(angle_deg))

        main_post = base_aligned_cylinder(
            main_post_r,
            main_post_h,
        ).moved(Location((px, py, main_post_z)))
        a.add(
            main_post,
            f"input_support_post_{i}|dof=fixed|mount=baseplate",
        )

    lower_bridge = make_bridge().moved(
        Location((0.0, 0.0, lower_bridge_z))
    )
    a.add(
        lower_bridge,
        "input_lower_bridge|dof=fixed|"
        "mount=input_support_post_1,input_support_post_2,input_support_post_3",
    )

    input_lower_bearing = annulus(
        input_bearing_outer_r,
        input_bearing_bore_r,
        input_bearing_h,
    ).moved(Location((0.0, 0.0, lower_bridge_z)))
    a.add(
        input_lower_bearing,
        "input_lower_bearing|dof=fixed|mount=input_lower_bridge",
    )

    for i, angle_deg in enumerate((60.0, 180.0, 300.0), start=1):
        px = bridge_outer_station_r * math.cos(math.radians(angle_deg))
        py = bridge_outer_station_r * math.sin(math.radians(angle_deg))

        spacer = base_aligned_cylinder(
            main_post_r,
            upper_spacer_h,
        ).moved(Location((px, py, upper_spacer_z)))
        a.add(
            spacer,
            f"upper_bridge_spacer_{i}|dof=fixed|mount=input_lower_bridge",
        )

    upper_bridge = make_bridge().moved(
        Location((0.0, 0.0, upper_bridge_z))
    )
    a.add(
        upper_bridge,
        "input_upper_bridge|dof=fixed|"
        "mount=upper_bridge_spacer_1,upper_bridge_spacer_2,"
        "upper_bridge_spacer_3",
    )

    input_upper_bearing = annulus(
        input_bearing_outer_r,
        input_bearing_bore_r,
        input_bearing_h,
    ).moved(Location((0.0, 0.0, upper_bridge_z)))
    a.add(
        input_upper_bearing,
        "input_upper_bearing|dof=fixed|mount=input_upper_bridge",
    )

    # Hand crank press fitted to the exposed input shaft.
    crank_arm = make_crank_arm().moved(Location((0.0, 0.0, crank_z)))
    a.add(
        crank_arm,
        "hand_crank_arm|dof=fixed|mount=input_shaft",
    )

    handle_grip = base_aligned_cylinder(
        handle_r,
        handle_h,
    ).moved(Location((crank_radius, 0.0, handle_z)))
    a.add(
        handle_grip,
        "hand_crank_grip|dof=fixed|mount=hand_crank_arm",
    )

    return a.build()