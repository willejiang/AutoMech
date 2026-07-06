"""The manager's output schema + a worked few-shot.

The manager returns ONE JSON object describing the product as a kinematic tree
of links and joints. We deliberately use plain JSON (not native tool-calling) —
the gateway's tool support is unverified, and a single JSON object is easy to
validate and repair.
"""

# Human-readable schema, restated inside the manager prompt so the model has the
# exact field contract in front of it.
SCHEMA_TEXT = """\
Return exactly one JSON object (no prose, no markdown fences) with this shape:

{
  "name": "<urdf-safe robot/product name, snake_case>",
  "root_link": "<name of the single root link>",
  "links": [
    {
      "name": "<urdf-safe: lowercase, starts with a letter, [a-z0-9_] only>",
      "description": "<what this part is; enough for a CAD worker to build it>",
      "shape_hint": "<box | cylinder | sphere | free text>",
      "size_mm": { "<dim>": <number>, ... },   // approx bounding size in MM
      "origin_note": "<where this part's LOCAL origin sits and which way it points>",
      "color": [<r>, <g>, <b>]                  // 0..1 RGB; the part's real-world color
    }
  ],
  "joints": [
    {
      "name": "<urdf-safe joint name>",
      "type": "<fixed | revolute | prismatic | continuous>",
      "parent": "<link name>",
      "child": "<link name>",
      "xyz_m": [<x>, <y>, <z>],     // METERS: parent-origin -> child-origin
      "rpy_rad": [<r>, <p>, <y>],   // radians, fixed-axis XYZ
      "axis": [<x>, <y>, <z>],      // required for non-fixed joints (unit vector)
      "lower": <number>,            // required for revolute/prismatic (rad or m)
      "upper": <number>,            // required for revolute/prismatic
      "effort": <number>,           // optional, default 10
      "velocity": <number>,         // optional, default 1
      "driver": <true|false>        // optional: true on the ONE input joint a user
                                    // drives (crank/handle) — the physics test
                                    // actuates it. Omit/false for all other joints.
    }
  ]
}

HARD RULES
- Exactly ONE root link (a link that is never a joint's child).
- The links + joints must form a single connected tree: every non-root link is
  the child of exactly one joint; no cycles; no orphans.
- Joint parent/child must reference link names that exist.
- Link and joint names are unique and URDF-safe (^[a-z][a-z0-9_]*$).
- "fixed" joints omit axis/lower/upper. "revolute"/"prismatic" REQUIRE a
  non-zero axis and lower < upper. "continuous" needs an axis, no limits.
- If the product is a MACHINE the user drives (a crank, winder, hand wheel, input
  shaft), set "driver": true on the SINGLE input joint they turn. The physics test
  drives that joint to check the mechanism transmits motion. At most one driver.

UNITS / ORIGIN CONTRACT (critical — this is how blindly-built parts line up)
- size_mm is in MILLIMETERS. Joint xyz_m is in METERS.
- Each worker builds its part ALONE, in the part's own local frame, with the
  part's joint-attachment point at the LOCAL ORIGIN (0,0,0). You decide, per
  link, WHERE that origin is and write it in `origin_note` precisely (e.g.
  "top face center at origin, body extends -Z").
- You author every joint's `xyz_m` as the vector FROM the parent link's origin
  TO the point where the child link's origin attaches. Workers never position
  parts relative to each other — all spatial relationships live in your joints.
- Pick origins and joint offsets so the assembled parts touch/mate without
  floating gaps or overlaps.

COLOR
- Give every link a `color` as [r, g, b] in 0..1 matching the part's real-world
  material (e.g. brushed metal ~[0.75,0.76,0.78], brass ~[0.80,0.62,0.20], black
  plastic ~[0.12,0.12,0.13]). Adjacent parts should differ enough to read apart.

PHYSICAL HARDWARE IS A PART, NOT A JOINT (critical)
- A joint is a massless kinematic relationship; it is NOT a physical object. Every
  real piece of hardware must be its OWN link, even when a joint also acts there.
- A rotating shaft/axle turns INSIDE a bearing (or bushing/journal). Emit BOTH:
  the bearing (or housing) as a fixed link AND the shaft as a separate link, and
  connect them with a continuous/revolute joint whose axis runs through the bore.
  Do the same for hinge pins, gear-on-shaft, wheel-on-axle: the pin/shaft/axle is
  a real link, the joint is on top of it. NEVER delete a shaft or bearing just
  because a joint could represent the motion — the worked example below shows the
  exact bearing_block + shaft + continuous-joint pattern to copy."""


# A complete, valid worked example used as a one-shot in the prompt. It is a
# MOTORIZED TURNTABLE chosen deliberately to demonstrate the two things the
# manager most often gets wrong:
#   1. PHYSICAL HARDWARE IS A REAL PART, NOT A JOINT. A shaft turns *inside* a
#      bearing. The bearing_block is its own link (fixed to the base); the shaft
#      is its own link; the rotation is a continuous joint whose axis runs
#      through the bearing bore. The joint and the bearing COEXIST -- the joint
#      does not "replace" the bearing. Copy this pattern for every rotating
#      shaft, axle, gear-on-shaft, hinge pin, etc.
#   2. The origin contract: each part's attach point is its local origin; joint
#      xyz_m (meters) is the parent-origin -> child-origin vector.
FEWSHOT_PRODUCT = "a motorized turntable: a platter that spins on a shaft carried by a bearing block on a base"

FEWSHOT_JSON = """\
{
  "name": "motorized_turntable",
  "root_link": "base",
  "links": [
    {
      "name": "base",
      "description": "A flat square base plate, 200 x 200 mm, 15 mm thick, that everything mounts to.",
      "shape_hint": "box",
      "size_mm": {"x": 200, "y": 200, "z": 15},
      "origin_note": "top-face center at local origin; slab extends -Z (0..-15mm), centered in X and Y (-100..100mm)",
      "color": [0.30, 0.30, 0.32]
    },
    {
      "name": "bearing_block",
      "description": "A pillow-block bearing: a 50 mm cube with a 12 mm vertical bore through its center that the shaft rotates inside. A REAL part, not the joint.",
      "shape_hint": "box",
      "size_mm": {"x": 50, "y": 50, "z": 50, "bore_dia": 12},
      "origin_note": "bottom-face center at local origin; block extends +Z (0..50mm); the 12mm bore runs vertically through the center",
      "color": [0.20, 0.22, 0.25]
    },
    {
      "name": "shaft",
      "description": "A vertical drive shaft, 12 mm diameter, 90 mm long, that turns inside the bearing bore and carries the platter on top.",
      "shape_hint": "cylinder",
      "size_mm": {"radius": 6, "height": 90},
      "origin_note": "bottom face center at local origin; cylinder extends +Z (0..90mm), coaxial with the bearing bore",
      "color": [0.75, 0.76, 0.78]
    },
    {
      "name": "platter",
      "description": "A round turntable platter, 160 mm diameter, 8 mm thick, fixed to the top of the shaft.",
      "shape_hint": "cylinder",
      "size_mm": {"radius": 80, "height": 8},
      "origin_note": "bottom-face center at local origin; disc extends +Z (0..8mm)",
      "color": [0.10, 0.10, 0.11]
    }
  ],
  "joints": [
    {
      "name": "base_to_bearing_block",
      "type": "fixed",
      "parent": "base",
      "child": "bearing_block",
      "xyz_m": [0.0, 0.0, 0.0],
      "rpy_rad": [0.0, 0.0, 0.0]
    },
    {
      "name": "shaft_in_bearing",
      "type": "continuous",
      "parent": "bearing_block",
      "child": "shaft",
      "xyz_m": [0.0, 0.0, 0.0],
      "rpy_rad": [0.0, 0.0, 0.0],
      "axis": [0.0, 0.0, 1.0]
    },
    {
      "name": "shaft_to_platter",
      "type": "fixed",
      "parent": "shaft",
      "child": "platter",
      "xyz_m": [0.0, 0.0, 0.090],
      "rpy_rad": [0.0, 0.0, 0.0]
    }
  ]
}"""


# --------------------------------------------------------------------------- #
# BOSS schema + few-shot (hierarchy). The boss splits a big machine into
# SUBASSEMBLIES (each one manager's job, <=35 links) and authors the INTERFACE/
# FRAME CONTRACT: named mount frames in GLOBAL meters + the SEAMS that join the
# subassemblies. The assembler stitches the per-sub URDFs deterministically from
# this. See maker2/boss.py and .claude/plans/precious-humming-wand.md.
# --------------------------------------------------------------------------- #

BOSS_SCHEMA_TEXT = """\
Return exactly one JSON object (no prose, no markdown fences) with this shape:

{
  "name": "<urdf-safe machine name, snake_case>",
  "root_sub": "<id of the single ROOT subassembly everything hangs off>",
  "global_origin_note": "<where the shared global origin is and axis convention>",
  "subassemblies": [
    {
      "id": "<urdf-safe: lowercase, starts with a letter, [a-z0-9_] only, unique>",
      "brief": "<one-paragraph product prompt for THIS subassembly's manager: what
                 parts it contains and what it does; it is built in isolation>",
      "function": "<what this subassembly does in the machine>",
      "est_link_budget": <int <=35>,          // keep each manager under the output cap
      "input_tags": ["<frame name that is a power INPUT>", ...],
      "output_tags": ["<frame name that is a power OUTPUT>", ...],
      "instances": [                            // OPTIONAL — ONLY for a sub that REPEATS
        {"xyz_m": [<x>,<y>,<z>], "rpy_rad": [<r>,<p>,<y>]},  // GLOBAL pose of copy #0's ROOT
        {"xyz_m": [<x>,<y>,<z>], "rpy_rad": [<r>,<p>,<y>]}   // ... copy #1's root, etc.
      ],                                        // omit or [] for a normal single sub
      "frames": [
        {
          "name": "<urdf-safe frame name, unique within the sub>",
          "xyz_m": [<x>, <y>, <z>],   // GLOBAL METERS (shared origin), where this
                                       // interface sits in the assembled machine
          "rpy_rad": [<r>, <p>, <y>], // GLOBAL radians, fixed-axis XYZ
          "axis": [<x>, <y>, <z>],    // frame primary axis (e.g. a shaft/gear axis)
          "role": "<mount | power_in | power_out | mesh>"
        }
      ]
    }
  ],
  "seams": [
    {
      "id": "<urdf-safe seam name>",
      "kind": "<weld | power>",
      "parent_sub": "<sub id>", "parent_frame": "<frame name on parent_sub>",
      "child_sub":  "<sub id>", "child_frame":  "<frame name on child_sub>",
      "joint_type": "<fixed for weld; continuous/revolute only for a shared-DOF power seam>",
      "axis": [<x>, <y>, <z>],       // for a non-fixed power seam
      "driver": <true|false>,         // true on the seam carrying the machine's single power input
      "owner_sub": "<for a power seam, the sub that owns the DRIVING link>",
      "mesh_pair": ["<drive_link>", "<driven_link>"]  // for a GEAR-MESH power seam only
    }
  ]
}

HARD RULES
- Split the machine into coherent functional subassemblies (an input/crank stage, a
  gear train, an escapement, a barrel, a chassis, a drivetrain, ...). Size each so it
  is a sensible unit for one manager + worker — roughly up to ~20 links; never above
  25. Include every real part WITHIN each subassembly (don't drop shafts/bearings to
  hit a number). Prefer splitting a large machine into MORE subassemblies over a few
  huge ones, but do NOT over-split a simple mechanism into trivial 1-2 part subs.
- IDENTICAL REPEATED SUBASSEMBLIES: when several subassemblies are the SAME (same
  parts, differing ONLY in position/orientation — a quadcopter's 4 rotors, a hexapod's
  6 legs, a car's 4 wheels), emit ONE subassembly and list each copy in "instances"
  (the GLOBAL xyz_m/rpy_rad pose of that copy's ROOT), instead of N separate specs. It
  is BUILT ONCE and the assembler stamps out the copies at those poses — this saves the
  whole machine from N redundant builds. Give it ONE weld seam (parent = the hub it
  bolts to, child = this sub); the assembler expands that seam into one weld per
  instance. Only do this when the parts are TRULY identical; if copies differ in
  geometry, keep them as separate subs.
- ONE global origin. EVERY frame's xyz_m/rpy_rad is in GLOBAL coordinates about
  that origin, so the assembler can place subassemblies without guessing.
- Exactly ONE root_sub. Every OTHER subassembly must be reachable through at least
  one "weld" seam (welds form the structural tree that holds the machine together).
- A "weld" seam is a fixed structural bolt/mate between two frames (joint_type
  "fixed"). Use welds to hold housings/frames/bearing-blocks in place.
- A "power" seam is where MOTION crosses a boundary. Two forms:
    * GEAR MESH: two gears (one per sub) couple by tooth contact. The STRUCTURAL
      link is STILL a weld between the housings holding the gear centers one mesh
      center-distance apart; add a SEPARATE "power" seam with mesh_pair =
      [drive_gear_link, driven_gear_link] and owner_sub = the driving sub. Do NOT
      make the gear pair a joint — they mesh geometrically.
    * SHARED SHAFT (advanced): a single shaft spanning two subs. Keep the rotating
      DOF inside the owner sub; the seam is a weld carrying torque. (A true
      articulating shared joint is not supported yet — don't emit joint_type
      continuous/revolute on a seam unless the seam itself must rotate.)
- Exactly ONE seam in the whole machine has driver:true — the single power input.
- Every seam's frames must EXIST on the named subs, and each sub must list a frame
  for every seam it participates in.

FRAME PLACEMENT (critical — this is how blindly-built subs line up)
- Each manager builds its subassembly ALONE in its own local frame, then reports
  where it actually put each interface frame. You give the GLOBAL target; the
  geometric pre-check verifies the managers' realized frames coincide.
- For a gear-mesh seam, place the two mesh frames (role "mesh") at the two gear
  CENTERS, exactly one meshing center-distance apart, on parallel axes."""


# A minimal, valid worked example = the crank-gear -> driven-gear milestone: two
# subs coupled by a GEAR MESH across a weld. `sub_crank` (crank + drive_gear) is
# the root; `sub_output` (driven_gear + output_shaft) welds its housing to the
# crank housing so the two gears mesh, and a "power" seam names the meshing pair.
BOSS_FEWSHOT_PRODUCT = "a hand crank that turns a drive gear which meshes a driven gear on an output shaft"

BOSS_FEWSHOT_JSON = """\
{
  "name": "crank_gear_train",
  "root_sub": "sub_crank",
  "global_origin_note": "origin at the drive-gear center on the base plane; +Z up, gear axes along +Z, the two gear centers lie on the X axis",
  "subassemblies": [
    {
      "id": "sub_crank",
      "brief": "A hand-crank input stage: a base/bearing-block, a crank handle on a short input shaft turning in the bearing, and a drive gear (module 2, 20 teeth, ~20 mm pitch radius) fixed on that shaft. The drive gear is the driver the user turns.",
      "function": "input: convert the user's crank rotation into drive-gear rotation",
      "est_link_budget": 6,
      "input_tags": [],
      "output_tags": ["drive_gear_center"],
      "frames": [
        {"name": "housing_mount", "xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "role": "mount"},
        {"name": "drive_gear_center", "xyz_m": [0.0, 0.0, 0.03], "rpy_rad": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "role": "mesh"}
      ]
    },
    {
      "id": "sub_output",
      "brief": "An output stage: a bearing-block, a driven gear (module 2, 20 teeth, ~20 mm pitch radius) on an output shaft turning in the bearing. The driven gear meshes the crank's drive gear; the output shaft is the mechanism output.",
      "function": "output: receive motion from the drive gear and turn the output shaft",
      "est_link_budget": 5,
      "input_tags": ["driven_gear_center"],
      "output_tags": [],
      "frames": [
        {"name": "housing_mount", "xyz_m": [0.04, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "role": "mount"},
        {"name": "driven_gear_center", "xyz_m": [0.04, 0.0, 0.03], "rpy_rad": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "role": "mesh"}
      ]
    }
  ],
  "seams": [
    {
      "id": "weld_housings",
      "kind": "weld",
      "parent_sub": "sub_crank", "parent_frame": "housing_mount",
      "child_sub": "sub_output", "child_frame": "housing_mount",
      "joint_type": "fixed"
    },
    {
      "id": "gear_mesh",
      "kind": "power",
      "parent_sub": "sub_crank", "parent_frame": "drive_gear_center",
      "child_sub": "sub_output", "child_frame": "driven_gear_center",
      "joint_type": "fixed",
      "axis": [0.0, 0.0, 1.0],
      "driver": true,
      "owner_sub": "sub_crank",
      "mesh_pair": ["drive_gear", "driven_gear"]
    }
  ]
}"""

