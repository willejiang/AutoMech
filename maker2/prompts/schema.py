"""The manager's output schema + a worked few-shot.

Pure-contact contract (maker2-mujoco-contact): the manager returns ONE JSON object
describing the product as a set of PARTS placed by parent-relative POSES. There are
NO joints and NO motors — a part's ability to move is a property of the PART
(`dof`), and transmission happens by real tooth contact under gravity in MuJoCo. We
deliberately use plain JSON (not native tool-calling) — the gateway's tool support
is unverified, and a single JSON object is easy to validate and repair.
"""

# Human-readable schema, restated inside the manager prompt so the model has the
# exact field contract in front of it.
SCHEMA_TEXT = """\
Return exactly one JSON object (no prose, no markdown fences) with this shape:

{
  "name": "<urdf-safe robot/product name, snake_case>",
  "root_link": "<name of the part that everything else is positioned relative to
                 (usually the base/frame that rests on the ground)>",
  "links": [
    {
      "name": "<urdf-safe: lowercase, starts with a letter, [a-z0-9_] only>",
      "description": "<what this part is; enough for a CAD worker to build it>",
      "shape_hint": "<box | cylinder | sphere | free text>",
      "size_mm": { "<dim>": <number>, ... },   // approx bounding size in MM
      "origin_note": "<where this part's LOCAL origin sits and which way it points>",
      "color": [<r>, <g>, <b>],                 // 0..1 RGB; the part's real-world color
      "dof": "<fixed | spin | free>",           // how this part MOVES (see below)
      "spin_axis": [<x>, <y>, <z>],             // rotation axis for dof "spin" (unit vec)
      "driver": <true|false>                    // true on the ONE part the test drives
    }
  ],
  "poses": [
    {
      "name": "<urdf-safe pose name>",
      "parent": "<link name this pose is relative to, or \"\" for a base/root part>",
      "child": "<link name being placed>",
      "xyz_m": [<x>, <y>, <z>],     // METERS: parent-origin -> child-origin
      "rpy_rad": [<r>, <p>, <y>]    // radians, fixed-axis XYZ
    }
  ],
  "mesh_pairs": [                    // OPTIONAL: gear pairs MEANT to mesh by teeth
    ["<drive_gear_link>", "<driven_gear_link>"], ...
  ]
}

HOW PARTS MOVE (this REPLACES joints — there are NO joints and NO motors)
- Every part declares a `dof`:
    "fixed" -> welded in place relative to its parent (a frame, housing, bracket, the
               base). Most structural parts are "fixed".
    "spin"  -> rotates freely about an implied axle along `spin_axis` (a gear on a
               shaft, a wheel on an axle, a rotor). Give it a `spin_axis` unit vector.
    "free"  -> a free-floating body with full 6-DOF (a loose ball, a pendulum bob that
               is not pinned). Use sparingly.
- Motion is transmitted ONLY by physical contact under gravity: meshing gear teeth
  push each other, a cam lifts a follower, a weight falls. Nothing is driven by a
  motor or held by an invisible joint. So parts must actually TOUCH to interact.
- Set `driver": true` on the SINGLE part the physics test spins to drive the machine
  (the input gear/crank/rotor). The test applies torque to that part's own dof; every
  downstream part moves ONLY if its teeth truly contact. At most one driver.
- `mesh_pairs` lists the (drive, driven) gear pairs you INTEND to mesh, by link name.
  This is how the checker knows two gears are supposed to couple (no joint says so).

HARD RULES
- Link and pose names are unique and URDF-safe (^[a-z][a-z0-9_]*$).
- Every pose `child` must be a real link name; `parent` is a real link name or "".
- A part with no pose (or a pose with parent "") is a base/root part placed at the
  origin. The model is a FOREST — you do NOT need one single connected tree.
- "spin" parts need a non-zero `spin_axis`; "fixed"/"free" ignore it.

UNITS / ORIGIN CONTRACT (critical — this is how blindly-built parts line up)
- size_mm is in MILLIMETERS. Pose xyz_m is in METERS.
- Each worker builds its part ALONE, in the part's own local frame, with the part's
  natural attach/rotation point at the LOCAL ORIGIN (0,0,0). You decide, per link,
  WHERE that origin is and write it in `origin_note` precisely (e.g. "gear center on
  the mid-plane at origin, teeth around +Z axis").
- You author every pose's `xyz_m` as the vector FROM the parent link's origin TO where
  the child link's origin sits. Workers never position parts relative to each other —
  all spatial relationships live in your poses.
- Place parts so they physically MATE the way they must function: meshing gears exactly
  one pitch-center-distance apart with teeth touching; a shaft through its bearing
  bore; a part resting ON the surface below it, not floating above or sunk into it.
  Under gravity, anything unsupported will fall — give every part real support.

COLOR
- Give every link a `color` as [r, g, b] in 0..1 matching the part's real-world
  material (e.g. brushed metal ~[0.75,0.76,0.78], brass ~[0.80,0.62,0.20], black
  plastic ~[0.12,0.12,0.13]). Adjacent parts should differ enough to read apart.

PHYSICAL HARDWARE IS A REAL PART (critical)
- Every real piece of hardware is its OWN link. A rotating shaft/axle turns inside a
  bearing (or bushing/journal): emit BOTH — the bearing/housing as a "fixed" link AND
  the shaft as a separate link with dof "spin" on the bore axis. Do the same for a
  gear-on-shaft, wheel-on-axle, hinge pin. NEVER delete a shaft or bearing — the
  worked example below shows the exact bearing + shaft + spin pattern to copy."""


# A complete, valid worked example used as a one-shot in the prompt. It is a
# MOTORIZED TURNTABLE chosen deliberately to demonstrate the two things the
# manager most often gets wrong:
#   1. PHYSICAL HARDWARE IS A REAL PART. A shaft turns *inside* a bearing. The
#      bearing_block is its own "fixed" link (on the base); the shaft is its own
#      "spin" link on the bore axis; the bearing and the spinning shaft COEXIST.
#      Copy this pattern for every rotating shaft, axle, gear-on-shaft, hinge pin.
#   2. The origin contract: each part's attach point is its local origin; pose
#      xyz_m (meters) is the parent-origin -> child-origin vector.
FEWSHOT_PRODUCT = "a motorized turntable: a platter that spins on a shaft carried by a bearing block on a base"

FEWSHOT_JSON = """\
{
  "name": "motorized_turntable",
  "root_link": "base",
  "links": [
    {
      "name": "base",
      "description": "A flat square base plate, 200 x 200 mm, 15 mm thick, that everything mounts to and that rests on the ground.",
      "shape_hint": "box",
      "size_mm": {"x": 200, "y": 200, "z": 15},
      "origin_note": "top-face center at local origin; slab extends -Z (0..-15mm), centered in X and Y (-100..100mm)",
      "color": [0.30, 0.30, 0.32],
      "dof": "fixed"
    },
    {
      "name": "bearing_block",
      "description": "A pillow-block bearing: a 50 mm cube with a 12 mm vertical bore through its center that the shaft rotates inside. A REAL fixed part.",
      "shape_hint": "box",
      "size_mm": {"x": 50, "y": 50, "z": 50, "bore_dia": 12},
      "origin_note": "bottom-face center at local origin; block extends +Z (0..50mm); the 12mm bore runs vertically through the center",
      "color": [0.20, 0.22, 0.25],
      "dof": "fixed"
    },
    {
      "name": "shaft",
      "description": "A vertical drive shaft, 12 mm diameter, 90 mm long, that turns inside the bearing bore and carries the platter on top. This is the driver the test spins.",
      "shape_hint": "cylinder",
      "size_mm": {"radius": 6, "height": 90},
      "origin_note": "bottom face center at local origin; cylinder extends +Z (0..90mm), coaxial with the bearing bore",
      "color": [0.75, 0.76, 0.78],
      "dof": "spin",
      "spin_axis": [0.0, 0.0, 1.0],
      "driver": true
    },
    {
      "name": "platter",
      "description": "A round turntable platter, 160 mm diameter, 8 mm thick, fixed on top of the shaft (rotates with it).",
      "shape_hint": "cylinder",
      "size_mm": {"radius": 80, "height": 8},
      "origin_note": "bottom-face center at local origin; disc extends +Z (0..8mm)",
      "color": [0.10, 0.10, 0.11],
      "dof": "spin",
      "spin_axis": [0.0, 0.0, 1.0]
    }
  ],
  "poses": [
    {
      "name": "place_bearing_block",
      "parent": "base",
      "child": "bearing_block",
      "xyz_m": [0.0, 0.0, 0.0],
      "rpy_rad": [0.0, 0.0, 0.0]
    },
    {
      "name": "place_shaft",
      "parent": "bearing_block",
      "child": "shaft",
      "xyz_m": [0.0, 0.0, 0.0],
      "rpy_rad": [0.0, 0.0, 0.0]
    },
    {
      "name": "place_platter",
      "parent": "shaft",
      "child": "platter",
      "xyz_m": [0.0, 0.0, 0.090],
      "rpy_rad": [0.0, 0.0, 0.0]
    }
  ],
  "mesh_pairs": []
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
          "shaft_dia_mm": <number>,   // HARD shaft/gear-pitch diameter in MM for a
                                       // power_in/power_out/mesh frame (0 for a plain mount)
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
  CENTERS, exactly one meshing center-distance apart, on parallel axes.

HARD INTERFACE POINTS (you OWN these; managers must NOT move them)
- You define — and ONLY you define — the hard points where subassemblies couple:
  every power_in / power_out / mesh frame carries its GLOBAL xyz_m, its `axis`, and
  its `shaft_dia_mm` (the shaft diameter, or for a mesh the gear PITCH diameter).
  These are immovable: a manager sizes its mating shaft/bore/gear to EXACTLY these
  numbers and places the frame at EXACTLY that global pose. Workers must not offset
  them.
- You OWN gear center distance. For a gear mesh, the center distance = the sum of the
  two gears' PITCH radii; set the two mesh frames that far apart and give each its
  pitch diameter via shaft_dia_mm. If a later fault says two gears don't mesh, YOU
  recompute the center distance and override BOTH subs' mesh-frame coords — the
  managers do not negotiate it between themselves.
- A shared shaft crossing a weld: give the power_out frame on the driving sub and the
  power_in frame on the driven sub the SAME axis and the SAME shaft_dia_mm so the two
  halves are the same shaft."""


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
        {"name": "housing_mount", "xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "shaft_dia_mm": 0.0, "role": "mount"},
        {"name": "drive_gear_center", "xyz_m": [0.0, 0.0, 0.03], "rpy_rad": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "shaft_dia_mm": 40.0, "role": "mesh"}
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
        {"name": "housing_mount", "xyz_m": [0.04, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "shaft_dia_mm": 0.0, "role": "mount"},
        {"name": "driven_gear_center", "xyz_m": [0.04, 0.0, 0.03], "rpy_rad": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "shaft_dia_mm": 40.0, "role": "mesh"}
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

