"""The manager's output schema + a worked few-shot.

Pure-contact contract (maker2-mujoco-contact): the manager describes the product as a
set of PARTS placed by parent-relative poses. There are NO joints and NO motors — a
part's ability to move is a property of the PART (`dof`), and transmission happens by
real tooth contact under gravity in MuJoCo.

Track 2 (see .claude/plans/precious-humming-wand.md Part A): the manager authors its
decomposition in TWO blocks — a PARTS-list JSON object FIRST, then an MJCF-style XML
skeleton. Parts-first forces the model to enumerate WHAT parts exist before reasoning
about WHERE they go; the MJCF skeleton then places every part as a nested `<body>` and
— because XML (unlike strict JSON) allows COMMENTS — forces a per-part comment stating
its role, interface frame, and what it meshes with. A deterministic parser
(`maker2/mjcf_skeleton.py`) converts the two blocks back into a `KinematicModel`, so the
entire downstream pipeline (gates, assembler, `build_mjcf`, physics) is unchanged. The
skeleton is a DESIGN document, NOT the compiled simulation MJCF.
"""

# Sentinel separating the PARTS block from the MJCF block in the manager's payload. The
# two-phase NOTES->payload split keys on the first "{", so PARTS (a JSON OBJECT) must come
# first; this sentinel then marks where the XML skeleton begins.
MJCF_SENTINEL = "=== MJCF ==="

# Human-readable schema, restated inside the manager prompt so the model has the
# exact field contract in front of it.
SCHEMA_TEXT = f"""\
Author your decomposition in TWO blocks, in THIS order — a PARTS block, then an MJCF block.

BLOCK 1 — PARTS (a single JSON object; NO prose, NO markdown fences):

{{
  "name": "<safe machine/product name, snake_case>",
  "parts": [
    {{
      "name": "<safe slug: lowercase, starts with a letter, [a-z0-9_] only, UNIQUE>",
      "description": "<what this part is; enough for a CAD worker to build it>",
      "shape_hint": "<box | cylinder | sphere | free text>",
      "size_mm": {{ "<dim>": <number>, ... }},   // approx bounding size in MM
      "origin_note": "<where this part's LOCAL origin sits and which way it points>",
      "color": [<r>, <g>, <b>],                 // 0..1 RGB; the part's real-world color
      "dof": "<fixed | spin | free>",           // how this part MOVES (see below)
      "spin_axis": [<x>, <y>, <z>],             // rotation axis for dof "spin" (unit vec)
      "driver": <true|false>,                   // true on the ONE part the test drives
      "material": "<steel | brass | ruby | plastic | aluminum | titanium | rubber | wood | gold>"
                                                // OPTIONAL; what the part is made of ->
                                                // its mass (density x volume) + contact
                                                // friction in the physics sim. Defaults to
                                                // steel. Pick the real material (a ruby
                                                // jewel is light + slippery; a brass plate
                                                // is heavy; a rubber grip has high friction).
    }}
  ],
  "mesh_pairs": [                    // OPTIONAL: gear pairs MEANT to mesh by teeth
    ["<drive_gear_part>", "<driven_gear_part>"], ...
  ]
}}

Then a line containing EXACTLY:  {MJCF_SENTINEL}

BLOCK 2 — MJCF (an MuJoCo-style XML skeleton; NO markdown fences):

<mujoco model="<same name as above>">
  <worldbody>
    <!-- one COMMENT per part: its role, where its interface frame sits, what it meshes with -->
    <body name="<part name>" pos="<x y z METERS>" quat="<w x y z>">
      <!-- nested <body> for each part placed RELATIVE to this one -->
      ...
    </body>
  </worldbody>
</mujoco>

THE MJCF SKELETON — RULES (this is a DESIGN skeleton; a builder recompiles the real sim)
- ONE <body> per part in BLOCK 1, referenced by the SAME name. Every part in PARTS has
  exactly one <body>, and every <body> names a real part — no extras, none missing.
- NEST a <body> inside the body it is placed RELATIVE to (its parent). A body directly
  under <worldbody> is a base/root part placed at the global origin. The model is a
  FOREST — you do NOT need one single connected tree; several roots are fine.
- `pos="x y z"` is in METERS: the vector FROM the parent body's origin TO this body's
  origin (for a root, its offset from the global origin; usually "0 0 0"). `quat="w x y z"`
  is the orientation (identity "1 0 0 0" unless the part is rotated).
- DOF via the body's joint element (this REPLACES joints-between-parts; there are none):
    dof "spin"  -> add  <joint type="hinge" axis="<x y z>" pos="0 0 0"/>  (the spin axle)
    dof "free"  -> add  <freejoint/>
    dof "fixed" -> add NOTHING (the body is welded to its parent).
  The `axis` MUST equal the part's spin_axis in PARTS.
- EVERY <body> carries an XML COMMENT (<!-- ... -->) on the line above it, stating the
  part's role, where its interface frame sits, and what it meshes with / rests on and why
  it is placed there. The comments are REQUIRED — they are how you reason about placement.

INTERFACE FRAMES (subassembly mode) — declare each as a <site> INSIDE its owning body:
    <site name="frame_<contract_frame_name>" pos="<x y z METERS>" euler="<r p y RAD>"/>
  The site's PARENT body is the link that realizes the frame; its pos/euler are the frame
  point in that body's LOCAL frame (meters / radians). Omit euler for an axis-aligned frame.
  (When NOT building a subassembly, you need no sites.)

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
- DEFAULT TO "fixed". The VAST MAJORITY of parts are structure and must be "fixed":
  plates, bridges, cocks, posts/pillars, frames, brackets, housings, bearings, JEWELS,
  screws, pins. A part is "spin" ONLY if it turns UNDER POWER relative to its parent —
  essentially just the wheels, pinions, arbors/shafts, rotors, and a balance. If you
  are unsure, it is "fixed". (A cage/carriage that rotates as a whole is ONE spin part
  — its plates, bridges, posts, and jewels are "fixed" and welded to it, NOT their own
  spin bodies. Marking structure "spin" makes those parts fly apart under gravity.)
- COAXIAL PARTS THAT TURN TOGETHER = ONE spin part, not a stack of spin bodies. A wheel
  + its pinion + the arbor they are pressed onto rotate as a unit: make the ARBOR the
  single "spin" part and mark the wheel and pinion "fixed" (they are welded to the
  arbor and nested under it). Never make several "spin" parts at the SAME xy on the SAME
  axis — as separate rigid bodies their solids interpenetrate.
- A part must occupy its OWN space — no two parts share the same solid. A jewel/bearing
  PRESSES INTO a hole in a plate: offset it in Z so it sits IN the bore, flush or
  proud, not buried at the plate's own origin. Put a wheel and the plate below it at
  DIFFERENT Z so they clear.
- Set `driver": true` on the SINGLE part the physics test spins to drive the machine
  (the input gear/crank/rotor). The test applies torque to that part's own dof; every
  downstream part moves ONLY if its teeth truly contact. At most one driver.
- `mesh_pairs` lists the (drive, driven) gear pairs you INTEND to mesh, by part name.
  This is how the checker knows two gears are supposed to couple (no joint says so).

HARD RULES
- Part names are unique, safe slugs (^[a-z][a-z0-9_]*$). Body names in the MJCF match them.
- Every nested body's parent is a real part; a top-level body is a base/root part.
- "spin" parts need a non-zero `spin_axis` (and a matching hinge `axis` in the MJCF);
  "fixed"/"free" parts get no <joint> (fixed) or a <freejoint> (free).

UNITS / ORIGIN CONTRACT (critical — this is how blindly-built parts line up)
- size_mm is in MILLIMETERS. Body `pos` is in METERS.
- Each worker builds its part ALONE, in the part's own local frame, with the part's
  natural attach/rotation point at the LOCAL ORIGIN (0,0,0). You decide, per part,
  WHERE that origin is and write it in `origin_note` precisely (e.g. "gear center on
  the mid-plane at origin, teeth around +Z axis").
- You author every body's `pos` as the vector FROM the parent part's origin TO where
  this part's origin sits. Workers never position parts relative to each other — all
  spatial relationships live in your body nesting + poses.
- Place parts so they physically MATE the way they must function: meshing gears exactly
  one pitch-center-distance apart with teeth touching; a shaft through its bearing
  bore; a part resting ON the surface below it, not floating above or sunk into it.
  Under gravity, anything unsupported will fall — give every part real support.

COLOR
- Give every part a `color` as [r, g, b] in 0..1 matching the part's real-world
  material (e.g. brushed metal ~[0.75,0.76,0.78], brass ~[0.80,0.62,0.20], black
  plastic ~[0.12,0.12,0.13]). Adjacent parts should differ enough to read apart.

PHYSICAL HARDWARE IS A REAL PART (critical)
- Every real piece of hardware is its OWN part. A rotating shaft/axle turns inside a
  bearing (or bushing/journal): emit BOTH — the bearing/housing as a "fixed" part AND
  the shaft as a separate part with dof "spin" on the bore axis. Do the same for a
  gear-on-shaft, wheel-on-axle, hinge pin. NEVER delete a shaft or bearing — the
  worked example below shows the exact bearing + shaft + spin pattern to copy."""


# A complete, valid worked example used as a one-shot in the prompt. It is a
# MOTORIZED TURNTABLE chosen deliberately to demonstrate the two things the
# manager most often gets wrong:
#   1. PHYSICAL HARDWARE IS A REAL PART. A shaft turns *inside* a bearing. The
#      bearing_block is its own "fixed" part (on the base); the shaft is its own
#      "spin" part on the bore axis; the bearing and the spinning shaft COEXIST.
#      Copy this pattern for every rotating shaft, axle, gear-on-shaft, hinge pin.
#   2. The origin contract: each part's attach point is its local origin; a body's
#      pos (meters) is the parent-origin -> child-origin vector, and nesting is placement.
FEWSHOT_PRODUCT = "a motorized turntable: a platter that spins on a shaft carried by a bearing block on a base"

# The few-shot is the SAME turntable authored in the new PARTS + MJCF form. This is the
# golden example that teaches the format; it round-trips (maker2/tests/golden_mjcf_roundtrip.py)
# to the SAME KinematicModel the old JSON few-shot produced.
FEWSHOT_JSON = f"""\
{{
  "name": "motorized_turntable",
  "parts": [
    {{
      "name": "base",
      "description": "A flat square base plate, 200 x 200 mm, 15 mm thick, that everything mounts to and that rests on the ground.",
      "shape_hint": "box",
      "size_mm": {{"x": 200, "y": 200, "z": 15}},
      "origin_note": "top-face center at local origin; slab extends -Z (0..-15mm), centered in X and Y (-100..100mm)",
      "color": [0.30, 0.30, 0.32],
      "dof": "fixed"
    }},
    {{
      "name": "bearing_block",
      "description": "A pillow-block bearing: a 50 mm cube with a 12 mm vertical bore through its center that the shaft rotates inside. A REAL fixed part.",
      "shape_hint": "box",
      "size_mm": {{"x": 50, "y": 50, "z": 50, "bore_dia": 12}},
      "origin_note": "bottom-face center at local origin; block extends +Z (0..50mm); the 12mm bore runs vertically through the center",
      "color": [0.20, 0.22, 0.25],
      "dof": "fixed"
    }},
    {{
      "name": "shaft",
      "description": "A vertical drive shaft, 12 mm diameter, 90 mm long, that turns inside the bearing bore and carries the platter on top. This is the driver the test spins.",
      "shape_hint": "cylinder",
      "size_mm": {{"radius": 6, "height": 90}},
      "origin_note": "bottom face center at local origin; cylinder extends +Z (0..90mm), coaxial with the bearing bore",
      "color": [0.75, 0.76, 0.78],
      "dof": "spin",
      "spin_axis": [0.0, 0.0, 1.0],
      "driver": true
    }},
    {{
      "name": "platter",
      "description": "A round turntable platter, 160 mm diameter, 8 mm thick, fixed on top of the shaft (rotates with it).",
      "shape_hint": "cylinder",
      "size_mm": {{"radius": 80, "height": 8}},
      "origin_note": "bottom-face center at local origin; disc extends +Z (0..8mm)",
      "color": [0.10, 0.10, 0.11],
      "dof": "spin",
      "spin_axis": [0.0, 0.0, 1.0]
    }}
  ],
  "mesh_pairs": []
}}
{MJCF_SENTINEL}
<mujoco model="motorized_turntable">
  <worldbody>
    <!-- base: structural root plate resting on the ground; everything mounts to its top face at z=0 -->
    <body name="base" pos="0 0 0" quat="1 0 0 0">
      <!-- bearing_block: fixed pillow block bolted to the base top; its vertical bore carries the shaft -->
      <body name="bearing_block" pos="0 0 0" quat="1 0 0 0">
        <!-- shaft: the driver; spins on +Z inside the bearing bore, coaxial with the block -->
        <body name="shaft" pos="0 0 0" quat="1 0 0 0">
          <joint name="shaft_spin" type="hinge" axis="0 0 1" pos="0 0 0"/>
          <!-- platter: seats on the shaft top (90mm up) and turns with it; no mesh, driven directly -->
          <body name="platter" pos="0 0 0.090" quat="1 0 0 0">
            <joint name="platter_spin" type="hinge" axis="0 0 1" pos="0 0 0"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>"""


# --------------------------------------------------------------------------- #
# BOSS schema + few-shot (hierarchy). The boss splits a big machine into
# SUBASSEMBLIES (each one manager's job, <=35 links) and authors the INTERFACE/
# FRAME CONTRACT: named mount frames in GLOBAL meters + the SEAMS that join the
# subassemblies. The assembler stitches the per-sub models into one KinematicModel
# deterministically from this. See maker2/boss.py and the plan.
# --------------------------------------------------------------------------- #

BOSS_SCHEMA_TEXT = """\
Return exactly one JSON object (no prose, no markdown fences) with this shape:

{
  "name": "<safe machine name, snake_case>",
  "root_sub": "<id of the single ROOT subassembly everything hangs off>",
  "global_origin_note": "<where the shared global origin is and axis convention>",
  "subassemblies": [
    {
      "id": "<safe slug: lowercase, starts with a letter, [a-z0-9_] only, unique>",
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
          "name": "<safe frame name, unique within the sub>",
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
      "id": "<safe seam name>",
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

