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
      "velocity": <number>          // optional, default 1
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
