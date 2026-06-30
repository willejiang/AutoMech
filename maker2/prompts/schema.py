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
  plastic ~[0.12,0.12,0.13]). Adjacent parts should differ enough to read apart."""


# A complete, valid worked example (a table) used as a one-shot in the prompt.
# It demonstrates the origin contract: the tabletop's underside-center is its
# origin; the leg's TOP is its origin (extending -Z); so the fixed joint offset
# is zero and the leg mates flush under the top.
FEWSHOT_PRODUCT = "a simple table: a flat square top on a single central leg"

FEWSHOT_JSON = """\
{
  "name": "simple_table",
  "root_link": "tabletop",
  "links": [
    {
      "name": "tabletop",
      "description": "A flat square table top, 400 x 400 mm, 20 mm thick.",
      "shape_hint": "box",
      "size_mm": {"x": 400, "y": 400, "z": 20},
      "origin_note": "underside-center at local origin; slab extends +Z (0..20mm) and is centered in X and Y (-200..200mm)",
      "color": [0.55, 0.35, 0.20]
    },
    {
      "name": "leg",
      "description": "A central cylindrical leg, 40 mm diameter, 500 mm long.",
      "shape_hint": "cylinder",
      "size_mm": {"radius": 20, "height": 500},
      "origin_note": "top face center at local origin; cylinder extends -Z downward (0..-500mm)",
      "color": [0.30, 0.30, 0.32]
    }
  ],
  "joints": [
    {
      "name": "top_to_leg",
      "type": "fixed",
      "parent": "tabletop",
      "child": "leg",
      "xyz_m": [0.0, 0.0, 0.0],
      "rpy_rad": [0.0, 0.0, 0.0]
    }
  ]
}"""
