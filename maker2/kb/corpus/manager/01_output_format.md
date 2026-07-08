# Manager output format: parts-list + placement contract

The manager decomposes a product into PARTS and places them by parent-relative
POSES. This is a pure-contact model: there are NO joints and NO motors. Whether a
part can move is a property of the PART itself (its `dof`), and motion is
transmitted only by real physical contact under gravity (meshing teeth, a cam
lifting a follower, a falling weight).

## The two blocks the manager emits

1. A PARTS list — one entry per physical part, with its geometry parameters:
   `name`, `shape_hint`, `size_mm`, `origin_note`, `color`, `material`, `driver`.
   Plus a top-level `mesh_pairs` listing the gear pairs meant to couple by teeth.
2. An MJCF-style placement skeleton — one nested `<body name pos quat>` per part,
   each carrying an XML comment stating the part's role, where its interface frame
   sits, what it meshes with, and why it is placed there. A `<joint type="hinge"
   axis>` marks a spin part; a `<freejoint>` marks a free part; nothing marks a
   fixed part. Interface frames are `<site name="frame_<name>" pos rpy>` inside the
   owning body.

The skeleton is an AUTHORING aid, not the final simulation MJCF: a deterministic
parser converts it (plus the parts list) back into a KinematicModel, and the whole
downstream pipeline (gates, assembler, MJCF compiler, physics) consumes that.

## Field contract (frozen — every consumer relies on these exact names)

A part (LinkSpec):
- `name` — safe slug, lowercase, starts with a letter, `[a-z0-9_]` only, unique.
- `shape_hint` — `box | cylinder | sphere | gear | free text`.
- `size_mm` — approximate bounding size in MILLIMETERS, keyed by the canonical
  dimension vocabulary (see the dimension-vocabulary doc).
- `origin_note` — where the part's LOCAL origin sits and which way it points.
- `color` — `[r, g, b]` in 0..1, matching the part's real material.
- `dof` — `fixed | spin | free` (see "how parts move").
- `spin_axis` — unit vector; the rotation axis when `dof` is `spin`.
- `driver` — true on the ONE part the physics test drives. At most one.
- `material` — optional; `steel | brass | ruby | plastic | aluminum | titanium |
  rubber | wood | gold`. Sets mass (density x volume) and contact friction.

A placement (PoseSpec):
- `parent` — the link name this pose is relative to, or "" for a base/root part.
- `child` — the link being placed.
- `xyz_m` — METERS, the vector from the parent origin to the child origin.
- `rpy_rad` — radians, fixed-axis XYZ.

Top level: `name` (snake_case product name), `root_link` (the part everything is
positioned relative to, usually the base that rests on the ground), `mesh_pairs`
(`[[drive_link, driven_link], ...]`).

## Units and origin contract (this is how blindly-built parts line up)

- `size_mm` is in MILLIMETERS. Pose `xyz_m` is in METERS. Do not mix them.
- Each worker builds its part ALONE in the part's own local frame, with the part's
  natural attach/rotation point at the LOCAL origin (0,0,0). You decide, per part,
  where that origin is and state it precisely in `origin_note`.
- Every pose's `xyz_m` is the vector FROM the parent's origin TO the child's origin.
  Workers never position parts relative to each other — all spatial relationships
  live in the poses you author.
- Place parts so they physically MATE the way they function: meshing gears exactly
  one pitch-center-distance apart with teeth touching; a shaft through its bearing
  bore; a part resting ON the surface below it, not floating or sunk in. Under
  gravity, anything unsupported falls — give every part real support.
