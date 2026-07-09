# Manager output format: parts + mates (a connection graph)

The manager decomposes a product into PARTS and connects them with MATES ("part A's
port connects to part B's port"). A deterministic solver computes every part's
position from the mates — you author NO coordinates. This is a pure-contact model:
there are NO joints and NO motors. Whether a part can move is a property of the PART
itself (its `dof`), and motion is transmitted only by real physical contact under
gravity (meshing teeth, a cam lifting a follower, a falling weight).

## What the manager emits

A SINGLE JSON object with `parts`, `mates`, and (in subassembly mode) `frames`:
- `parts` — one entry per physical part, with its geometry parameters: `name`,
  `shape_hint`, `size_mm`, `origin_note`, `color`, `material`, `dof`, `spin_axis`,
  `driver`.
- `mates` — one entry per connection: which port of one part connects to which port
  of another, and the mate type. The solver places the parts from these.
- `root_part` — the ONE part pinned at the origin (usually the base/frame/housing).
- optional `mesh_pairs` (gear pairs; also auto-derived from gear mates) and `frames`
  (subassembly interface frames, each naming a part + port).

The solver (`mate_solver`) turns this into a KinematicModel, and the whole downstream
pipeline (gates, assembler, MJCF compiler, physics) consumes that. The MJCF simulation
is compiled separately by `build_mjcf` — you never write MJCF.

## Field contract (frozen — every consumer relies on these exact names)

A part:
- `name` — safe slug, lowercase, starts with a letter, `[a-z0-9_]` only, unique.
- `shape_hint` — `box | cylinder | gear | sphere | free text`.
- `size_mm` — approximate size in MILLIMETERS, keyed by the canonical dimension
  vocabulary (see the dimension-vocabulary doc).
- `origin_note` — where the part's LOCAL origin sits and which way it points.
- `color` — `[r, g, b]` in 0..1, matching the part's real material.
- `dof` — `fixed | spin | free`.
- `spin_axis` — unit vector; the rotation axis when `dof` is `spin`.
- `driver` — true on the ONE part the physics test drives. At most one.
- `material` — optional; `steel | brass | ruby | plastic | aluminum | titanium |
  rubber | wood | gold`. Sets mass (density x volume) and contact friction.

A mate:
- `mate_type` — the connection kind (see the connection-types doc).
- `base_part` + `base_port` — the part already placed and the port on it.
- `incoming_part` + `incoming_port` — the part this mate positions and the port on it.
- `offset_mm` — optional; slide the incoming part along the shared axis / face gap.
- `angle_rad` — optional; roll about the shared axis.
- `separation_axis` — gears: direction from base gear center to incoming gear center
  (REQUIRED when a gear meshes >1 other gear).
- `axis_angle_deg` — gears: 0 = parallel spur (default), 90 = bevel/worm.

## Ports are INFERRED — you never declare them

Each part automatically has named ports from its shape; reference them in mates:
- CYLINDER / SHAFT / GEAR: `outer`, `bore` (if it has a `bore_dia`), `end_a` (-Z face),
  `end_b` (+Z face); a GEAR also has `teeth`.
- BOX / PLATE: `face_px/nx/py/ny/pz/nz`, `center`, `bore` (if it has a `bore_dia`).
A part's primary axis is its local +Z.

Port naming is forgiving, so prefer the natural name and it will resolve:
- On a CYLINDER you may use `face_pz`/`face_nz` for the two flat ends (they alias
  `end_b`/`end_a`); `face_px/nx/py/ny` alias the round `outer` wall.
- On a BOX you may use `end_b`/`end_a` for the +Z/-Z faces (they alias `face_pz`/`face_nz`).
- A descriptive `shape_hint` still gets full ports: a part sized with `x`/`y`/`z` is
  treated as a BOX (keeps its six faces) and one sized with `radius`/`diameter` as a
  CYLINDER (keeps `outer`/`end_a`/`end_b`) — even if the hint is "bridge", "cock",
  "click", "pallet", "arbor", "jewel", etc. You do not lose ports by naming a part
  descriptively. (A part with NO usable size keys falls back to a single `center` port.)

## Units and origin contract

- `size_mm` is in MILLIMETERS. You do NOT write any meter coordinates — the solver
  computes positions from the mates.
- Each worker builds its part ALONE in the part's own local frame, with the part's
  natural attach/rotation point at the LOCAL origin (0,0,0). State it in `origin_note`.
- Connect parts so they physically MATE the way they function: meshing gears via a
  gear mate (the solver places them one pitch-center-distance apart); a shaft through
  a bearing bore via a coaxial mate; a part resting on the surface below via a
  face_to_face mate. Every part must connect (through mates) to the rest — an
  unconnected part is rejected.
- COMMON BASE, parallel not chained: when many parts sit on ONE base (a plate with
  several bosses / bearing seats / pillars, a bracket carrying several posts), mate
  EACH of them to the base INDEPENDENTLY — the base is the single shared anchor and the
  parts fan off it in parallel. Do NOT chain them to each other (boss → next boss →
  next boss); chaining fixes each part's position by TWO paths (its own base mate plus
  the neighbor's) and over-constrains it. "Connect to the rest" is satisfied by
  connecting to the BASE, not to sibling parts. Their relative spacing comes from WHERE
  each one mates on the base (its face position / `offset_mm`), not from mating one to
  the next. Example: three bearing bosses 40 mm apart on a plate = three separate
  face_to_face mates to the plate at x=0, 40, 80 — never boss_a→boss_b→boss_c.
