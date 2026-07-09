# Connection types — the mate catalog (how two parts join)

You describe a machine as PARTS joined by MATES. Each mate says "part A's PORT connects to
part B's PORT" and a deterministic solver computes the positions. This is the menu of
mate types, grounded in standard machine design. Pick the REAL mechanical connection.

Every part automatically has INFERRED PORTS from its shape — you never declare ports, just
reference them by name:
- a CYLINDER / SHAFT / GEAR: `outer` (outer surface, +Z axis), `bore` (central hole, if
  size_mm has `bore_dia`), `end_a` (-Z face), `end_b` (+Z face); a GEAR also has `teeth`.
- a BOX / PLATE: `face_px/nx/py/ny/pz/nz` (six faces by outward normal), `center`, and
  `bore` if it has a `bore_dia`.
A part's primary axis is its local +Z, so `outer`/`bore`/`teeth` all run along +Z.

## Shaft / bore / hub family (all COAXIAL — the two axes align and center)

Use `coaxial` (or a more specific alias) for anything that sits ON or INSIDE a round axis.
`offset_mm` slides the incoming part ALONG the shared axis; `angle_rad` rolls it.

| mate_type | use for | leaves free |
|---|---|---|
| `coaxial` | general shaft-in-bore, part-on-axis | (set by the part's `dof`) |
| `revolute` | a pin/hinge — 1 rotational DOF | spin about the axis |
| `press_fit` / `shrink_fit` | interference fit (gear pressed on a shaft) | nothing (one rigid unit) |
| `pin` / `dowel` / `key` | torque + axial lock through a round feature | nothing |
| `ball_bearing` / `journal_bearing` | shaft in a bearing (inner race on shaft) | spin |

SHAFT THROUGH A GEAR'S HOLE = `coaxial`, base = the gear's `bore`, incoming = the shaft's
`outer`. The shaft freely spans past the gear. Example:
`{"mate_type":"coaxial","base_part":"gear","base_port":"bore","incoming_part":"shaft","incoming_port":"outer"}`

## Flat-face / structural family (seat one face on another; all FIXED, 0 DOF)

Use `face_to_face` (or an alias) to rest a flat face on a flat face — a bearing on a plate,
a cap on a housing, a cover on a box. The faces meet front-to-front; `offset_mm` is a gap.
A BOX face is `face_pz`/`face_nz`/etc.; a CYLINDER face is `end_a`/`end_b`.

| mate_type | use for |
|---|---|
| `face_to_face` | seat any flat face on another |
| `bolted` / `flanged` | two faces + a bolt pattern / bolt circle |
| `welded` | permanent face-to-face join |
| `snap_fit` | elastic snap onto a ledge |

Locating rule (3-2-1): a face removes 3 DOF; a bolt pattern or two pins removes the rest.

## Gear family (teeth mesh; the mesh couples the two rotations)

Both gears of a mesh share the SAME `module`; pitch radius = module × teeth / 2. Give every
gear a `module` + `teeth` (or a pitch diameter) so the solver knows the pitch radius.

| mate_type | axes | placement |
|---|---|---|
| `gear_spur_external` | PARALLEL | centers one center-distance `C = r_a + r_b` apart |
| `gear_spur_internal` | PARALLEL | pinion inside a ring gear; `C = r_a − r_b` |
| `gear_bevel` | PERPENDICULAR (90°) | right-angle drive; set `axis_angle_deg: 90` |
| `worm` | PERPENDICULAR skew | high-ratio right-angle; `axis_angle_deg: 90` |
| `rack_pinion` | rotation ↔ translation | a pinion driving a linear rack |

- **Two meshing spur gears** — `gear_spur_external`, base = one gear's `teeth`, incoming =
  the other's `teeth`, plus `separation_axis` (the direction from the base gear center to the
  incoming one, e.g. `"+x"`).
- **A gear TRAIN** (one gear meshing MORE THAN ONE other) — each mesh MUST give its own
  `separation_axis`, so each neighbor sits in the right direction. Omitting it is an error.
- **One gear horizontal + one vertical** (right-angle drive) — `gear_bevel` with
  `axis_angle_deg: 90`; the two axes end up perpendicular.

Gear layout math: pitch diameter `d = module * teeth`; center distance of a meshing pair
`C = module * (teeth_a + teeth_b) / 2`. Gear ratio (speed) = teeth_driven / teeth_drive.

## Contact / transmission family (no rigid joint — motion by contact)

For parallel-axis power transfer that is NOT a rigid joint: `belt_pulley`, `chain_sprocket`,
`friction_wheel` (all parallel axes, placed one center-distance apart). These transmit by
contact under gravity, like meshing gears, and need BOTH parts' radii.

A PAWL / CLICK / DETENT / CAM-FOLLOWER touches a wheel's RIM but is NOT a gear — it is a
lever with no pitch radius. Do NOT use a gear mate (that demands a pitch radius on the pawl
and fails). Use a CONTACT mate: `ratchet`, `pawl`, `click`, `detent`, `cam_follower`. Only
the WHEEL needs a radius (its `outer`, or `teeth`/module if toothed); the pawl is seated
tangent on the wheel rim along `separation_axis`. Example — a click holding a ratchet:
`{"mate_type":"ratchet","base_part":"ratchet_wheel","base_port":"teeth","incoming_part":"click","incoming_port":"end_a","separation_axis":"+x"}`
The `click` here is just a small box/lever (no module, no teeth) — it needs no radius.

## A shaft spanning TWO bearings (the #1 over-constraint trap)

The mates must form a TREE — every part positioned by exactly ONE path. A shaft that runs
through two bearings is where this goes wrong. Do NOT mate the shaft coaxially to BOTH
bearings: each coaxial mate pins the shaft AT that bore, so two bores at different places
demand the shaft be in two spots → "over-constrained, placed N mm apart".

CORRECT pattern — make the SHAFT the base and hang everything off it as a tree:
1. Mate ONE bearing to the plate (face_to_face) to locate the assembly. Call it the fixed
   bearing.
2. Put the shaft ON that bearing coaxially (`base = bearing.bore`, `incoming = shaft.outer`),
   sliding it with `offset_mm` so it protrudes the right way.
3. Hang the SECOND bearing on the SHAFT (`base = shaft.outer`, `incoming = bearing_b.bore`)
   with a DIFFERENT `offset_mm` to slide it to the far end. The second bearing rides the
   shaft; it is NOT independently mated to the plate.

```json
{ "mate_type":"face_to_face","base_part":"plate","base_port":"face_pz",
  "incoming_part":"bearing_a","incoming_port":"end_a","offset_mm":0 },
{ "mate_type":"coaxial","base_part":"bearing_a","base_port":"bore",
  "incoming_part":"shaft","incoming_port":"outer","offset_mm":-4 },
{ "mate_type":"coaxial","base_part":"shaft","base_port":"outer",
  "incoming_part":"bearing_b","incoming_port":"bore","offset_mm":46 }
```
Now shaft, bearing_a, bearing_b form a chain (plate → bearing_a → shaft → bearing_b), one
path each, no loop. The same rule covers a gear on a shaft in two bearings, a rotor between
two supports, etc.: pick ONE support as the anchor, then chain the rest along the shaft.

## Rules

- Every part must be reachable through the mates from `root_part` — a part connected by NO
  mate would float and is rejected.
- The mates form a TREE: each part is placed by exactly ONE mate path. Mating a part to two
  things that fix its position (a shaft into two bearings; a bearing onto both a shaft and a
  plate) creates a LOOP and over-constrains it. Anchor one, chain the rest (see the two-
  bearing pattern above).
- A rotating shaft turns inside a bearing: emit BOTH (bearing `dof:"fixed"`, shaft
  `dof:"spin"`) and a `coaxial`/`ball_bearing` mate between them. Never drop a shaft or bearing.
- Coaxial parts that turn together = ONE `spin` part (make the arbor the spin part; mate the
  wheel/pinion to it as `fixed`). Never stack several spin bodies at the same place on one axis.
