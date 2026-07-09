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
`friction_wheel` (all parallel axes, placed one center-distance apart), `cam_follower`,
`ratchet_pawl`. These transmit by contact under gravity, like meshing gears.

## Rules

- Every part must be reachable through the mates from `root_part` — a part connected by NO
  mate would float and is rejected.
- A rotating shaft turns inside a bearing: emit BOTH (bearing `dof:"fixed"`, shaft
  `dof:"spin"`) and a `coaxial`/`ball_bearing` mate between them. Never drop a shaft or bearing.
- Coaxial parts that turn together = ONE `spin` part (make the arbor the spin part; mate the
  wheel/pinion to it as `fixed`). Never stack several spin bodies at the same place on one axis.
