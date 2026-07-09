# Golden connection graph: deadbeat escapement (escape wheel + pallet fork)

An escapement converts a continuously-driven wheel into a stepped, oscillating motion.
The escape wheel is a spin part (driven); the pallet fork is a second spin part on a
PARALLEL axis, offset so its two pallets sit at the escape wheel's tooth tips. They
interact by CONTACT (a tooth pushes a pallet), which is NOT a gear mesh — so it is NOT
in mesh_pairs, and there is no gear mate between them.

The two arbors pivot at a FIXED spacing on the plate. To place two independent mount
points on one plate, this example DECLARES explicit `ports` on the plate (a feature you
use only when the inferred ports aren't enough) — two `flat_face` ports at the two pivot
locations. The pallet pivot is offset in +Y by the escape-wheel outside radius plus the
fork reach so the pallets fall on the tooth-tip circle.

```json
{
  "name": "deadbeat_escapement",
  "root_part": "plate",
  "parts": [
    { "name": "plate", "shape_hint": "box", "size_mm": { "x": 80, "y": 60, "z": 6 },
      "origin_note": "center of top face at origin; both arbors rise from it",
      "color": [0.75, 0.76, 0.78], "material": "brass", "dof": "fixed",
      "ports": [
        { "name": "escape_pivot", "type": "flat_face", "xyz_mm": [0, 0, 3], "axis": [0, 0, 1] },
        { "name": "pallet_pivot", "type": "flat_face", "xyz_mm": [0, 22, 3], "axis": [0, 0, 1] }
      ] },
    { "name": "escape_arbor", "shape_hint": "cylinder", "size_mm": { "radius": 2, "height": 18 },
      "origin_note": "axis +Z; origin at bottom-face center",
      "color": [0.70, 0.70, 0.72], "material": "steel", "dof": "spin", "spin_axis": [0,0,1], "driver": true },
    { "name": "escape_wheel", "shape_hint": "gear",
      "size_mm": { "module": 1, "teeth": 30, "thickness": 3, "bore_dia": 4 },
      "origin_note": "wheel center on mid-plane at origin; ratchet teeth around +Z",
      "color": [0.80, 0.62, 0.20], "material": "steel", "dof": "spin", "spin_axis": [0,0,1] },
    { "name": "pallet_arbor", "shape_hint": "cylinder", "size_mm": { "radius": 2, "height": 18 },
      "origin_note": "axis +Z; origin at bottom-face center",
      "color": [0.70, 0.70, 0.72], "material": "steel", "dof": "spin", "spin_axis": [0,0,1] },
    { "name": "pallet_fork", "shape_hint": "box", "size_mm": { "x": 30, "y": 8, "z": 3, "bore_dia": 4 },
      "origin_note": "the fork pivots about its arbor at origin; the two pallet faces are at the fork's ends, at the escape-wheel tooth tips",
      "color": [0.30, 0.30, 0.32], "material": "steel", "dof": "spin", "spin_axis": [0,0,1] }
  ],
  "mates": [
    { "name": "escape_arbor_on_plate", "mate_type": "face_to_face",
      "base_part": "plate", "base_port": "escape_pivot",
      "incoming_part": "escape_arbor", "incoming_port": "end_a" },
    { "name": "wheel_on_arbor", "mate_type": "coaxial",
      "base_part": "escape_arbor", "base_port": "outer",
      "incoming_part": "escape_wheel", "incoming_port": "bore", "offset_mm": 8 },
    { "name": "pallet_arbor_on_plate", "mate_type": "face_to_face",
      "base_part": "plate", "base_port": "pallet_pivot",
      "incoming_part": "pallet_arbor", "incoming_port": "end_a" },
    { "name": "fork_on_arbor", "mate_type": "coaxial",
      "base_part": "pallet_arbor", "base_port": "outer",
      "incoming_part": "pallet_fork", "incoming_port": "bore", "offset_mm": 8 }
  ],
  "mesh_pairs": []
}
```

The escapement is finicky: the `pallet_pivot` port's +Y offset (here 22 mm) must put the
pallet faces exactly on the escape-wheel tooth-tip circle so a tooth can push a pallet and
then release. Because you declare the pivot spacing as an explicit port position, a
reviewer can check the geometry directly. The tooth-pushes-pallet interaction is pure
contact — there is no gear mate and no mesh_pairs entry.
