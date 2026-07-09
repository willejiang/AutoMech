# Golden connection graph: meshing gear pair (spur pinion drives a wheel)

Demonstrates a gear mesh: a module-1 pinion (16 teeth) drives a module-1 wheel (48
teeth), 3:1 speed reduction. You do NOT compute the center distance — you declare a
`gear_spur_external` mate and the solver places the wheel exactly one center distance
`C = module*(z1+z2)/2 = 32 mm` from the pinion along `separation_axis`. Both gears
spin about parallel +Z axes on their own shafts; the mesh mate also registers the pair
in mesh_pairs so the transmission checker knows they couple by teeth.

```json
{
  "name": "gear_pair",
  "root_part": "plate",
  "parts": [
    { "name": "plate", "shape_hint": "box", "size_mm": { "x": 100, "y": 60, "z": 8 },
      "origin_note": "center of top face at origin; the two shafts rise from it",
      "color": [0.75, 0.76, 0.78], "material": "aluminum", "dof": "fixed" },
    { "name": "pinion_shaft", "shape_hint": "cylinder", "size_mm": { "radius": 3, "height": 20 },
      "origin_note": "axis +Z; origin at bottom-face center",
      "color": [0.70, 0.70, 0.72], "material": "steel", "dof": "spin", "spin_axis": [0,0,1], "driver": true },
    { "name": "pinion", "shape_hint": "gear",
      "size_mm": { "module": 1, "teeth": 16, "thickness": 6, "bore_dia": 6 },
      "origin_note": "gear center on its mid-plane at origin; teeth around +Z",
      "color": [0.80, 0.62, 0.20], "material": "brass", "dof": "spin", "spin_axis": [0,0,1] },
    { "name": "wheel_shaft", "shape_hint": "cylinder", "size_mm": { "radius": 3, "height": 20 },
      "origin_note": "axis +Z; origin at bottom-face center",
      "color": [0.70, 0.70, 0.72], "material": "steel", "dof": "spin", "spin_axis": [0,0,1] },
    { "name": "wheel", "shape_hint": "gear",
      "size_mm": { "module": 1, "teeth": 48, "thickness": 6, "bore_dia": 6 },
      "origin_note": "gear center on its mid-plane at origin; teeth around +Z",
      "color": [0.80, 0.62, 0.20], "material": "brass", "dof": "spin", "spin_axis": [0,0,1] }
  ],
  "mates": [
    { "name": "pinion_shaft_on_plate", "mate_type": "face_to_face",
      "base_part": "plate", "base_port": "face_pz",
      "incoming_part": "pinion_shaft", "incoming_port": "end_a" },
    { "name": "pinion_on_shaft", "mate_type": "coaxial",
      "base_part": "pinion_shaft", "base_port": "outer",
      "incoming_part": "pinion", "incoming_port": "bore", "offset_mm": 10 },
    { "name": "gears_mesh", "mate_type": "gear_spur_external",
      "base_part": "pinion", "base_port": "teeth",
      "incoming_part": "wheel", "incoming_port": "teeth", "separation_axis": "+x" },
    { "name": "wheel_on_shaft", "mate_type": "coaxial",
      "base_part": "wheel", "base_port": "bore",
      "incoming_part": "wheel_shaft", "incoming_port": "outer", "offset_mm": -10 }
  ],
  "mesh_pairs": [ ["pinion", "wheel"] ]
}
```

Why this works: the pinion shaft mounts on the plate top (face_to_face), the pinion rides
on it (coaxial), the `gear_spur_external` mate places the wheel's center exactly `C` from
the pinion's center on +X (the solver reads both pitch radii from module×teeth) so the
teeth touch — you never write a coordinate. The wheel's shaft then hangs off the wheel
(coaxial), so the whole output side is positioned BY the mesh, not placed independently
(placing it twice would over-constrain it). EVERY part connects back to the plate root
through the mates — an unconnected part would float and is rejected. Change the teeth
counts and the solver recomputes C automatically.
