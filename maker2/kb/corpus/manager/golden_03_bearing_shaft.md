# Golden connection graph: bearing + shaft (the minimal rotating-hardware pattern)

The single most-copied pattern: a shaft that turns inside a bearing. Emit BOTH the
bearing (fixed) AND the shaft (spin) as separate parts and connect them with a
`coaxial` mate — the shaft sits inside the bearing bore. This is the atom every
gear-on-shaft / wheel-on-axle / hinge-pin is built from. The bearing needs a
`bore_dia` so it has a `bore` port for the shaft.

```json
{
  "name": "bearing_shaft",
  "root_part": "housing",
  "parts": [
    { "name": "housing", "shape_hint": "box", "size_mm": { "x": 40, "y": 40, "z": 8 },
      "origin_note": "center of top face at origin",
      "color": [0.75, 0.76, 0.78], "material": "aluminum", "dof": "fixed" },
    { "name": "bearing", "shape_hint": "cylinder", "size_mm": { "radius": 8, "height": 6, "bore_dia": 6 },
      "origin_note": "bore axis +Z; origin at the bearing's base-face center; a 6mm bore through it",
      "color": [0.30, 0.30, 0.32], "material": "steel", "dof": "fixed" },
    { "name": "shaft", "shape_hint": "cylinder", "size_mm": { "radius": 3, "height": 30 },
      "origin_note": "axis +Z; origin at the shaft's bottom-face center",
      "color": [0.70, 0.70, 0.72], "material": "steel", "dof": "spin", "spin_axis": [0,0,1], "driver": true }
  ],
  "mates": [
    { "name": "bearing_on_housing", "mate_type": "face_to_face",
      "base_part": "housing", "base_port": "face_pz",
      "incoming_part": "bearing", "incoming_port": "end_a" },
    { "name": "shaft_in_bearing", "mate_type": "coaxial",
      "base_part": "bearing", "base_port": "bore",
      "incoming_part": "shaft", "incoming_port": "outer" }
  ],
  "mesh_pairs": []
}
```

Key point: the bearing and the shaft are DIFFERENT parts on the same axis, connected by
a `coaxial` mate (base = the bearing's `bore`, incoming = the shaft's `outer`). The
shaft's solid (r=3) fits inside the bore (r=3) so they do not share metal. The bearing
mounts on the housing top (face_to_face). Never collapse the two into one part, and
never mark the bearing `spin`.
