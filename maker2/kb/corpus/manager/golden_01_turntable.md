# Golden connection graph: motorized turntable (bearing + shaft + platter)

Demonstrates the two things managers most often get right with a connection graph:
(1) physical hardware is a real part — a shaft turns INSIDE a bearing, so the
bearing_block is a fixed part AND the shaft is its own spin part, connected by a
`coaxial` mate; (2) you author NO coordinates — you declare which port connects to
which, and the solver computes the positions.

```json
{
  "name": "motorized_turntable",
  "root_part": "base",
  "parts": [
    { "name": "base", "shape_hint": "box", "size_mm": { "x": 200, "y": 200, "z": 15 },
      "origin_note": "top-face center at local origin; slab extends -Z",
      "color": [0.30, 0.30, 0.32], "material": "aluminum", "dof": "fixed" },
    { "name": "bearing_block", "shape_hint": "box", "size_mm": { "x": 50, "y": 50, "z": 50, "bore_dia": 12 },
      "origin_note": "bottom-face center at local origin; a 12mm bore runs vertically through the center",
      "color": [0.20, 0.22, 0.25], "material": "steel", "dof": "fixed" },
    { "name": "shaft", "shape_hint": "cylinder", "size_mm": { "radius": 6, "height": 90 },
      "origin_note": "bottom face center at local origin; cylinder extends +Z",
      "color": [0.75, 0.76, 0.78], "material": "steel", "dof": "spin", "spin_axis": [0,0,1], "driver": true },
    { "name": "platter", "shape_hint": "cylinder", "size_mm": { "radius": 80, "height": 8, "bore_dia": 12 },
      "origin_note": "bottom-face center at local origin; disc extends +Z; 12mm center bore",
      "color": [0.10, 0.10, 0.11], "material": "aluminum", "dof": "spin", "spin_axis": [0,0,1] }
  ],
  "mates": [
    { "name": "block_on_base", "mate_type": "face_to_face",
      "base_part": "base", "base_port": "face_pz",
      "incoming_part": "bearing_block", "incoming_port": "face_nz" },
    { "name": "shaft_in_block", "mate_type": "coaxial",
      "base_part": "bearing_block", "base_port": "bore",
      "incoming_part": "shaft", "incoming_port": "outer" },
    { "name": "platter_on_shaft", "mate_type": "coaxial",
      "base_part": "shaft", "base_port": "outer",
      "incoming_part": "platter", "incoming_port": "bore", "offset_mm": 90 }
  ],
  "mesh_pairs": []
}
```

The bearing block sits on the base (face_to_face), the shaft turns in its bore
(coaxial), and the platter rides on top of the shaft (coaxial, slid 90 mm up the
shaft via `offset_mm`). Every part connects back to the base root through the mates.
The shaft is the driver; both it and the platter are `spin` on +Z.
