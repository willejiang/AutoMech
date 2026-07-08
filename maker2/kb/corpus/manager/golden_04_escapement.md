# Golden skeleton: deadbeat escapement (escape wheel + pallet fork)

An escapement converts a continuously-driven wheel into a stepped, oscillating
motion. The escape wheel is a spin part (driven); the pallet fork is a second spin
part on a parallel axis whose two pallets alternately catch the wheel teeth. They
interact by CONTACT (a tooth pushes a pallet), so the pallet faces must sit exactly
at the wheel's tooth tips — this is a contact interface, not a mesh, so it is NOT in
mesh_pairs.

## PARTS
[
  { "name": "plate", "shape_hint": "box",
    "size_mm": { "x": 80, "y": 60, "z": 6 },
    "origin_note": "center of top face at origin; both arbors rise from it",
    "color": [0.75, 0.76, 0.78], "material": "brass", "driver": false },
  { "name": "escape_arbor", "shape_hint": "cylinder",
    "size_mm": { "radius": 2, "height": 18 },
    "origin_note": "axis +Z; origin at bottom-face center",
    "color": [0.70, 0.70, 0.72], "material": "steel", "driver": true },
  { "name": "escape_wheel", "shape_hint": "gear",
    "size_mm": { "module": 1, "teeth": 30, "thickness": 3, "bore_dia": 4 },
    "origin_note": "wheel center on mid-plane at origin; ratchet teeth around +Z",
    "color": [0.80, 0.62, 0.20], "material": "steel", "driver": false },
  { "name": "pallet_arbor", "shape_hint": "cylinder",
    "size_mm": { "radius": 2, "height": 18 },
    "origin_note": "axis +Z; origin at bottom-face center",
    "color": [0.70, 0.70, 0.72], "material": "steel", "driver": false },
  { "name": "pallet_fork", "shape_hint": "free text",
    "size_mm": { "x": 30, "y": 8, "z": 3 },
    "origin_note": "the fork pivots about its arbor at origin; the two pallet faces are at the fork's ends, positioned to touch the escape-wheel tooth tips",
    "color": [0.30, 0.30, 0.32], "material": "steel", "driver": false }
]
mesh_pairs: []

## MJCF (placement skeleton)
<mujoco>
  <worldbody>
    <!-- plate: fixed base; root_link. -->
    <body name="plate" pos="0 0 0.003">
      <!-- escape_arbor: driven wheel's arbor, spins about +Z. -->
      <body name="escape_arbor" pos="0 0 0.003">
        <joint type="hinge" axis="0 0 1"/>
        <!-- escape_wheel: welded to its arbor; a module-1 30-tooth wheel, so its
             pitch radius is 0.015 m and tooth tips reach ~0.016 m. -->
        <body name="escape_wheel" pos="0 0 0.008"/>
      </body>
      <!-- pallet_arbor: the fork's pivot, spins about +Z. Placed so the fork's
           pallets fall at the escape wheel's tooth tips: offset in +Y by about the
           wheel outside radius plus the fork reach. -->
      <body name="pallet_arbor" pos="0 0.022 0.003">
        <joint type="hinge" axis="0 0 1"/>
        <!-- pallet_fork: welded to the pallet arbor at the wheel's tooth-plane so
             its pallet faces contact the tooth tips. -->
        <body name="pallet_fork" pos="0 0 0.008"/>
      </body>
    </body>
  </worldbody>
</mujoco>

The escapement is finicky: the pallet-arbor offset must put the pallet faces exactly
on the escape-wheel tooth-tip circle so a tooth can push a pallet and then release.
State the geometry reasoning in the pallet_arbor comment so a reviewer can check it.
