# Golden skeleton: meshing gear pair (spur pinion drives a wheel)

Demonstrates gear layout math: a module-1 pinion (16 teeth) drives a module-1 wheel
(48 teeth), 3:1 speed reduction. Center distance C = module*(z1+z2)/2 =
1*(16+48)/2 = 32 mm = 0.032 m. Both gears spin about parallel +Z axes on their own
shafts; the pair is listed in mesh_pairs so the transmission checker knows they must
couple by teeth.

## PARTS
[
  { "name": "plate", "shape_hint": "box",
    "size_mm": { "x": 100, "y": 60, "z": 8 },
    "origin_note": "center of top face at origin; the two shafts rise from it",
    "color": [0.75, 0.76, 0.78], "material": "aluminum", "driver": false },
  { "name": "pinion_shaft", "shape_hint": "cylinder",
    "size_mm": { "radius": 3, "height": 20 },
    "origin_note": "axis +Z; origin at bottom-face center",
    "color": [0.70, 0.70, 0.72], "material": "steel", "driver": true },
  { "name": "pinion", "shape_hint": "gear",
    "size_mm": { "module": 1, "teeth": 16, "thickness": 6, "bore_dia": 6 },
    "origin_note": "gear center on its mid-plane at origin; teeth around +Z",
    "color": [0.80, 0.62, 0.20], "material": "brass", "driver": false },
  { "name": "wheel_shaft", "shape_hint": "cylinder",
    "size_mm": { "radius": 3, "height": 20 },
    "origin_note": "axis +Z; origin at bottom-face center",
    "color": [0.70, 0.70, 0.72], "material": "steel", "driver": false },
  { "name": "wheel", "shape_hint": "gear",
    "size_mm": { "module": 1, "teeth": 48, "thickness": 6, "bore_dia": 6 },
    "origin_note": "gear center on its mid-plane at origin; teeth around +Z",
    "color": [0.80, 0.62, 0.20], "material": "brass", "driver": false }
]
mesh_pairs: [ ["pinion", "wheel"] ]

## MJCF (placement skeleton)
<mujoco>
  <worldbody>
    <!-- plate: fixed base carrying both shafts; root_link. -->
    <body name="plate" pos="0 0 0.004">
      <!-- pinion_shaft: driver, spins about +Z at the origin corner. -->
      <body name="pinion_shaft" pos="-0.016 0 0.004">
        <joint type="hinge" axis="0 0 1"/>
        <!-- pinion: welded to the pinion shaft, mid-height on it. -->
        <body name="pinion" pos="0 0 0.010"/>
      </body>
      <!-- wheel_shaft: spins about +Z. Its center is C=0.032 m from the pinion
           shaft in +X so the gears mesh (pinion at x=-0.016, wheel at x=+0.016). -->
      <body name="wheel_shaft" pos="0.016 0 0.004">
        <joint type="hinge" axis="0 0 1"/>
        <!-- wheel: welded to the wheel shaft at the SAME height as the pinion so
             their teeth share a plane and actually contact. -->
        <body name="wheel" pos="0 0 0.010"/>
      </body>
    </body>
  </worldbody>
</mujoco>

The mesh works because |x_wheel - x_pinion| = 0.032 m = C, both gears sit at the
same z, and both are module 1. Change the teeth counts and you must recompute C and
move the wheel shaft accordingly.
