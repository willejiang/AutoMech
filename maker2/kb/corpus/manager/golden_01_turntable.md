# Golden skeleton: motorized turntable (bearing + shaft + platter)

Demonstrates the two things managers most often get wrong: (1) physical hardware is
a real part — a shaft turns INSIDE a bearing, so the bearing_block is a fixed part
on the base AND the shaft is its own spin part on the bore axis, coexisting; (2) the
origin contract — each part's attach point is its local origin, and each pose xyz_m
(meters) is the parent-origin -> child-origin vector.

## PARTS
[
  { "name": "base", "shape_hint": "box",
    "size_mm": { "x": 120, "y": 120, "z": 10 },
    "origin_note": "center of the top face at origin; plate lies in the XY plane",
    "color": [0.75, 0.76, 0.78], "material": "aluminum", "driver": false },
  { "name": "bearing_block", "shape_hint": "cylinder",
    "size_mm": { "radius": 15, "height": 20 },
    "origin_note": "bore axis is +Z; origin at the block's base-face center",
    "color": [0.30, 0.30, 0.32], "material": "steel", "driver": false },
  { "name": "shaft", "shape_hint": "cylinder",
    "size_mm": { "radius": 5, "height": 40 },
    "origin_note": "cylinder axis is +Z; origin at the shaft's bottom-face center",
    "color": [0.70, 0.70, 0.72], "material": "steel", "driver": true },
  { "name": "platter", "shape_hint": "cylinder",
    "size_mm": { "radius": 50, "height": 4 },
    "origin_note": "disc axis is +Z; origin at the disc center on its mid-plane",
    "color": [0.12, 0.12, 0.13], "material": "plastic", "driver": false }
]
mesh_pairs: []

## MJCF (placement skeleton)
<mujoco>
  <worldbody>
    <!-- base: the fixed frame that rests on the ground; root_link. -->
    <body name="base" pos="0 0 0.005">
      <!-- bearing_block: fixed to the base, carries the shaft's bore. Its top is
           at z = base_top(0.010) + block_height(0.020) = 0.030. -->
      <body name="bearing_block" pos="0 0 0.010">
        <!-- shaft: spins inside the bore about +Z; the ONE driver. Sits with its
             bottom at the block base so it rises through the bore. -->
        <body name="shaft" pos="0 0 0.000">
          <joint type="hinge" axis="0 0 1"/>
          <!-- platter: welded to the top of the shaft, turns with it. shaft top is
               at z=0.040 above the shaft origin, so place the disc there. -->
          <body name="platter" pos="0 0 0.040">
            <site name="frame_top" pos="0 0 0.002" rpy="0 0 0"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>

Note the nesting encodes parentage: platter is a child of shaft (so it inherits the
spin), shaft is a child of bearing_block, bearing_block is a child of base. The
platter and shaft have NO joint of their own besides the shaft's hinge — the platter
is fixed relative to the shaft and rides its rotation.
