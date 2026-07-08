# Golden skeleton: bearing + shaft (the minimal rotating-hardware pattern)

The single most-copied pattern: a shaft that turns inside a bearing. Emit BOTH the
bearing (fixed) AND the shaft (spin) as separate parts that coexist — the shaft
sits inside the bearing bore, offset in Z so it is not buried at the bearing origin.
This is the atom every gear-on-shaft / wheel-on-axle / hinge-pin is built from.

## PARTS
[
  { "name": "housing", "shape_hint": "box",
    "size_mm": { "x": 40, "y": 40, "z": 8 },
    "origin_note": "center of top face at origin",
    "color": [0.75, 0.76, 0.78], "material": "aluminum", "driver": false },
  { "name": "bearing", "shape_hint": "cylinder",
    "size_mm": { "radius": 8, "height": 6 },
    "origin_note": "bore axis +Z; origin at the bearing's base-face center; the bore is a hole of radius 3 through it",
    "color": [0.30, 0.30, 0.32], "material": "steel", "driver": false },
  { "name": "shaft", "shape_hint": "cylinder",
    "size_mm": { "radius": 3, "height": 30 },
    "origin_note": "axis +Z; origin at the shaft's bottom-face center",
    "color": [0.70, 0.70, 0.72], "material": "steel", "driver": true }
]
mesh_pairs: []

## MJCF (placement skeleton)
<mujoco>
  <worldbody>
    <!-- housing: fixed base; root_link. -->
    <body name="housing" pos="0 0 0.004">
      <!-- bearing: fixed to the housing top, provides the bore the shaft turns in. -->
      <body name="bearing" pos="0 0 0.004">
        <!-- shaft: spins about +Z inside the bore. Its radius (3) is less than the
             bore radius so it clears; it rises through and above the bearing. -->
        <body name="shaft" pos="0 0 0.000">
          <joint type="hinge" axis="0 0 1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>

Key point: the bearing and the shaft are DIFFERENT parts at (roughly) the same xy,
but the shaft's solid (r=3) fits inside the bearing bore (r=3 hole) — they do not
share a solid. Never collapse the two into one part, and never mark the bearing
`spin`.
