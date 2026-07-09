# Canonical dimension vocabulary + gear layout math

## Canonical size_mm keys per shape_hint (MANDATORY — the gates assume these)

Use exactly these keys. Do NOT invent synonyms (no `gear_dia` vs `pitch_dia` vs
`wheel_dia`; no `len` vs `length` vs `l`). The dimension-lock gate and the mating
checks key off these names, so a synonym reads as a missing dimension.

- `cylinder` -> `{ "radius": <mm>, "height": <mm> }`
- `box`      -> `{ "x": <mm>, "y": <mm>, "z": <mm> }`
- `sphere`   -> `{ "radius": <mm> }`
- `gear`     -> `{ "module": <mm>, "teeth": <int>, "thickness": <mm>,
                   "bore_dia": <mm> }`

A gear is specified by module + teeth (never by a raw diameter): its pitch diameter
is derived, and a meshing pair must share the same module.

## Gear layout math (get spacing right so teeth actually touch)

- Pitch diameter: `d = module * teeth`. Pitch radius: `r = d / 2`.
- Center distance of a meshing pair: `C = r_drive + r_driven =
  module * (teeth_drive + teeth_driven) / 2`. Place the two gear centers exactly
  this far apart (in the plane normal to their shared axis). Too close -> teeth
  interpenetrate; too far -> they never contact and nothing transmits.
- Both gears of a pair must share the SAME module to mesh.
- Gear ratio (speed) = teeth_driven / teeth_drive. A small pinion driving a large
  wheel steps speed DOWN and torque UP.
- Involute standard: addendum = module, dedendum = 1.25 * module, so tooth height ~
  2.25 * module; outside diameter = d + 2 * module. Adjacent gears on parallel
  shafts must clear at their outside diameters if they are NOT the meshing pair.

## Putting it together

For a two-gear train that must mesh: pick a module, pick teeth counts for the ratio
you want, give BOTH gears that `module` + `teeth`, and connect them with a
`gear_spur_external` mate (with a `separation_axis`). The solver places the driven
gear exactly one center distance C from the drive gear automatically — you do NOT
compute or write C yourself. List the pair in `mesh_pairs` (a gear mate also adds it
automatically).
