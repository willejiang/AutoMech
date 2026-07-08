# How parts move: dof, and the coaxial/structure rules

Motion replaces joints entirely. Each part declares a `dof`:

- `fixed` — welded in place relative to its parent (a frame, housing, bracket,
  plate, bridge, post, bearing, jewel, screw, pin, the base). MOST parts are fixed.
- `spin` — rotates freely about an implied axle along `spin_axis` (a gear on a
  shaft, a wheel on an axle, a rotor, a balance). Give it a unit `spin_axis`.
- `free` — a free-floating 6-DOF body (a loose ball, an unpinned pendulum bob).
  Use sparingly.

## Default to fixed

The vast majority of parts are structure and must be fixed. A part is `spin` ONLY
if it turns under power relative to its parent — essentially just wheels, pinions,
arbors/shafts, rotors, and a balance. If unsure, it is fixed.

A cage or carriage that rotates as a whole is ONE spin part. Its plates, bridges,
posts, and jewels are fixed and welded to it — NOT their own spin bodies. Marking
structure `spin` makes those parts fly apart under gravity.

## Coaxial parts that turn together = ONE spin part

A wheel + its pinion + the arbor they are pressed onto rotate as a unit: make the
ARBOR the single `spin` link and mark the wheel and pinion `fixed` (they are welded
to the arbor and placed on it). NEVER emit several `spin` parts at the same xy on
the same axis — as separate rigid bodies their solids interpenetrate and the sim
explodes.

## Parts must occupy their own space

No two parts share the same solid. A jewel/bearing PRESSES INTO a hole in a plate:
offset it in Z so it sits IN the bore (flush or proud), not buried at the plate's
origin. Put a wheel and the plate below it at DIFFERENT Z so they clear.

## The driver

Set `driver: true` on the SINGLE part the physics test spins to drive the machine
(the input gear/crank/rotor). The test applies torque to that part's own dof; every
downstream part moves ONLY if its teeth truly contact. At most one driver.

## Physical hardware is a real part

Every real piece of hardware is its own part. A rotating shaft/axle turns INSIDE a
bearing: emit BOTH — the bearing/housing as a `fixed` part AND the shaft as a
separate `spin` part on the bore axis. Do the same for gear-on-shaft,
wheel-on-axle, hinge pin. Never delete a shaft or bearing to simplify.
