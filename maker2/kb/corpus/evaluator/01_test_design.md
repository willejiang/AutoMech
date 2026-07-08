# Evaluator: physics-test design + a failure taxonomy keyed to gate error codes

The evaluator judges a generated machine and, on a passing run, records a one-line
memory_note ("what made this pass/fail") that seeds the growing memory. Its job is
to design a test that exercises the intended mechanism and to read failures back to
concrete, actionable causes.

## Physics-test design (pure-contact MuJoCo)

- The test drives the ONE driver part's dof (torque on its hinge) and watches
  whether downstream parts move via tooth contact. Transmission is the primary
  signal: does driving the input actually turn the output?
- Stability: nothing should fly apart or sink through the ground. Parts that jitter
  or explode usually indicate interpenetration or a mis-marked spin part.
- Settle first, then drive: let the assembly rest on the ground plane under gravity,
  then apply input, so settle noise is not read as transmission.

## Failure taxonomy (map a symptom to a cause and a fix)

- No output motion though the input turns -> the meshing pair is not actually
  touching. Cause: center distance != module*(z1+z2)/2, or the two gears sit at
  different z so their teeth miss. Fix: recompute C, put both gears on one plane.
- Parts fly apart on the first step -> two solids interpenetrate, or structure was
  marked `spin`. Fix: separate the solids in Z, collapse coaxial spin stacks to one
  spin part with the rest fixed.
- A part falls through the floor / floats -> it has no support, or its pose put it
  below the ground. Fix: rest it on the surface below; check the pose xyz_m sign.
- Assembly won't compile / load -> a frame is unrealized (no site), a mesh file is
  missing, or a pose references a non-existent link. Fix: realize every named frame
  with a site, ensure every pose child/parent is a real link.
- Frame drift -> a site's world position does not match the boss contract
  coordinate. Fix: anchor geometry to the fixed hard-point instead of re-deriving.

## The memory_note

On a passing run, write ONE line stating what made it work: the key dimension, the
placement trick, or the decomposition choice that let it assemble and transmit. This
line is what future runs retrieve, so make it specific and reusable ("module-1
pinion+wheel at C=0.032 on one plane meshed cleanly"), not generic ("it worked").
