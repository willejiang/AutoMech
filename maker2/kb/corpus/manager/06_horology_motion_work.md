# Clock and watch geometry: motion work, hands, and the going train

A timepiece is the one mechanism where several wheels share ONE axis and still have to be
reachable from outside. Getting that nesting wrong is the single most common way a
generated watch comes out with its hands hanging in mid-air. The rules below are about
GEOMETRY, not styling — each one has a coordinate consequence.

## Motion work: the inner pipe must protrude past the outer pipe

The minute and hour hands are concentric: they turn about the same axis, at a 12:1 ratio.
That is achieved by nesting TUBES, not by stacking discs.

- The cannon pinion carries a pipe (the INNER tube) that rides on the centre arbor.
- The hour wheel carries a wider pipe (the OUTER tube) that rides ON the inner tube.
- The hour hand sits on the OUTER pipe; the minute hand sits on the INNER pipe.

The controlling constraint is axial, and it is easy to get backwards: the INNER pipe must
be LONGER than the outer one, and its top face must end ABOVE the outer pipe's top face —
by at least the hour hand's thickness plus a clearance. Otherwise the inner pipe is buried
inside the outer pipe and the minute hand has no material to land on.

    hour_pipe_top   = hour_pipe_z   + hour_pipe_h
    hour_hand_z     = hour_pipe_top - hour_hand_h        # hour hand rides the OUTER pipe
    minute_pipe_top = minute_pipe_z + minute_pipe_h
    assert minute_pipe_top > hour_pipe_top + hour_hand_h + 0.5   # inner pipe emerges
    minute_hand_z   = minute_pipe_top - minute_hand_h

Write that assertion into the script. A layout where `minute_pipe_top <= hour_pipe_top` is
wrong no matter how the parts are labelled.

## A hand is a part with a HOLE, not a flat blade

A hand mounts by SLIDING ONTO its pipe. Its solid must therefore be an annulus at the hub:
subtract a bore whose radius comes from the OD of the pipe it rides, and give the hub
enough thickness that hand and pipe genuinely OVERLAP along the axis.

    minute_hand_bore_r = minute_pipe_outer_r + 0.05
    hour_hand_bore_r   = hour_pipe_outer_r   + 0.05

A hand built as one solid polygon extrusion has no hole. It cannot be on the pipe at all;
the best it can do is rest on the pipe's top face, and under gravity it falls. If a mesh
check reports a hand with `euler_number = 2` (genus 0, no through-hole) while its pipe
reports `euler_number = 0` (genus 1), the hand is solid and the mount is unrealizable.

## Wheel, pinion and arbor on one axis rotate as one part

A going train is a chain of WHEEL-drives-PINION stages: a large wheel meshes a small
pinion on the next arbor, and that arbor carries the next wheel. The wheel, the pinion and
the arbor they are pressed onto turn together — so the ARBOR is the single `spin` part and
the wheel and pinion mounted on it are `fixed`, placed at DISTINCT axial stations along it.
Never emit two `spin` bodies at the same xy on the same axis.

Each mesh still obeys ordinary gear arithmetic: one module shared by the meshing pair, and
centre distance = module*(z_wheel + z_pinion)/2. The reduction of a stage is
z_wheel / z_pinion, and the train's total ratio is the product of its stages. Standard
motion work is 12:1 overall, conventionally split as two stages (e.g. 40/10 and 36/12).

## Plates, bridges and pivots are structure

The mainplate and the bridges above it are `fixed` and are what the whole movement hangs
off; every arbor is held between a hole in the mainplate and a hole in a bridge. A bridge
that is not itself carried by posts standing on the mainplate has nothing holding it up —
give each bridge real pillars, mounted on the plate, and set the bridge's z from the
pillars' REAL top face.

## Do not

- Do not put the minute hand on the hour pipe or vice versa — the outer pipe is the hour
  side, always.
- Do not mark a hand, a pipe collar, or a bridge `spin`. A hand is welded to its pipe.
- Do not size a hand's bore from a round number; take it from the pipe's outer radius.
- Do not model motion work as two flat discs stacked on the same arbor — without nested
  pipes of different lengths the two hands cannot both be reached from outside.
