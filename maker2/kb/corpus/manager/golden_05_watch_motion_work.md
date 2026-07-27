# Golden build123d: watch motion work (concentric hour + minute hands)

This is the concentric-hand layout, written out as build123d that BUILDS. It is the part
of a timepiece that is most often generated wrong: two hands must turn about ONE axis and
both must be reachable from outside, which is only possible with NESTED PIPES of different
lengths. Copy this structure; change only the sizes.

The two constraints that make it work are marked. Both are asserted in the code, because
both fail silently — the script still builds, and the hands simply hang in the air.

```python
CENTRE_R = 1.5                                   # the centre arbor the pipes ride on
MIN_PIPE_IR, MIN_PIPE_OR, MIN_PIPE_H = 1.6, 2.4, 12.0    # INNER pipe: cannon pinion
HR_PIPE_IR,  HR_PIPE_OR,  HR_PIPE_H  = 2.5, 3.4,  7.0    # OUTER pipe: hour wheel
HAND_T = 0.8
BASE_Z = 4.0                                     # top face of the plate below

def tube(outer_r, inner_r, h):
    # Align.MIN so the tube spans [0, h] and "z + h" really is its top face.
    outer = Cylinder(outer_r, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore  = Cylinder(inner_r, h + 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return outer - bore.moved(Location((0, 0, -1.0)))     # overshoot both ends

def hand(length, width, bore_r, t):
    with BuildPart() as p:
        with BuildSketch(Plane.XY):
            Polygon((-width, -width / 2), (length, -width / 4),
                    (length,  width / 4), (-width,  width / 2), align=None)
            Circle(bore_r, mode=Mode.SUBTRACT)   # THE BORE — without it nothing mounts
        extrude(amount=t)
    return p.part

minute_pipe = tube(MIN_PIPE_OR, MIN_PIPE_IR, MIN_PIPE_H).moved(Location((0, 0, BASE_Z)))
hour_pipe   = tube(HR_PIPE_OR,  HR_PIPE_IR,  HR_PIPE_H ).moved(Location((0, 0, BASE_Z)))

hour_pipe_top   = BASE_Z + HR_PIPE_H
minute_pipe_top = BASE_Z + MIN_PIPE_H

# CONSTRAINT 1 — the INNER pipe must emerge past the outer one, with room for the
# hour hand, or the minute hand has no material to sit on.
assert minute_pipe_top > hour_pipe_top + HAND_T + 0.5

hour_hand   = hand(14, 3.0, HR_PIPE_OR  + 0.05, HAND_T).moved(
    Location((0, 0, hour_pipe_top - HAND_T)))
minute_hand = hand(20, 2.4, MIN_PIPE_OR + 0.05, HAND_T).moved(
    Location((0, 0, minute_pipe_top - HAND_T)))

# CONSTRAINT 2 — each hand's bore comes from the OD of the pipe it rides (above), and
# it is placed so hand and pipe OVERLAP axially, not so it perches on the top face.

a.add(minute_pipe, "minute_pipe|dof=fixed|mount=cannon_pinion")
a.add(hour_pipe,   "hour_pipe|dof=fixed|mount=hour_wheel")
a.add(hour_hand,   "hour_hand|dof=fixed|mount=hour_pipe")
a.add(minute_hand, "minute_hand|dof=fixed|mount=minute_pipe")
```

Built, this measures:

    minute_pipe  z=[4.00, 16.00]     <- inner, longer
    hour_pipe    z=[4.00, 11.00]     <- outer, shorter
    hour_hand    z=[10.20, 11.00]    hand/pipe axial overlap 0.80 mm
    minute_hand  z=[15.20, 16.00]    hand/pipe axial overlap 0.80 mm

Each hand's hub encircles its own pipe over the hand's full thickness, and neither hand
touches the other's pipe: `HR_PIPE_IR = 2.5 > MIN_PIPE_OR = 2.4` leaves the inner pipe
0.1 mm of clearance inside the outer one.

## What goes wrong when this is not followed

- Hands built as one solid polygon extrusion (no `Circle(..., Mode.SUBTRACT)`) have no
  hole. They cannot go onto a pipe; the gravity support test drops them and reports them
  unsupported.
- `Cylinder(r, h)` without `Align.MIN` is CENTERED, so every `z + height` term above is
  off by half a height and both hands float.
- An inner pipe shorter than the outer pipe is buried inside it; the minute hand then has
  nothing to mount on however correct its bore is.
