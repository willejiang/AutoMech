# Placing a part: rotation and resting height

A handful of mistakes account for most parts that end up in the wrong place. All of them
look correct in the code, build without error, and are only visible once the machine is
rendered or simulated.

## 1. `Axis.X` / `Axis.Y` / `Axis.Z` pass through the WORLD ORIGIN

`part.rotate(Axis.X, 90)` does not spin the part about its own axis. It rotates it about
the world X axis — the line through `(0, 0, 0)` — so a part that is not already centred on
that line is swung to a completely different place.

This is what breaks a propeller, a fan, a spoked wheel, a planet-carrier: anything built by
making one element and rotating copies of it around a hub.

```python
# WRONG — measured: hub at z=60, blade at x=386
blade0 = make_blade().moved(Location((386.0, 0.0, 60.0)))
blade1 = blade0.rotate(Axis.X,  90.0)
blade2 = blade0.rotate(Axis.X, 180.0)
blade3 = blade0.rotate(Axis.X, 270.0)
```

The four centres come out at:

| part | centre | |
|------|--------|--|
| blade0 | (386, 0, 60) | correct — it was never rotated |
| blade1 | (386, -60, 0) | swung down to the world axis height |
| blade2 | (386, 0, -60) | now BELOW the aircraft |
| blade3 | (386, 60, 0) | |

They orbit `(0,0,0)`, not the hub at `z = 60`, so exactly one blade lands where intended.

Two correct forms — use either:

```python
# A. Rotate the part while it is still AT THE ORIGIN, then move it into place.
blade = make_blade()                                   # still centred on the origin
hub   = Location((386.0, 0.0, 60.0))
blades = [blade.rotate(Axis.X, a).moved(hub) for a in (0.0, 90.0, 180.0, 270.0)]

# B. Rotate about an axis THROUGH THE HUB: Axis(origin_point, direction_vector).
hub_axis = Axis((386.0, 0.0, 60.0), (1, 0, 0))
blades = [blade0.rotate(hub_axis, a) for a in (0.0, 90.0, 180.0, 270.0)]
```

The same applies to `Location((x,y,z), (rx,ry,rz))`: the rotation is applied about the
part's own origin *before* the translation, which is what you want — but only if the part's
origin is where you think it is (§5) and you rotate about the right axis (§2).

## 2. A radial array: three things must agree

Blades on a propeller, spokes on a wheel, vanes on an impeller, planets on a carrier — all
are one element repeated around a shaft. Three choices have to point the same way, and
getting any one wrong produces a flat or edge-on array that still builds without error:

1. the PLANE the element is sketched on,
2. the AXIS it is rotated about,
3. the direction of the SHAFT it sits on.

`BuildSketch()` with no argument sketches on **Plane.XY** and `extrude` grows along **+Z**.
So a bare `BuildSketch()` element lies flat, with its length in X/Y and its thickness in Z.
Rotating that about Z keeps every copy in the same horizontal plane.

```python
# WRONG — measured on a P-51 whose propeller shaft runs along X
def prism(points, h):
    with BuildPart() as bp:
        with BuildSketch():          # <- Plane.XY: the blade lies FLAT
            Polygon(*points)
        extrude(amount=h)            # <- thickness along Z
    return bp.part

blade_template = prism(profile, 3.0)                 # x=19, y=77, z=3  (flat)
for angle in (0.0, 120.0, 240.0):
    blade = blade_template.moved(Location((PROP_X, 0.0, hub_z), (0.0, 0.0, angle)))
```

All three copies come back spanning `z = 60.0 .. 63.0` — 3mm of thickness, no spread at
all. The propeller is a pinwheel lying on top of the aircraft instead of a disc in front
of it.

For a shaft along X, sketch on `Plane.YZ` and rotate about X:

```python
with BuildPart() as blade_bp:
    with BuildSketch(Plane.YZ):      # blade stands UP, across the shaft
        Polygon(*profile)
    extrude(amount=thickness)        # thickness now runs along X, the shaft direction
blade_template = blade_bp.part

for angle in (0.0, 120.0, 240.0):
    blade = blade_template.moved(Location((PROP_X, 0.0, hub_z), (angle, 0.0, 0.0)))
    #                                                            ^^^^^ rotate about X
```

The rule, whatever the shaft direction: **the rotation angle goes on the SAME axis the
shaft runs along, and the element is sketched on the plane PERPENDICULAR to it.**

| shaft along | sketch on | rotation tuple |
|-------------|-----------|----------------|
| X (nose-forward: propeller, spinner) | `Plane.YZ` | `(angle, 0, 0)` |
| Y | `Plane.XZ` | `(0, angle, 0)` |
| Z (vertical: turntable, planet carrier) | `Plane.XY` (the default) | `(0, 0, angle)` |

A quick check before moving on: print the element's bounding box. Its LONG dimension must
be perpendicular to the shaft, and its THIN dimension must be along the shaft. If the thin
dimension is not on the shaft axis, the array will be flat.

## 3. Slicing along an axis: use `Plane.XZ.offset(d)`, do NOT build your own `Plane`

Lofting a wing, a blade or a tapered duct means sketching a section at several stations
along one axis. `Plane.XZ` sits at y=0 and there is no `Plane.XZ(y=-24)`, so it is tempting
to construct the plane by hand. That is where a whole aircraft ended up underground:

```python
# WRONG — every part built through this function came out MIRRORED in Z
def xz_plane(y):
    return Plane(origin=(0, y, 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
```

A `Plane`'s in-plane Y axis is DERIVED, not declared: it is `z_dir x x_dir`. With
`x_dir=(1,0,0)` and `z_dir=(0,1,0)` that gives `y_dir = (0, 0, -1)` — "up" inside the
sketch points at world DOWN. A section drawn at `zc = +70`, meaning 70mm above the ground,
lands at world `z = -70`:

    sketch zc = +70   ->   world z = -74.0 .. -70.0

The wings and tailplanes of a P-51 sat 77-85mm below the display base. Nothing else was
wrong: 128 of the 132 parts were fine, because only the four surfaces went through that
one function. Note also that the interpenetration check did NOT catch it — a wing buried
under the ground plane collides with nothing.

Use the built-in plane and shift it. Its orientation is already correct:

```python
Plane.XZ.offset(d).y_dir == (0, 0, 1)      # in-plane "up" IS world up
```

```python
# RIGHT — measured: a section drawn at zc=70 lands at world z=65.8..77.0
with BuildPart() as wing:
    with BuildSketch(Plane.XZ.offset(24.0)):    Polygon(*root_profile)   # y = -24
    with BuildSketch(Plane.XZ.offset(158.0)):   Polygon(*tip_profile)    # y = -158
    loft()
```

**`offset` moves along the plane's NORMAL, and `Plane.XZ`'s normal is -Y**, so a positive
offset goes to negative y:

| call | lands at |
|------|----------|
| `Plane.XZ.offset(+24)` | y = **-24** |
| `Plane.XZ.offset(-24)` | y = **+24** |

If that sign is confusing, state the origin explicitly instead — this form is also correct
and has no sign to get backwards:

```python
Plane(origin=(0, y, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))   # y_dir = (0,0,1)
```

Whenever you do construct a `Plane` yourself, print `plane.y_dir` and confirm it points
where you assume before drawing anything on it.

## 4. A part that rests on something: its height is DERIVED, never typed

A wheel touching the ground, a gear sitting on a bearing, a plate on posts — the height is
determined by what is underneath plus the part's own size. Writing the number by hand
means recomputing it in your head every time any dimension changes, and being wrong once.

```python
# WRONG — a real failure: the wheel sank 2mm into the ground plate
base   = Box(900, 1000, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN))  # top at z=8
wheel  = ring(25.0, bore_r, 12.0).moved(Location((30.0, 169.0, 23.0)))       # r=25
# lowest point = 23 - 25 = -2.0  ->  2mm below the ground it is meant to stand on

# RIGHT — say what it rests on
base_h        = 8.0
base_top      = base_h                 # Align.MIN: the box starts at z=0
wheel_r       = 25.0
wheel_centre_z = base_top + wheel_r    # touching, by construction
wheel = ring(wheel_r, bore_r, 12.0).moved(Location((30.0, 169.0, wheel_centre_z)))
```

Stack every vertical position the same way, from the ground up:

```python
base_top          = base_h
lower_bearing_z   = base_top
lower_bearing_top = lower_bearing_z + lower_bearing_h
stage1_z          = lower_bearing_top + axial_clearance
stage1_top        = stage1_z + gear_face
```

Then moving the baseplate, or making a bearing thicker, moves everything above it
correctly and by itself.

## 5. Where a part's origin actually is

A `Location` rotates about the part's own origin, and that origin is not always the centre:

- `Box(l, w, h)` / `Cylinder(r, h)` — centred on the origin by default.
- `align=(..., ..., Align.MIN)` — the part starts AT z=0 and grows upward, so its origin is
  its BOTTOM face. This is the useful form for anything sitting on a surface.
- `make_gear(module, teeth, face_width, bore)` — the origin is the -Z END FACE, not the
  centre. `.moved(Location((x, y, z)))` therefore puts the gear's back face at `z`, and it
  occupies `z` to `z + face_width`.
- A part built inside `BuildPart()` keeps whatever origin its sketches implied.

When two parts must be concentric or flush, derive one from the other's stated face
(`z = other_top`), rather than assuming both are centred.
