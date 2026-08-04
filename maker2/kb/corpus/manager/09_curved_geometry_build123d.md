# Curved and organic geometry: what build123d can actually do

Boxes, cylinders and the injected `make_gear` cover a gear train. They cannot make a wing,
a fuselage, a duct, an impeller blade or a fairing. build123d can — the whole library is
importable inside `build_machine()`, not just the handful of names pre-injected for you.

Everything below was executed against build123d 0.11.1 in this repo; the volumes are what
the code actually produced.

## Import what you need

The pre-injected names (`Box`, `Cylinder`, `Circle`, `Polygon`, `extrude`, `BuildPart`,
`BuildSketch`, `Location`, `Plane`, `Align`, `Mode`, `AssemblyHelper`, `make_gear`) are a
convenience, not a limit. For anything else, import it:

```python
from build123d import (Sphere, Torus, Cone, Wedge, RegularPolygon, Ellipse, Text,
                       Spline, Bezier, Polyline, Line, CenterArc, ThreePointArc, Helix,
                       revolve, loft, sweep, offset, fillet, chamfer, mirror, scale, split,
                       BuildLine, make_face, make_hull, add, Axis, Rotation, Pos, Kind)
```

All of these are verified present. `b3d` is also injected, so `b3d.Sphere(10)` works
without any import.

## The four operations that make curved bodies

`extrude` gives you prisms. These give you shapes:

| operation | what it does | typical use |
|-----------|--------------|-------------|
| `loft()` | blends between 2+ sketches on different planes | wing, blade, tapered fairing |
| `revolve(axis=…)` | spins a profile about an axis | fuselage, nose cone, nozzle, pulley |
| `sweep(sections=…, path=…)` | drags a profile along a curve | duct, pipe, curved spar |
| `fillet(edges, radius=)` / `chamfer` | rounds/bevels edges | every real trailing edge and corner |

## Wing: sketch each section, then loft

Sections go on *different* planes; `add(...)` then `loft()` skins them. This is how you
get taper and twist in one operation.

```python
def naca(chord, t=0.12, n=28):
    """NACA 00xx half-thickness, upper surface then lower, ready for a closed Polyline."""
    f = lambda x: 5*t*chord*(0.2969*x**0.5 - 0.1260*x - 0.3516*x*x
                             + 0.2843*x**3 - 0.1015*x**4)
    pts  = [(i/n*chord,  f(i/n)) for i in range(n + 1)]
    pts += [(i/n*chord, -f(i/n)) for i in range(n - 1, 0, -1)]
    return pts

sections = []
for span_z, chord in ((0, 120.0), (200, 90.0), (400, 60.0)):   # root -> tip taper
    with BuildSketch(Plane.XY.offset(span_z)) as s:
        with BuildLine():
            Polyline(*naca(chord), close=True)
        make_face()
    sections.append(s.sketch)

with BuildPart() as wing:
    add(sections)
    loft()
# -> 274348.5 mm^3, valid solid
```

Add dihedral or sweep by offsetting each section's plane in Y/X as well as Z; add twist by
rotating the plane (`Plane.XY.offset(z) * Rotation(0, 0, twist_deg)`).

## Fuselage / nose cone: draw the silhouette, revolve it

The profile must be a CLOSED face that touches the axis, so add the closing `Line` back
along the centreline.

```python
with BuildPart() as fuselage:
    with BuildSketch(Plane.XZ) as s:
        with BuildLine():
            Spline((0, 0), (120, 55), (400, 70), (760, 30), (900, 0))
            Line((900, 0), (0, 0))          # close along the axis
        make_face()
    revolve(axis=Axis.X)
# -> 8369756.9 mm^3, valid solid
```

`Spline` through control points is the workhorse for a smooth body; use `revolve(axis=…,
revolution_arc=180)` for a half-body.

## Duct / curved pipe: sweep a profile along a path

```python
with BuildPart() as duct:
    with BuildLine() as path:
        Spline((0, 0, 0), (0, 60, 25), (0, 110, 90))
    with BuildSketch(Plane.XZ) as prof:
        Circle(6)
    sweep(sections=prof.sketch, path=path.line)
# -> 16500.3 mm^3, valid solid
```

## Fillets are not decoration

A sharp trailing edge is a modelling artefact, and sharp internal corners are stress
risers. Select edges and round them:

```python
with BuildPart() as bracket:
    Box(40, 40, 10)
    fillet(bracket.edges().filter_by(Axis.Z), radius=4)
# -> 15862.7 mm^3, valid solid
```

`filter_by(Axis.Z)` / `.group_by(Axis.Z)[-1]` / `.sort_by(Axis.X)` are how you name edges
and faces without hard-coding indices that shift when a dimension changes.

## Rules that still apply to a curved part

- **A radius bigger than the local material fails.** `fillet` raises rather than silently
  shrinking; keep radius under the thinnest adjacent wall.
- **Loft needs consistent sections.** Same winding direction and a similar point count;
  wildly different profiles produce a twisted or invalid solid.
- **It is still one part in the assembly.** A lofted wing is `a.add(wing.part,
  "wing|dof=fixed|mount=fuselage")` like anything else — the label convention and the
  fits/support rules do not change because the shape is curved.
- **Check `.volume` is non-zero** before adding a swept/lofted body. A malformed profile
  can yield an empty or invalid result, and an empty part fails much later, as "the
  machine is missing a wing", with no hint that the sketch was the problem.
