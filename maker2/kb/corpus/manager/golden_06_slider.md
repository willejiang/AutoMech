# Worked example — one guided slider (`dof=slide`)

Use `slide` only when the part is geometrically GUIDED so it may translate along one axis
and not rotate. A plain round hole around a round shaft is not enough: that is a free or
cylindrical fit, not a slider.

Minimal pattern:

```python
def build_machine():
    from build123d import Box, Cylinder, Align, Location
    from cadpy.assembly import AssemblyHelper

    a = AssemblyHelper("slider_demo")

    base = Box(120, 40, 8, align=(Align.CENTER, Align.CENTER, Align.MIN))
    a.add(base, "base|dof=fixed")

    rail_left = Box(80, 4, 12, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, -12, 8))
    )
    rail_right = Box(80, 4, 12, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 12, 8))
    )
    a.add(rail_left, "rail_left|dof=fixed|mount=base")
    a.add(rail_right, "rail_right|dof=fixed|mount=base")

    carriage = Box(20, 20, 10, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((-20, 0, 10))
    )
    a.add(carriage, "carriage|dof=slide|slide_axis=x|driver=True|mount=rail_left,rail_right")

    plunger = Cylinder(2, 30, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, 10))
    )
    a.add(plunger, "plunger|dof=fixed|mount=carriage")

    return a.build()
```

What matters:
- the **carriage** is the thing that slides, so it gets `dof=slide`
- the axis is explicit: `slide_axis=x`
- the guides remain `fixed`
- the carried plunger is `fixed` on the carriage, not another slider

A crank-slider adds two `spin` joints (crank and rod ends) to this same slider primitive; a
rack-and-pinion replaces the plunger with a toothed rack, but the rack is still `dof=slide`.
