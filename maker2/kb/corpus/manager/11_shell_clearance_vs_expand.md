# When a shell collides with a mechanism: cut a passage, or enlarge the shell?

A collision between a body panel and a mechanism does NOT have one universal fix. Sometimes
it means the shell must make room; other times it means the shell itself is too small and
must grow. Treating every shell-vs-part collision as "move the mechanism away" is what
pushes the whole aircraft upward; treating every one as "drill a hole" is what turns a
gearbox into swiss cheese.

The decision is by FUNCTION, not by geometry alone.

## 1. Through-body parts -> cut a passage in the shell

If the colliding part must extend to the EXTERIOR or sweep through the shell's boundary in
normal operation, the shell should yield to it. Typical examples:

- a wheel and strut entering a wing or fuselage -> wheel well / strut slot
- a tailwheel entering the tailcone -> tailwheel opening
- a propeller shaft through a spinner or cowl -> shaft hole
- a control stick, rudder pedal linkage, pushrod, or rudder/elevator shaft crossing a skin
  or canopy -> slot / tunnel / local blister / opening
- a movable rudder or elevator needing hinge-line and horn clearance -> trailing-edge cutout

**Do NOT move the mechanism to clear the shell.** The mechanism's axis is the thing that
matters; the shell is packaging around it.

The opening is derived from the moving part's own parameters, never guessed:

```python
wheel_r = 25.0
wheel_clearance = 3.0
wheel_well_r = wheel_r + wheel_clearance
wheel_well_center = (axle_x, wheel_center_z)
```

```python
shaft_r = 3.0
shaft_clearance = 0.5
shaft_hole_r = shaft_r + shaft_clearance
```

That is the pattern: **opening position follows the part's centerline / axis; opening size
= the part's working envelope + explicit clearance**.

## 2. Internal-only functional parts -> enlarge the housing envelope

If the colliding part is supposed to be COMPLETELY INSIDE the body or housing in normal
operation, then the shell or housing is too small and must grow. Typical examples:

- gears inside a reducer housing
- internal shafts and bearings inside that housing
- a crank, connecting rod, or internal linkage inside a casing
- an engine block / crankcase / gearbox casing that clips its own internal rotating parts

**Do NOT cut holes for these.** A reducer housing that "solves" gear collision by exposing
the gears is not a housing anymore.

Instead, derive the shell's inner envelope from the largest internal part envelope:

```python
gear_tip_r = ...
bearing_outer_r = ...
wall = 3.0
internal_clearance = 1.5
housing_inner_r = max(gear_tip_r, bearing_outer_r) + internal_clearance
housing_outer_r = housing_inner_r + wall
```

General rule: **inner shell = max enclosed part envelope + internal clearance; outer shell
= inner shell + wall thickness**.

## 3. The three-way decision

When shell/body S overlaps part X, ask in this order:

1. Is X a functional/internal part that should remain fully enclosed?
   - yes -> enlarge S
2. Is X a through-body or exterior-moving part whose working envelope must cross the shell?
   - yes -> cut a passage in S
3. Is X itself another structural shell / support / panel rather than a mechanism part?
   - then this is probably a placement/origin error, not a clearance issue

## 4. Reuse parameters; never invent a fresh hole size

A good shell fix reuses existing variables:

- `wheel_r`, `wheel_center_z`, `axle_x`, `axle_y`
- `shaft_r`, `shaft_axis_z`, `running_clearance`
- `gear_tip_r`, `gear_face`, `wall`, `internal_clearance`

A bad shell fix invents magic numbers:

```python
# BAD — no relation to the colliding mechanism, will be wrong next iteration
cutout_r = 31.7
cutout_x = 42.3
cutout_y = -176.8
```

If the wheel radius changes, the wheel well must follow automatically. If the shaft radius
changes, the hole radius must follow automatically. If the gears get larger, the housing
must follow automatically.

## 5. Examples from the aircraft failure

These collisions demand DIFFERENT fixes even though all are "shell vs mechanism":

- `right_main_wing` x `right_main_wheel`, `right_wheel_arbor` -> **cut a wheel well / axle
  passage in the wing**, because the wheel must occupy that space.
- `fuselage` x `tailwheel`, `tailwheel_strut`, `tailwheel_arbor` -> **cut a tailwheel bay /
  passage**, not move the wheel.
- `cockpit_canopy` x `rudder_control_input_shaft`, `...pinion`, `...pedal` -> **open canopy /
  cockpit clearance volume**, because those controls legitimately occupy cabin space.
- reducer housing x internal gear pair -> **make the housing larger**, not a hole.

The shell is packaging. Decide whether the mechanism is meant to stay inside it or pass
through it, then derive the fix from that function.
