# What a bearing, washer, spacer or hand actually rides on

A part that goes ONTO something rides on a THIN round part — an arbor, a staff, a pipe, a
post. It never rides on the RIM of a wheel. This sounds obvious and is the single most
common way a generated movement comes out unassemblable: the wheel and the arbor are
concentric, so "on the wheel" and "on the arbor" look the same in a drawing, and only one
of them is a part you can actually slide a 3mm jewel onto.

## The test: compare the two radii before you write `mount=`

    jewel_outer_r      = 3.2      # the whole part is 3.2mm across
    centre_arbor_r     = 1.0      # thin: a jewel rings it easily
    hour_wheel_outer_r = 12.5     # the wheel's TOOTH TIPS

A part can only mount on something it can encircle, so its OUTER radius must exceed the
other part's outer radius, with the bore in between:

    shaft_r  <  bore_r  <  part_outer_r

`mount="centre_arbor"` satisfies that (1.0 < 1.05 < 3.2). `mount="hour_wheel"` cannot:
the jewel is 3.2mm across and the wheel is 25mm across, so no bore you cut will ever let
that jewel go onto the wheel. Enlarging the bore does not help — at bore 12.55 there is no
part left. **The fix is the mount, not the bore.**

## Concentric is not the same as mounted

An hour wheel, its pipe and the centre arbor all share one axis. A washer sitting at that
axis is touching the PIPE, not the wheel — the wheel's material is out at radius 12.5,
nowhere near the washer. Write the mount that names the part whose SURFACE the washer
actually contacts.

    hour_wheel_thrust_washer  mount="hour_pipe"     # correct: it rides the pipe
    hour_wheel_thrust_washer  mount="hour_wheel"    # wrong: nothing to ride

## Where each kind of part belongs

| part | rides on | never on |
|---|---|---|
| jewel / bearing / bushing | the arbor or staff that turns in it | a wheel, a pinion |
| thrust washer / spacer / collar | the arbor or pipe it stacks along | a wheel's rim |
| hour hand | the hour pipe (outer tube) | the hour wheel |
| minute hand | the cannon pinion pipe (inner tube) | the centre wheel |
| a wheel or pinion | its own arbor (press fit) | another wheel |
| a bridge / plate | its pillars or posts | anything rotating |

## Do not

- Do not `mount=` a small passive part onto a gear, wheel or pinion whose radius is much
  larger than the part itself. If the two radii are within a couple of millimetres it may
  be a genuine press fit; if the "shaft" is several times wider than the part, the mount is
  wrong.
- Do not fix a "bore smaller than the shaft" report by enlarging the bore when the shaft in
  question is a wheel. That turns a wrong mount into a hollowed-out ring and still does not
  assemble.
- Do not size a bore from a round number. Take it from the outer radius of the part it
  slides onto, and pick the class from what it must do: `<that part>_outer_r + 0.05` for a
  passive part that slides on and turns freely (bearing, washer, spacer, hand), or
  `<that part>_outer_r - 0.005` for one that must turn WITH the shaft. See
  `08_mujoco_fits_press_vs_running.md`.
