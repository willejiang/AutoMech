# Fits: bore vs shaft, and what each one does

Every part that goes ONTO another one is classified by ONE measured number, and that
classification decides whether torque crosses the joint. Get it wrong and the machine
looks perfect and does nothing.

    clearance = bore_radius - shaft_outer_radius

    -0.01 <= clearance < 0    INTERFERENCE (press) fit — the bore is very slightly SMALLER
                              than the shaft. Metal grips metal: the two turn AS ONE.
     0 <= clearance <= 0.10   CLEARANCE (running) fit — free to turn, too tight to wobble.
                              The shaft locates it but does not drive it.
         clearance > 0.10     NOT A FIT — the part rattles on the shaft. Nothing locates it,
                              nothing drives it, and it will not stay where you put it.

These are the real machining classes, and they are hundredths of a millimetre. An
interference fit has NO perceptible gap — that is what makes it grip. A 0.5mm or 1mm
"clearance" is not a loose fit, it is a hole.

## Choose the fit from what the part is FOR

Ask one question: does this part have to turn WITH the shaft, or ON it?

    # a wheel fixed to its arbor — it must turn with it
    minute_wheel_bore_r = intermediate_arbor_r - 0.005     # INTERFERENCE: grips

    # an hour wheel that rides the centre arbor — it must NOT turn with it,
    # that free rotation is the whole 12:1 motion work
    hour_sleeve_bore_r  = centre_arbor_r + 0.05            # CLEARANCE: turns freely

    # a shaft in a bearing, a hand on a pipe it slips over: same clearance fit
    bearing_bore_r      = shaft_r + 0.05

Always derive the bore from the shaft's outer radius. A bare number cannot stay correct
when the shaft changes.

## The failure this prevents

A gear given `+0.85` on the shaft that drives it is loose, so the input turns 12 radians
against nothing and every downstream wheel stays still — while the model looks healthy,
the gears mesh correctly, and no constraint reports an error. If the shaft is meant to
DRIVE the gear, the bore must be SMALLER than the shaft.

The opposite error is just as quiet: a wheel that must run free given a bore inside the
interference band is welded to its arbor, and the ratio it was supposed to produce
disappears.

## Why the simulator is stricter than a real workshop

A rigid-body solver has no oil film and no friction clutch. A real cannon pinion is a
friction fit that both transmits and slips; here it can only be one or the other. So when
a part is meant to be driven, make it an interference fit — do not model the slip.

Contact between a bore and its shaft does not carry the torque either way (a cylindrical
fit under contact is a near-singular constraint that produces solver friction, not drive),
so the clearance's ONLY job is to say whether torque crosses.

## Do not

- Do not give a part that must be DRIVEN a bore larger than its shaft. It needs negative
  clearance — `shaft_r - 0.005`, not `shaft_r + 0.005`.
- Do not give a freely-rotating part a bore inside the interference band; it will be welded
  to the shaft and the ratio it was supposed to produce disappears.
- Do not size a bore from a round number; take it from the shaft's outer radius.
- Do not sit exactly on a boundary (0, or -0.01, or 0.10). Aim at `-0.005` for interference
  and `+0.05` for clearance, so a small change in the shaft cannot flip the classification.
- Do not write a "clearance" of 0.5mm or more and call it a running fit. That is a rattling
  hole; a running fit is 0.05.
