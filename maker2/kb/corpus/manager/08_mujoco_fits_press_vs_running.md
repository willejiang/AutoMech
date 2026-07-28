# Fits in this simulator: bore vs shaft, and what each one does

Every part that goes ONTO another one is classified by ONE measured number, and that
classification decides whether torque crosses the joint. Get it wrong and the machine
looks perfect and does nothing.

    clearance = bore_radius - shaft_outer_radius

    0 < clearance <= 0.10 mm     PRESS FIT    the two turn AS ONE (locked 1:1)
        clearance >  0.10 mm     RUNNING FIT  it turns FREELY, the shaft does not drive it

There is no third case and no grey zone: 0.10mm is the line. A negative clearance (bore
smaller than the shaft) is not a tight fit — the part cannot be assembled at all.

## Choose the fit from what the part is FOR

Ask one question: does this part have to turn WITH the shaft, or ON it?

    # a wheel keyed to its arbor — it must turn with it
    minute_wheel_bore_r = intermediate_arbor_r + 0.05      # PRESS: 0.05 <= 0.10

    # an hour wheel that rides the centre arbor — it must NOT turn with it,
    # that free rotation is the whole 12:1 motion work
    hour_sleeve_bore_r  = centre_arbor_r + 0.60            # RUNNING: 0.60 > 0.10

Always derive the bore from the shaft's outer radius. A bare number cannot stay correct
when the shaft changes.

## The failure this prevents

A gear given `+0.85` on the shaft that drives it is a running fit, so the input turns 12
radians against nothing and every downstream wheel stays still — while the model looks
healthy, the gears mesh correctly, and no constraint reports an error. If the shaft is
meant to DRIVE the gear, the clearance must be 0.10mm or less.

## Why the simulator is stricter than a real workshop

A rigid-body solver has no oil film and no friction clutch. A real cannon pinion is a
friction fit that both transmits and slips; here it can only be one or the other. So when
a part is meant to be driven, make it a press fit — do not model the slip.

Contact between a bore and its shaft is excluded either way (simulating it produces only
solver friction that stalls the train), so the clearance's ONLY job is to say whether
torque crosses.

## Do not

- Do not give a driven gear more than 0.10mm of clearance on its shaft.
- Do not give a freely-rotating part less than 0.10mm — it will be welded to the shaft and
  the ratio it was supposed to produce disappears.
- Do not size a bore from a round number; take it from the shaft's outer radius.
- Do not put a clearance right at 0.10mm. Aim at 0.05 for a press fit and 0.5+ for a
  running fit, so a small change in the shaft cannot flip the classification.
