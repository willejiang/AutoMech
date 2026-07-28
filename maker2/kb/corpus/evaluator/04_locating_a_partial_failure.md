# Locating the fault when a mechanism only partly works

"5 of 6 joints moved" is not a diagnosis. It says something is wrong and nothing about
what — and a verdict of "the mechanism did not work, but no specific fault was isolated"
is worth nothing to whoever has to fix the CAD. The measurements needed to name the
culprit are already in front of you. This is how to read them.

## Start from PER-JOINT TRAVEL, not from the video

You are given how far every joint actually turned. Sort it and the fault usually names
itself. Read the numbers as a chain running from the driven input outward:

    minute_cannon_pinion   14.87    <- driven input
    minute_arbor            3.06
    compound_minute_wheel   1.06
    compound_arbor          1.01
    compound_hour_pinion    0.97
    hour_wheel              0.099
    hour_hand_pipe          0.0003  <- did not move

Two different faults are visible here, and they need different words:

- **A cliff to zero** — `hour_hand_pipe` gets 0.0003 while the part driving it turned
  0.099. Motion reaches its neighbour and stops. That is a BREAK: the last joint that
  moved and the first that did not are the two parts to talk about.
- **A step that should not exist** — 14.87 to 3.06 is a 4.9x reduction between two parts
  that are supposed to be pressed on the same arbor and turn as one. A 1:1 pair that does
  not come out 1:1 is a fault even though both parts moved.

## Then say WHICH parts and WHY, in that order

A useful reason names the two parts at the break and the reason it is a break. Work
through the possibilities in order of how easily each is confirmed:

1. **Nothing connects them.** No constraint was emitted for that pair, and their teeth do
   not touch either. The downstream part is only carried by whatever it rests on.
2. **The fit is too loose to drive.** The part rides its shaft instead of being pressed
   to it, so the shaft turns inside it. Look for a clearance far above a press fit.
3. **Something blocks it.** Another solid occupies the space it must sweep through, or a
   contact force at that joint is enormous. The joint moves a little and stalls.
4. **It is driven, but at the wrong rate.** Both parts move and the ratio between them is
   not what the tooth counts should give. That is arithmetic, not assembly.

## Ratios are evidence, not just a verdict

When a functional check reports a ratio far from its target, do not stop at "the ratio is
wrong". Walk the per-joint travels and find WHERE it went wrong: divide each joint's
travel by the one before it in the chain, and the stage that contributes the unexpected
factor is the one to name. A 12:1 machine measuring 150:1 has roughly 12x of extra
reduction hiding in one or two stages; say which.

## What to write

Bad:  "the mechanism did not work, but no specific fault was isolated"
Bad:  "the gear train has issues"
Good: "motion dies between hour_wheel (0.099 rad) and hour_hand_pipe (0.0003 rad) —
       the pipe is not driven by the wheel it is mounted on"
Good: "minute_arbor turns 4.9x slower than the cannon pinion pressed onto it; a 1:1
       press fit is not being transmitted, which is most of the 150:1 vs 12:1 error"

Name parts. Give the numbers you used. If the evidence genuinely does not single anything
out, say what you ruled out rather than saying nothing.

## Two joints turning at the SAME rate that should not

A reduction stage that comes out 1.00 is not a mild ratio error — it means the stage does
not exist. Some pair that was supposed to turn at different rates is locked together.

    hour_pipe_spin    0.9479
    hour_wheel_spin   0.9479     <- identical to the last digit

Identical travel is the signature. Gear teeth never produce that; a rigid coupling does.
Before blaming tooth counts, check whether the two were WELDED, and look in this order:

1. **A running fit judged as a press fit.** A part that is meant to rotate freely on a
   shaft gets locked 1:1 to it when its bore sits within press-fit clearance of that
   shaft. In concentric motion work this is fatal in a specific way: the hour side is
   supposed to turn slowly INSIDE the minute side, and if the pipe is pressed to the
   centre arbor instead of riding it, the whole 12:1 is short-circuited to 1:1 and both
   hands sweep together. Say so — the fix is bore clearance, not tooth counts.
2. **Both parts pressed onto one shaft.** Correct for a wheel and its own pinion; wrong
   when the two are supposed to be different stages of the train.
3. **The stage was never built.** No constraint and no meshing pair exists between them,
   and they only move together because a third part carries both.

The counterpart symptom is a ratio that is a clean multiple or fraction of the target
(1:1, 2x, half): that points at a missing or duplicated STAGE rather than at a tooth count
being a few teeth off. A few-teeth error shows up as a few percent, not as a factor.
