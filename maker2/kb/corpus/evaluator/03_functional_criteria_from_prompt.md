# Turning a product description into a checkable functional criterion

A machine can move perfectly and still be the wrong machine. Every joint turns, nothing
jams, the recording looks alive — and the tooth counts produce 10.4:1 where the product
needed 12:1. "Did it move" cannot see that. A RATIO can.

Your job when designing the test: read the user's own words, work out what number the
machine is supposed to produce, and declare it as `expected_ratio` so the run can be
checked against it instead of merely watched.

## Read the prompt for the number hiding in the domain word

Users describe machines in domain language, not in ratios. The ratio is usually implied
by the noun rather than stated:

| what the prompt says | what it implies you can check |
|---|---|
| names a reduction ("a 20:1 reducer", "reduces 5 times") | the number is given: input turns / output turns = 20 |
| a device with two concentric hands showing time | those two outputs are geared 12:1 to each other |
| "steps the speed up/down" with no number | no number to check — say so, don't invent one |
| a static object, a fixture, a case | no transmission at all; there is no ratio to declare |

Work it out like this:

    "a skeleton wristwatch"
      -> a watch shows hours and minutes on ONE axis
      -> those two hands are geared to each other at 12:1 by definition
      -> which part carries the minute hand? which carries the hour hand?
      -> declare expected_ratio = 12.0 between them

    "a two-stage gearbox, total reduction 20:1"
      -> the number is stated outright
      -> input link is the driver; output is the last wheel in the chain
      -> declare expected_ratio = 20.0

    "a hand-cranked fan"
      -> a fan has no specified speed; the crank just has to drive it
      -> declare NO expected_ratio, and let "the output turned" be the criterion

## Do not invent a number

If the description does not pin a ratio down, leave `expected_ratio` unset. A wrong
expectation is worse than none: it fails a machine that was built exactly as asked, and
the next iteration will "fix" something that was never broken.

## Drive it at a rate, not with a shove

State `mode` and `target_velocity` for the input. A fixed torque is meaningless across
machines — the same push that barely turns a steel gearbox spins a milligram watch train
hundreds of revolutions, and then the input travel says more about the part masses than
about the mechanism. Commanding a rate makes the input's motion a property of the test,
which is also what makes an output/input ratio comparable to the designed one.

## What the ratio then tells the loop

- ratio matches the declared one -> the train is not just connected, it is CORRECT.
- ratio is off by tens of percent -> the tooth counts are wrong, even though every joint
  moved. This is a design fault, and naming it is the only way it gets fixed.
- no output motion at all -> not a ratio problem; something upstream is jammed or loose.
