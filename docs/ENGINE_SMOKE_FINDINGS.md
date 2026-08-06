# Engine smoke findings (single-cylinder slider prototype)

_Date: 2026-08-05_

This note records what the "small single-cylinder piston engine" smoke actually proved,
what it did **not** prove, and the current conclusions about why it fails.

Primary run examined:
- `output/a_small_single_cylinder_piston_e_20260805_173823/`
- especially `iter_47`, `iter_48`, `iter_49`

---

## Executive summary

The smoke shows a **real translational slider** can be driven and measured in the current
single-agent + MuJoCo path. The piston slider moves reciprocally along x with negligible
lateral drift.

However, it does **not** show a correct crank-slider linkage yet. In the generated model,
`piston_wrist_pin` and `connecting_rod` are both welded to `piston_slider`, so the "rod"
does not articulate — it is effectively decoration attached to the slider. This is the key
current limitation.

Separately, the VLM/diagnosis text for the last iterations blamed a `flywheel` /
`crankshaft` interference, but a direct re-measurement of the final geometry found **no such
real overlap**. The actual reproducible overlaps are elsewhere.

---

## What the smoke got right

### 1. The slider primitive works

The generated part is:
- `piston_slider|dof=slide|slide_axis=x|mount=cylinder_guide`

That is the first meaningful sign that the new `slide` DOF is being used as intended.

### 2. The piston motion is real and correctly constrained

From `mujoco_48`:

- functional check: `x_only_piston_reciprocation` -> **PASS**
- `stroke_m = 0.004941`
- `direction_reversal = True`
- `lateral_to_x_ratio = 0.0`

Interpretation:
- the slider actually reciprocates
- it moves essentially only along x
- this is not just a free body wobbling in space

### 3. Support/stability are not the failure mode

Also from `mujoco_48`:
- support test: **every part held up under gravity**
- stability verdict: **PASS**
- no explosion / no large drift of the assembly as a whole

So the current engine smoke is not failing because the machine collapses or floats.

---

## What the smoke got wrong

### 1. `connecting_rod` is not a real connecting rod

The final `iter_49/machine.py` declares:

```python
"piston_slider|dof=slide|slide_axis=x|mount=cylinder_guide"
"piston_wrist_pin|dof=fixed|mount=piston_slider"
"connecting_rod|dof=fixed|mount=piston_slider"
```

And the emitted MJCF confirms:

```xml
<body name="piston_slider">
  <joint name="piston_slider_slide" type="slide" axis="1 0 0" />
</body>
<body name="piston_wrist_pin"> ... </body>
<body name="connecting_rod"> ... </body>
<weld body1="piston_wrist_pin" body2="piston_slider" />
<weld body1="connecting_rod" body2="piston_slider" />
```

That means both the wrist pin and the connecting rod are rigidly attached to the slider.
There is no revolute relation at the wrist pin and no independent rod articulation.

### 2. The body trajectories confirm that only the slider moves

From `trajectory.json` (same smoke):

| body | x start -> end | delta x |
|---|---:|---:|
| `piston_slider` | `1.577 -> 3.327` | `+1.750 mm` |
| `piston_wrist_pin` | `49.619 -> 49.619` | `0` |
| `connecting_rod` | `8.485 -> 8.485` | `0` |

So the slider moves, but the wrist pin and rod bodies do not. This is incompatible with a
real crank-slider.

### 3. The geometry overlap the diagnoser kept citing is the wrong one

The final VLM/diagnosis text repeatedly said:

> The flywheel physically intersects the crankshaft by 0.753 mm³...

That is **not** true for the final geometry.

Direct re-measurement of `iter_49` (re-exported STLs with world transforms applied):

#### `crankshaft` vs `flywheel`
- AABB z-gap: **0.25 mm**
- sampled mesh nearest distance: **0.25 mm**
- boolean intersection volume: **0.0**
- `_solid_intersection_frac(...)`: **0.0**

So the flywheel/crankshaft interference diagnosis for the final geometry is a false
positive or stale signal.

---

## The real final overlaps

Rebuilding the final geometry and re-running the interference check finds these real
collisions instead:

### 1. `piston_slider` vs `connecting_rod`
- boolean intersection volume: **42.0 mm³**

Their world-space extents are almost identical in x and y, with the rod as a thin plate
embedded through the slider's body:

- `piston_slider`
  - x: `2.44 .. 61.62`
  - y: `-19 .. 19`
  - z: `10 .. 30`
- `connecting_rod`
  - x: `2.44 .. 61.62`
  - y: `-19 .. 19`
  - z: `21.1 .. 22.1`

This is not a tiny numerical overlap. It means the rod occupies solid volume already owned
by the slider.

### 2. `piston_slider` vs `left_frame_column`
- boolean intersection volume: **6.259 mm³**

### 3. `piston_slider` vs `right_frame_column`
- boolean intersection volume: **6.259 mm³**

These indicate the guide/frame columns are too close to the slider envelope.

### 4. `piston_slider` vs `cylinder_guide`
- nearest distance: **~0.047 mm**
- boolean intersection volume: **0.0**

This one is fine: it is a tight guide / running-clearance relationship, not a real
interference.

---

## Interpretation

### What this smoke proves

- The newly added `slide` primitive is useful and physically meaningful.
- The system can make a linearly constrained moving part and verify its motion.
- A single-agent prompt can now produce a machine whose **slider** behaves correctly.

### What this smoke does not yet prove

- It does **not** prove the system can generate a proper crank-slider linkage.
- It does **not** prove pin-connected multi-body linkages are expressible in the current
  single-agent semantics.

### The real current limitation

The current `dof` vocabulary can express:
- `fixed`
- `spin`
- `slide`
- `free`

But it still cannot express the missing relation for the connecting rod:

> a rigid part that is neither fixed to the slider nor free in space, but rotates about a
> pin relative to another moving body.

That is why the generator falls back to welding the rod to the slider.

---

## Working conclusion

The engine smoke should be read as:

> **P1 (`slide`) succeeded.**
>
> **P2 (pin-connected linkage semantics) is still missing.**

The current single-cylinder engine is therefore not a failed slider; it is a successful
slider embedded in an incorrectly expressed linkage.

---

## Recommended next step

Do **not** keep iterating this engine with prompt tweaks alone.

Instead, treat the run as evidence that the next capability to add after `slide` is a way
to express the rod / wrist-pin relationship honestly, e.g. a pin-connected linkage semantic
for single-agent authoring.

Until that exists, the most honest fallback is:
- keep `piston_slider` real and physical
- keep `connecting_rod` / `piston_wrist_pin` as display-only geometry
- do not claim the result is a correct crank-slider mechanism
