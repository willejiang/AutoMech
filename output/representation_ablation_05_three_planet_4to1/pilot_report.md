# Task 05 representation ablation pilot

## Result

| Arm | Score | Verdict | Iterations | Runtime | Exact non-exempt conflicts |
|---|---:|---|---:|---:|---:|
| Executable dependencies (archived baseline) | 100/100 | PASS | 1 | 731.576 s | 0 |
| Independent values (fresh pilot) | 25/100 | FAIL | 3 | 1248.293 s | 17 |

The independent-values candidate passed source execution and final structural role/cardinality checks, but failed the cumulative Geometry gate and therefore received no Physics-ready or Functional points. Its MJCF candidate compiled and remained finite, but retained contacts reduced driver travel from `0.977454` rad to `0.001541` rad (0.158% of the no-contact control), so policy v5 correctly rejected it.

## Dependency consistency

Evaluated-only $Q_{dep}=0.8800$ (22/25 checks pass); 5 dynamic checks are explicitly unavailable and excluded from the denominator. This high static score does **not** imply a functional mechanism: repeated literals happened to preserve tooth arithmetic, centers, and spacing, while the independent axial/radial choices produced invalid fit/contact realization.

Failed evaluated dependencies:
- `authored_ring_planet_mesh_count`: expected 3, observed 0
- `input_bearing_press_fit`: expected negative, observed 0.1999999999999993
- `contact_preserved_input_mobility`: expected 1.0, observed 0.0015766805001498024

Unavailable dynamic dependencies:
- `trajectory_carrier_sun_ratio`: No MJCF compiler was accepted; no selected final trajectory exists.
- `planet_orbit`: No accepted physics trajectory exists.
- `planet_local_spin`: No accepted physics trajectory exists.
- `ring_fixed_trajectory`: No accepted physics trajectory exists.
- `planet_pin_carrying_constancy`: No accepted physics trajectory exists.

## Cost and refinement

- Runtime: `731.576 s -> 1248.293 s`
- Requests: `24 -> 31`
- Total tokens: `1255551 -> 2452556`
- Tool calls: `156 -> 210`
- MJCF candidates/submissions: `4/4 -> 7/8`
- Iterations 0 and 1 both failed declared crank/shaft proximity; iteration 2 repaired that gate but still failed retained-contact dynamics.
- Both runs observed provider/project cache activity, so neither qualifies as a strict cold run.

## Isolation

Production source hashes unchanged: `True`. Production prompt hash unchanged: `True`. The overlay existed only in the experiment process and all added files are under ignored `output/representation_ablation_05_three_planet_4to1/`.

## Interpretation

This single pilot is direct evidence of degradation on the selected task (`100/PASS -> 25/FAIL`), but it is not sufficient for a population estimate or statistical claim. No additional stochastic rerun was performed.
