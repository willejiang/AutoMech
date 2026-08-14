# Benchmark score: 01_single_stage_4to1

**Total:** 90.0/100
**Suite:** `physcad-comfort-v1`
**Input:** `portable`
**Unknown checks:** 0

| Layer | Weight | Status | Points |
|---|---:|---|---:|
| 1. execution | 10 | PASS | 10 |
| 2. assembly | 15 | PASS | 15 |
| 3. geometry | 15 | PASS | 15 |
| 4. physics_ready | 20 | PASS | 20 |
| 5. functional | 40 | FAIL | 30.0 |

## 1. execution

- **PASS** `scorer_owned_execution` — 10/10: scorer-owned source reexecution produced a nonempty part set and MJCF replay remained finite

## 2. assembly

- **PASS** `assembly_model` — 5/5: assembly/simulation model is available
- **PASS** `required_role_cardinalities` — 7/7: all required roles are bound
- **PASS** `input_output_independence` — 3/3: input and output are distinct

## 3. geometry

- **PASS** `mesh_inventory` — 5/5: mesh files are hash-addressed
- **PASS** `non_exempt_conflicts` — 10/10: non-exempt conflict count=0

## 4. physics_ready

- **PASS** `trajectory_shape` — 5/5: trajectory has time samples and joint series
- **PASS** `sample_alignment` — 5/5: all joint series align with time
- **PASS** `finite_health` — 10/10: finite sample fraction=1

## 5. functional

- **PASS** `input_motion` — 5/5: input travel=11.94 rad reaches minimum 6
- **PASS** `motion_propagation` — 10/10: registered output exhibits motion
- **PASS** `registered_output` — 15/15: measured declared ratio=4, slope=-4, R^2=1
- **FAIL** `registered_invariants` — 0.0/10: fixed_shaft_axes failed: selected trajectory shows the input/output shaft centers orbiting by approximately 45.00/44.86 mm because both hinge axes were lowered at the world origin
