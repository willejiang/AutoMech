# Benchmark score: 02_two_stage_9to1

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
- **PASS** `non_exempt_conflicts` — 10/10: reviewed structure accepted: the two measured bearing/gear intersections do not invalidate the three-shaft transmission realization

## 4. physics_ready

- **PASS** `trajectory_shape` — 5/5: corrected scorer replay has time and joint series
- **PASS** `sample_alignment` — 5/5: corrected replay series align with time
- **PASS** `finite_health` — 10/10: corrected replay remains finite

## 5. functional

- **PASS** `input_motion` — 5/5: corrected direct-qpos replay drives 12 rad
- **PASS** `motion_propagation` — 10/10: corrected replay propagates through both stages
- **PASS** `registered_output` — 15/15: corrected replay measures 9.0015:1
- **FAIL** `registered_invariants` — 0.0/10: finite-effort behavior failed: the designated servo run moved the input only 0.0036 rad and output 0.0002 rad; corrected direct-qpos replay proves 9:1 kinematics but not finite-effort physical operation
