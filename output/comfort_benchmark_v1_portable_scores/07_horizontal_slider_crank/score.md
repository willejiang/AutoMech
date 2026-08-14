# Benchmark score: 07_horizontal_slider_crank

**Total:** 70.0/100  
**Suite:** `physcad-comfort-v1`  
**Input:** `portable`  
**Unknown checks:** 3

| Layer | Weight | Status | Points |
|---|---:|---|---:|
| 1. execution | 10 | UNKNOWN | 0.0 |
| 2. assembly | 15 | PASS | 15 |
| 3. geometry | 15 | UNKNOWN | 5.0 |
| 4. physics_ready | 20 | PASS | 20 |
| 5. functional | 40 | UNKNOWN | 30.0 |

## 1. execution

- **UNKNOWN** `source_reexecution` — 0.0/10: source is present but was not executed by the untrusted-folder scorer

## 2. assembly

- **PASS** `assembly_model` — 5/5: assembly/simulation model is available
- **PASS** `required_role_cardinalities` — 7/7: all required roles are bound
- **PASS** `input_output_independence` — 3/3: input and output are distinct

## 3. geometry

- **PASS** `mesh_inventory` — 5/5: mesh files are hash-addressed
- **UNKNOWN** `non_exempt_conflicts` — 0.0/10: no scorer-owned solid-intersection result was submitted

## 4. physics_ready

- **PASS** `trajectory_shape` — 5/5: trajectory has time samples and joint series
- **PASS** `sample_alignment` — 5/5: all joint series align with time
- **PASS** `finite_health` — 10/10: finite sample fraction=1

## 5. functional

- **PASS** `input_motion` — 5/5: input travel=12.1009 rad reaches minimum 6.28319
- **PASS** `motion_propagation` — 10/10: registered output exhibits motion
- **PASS** `registered_output` — 15/15: output span=24.006 mm, reversals=3
- **UNKNOWN** `registered_invariants` — 0.0/10: missing named invariant evidence: lateral_drift_le_2pct_span, closures_below_2pct_scale
