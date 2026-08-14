# Benchmark score: 05_three_planet_4to1

**Total:** 55.0/100  
**Suite:** `physcad-comfort-v1`  
**Input:** `portable`  
**Unknown checks:** 3

| Layer | Weight | Status | Points |
|---|---:|---|---:|
| 1. execution | 10 | UNKNOWN | 0.0 |
| 2. assembly | 15 | PASS | 15 |
| 3. geometry | 15 | UNKNOWN | 5.0 |
| 4. physics_ready | 20 | PASS | 20 |
| 5. functional | 40 | FAIL | 15.0 |

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

- **PASS** `input_motion` — 5/5: input travel=11.94 rad reaches minimum 12
- **PASS** `motion_propagation` — 10/10: registered output exhibits motion
- **FAIL** `registered_output` — 0.0/15: input travel 9.54 is below minimum 12
- **UNKNOWN** `registered_invariants` — 0.0/10: missing named invariant evidence: equally_spaced
