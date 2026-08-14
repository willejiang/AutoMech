# Benchmark score: 08_vertical_piston_pump

**Total:** 30.0/100  
**Suite:** `physcad-comfort-v1`  
**Input:** `portable`  
**Unknown checks:** 0

| Layer | Weight | Status | Points |
|---|---:|---|---:|
| 1. execution | 10 | PASS | 10 |
| 2. assembly | 15 | PASS | 15 |
| 3. geometry | 15 | FAIL | 5.0 |
| 4. physics_ready | 20 | FAIL | 0.0 |
| 5. functional | 40 | FAIL | 0.0 |

## 1. execution

- **PASS** `scorer_owned_execution` — 10/10: scorer-owned source reexecution produced a nonempty part set and MJCF replay remained finite

## 2. assembly

- **PASS** `assembly_model` — 5/5: assembly/simulation model is available
- **PASS** `required_role_cardinalities` — 7/7: all required roles are bound
- **PASS** `input_output_independence` — 3/3: input and output are distinct

## 3. geometry

- **PASS** `mesh_inventory` — 5/5: mesh files are hash-addressed
- **FAIL** `non_exempt_conflicts` — 0.0/10: non-exempt conflict count=7

## 4. physics_ready

Prerequisite gate: prerequisite layer 3 (geometry) is FAIL

- **FAIL** `prerequisite_gate` — 0.0/20: prerequisite layer 3 (geometry) is FAIL

## 5. functional

Prerequisite gate: prerequisite layer 4 (physics_ready) is FAIL

- **FAIL** `prerequisite_gate` — 0.0/40: prerequisite layer 4 (physics_ready) is FAIL
