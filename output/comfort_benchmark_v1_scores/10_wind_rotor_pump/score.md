# Benchmark score: 10_wind_rotor_pump

**Total:** 25.0/100  
**Suite:** `physcad-comfort-v1`  
**Input:** `automech`  
**Unknown checks:** 4

| Layer | Weight | Status | Points |
|---|---:|---|---:|
| 1. integrity | 10 | PASS | 10 |
| 2. artifacts | 15 | PASS | 15 |
| 3. semantics | 15 | UNKNOWN | 0.0 |
| 4. trajectory | 20 | UNKNOWN | 0.0 |
| 5. function | 40 | UNKNOWN | 0.0 |

## 1. integrity

- **PASS** `trusted_ingestion` — 10/10: AutoMech folder safely inventoried; audit verdicts excluded

## 2. artifacts

- **PASS** `assembly_model` — 5/5: assembly/kinematic model available
- **PASS** `simulation_model` — 5/5: a structural or simulator model is available
- **PASS** `geometry` — 5/5: mesh geometry is hash-addressed

## 3. semantics

- **UNKNOWN** `required_role_cardinalities` — 0.0/10: missing role bindings: vertical_crosshead, vertical_guide, piston_output
- **UNKNOWN** `input_output_independence` — 0.0/5: input/output bindings are incomplete

## 4. trajectory

Prerequisite gate: prerequisite layer 3 (semantics) is UNKNOWN

- **UNKNOWN** `prerequisite_gate` — 0.0/20: prerequisite layer 3 (semantics) is UNKNOWN

## 5. function

Prerequisite gate: prerequisite layer 4 (trajectory) is UNKNOWN

- **UNKNOWN** `prerequisite_gate` — 0.0/40: prerequisite layer 4 (trajectory) is UNKNOWN
