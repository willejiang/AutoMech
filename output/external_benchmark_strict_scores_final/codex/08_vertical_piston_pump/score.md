# External strict score: codex / 08_vertical_piston_pump

**Total:** 30/100
**Strict verdict:** FAIL
**First blocker:** non_exempt_conflicts: strict non-exempt conflicts=7; unavailable=0

| Layer | Status | Points |
|---|---|---:|
| execution | PASS | 10/10 |
| assembly | PASS | 15/15 |
| geometry | FAIL | 5/15 |
| physics_ready | GATED | 0/20 |
| functional | GATED | 0/40 |

## execution

- **PASS** `scorer_owned_execution` — 10/10: scorer-owned source build produced a nonempty part set; MJCF replay compiled, initialized, and stayed finite

## assembly

- **PASS** `assembly_model` — 5/5: assembly/simulation model is available
- **PASS** `required_role_cardinalities` — 7/7: all required roles are bound
- **PASS** `input_output_independence` — 3/3: input and output are distinct

## geometry

- **PASS** `mesh_inventory` — 5/5: strict geometry hashes every scanned usable mesh
- **FAIL** `non_exempt_conflicts` — 0/10: strict non-exempt conflicts=7; unavailable=0

## physics_ready

Strict gate: FAIL — collision_coverage_ok, transmission_binding_ok, one or more physical mesh links lack active collision coverage

- **GATED** `trajectory_shape` — 0/5; raw=PASS: trajectory has time samples and joint series
- **GATED** `sample_alignment` — 0/5; raw=PASS: all joint series align with time
- **GATED** `finite_health` — 0/10; raw=PASS: finite sample fraction=1

## functional

- **GATED** `input_motion` — 0/5; raw=FAIL: input travel=1.59567 rad is below minimum 6.28319
- **GATED** `motion_propagation` — 0/10; raw=FAIL: input minimum not reached
- **GATED** `registered_output` — 0/15; raw=FAIL: input minimum not reached
- **GATED** `registered_invariants` — 0/10; raw=FAIL: input minimum not reached
