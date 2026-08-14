# External strict score: codex / 01_single_stage_4to1

**Total:** 100/100
**Strict verdict:** PASS
**First blocker:** none

| Layer | Status | Points |
|---|---|---:|
| execution | PASS | 10/10 |
| assembly | PASS | 15/15 |
| geometry | PASS | 15/15 |
| physics_ready | PASS | 20/20 |
| functional | PASS | 40/40 |

## execution

- **PASS** `scorer_owned_execution` — 10/10: scorer-owned source build produced a nonempty part set; MJCF replay compiled, initialized, and stayed finite

## assembly

- **PASS** `assembly_model` — 5/5: assembly/simulation model is available
- **PASS** `required_role_cardinalities` — 7/7: all required roles are bound
- **PASS** `input_output_independence` — 3/3: input and output are distinct

## geometry

- **PASS** `mesh_inventory` — 5/5: strict geometry hashes every scanned usable mesh
- **PASS** `non_exempt_conflicts` — 10/10: strict non-exempt conflicts=0; unavailable=0

## physics_ready

Strict gate: PASS — strict model audit passed

- **PASS** `trajectory_shape` — 5/5: trajectory has time samples and joint series
- **PASS** `sample_alignment` — 5/5: all joint series align with time
- **PASS** `finite_health` — 10/10: finite sample fraction=1

## functional

- **PASS** `input_motion` — 5/5: input travel=8 rad reaches minimum 6
- **PASS** `motion_propagation` — 10/10: registered output exhibits motion
- **PASS** `registered_output` — 15/15: measured declared ratio=4, slope=-4, R^2=1
- **PASS** `registered_invariants` — 10/10: all registered invariants have passing named evidence
