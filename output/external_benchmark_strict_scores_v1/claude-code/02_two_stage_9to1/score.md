# External strict score: claude-code / 02_two_stage_9to1

**Total:** 40/100
**Strict verdict:** FAIL
**First blocker:** strict_model_audit: collision_coverage_ok, one or more physical mesh links lack active collision coverage

| Layer | Status | Points |
|---|---|---:|
| execution | PASS | 10/10 |
| assembly | PASS | 15/15 |
| geometry | PASS | 15/15 |
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
- **PASS** `non_exempt_conflicts` — 10/10: strict non-exempt conflicts=0; unavailable=0

## physics_ready

Strict gate: FAIL — collision_coverage_ok, one or more physical mesh links lack active collision coverage

- **GATED** `trajectory_shape` — 0/5; raw=PASS: trajectory has time samples and joint series
- **GATED** `sample_alignment` — 0/5; raw=PASS: all joint series align with time
- **GATED** `finite_health` — 0/10; raw=PASS: finite sample fraction=1

## functional

- **GATED** `input_motion` — 0/5; raw=PASS: input travel=10 rad reaches minimum 9
- **GATED** `motion_propagation` — 0/10; raw=PASS: registered output exhibits motion
- **GATED** `registered_output` — 0/15; raw=PASS: measured declared ratio=9, slope=9, R^2=1
- **GATED** `registered_invariants` — 0/10; raw=PASS: all registered invariants have passing named evidence
