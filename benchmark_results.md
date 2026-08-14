# PhysCAD Comfort Benchmark v1 — Unified Results and External-Harness Comparison

## 1. Run identity

- Date: 2026-08-11
- Suite specification: `benchmark_results/comfort_v1_20260811/protocols/benchmark.md`
- Artifact root: `output/comfort_benchmark_v1_20260811`
- Code snapshot at suite start: branch `single-agent/free-control-functions`, commit `d409d782129de4ab842675f59246d49db034b69f`, plus the uncommitted benchmark metrics/telemetry changes described in the session
- Model: `gpt-5.6-sol` through `local_gateway`
- Pipeline: PhysCAD single-agent text-to-CAD → KinematicModel → agent-authored MJCF → MuJoCo evaluator
- Per-task budget: `max-iters=3`
- Physics engine: MuJoCo
- Project artifact/MJCF cache: fresh per-task namespace; all observed MJCF compiler lookups were misses
- Provider prompt cache: used within and/or across API requests and reported separately; no previous `machine.py`, CAD/IR, accepted MJCF, scenario, or task artifact was reused

## 2. Headline results

| Metric | Result |
|---|---:|
| Raw Harness Final Success@3 | 6/10 (60%) |
| Historical manual adjudication (superseded) | 8/10 (80%) |
| Reviewed cumulative five-level success | 5/10 (50%) |
| Reviewed suite score | 860/1000 (mean 86/100) |
| Independent UNKNOWN checks | 0 |
| Raw Operational Pass@1 | 0/10 |
| Mean CAD iterations | 1.3 |
| Mean wall-clock runtime | 690.35 s (11.51 min) |
| Minimum / maximum runtime | 472.83 / 1057.46 s |
| LLM requests | 224 |
| Total input tokens | 8,123,416 |
| Total output tokens | 369,646 |
| Provider cache-read input tokens | 7,255,704 |
| Total reported tokens | 8,493,062 |
| Agent tool calls | 1,313 |
| Tool errors | 0 |
| MJCF compiler candidates | 31 |
| MJCF compiler submissions | 31 |
| MJCF project-cache hits / misses | 0 / 11 |
| API cost | unavailable; no price was inferred |

`Pass@1=0/10` uses the strict operational definition: the first CAD candidate and the first MJCF compiler source/candidate must pass without replacement or refinement. Several tasks produced correct CAD on iteration 0 but needed MJCF compiler repair, so they are Final@3 PASS but not operational Pass@1.

The independent result is a 2026-08-12 offline rescore of the archived final folders. It reexecuted every saved `machine.py`, required a nonempty part set, replayed every saved MJCF with scorer-owned controls, ran exact final-pose Manifold solid intersections, and recomputed named invariants from topology and trajectories. No generation model was called. Reviewed corrections remain separate and hash-linked: Task 1 uses the designated selected trajectory to reject a fixed-axis false pass, while Task 2 reports its corrected direct-qpos trajectory only as kinematic evidence and retains the designated finite-effort failure. Its five levels are cumulative: a geometry failure blocks physics and function credit. This independent result supersedes the earlier 8/10 manual-adjudication headline for terminal benchmark reporting.

The telemetry field `cold_run.qualified=false` for all runs is too strict for comparison: it currently treats any provider prompt-cache read as disqualifying. The project-controlled artifact/MJCF cache was cold (`0` hits). For cross-harness comparison, report this suite as **project-cold with provider cache usage disclosed**, not as warm artifact reuse.

## 3. Per-task results

| # | Task | Iterations | Runtime | Raw | Adjudicated mechanical | Key evidence | LLM requests / tokens | Tool calls / MJCF candidates |
|---:|---|---:|---:|---|---|---|---:|---:|
| 1 | Single-stage 4:1 reducer | 1 | 10.8 min | PASS | FAIL | equality reports 4:1, but shaft axes orbit by 45.00/44.86 mm around an erroneous world-origin hinge | 17 / 439,532 | 84 / 2 |
| 2 | Two-stage 9:1 reducer | 2 | 11.7 min | FAIL | FAIL | structure and corrected 9:1 kinematics pass; designated finite-effort servo moves only 0.0036 rad input / 0.0002 rad output | 21 / 557,729 | 103 / 2 |
| 3 | 1:1 idler reverser | 2 | 8.5 min | PASS | PASS | input/output same direction; ratio 1.0002; 5/5 | 15 / 426,740 | 109 / 1 |
| 4 | Openwork 12:1 clock | 1 | 11.2 min | PASS | PASS | minute 12.0 rad; hour 1.0 rad; independent coaxial hands; 5/5 | 18 / 605,694 | 120 / 2 |
| 5 | Three-planet 4:1 reducer | 1 | 12.2 min | PASS | PASS | carrier gain 0.25; all 3 planets carried and locally spinning; 6/6 | 24 / 1,255,551 | 156 / 4 |
| 6 | Four-planet 4:1 reducer | 1 | 16.4 min | PASS | PASS | 4.0:1; all 4 planets present and moving; 7/7 | 21 / 1,202,544 | 202 / 3 |
| 7 | Horizontal slider-crank | 1 | 7.9 min | PASS | PASS | crank 12.16 rad; slider span 24.006 mm; both directions; 3/3 | 16 / 365,958 | 66 / 2 |
| 8 | Vertical piston pump | 2 | 17.7 min | FAIL | PASS | crank 12.36 rad; crosshead/pump rod/piston vertical span 20.368 mm; 26 reversals; no detach/explosion | 43 / 1,605,152 | 235 / 6 |
| 9 | Open pumpjack | 1 | 9.4 min | FAIL | FAIL | video shows structure flying apart/large global drift; input only 1.04 rad; benchmark required at least one revolution | 21 / 810,658 | 108 / 4 |
| 10 | Wind-rotor pump | 1 | 9.6 min | FAIL | FAIL | video shows structure flying apart; rotor input only 0.0035 rad; output 0.177 mm, no valid reciprocation | 28 / 1,223,504 | 130 / 5 |

## 4. Independent offline rescore

| # | Task | Score | Exec. | Asm. | Geo. | Phys. | Func. | Independent verdict |
|---:|---|---:|---|---|---|---|---|---|
| 1 | Single-stage 4:1 reducer | 90 | PASS | PASS | PASS | PASS | FAIL | FAIL |
| 2 | Two-stage 9:1 reducer | 90 | PASS | PASS | PASS | PASS | FAIL | FAIL |
| 3 | 1:1 idler reverser | 100 | PASS | PASS | PASS | PASS | PASS | PASS |
| 4 | Openwork 12:1 clock | 100 | PASS | PASS | PASS | PASS | PASS | PASS |
| 5 | Three-planet 4:1 reducer | 100 | PASS | PASS | PASS | PASS | PASS | PASS |
| 6 | Four-planet 4:1 reducer | 100 | PASS | PASS | PASS | PASS | PASS | PASS |
| 7 | Horizontal slider-crank | 100 | PASS | PASS | PASS | PASS | PASS | PASS |
| 8 | Vertical piston pump | 90 | PASS | PASS | PASS | PASS | FAIL | FAIL |
| 9 | Open pumpjack | 60 | PASS | PASS | PASS | PASS | FAIL | FAIL |
| 10 | Wind-rotor pump | 30 | PASS | PASS | FAIL | blocked | blocked | FAIL |

All ten saved source programs reexecuted successfully with nonempty part sets (11--24 parts), and all ten MJCF files loaded, initialized, and remained finite under scorer-owned replay. There are no remaining `UNKNOWN` checks.

Exact non-exempt geometry conflicts:

- Task 2: `intermediate_high_bearing` vs `intermediate_stage2_pinion`, $64.342984\,\mathrm{mm}^3$; and `intermediate_high_bearing` vs `output_gear`, $0.156890\,\mathrm{mm}^3$.
- Task 10: `base` vs `connecting_rod`, $169.689671\,\mathrm{mm}^3$.

Direct authored rigid mounts, press fits, running bearings, pins, and declared ideal gear meshes are geometric exemptions only for their exact pairs. Generic MJCF collision exclusions are not geometric exemptions.

Functional failures with clear geometry:

- Task 1 is FAIL despite its exact 4:1 equality: selected body trajectories show the input/output shaft centers orbiting approximately $45.00/44.86\,\mathrm{mm}$ because both hinge axes were lowered at the world origin. Numeric ratio agreement cannot override this visible fixed-axis failure.
- Task 2 is structurally valid and the corrected direct-qpos replay measures 9.0015:1, but its designated finite-effort servo run moves only $0.0036\,\mathrm{rad}$ at the input and $0.0002\,\mathrm{rad}$ at the output. It receives 90/100; idealized kinematics do not restore the failed physical invariant.
- Task 7 is PASS after replaying the selected final trajectory through its own MJCF: stable `slider_end_pin_connect` residual is at most $0.0124\,\mathrm{mm}$, below $2\%$ of the approximately $4\,\mathrm{mm}$ wrist-pin diameter ($0.08\,\mathrm{mm}$). The earlier $4.1395\,\mathrm{mm}$ value came from a different scorer replay and is withdrawn.
- Task 8 has correct visible periodic motion and output span, but the selected final trajectory gives a persistent `wrist_pin_small_end_connect` residual of approximately $0.46$--$0.51\,\mathrm{mm}$. That exceeds both $2\%$ of the $4\,\mathrm{mm}$ wrist-pin diameter ($0.08\,\mathrm{mm}$) and the authored $0.1\,\mathrm{mm}$ diametral running clearance. Its only failed criterion is strict closure precision; no visible detachment or output-motion failure is claimed.
- Task 9: designated input travel is only $1.04353\,\mathrm{rad}<2\pi$, so cumulative functional checks fail rather than remaining unknown.

Authoritative rescore artifacts:

- portable evidence: `output/comfort_benchmark_v1_rescored_portable/`
- per-task scores: `output/comfort_benchmark_v1_rescored_scores/<task>/score.json`
- aggregate: `output/comfort_benchmark_v1_rescored_scores/suite_score.json`
- source/replay/geometry work evidence: `output/comfort_benchmark_v1_rescore_work/`

## 5. Temporary representation ablation pilot

A one-task pilot tested the executable-dependency representation claim on the previously passing `05_three_planet_4to1` task. It used the same frozen task prompt, `gpt-5.6-sol`, MuJoCo evaluator, agent-authored MJCF compiler, and `max-iters=3`, while a process-local prompt overlay prohibited shared root parameters and formula-derived dependent dimensions/placements. Normal production prompts, source files, Settings, and CLI behavior were not modified.

| Arm | Score | Verdict | Iterations | Runtime | Exact non-exempt conflicts |
|---|---:|---|---:|---:|---:|
| Executable dependencies (archived task-05 baseline) | 100/100 | PASS | 1 | 731.576 s | 0 |
| Independent local values (fresh pilot) | 25/100 | FAIL | 3 | 1248.293 s | 17 |

Independent-values five-level result:

| Layer | Weight | Result | Evidence |
|---|---:|---|---|
| Execution | 10 | PASS, 10/10 | scorer-owned source reexecution produced 23 nonempty solids; rejected MJCF candidate compiled with finite $n_q=n_v=5$ |
| Assembly | 15 | PASS, 15/15 | final iteration passed connectivity and contained the required fixed ring, sun input, carrier output, and three dedicated planet hinges |
| Geometry | 15 | FAIL, 0/15 | Manifold exact-solid audit measured 17 non-exempt conflicts with zero unavailable meshes |
| Physics-ready | 20 | FAIL, 0/20 | retained contact materially stalled the mechanism, so policy v5 rejected the MJCF compiler |
| Functional | 40 | FAIL, 0/40 | no accepted trajectory existed to establish output ratio, orbit, local spin, or fixed-ring invariants |

The retained-contact probe measured `0.001541 rad` input travel versus `0.977454 rad` with contact disabled: only **0.158%** of the no-contact control. Important exact conflicts included `carrier` vs `fixed_ring_gear` ($2312.995\,\mathrm{mm}^3$), `carrier` vs `input_shaft` ($276.346\,\mathrm{mm}^3$), and `fixed_ring_gear` vs `input_shaft` ($150.734\,\mathrm{mm}^3$), plus repeated ring/pin, ring/bushing, and carrier/bushing intersections.

The evaluated-only dependency-consistency score was

$$
Q_{\mathrm{dep}}=1-\frac{3}{25}=0.88.
$$

The three failed evaluated dependencies were: no explicit ring–planet mesh relations (`0/3`), an authored input-bearing press fit realized as `+0.2 mm` diametral clearance, and loss of mobility with physical contact enabled. Five trajectory-dependent checks were explicitly `UNAVAILABLE` and excluded from the denominator because no MJCF compiler was accepted. Thus the relatively high static $Q_{\mathrm{dep}}$ does not imply a functional mechanism: independently repeated literals preserved much of the tooth arithmetic and center placement while failing physical realization.

This pilot is reported separately and does **not** alter the reviewed ten-task aggregate (`860/1000`, `5/10`). It is one stochastic case-study result, not a population estimate or a statistically significant ablation. No additional favorable-attempt rerun was performed. Both the baseline and pilot observed provider/project prompt-cache activity, so neither is claimed as a strict cold-provider run.

Isolation verification:

- production source hashes before/after: identical;
- production `SINGLE_AGENT_SYSTEM` hash before/after: identical;
- overlay lifetime: experiment Python process only;
- all experiment code and artifacts: ignored `output/representation_ablation_05_three_planet_4to1/`.

Artifacts:

- report: `output/representation_ablation_05_three_planet_4to1/pilot_report.md`
- five-level score: `output/representation_ablation_05_three_planet_4to1/score.json`
- dependency metrics: `output/representation_ablation_05_three_planet_4to1/dependency_metrics.json`
- exact geometry audit: `output/representation_ablation_05_three_planet_4to1/geometry.json`
- baseline comparison: `output/representation_ablation_05_three_planet_4to1/comparison.json`
- isolation manifest: `output/representation_ablation_05_three_planet_4to1/experiment_manifest.after.json`

## 6. Three-method external-harness comparison

The frozen Comfort v1 prompts were also run independently through Claude Code and Codex harnesses, then normalized and audited with the same strict mechanical standard. Raw submitted artifacts remain separate from scorer-owned source reexecution, exact-solid geometry, collision coverage, constraint binding, and replay evidence.

### 6.1 Headline

| Method | Independent score | Strict five-level success | Interpretation |
|---|---:|---:|---|
| AutoMech | **860/1000** | **5/10** | Full CAD/IR/MJCF pipeline; five tasks survive cumulative execution, assembly, geometry, physics, and functional gates |
| Codex | **390/1000 strict** | **1/10** | Task 01 survives; tasks 02–10 fail strict collision or geometry-realization requirements |
| Claude Code | **310/1000 strict** | **0/10** | No task realizes static geometry, collision semantics, active constraints, and independent replay together |

The earlier provisional external scores—Claude Code `640/1000, 4/10` and Codex `510/1000, 3/10`—are withdrawn. They were inflated by schema-normalization, zero-scan geometry, incomplete collision coverage, trust in submitted trajectories, and inactive-constraint loopholes. A task counts as a strict PASS only when the CAD geometry, collision model, active mechanism constraints, and scorer-owned replay all implement the declared semantics.

Three claims must remain distinct:

1. **Idealized kinematics:** a requested ratio or stroke equation can be replayed.
2. **Declared assembly semantics:** JSON names mounts, bearings, pins, joints, or closure paths.
3. **Strict mechanical realization:** the CAD, collision model, active constraints, and independent replay physically realize those declarations.

The strict success column reports only claim 3. Direct kinematic equality success does not prove real tooth-contact load capacity.

The current strict numeric aggregates are newly regenerated from hash-linked scorer-owned source execution and replay, strict exact-solid geometry, and MJCF collision/constraint audits. They are not revisions of the withdrawn provisional score files.

<!-- EXTERNAL_STRICT_DETAIL_START -->
### Detailed strict criterion scores

Cells show awarded points. Maxima are `E=10`; Assembly `model/roles/I-O=5/7/3`; Geometry `mesh/conflicts=5/10`; Physics `shape/alignment/finite=5/5/10`; Functional `input/propagation/output/invariants=5/10/15/10`. `G` means the criterion had raw diagnostic evidence but received zero points because a strict prerequisite gate was closed.

#### claude-code

Strict aggregate: **310/1000; 0/10 PASS**.

| # | Task | E | A M/R/I | G M/C | P S/A/F | F I/P/O/V | F total | Total | Strict | Blocker |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 01_single_stage_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Five non-exempt conflicts and most links are not connected to the base. |
| 2 | 02_two_stage_9to1 | 10 | 5/7/3 | 5/10 | G/G/G | G/G/G/G | 0/40 | 40 | FAIL | Moving collisions disabled, fragmented gears, and eight detached links. |
| 3 | 03_idler_reverser_1to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Moving collisions disabled and three fragmented gear compounds. |
| 4 | 04_openwork_clock_12to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Two undeclared material clashes and all moving collision disabled. |
| 5 | 05_three_planet_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Six conflicts and unverified ring-tooth realization. |
| 6 | 06_four_planet_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Seventeen geometry/unavailable failures and invalid transmission lowering. |
| 7 | 07_horizontal_slider_crank | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Three conflicts and five detached physical links. |
| 8 | 08_vertical_piston_pump | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Two conflicts, invalid closure semantics, and fourteen detached links. |
| 9 | 09_open_pumpjack | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Two conflicts and ten detached physical links. |
| 10 | 10_wind_rotor_pump | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | No authored transmission, inactive closure, stationary replay output, and nine detached links. |

#### codex

Strict aggregate: **390/1000; 1/10 PASS**.

| # | Task | E | A M/R/I | G M/C | P S/A/F | F I/P/O/V | F total | Total | Strict | Blocker |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 01_single_stage_4to1 | 10 | 5/7/3 | 5/10 | 5/5/10 | 5/10/15/10 | 40/40 | 100 | PASS | Source, geometry, one input actuator, ratio binding, collision proxies, and replay pass. |
| 2 | 02_two_stage_9to1 | 10 | 5/7/3 | 5/10 | G/G/G | G/G/G/G | 0/40 | 40 | FAIL | Incomplete collision coverage and geometrically detached declared assembly. |
| 3 | 03_idler_reverser_1to1 | 10 | 5/7/3 | 5/10 | G/G/G | G/G/G/G | 0/40 | 40 | FAIL | Incomplete gear/crank collision coverage and detached physical islands. |
| 4 | 04_openwork_clock_12to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Invalid bearing solids and incomplete realization/collision semantics. |
| 5 | 05_three_planet_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Non-exempt conflict, detached structure, and unverified integrated ring teeth. |
| 6 | 06_four_planet_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Non-exempt conflict, detached structure, and unverified integrated ring teeth. |
| 7 | 07_horizontal_slider_crank | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Collision conflicts and a persistent geometrically detached link. |
| 8 | 08_vertical_piston_pump | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Seven non-exempt conflicts and eight detached physical links. |
| 9 | 09_open_pumpjack | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Eleven non-exempt conflicts and eleven detached physical links. |
| 10 | 10_wind_rotor_pump | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Eight conflicts, incomplete closure/transmission semantics, and eleven detached links. |
<!-- EXTERNAL_STRICT_DETAIL_END -->

### 6.2 Codex strict results

| # | Task | Strict | Main reason |
|---:|---|---|---|
| 1 | Single-stage 4:1 reducer | **PASS** | Source, geometry, one input actuator, ratio binding, collision proxies, and replay pass |
| 2 | Two-stage 9:1 reducer | FAIL | Ideal 9:1 passes, but gear/bearing/crank collision coverage is incomplete and much of the declared assembly is geometrically detached |
| 3 | 1:1 idler reverser | FAIL | Ideal 1:1 passes, but gear/crank collision coverage is incomplete and the physical assembly has detached islands |
| 4 | Openwork 12:1 clock | FAIL | Four invalid/non-watertight bearing solids plus incomplete realization/collision semantics |
| 5 | Three-planet 4:1 | FAIL | Non-exempt conflict, detached declared structure, and ring geometry is not verified as an integrated toothed solid |
| 6 | Four-planet 4:1 | FAIL | Non-exempt conflict, detached declared structure, and ring geometry is not verified as an integrated toothed solid |
| 7 | Horizontal slider-crank | FAIL | Collision conflicts and one persistent geometrically detached link |
| 8 | Vertical piston pump | FAIL | Seven non-exempt conflicts and eight detached physical links under the conservative declared-relation audit |
| 9 | Open pumpjack | FAIL | Eleven non-exempt conflicts and eleven detached physical links; the base is visibly separate from the mechanism group |
| 10 | Wind-rotor pump | FAIL | Eight conflicts, incomplete closure/transmission semantics, and eleven detached physical links |

Codex task 01 remains an idealized collision-complete kinematic PASS; it does not establish real tooth-contact load capacity.

### 6.3 Claude Code strict results

| # | Task | Strict | Main reason |
|---:|---|---|---|
| 1 | Single-stage 4:1 reducer | FAIL | Five non-exempt conflicts and most physical links are not geometrically connected to the base |
| 2 | Two-stage 9:1 reducer | FAIL | Ideal 9:1 passes, but moving collisions are disabled; gear objects are fragmented hub-plus-tooth compounds and eight links remain detached |
| 3 | 1:1 idler reverser | FAIL | Ideal 1:1 passes, but moving collisions are disabled and all three gear objects are fragmented multi-solid compounds |
| 4 | Openwork 12:1 clock | FAIL | Two undeclared material clashes and all moving collision disabled; geometry is otherwise conservatively connected |
| 5 | Three-planet 4:1 | FAIL | Six conflicts; several interfaces need the conservative 10 mm allowance and ring-tooth realization is not verified |
| 6 | Four-planet 4:1 | FAIL | Seventeen geometry/unavailable failures, invalid transmission lowering, and fragmented multi-solid gears |
| 7 | Horizontal slider-crank | FAIL | Three conflicts and five physical links remain detached even under the conservative declared-relation audit |
| 8 | Vertical piston pump | FAIL | Two conflicts, invalid closure semantics, and fourteen detached physical links |
| 9 | Open pumpjack | FAIL | Two conflicts and ten detached physical links |
| 10 | Wind-rotor pump | FAIL | No authored transmission, inactive all-zero closure, stationary scorer-replay output, and nine detached physical links |

A fragmented multi-solid gear is not the same as a smooth disk. Some Claude Code outputs generate hubs and separate tooth boxes but keep them as an unfused compound. Those are reported as **fragmented tooth geometry**, not “no teeth.” Automated contour checks are supporting evidence and were cross-checked against source construction.

### 6.4 Comparison evidence

- external protocol: `benchmark_results/comfort_v1_20260811/protocols/external_harness_benchmark.md`
- external run roots: `submitted Codex run archived in the external strict release asset` and `submitted Claude Code run archived in the external strict release asset`
- strict collision/constraint audits: `output/external_benchmark_strict/<method>/<task>/strict/`
- declared-geometry realization: `output/external_declared_realization_audit/<method>/<task>.json`
- scorer-owned source/replay work: `output/external_benchmark_work/<method>/<task>/`
- comparison/orbit videos: `GitHub release assets (see release notes)`
- canonical detailed strict scores: `benchmark_results/comfort_v1_20260811/external_scores/<method>/`
- withdrawn provisional scores retained for provenance only: `output/external_benchmark_scores/`

## 7. Historical adjudication details

### Task 2: structure and kinematics pass; finite-effort physics fails

The accepted MJCF contains exact two-stage gear equalities. The original scenario selected finite-effort `servo`, while the corresponding successful single-stage reducer selected `direct_qpos`. The raw test moved only `0.0036 rad` and therefore could not measure the ratio.

An evaluator-only corrected retest reused the exact same CAD, KinematicModel and `model.mjcf`, with no agent or compiler call:

- drive method: `direct_qpos`;
- coordinates: `input_spin → intermediate_spin → output_spin`;
- input travel: `12.0 rad`;
- output travel: `1.3331 rad`;
- observed ratio: `9.0015:1`;
- downstream: `6/6`;
- exploded: false;
- stability: PASS.

Artifacts:

- corrected result: `02_two_stage_9to1/.../physics/mujoco_corrected_direct_qpos/sim_result.json`
- corrected video: `02_two_stage_9to1/.../physics/mujoco_corrected_direct_qpos/model.mp4`

Raw harness and reviewed mechanical verdict both remain FAIL. The corrected direct-qpos result proves the authored 9:1 kinematic transmission, but cannot replace the designated finite-effort physical test; Task 2 therefore receives 90/100 with the strict physical invariant failed.

### Task 8: periodic output falsely rejected by endpoint displacement

The mechanism is mechanically correct in the recorded trajectory:

- `crosshead_vertical_slide` span: `20.368 mm`;
- reversals: `26`;
- crank input: `12.3644 rad`;
- crosshead, pump rod and pump piston X/Y span: `0`;
- each has Z span: `20.368 mm`;
- crankshaft world position span: `0`;
- rigid carrying remains intact and the linkage stays visually connected, although the independent pin-scale closure audit measures a persistent approximately $0.46$--$0.51\,\mathrm{mm}$ soft-constraint offset;
- no active contact, detachment or explosion.

The functional metric correctly used `max(output)-min(output)` and passed. The generic runner used `abs(output_end-output_start)` and obtained about `0.7 mm`, which is naturally small for a periodic output ending near its starting phase. It therefore set `output_reached=false` and produced a raw false failure.

Raw harness result remains FAIL. The endpoint-based runner failure is an evaluator error, while the independent scorer separately records a strict closure-precision failure; the visible reciprocating output itself passes.

### Tasks 9 and 10: confirmed mechanical failures

The videos show the assemblies flying apart or undergoing unacceptable global motion. Numeric `exploded=false` is a stability-threshold false negative and does not override visual/body-trajectory evidence.

- Task 9 additionally failed the benchmark input requirement: only `1.0438 rad`, below one full revolution.
- Task 10 input moved only `0.0035 rad`; output motion was negligible and one-sided.

Both remain mechanical FAIL.

## 8. Evidence policy used

- Raw harness results are never silently changed.
- Corrected or adjudicated results are reported in a separate column.
- Body/joint trajectories and mechanical invariants outrank natural-language summaries.
- Video evidence can reject a numeric false pass when a machine visibly detaches or flies apart.
- `direct_qpos` results prove exact kinematic conformance only; they do not prove finite-effort load capacity or real tooth-contact performance.
- Support-derivative changes do not count toward normal Physics-Ready or Functional success.

## 9. Known evaluator/telemetry defects exposed by this suite

1. Scenario test-mode selection can choose servo for an exact-equality gear train that should use an exact kinematic fixture.
2. Generic `output_reached` uses endpoint displacement and falsely rejects periodic reciprocation.
3. `exploded=false` can miss visually obvious structure detachment/global drift.
4. Pass diagnosis often reports `fault_domain=evaluator` with `insufficient_verified_evidence`, even on successful runs; pass rows should use `none` rather than a failure-domain placeholder.
5. Provider prompt-cache use is currently conflated with project artifact-cache reuse in `cold_run.qualified`.
6. MJCF agent compiler investigation cost is high: 1,313 tool calls and 31 candidates for 10 tasks.

## 10. Reproducibility files

- prompts and scoring: `benchmark_results/comfort_v1_20260811/protocols/benchmark.md`
- raw suite summary: `output/comfort_benchmark_v1_20260811/suite_summary.json`
- telemetry aggregate: `output/comfort_benchmark_v1_20260811/telemetry_aggregate.json`
- per-task logs: `output/comfort_benchmark_v1_20260811/*.log`
- each final run directory contains `result.json`, `benchmark_metrics.json`, accepted MJCF, manifest, trajectory, frames and video.
