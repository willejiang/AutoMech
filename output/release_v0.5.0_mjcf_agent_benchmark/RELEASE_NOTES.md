# v0.5.0 — MJCF agent and Comfort benchmark

This release adds agent-authored MJCF topology compilation, deterministic anti-cheating validation, and a reproducible strict benchmark scorer for mechanically realized CAD.

## Highlights

- Agent-authored MJCF body trees, joints, equalities, support topology, and exact-pair contact decisions.
- Immutable MJCF facts, restricted compiler ABI, deterministic acceptance gates, equality provenance, collision coverage, and contact-on/off transmission probes.
- Fail-closed exact-solid geometry checks and stricter authored transmission/fit semantics.
- Strict JSON serialization across model, evaluator, metrics, and telemetry boundaries.
- Comfort v1 benchmark contract, portable submission adapters, scorer-owned replay, exact geometry audit, aggregation, and executable golden tests.
- README demonstrations for the successful four-planet reducer and horizontal slider-crank.
- GitHub Pages updates from `main` are included in the same release commit.

## Strict benchmark results

| Method | Result | Interpretation |
|---|---:|---|
| AutoMech | **810/1000; 6/10 strict PASS** | Independent cumulative execution, assembly, geometry, physics-ready, and functional audit |
| Codex | **1/10 strict PASS** | Only task 01 survives strict physical-realization checks |
| Claude Code | **0/10 strict PASS** | No task realizes geometry, collision semantics, active constraints, and scorer replay together |

Earlier provisional external aggregate scores are withdrawn and are not release headline results. Raw harness evidence remains separate from scorer-owned adjudication.

Task 07 is `100/PASS` after selected-trajectory closure replay. Task 08 remains `90/FAIL` only on strict pin-scale closure precision.

## Representation ablation pilot

A temporary process-local task-05 experiment removed prompt pressure toward shared executable dependencies without changing normal CLI or frontend behavior. It fell from the archived `100/PASS` baseline to `25/FAIL`, with 17 exact non-exempt solid conflicts and evaluated-only `Q_dep=0.88`. This is a one-sample case study, not a population estimate, and does not change the ten-task aggregate.

## Validation

- 19 benchmark/MJCF/planetary/support/fit golden modules passed before and after merging the latest `main`.
- All ten portable AutoMech submissions revalidated and rescored to `810/1000`, `6/10`, with zero UNKNOWN checks.
- Task 07 reproduced `100/PASS`; task 08 reproduced `90/FAIL`.
- Both GIFs validate as 640×480, 40 frames, 3.33 seconds.
- Release archives were manifest-checked, secret-scanned, and independently SHA-256 verified.

## Evidence assets

- `physcad-comfort-v1-automech-rescored-portable.zip` — ten AutoMech portable submissions and independent evidence.
- `physcad-comfort-v1-external-strict-audit.zip` — twenty Claude Code/Codex submissions with strict audits.
- `physcad-comfort-v1-representation-ablation-task05.zip` — complete temporary ablation evidence.
- `SHA256SUMS.txt` — archive checksums.

See [`benchmark_results.md`](https://github.com/willejiang/AutoMech/blob/v0.5.0-mjcf-agent-benchmark/benchmark_results.md) for the unified report.
