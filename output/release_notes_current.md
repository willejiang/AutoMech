# v0.5.0 — MJCF agent and Comfort benchmark

This release adds agent-authored MJCF topology compilation, deterministic anti-cheating validation, and a reproducible strict benchmark scorer for mechanically realized CAD.

## Strict benchmark results

| Method | Strict score | Strict passes |
|---|---:|---:|
| AutoMech | **860/1000** | **5/10** |
| Codex | **390/1000** | **1/10** |
| Claude Code | **310/1000** | **0/10** |

The external scores were regenerated from scorer-owned source execution/replay, strict exact-solid geometry, and MJCF collision/constraint audits. Every task now has all 13 criterion scores: Execution 10; Assembly 5/7/3; Geometry 5/10; Physics-ready 5/5/10; Functional 5/10/15/10. Failed strict prerequisites gate downstream credit while retaining raw diagnostics.

Earlier provisional Claude Code `640/1000` and Codex `510/1000` aggregates remain withdrawn; they used pre-strict staged evidence and are not current results.

See the current [unified benchmark report](https://github.com/willejiang/AutoMech/blob/main/benchmark_results.md) and `external-strict-detailed-scores.zip` for per-task JSON/Markdown evidence.

## Representation ablation pilot

A temporary process-local task-05 experiment removed shared executable-dependency prompting without changing normal CLI or frontend behavior. It fell from `100/PASS` to `25/FAIL`, with 17 exact non-exempt conflicts and evaluated-only `Q_dep=0.88`. It is a one-sample pilot and does not alter the ten-task aggregate.

## Validation

- All benchmark scorer goldens pass, including strict external scoring, tamper rejection, gating, aggregate completeness, and deterministic regeneration.
- All ten corrected AutoMech portable submissions validate at `860/1000`, `5/10`, with zero UNKNOWN checks. Task 1 fails fixed-axis motion; Task 2 fails finite-effort physical operation.
- External strict regeneration reproduces Claude Code `310/1000, 0/10` and Codex `390/1000, 1/10`.
- Release evidence archives remain separately hash-addressed.



The corrected portable bundle, including all ten MP4s, is also tracked directly under `output/comfort_benchmark_v1_rescored_portable/` on `main`. Corrected ZIP SHA-256: `644ad2fa2b86a0811e51abf64bfb24c875d8dba5824cd19459996eae875d407a`.
