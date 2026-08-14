# Comfort v1 full benchmark backup

This branch is a data-backup branch for the complete Comfort v1 benchmark and its
supporting MJCF audits. It is intentionally not part of `main`.

## Baseline and authoritative reviewed results

- Source baseline: `ac59696c63ab16273c54a97bbe0f4711b53899f1`
- AutoMech: **860/1000**, **5/10 strict PASS**
- Codex: **390/1000**, **1/10 strict PASS**
- Claude Code: **310/1000**, **0/10 strict PASS**

The canonical reviewed report is `benchmark_results.md`, with structured results
under `benchmark_results/comfort_v1_20260811/`. Raw harness evidence and
reviewed/scorer-owned adjudication remain separate. In particular, Tasks 1 and 2
retain `score.pre_adjudication.json` and `adjudication.json` alongside the final
`score.json`.

`output/DETAILED_SCORE_RELEASE_NOTES.md` and `output/release_notes_current.md`
contain the superseded provisional AutoMech headline `810/1000, 6/10`. They are
preserved only as historical build output. They are not the current result and
must not override the canonical reviewed report or score JSON.

## Included data

- Final Comfort v1 reports, source runs, portable bundles, rescoring work,
  collision audits, trajectories, meshes, models, and media.
- Claude Code and Codex staging, replay, strict scoring, geometry, realization,
  and verification outputs.
- Task-05 independent-value representation ablation evidence.
- MJCF precursor and frozen-rerun evidence used to establish the benchmark.
- Release snapshots, downloaded verification copies, checksums, and helper
  scripts used to build or validate the published evidence.

`benchmark_full_result_manifest.sha256` records SHA-256 for every allowlisted
payload file. The manifest excludes itself and this explanatory file.

## Explicit exclusions

This branch does not include credentials or local development state (`.env`,
`.claude`, virtual environments, `node_modules`, or worker-local configuration).
It also excludes unrelated smoke logs, root scratch files, generic product/P51
runs, Chrono experiments, manual correction runs, uploads, and other output not
used by this benchmark.

Some immutable raw evidence contains Windows absolute paths identifying the
machine-local run directory. These strings are provenance metadata, not
credentials. They are intentionally preserved so the raw evidence and published
archive hashes remain unchanged.
