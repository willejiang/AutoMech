# KB sources and licenses

The maker2 knowledge base is deliberately weighted toward **our own authored format
and convention docs** (the highest-value content — they teach the exact output
contract the agents keep violating). Any third-party reference material added here
must be CC-licensed or public-domain; record it below with its license.

## Authored by this project (no external license — first-party docs)

- `corpus/manager/01_output_format.md` — the parts-list + placement contract.
- `corpus/manager/02_dof_and_motion.md` — dof rules, coaxial/structure rules.
- `corpus/manager/03_interface_frames.md` — the `<site>` / frames_realized contract.
- `corpus/manager/04_dimension_vocabulary.md` — canonical size_mm keys + gear math.
- `corpus/manager/09_curved_geometry_build123d.md` — loft/revolve/sweep/fillet for
  wings, fuselages, ducts and fairings. Every code block in it was executed against
  build123d 0.11.1; the quoted volumes are what it produced.
- `corpus/manager/golden_01_turntable.md` — hand-authored bearing+shaft+platter example.
- `corpus/manager/golden_02_gear_pair.md` — hand-authored meshing gear-pair example.
- `corpus/manager/golden_03_bearing_shaft.md` — hand-authored rotating-hardware atom.
- `corpus/manager/golden_04_escapement.md` — hand-authored escapement example.
- `corpus/boss/01_decomposition.md` — subassembly/seam/one-driver conventions.
- `corpus/worker/01_manifold_idioms.md` — CadQuery/OpenSCAD manifold-safe patterns.
- `corpus/evaluator/01_test_design.md` — physics-test design + failure taxonomy.
- `corpus/analyzer/golden_01_multibore_housing.md` — localized multi-bore housing diagnosis/repair case.

These are the format/convention docs the plan calls out as the actual compliance
gap; they carry no external license.

## Third-party reference material (add below with license — CC / public-domain ONLY)

None yet. When adding, cite source + license, e.g.:

- Wikipedia "Gear" / "Involute gear" / "Module (gears)" — CC BY-SA 4.0.
- LibreTexts Engineering — CC BY-NC-SA (check per-page).
- MIT OpenCourseWare 2.007 / 2.72 notes — CC BY-NC-SA 4.0.
- NPTEL Machine Design / Kinematics of Machines — check per-course license.
- Wikibooks Engineering Mechanics — CC BY-SA.

Do NOT ingest copyrighted textbooks (e.g. Shigley) — first-party docs + CC/PD
sources only.

## Your own material: `corpus_local/` (gitignored)

The rule above governs what this repository *ships*. It is not a limit on what you can
run locally. `maker2/kb/corpus_local/` mirrors `corpus/` — one directory per collection —
and `python -m maker2.kb.ingest` folds it into the same index, so a local doc is
retrieved exactly like a first-party one. It is gitignored, so it never enters the repo.

That split is the whole design: this is an open-source project, so the committed corpus
stays first-party and license-clean, while whatever *you* are allowed to read (a
CC-licensed course, a textbook you own, internal standards) stays on your machine under
your own terms. The directory and its README are created on first ingest.

Two things worth knowing before filling it:

- **Ingest reports the split** — `manager: 50 chunks (49 first-party + 1 local)` — and
  every chunk's source is prefixed `corpus:` or `corpus_local:`. When a local doc turns
  out to be wrong, that label is how you find it.
- **A wrong line is worse than a missing one.** It comes back to the agent looking
  authoritative, and it will build to it. This repo already carries
  `index/memory_manager.poisoned.bak` from exactly that, and cleaning that index meant
  discarding it wholesale. Prefer claims physics can contradict (a ratio, a stroke, a
  centre distance) over unfalsifiable ones.
