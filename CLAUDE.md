# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository shape

PhysCAD Researcher generates CAD, evaluates it under a task, and feeds visual/physics failures back into generation. It contains two related generation paths:

- `worker/` is the cadam React 19/TanStack Start web app. It generates OpenSCAD in the browser, persists through Supabase, and also contains the UI/API routes for `maker2` runs.
- `orchestrator/` is the original outer loop: direct LLM CAD generation -> native OpenSCAD render -> six-view VLM gate -> VLM-authored manifest -> evaluator. Its seam with the evaluator is a design directory containing `manifest.json` plus `.scad`/`.stl`; the evaluator returns `out/result.json`.
- `maker2/` is the newer local articulated-CAD pipeline. `maker2/run.py` drives manager/worker/assembler/judger refinement and emits URDF/MJCF, meshes, views, and physics results. `--hierarchy` uses a boss to define typed cross-subassembly seams, managers to build subassemblies independently, and deterministic assembly/gates to solve placement.
- `evaluator/` supports both paths. The original path splits GPU simulation inside an Isaac Sim container from host-side VLM analysis through `sim_result.json`. `maker2/physics.py` instead imports the evaluator's CPU PyBullet/MuJoCo backend code directly; maker2 does not require Isaac Sim.
- `pipeline_context.py` contains the shared pipeline description injected into agent prompts.

Read `maker2/PIPELINE.md` before changing the hierarchical path. Its central invariant is that the boss describes connection topology, not placement coordinates. Deterministic gates route an internal subassembly fault back to that manager and a cross-subassembly/interface fault back to the boss. `DESIGN_LOOP.md` and `evaluator/ARCHITECTURE.md` describe the older cadam/Isaac flow and its host/container file boundary.

## Commands

Run Python module commands from the repository root unless stated otherwise.

### maker2 (Python)

```bash
python -m pip install -r maker2/requirements.txt
python -m pip install -r evaluator/requirements.txt  # CPU physics and media dependencies
python -m maker2.run "a hand-cranked gear reducer" --json --physics
python -m maker2.run "a two-stage gear reducer" --hierarchy --kb --deep-think --json
```

The default geometry backend is CadQuery. OpenSCAD is a legacy fallback and requires `OPENSCAD_BIN`. LLM calls use an OpenAI-compatible gateway configured by `FREECAD_AI_BASE_URL` (default `:8313`).

The Python tests are executable golden scripts, not a pytest suite. Run one with:

```bash
python -m maker2.tests.golden_slvs_cross_sub
```

Other important suites use the same form: `golden_mate_solver_roundtrip`, `golden_mjcf_roundtrip`, `golden_two_gears`, `golden_assembly_analyzer_tools`, `golden_assembly_repair`, and `isolation_scad_table`. There is no aggregate runner or configured Python linter.

### worker web app (run from `worker/`)

```bash
npm install
npm run dev
npm run build
npm run typecheck
npm run lint
npm run format
npm run preview
```

Node must be `^20.19.0 || >=22.12.0` with npm `>=10`. There is no npm test script; tests use Node's built-in runner. From the repository root, run one test file with:

```bash
node --experimental-strip-types --test worker/shared/parametricParts.test.ts
```

The interactive app requires local Supabase plus `worker/.env`/`.env.local`; generation has no dependency-free headless mode. The orchestrator's direct worker path does not require Supabase.

### original orchestrator/evaluator path

From `orchestrator/`:

```bash
python automech_loop.py --task "quarter-car suspension that clears a 10cm curb" --dry-run --max-iters 3
python render_views.py ../worker/benchmarks/07-bevel-gear-drive.scad
python visual_gate.py --task "a bevel gear drive" path/to/views/*.png
```

`--dry-run` stubs only Isaac simulation; generation and visual gates still require `orchestrator/.env` and the native OpenSCAD CLI. A live evaluator run occurs on the GPU host:

```bash
cd evaluator
./evaluate.sh /data/physcad/<design-dir>
python3 loop.py --urdf <robot.urdf> --asset-root <assets> --task "stand still" --workdir <dir> --max-iters 4
```

Isaac Sim/Isaac Lab are external installations, not repository dependencies. Host code and the container exchange files through the shared mount (`/data/physcad` on the host is `/work` in the container).

## worker-specific rules

Rules under `worker/.cursor/rules/` are part of the repository contract:

- TypeScript uses 2-space indentation, single quotes, semicolons, and 100-character lines. Run `npm run typecheck` after type changes; run `deno check index.ts` in each modified Supabase Edge Function directory.
- Change database definitions in `worker/supabase/schemas/`; do not hand-author migrations. Generate them locally with `supabase db diff -f <name>`, then regenerate `worker/shared/database.ts` with `supabase gen types typescript --local`. Do not edit that generated file manually.
- Never run `supabase db push`, `supabase db pull`, or any `supabase functions deploy` command. Supabase development and testing are local-only; deployment is human-controlled.

The only other `CLAUDE.md` is inside vendored `orchestrator/oscad-libs/BOSL2/` and applies only when modifying that library.

## Artifacts and validation semantics

Generated run data belongs under ignored output/run directories, not alongside source. The curated `maker2/kb/index/` collections are committed, but `memory_*` and `analyzer` indices are machine-local and ignored.

The project's visual and physical checks answer different questions: the six-view maker gate rejects malformed geometry before simulation, while the evaluator judges task behavior. In the original Isaac path, camera/VLM evidence is treated as authoritative because articulation-root numeric pose metrics have produced false passes. Preserve the `manifest.json`/`sim_result.json`/`result.json` file contracts when changing process boundaries.
