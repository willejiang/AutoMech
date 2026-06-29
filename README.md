# AutoMech-maker2.0

This is the 2.0 version of maker. 1.0 generated a complete CAD STL first and
then split it into modules to derive the URDF; 2.0 plans each module's design up
front and generates the final URDF directly.

A multi-agent CAD generator. Give it one line of natural language ("a 2-DOF
pan-tilt camera mount", "a large clock with the full gear train") and it
decomposes the product into parts, builds each part in FreeCAD, and assembles
them into a single articulated **URDF** with meshes -- ready for physics
simulation (e.g. Isaac Sim) and production.

This branch holds the standalone CAD-maker pipeline. It is *not* a FreeCAD
addon; it is an orchestrator that drives headless FreeCAD as a subprocess.

## How it works -- URDF-as-contract

One product prompt flows through three agent roles. The URDF is the contract
between them: the manager writes link+joint definitions up front, workers fill
in the geometry, and the judge grades the assembled result.

```
prompt -> MANAGER -> kinematic model (links + joints, written as model.urdf)
              |
              v  one empty meshes/<link>.stl scaffolded per link
          WORKERS (parallel) -> generate FreeCAD Python -> headless freecadcmd -> fill each STL
              |
              v  yourdfpy + trimesh assemble + render
          JUDGER (optional) -> 6 views + summary -> {pass, reasons, suggestions}
              |
        fail  v  feed suggestions back to MANAGER, regenerate (<= max-iterations)
        pass  -> done
```

- **Manager** decomposes the prompt into a kinematic tree: every distinct rigid
  part is a *link*; every connection is a *joint* (fixed / revolute / prismatic /
  continuous). It owns all spatial layout and writes `model.urdf`.
- **Workers** run in a thread pool. Each gets ONE link's brief, emits FreeCAD
  Python, runs it via `freecadcmd`, and writes that link's mesh. Workers never
  see siblings -- they build in the part's local frame only.
- **Judger** (opt-in `--judge`) renders 6 orthographic views, hands them plus a
  text summary to an evaluator LLM, and gets a JSON verdict. On a fail, the
  suggestions are fed back to the manager for another pass.

## The units / origin contract

This is what lets parts built in isolation line up:

- `size_mm` is in **millimeters**; joint `xyz_m` is in **meters** (URDF meshes
  use `scale=0.001`).
- Each worker builds its part alone with the part's joint-attach point at the
  **local origin (0,0,0)**; the manager records where that origin sits in
  `origin_note`.
- The manager authors each joint's `xyz_m` as the vector from the parent origin
  to the child origin. Workers never position parts relative to each other.

## Install

Requires Python 3.14 (orchestrator) and FreeCAD 1.1 (bundled Python 3.11 for the
worker subprocess). Default `freecadcmd` path: `C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe`.

```bash
py -3.14 -m pip install -r requirements.txt   # yourdfpy, trimesh, pyglet<2
```

LLM access goes through an OpenAI-compatible gateway (default
`http://127.0.0.1:8313/v1`, model `claude-opus-4.8`). Set the key via env -- the
in-code default is a placeholder:

```bash
export FREECAD_AI_API_KEY=...        # also: FREECAD_AI_BASE_URL, FREECAD_AI_MODEL, FREECADCMD
```

## Usage

```bash
# Basic build
py -3.14 main.py --prompt "a simple table: a flat square top on a single central leg"

# Articulated, with interactive viewer
py -3.14 main.py --prompt "a 2-DOF pan-tilt camera mount" --show

# From a reference image (prompt becomes a hint)
py -3.14 main.py --prompt "a desk lamp" --image lamp.png

# Generate -> judge -> refine loop, up to 3 passes
py -3.14 main.py --prompt "a large clock with the full gear train" --judge --max-iterations 3
```

Flags: `--out` (default `output/`), `--model`, `--max-workers`, `--retries`,
`--allow-partial`, `--no-viz`, `--show`, `--judge`, `--max-iterations`.

## Output layout

A plain run writes one `<slug>_<timestamp>/` folder (model.urdf, meshes/, logs).
A `--judge` run nests one `iter_NN/` per attempt, each with model.urdf, meshes/,
6 rendered views, and a `judge.json` ({pass, reasons, suggestions}). Generated
`output/` is gitignored.

## Layout

```
main.py                 CLI entry; resolves Settings, calls the orchestrator
config.py               Settings dataclass (gateway, FreeCAD path, retries, limits)
workflow4freecad/
  manager.py            decompose prompt -> kinematic model (+ evaluator refine)
  worker.py             build one link's STL via headless freecadcmd
  judger.py             evaluator agent: 6 views + summary -> verdict JSON
  orchestrator.py       3-phase pipeline + run_with_judge loop
  urdf_builder.py       kinematic model -> model.urdf
  viz.py                assemble/render; render_six_views for the judge
  model.py              KinematicModel, WorkerResult, RunSummary, JudgeVerdict
  llm/                  hand-rolled OpenAI/Anthropic client + provider table
  prompts/              manager/worker/judger system prompts + schema
tests/                  unit tests + freecad isolation runners
```

## Notes

- Console/log output is ASCII-only (Windows cp1252); use `--`, `->`, `~`, `x`.
- Manager and judger parse strict JSON with a brace-depth extractor + repair loop;
  a continuous gear joint does NOT couple two wheels' spin (use `mimic` or a sim
  gear constraint for that), and fixed joints rigidly carry hands with their wheel.
