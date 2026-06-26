# AutoMech Orchestrator

The **outer automation loop** that wires the two halves of AutoMech together:

```
prompt → cadam (.scad) → render 6 views → VISUAL GATE (VLM) → author manifest/URDF
       → Isaac Sim evaluator → PASS? done : fix_hint → back to cadam
```

- **Maker** = `worker/` (cadam, text→OpenSCAD). Driven here at the **`.scad` seam**,
  not over HTTP — cadam's server path is Supabase-bound, so the loop replicates its
  generation by calling the LLM with cadam's own `PARAMETRIC_AGENT_PROMPT`.
- **Evaluator** = `evaluator/` (Isaac Sim + VLM). Consumed unchanged via
  `evaluate.sh <design_dir>`.

See [`../DESIGN_LOOP.md`](../DESIGN_LOOP.md) for the architecture and why it's two
nested loops with an inner visual gate.

## Pieces

| File | Runs | Does |
|------|------|------|
| `automech_loop.py` | host | The loop. Single entry point. |
| `cadam_bridge.py` | host | `generate(task, feedback)` → `.scad` (cadam's prompt, direct LLM). |
| `render_views.py` | host | `.scad` → STL + 6 orthographic PNGs + module names (native OpenSCAD CLI). |
| `visual_gate.py` | host | 6-view VLM geometry/prompt-match gate (ports `evaluateModel.ts`). |
| `urdf_author.py` | host | VLM authors `manifest.json` (the "VLM codes the URDF" step). |
| `evaluator_bridge.py` | host | Shells `evaluator/evaluate.sh`; stubs the verdict under `--dry-run`. |

## Setup

```bash
cp .env.example .env          # fill in the VLM key (reuses evaluator's AZURE_* names)
pip install openai            # OpenAI-compatible client for the gateway

# native OpenSCAD CLI + libraries (so render_views can compile .scad headless)
#   macOS:  brew install openscad imagemagick
#   linux:  apt install openscad imagemagick
#   win:    install OpenSCAD, set OPENSCAD_BIN to openscad.exe
mkdir -p /tmp/oscad-libs && cd /tmp/oscad-libs \
  && unzip -o ../../worker/public/libraries/BOSL2.zip \
  && unzip -o ../../worker/public/libraries/MCAD.zip
export OPENSCADPATH=/tmp/oscad-libs
```

## Run

```bash
# Full loop, Isaac Sim STUBBED (Aliyun box offline → no GPU needed):
python automech_loop.py --task "quarter-car suspension that clears a 10cm curb" \
   --dry-run --max-iters 3

# Live (only when the Isaac Sim box is back up): drop --dry-run.
python automech_loop.py --task "..." --max-iters 4
```

Each iteration writes a self-contained design dir under `runs/<id>/iter_<N>/`:
`model.scad`, `model.stl`, `views/*.png`, `gate.json`, `manifest.json`,
`out/result.json`.

### Test individual stages

```bash
# render only (no LLM): uses a real cadam benchmark as known-good input
python render_views.py ../worker/benchmarks/07-bevel-gear-drive.scad
# gate only:
python visual_gate.py --task "a bevel gear drive" path/to/views/*.png
# generation only:
python cadam_bridge.py --task "a coffee mug" --out /tmp/mug.scad
```

## Offline status (important)

The **Aliyun A10 box is down**, so the real Isaac Sim step cannot run. The loop is
wired so `--dry-run` (and any missing-Docker host) stubs `result.json`
(`passed:false`, synthetic fix hint) and logs that the sim was **skipped** — the
whole chain is then exercisable without a GPU. Flip `--dry-run` off, with the box
online, for a true physics verdict. This is the **only** stage the offline box
blocks; generation, rendering, the visual gate, and manifest authoring all run now.
