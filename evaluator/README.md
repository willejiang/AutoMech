# PhysCAD Evaluator

The **Isaac Sim evaluator** half of PhysCADResearcher. Takes a CAD design from the
maker loop + the user's task, simulates it in NVIDIA Isaac Sim, and returns a
pass/fail verdict with per-failure frames + feedback for the maker to iterate on.

## What this owns
```
maker loop  --(manifest.json + .scad/.stl)-->  [EVALUATOR]  --(result.json)-->  maker loop
```
Everything else (worker, prejudger, prompt→CAD generation) lives elsewhere.

## The contract: what the maker hands us

A **design dir** containing:
- `manifest.json` — parts, joints, materials, scenario, pass criteria (see `examples/quarter_car_manifest.json`)
- the `.scad` (or `.stl`) it references

Key point: a fused mesh can't articulate. For suspension tests the maker should emit
**a `.scad` with one module per part** (named in `manifest.parts[].render_module`) so we
render parts separately and synthesize joints from `manifest.joints`. STL-only input
falls back to a rigid drop test (no articulation).

## Output: the maker handoff

`out/result.json`:
```json
{
  "passed": false,
  "summary": "Suspension bottomed out on the curb; chassis struck ground.",
  "failures": [
    {
      "failure_mode": "bottoming_out",
      "explanation": "Spring fully compressed at frame 5; chassis contacted ground.",
      "frame_indices": [5, 6, 7],
      "frame_files": ["out/frames/rgb_0005.png", "..."],
      "fix_hint": "Increase spring stiffness or suspension travel limit."
    }
  ],
  "metrics": { "...": "measured physical quantities" }
}
```

## Run it
```bash
cp .env.example .env       # fill in the Azure key
./evaluate.sh examples/    # runs sim (container) + analysis (host)
```

## Pieces
| file | runs | does |
|------|------|------|
| `run_eval.py` | inside isaac-sim container | scad→STL, synth URDF, import, sim, capture frames + metrics |
| `analyze.py` | host venv | sample frames → Azure VLM → structured pass/fail |
| `evaluate.sh` | host | orchestrates both halves |

## Stack
- Box: Aliyun A10 (ecs.gn7i), Ubuntu 26.04, driver 595.71.05, Docker + NVIDIA toolkit
- Sim: `nvcr.io/nvidia/isaac-sim:6.0.1` (namespace `isaacsim.*`)
- VLM: Azure OpenAI (no native video on Azure → frame sampling)

## RL evaluation (Isaac Lab)

For tasks needing a learned controller (e.g. "create a walkable robot"), the
evaluator trains an RL policy instead of holding a static pose:

- `strategy_selector.py` — LLM decides static_stability / scripted_motion / **rl_training** from the task + URDF (counts actuated DOF, flags structural feasibility).
- `isaaclab/install_isaaclab.sh` — install Isaac Lab into the isaac-sim container (committed as image `isaac-lab:6.0.1`).
- `isaaclab/convert_urdf_to_usd.sh` — URDF → USD (Isaac Lab spawns from USD).
- `isaaclab/train_locomotion.sh` — train a shipped velocity task (e.g. `Isaac-Velocity-Flat-Cassie-v0`) on the GPU, headless.
- `isaaclab/play_policy.sh` — render a trained checkpoint to MP4 for VLM judging.
- `analyze.py` — opus-4.8 judges the rendered behavior: locomotion progress vs structural failure.
- `compare_before_after.py` — opus-4.8 judges BEFORE (untrained, iter 0) vs AFTER (trained) keyframes side-by-side, to confirm RL produced a real behavioral improvement rather than a single-checkpoint guess.

Proven: Cassie trains on a single A10 (~0.73 s/iter; ~12–18 min for a walking
policy); opus-4.8 correctly reads early behavior as "progressing, structurally
sound" vs collapse.
