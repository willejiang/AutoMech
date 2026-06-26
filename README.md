# PhysCAD Researcher

**Task-oriented CAD generation, closed-loop with physics.** Instead of producing
geometry that merely *looks* plausible, every design is simulated in **NVIDIA Isaac
Sim** under the user's actual task and judged by a vision model. Failures come back
as rendered frames plus concrete fix hints, and the design is revised until it
physically works.

## Demo

https://github.com/willejiang/PhysCADResearcher/raw/main/assets/AutoMech1.mp4

<!-- Inline player (renders where raw HTML video is allowed; falls back to the link above on github.com Markdown). -->
<video src="assets/AutoMech1.mp4" controls muted width="720">
  Your viewer can't embed the video —
  <a href="assets/AutoMech1.mp4">download AutoMech1.mp4</a>.
</video>

> If the player doesn't appear, use the link above or open
> [`assets/AutoMech1.mp4`](assets/AutoMech1.mp4) directly.

## How it works

The system is **two nested feedback loops** connected by a file contract:

- **Maker subloop (inner, fast, no physics):** a *worker* (`cadam`) generates CAD
  geometry; a *6-perspective visual judger* (VLM) gates it on geometry sanity and
  prompt-match before any sim runs. On pass, a VLM authors the URDF directly
  (joints, axes, densities) — no CAD kernel / FreeCAD step.
- **Evaluator loop (outer, slow, physical):** only a design that clears the visual
  gate is simulated in Isaac Sim and judged under the task. Physics failures
  (`fix_hint`s) feed back to the worker.

> **Thesis (shown empirically):** numeric pose metrics gave a *false PASS* on the
> ANYmal stand-still run (tilt read 2.1°), while the VLM watching the frames
> correctly said *FAIL — "tips onto its side by frame 3, ends overturned."*
> The camera/VLM judge is what makes the evaluation trustworthy.

See **[DESIGN_LOOP.md](DESIGN_LOOP.md)** for the full architecture with diagrams of
both loops.

## Repo layout

| Path | What |
|------|------|
| [`DESIGN_LOOP.md`](DESIGN_LOOP.md) | Architecture: maker subloop + evaluator loop, with Mermaid diagrams |
| [`evaluator/`](evaluator/) | The Isaac Sim evaluator (this project's owned half) |
| [`evaluator/loop.py`](evaluator/loop.py) | Host orchestrator: scenario design → sim → VLM judge → revise |
| [`evaluator/run_scenario.py`](evaluator/run_scenario.py) | In-container sim runner (URDF + spec → frames + metrics) |
| [`evaluator/analyze.py`](evaluator/analyze.py) | Host-side VLM pass/fail judge over sampled frames |
| `assets/AutoMech1.mp4` | Demo recording |

## Stack

- **Compute:** Aliyun A10 (`ecs.gn7i`), Ubuntu 26.04, NVIDIA driver 595.71.05
- **Sim:** `nvcr.io/nvidia/isaac-sim:6.0.1` (namespace `isaacsim.*`)
- **VLM:** Azure OpenAI — no native-video model, so the evaluator samples keyframes
  into a multi-image message
