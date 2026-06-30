#!/usr/bin/env python3
"""Run PyBullet physics on a maker2-produced URDF — NO urdf_author, NO manifest.

maker2 already emits an articulated model.urdf + per-link meshes. This consumes
that URDF directly: a rigid drop/stability test (does the assembled product settle
without falling apart or toppling). CPU-only via evaluator/run_scenario_pybullet.

This is the "physics on maker2's URDF" step the UI bridge calls after the judge.
"""
from __future__ import annotations

import sys
from pathlib import Path

# evaluator/ holds the PyBullet runner; add repo root for the import.
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "evaluator"))


# A minimal stability spec: spawn slightly above ground, no joint drive, let it
# settle, and judge that the base doesn't sink/topple/drift. Enough for a v1
# "does the assembly hold together" check on any URDF.
def _default_spec() -> dict:
    return {
        "base_height": 0.5,
        "base_orientation_euler": [0, 0, 0],
        "self_collision": False,
        "control": {},
        "joint_pose": [],
        "duration_s": 4.0,
        "pass_criteria": {"min_base_height": 0.05, "max_drift": 0.5, "survive_s": 4.0},
    }


def run_physics(urdf_path: str, task: str, run_dir: str) -> dict:
    """-> {passed, verdict, summary, metrics, frames_dir}. Shells the PyBullet
    runner on maker2's URDF (the URDF is the input; nothing is re-authored)."""
    import run_scenario_pybullet as pyb  # from evaluator/

    out = str(Path(run_dir) / "physics")
    res = pyb.run(urdf_path, _default_spec(), out, task or "settle stably")
    m = res.get("metrics", {})
    verdict = m.get("verdict", "FAIL")
    return {
        "passed": verdict == "PASS",
        "verdict": verdict,
        "summary": (f"{verdict}: settled to z={m.get('end_z')} "
                    f"(min {m.get('min_base_z')}), tilt {m.get('max_tilt_deg')}deg, "
                    f"drift {m.get('max_drift')}m over {m.get('survive_s')}s"),
        "metrics": m,
        "frames_dir": res.get("frames_dir"),
    }


if __name__ == "__main__":
    import argparse, json
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--task", default="")
    ap.add_argument("--out", default=".")
    a = ap.parse_args()
    print(json.dumps(run_physics(a.urdf, a.task, a.out), indent=2))
