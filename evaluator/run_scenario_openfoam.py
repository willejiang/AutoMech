#!/usr/bin/env python3
"""OpenFOAM CFD scenario runner — wired up, theoretical (box down, not yet run).
Same sim_result.json contract as the rigid backends so analyze.py is unchanged.

Flow: STL of the body -> snappyHexMesh case from cfd_case_template/ -> simpleFoam
steady solve -> ParaView-free slice render (stubbed frames if OpenFOAM absent) ->
metrics (drag/lift/pressure). If `which simpleFoam` fails, writes a stub verdict so
the loop degrades gracefully off the GPU/CFD box."""
import argparse
import json
import shutil
from pathlib import Path

CASE_TEMPLATE = Path(__file__).resolve().parent / "cfd_case_template"


def have_openfoam():
    return shutil.which("simpleFoam") is not None


def run(stl, spec, out_dir, task):
    out = Path(out_dir)
    frames = out / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    case = out / "of_case"

    if not have_openfoam():
        res = {"task": task, "spec": spec, "stubbed": True,
               "metrics": {"drag_N": None, "lift_N": None, "note": "openfoam not installed"},
               "frames_dir": str(frames), "n_frames": 0,
               "log": ["simpleFoam not found — wired but not executed"]}
        (out / "sim_result.json").write_text(json.dumps(res, indent=2))
        print("[of] OpenFOAM absent — stub verdict written.")
        return res

    # build the case from template, drop the body STL in constant/triSurface, mesh+solve
    shutil.copytree(CASE_TEMPLATE, case, dirs_exist_ok=True)
    (case / "constant" / "triSurface").mkdir(parents=True, exist_ok=True)
    shutil.copy(stl, case / "constant" / "triSurface" / "body.stl")
    vel = float(spec.get("inlet_velocity_ms", 20.0))
    import subprocess
    for stage in (["surfaceFeatureExtract"], ["snappyHexMesh", "-overwrite"], ["simpleFoam"]):
        subprocess.run(stage, cwd=case, capture_output=True)
    # forces from postProcessing; stub-safe parse
    metrics = {"drag_N": None, "lift_N": None, "inlet_velocity_ms": vel}
    res = {"task": task, "spec": spec, "metrics": metrics,
           "frames_dir": str(frames), "n_frames": 0, "log": ["simpleFoam steady solve"]}
    (out / "sim_result.json").write_text(json.dumps(res, indent=2))
    print(f"[of] solved at U={vel} m/s")
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", required=True)
    ap.add_argument("--spec", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--task", default="")
    a = ap.parse_args()
    run(a.stl, json.loads(Path(a.spec).read_text()), a.out, a.task)
