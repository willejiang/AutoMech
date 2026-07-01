#!/usr/bin/env python3
"""Physics evaluation of a maker2 URDF — category-aware, not a fool stand-still.

The old version dropped the model from 0.5 m and PASSed if it didn't topple — a
solid brick passed, and a gearbox was never actually driven. This version routes
through the evaluator's planner:

  strategy_selector.decide  -> pick static_stability / driven_mechanism / ... + a
                               test set (via maker2's 8313 gateway)
  scenario_designer.design  -> a scenario spec per test (a `drive` block for a
                               machine: which input joint to turn, what downstream
                               joints should move)
  run_scenario_pybullet.run -> actually actuate + measure transmission (or hold +
                               measure stability), capture frames

For a MACHINE this drives the input joint and checks the mechanism transmits; for a
structure/toy it keeps the stability check. Falls back gracefully (a static test)
if the planner is unavailable, so --physics never hard-crashes the run.
"""
from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path

# evaluator/ holds the planner + PyBullet runner; add repo root for the import.
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "evaluator"))
sys.path.insert(0, str(_ROOT))

_MOVABLE = {"revolute", "prismatic", "continuous"}
_DRIVER_HINT = re.compile(r"crank|handle|input|winder|drive|knob", re.I)


def _encode_mp4(frames_dir: str, out_path: str, fps: int = 12) -> str | None:
    """Stitch a test's rgb_*.png frames into an MP4 via imageio-ffmpeg's bundled
    ffmpeg (no system ffmpeg needed). Returns out_path on success, else None — a
    missing binary or zero frames just means "no video" (the breakdown still shows).
    Mirrors evaluator/make_mp4.sh's args."""
    import glob
    import subprocess
    frames = sorted(glob.glob(os.path.join(frames_dir, "rgb_*.png")))
    if not frames:
        return None
    try:
        import imageio_ffmpeg
        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception as e:
        print(f"[physics] no ffmpeg ({e}); skipping video")
        return None
    pattern = os.path.join(frames_dir, "rgb_*.png").replace("\\", "/")
    # +faststart moves the moov atom to the FRONT so browsers can start playback
    # without downloading the whole file (default libx264 puts moov at the end,
    # which makes an HTML5 <video> refuse to play).
    cmd = [ffmpeg, "-y", "-framerate", str(fps), "-pattern_type", "glob",
           "-i", pattern, "-c:v", "libx264", "-pix_fmt", "yuv420p",
           "-movflags", "+faststart",
           "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", out_path]
    try:
        subprocess.run(cmd, check=True, capture_output=True, timeout=120)
    except Exception as e:
        # Windows ffmpeg glob support varies; fall back to a frame-list concat.
        try:
            lst = os.path.join(frames_dir, "_frames.txt")
            with open(lst, "w") as f:
                for fr in frames:
                    f.write(f"file '{os.path.abspath(fr)}'\n")
            cmd2 = [ffmpeg, "-y", "-r", str(fps), "-f", "concat", "-safe", "0",
                    "-i", lst, "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart",
                    "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", out_path]
            subprocess.run(cmd2, check=True, capture_output=True, timeout=120)
        except Exception as e2:
            print(f"[physics] mp4 encode failed ({e2}); skipping video")
            return None
    return out_path if os.path.exists(out_path) and os.path.getsize(out_path) > 0 else None


def _static_spec() -> dict:
    """The legacy stability spec: hold, settle, check it doesn't sink/topple/drift."""
    return {
        "base_height": 0.5, "base_orientation_euler": [0, 0, 0],
        "self_collision": False, "control": {}, "joint_pose": [], "drive": None,
        "fixed_base": False, "duration_s": 4.0,
        "pass_criteria": {"min_base_height": 0.05, "max_drift": 0.5, "survive_s": 4.0},
    }


def _load_model(run_dir: str):
    """Load the KinematicModel saved next to this run, if present (for joint info)."""
    try:
        from maker2.manager import load_model
        p = os.path.join(run_dir, "kinematic_model.json")
        return load_model(p) if os.path.exists(p) else None
    except Exception:
        return None


def _robot_info(model) -> dict:
    """Selector/designer input: movable joints (with chain) + links + driver tag."""
    joints = []
    for j in model.joints:
        if j.type in _MOVABLE:
            joints.append({"name": j.name, "type": j.type,
                           "parent": j.parent, "child": j.child,
                           "driver": bool(getattr(j, "driver", False))})
    return {"name": model.name,
            "joints": joints,
            "links": [l.name for l in model.links]}


def _infer_driver(model) -> str | None:
    """Pick the input joint when the manager didn't tag one: (a) driver flag, else
    (b) a movable joint on a crank/handle/input-named link, else (c) the movable
    joint nearest the root, else (d) the first movable joint. Returns joint name."""
    movable = [j for j in model.joints if j.type in _MOVABLE]
    if not movable:
        return None
    for j in movable:                                   # (a) explicit tag
        if getattr(j, "driver", False):
            return j.name
    for j in movable:                                   # (b) name heuristic
        if _DRIVER_HINT.search(j.child) or _DRIVER_HINT.search(j.parent):
            return j.name
    root = model.root_link                              # (c) nearest the root
    for j in movable:
        if j.parent == root:
            return j.name
    return movable[0].name                              # (d) first movable


def _gateway():
    """maker2's 8313 gateway creds, for the selector/designer LLM calls."""
    try:
        from maker2.config import Settings
        s = Settings()
        return {"base_url": s.base_url, "api_key": s.api_key, "model": s.model}
    except Exception:
        return {"base_url": None, "api_key": None, "model": None}


def _plan(task: str, model) -> list[dict] | None:
    """Ask the strategy selector how to test this. Returns its `tests` list, or None
    if the planner/gateway is unavailable (caller then does a static test)."""
    try:
        import strategy_selector
        gw = _gateway()
        d = strategy_selector.decide(task, _robot_info(model),
                                     base_url=gw["base_url"], api_key=gw["api_key"],
                                     model=gw["model"])
        print(f"[physics] strategy: {d.get('strategy')} | "
              f"backend={d.get('sim_backend')} | dof={d.get('actuated_dof_count')} "
              f"| tests={[t.get('name') for t in d.get('tests', [])]}")
        tests = d.get("tests") or [{"name": d.get("strategy", "test"),
                                    "goal": task, "strategy": d.get("strategy")}]
        return tests
    except Exception as e:
        print(f"[physics] planner unavailable ({e}); static stability test only")
        return None


def _design_spec(task: str, model, test: dict) -> dict:
    """scenario_designer -> a spec for this test. For a driven test, backfill any
    drive sub-fields the gateway dropped (it often ignores schema keys): input
    joint (inferred), watched joints (all other movable), self-collision, etc."""
    from scenario_designer import design
    gw = _gateway()
    spec = design(task, _robot_info(model), test,
                  base_url=gw["base_url"], api_key=gw["api_key"], model=gw["model"])
    if test.get("strategy") == "driven_mechanism":
        drive = spec.get("drive") or {}
        if not drive.get("input_joint"):
            drive["input_joint"] = _infer_driver(model)
            print(f"[physics] inferred driver '{drive['input_joint']}'")
        drive.setdefault("mode", "velocity")
        if not drive.get("target_velocity"):
            drive["target_velocity"] = 5.0
        if not drive.get("duration_s"):
            drive["duration_s"] = 3.0
        if drive.get("self_collision") is None:
            drive["self_collision"] = True
        if not drive.get("min_watched_travel"):
            drive["min_watched_travel"] = 0.05
        if not drive.get("watch_joints"):
            inp = drive.get("input_joint")
            drive["watch_joints"] = [j.name for j in model.joints
                                     if j.type in _MOVABLE and j.name != inp]
            print(f"[physics] watching {len(drive['watch_joints'])} downstream joints")
        spec["drive"] = drive
        spec["fixed_base"] = True
    return spec


def _summarize(test: dict, m: dict) -> str:
    if m.get("test_kind") == "driven_mechanism":
        return (f"{test.get('name','drive')}: {m.get('verdict')} — input turned "
                f"{m.get('input_travel')} rad, {m.get('moved_count')}/"
                f"{m.get('watched_count')} downstream joints moved"
                + (" (JAMMED/EXPLODED)" if m.get('exploded') else ""))
    return (f"{test.get('name','stability')}: {m.get('verdict')} — settled z="
            f"{m.get('end_z')} tilt {m.get('max_tilt_deg')}deg "
            f"drift {m.get('max_drift')}m")


def run_physics(urdf_path: str, task: str, run_dir: str) -> dict:
    """Category-aware physics on maker2's URDF. Returns the same shape the UI reads:
    {passed, verdict, summary, metrics, frames_dir}. `metrics` is the FINAL/primary
    test; `summary` spans all tests run."""
    import run_scenario_pybullet as pyb

    model = _load_model(run_dir)
    tests = _plan(task, model) if model is not None else None

    # No model or no plan -> the legacy single static stability test.
    if not tests:
        out = str(Path(run_dir) / "physics" / "test_0")
        res = pyb.run(urdf_path, _static_spec(), out, task or "settle stably")
        m = res.get("metrics", {})
        video = None
        if res.get("frames_dir"):
            mp4 = _encode_mp4(res["frames_dir"], os.path.join(out, "model.mp4"))
            if mp4:
                video = "physics/test_0/model.mp4"
        entry = {"name": "stability", "strategy": "static_stability",
                 "verdict": m.get("verdict"), "metrics": m,
                 "summary": _summarize({"name": "stability"}, m),
                 "frames_dir": res.get("frames_dir"), "video": video}
        return {"passed": m.get("verdict") == "PASS", "verdict": m.get("verdict", "FAIL"),
                "summary": entry["summary"], "metrics": m,
                "frames_dir": res.get("frames_dir"), "video": video,
                "tests": [entry]}

    per_test = []
    primary = None                     # the machinery test if any, else the last
    for i, test in enumerate(tests):
        try:
            if test.get("strategy") == "driven_mechanism":
                spec = _design_spec(task, model, test)
            elif test.get("strategy") in ("static_stability", None):
                spec = _design_spec(task, model, test) if model is not None else _static_spec()
                spec.setdefault("drive", None)
            else:
                # scripted_motion / rl_training not executed yet -> stability proxy.
                print(f"[physics] test '{test.get('name')}' strategy "
                      f"'{test.get('strategy')}' not runnable here; stability proxy")
                spec = _static_spec()
        except Exception as e:
            print(f"[physics] designer failed for '{test.get('name')}' ({e}); static")
            spec = _static_spec()

        out = str(Path(run_dir) / "physics" / f"test_{i}")
        res = pyb.run(urdf_path, spec, out, f"{task} :: {test.get('name','')}")
        m = res.get("metrics", {})
        video = None
        if res.get("frames_dir"):
            mp4 = _encode_mp4(res["frames_dir"], os.path.join(out, "model.mp4"))
            if mp4:
                video = f"physics/test_{i}/model.mp4"
                print(f"[physics] test {i} video -> {video}")
        entry = {"name": test.get("name"), "strategy": test.get("strategy"),
                 "verdict": m.get("verdict"), "metrics": m,
                 "frames_dir": res.get("frames_dir"), "summary": _summarize(test, m),
                 "video": video}
        per_test.append(entry)
        if m.get("test_kind") == "driven_mechanism":
            primary = entry
    if primary is None:
        primary = per_test[-1]

    passed = all(t.get("verdict") == "PASS" for t in per_test)
    verdict = "PASS" if passed else "FAIL"
    summary = " | ".join(t["summary"] for t in per_test)
    return {"passed": passed, "verdict": verdict, "summary": summary,
            "metrics": primary.get("metrics", {}),
            "frames_dir": primary.get("frames_dir"),
            "video": primary.get("video"),
            "tests": per_test}


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--task", default="")
    ap.add_argument("--out", default=".", help="run_dir (holds kinematic_model.json)")
    a = ap.parse_args()
    print(json.dumps(run_physics(a.urdf, a.task, a.out), indent=2))
