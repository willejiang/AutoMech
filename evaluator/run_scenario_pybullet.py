#!/usr/bin/env python3
"""PyBullet scenario runner — CPU, no GPU/server. Mirrors run_scenario.py's contract
so analyze.py is unchanged. Spawns the URDF in the spec's pose, holds joints, sims
duration_s, captures frames, writes sim_result.json (task, spec, metrics, frames_dir,
n_frames, log). Verdict is the VLM's job; this just produces honest physics + frames."""
import argparse
import json
import math
import os
from pathlib import Path

import pybullet as p
import pybullet_data
import numpy as np
from PIL import Image


def euler_quat(e):
    return p.getQuaternionFromEuler(e if len(e) == 3 else [0, 0, 0])


def run(urdf, spec, out_dir, task):
    out = Path(out_dir)
    frames = out / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    log = []

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    z0 = float(spec.get("base_height", 0.5))
    quat = euler_quat(spec.get("base_orientation_euler", [0, 0, 0]))
    flags = p.URDF_USE_SELF_COLLISION if spec.get("self_collision") else 0
    rid = p.loadURDF(urdf, [0, 0, z0], quat, useFixedBase=False, flags=flags)

    name_to_idx = {p.getJointInfo(rid, i)[1].decode(): i for i in range(p.getNumJoints(rid))}
    ctrl = spec.get("control", {})
    stiff, damp = float(ctrl.get("stiffness", 40)), float(ctrl.get("damping", 2))
    targets = {jp["joint"]: jp["angle"] for jp in spec.get("joint_pose", [])}
    for jn, ang in targets.items():
        if jn in name_to_idx:
            p.resetJointState(rid, name_to_idx[jn], ang)

    dur = float(spec.get("duration_s", 4.0))
    hz, steps = 240, int(dur * 240)
    p.setTimeStep(1.0 / hz)
    # auto-settle: 1s with joints held so the body drops onto its feet instead of
    # being judged mid-fall from a too-high spawn (the dog-above-plane bug).
    for _ in range(240):
        for jn, ang in targets.items():
            if jn in name_to_idx:
                p.setJointMotorControl2(rid, name_to_idx[jn], p.POSITION_CONTROL, targetPosition=ang, force=stiff)
        p.stepSimulation()
    z0 = p.getBasePositionAndOrientation(rid)[0][2]  # post-settle standing height
    cap_every = max(1, steps // 60)
    x0, y0, _ = p.getBasePositionAndOrientation(rid)[0]
    zmin, max_tilt, nf = 99.0, 0.0, 0
    for s in range(steps):
        if ctrl.get("mode", "position_hold") == "position_hold":
            for jn, ang in targets.items():
                if jn in name_to_idx:
                    p.setJointMotorControl2(rid, name_to_idx[jn], p.POSITION_CONTROL,
                                            targetPosition=ang, positionGain=stiff/100, force=stiff)
        p.stepSimulation()
        pos, orn = p.getBasePositionAndOrientation(rid)
        roll, pitch, _ = p.getEulerFromQuaternion(orn)
        zmin = min(zmin, pos[2]); max_tilt = max(max_tilt, math.degrees(max(abs(roll), abs(pitch))))
        if s % cap_every == 0:
            w, h, px, _, _ = p.getCameraImage(640, 480,
                viewMatrix=p.computeViewMatrixFromYawPitchRoll([0, 0, 0.1], 1.6, 35, -20, 0, 2),
                projectionMatrix=p.computeProjectionMatrixFOV(60, 1.33, 0.05, 40))
            rgb = np.reshape(np.array(px, dtype=np.uint8), (h, w, 4))[..., :3]
            Image.fromarray(rgb).save(frames / f"rgb_{nf:04d}.png")
            nf += 1
    pos = p.getBasePositionAndOrientation(rid)[0]
    drift = math.hypot(pos[0]-x0, pos[1]-y0)
    pc = spec.get("pass_criteria", {})
    floor, max_drift, need_s = pc.get("min_base_height", 0.1), pc.get("max_drift", 1.0), pc.get("survive_s", dur)
    verdict = "PASS" if (zmin >= floor and drift <= max_drift) else "FAIL"
    base_dx = round(pos[0] - x0, 4); base_dz = round(pos[2] - z0, 4)
    metrics = {"verdict": verdict, "survive_s": round(dur, 2), "min_base_z": round(zmin, 4),
               "max_drift": round(drift, 4), "max_tilt_deg": round(max_tilt, 2),
               "start_z": round(z0, 4), "end_z": round(pos[2], 4),
               "base_dx": base_dx, "base_dz": base_dz}
    p.disconnect()
    res = {"task": task, "spec": spec, "metrics": metrics,
           "frames_dir": str(frames), "n_frames": nf, "log": log}
    (out / "sim_result.json").write_text(json.dumps(res, indent=2))
    print(f"[pyb] {nf} frames, zmin={zmin:.3f} tilt={max_tilt:.1f} drift={drift:.3f}")
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--spec", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--task", default="")
    a = ap.parse_args()
    run(a.urdf, json.loads(Path(a.spec).read_text()), a.out, a.task)
