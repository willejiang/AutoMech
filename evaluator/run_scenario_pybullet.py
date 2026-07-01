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


def _joint_angle(rid, idx):
    return p.getJointState(rid, idx)[0]


def _frame_camera(rid):
    """Auto-frame the whole model: target its AABB center, back off proportional to
    its size so a tiny mechanism (a 40 mm watch) fills the frame instead of being a
    speck at a fixed 1.2 m. Returns (target[3], distance)."""
    lo = np.array([1e9, 1e9, 1e9]); hi = -lo
    n = p.getNumJoints(rid)
    for link in range(-1, n):
        try:
            amin, amax = p.getAABB(rid, link)
            lo = np.minimum(lo, amin); hi = np.maximum(hi, amax)
        except Exception:
            pass
    if not np.all(np.isfinite(lo)):
        return [0, 0, 0.1], 1.2
    center = ((lo + hi) / 2.0).tolist()
    extent = float(np.linalg.norm(hi - lo))
    # ~0.9x the diagonal fills a 60deg FOV without much dead space; clamp to sane.
    dist = max(0.08, min(4.0, extent * 0.9))
    return center, dist


def _run_driven(rid, name_to_idx, spec, drive, frames, dur):
    """Actuate an INPUT joint and watch whether downstream joints move — the real
    "does the mechanism transmit" test. Returns (metrics_patch, n_frames)."""
    hz = 240
    steps = int(dur * hz)
    p.setTimeStep(1.0 / hz)

    in_name = drive.get("input_joint")
    in_idx = name_to_idx.get(in_name)
    watch = [w for w in drive.get("watch_joints", []) if w in name_to_idx]
    mode = drive.get("mode", "velocity")
    vel = float(drive.get("target_velocity", 3.0))
    sweep = drive.get("sweep", [0.0, 6.283])

    # Free the driven + watched joints from the default motor so they can move.
    for jn in ([in_name] + watch):
        if jn in name_to_idx:
            p.setJointMotorControl2(rid, name_to_idx[jn], p.VELOCITY_CONTROL, force=0)

    a0 = {jn: _joint_angle(rid, name_to_idx[jn]) for jn in ([in_name] + watch)
          if jn in name_to_idx}
    amin = dict(a0); amax = dict(a0)
    exploded = False
    cap_every = max(1, steps // 60)
    nf = 0
    base0 = np.array(p.getBasePositionAndOrientation(rid)[0])
    cam_target, cam_dist = _frame_camera(rid)   # frame to the mechanism's size

    for s in range(steps):
        if in_idx is not None:
            if mode == "position_sweep":
                frac = (s / max(1, steps - 1))
                tgt = sweep[0] + frac * (sweep[1] - sweep[0])
                p.setJointMotorControl2(rid, in_idx, p.POSITION_CONTROL,
                                        targetPosition=tgt, force=200)
            else:  # velocity
                p.setJointMotorControl2(rid, in_idx, p.VELOCITY_CONTROL,
                                        targetVelocity=vel, force=200)
        p.stepSimulation()

        for jn in ([in_name] + watch):
            if jn in name_to_idx:
                a = _joint_angle(rid, name_to_idx[jn])
                if not math.isfinite(a):
                    exploded = True
                    a = amax.get(jn, 0.0)
                amin[jn] = min(amin.get(jn, a), a)
                amax[jn] = max(amax.get(jn, a), a)
        # link separation heuristic: base flies away -> the model blew apart
        base = np.array(p.getBasePositionAndOrientation(rid)[0])
        if not np.all(np.isfinite(base)) or np.linalg.norm(base - base0) > 5.0:
            exploded = True

        if s % cap_every == 0:
            w, h, px, _, _ = p.getCameraImage(640, 480,
                viewMatrix=p.computeViewMatrixFromYawPitchRoll(cam_target, cam_dist, 45, -45, 0, 2),
                projectionMatrix=p.computeProjectionMatrixFOV(60, 1.33, 0.02, 40))
            rgb = np.reshape(np.array(px, dtype=np.uint8), (h, w, 4))[..., :3]
            Image.fromarray(rgb).save(frames / f"rgb_{nf:04d}.png")
            nf += 1

    def travel(jn):
        return round(amax.get(jn, 0.0) - amin.get(jn, 0.0), 4)

    input_travel = travel(in_name) if in_name else 0.0
    watched = {jn: travel(jn) for jn in watch}
    move_thresh = float(drive.get("min_watched_travel", 0.05))
    moved_count = sum(1 for v in watched.values() if v >= move_thresh)
    patch = {
        "input_joint": in_name,
        "input_travel": input_travel,
        "watched": watched,
        "moved_count": moved_count,
        "watched_count": len(watched),
        "exploded": bool(exploded),
    }
    return patch, nf


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
    drive = spec.get("drive")
    # A mechanism is bench-mounted: fix the base so it can't just topple, and let
    # its own parts self-collide so gears/teeth actually mesh.
    fixed_base = bool(spec.get("fixed_base") or drive)
    self_coll = bool(spec.get("self_collision") or (drive and drive.get("self_collision")))
    flags = p.URDF_USE_SELF_COLLISION if self_coll else 0
    rid = p.loadURDF(urdf, [0, 0, z0], quat, useFixedBase=fixed_base, flags=flags)

    name_to_idx = {p.getJointInfo(rid, i)[1].decode(): i for i in range(p.getNumJoints(rid))}
    dur = float((drive or spec).get("duration_s", spec.get("duration_s", 4.0)))

    # ---- DRIVEN MECHANISM path: actuate the input, watch the transmission ----
    if drive:
        dpatch, nf = _run_driven(rid, name_to_idx, spec, drive, frames, dur)
        pos = p.getBasePositionAndOrientation(rid)[0]
        # Verdict for a driven test: the input actually turned, it drove at least
        # one downstream joint, and nothing exploded. (The VLM refines this.)
        transmitted = (dpatch["input_travel"] > 0.05
                       and dpatch["moved_count"] >= 1
                       and not dpatch["exploded"])
        verdict = "PASS" if transmitted else "FAIL"
        metrics = {"verdict": verdict, "test_kind": "driven_mechanism",
                   "survive_s": round(dur, 2), **dpatch,
                   "end_z": round(pos[2], 4)}
        p.disconnect()
        res = {"task": task, "spec": spec, "metrics": metrics,
               "frames_dir": str(frames), "n_frames": nf, "log": log}
        (out / "sim_result.json").write_text(json.dumps(res, indent=2))
        print(f"[pyb] driven: input_travel={dpatch['input_travel']} "
              f"moved={dpatch['moved_count']}/{dpatch['watched_count']} "
              f"exploded={dpatch['exploded']} -> {verdict}")
        return res

    # ---- STATIC STABILITY path (unchanged) ----
    ctrl = spec.get("control", {})
    stiff, damp = float(ctrl.get("stiffness", 40)), float(ctrl.get("damping", 2))
    targets = {jp["joint"]: jp["angle"] for jp in spec.get("joint_pose", [])}
    for jn, ang in targets.items():
        if jn in name_to_idx:
            p.resetJointState(rid, name_to_idx[jn], ang)

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
