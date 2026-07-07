#!/usr/bin/env python3
"""MuJoCo scenario runner — pure contact under gravity (maker2-mujoco-contact).

Mirrors run_scenario_pybullet.run's subprocess+timeout contract so physics.py and
the UI are unchanged: same CLI (--mjcf --spec --out --task), writes sim_result.json
with the SAME metrics keys `_summarize`/`_aggregate` read, and test_kind=
"driven_mechanism" for a driven test.

Unlike the PyBullet path (joint motors, fixed base), this DROPS the assembly on a
ground plane, settles it under gravity, then applies torque to the DRIVER PART's own
dof (`qfrc_applied`). Downstream parts move ONLY if their teeth truly contact — that
IS the transmission test. Frames come from mujoco.Renderer (best-effort; metrics
never depend on video).

The driver + watched parts are read from the model, NOT from a topology walk:
  driver  = the one LinkSpec with driver=True (spin/free).
  watched = every OTHER spin/free LinkSpec (contact decides if they move).
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

import numpy as np


def _load_model(run_dir: str):
    """Load the KinematicModel saved for this run (driver/watched/dof info)."""
    try:
        import sys
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if root not in sys.path:
            sys.path.insert(0, root)
        from maker2.manager import load_model
        for cand in ("kinematic_model.json", "model.json"):
            p = os.path.join(run_dir, cand)
            if os.path.exists(p):
                return load_model(p)
    except Exception:
        pass
    return None


def _spin_links(model):
    return [l for l in model.links if getattr(l, "dof", "fixed") in ("spin", "free")]


def _driver_link(model):
    for l in model.links:
        if getattr(l, "driver", False):
            return l
    spins = _spin_links(model)
    return spins[0] if spins else None


def run(mjcf: str, spec: dict, out_dir: str, task: str) -> dict:
    import mujoco as mj

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    frames = out / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    log: list[str] = []

    run_dir = spec.get("run_dir") or str(Path(mjcf).parent)
    model = _load_model(run_dir)

    m = mj.MjModel.from_xml_path(mjcf)
    d = mj.MjData(m)

    dur = float(spec.get("duration_s", 4.0))
    settle_steps = int(0.5 / m.opt.timestep)      # ~0.5 s settle
    drive_steps = int(dur / m.opt.timestep)

    # Renderer is best-effort; capture ~40 frames across the driven phase.
    renderer = None
    try:
        renderer = mj.Renderer(m, 480, 640)
    except Exception as e:
        log.append(f"renderer unavailable: {e}")

    def capture(idx: int):
        if renderer is None:
            return False
        try:
            from PIL import Image
            renderer.update_scene(d)
            img = renderer.render()
            Image.fromarray(img).save(frames / f"rgb_{idx:04d}.png")
            return True
        except Exception:
            return False

    # ---- settle under gravity ----
    for _ in range(settle_steps):
        mj.mj_step(m, d)

    driver = _driver_link(model) if model is not None else None
    watched = [l for l in _spin_links(model)
               if driver is None or l.name != driver.name] if model is not None else []

    def joint_qadr(link_name, suffix):
        try:
            j = m.joint(f"{link_name}_{suffix}")
            return m.jnt_qposadr[j.id], m.jnt_dofadr[j.id]
        except Exception:
            return None, None

    # Baseline angles (post-settle) for the driver + watched parts.
    def angle_of(link):
        for suf in ("spin", "free"):
            qadr, dofadr = joint_qadr(link.name, suf)
            if qadr is not None:
                return float(d.qpos[qadr]), dofadr
        return None, None

    driver_dofadr = None
    a0 = 0.0
    if driver is not None:
        a0, driver_dofadr = angle_of(driver)
        a0 = a0 or 0.0
    watched_base = {}
    for l in watched:
        ang, _ = angle_of(l)
        if ang is not None:
            watched_base[l.name] = ang

    # Track exploded = any body flew far from its start (settle-relative). Also keep the
    # settle position of every body so we can attribute a FLOATING/EXPLODING fault to the
    # specific part (its body name == the link name) for per-manager blame.
    xpos0 = d.xpos.copy()

    torque = float((spec.get("drive") or {}).get("torque", 0.5))
    cap_every = max(1, drive_steps // 40)
    nf = 0
    max_disp = 0.0
    per_body_disp = np.zeros(m.nbody)
    if driver is not None and driver_dofadr is not None:
        for s in range(drive_steps):
            d.qfrc_applied[driver_dofadr] = torque
            mj.mj_step(m, d)
            if len(d.xpos):
                dvec = np.linalg.norm(d.xpos - xpos0, axis=1)
                per_body_disp = np.maximum(per_body_disp, dvec)
                max_disp = max(max_disp, float(dvec.max()))
            if s % cap_every == 0 and capture(nf):
                nf += 1
        d.qfrc_applied[driver_dofadr] = 0.0
    else:
        # No driver — a stand-still stability observation.
        for s in range(drive_steps):
            mj.mj_step(m, d)
            if len(d.xpos):
                dvec = np.linalg.norm(d.xpos - xpos0, axis=1)
                per_body_disp = np.maximum(per_body_disp, dvec)
                max_disp = max(max_disp, float(dvec.max()))
            if s % cap_every == 0 and capture(nf):
                nf += 1

    # ---- measure transmission ----
    input_travel = 0.0
    if driver is not None:
        a1, _ = angle_of(driver)
        input_travel = abs((a1 or 0.0) - a0)
    moved = 0
    for l in watched:
        ang, _ = angle_of(l)
        if ang is None:
            continue
        if abs(ang - watched_base.get(l.name, 0.0)) > 0.05:
            moved += 1
    watched_count = len(watched_base)
    output_reached = None
    out_link = (spec.get("drive") or {}).get("output_link")
    if out_link:
        for l in watched:
            if l.name == out_link:
                ang, _ = angle_of(l)
                output_reached = bool(ang is not None
                                      and abs(ang - watched_base.get(l.name, 0.0)) > 0.05)
    elif watched_count:
        output_reached = moved >= 1

    exploded = bool(max_disp > 0.5)   # a part flew >0.5 m from settle => blew up
    # base height + tilt for stability signal
    end_z = float(d.xpos[:, 2].min()) if len(d.xpos) else 0.0
    max_tilt = 0.0
    try:
        # tilt of the root body (body 1; body 0 is world)
        if m.nbody > 1:
            quat = d.xquat[1]
            w, x, y, z = quat
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
            max_tilt = math.degrees(max(abs(roll), abs(pitch)))
    except Exception:
        pass

    if renderer is not None:
        try:
            renderer.close()
        except Exception:
            pass

    # Attribute a FLOATING/EXPLODING fault to specific parts: bodies whose settle-relative
    # displacement is large (they fell off / flew apart / were never supported). Body name
    # == link name, so this feeds per-manager blame downstream. Reported worst-first.
    _FLOAT_M = 0.003            # >3 mm of drift from settle = not held in place
    displaced_parts = []
    try:
        for b in range(1, m.nbody):          # skip world (body 0)
            name = m.body(b).name
            disp = float(per_body_disp[b])
            if name and disp > _FLOAT_M:
                displaced_parts.append({"part": name, "disp_mm": round(disp * 1000, 2)})
        displaced_parts.sort(key=lambda e: e["disp_mm"], reverse=True)
        displaced_parts = displaced_parts[:8]
    except Exception:
        displaced_parts = []

    if driver is not None:
        transmitted = (input_travel > 0.05 and moved >= 1 and not exploded)
        verdict = "PASS" if transmitted else "FAIL"
        metrics = {
            "verdict": verdict, "test_kind": "driven_mechanism",
            "survive_s": round(dur, 2),
            "input_travel": round(input_travel, 4),
            "moved_count": moved, "watched_count": watched_count,
            "output_reached": output_reached, "exploded": exploded,
            "displaced_parts": displaced_parts,
            "end_z": round(end_z, 4), "max_tilt_deg": round(max_tilt, 2),
            "max_drift": round(max_disp, 4),
        }
    else:
        # stand-still stability
        verdict = "PASS" if (not exploded and end_z > -0.1) else "FAIL"
        metrics = {
            "verdict": verdict, "survive_s": round(dur, 2),
            "end_z": round(end_z, 4), "max_tilt_deg": round(max_tilt, 2),
            "max_drift": round(max_disp, 4), "exploded": exploded,
            "displaced_parts": displaced_parts,
        }

    res = {"task": task, "spec": spec, "metrics": metrics,
           "frames_dir": str(frames) if nf else None, "n_frames": nf, "log": log}
    (out / "sim_result.json").write_text(json.dumps(res, indent=2))
    print(f"[mujoco] {'driven' if driver is not None else 'static'}: "
          f"input_travel={metrics.get('input_travel')} "
          f"moved={metrics.get('moved_count')}/{metrics.get('watched_count')} "
          f"exploded={metrics.get('exploded')} frames={nf} -> {verdict}")
    return res


def main() -> int:
    ap = argparse.ArgumentParser(description="MuJoCo pure-contact scenario runner")
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--spec", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--task", default="")
    a = ap.parse_args()
    spec = json.loads(Path(a.spec).read_text())
    run(a.mjcf, spec, a.out, a.task)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
