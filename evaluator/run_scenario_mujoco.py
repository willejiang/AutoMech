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


def _dump_initial_contacts(m, d, mj, out_dir, log) -> None:
    """Write the sim's OWN collision verdict at the design pose to contacts.json: every
    detected contact as {body1, body2, depth_mm (penetration, +ve = overlap), pos_mm,
    r1_mm/r2_mm (radial distance of the contact point to each body's local axis)}.

    depth tells HOW MUCH two parts interpenetrate as MuJoCo sees them; the radial r's
    tell WHERE — a contact at r≈bore-wall means the parts are too wide to fit (radial),
    a contact at r≈0 with the bodies coaxial means an axial end-face is poking through
    (axial), a contact out at the tooth radius of a mesh pair is normal meshing."""
    import json
    import numpy as np
    try:
        mj.mj_forward(m, d)
        mj.mj_step(m, d)
    except Exception as e:
        log.append(f"initial-contact dump: step failed ({e})")
        return
    rows = []
    for i in range(d.ncon):
        c = d.contact[i]
        b1 = int(m.geom_bodyid[c.geom1])
        b2 = int(m.geom_bodyid[c.geom2])
        if b1 == b2:
            continue
        n1 = m.body(b1).name or f"body{b1}"
        n2 = m.body(b2).name or f"body{b2}"
        depth = -float(c.dist) * 1000.0            # dist<0 => interpenetration
        if depth <= 0.02:                          # skip mere touching
            continue
        pos = np.asarray(c.pos) * 1000.0
        # radial distance of the contact point to each body's own vertical axis (its
        # body-frame origin projected to the x,y of the contact).
        def _r(bid):
            bp = np.asarray(d.xpos[bid]) * 1000.0
            return float(np.hypot(pos[0] - bp[0], pos[1] - bp[1]))
        rows.append({"body1": n1, "body2": n2, "depth_mm": round(depth, 3),
                     "pos_mm": [round(v, 2) for v in pos],
                     "r1_mm": round(_r(b1), 3), "r2_mm": round(_r(b2), 3)})
    # Collapse duplicate (multi-point) contacts to the deepest per body pair.
    best: dict = {}
    for r in rows:
        key = tuple(sorted((r["body1"], r["body2"])))
        if key not in best or r["depth_mm"] > best[key]["depth_mm"]:
            best[key] = r
    merged = sorted(best.values(), key=lambda r: r["depth_mm"], reverse=True)
    try:
        (Path(out_dir) / "contacts.json").write_text(
            json.dumps({"contacts": merged, "n_pairs": len(merged)}, indent=2))
        log.append(f"initial contacts: {len(merged)} interpenetrating pair(s)")
    except Exception as e:
        log.append(f"initial-contact dump: write failed ({e})")


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

    # Fixed framing camera: locked on the mechanism's settle position so a part that
    # flies off does NOT drag the auto-camera out to meters away (which shrinks the
    # whole machine to an unreadable dot — the "black video" symptom). Built once, after
    # settle, from the assembly's own bounding box; every frame renders through it.
    fixed_cam = None

    def capture(idx: int):
        if renderer is None:
            return False
        try:
            from PIL import Image
            if fixed_cam is not None:
                renderer.update_scene(d, camera=fixed_cam)
            else:
                renderer.update_scene(d)
            img = renderer.render()
            Image.fromarray(img).save(frames / f"rgb_{idx:04d}.png")
            return True
        except Exception:
            return False

    # ---- initial contact snapshot (BEFORE settle) ----
    # At the design pose, dump every contact MuJoCo actually detects: which two bodies,
    # how deep they interpenetrate, where, and the radial distance of the contact point
    # to each body's local axis. This is the GROUND TRUTH the diagnoser needs — the sim's
    # own collision verdict — so it can tell a real geometry fault (bore too small, wrong
    # coords) from a convex-hull artifact (a decomposed piece filling a bore, an axial
    # end-face poking a coaxial part) instead of guessing from keyframes.
    _dump_initial_contacts(m, d, mj, out, log)

    # ---- STAGE 1: stability under gravity (settle, NO drive) ----
    # A machine must first EXIST stably on the bench: dropped under gravity it should
    # settle and hold together, not fall apart or collapse. This is the precondition for
    # any function test — a thing that explodes while just sitting there transmits nothing.
    # Track each body's drift from its pre-settle pose; if anything flies off or the stack
    # sinks, stability FAILS and the diagnoser is told so (function data is then suspect).
    import numpy as _np
    xpos_pre = d.xpos.copy()
    settle_disp = _np.zeros(m.nbody)
    for _ in range(settle_steps):
        mj.mj_step(m, d)
        if len(d.xpos):
            dv = _np.linalg.norm(d.xpos - xpos_pre, axis=1)
            settle_disp = _np.maximum(settle_disp, dv)

    settle_max_disp = float(settle_disp.max()) if m.nbody > 1 else 0.0
    settle_end_z = float(d.xpos[:, 2].min()) if len(d.xpos) else 0.0
    settle_exploded = bool(settle_max_disp > 0.5)     # a part flew >0.5 m just settling
    _STABLE_DRIFT_M = 0.003
    settle_displaced = []
    try:
        for b in range(1, m.nbody):
            name = m.body(b).name
            disp = float(settle_disp[b])
            if name and disp > _STABLE_DRIFT_M:
                settle_displaced.append({"part": name, "disp_mm": round(disp * 1000, 2)})
        settle_displaced.sort(key=lambda e: e["disp_mm"], reverse=True)
        settle_displaced = settle_displaced[:8]
    except Exception:
        settle_displaced = []
    stability = {
        "verdict": "FAIL" if settle_exploded else "PASS",
        "exploded": settle_exploded,
        "max_disp_m": round(settle_max_disp, 4),
        "end_z": round(settle_end_z, 4),
        "displaced_parts": settle_displaced,
    }
    log.append(f"stability(settle): {stability['verdict']} max_disp={settle_max_disp:.3f}m "
               f"exploded={settle_exploded}")

    # Lock the framing camera on the settled assembly. The environment DESIGNER may
    # override azimuth/elevation/distance via spec.design.camera; a value of 0 (or a
    # missing key) means "auto-fit this aspect". lookat + distance use a ROBUST center
    # that ignores parts already ejected far from the pack, so a single body that flew
    # off during settle does NOT drag the frame out to infinity (the "tiny dot" symptom).
    cam_spec = ((spec.get("design") or {}).get("camera")) or {}

    def _cam(*keys):
        for k in keys:
            v = cam_spec.get(k)
            if v not in (None, 0, 0.0, ""):
                return float(v)
        return None

    try:
        if m.nbody > 1:
            pts = d.xpos[1:]                       # skip world body 0
            # Robust center: median position; keep only bodies within 5x the median
            # spread of it, so far-ejected parts don't define the bounding box.
            med = np.median(pts, axis=0)
            dists = np.linalg.norm(pts - med, axis=1)
            keep = pts[dists <= max(np.median(dists) * 5.0, 1e-6)] if len(pts) > 2 else pts
            if len(keep) == 0:
                keep = pts
            lo, hi = keep.min(axis=0), keep.max(axis=0)
            center = (lo + hi) / 2.0
            diag = float(np.linalg.norm(hi - lo))
            fixed_cam = mj.MjvCamera()
            fixed_cam.type = mj.mjtCamera.mjCAMERA_FREE
            fixed_cam.lookat[:] = center
            dist_scale = _cam("distance_scale") or 2.5
            fixed_cam.distance = max(diag * dist_scale, 0.05)
            fixed_cam.azimuth = _cam("azimuth_deg", "azimuth") or 90.0
            fixed_cam.elevation = _cam("elevation_deg", "elevation") or -25.0
            log.append(f"camera: az={fixed_cam.azimuth} el={fixed_cam.elevation} "
                       f"dist={fixed_cam.distance:.3f} "
                       f"({'designer' if cam_spec else 'auto'})")
    except Exception as e:
        log.append(f"fixed camera unavailable: {e}")

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
            # Name the part we ACTUALLY drove. Without this the diagnosis rendered
            # "input joint 'None' moved only 0.003 rad", which tells the agent nothing
            # about where to look — and the whole point of that message is to point at
            # a part. The driver is chosen from the model here, so this is the only
            # place that knows its real name.
            "input_joint": f"{driver.name}_spin",
            "input_part": driver.name,
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

    res = {"task": task, "spec": spec, "metrics": metrics, "stability": stability,
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
