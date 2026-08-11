"""Optional PyChrono backend seam over the shared KinematicModel.

PyChrono stays optional. This module always emits an auditable bundle/manifest; when the
runtime is unavailable it returns status=unavailable instead of blaming the generated CAD.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from .physics_contract import normalize_result, write_json


def _chrono_command() -> list[str]:
    import os
    configured = os.environ.get("CHRONO_PYTHON", "").strip()
    if configured and Path(configured).exists():
        return [configured]
    root = os.environ.get("MAMBA_ROOT_PREFIX", "").strip() or \
        r"C:\Users\t-zhijjiang\micromamba_root"
    wrapper = Path(root) / "condabin" / "micromamba.bat"
    env = Path(root) / "envs" / "chrono"
    if wrapper.exists() and env.exists():
        # The Windows .exe intermittently exits with 0xC0000409 before Python starts;
        # the installed activation wrapper reliably supplies Chrono's DLL search paths.
        return ["cmd.exe", "/c", "call", str(wrapper), "run", "-p", str(env), "python"]
    mamba = os.environ.get("MICROMAMBA_EXE", "").strip() or \
        r"C:\Users\t-zhijjiang\micromamba\micromamba.exe"
    if Path(mamba).exists():
        return [mamba, "run", "-r", root, "-n", "chrono", "python"]
    return []


def _quat_wxyz(T) -> list[float]:
    """Normalized w,x,y,z quaternion from a homogeneous transform."""
    import numpy as np
    r = np.asarray(T, dtype=float)[:3, :3]
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = [0.25*s, (r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s, (r[1,0]-r[0,1])/s]
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(max(1+r[0,0]-r[1,1]-r[2,2], 0.0))*2.0
            q = [(r[2,1]-r[1,2])/s, .25*s, (r[0,1]+r[1,0])/s, (r[0,2]+r[2,0])/s]
        elif i == 1:
            s = math.sqrt(max(1+r[1,1]-r[0,0]-r[2,2], 0.0))*2.0
            q = [(r[0,2]-r[2,0])/s, (r[0,1]+r[1,0])/s, .25*s, (r[1,2]+r[2,1])/s]
        else:
            s = math.sqrt(max(1+r[2,2]-r[0,0]-r[1,1], 0.0))*2.0
            q = [(r[1,0]-r[0,1])/s, (r[0,2]+r[2,0])/s, (r[1,2]+r[2,1])/s, .25*s]
    n = math.sqrt(sum(float(x)*float(x) for x in q)) or 1.0
    return [float(x)/n for x in q]


def _frame(T) -> dict:
    return {"xyz_m": [float(x) for x in T[:3, 3]], "quat_wxyz": _quat_wxyz(T)}


def _collision_policy(link, mode: str) -> dict:
    if mode != "contact_dynamic":
        return {"representation": "none", "reason": "ideal declared constraints"}
    if link.dof == "fixed":
        return {"representation": "triangle_mesh", "mesh_scale": 0.001,
                "concave": True, "swept_radius_m": 0.0,
                "reason": "static concave geometry"}
    # Dynamic concave triangle meshes can crash Chrono/Bullet below Python and have caused
    # host reboots on full gear trains. Fail closed until dynamic gears are represented by
    # semantic primitives or bounded convex tooth segments.
    return {"representation": "disabled_dynamic_triangle_mesh",
            "reason": "unsafe dynamic concave STL disabled",
            "fallback": "semantic_primitives_or_convex_tooth_segments",
            "fallback_enabled": False}


def _port_by_name(model, link: str, name: str):
    return next((p for p in (model.ports_by_link or {}).get(link, []) if p.name == name), None)


def _bundle(model, run_dir: str, mode: str) -> dict:
    import numpy as np
    from .mjcf_builder import _port_world_frame, _world_transforms
    W = _world_transforms(model)
    bodies = []
    for link in model.links:
        T = W.get(link.name)
        bodies.append({"source_kind": "link", "source_name": link.name,
                       "compiled": T is not None,
                       "rejected": T is None,
                       "reason": "" if T is not None else "missing world pose",
                       "mesh": link.mesh_filename or f"meshes/{link.name}.stl",
                       "material": link.material, "dof": link.dof,
                       "spin_axis": list(link.spin_axis),
                       "slide_axis": list(getattr(link, "slide_axis", (1, 0, 0))),
                       "driver": bool(link.driver),
                       "world_pose": _frame(T) if T is not None else None,
                       "collision": _collision_policy(link, mode)})
    ports = {}
    for link, entries in (getattr(model, "ports_by_link", None) or {}).items():
        T = W.get(link)
        ports[link] = []
        for p in entries:
            P = T @ _port_world_frame(p) if T is not None else None
            ports[link].append({"name": p.name, "type": p.type,
                "xyz_mm": list(p.xyz_mm), "axis": list(p.axis),
                "diameter_mm": p.diameter_mm, "depth_mm": p.depth_mm,
                "pitch_radius_mm": p.pitch_radius_mm, "normal_sign": p.normal_sign,
                "world_frame": _frame(P) if P is not None else None})
    constraints = []
    for j in (getattr(model, "motion_joints", None) or []):
        T = W.get(j.child)
        ok = T is not None
        axis = np.asarray(j.axis, dtype=float)
        n = float(np.linalg.norm(axis))
        ok = ok and n > 1e-12
        joint_frame = None
        if ok:
            pos = T[:3, 3] + T[:3, :3] @ (np.asarray(j.pos_mm, dtype=float) / 1000.0)
            world_axis = T[:3, :3] @ (axis / n)
            joint_frame = {"xyz_m": [float(x) for x in pos],
                           "axis_world": [float(x) for x in world_axis]}
        constraints.append({"source_kind": "motion_joint", "source_name": j.name,
            "compiled": ok, "rejected": not ok,
            "reason": "" if ok else "missing child pose or zero axis",
            "type": j.type, "parent": j.parent, "child": j.child,
            "axis": list(j.axis), "pos_mm": list(j.pos_mm),
            "world_frame": joint_frame})
    # A fixed accessory mounted on a moving carrier is a rigid motion edge even when the
    # author omitted an explicit MotionJointSpec (planet pins are the common case).
    explicit_children = {j.child for j in (getattr(model, "motion_joints", None) or [])}
    for link in model.links:
        if link.dof != "fixed" or not link.mount or link.name in explicit_children:
            continue
        T = W.get(link.name)
        constraints.append({"source_kind": "motion_joint",
            "source_name": f"{link.name}_mount_fixed", "compiled": T is not None,
            "rejected": T is None, "reason": "" if T is not None else "missing child pose",
            "type": "fixed", "parent": link.mount, "child": link.name,
            "axis": [0.0, 0.0, 1.0], "pos_mm": [0.0, 0.0, 0.0],
            "world_frame": {"xyz_m": list(T[:3, 3]), "axis_world": [0.0, 0.0, 1.0]}
                           if T is not None else None})
    motion_pairs = {(j.parent, j.child) for j in (getattr(model, "motion_joints", None) or [])}
    motion_pairs.update((r["parent"], r["child"]) for r in constraints
                        if r["source_kind"] == "motion_joint")
    motion_children = {r["child"] for r in constraints
                       if r["source_kind"] == "motion_joint"}
    planetary_planets = {p.get("gear", "")
                         for stage in (getattr(model, "planetary_stages", None) or [])
                         for p in stage.planets}
    for r in (getattr(model, "relations", None) or []):
        duplicate = ((r.base_part, r.incoming_part) in motion_pairs or
                     (r.incoming_part, r.base_part) in motion_pairs or
                     (r.incoming_part in motion_children
                      and r.incoming_part in planetary_planets
                      and r.mate_type in {"pin", "revolute", "journal_bearing"}))
        bp = _port_by_name(model, r.base_part, r.base_port)
        ip = _port_by_name(model, r.incoming_part, r.incoming_port)
        supported = r.mate_type in {"pin", "revolute", "journal_bearing",
                                    "cylindrical", "coaxial", "press_fit",
                                    "fixed", "weld", "welded", "bolted"}
        compiled = not duplicate and supported and bp is not None and ip is not None
        reason = ("represented by motion joint" if duplicate else
                  f"unsupported relation type '{r.mate_type}'" if not supported else
                  "missing named relation port" if bp is None or ip is None else "")
        Tb, Ti = W.get(r.base_part), W.get(r.incoming_part)
        constraints.append({"source_kind": "relation", "source_name": r.name,
                            "compiled": compiled, "rejected": not compiled and not duplicate,
                            "type": r.mate_type, "body_a": r.base_part,
                            "port_a": r.base_port, "body_b": r.incoming_part,
                            "port_b": r.incoming_port,
                            "frame_a": _frame(Tb @ _port_world_frame(bp))
                                       if bp is not None and Tb is not None else None,
                            "frame_b": _frame(Ti @ _port_world_frame(ip))
                                       if ip is not None and Ti is not None else None,
                            "reason": reason})
    transmissions = []
    for t in (getattr(model, "transmissions", None) or []):
        supported = t.type in {"gear_external", "gear_internal", "compound_1to1"}
        compiled = supported and (mode == "ideal_dynamic" or t.type == "compound_1to1")
        reason = (f"unsupported transmission type '{t.type}'" if not supported else
                  "contact_dynamic uses physical contact" if not compiled else "")
        transmissions.append({"source_kind": "transmission", "source_name": t.name,
                              "compiled": compiled, "rejected": not supported,
                              "type": t.type, "driving_link": t.driving_link,
                              "driven_link": t.driven_link, "ratio": t.ratio,
                              "reason": reason})
    stages = []
    for s in (getattr(model, "planetary_stages", None) or []):
        valid = min(s.sun_teeth, s.planet_teeth, s.ring_teeth) > 0 \
            and s.ring_teeth == s.sun_teeth + 2*s.planet_teeth
        compiled = valid
        reason = "" if valid else "invalid planetary tooth-count identity"
        stages.append({"source_kind": "planetary_stage", "source_name": s.name,
            "compiled": compiled, "rejected": not compiled, "reason": reason,
            "lowering": "willis_constraint" if mode == "ideal_dynamic" else "mesh_contact",
            "sun": s.sun, "ring": s.ring, "carrier": s.carrier,
            "planets": list(s.planets), "sun_teeth": s.sun_teeth,
            "planet_teeth": s.planet_teeth, "ring_teeth": s.ring_teeth,
            "fixed_member": s.fixed_member, "input_member": s.input_member,
            "output_member": s.output_member})
    driver = next((l.name for l in model.links if l.driver), None)
    driver_coordinate = driver
    # A physical input may be a shaft rigidly pressed to the actual hinged gear. Follow
    # compound locks to the body that owns a motion coordinate; torque must be applied there.
    motion_children = {j.child for j in (getattr(model, "motion_joints", None) or [])}
    changed = True
    while changed and driver_coordinate not in motion_children:
        changed = False
        for t in transmissions:
            if not t["compiled"] or t["type"] != "compound_1to1":
                continue
            if t["driving_link"] == driver_coordinate:
                driver_coordinate, changed = t["driven_link"], True
                break
            if t["driven_link"] == driver_coordinate:
                driver_coordinate, changed = t["driving_link"], True
                break
    excludes = [{"body_a": r.base_part, "body_b": r.incoming_part,
                 "reason": r.mate_type}
                for r in (getattr(model, "relations", None) or [])
                if r.mate_type in {"press_fit", "fixed", "weld", "welded", "bolted"}]
    return {"bundle_version": 2, "engine": "chrono", "mode": mode,
            "units": {"length": "m", "mesh_source_length": "mm", "mass": "kg",
                      "angle": "rad", "time": "s"},
            "bodies": bodies, "ports_by_link": ports, "constraints": constraints,
            "transmissions": transmissions, "planetary_stages": stages,
            "contact_excludes": excludes,
            "collision_policy": {"static": "direct_triangle_mesh",
                                 "dynamic": "disabled_until_safe_segmentation",
                                 "convex_decomposition": "explicit_disabled_fallback"},
            "driver": driver, "driver_coordinate": driver_coordinate,
            "output_link": model.output_link, "watch_links": list(model.watch_links),
            "run_dir": str(run_dir)}


def _diagnose_result(result: dict, model, mode: str) -> dict | None:
    """Route deterministic Chrono failures without asking the CAD agent to fix harness bugs."""
    if result.get("passed") is not False or result.get("diagnosis"):
        return result.get("diagnosis")
    from evaluator.attribution import diagnosis

    metrics = result.get("metrics") or {}
    if metrics.get("compile_errors"):
        return diagnosis("builder_compiler", "chrono_lowering_failed",
                         "Chrono could not lower authored mechanical semantics.",
                         verified=True,
                         evidence=[{"kind": "compile_errors",
                                    "observation": metrics["compile_errors"]}],
                         culprits=metrics["compile_errors"])
    health = metrics.get("numerical_health") or {}
    if health.get("finite") is False or health.get("exploded") or not metrics.get(
            "constraints_healthy", True):
        return diagnosis("simulator_numerics", "chrono_unhealthy_simulation",
                         result.get("reason") or "Chrono constraint solve was unhealthy.",
                         verified=True,
                         evidence=[{"kind": "numerical_health", "observation": health},
                                   {"kind": "constraint_residual",
                                    "observation": metrics.get("max_constraint_residual")}])
    unsafe = metrics.get("unsafe_collision_bodies") or []
    if unsafe:
        return diagnosis("simulator_numerics", "unsafe_collision_representation",
                         "Dynamic concave triangle meshes are disabled for Chrono safety.",
                         verified=True,
                         evidence=[{"kind": "disabled_collision_bodies",
                                    "observation": unsafe}])
    if mode == "ideal_dynamic" and not metrics.get("transmission_healthy", True):
        return diagnosis("evaluator", "declared_ratio_check_failed",
                         "Deterministic ratio check disagrees with the compiled ideal stage.",
                         verified=True,
                         evidence=[{"kind": "transmission_checks",
                                    "observation": metrics.get("transmission_checks") or []}])
    if mode == "contact_dynamic" and not metrics.get("contact_mesh_healthy", True):
        pairs = [x.get("pair") for x in metrics.get("contact_pair_checks") or []
                 if not x.get("observed")]
        return diagnosis("agent_geometry", "declared_mesh_contact_missing",
                         "Expected physical gear contact pairs were not observed.",
                         verified=True,
                         evidence=[{"kind": "missing_contact_pairs", "observation": pairs}],
                         culprits=[{"kind": "contact_pair", "name": "/".join(x)}
                                   for x in pairs if x])
    if not metrics.get("output_moved", False):
        culprits = [{"kind": "link", "name": name} for name in
                    (model.output_link, next((x.name for x in model.links if x.driver), ""))
                    if name]
        return diagnosis("agent_ir", "output_not_reached",
                         "Finite input torque did not reach the declared output link.",
                         verified=True,
                         evidence=[{"kind": "joint_coordinates",
                                    "observation": metrics.get("joint_coordinates") or {}}],
                         culprits=culprits)
    return diagnosis("evaluator", "unclassified_chrono_failure",
                     result.get("reason") or "Chrono failed without an agent-authorized cause.",
                     verified=False)


def run_chrono_backend(model, urdf_path: str, task: str, run_dir: str, settings,
                       *, iteration=None, log_fn=print) -> dict:
    mode = getattr(settings, "chrono_mode", "ideal_dynamic")
    sub = "chrono" if iteration is None else f"chrono_{iteration}"
    out = Path(run_dir) / "physics" / sub
    out.mkdir(parents=True, exist_ok=True)
    bundle = _bundle(model, run_dir, mode)
    bundle_path = write_json(out / "chrono_bundle.json", bundle)
    unsupported = [{"kind": rec.get("source_kind"), "name": rec.get("source_name"),
                    "reason": rec.get("reason")}
                   for key in ("bodies", "constraints", "transmissions", "planetary_stages")
                   for rec in bundle[key] if rec.get("rejected")]
    manifest = {"manifest_version": 2, **bundle, "backend_model": bundle_path,
                "unsupported": unsupported, "warnings": []}
    manifest_path = write_json(out / "builder_manifest.json", manifest)
    chrono_command = _chrono_command()
    if not chrono_command:
        e = RuntimeError("CHRONO_PYTHON is unset and no sidecar environment was found")
        result = {"passed": None, "verdict": "UNAVAILABLE",
                  "summary": "PyChrono is not installed; generated artifact was not judged.",
                  "metrics": {"numerical_health": {"finite": True}}, "tests": [],
                  "cause": "backend", "reason": f"{type(e).__name__}: {e}",
                  "diagnosis": {"diagnosis_version": 2,
                      "fault_domain": "simulator_numerics", "fault_code": "backend_unavailable",
                      "verified": True, "confidence": 1.0, "culprit_entities": [],
                      "evidence": [{"kind": "import_error", "observation": str(e)}],
                      "routing": {"action": "halt_harness", "allow_agent_refinement": False}}}
        result = normalize_result(result, engine="chrono", mode=mode, run_dir=run_dir,
                                  model=model, status="unavailable",
                                  manifest_path=manifest_path)
        write_json(out / "sim_result.json", result)
        log_fn("[chrono] unavailable: PyChrono optional dependency is not installed")
        return result
    import subprocess, sys
    spec = {"duration_s": 1.0, "settle_s": 0.2, "timestep_s": 0.001,
            "drive": {"target_accel_rad_s2": 2.0}, "task": task}
    spec_path = write_json(out / "spec.json", spec)
    runner = Path(__file__).resolve().parents[1] / "evaluator" / "run_scenario_chrono.py"
    proc = subprocess.run([*chrono_command, str(runner), "--bundle", bundle_path,
                           "--spec", spec_path, "--out", str(out)],
                          capture_output=True, text=True, timeout=600)
    result_path = out / "sim_result.json"
    if proc.returncode != 0 or not result_path.exists():
        result = {"passed": None, "verdict": "UNAVAILABLE",
                  "summary": "Chrono sidecar runner failed.", "metrics": {}, "tests": [],
                  "cause": "backend", "reason": (proc.stderr or proc.stdout)[-2000:]}
        status = "runtime_failed"
    else:
        result = json.loads(result_path.read_text(encoding="utf-8")); status = "completed"
        frames_dir = result.get("frames_dir")
        if frames_dir:
            try:
                from evaluator.diagnose import encode_mp4
                mp4 = encode_mp4(frames_dir, str(out/"model.mp4"))
                if mp4:
                    result["video"] = f"physics/{sub}/model.mp4"
            except Exception as exc:
                log_fn(f"[chrono] trajectory video unavailable ({exc})")
    result["diagnosis"] = _diagnose_result(result, model, mode)
    result = normalize_result(result, engine="chrono", mode=mode, run_dir=run_dir,
                              model=model, status=status, manifest_path=manifest_path)
    write_json(result_path, result)
    log_fn(f"[chrono] sidecar rc={proc.returncode} status={status}")
    return result
