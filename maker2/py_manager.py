"""Stage-1 of the Python-authoring pipeline (方案B): evaluate a MANAGER-authored
parametric build123d module into a KinematicModel with GLOBAL poses.

The manager no longer emits a connection-graph JSON that mate_solver solves. Instead it
writes ONE Python module that:
  * imports the boss parameter module (``params``) for all dimensions,
  * defines ``build_subassembly()`` returning a cadpy ``AssemblyHelper`` (or its built
    Compound) where every part is added with a NAME and mated by named cadpy frames, the
    whole sub anchored to its global params frame, and a metadata dict carrying the
    KinematicModel fields the downstream needs: ``dof`` (fixed|spin|free), ``spin_axis``,
    ``driver``, and, for a gear, ``mesh_role``/``mesh_id`` so mesh_pairs can be recovered.

This module runs that authored Python in the SAME sandboxed subprocess pattern the worker
uses (never importing the CAD kernel in-process), extracts each leaf's accumulated world
transform + metadata + exports its STL, and assembles a ``KinematicModel`` whose poses are
GLOBAL (parent="" root-relative). Because the manager authored global coordinates, the
downstream libslvs cross-sub solve becomes unnecessary (see plan 方案B, assembler shrink).

The parent process only ever touches JSON + STL/STEP paths; build123d/OCCT stays in the child.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from .model import KinematicModel, LinkSpec, PoseSpec

_EXEC_TIMEOUT = 240  # a whole-subassembly assembly solve is heavier than one part


class PyManagerError(ValueError):
    """The authored manager Python failed to evaluate into a valid subassembly."""


# Subprocess body: exec the authored module, build the cadpy AssemblyHelper/Compound, then for
# every leaf emit {name, world 4x4, metadata} + export its STL. Emits ONE json line.
# The build123d/cadpy subprocess body lives in a sibling file (avoids nested triple-quote
# escaping) and is read at import time. It exec's the manager module, builds the cadpy
# Compound, and emits per-part {name,R,T,metadata,stl,volume,bbox} as one JSON line.
_EVAL_RUNNER = (Path(__file__).parent / "_eval_runner_b123.py").read_text(encoding="utf-8")


def _params_public_names(params_text: str) -> list:
    """Top-level public names (functions + constants) the params module defines, so an
    AttributeError can tell the manager exactly what it may call. Skips private `_` names."""
    import re as _re
    names = []
    for m in _re.finditer(r"^(?:def\s+([A-Za-z]\w*)\s*\(|([A-Za-z]\w*)\s*=)", params_text,
                          _re.MULTILINE):
        nm = m.group(1) or m.group(2)
        if nm and not nm.startswith("_") and nm not in names:
            names.append(nm)
    return names


def _rot_to_rpy(R):
    """3x3 rotation -> (roll, pitch, yaw) XYZ, radians. Best-effort, gimbal-safe enough."""
    import math
    sy = math.hypot(R[0][0], R[1][0])
    if sy > 1e-9:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0.0
    return (roll, pitch, yaw)


def evaluate_manager_python(script_text: str, run_dir: str, sub_name: str,
                            *, params_text: str = "", frames=None, log_fn=print) -> KinematicModel:
    """Run the manager-authored build123d module in a sandbox, export each part's STL, and
    build a KinematicModel whose poses are GLOBAL (root-relative, meters). Raises
    PyManagerError on any failure so the caller's retry/debug loop can react.

    ``frames`` (v3, optional): the boss's interface frames (list of MountFrame with
    ``name``/``xyz_m`` in global meters). When a built part's metadata tags a ``frame`` that
    matches one, we VERIFY the manager's params-derived location coincides with the boss's
    frame coordinate (a pure consistency guard — it does NOT overwrite the coordinate, per the
    'precheck backstops, does not override' choice). A drift beyond tolerance raises
    PyManagerError so the debugger rewrites the offending params call."""
    run = Path(run_dir)
    run.mkdir(parents=True, exist_ok=True)
    (run / "meshes").mkdir(exist_ok=True)
    # Persist params + the authored module side by side so `import params` resolves (cwd=run).
    # In 方案B the manager module always `import params`; an empty params_text means the boss
    # failed to emit the shared params block. Fail fast with a clear message instead of letting
    # every manager attempt collapse into an opaque `ModuleNotFoundError: No module named
    # 'params'` (which burns all retries). The boss-side gate (ERR_PARAMS_MISSING) should catch
    # this first; this is the backstop.
    if not (params_text or "").strip():
        raise PyManagerError(
            "no params module provided — the boss did not emit a ```python params block, so "
            "`import params` in the manager module would fail. Re-plan the boss to author params.")
    (run / "params.py").write_text(params_text, encoding="utf-8")
    src = run / "manager_sub.py"
    src.write_text(script_text, encoding="utf-8")
    runner = run / "_cq_eval_runner.py"
    runner.write_text(_EVAL_RUNNER, encoding="utf-8")
    out_json = run / "sub_eval.json"

    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        r = subprocess.run(
            [sys.executable, str(runner), str(src), str(out_json), str(run / "meshes")],
            capture_output=True, text=True, timeout=_EXEC_TIMEOUT, cwd=str(run), env=env)
    except subprocess.TimeoutExpired:
        raise PyManagerError(f"manager build123d eval timed out after {_EXEC_TIMEOUT}s")
    except Exception as e:
        raise PyManagerError(f"eval subprocess failed: {type(e).__name__}: {e}")

    payload = None
    for line in reversed((r.stdout or "").strip().splitlines()):
        line = line.strip()
        if line.startswith("{"):
            try:
                payload = json.loads(line)
                break
            except Exception:
                continue
    if payload is None or not payload.get("ok"):
        err = (payload or {}).get("error") if payload else None
        tail = (r.stderr or r.stdout or "").strip()[-400:]
        msg = f"manager build123d eval failed: {err or tail}"
        # If the manager called a params name that doesn't exist, tell it EXACTLY what params
        # DOES define so it fixes the call in one shot instead of guessing across retries.
        if err and "has no attribute" in err and params_text:
            names = _params_public_names(params_text)
            if names:
                msg += ("\nThe `params` module defines ONLY these names — call one of these (or "
                        "compose them with params.add/params.mul), never invent a params name: "
                        + ", ".join(names))
        raise PyManagerError(msg)
    if not out_json.exists():
        raise PyManagerError("manager eval produced no sub_eval.json")

    spec = json.loads(out_json.read_text(encoding="utf-8"))
    parts = spec.get("parts") or []
    if not parts:
        raise PyManagerError("manager assembly has no parts with solids")

    # v3 consistency guard: index the boss's interface frames by name so a part that tags a
    # `frame` in its metadata can be checked against the boss's authoritative coordinate.
    frames_by_name: dict = {}
    for fr in (frames or []):
        nm = getattr(fr, "name", None)
        if nm:
            frames_by_name[str(nm)] = fr
    _GUARD_TOL_M = 0.002  # 2 mm: params-derived loc must coincide with the boss frame

    links: list[LinkSpec] = []
    poses: list[PoseSpec] = []
    mesh_by_id: dict = {}
    coord_log: list = []
    root = spec.get("root") or parts[0]["name"]
    for p in parts:
        name = p["name"]
        meta = p.get("metadata") or {}
        if float(p.get("volume_mm3", 0.0)) <= 0.0:
            raise PyManagerError(f"part '{name}' built an empty/zero-volume solid")
        links.append(LinkSpec(
            name=name, description=meta.get("description", ""),
            shape_hint=meta.get("shape_hint", ""),
            mesh_filename=f"meshes/{name}.stl",
            dof=str(meta.get("dof", "fixed")),
            spin_axis=tuple(meta.get("spin_axis", (0.0, 0.0, 1.0))),
            driver=bool(meta.get("driver", False)),
            material=str(meta.get("material", "steel"))))
        # GLOBAL pose: root-relative. T is mm -> meters.
        T = [float(v) / 1000.0 for v in p["T"]]
        rpy = _rot_to_rpy(p["R"])
        # v3 guard: if the part declares which interface frame it realizes, verify the manager's
        # params-derived coordinate matches the boss's frame — a params call gone wrong (wrong
        # function, wrong axis) surfaces HERE as a clear per-sub error, not later as a precheck
        # weld gap. Pure check: the coordinate is NOT overwritten.
        fr_tag = meta.get("frame")
        if fr_tag and str(fr_tag) in frames_by_name:
            bf = frames_by_name[str(fr_tag)]
            bx = tuple(float(v) for v in getattr(bf, "xyz_m", (0.0, 0.0, 0.0)))
            drift = sum((a - b) ** 2 for a, b in zip(T, bx)) ** 0.5
            if drift > _GUARD_TOL_M:
                raise PyManagerError(
                    f"part '{name}' claims frame '{fr_tag}' but its params-derived location "
                    f"{tuple(round(v, 4) for v in T)} m is {drift*1000:.1f} mm from the boss "
                    f"frame {tuple(round(v, 4) for v in bx)} m — recompute its `loc` from "
                    f"`params.{fr_tag}()` (do not type a coordinate).")
            # axis guard: the part is built with its revolution axis along local +Z, so its
            # REALIZED world axis is R·[0,0,1] = the 3rd column of R. It must match the frame's
            # params axis. A mismatch means the manager oriented the part by hand / with a rotating
            # Location instead of anchoring via the params-axis Plane.
            bax = getattr(bf, "axis", None)
            if bax is not None:
                bv = [float(v) for v in bax]
                bn = sum(v * v for v in bv) ** 0.5
                if bn > 1e-9:
                    bv = [v / bn for v in bv]
                    R = p["R"]
                    realized = [R[0][2], R[1][2], R[2][2]]
                    rn = sum(v * v for v in realized) ** 0.5 or 1.0
                    realized = [v / rn for v in realized]
                    cosang = sum(a * b for a, b in zip(realized, bv))
                    import math as _m
                    # axis of revolution is UNSIGNED: +v and -v are the same physical axis, so
                    # compare parallelism via |cos| (0° or 180° both mean aligned).
                    align_deg = _m.degrees(_m.acos(max(-1.0, min(1.0, abs(cosang)))))
                    if align_deg > 5.0:      # 5° tolerance
                        raise PyManagerError(
                            f"part '{name}' claims frame '{fr_tag}' but its realized spin axis "
                            f"{tuple(round(v,3) for v in realized)} is {align_deg:.0f}° off the "
                            f"frame axis {tuple(round(v,3) for v in bv)} — build the part with its "
                            f"axis along local +Z and anchor the sub with "
                            f"`.moved(Plane(origin=params.{fr_tag}(), z_dir=params.{fr_tag}_axis())"
                            f".location)`; do NOT hand-rotate it or pass a rotating Location.")
        poses.append(PoseSpec(name=f"place_{name}", parent="", child=name,
                              xyz_m=tuple(T), rpy_rad=tuple(rpy)))
        coord_log.append(f"{name}@{tuple(round(v*1000, 1) for v in T)}mm")
        mid = meta.get("mesh_id")
        if mid:
            mesh_by_id.setdefault(str(mid), []).append(name)

    mesh_pairs = [tuple(v[:2]) for v in mesh_by_id.values() if len(v) >= 2]

    model = KinematicModel(name=sub_name, root_link=root, links=links, poses=poses,
                           mesh_pairs=mesh_pairs)
    if log_fn:
        log_fn(f"[py-manager] {sub_name}: {len(links)} part(s), {len(mesh_pairs)} mesh pair(s), "
               f"global poses from params: {', '.join(coord_log)}")

    # 补精量: if the CAD skill's inspect tool is available and the runner exported a sub-level STEP,
    # run a cheap single-file sanity probe (real non-empty solid, plausible bounding box). This is
    # a DIAGNOSTIC complement to geocheck — it never gates or overrides the KinematicModel, matching
    # the 'precheck backstops, does not override' choice. Skipped silently if the skill is absent.
    try:
        from . import skill_inspect
        step_path = run / "sub.step"
        if step_path.exists() and skill_inspect.available():
            facts = skill_inspect.entry_facts(str(step_path))
            if facts.get("ok") and log_fn:
                ef = facts.get("entryFacts", {})
                sm = facts.get("summary", {})
                log_fn(f"[py-manager] {sub_name}: inspect sub.step -> "
                       f"size={ef.get('size')} faces={sm.get('faceCount')} "
                       f"solids={sm.get('shapeCount')}")
    except Exception:
        pass
    return model
