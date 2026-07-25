"""Single-agent text-to-cad path: one agent authors the WHOLE machine as ONE build123d
script (no boss / no per-sub managers / no assembler), and this module evaluates it into a
maker2 KinematicModel — reusing the existing precheck / MuJoCo physics / URDF / UI unchanged.

``evaluate_machine_python(script_text, run_dir, machine_name)`` runs the authored
``build_machine()`` in the same sandboxed subprocess the multi-manager path uses
(``_eval_runner_machine.py``): build123d + cadpy AssemblyHelper + make_gear are injected, the
script's returned Compound is walked into per-part local STLs + world poses, and a whole
machine STEP (``machine.step``) is written so the text-to-cad inspect tools can run
selector-level self-checks in the modeling loop. The parts array (name / world 4x4 / STL /
volume / metadata) becomes a flat-global KinematicModel.
"""
from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

from .model import KinematicModel, LinkSpec, PoseSpec

_EXEC_TIMEOUT = 300  # a whole machine is heavier than one sub
_RUNNER = (Path(__file__).parent / "_eval_runner_machine.py").read_text(encoding="utf-8")
_MESH_RE = re.compile(r"__mesh[_-]?([a-z0-9]+)$", re.I)


class SingleAgentError(ValueError):
    """The authored whole-machine build123d script failed to evaluate."""


def _rot_to_rpy(R):
    sy = math.hypot(R[0][0], R[1][0])
    if sy > 1e-9:
        return (math.atan2(R[2][1], R[2][2]), math.atan2(-R[2][0], sy),
                math.atan2(R[1][0], R[0][0]))
    return (math.atan2(-R[1][2], R[1][1]), math.atan2(-R[2][0], sy), 0.0)


def evaluate_machine_python(script_text: str, run_dir: str, machine_name: str,
                            *, log_fn=print) -> KinematicModel:
    """Run the authored whole-machine build123d script in a sandbox and return a
    KinematicModel with GLOBAL poses (mm->m). Also writes machine.step next to the eval
    output for the self-check loop. Raises SingleAgentError on any failure (with the
    subprocess traceback tail) so the modeling loop can feed it back to the agent."""
    run = Path(run_dir)
    run.mkdir(parents=True, exist_ok=True)
    (run / "meshes").mkdir(exist_ok=True)
    src = run / "machine.py"
    src.write_text(script_text, encoding="utf-8")
    runner = run / "_machine_runner.py"
    runner.write_text(_RUNNER, encoding="utf-8")
    out_json = run / "machine_eval.json"

    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        r = subprocess.run(
            [sys.executable, runner.name, src.name, out_json.name, "meshes"],
            capture_output=True, text=True, timeout=_EXEC_TIMEOUT, cwd=str(run), env=env)
    except subprocess.TimeoutExpired:
        raise SingleAgentError(f"machine build123d eval timed out after {_EXEC_TIMEOUT}s")
    except Exception as e:
        raise SingleAgentError(f"eval subprocess failed: {type(e).__name__}: {e}")

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
        trace = (payload or {}).get("trace") if payload else ""
        tail = trace or (r.stderr or r.stdout or "").strip()[-400:]
        raise SingleAgentError(f"machine build123d eval failed: {err or tail}")
    if not out_json.exists():
        raise SingleAgentError("machine eval produced no machine_eval.json")

    spec = json.loads(out_json.read_text(encoding="utf-8"))
    parts = spec.get("parts") or []
    if not parts:
        raise SingleAgentError("machine has no parts with solids")

    links: list[LinkSpec] = []
    poses: list[PoseSpec] = []
    mesh_by_id: dict = {}
    root = spec.get("root") or parts[0]["name"]
    coord_log: list = []
    for p in parts:
        name = p["name"]
        meta = p.get("metadata") or {}
        if float(p.get("volume_mm3", 0.0)) <= 0.0:
            continue
        dof = str(meta.get("dof", "")) or _infer_dof(name)
        driver = bool(meta.get("driver", False)) or (_infer_driver(name) and dof == "spin")
        spin_axis = meta.get("spin_axis")
        if not isinstance(spin_axis, (list, tuple)) or len(spin_axis) != 3:
            spin_axis = (0.0, 0.0, 1.0)
        links.append(LinkSpec(
            name=name, description=meta.get("description", name),
            mesh_filename=f"meshes/{name}.stl",
            dof=dof, spin_axis=tuple(spin_axis), driver=driver,
            material=str(meta.get("material", "steel"))))
        T = [float(v) / 1000.0 for v in p["T"]]
        rpy = _rot_to_rpy(p["R"])
        poses.append(PoseSpec(name=f"place_{name}", parent="", child=name,
                              xyz_m=tuple(T), rpy_rad=tuple(rpy)))
        coord_log.append(f"{name}@{tuple(round(v*1000, 1) for v in T)}mm")
        mid = meta.get("mesh_id")
        if not mid:
            m = _MESH_RE.search(name.lower())
            mid = m.group(1) if m else None
        if mid:
            mesh_by_id.setdefault(str(mid), []).append(name)

    # One driver max.
    seen = False
    for l in links:
        if l.driver:
            if seen:
                l.driver = False
            seen = True

    mesh_pairs = [tuple(v[:2]) for v in mesh_by_id.values() if len(v) >= 2]
    model = KinematicModel(name=machine_name, root_link=root, links=links, poses=poses,
                           mesh_pairs=mesh_pairs)
    if log_fn:
        log_fn(f"[single-agent] {machine_name}: {len(links)} part(s), "
               f"{len(mesh_pairs)} mesh pair(s), STEP={'yes' if spec.get('step') or (run/'machine.step').exists() else 'no'}")
    return model


_SPIN_RE = re.compile(r"gear|pinion|wheel|arbor|shaft|rotor|cam|spindle", re.I)
_DRIVER_RE = re.compile(r"driver|input|barrel|crank|winding", re.I)


def _infer_dof(name: str) -> str:
    return "spin" if _SPIN_RE.search(name.lower()) else "fixed"


def _infer_driver(name: str) -> bool:
    return bool(_DRIVER_RE.search(name.lower()))
