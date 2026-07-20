"""Stage-1 of the Python-authoring pipeline (方案B): evaluate a MANAGER-authored
parametric CadQuery module into a KinematicModel with GLOBAL poses.

The manager no longer emits a connection-graph JSON that mate_solver solves. Instead it
writes ONE Python module that:
  * imports the boss parameter module (``params``) for all dimensions,
  * defines ``build_subassembly() -> cq.Assembly`` where every part is added to a
    ``cq.Assembly`` with a NAME, a global ``cq.Location`` (the part's placement in the
    subassembly's own frame), and a metadata dict carrying the KinematicModel fields the
    downstream needs: ``dof`` (fixed|spin|free), ``spin_axis``, ``driver``, and, for a
    gear, ``mesh_role``/``mesh_id`` so mesh_pairs can be recovered.

This module runs that authored Python in the SAME sandboxed subprocess pattern the cq
worker uses (never importing cadquery in-process), extracts each assembly child's world
transform + metadata + exports its STL, and assembles a ``KinematicModel`` whose poses are
GLOBAL (parent="" root-relative). Because the manager authored global coordinates, the
downstream libslvs cross-sub solve becomes unnecessary (see plan 方案B, assembler shrink).

The parent process only ever touches JSON + STL paths; cadquery/OCCT stays in the child.
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


# Subprocess body: exec the authored module, build the cq.Assembly, solve it, then for
# every leaf child emit {name, world 4x4, metadata} + export its STL. Emits ONE json line.
_EVAL_RUNNER = r'''
import sys, json, math
src_path, out_json, meshes_dir = sys.argv[1], sys.argv[2], sys.argv[3]
import os
os.makedirs(meshes_dir, exist_ok=True)
ns = {}
try:
    import cadquery as cq
    with open(src_path, "r", encoding="utf-8") as f:
        code = f.read()
    exec(compile(code, src_path, "exec"), ns)
    fn = ns.get("build_subassembly")
    if fn is None:
        print(json.dumps({"ok": False, "error": "build_subassembly() not defined"}))
        sys.exit(0)
    asm = fn()
    if not hasattr(asm, "traverse"):
        print(json.dumps({"ok": False, "error": "build_subassembly() must return a cq.Assembly"}))
        sys.exit(0)
    # Solve any constraints the author declared (no-op if positioned by loc=).
    try:
        asm.solve()
    except Exception:
        pass  # location-based placement needs no solve; constraint errors surface as bad geometry

    def _mat(loc):
        # world 4x4 (mm) of a cq.Location
        t = loc.toTuple() if hasattr(loc, "toTuple") else None
        # robust path: use the transform matrix
        m = loc.wrapped.Transformation()
        R = [[m.Value(r, c) for c in range(1, 4)] for r in range(1, 4)]
        T = [m.Value(r, 4) for r in range(1, 4)]
        return R, T

    parts = []
    root_name = None
    for child in asm.traverse():
        # asm.traverse() yields (name, Assembly) pairs; the top object has obj None-ish
        name, sub = child
        obj = getattr(sub, "obj", None)
        if obj is None:
            continue  # a grouping node with no solid
        loc = sub.loc
        R, Tv = _mat(loc)
        meta = dict(getattr(sub, "metadata", {}) or {})
        stl = os.path.join(meshes_dir, name + ".stl")
        shape = obj.val() if hasattr(obj, "val") else (obj.toCompound() if hasattr(obj, "toCompound") else obj)
        shape.exportStl(stl)
        bb = shape.BoundingBox()
        vol = 0.0
        try:
            vol = shape.Volume()
        except Exception:
            pass
        parts.append({"name": name, "R": R, "T": Tv, "metadata": meta,
                      "stl": os.path.relpath(stl, meshes_dir),
                      "volume_mm3": vol,
                      "bbox": [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax]})
        if root_name is None:
            root_name = name
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump({"ok": True, "parts": parts, "root": root_name}, f)
    print(json.dumps({"ok": True, "n_parts": len(parts)}))
except Exception as e:
    import traceback
    print(json.dumps({"ok": False, "error": (type(e).__name__ + ": " + str(e))[:400],
                      "trace": traceback.format_exc()[-800:]}))
'''


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
                            *, params_text: str = "", log_fn=print) -> KinematicModel:
    """Run the manager-authored CadQuery module in a sandbox, export each part's STL, and
    build a KinematicModel whose poses are GLOBAL (root-relative, meters). Raises
    PyManagerError on any failure so the caller's retry/debug loop can react."""
    run = Path(run_dir)
    run.mkdir(parents=True, exist_ok=True)
    (run / "meshes").mkdir(exist_ok=True)
    # Persist params + the authored module side by side so `import params` resolves (cwd=run).
    if params_text:
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
        raise PyManagerError(f"manager CadQuery eval timed out after {_EXEC_TIMEOUT}s")
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
        raise PyManagerError(f"manager CadQuery eval failed: {err or tail}")
    if not out_json.exists():
        raise PyManagerError("manager eval produced no sub_eval.json")

    spec = json.loads(out_json.read_text(encoding="utf-8"))
    parts = spec.get("parts") or []
    if not parts:
        raise PyManagerError("manager assembly has no parts with solids")

    links: list[LinkSpec] = []
    poses: list[PoseSpec] = []
    mesh_by_id: dict = {}
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
        poses.append(PoseSpec(name=f"place_{name}", parent="", child=name,
                              xyz_m=tuple(T), rpy_rad=tuple(rpy)))
        mid = meta.get("mesh_id")
        if mid:
            mesh_by_id.setdefault(str(mid), []).append(name)

    mesh_pairs = [tuple(v[:2]) for v in mesh_by_id.values() if len(v) >= 2]

    model = KinematicModel(name=sub_name, root_link=root, links=links, poses=poses,
                           mesh_pairs=mesh_pairs)
    if log_fn:
        log_fn(f"[py-manager] {sub_name}: {len(links)} part(s), {len(mesh_pairs)} mesh pair(s), "
               f"global poses authored")
    return model
