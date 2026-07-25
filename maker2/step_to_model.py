"""Convert a text-to-cad STEP scene into a maker2 KinematicModel.

The single-agent text-to-cad path authors ONE whole-machine build123d script whose
``gen_step()`` returns a cadpy Compound; ``scripts/step`` exports it to a STEP file whose
XCAF tree carries every part's NAME, world TRANSFORM and COLOR. This module re-reads that
STEP with cadpy's ``load_step_scene``, exports each leaf occurrence's mesh as a per-part
STL (in the part's LOCAL frame), and reads each occurrence's world transform into a
``PoseSpec`` — producing the same ``KinematicModel`` the multi-manager path emits, so the
whole downstream (precheck / MuJoCo physics / URDF / UI) is reused unchanged.

DOF / driver / mesh-pair metadata: cadpy encodes joints in the compound's
``assembly_mates`` and the agent tags parts by name convention; we map a spin/free hint and
a ``mesh_id`` suffix from the part name (the agent is instructed to name accordingly), and
recover gear pairs from equal ``mesh_id`` tags. This keeps the STEP the single source of
geometry while the KinematicModel stays the integration contract.

Runs in the SAME sandboxed-subprocess spirit as py_manager: cadpy/OCCT are imported here,
so the caller invokes this module in a child process (see single_agent runner).
"""
from __future__ import annotations

import math
import re
from pathlib import Path

from .model import KinematicModel, LinkSpec, PoseSpec

_URDF_SAFE = re.compile(r"^[a-z][a-z0-9_]*$")
# A gear part the agent means to mesh is named "<role>__mesh_<id>" (double underscore
# so the id survives slugify); two parts sharing <id> are a mesh pair.
_MESH_RE = re.compile(r"__mesh[_-]?([a-z0-9]+)$", re.I)
_SPIN_RE = re.compile(r"gear|pinion|wheel|arbor|shaft|rotor|cam|spindle", re.I)
_DRIVER_RE = re.compile(r"driver|input|barrel|crank|winding", re.I)


def _slug(name: str, fallback: str) -> str:
    s = re.sub(r"[^a-z0-9_]+", "_", str(name or "").strip().lower())
    s = re.sub(r"_+", "_", s).strip("_")
    return s if _URDF_SAFE.match(s) else fallback


def _rot_to_rpy(R):
    """Row-major 3x3 (from the 16-float world transform) -> (roll, pitch, yaw) XYZ, rad."""
    sy = math.hypot(R[0][0], R[1][0])
    if sy > 1e-9:
        return (math.atan2(R[2][1], R[2][2]),
                math.atan2(-R[2][0], sy),
                math.atan2(R[1][0], R[0][0]))
    return (math.atan2(-R[1][2], R[1][1]), math.atan2(-R[2][0], sy), 0.0)


def _tessellate(shape, linear=0.1, angular=0.5):
    """OCCT StlAPI_Writer needs a triangulated shape; a raw B-rep prototype has none.
    Run an incremental mesh in place so the subsequent STL write succeeds."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    s = getattr(shape, "wrapped", shape)  # build123d wraps the TopoDS_Shape
    BRepMesh_IncrementalMesh(s, linear, False, angular, True)


def _iter_leaves(nodes):
    """Yield every leaf occurrence (a node with a prototype shape and no children)."""
    for n in nodes:
        kids = getattr(n, "children", None) or []
        if kids:
            yield from _iter_leaves(kids)
        elif getattr(n, "prototype_key", None) is not None:
            yield n


def step_to_kinematic_model(step_path: str, meshes_dir: str, model_name: str,
                            *, log_fn=print) -> KinematicModel:
    """Load `step_path`, export each leaf part's LOCAL-frame STL under `meshes_dir`, and
    return a KinematicModel with GLOBAL poses (each part's world transform from the STEP).
    Raises RuntimeError on an unreadable/empty scene."""
    from cadpy.step_scene import (load_step_scene, scene_occurrence_prototype_shape)
    from cadpy.stl import export_shape_stl

    scene = load_step_scene(Path(step_path))
    leaves = list(_iter_leaves(scene.roots))
    if not leaves:
        raise RuntimeError("STEP scene has no leaf parts with geometry")

    meshes = Path(meshes_dir)
    meshes.mkdir(parents=True, exist_ok=True)

    links: list[LinkSpec] = []
    poses: list[PoseSpec] = []
    mesh_by_id: dict = {}
    used: set = set()
    root_name = None

    for i, node in enumerate(leaves):
        raw = getattr(node, "name", None) or getattr(node, "source_name", None) or f"part_{i}"
        name = _slug(raw, f"part_{i}")
        while name in used:
            name = f"{name}_{i}"
        used.add(name)
        if root_name is None:
            root_name = name

        # Per-part STL from the PROTOTYPE shape (local frame; the world transform is the pose).
        proto = scene_occurrence_prototype_shape(scene, node)
        stl = meshes / f"{name}.stl"
        try:
            _tessellate(proto)
            export_shape_stl(proto, stl)
        except Exception as e:
            raise RuntimeError(f"part '{name}' STL export failed: {e}")

        # World transform: 16-float row-major (mm). Rotation 3x3 + translation mm->m.
        T = tuple(float(v) for v in (getattr(node, "transform", None) or ()))
        if len(T) >= 12:
            R = [[T[0], T[1], T[2]], [T[4], T[5], T[6]], [T[8], T[9], T[10]]]
            xyz = (T[3] / 1000.0, T[7] / 1000.0, T[11] / 1000.0)
            rpy = _rot_to_rpy(R)
        else:
            xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

        low = raw.lower()
        dof = "spin" if _SPIN_RE.search(low) else "fixed"
        driver = bool(_DRIVER_RE.search(low)) and dof == "spin"
        col = getattr(node, "color", None)
        color = tuple(getattr(col, c) for c in ("r", "g", "b", "a")) if col else ()

        links.append(LinkSpec(
            name=name, description=raw, mesh_filename=f"meshes/{name}.stl",
            dof=dof, spin_axis=(0.0, 0.0, 1.0), driver=driver, color=color))
        poses.append(PoseSpec(name=f"place_{name}", parent="", child=name,
                              xyz_m=xyz, rpy_rad=rpy))

        m = _MESH_RE.search(low)
        if m:
            mesh_by_id.setdefault(m.group(1), []).append(name)

    # One driver max: keep the first tagged, demote the rest to plain spin.
    seen_driver = False
    for l in links:
        if l.driver:
            if seen_driver:
                l.driver = False
            seen_driver = True

    mesh_pairs = [tuple(v[:2]) for v in mesh_by_id.values() if len(v) >= 2]
    model = KinematicModel(name=model_name, root_link=root_name or (links[0].name if links else ""),
                           links=links, poses=poses, mesh_pairs=mesh_pairs)
    if log_fn:
        log_fn(f"[step->model] {model_name}: {len(links)} part(s), "
               f"{len(mesh_pairs)} mesh pair(s) from STEP scene")
    return model
