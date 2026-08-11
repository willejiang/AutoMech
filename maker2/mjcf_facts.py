"""Backend-neutral measured facts for the agent-authored MJCF topology compiler."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

FACTS_VERSION = 2


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _quat_wxyz(transform) -> list[float]:
    import trimesh.transformations as tf
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(transform)[:3, :3]
    return [float(x) for x in tf.quaternion_from_matrix(matrix)]


def _frame(transform) -> dict:
    return {"xyz_m": [float(x) for x in transform[:3, 3]],
            "quat_wxyz": _quat_wxyz(transform),
            "matrix": np.asarray(transform, dtype=float).tolist()}


def _mesh_facts(path: Path, density: float) -> dict:
    import trimesh

    mesh = trimesh.load_mesh(path, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces):
        raise ValueError(f"mesh has no triangular faces: {path}")
    volume_mm3 = abs(float(mesh.volume))
    mass = max(volume_mm3 * 1e-9 * density, 1e-6)
    native_mass = abs(float(mesh.mass))
    inertia = np.asarray(mesh.moment_inertia, dtype=float)
    if native_mass > 0:
        inertia *= mass / native_mass * 1e-6
    else:
        inertia = np.diag([1e-8, 1e-8, 1e-8])
    return {
        "sha256": _sha256(path),
        "vertices": int(len(mesh.vertices)), "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "bounds_mm": np.asarray(mesh.bounds, dtype=float).tolist(),
        "extents_mm": np.asarray(mesh.extents, dtype=float).tolist(),
        "volume_m3": volume_mm3 * 1e-9, "mass_kg": mass,
        "com_m": (np.asarray(mesh.center_mass, dtype=float) / 1000.0).tolist(),
        "inertia_kg_m2": inertia.tolist(),
    }


def _entity_ids(model) -> list[str]:
    ids = [f"link/{x.name}" for x in model.links]
    ids += [f"pose/{x.name}" for x in model.poses]
    ids += [f"port/{link}/{port.name}" for link, ports in
            (getattr(model, "ports_by_link", None) or {}).items() for port in ports]
    ids += [f"relation/{x.name}" for x in (getattr(model, "relations", None) or [])]
    ids += [f"motion_joint/{x.name}" for x in
            (getattr(model, "motion_joints", None) or [])]
    ids += [f"transmission/{x.name}" for x in
            (getattr(model, "transmissions", None) or [])]
    ids += [f"planetary_stage/{x.name}" for x in
            (getattr(model, "planetary_stages", None) or [])]
    driver = next((x.name for x in model.links if x.driver), "")
    if driver:
        ids.append(f"role/driver/{driver}")
    if getattr(model, "output_link", ""):
        ids.append(f"role/output/{model.output_link}")
    ids += [f"role/watch/{x}" for x in (getattr(model, "watch_links", None) or [])]
    return ids


def extract_mjcf_facts(model, ctx, settings=None) -> dict:
    """Persist immutable IR, frame, mesh, mass and port facts without lowering semantics."""
    from .manager import model_to_dict
    from .materials import density_of, friction_of
    from .mjcf_builder import _port_world_frame, _world_transforms

    root = Path(ctx.run_dir)
    meshes = Path(ctx.meshes_dir)
    world = _world_transforms(model)
    links = {}
    for link in model.links:
        path = root / (link.mesh_filename or f"meshes/{link.name}.stl")
        if not path.exists():
            path = meshes / f"{link.name}.stl"
        transform = world.get(link.name)
        if transform is None:
            raise ValueError(f"link '{link.name}' has no world transform")
        density = density_of(link.material)
        measured = _mesh_facts(path, density)
        links[link.name] = {
            "entity_id": f"link/{link.name}", "world_frame": _frame(transform),
            "mesh_path": str(path.relative_to(root)).replace("\\", "/"),
            "material": link.material, "density_kg_m3": density,
            "friction": [float(x) for x in friction_of(link.material).split()],
            "dof": link.dof, "spin_axis": list(link.spin_axis),
            "slide_axis": list(getattr(link, "slide_axis", (1.0, 0.0, 0.0))),
            "driver": bool(link.driver), "mount": link.mount,
            "extra_mounts": list(link.extra_mounts or []), **measured,
        }

    ports = {}
    for link_name, entries in (getattr(model, "ports_by_link", None) or {}).items():
        transform = world.get(link_name)
        ports[link_name] = {}
        for port in entries:
            local = _port_world_frame(port)
            ports[link_name][port.name] = {
                "entity_id": f"port/{link_name}/{port.name}", "type": port.type,
                "local_frame": _frame(local), "world_frame": _frame(transform @ local),
                "axis": list(port.axis), "diameter_mm": float(port.diameter_mm),
                "depth_mm": float(port.depth_mm),
                "pitch_radius_mm": float(port.pitch_radius_mm),
                "normal_sign": float(port.normal_sign),
            }

    facts = {
        "facts_version": FACTS_VERSION, "run_dir": str(root),
        "units": {"model_length": "m", "mesh_source_length": "mm",
                  "mass": "kg", "angle": "rad"},
        "model": model_to_dict(model), "entity_ids": _entity_ids(model),
        "links": links, "ports": ports,
        "simulation": {"gravity": [0.0, 0.0, -9.81], "timestep": 0.0002,
                       "solver": "Newton", "iterations": 100,
                       "base_rests_on_plane": bool(getattr(
                           settings, "base_rests_on_plane", True))},
    }
    path = root / "mjcf_facts.json"
    path.write_text(json.dumps(facts, indent=2), encoding="utf-8")
    return facts


def facts_hash(facts: dict, *, prompt_version: int, policy_version: int,
               model_id: str) -> str:
    stable_facts = dict(facts)
    stable_facts.pop("run_dir", None)
    payload = {"facts": stable_facts, "prompt_version": prompt_version,
               "policy_version": policy_version, "model_id": model_id}
    return hashlib.sha256(json.dumps(payload, sort_keys=True,
                                     separators=(",", ":")).encode()).hexdigest()


def query_port_fit(facts: dict, a: str, port_a: str, b: str, port_b: str) -> dict:
    pa = facts["ports"].get(a, {}).get(port_a)
    pb = facts["ports"].get(b, {}).get(port_b)
    fact_id = f"fit/{a}/{port_a}/{b}/{port_b}"
    if not pa or not pb:
        return {"ok": False, "reason": "unknown port", "fact_id": fact_id}
    ma = np.asarray(pa["world_frame"]["matrix"], dtype=float)
    mb = np.asarray(pb["world_frame"]["matrix"], dtype=float)
    axis_a, axis_b = ma[:3, 2], mb[:3, 2]
    delta = mb[:3, 3] - ma[:3, 3]
    radial = float(np.linalg.norm(delta - np.dot(delta, axis_a) * axis_a))
    axial = float(abs(np.dot(delta, axis_a)))
    axis_cos = float(abs(np.dot(axis_a, axis_b) /
                         max(np.linalg.norm(axis_a)*np.linalg.norm(axis_b), 1e-12)))
    typed = {pa["type"]: pa, pb["type"]: pb}
    clearance = None
    if "shaft" in typed and "bore" in typed:
        clearance = typed["bore"]["diameter_mm"] - typed["shaft"]["diameter_mm"]
    return {"ok": True, "fact_id": fact_id, "radial_offset_mm": radial*1000.0,
            "axial_offset_mm": axial*1000.0, "axis_cos": axis_cos,
            "diametral_clearance_mm": clearance,
            "fit_sign": ("clearance" if clearance is not None and clearance > 0 else
                         "interference" if clearance is not None and clearance < 0 else
                         "exact" if clearance == 0 else "not_shaft_bore")}


def _world_aabb_mm(link: dict) -> tuple[np.ndarray, np.ndarray]:
    """Conservative placed AABB from local mesh bounds and the authored world frame."""
    bounds = np.asarray(link["bounds_mm"], dtype=float)
    corners = np.array([[x, y, z]
                        for x in bounds[:, 0]
                        for y in bounds[:, 1]
                        for z in bounds[:, 2]], dtype=float)
    transform = np.asarray(link["world_frame"]["matrix"], dtype=float)
    world = corners @ transform[:3, :3].T + transform[:3, 3] * 1000.0
    return world.min(axis=0), world.max(axis=0)


def aabb_surface_distance_mm(a: dict, b: dict) -> float:
    """Zero for touching/overlapping placed AABBs, Euclidean gap otherwise."""
    alo, ahi = _world_aabb_mm(a)
    blo, bhi = _world_aabb_mm(b)
    gap = np.maximum(np.maximum(alo - bhi, blo - ahi), 0.0)
    return float(np.linalg.norm(gap))


def query_pair_geometry(facts: dict, a: str, b: str) -> dict:
    fact_id = f"pair/{'/'.join(sorted((a,b)))}"
    if a not in facts["links"] or b not in facts["links"]:
        return {"ok": False, "reason": "unknown link", "fact_id": fact_id}
    import trimesh
    aa, bb = facts["links"][a], facts["links"][b]
    Ta = np.asarray(aa["world_frame"]["matrix"], dtype=float)
    Tb = np.asarray(bb["world_frame"]["matrix"], dtype=float)
    placed = []
    for row, transform in ((aa, Ta), (bb, Tb)):
        path = Path(facts["run_dir"]) / row["mesh_path"]
        mesh = trimesh.load_mesh(path, force="mesh", process=False).copy()
        world_mm = transform.copy(); world_mm[:3, 3] *= 1000.0
        mesh.apply_transform(world_mm); placed.append(mesh)
    ma, mb = placed
    aabb_lo = np.maximum(ma.bounds[0], mb.bounds[0])
    aabb_hi = np.minimum(ma.bounds[1], mb.bounds[1])
    aabb_overlap = np.maximum(aabb_hi-aabb_lo, 0.0)
    distance = None
    try:
        samples_a = ma.vertices[::max(1, len(ma.vertices)//500)]
        samples_b = mb.vertices[::max(1, len(mb.vertices)//500)]
        da = float(trimesh.proximity.closest_point(mb, samples_a)[1].min())
        db = float(trimesh.proximity.closest_point(ma, samples_b)[1].min())
        distance = min(da, db)
    except Exception:
        pass
    overlap_volume = None
    if np.all(aabb_hi > aabb_lo):
        try:
            inter = trimesh.boolean.intersection([ma, mb])
            overlap_volume = 0.0 if inter is None or inter.is_empty else abs(float(inter.volume))
        except Exception:
            pass
    return {"ok": True, "fact_id": fact_id,
            "origin_distance_mm": float(np.linalg.norm(Ta[:3, 3]-Tb[:3, 3])*1000.0),
            "surface_distance_mm": distance,
            "aabb_overlap_extents_mm": aabb_overlap.tolist(),
            "solid_overlap_mm3": overlap_volume,
            "relative_transform": (np.linalg.inv(Ta) @ Tb).tolist(),
            "a_extents_mm": aa["extents_mm"], "b_extents_mm": bb["extents_mm"]}
