"""Scorer-owned exact-solid geometry audit for archived final assemblies."""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any, Mapping

REPORT_VOLUME_MM3 = 1.0e-4
GEOMETRY_ANALYZER_VERSION = "comfort-geometry/1.0"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _rotation_rpy(values) -> Any:
    import numpy as np

    roll, pitch, yaw = (float(value) for value in values)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def _world_transforms(model: Mapping[str, Any]) -> dict[str, Any]:
    import numpy as np

    poses = {str(row["child"]): row for row in model.get("poses", ())
             if isinstance(row, Mapping) and row.get("child")}
    cache: dict[str, Any] = {"": np.eye(4)}
    visiting: set[str] = set()

    def world(link: str):
        if link in cache:
            return cache[link]
        if link in visiting:
            raise ValueError(f"cyclic pose tree at {link!r}")
        visiting.add(link)
        pose = poses.get(link)
        if pose is None:
            transform = np.eye(4)
        else:
            local = np.eye(4)
            local[:3, :3] = _rotation_rpy(pose.get("rpy_rad", (0, 0, 0)))
            local[:3, 3] = np.asarray(pose.get("xyz_m", (0, 0, 0)), dtype=float) * 1000.0
            transform = world(str(pose.get("parent", ""))) @ local
        visiting.remove(link)
        cache[link] = transform
        return transform

    return {str(row["name"]): world(str(row["name"]))
            for row in model.get("links", ()) if isinstance(row, Mapping) and row.get("name")}


def _relation_index(model: Mapping[str, Any]) -> dict[frozenset[str], Mapping[str, Any]]:
    return {
        frozenset((str(row["base_part"]), str(row["incoming_part"]))): row
        for row in model.get("relations", ())
        if isinstance(row, Mapping) and row.get("base_part") and row.get("incoming_part")
    }


def _mount_index(model: Mapping[str, Any]) -> dict[frozenset[str], str]:
    mounts: dict[frozenset[str], str] = {}
    for row in model.get("links", ()):
        if not isinstance(row, Mapping) or not row.get("name"):
            continue
        child = str(row["name"])
        parents = [row.get("mount"), *(row.get("extra_mounts") or ())]
        for parent in parents:
            if parent:
                mounts[frozenset((str(parent), child))] = f"mount/{parent}/{child}"
    return mounts


def _decision_index(manifest: Mapping[str, Any]) -> dict[frozenset[str], Mapping[str, Any]]:
    topology = manifest.get("topology_plan")
    decisions = topology.get("contact_decisions", ()) if isinstance(topology, Mapping) else ()
    return {frozenset(str(name) for name in row["pair"]): row
            for row in decisions if isinstance(row, Mapping)
            and isinstance(row.get("pair"), (list, tuple)) and len(row["pair"]) == 2}


def _exemption(pair: frozenset[str], relations: Mapping, mounts: Mapping,
               decisions: Mapping) -> tuple[str | None, str | None, str]:
    relation = relations.get(pair)
    if relation:
        mate = str(relation.get("mate_type", ""))
        relation_id = f"relation/{relation.get('name', '')}"
        if mate == "press_fit":
            return "authored_press_fit", relation_id, "kinematic_model"
        # A running bearing authorizes AABB nesting around a bore, not positive
        # material/material intersection. Exact positive solid overlap therefore remains
        # a geometry conflict rather than being erased by bearing semantics.
        if mate in {"gear_spur_external", "gear_spur_internal", "gear_external", "gear_internal"}:
            return "ideal_gear_mesh", relation_id, "kinematic_model"
        if mate in {"pin", "revolute"}:
            return "authored_pin_or_revolute", relation_id, "kinematic_model"
        if mate in {"rigid_carry", "rigid_mount", "fixed", "weld"}:
            return "authored_rigid_mount", relation_id, "kinematic_model"

    mount_id = mounts.get(pair)
    if mount_id:
        return "authored_rigid_mount", mount_id, "kinematic_model"

    decision = decisions.get(pair)
    if not decision or decision.get("action") != "exclude":
        return None, None, ""
    reason = str(decision.get("reason", "")).casefold()
    sources = decision.get("source_entity_ids") or ()
    evidence_id = next((str(item) for item in sources
                        if str(item).startswith("relation/")), "manifest_contact_decision")
    if "ideal" in reason and ("gear" in reason or "tooth" in reason or "mesh" in reason):
        return "ideal_gear_mesh", evidence_id, "builder_manifest"
    if "journal" in reason or "running bearing" in reason:
        return "running_bearing", evidence_id, "builder_manifest"
    if (("press" in reason or "interference" in reason)
            and ("rigid" in reason or "carried" in reason or "seat" in reason)):
        return "authored_press_fit", evidence_id, "builder_manifest"
    if "dedicated pin" in reason and ("rigid" in reason or "interference" in reason):
        return "authored_pin_or_revolute", evidence_id, "builder_manifest"
    return None, None, ""


def analyze_geometry(root: str | Path, model: Mapping[str, Any],
                     manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Measure every final-pose solid intersection and classify exact exemptions."""
    import numpy as np
    import trimesh

    run_root = Path(root)
    transforms = _world_transforms(model)
    solids: dict[str, Any] = {}
    input_hashes: dict[str, str] = {}
    rows: list[dict[str, Any]] = []
    unavailable: list[dict[str, str]] = []

    for link in model.get("links", ()):
        if not isinstance(link, Mapping) or not link.get("name"):
            continue
        name = str(link["name"])
        relative = str(link.get("mesh_filename", ""))
        # A semantic grouping/body may intentionally have no independent solid. It is
        # represented by its mounted child solids and is not a malformed geometry file.
        if not relative:
            continue
        path = run_root / relative
        try:
            mesh = trimesh.load_mesh(path, force="mesh")
            mesh.apply_transform(transforms[name])
            if not mesh.is_watertight or abs(float(mesh.volume)) <= 0.0:
                raise ValueError(f"not a usable watertight volume (watertight={mesh.is_watertight}, volume={mesh.volume})")
            solids[name] = mesh
            input_hashes[relative.replace("\\", "/")] = _sha256(path)
        except Exception as exc:
            unavailable.append({"link": name, "detail": f"{type(exc).__name__}: {exc}"})

    relations = _relation_index(model)
    mounts = _mount_index(model)
    decisions = _decision_index(manifest)
    names = sorted(solids)
    for index, name_a in enumerate(names):
        mesh_a = solids[name_a]
        for name_b in names[index + 1:]:
            mesh_b = solids[name_b]
            lower = np.maximum(mesh_a.bounds[0], mesh_b.bounds[0])
            upper = np.minimum(mesh_a.bounds[1], mesh_b.bounds[1])
            if np.any(lower >= upper):
                continue
            try:
                intersection = trimesh.boolean.intersection(
                    [mesh_a, mesh_b], engine="manifold")
                volume = (0.0 if intersection is None or len(intersection.vertices) == 0
                          else abs(float(intersection.volume)))
            except Exception as exc:
                rows.append({"pair": [name_a, name_b], "status": "CONFLICT",
                             "kind": "exact_boolean_unavailable", "volume_mm3": None,
                             "fraction_smaller": None, "exemption": None,
                             "detail": f"{type(exc).__name__}: {exc}"})
                continue
            if volume < REPORT_VOLUME_MM3:
                continue
            fraction = volume / min(abs(float(mesh_a.volume)), abs(float(mesh_b.volume)))
            category, evidence_id, source = _exemption(
                frozenset((name_a, name_b)), relations, mounts, decisions)
            rows.append({"pair": [name_a, name_b],
                         "status": "EXEMPT" if category else "CONFLICT",
                         "kind": "exact_solid_intersection",
                         "volume_mm3": round(volume, 6),
                         "fraction_smaller": round(fraction, 9),
                         "exemption": category, "evidence_id": evidence_id,
                         "exemption_source": source})

    conflict_rows = [row for row in rows if row["status"] == "CONFLICT"]
    manifest_only = [row for row in rows if row["status"] == "EXEMPT"
                     and row.get("exemption_source") == "builder_manifest"]
    return {
        "schema": "physcad-scorer-geometry/1.0",
        "analyzer_version": GEOMETRY_ANALYZER_VERSION,
        "method": "final-pose watertight STL boolean intersection using manifold",
        "report_volume_mm3": REPORT_VOLUME_MM3,
        "fail_closed": True,
        "input_hashes": dict(sorted(input_hashes.items())),
        "links_scanned": len(solids),
        "scan_unavailable": unavailable,
        "non_exempt_conflict_count": len(conflict_rows) + len(unavailable),
        "manifest_only_exemption_count": len(manifest_only),
        "provenance_warnings": (["one or more exemptions are supported only by builder_manifest"]
                                if manifest_only else []),
        "intersections": rows,
    }


__all__ = ["GEOMETRY_ANALYZER_VERSION", "REPORT_VOLUME_MM3", "analyze_geometry"]
