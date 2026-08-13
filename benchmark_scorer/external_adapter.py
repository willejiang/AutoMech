"""Normalize one external-harness task into the portable benchmark contract."""
from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path
from typing import Any, Mapping
import xml.etree.ElementTree as ET

from .contract import CONTRACT_ID, ContractError
from .tasks.comfort_v1 import get_task


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ContractError(f"expected JSON object: {path}")
    return value


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _normalize_relation(row: Mapping[str, Any], index: int) -> dict[str, Any] | None:
    relation = dict(row)
    name = str(relation.get("name") or f"relation_{index}")
    raw_type = str(relation.get("mate_type") or relation.get("type") or
                   relation.get("kind") or "")
    aliases = {
        "running_bearing": "journal_bearing", "bearing": "journal_bearing",
        "ideal_gear_mesh": "gear_spur_external",
        "ideal_external_gear_mesh": "gear_spur_external",
        "external_spur": "gear_spur_external",
        "rigid_carry": "rigid_carry", "rigid_mount": "rigid_mount",
        "press_fit": "press_fit", "pin": "pin", "revolute": "revolute",
        "weld": "weld", "fixed": "fixed",
    }
    mate_type = aliases.get(raw_type, raw_type)
    pair = relation.get("links")
    if not (isinstance(pair, list) and len(pair) == 2):
        pair = None
    base = (relation.get("base_part") or relation.get("a") or relation.get("parent") or
            relation.get("bearing") or relation.get("outer") or relation.get("driving") or
            (pair[0] if pair else None))
    incoming = (relation.get("incoming_part") or relation.get("b") or relation.get("child") or
                relation.get("shaft") or relation.get("inner") or relation.get("driven") or
                (pair[1] if pair else None))
    if not base or not incoming:
        return None
    return {"name": name, "mate_type": mate_type,
            "base_part": str(base), "incoming_part": str(incoming)}


def normalize_assembly(raw: Mapping[str, Any]) -> dict[str, Any]:
    links = []
    for row in raw.get("links", ()):
        if not isinstance(row, Mapping) or not row.get("name"):
            continue
        link = dict(row)
        mesh = link.get("mesh_filename", link.get("mesh", link.get("mesh_file")))
        axes = link.get("axes")
        first_axis = axes[0] if isinstance(axes, list) and axes and isinstance(axes[0], list) else None
        axis = link.get("spin_axis", link.get("axis", first_axis)) or [0.0, 0.0, 1.0]
        dof = str(link.get("dof", link.get("degree_of_freedom", "fixed")))
        dof = {"revolute": "spin", "hinge": "spin", "prismatic": "slide",
               "rigid": "fixed"}.get(dof, dof)
        mount = link.get("mount", link.get("rigid_mount"))
        if mount == "world": mount = ""
        normalized = {**link, "name": str(link["name"]),
                      "dof": dof, "spin_axis": list(axis),
                      "slide_axis": list(link.get("slide_axis") or axis),
                      "mount": str(mount) if mount else ""}
        if mesh:
            normalized["mesh_filename"] = str(mesh).replace("\\", "/")
        else:
            normalized.pop("mesh_filename", None)
            normalized.pop("mesh", None)
        links.append(normalized)
    poses = []
    for row in raw.get("poses", ()):
        if not isinstance(row, Mapping): continue
        child = row.get("child", row.get("link"))
        if not child: continue
        poses.append({"name": str(row.get("name") or f"place_{child}"),
                      "parent": str(row.get("parent") or ""), "child": str(child),
                      "xyz_m": list(row.get("xyz_m", row.get("position_m", [0, 0, 0]))),
                      "rpy_rad": list(row.get("rpy_rad", row.get("rotation_rad", [0, 0, 0])))})
    relations = [normalized for index, row in enumerate(raw.get("relations", ()))
                 if isinstance(row, Mapping)
                 for normalized in [_normalize_relation(row, index)] if normalized]
    known_pairs = {frozenset((row["base_part"], row["incoming_part"])) for row in relations}
    for index, row in enumerate(raw.get("mesh_pairs", ())):
        if isinstance(row, Mapping):
            a = row.get("a", row.get("driving")); b = row.get("b", row.get("driven"))
            if a and b and frozenset((str(a), str(b))) not in known_pairs:
                kind = str(row.get("kind", row.get("type", ""))).casefold()
                mate = "gear_spur_internal" if "internal" in kind else "gear_spur_external"
                relations.append({"name": str(row.get("name") or f"mesh_{index}"),
                                  "mate_type": mate, "base_part": str(a),
                                  "incoming_part": str(b)})
    transmissions = []
    for row in raw.get("transmissions", ()):
        if not isinstance(row, Mapping): continue
        ratio = row.get("ratio", row.get("ratio_driven_over_driving",
                         row.get("ratio_driven_per_driving", row.get("driven_over_driving"))))
        driving = row.get("driving_link", row.get("driving_joint", row.get("driving")))
        driven = row.get("driven_link", row.get("driven_joint", row.get("driven")))
        transmissions.append({**row, "name": str(row.get("name") or "transmission"),
                              "type": str(row.get("type", row.get("kind", "transmission"))),
                              "driving_link": str(driving or ""),
                              "driven_link": str(driven or ""), "ratio": ratio})
    motion = []
    for row in raw.get("motion_joints", ()):
        if not isinstance(row, Mapping): continue
        motion.append({**row, "type": str(row.get("type", row.get("kind", ""))),
                       "pos_mm": [1000.0 * float(x) for x in
                                  row.get("pos_m", row.get("origin_m", row.get("position_m", [0,0,0])))]})
    return {"name": str(raw.get("name") or "external"),
            "root_link": str(raw.get("root_link") or (links[0]["name"] if links else "")),
            "links": links, "poses": poses,
            "ports_by_link": raw.get("ports_by_link", {}), "relations": relations,
            "motion_joints": motion, "transmissions": transmissions,
            "planetary_stages": raw.get("planetary_stages", []),
            "mesh_pairs": [[row["base_part"], row["incoming_part"]]
                           for row in relations if "gear" in row["mate_type"]],
            "output_link": str(raw.get("output_link") or
                               ((raw.get("output") or {}).get("link") if isinstance(raw.get("output"), Mapping) else "") or ""),
            "watch_links": list(raw.get("watch_links", []))}


def _record(root: Path, relative: str, role: str) -> dict[str, Any]:
    path = root / relative
    media = ("application/json" if path.suffix == ".json" else
             "text/x-python" if path.suffix == ".py" else
             "video/mp4" if path.suffix == ".mp4" else "application/octet-stream")
    return {"path": relative, "sha256": _sha(path), "size": path.stat().st_size,
            "media_type": media, "role": role}


def stage_external_task(task_dir: str | Path, output_dir: str | Path,
                        suite_config: Mapping[str, Any]) -> Path:
    source = Path(task_dir); target = Path(output_dir)
    if target.exists():
        raise ContractError(f"external staging destination exists: {target}")
    target.mkdir(parents=True)
    task = get_task(source.name)
    assembly = normalize_assembly(_read(source / "assembly.json"))
    for link in assembly["links"]:
        mesh = link.get("mesh_filename")
        if mesh:
            link["mesh_filename"] = "meshes/" + Path(mesh).name
    (target / "assembly.json").write_text(json.dumps(assembly, indent=2, sort_keys=True,
                                                       allow_nan=False), encoding="utf-8")
    shutil.copy2(source / "task_bindings.json", target / "task_bindings.json")
    for relative in ("source/machine.py", "evidence/trajectory.json", "evidence/contacts.json"):
        destination = target / relative; destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / relative, destination)
    (target / "meshes").mkdir()
    for mesh in sorted((source / "meshes").glob("*.stl")):
        shutil.copy2(mesh, target / "meshes" / mesh.name)
    original_model = source / "models/model.mjcf"
    xml = ET.parse(original_model); root = xml.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler"); root.insert(0, compiler)
    compiler.set("meshdir", "meshes")
    for element in root.findall(".//mesh"):
        file = element.get("file")
        if file: element.set("file", Path(file).name)
    model_dir = target / "models"; (model_dir / "meshes").mkdir(parents=True)
    for mesh in sorted((source / "meshes").glob("*.stl")):
        shutil.copy2(mesh, model_dir / "meshes" / mesh.name)
    xml.write(model_dir / "model.mjcf", encoding="unicode")
    raw_dir = target / "raw"; raw_dir.mkdir()
    for name in ("run.json", "evaluator_result.json"):
        if (source / "raw" / name).is_file(): shutil.copy2(source / "raw" / name, raw_dir / name)
    files = [_record(target, "assembly.json", "assembly"),
             _record(target, "task_bindings.json", "task_bindings"),
             _record(target, "source/machine.py", "source"),
             _record(target, "models/model.mjcf", "model_mjcf"),
             _record(target, "evidence/trajectory.json", "trajectory"),
             _record(target, "evidence/contacts.json", "contacts")]
    for path in sorted((target / "meshes").glob("*.stl")):
        files.append(_record(target, f"meshes/{path.name}", "mesh"))
    for path in sorted((target / "models/meshes").glob("*.stl")):
        files.append(_record(target, f"models/meshes/{path.name}", "model_asset"))
    raw_audit=[]
    for path in sorted(raw_dir.glob("*.json")):
        rel=f"raw/{path.name}"; files.append(_record(target, rel, "audit")); raw_audit.append(rel)
    run_id = str((_read(source / "raw/run.json")).get("selected_iteration", source.name))
    physics_mode = "finite_effort" if task.finite_effort_required else "submitted"
    manifest = {"contract": CONTRACT_ID, "suite_id": "physcad-comfort-v1",
        "task_id": task.task_id, "prompt_sha256": task.prompt_sha256,
        "producer": {"harness": str(suite_config.get("method") or "external"),
                     "harness_version": str(suite_config.get("cli_version") or "unknown"),
                     "run_id": f"{source.name}:{run_id}"},
        "evidence_lane": {"submitted": True, "physics_mode": physics_mode,
                          "engine": "mujoco", "engine_version": "unknown"},
        "files": files, "units": {"length": "mm", "angle": "rad", "time": "s"},
        "telemetry_provenance": {"simulator": "mujoco",
            "sampling_source": "evidence/trajectory.json", "from": "external_harness"},
        "idealizations": [], "raw_audit": raw_audit}
    (target / "benchmark_submission.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
    return target


__all__ = ["normalize_assembly", "stage_external_task"]
