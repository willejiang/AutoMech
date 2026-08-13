"""Deterministic MJCF semantic and collision audit for external submissions."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping
import xml.etree.ElementTree as ET

MODEL_AUDIT_VERSION = "external-model-audit/1.0"


def _active(element: ET.Element) -> bool:
    return str(element.get("active", "true")).casefold() not in {"false", "0"}


def _coefficients(element: ET.Element) -> tuple[float, ...] | None:
    try:
        values = tuple(float(value) for value in
                       element.get("polycoef", "0 1 0 0 0").split())
    except ValueError:
        return None
    return values if len(values) == 5 and all(math.isfinite(value) for value in values) else None


def _transmissions(assembly: Mapping[str, Any]) -> list[dict[str, Any]]:
    result = []
    for row in assembly.get("transmissions", ()):
        if not isinstance(row, Mapping):
            continue
        try:
            parsed_ratio = float(row.get("ratio"))
            ratio = parsed_ratio if math.isfinite(parsed_ratio) and parsed_ratio != 0.0 else None
        except (TypeError, ValueError):
            ratio = None
        result.append({"name": str(row.get("name") or ""),
                       "driving": str(row.get("driving_link") or ""),
                       "driven": str(row.get("driven_link") or ""),
                       "ratio": ratio})
    return result


def audit_model(model_path: str | Path, assembly: Mapping[str, Any],
                bindings: Mapping[str, Any], task_id: str) -> dict[str, Any]:
    import mujoco

    path = Path(model_path)
    xml = ET.parse(path).getroot()
    model = mujoco.MjModel.from_xml_path(str(path))
    body_ids = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id): body_id
                for body_id in range(1, model.nbody)}
    mesh_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id): mesh_id
                  for mesh_id in range(model.nmesh)}

    coverage = []
    for link in assembly.get("links", ()):
        if not isinstance(link, Mapping) or not link.get("mesh_filename"):
            continue
        name = str(link.get("name"))
        candidates = set()
        body_id = body_ids.get(name)
        if body_id is not None:
            candidates.update(geom_id for geom_id in range(model.ngeom)
                              if int(model.geom_bodyid[geom_id]) == body_id)
        folded = name.casefold()
        for geom_id in range(model.ngeom):
            geom_name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                         or "").casefold()
            mesh_id = int(model.geom_dataid[geom_id]) if int(model.geom_type[geom_id]) == int(
                mujoco.mjtGeom.mjGEOM_MESH) else -1
            mesh_name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
                         or "").casefold() if mesh_id >= 0 else ""
            if geom_name == folded or geom_name.startswith(folded + "_") \
                    or mesh_name == folded or mesh_name.startswith(folded + "_"):
                candidates.add(geom_id)
        active_geoms = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                        or f"geom_{geom_id}" for geom_id in sorted(candidates)
                        if int(model.geom_contype[geom_id]) != 0
                        and int(model.geom_conaffinity[geom_id]) != 0]
        coverage.append({"link": name, "body_present": body_id is not None,
                         "candidate_geom_count": len(candidates),
                         "active_collision_geoms": active_geoms,
                         "covered": bool(active_geoms)})

    input_roles = [role for role in bindings if "input" in str(role)]
    if task_id == "10_wind_rotor_pump":
        input_roles = ["rotor_shaft_input"]
    input_joints = {str(value) for role in input_roles
                    for value in (bindings.get(role) or ())}
    actuator_targets = []
    actuator_root = xml.find("actuator")
    if actuator_root is not None:
        for element in actuator_root:
            target = element.get("joint")
            if target:
                actuator_targets.append({"name": element.get("name"), "joint": target,
                                         "type": element.tag,
                                         "is_registered_input": target in input_joints})

    transmissions = _transmissions(assembly)
    active_joint_equalities = [element for element in xml.findall("./equality/joint")
                               if _active(element)]
    equality_checks = []
    used_equalities: set[int] = set()
    for transmission in transmissions:
        driving, driven, ratio = (transmission["driving"], transmission["driven"],
                                  transmission["ratio"])
        matched = False
        detail = "authored transmission ratio is missing, non-finite, or zero"
        if ratio is not None:
            detail = "no active equality matches authored driving/driven coordinates"
            for index, element in enumerate(active_joint_equalities):
                first, second = element.get("joint1"), element.get("joint2")
                coeff = _coefficients(element)
                if coeff is None or any(abs(value) > 1.0e-12 for value in coeff[2:]):
                    continue
                expected = None
                if first == driven and second == driving:
                    expected = ratio
                elif first == driving and second == driven:
                    expected = 1.0 / ratio
                if expected is None:
                    continue
                if abs(coeff[0]) <= 1.0e-12 and math.isclose(
                        coeff[1], expected, rel_tol=1.0e-8, abs_tol=1.0e-10):
                    matched = True
                    used_equalities.add(index)
                    detail = f"{first}={coeff[1]:.12g}*{second} matches ratio {ratio:.12g}"
                    break
                detail = f"equality coefficient {coeff[1]:.12g} != expected {expected:.12g}"
        equality_checks.append({**transmission, "matched": matched, "detail": detail})

    linkage_task = task_id.startswith(("07_", "08_", "09_", "10_"))
    active_closures = []
    equality_root = xml.find("equality")
    if equality_root is not None:
        for element in equality_root:
            if not _active(element):
                continue
            nontrivial = True
            if element.tag == "joint":
                coeff = _coefficients(element)
                nontrivial = bool(coeff and any(abs(value) > 1.0e-12 for value in coeff[1:]))
            if element.tag in {"connect", "weld"} or nontrivial:
                active_closures.append({"name": element.get("name"), "type": element.tag})

    collision_ok = bool(coverage) and all(row["covered"] for row in coverage)
    actuator_ok = all(row["is_registered_input"] for row in actuator_targets)
    transmission_ok = bool(transmissions) and all(row["matched"] for row in equality_checks)
    if linkage_task:
        closure_ok = bool(active_closures)
        # Linkages can have no scalar transmission, but they must have a real active closure.
        semantic_ok = closure_ok
    else:
        closure_ok = True
        semantic_ok = transmission_ok
    errors = []
    if not collision_ok:
        errors.append("one or more physical mesh links lack active collision coverage")
    if not actuator_ok:
        errors.append("an actuator targets a non-input coordinate")
    if not semantic_ok:
        errors.append("authored transmission/closure is not represented by a valid active constraint")
    return {"schema": "physcad-scorer-model-audit/1.0",
            "analyzer_version": MODEL_AUDIT_VERSION, "task_id": task_id,
            "collision_coverage": coverage, "collision_coverage_ok": collision_ok,
            "registered_input_joints": sorted(input_joints),
            "actuators": actuator_targets, "actuator_policy_ok": actuator_ok,
            "authored_transmissions": transmissions,
            "equality_checks": equality_checks, "transmission_binding_ok": transmission_ok,
            "active_closures": active_closures, "closure_ok": closure_ok,
            "passed": not errors, "errors": errors}


__all__ = ["MODEL_AUDIT_VERSION", "audit_model"]
