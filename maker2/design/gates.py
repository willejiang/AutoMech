"""Deterministic gates for design intent, compilation, and frozen contracts."""
from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import HardpointContract
from .ir import COMPILER_VERSION, DesignIntentIR


@dataclass(frozen=True)
class DesignGateError:
    code: str
    message: str
    field: str = ""


_FORBIDDEN_AUTHORITY = ("coordinate", "xyz", "center_distance", "shaft_length",
                        "bearing_position", "module_mm", "teeth")


def intent_gate(intent: DesignIntentIR, *, template_ids: set[str], fact_ids: set[str],
                catalog_ids: set[str]) -> tuple[DesignGateError, ...]:
    errors = []
    if intent.version != "design_intent_v1":
        errors.append(DesignGateError("ERR_DESIGN_VERSION", f"unsupported intent version '{intent.version}'"))
    if intent.template_id not in template_ids:
        errors.append(DesignGateError("ERR_TEMPLATE", f"unknown template '{intent.template_id}'", "template_id"))
    unknown_facts = sorted(set(intent.requirement_fact_ids) - fact_ids)
    if unknown_facts:
        errors.append(DesignGateError("ERR_REQUIREMENT_REF", f"unknown requirement facts: {unknown_facts}"))
    refs = set(intent.standards_profile_ids) | set(intent.allowed_component_family_ids)
    unknown_refs = sorted(refs - catalog_ids)
    if unknown_refs:
        errors.append(DesignGateError("ERR_CATALOG_REF", f"unknown catalog references: {unknown_refs}"))
    for key, value in intent.discrete_choices:
        normalized = key.lower()
        if any(token in normalized for token in _FORBIDDEN_AUTHORITY):
            errors.append(DesignGateError("ERR_RAW_NUMERIC_AUTHORITY",
                                          f"intent choice '{key}' attempts derived numeric authority", key))
        if not isinstance(value, str):
            errors.append(DesignGateError("ERR_DISCRETE_CHOICE", f"choice '{key}' must reference a string ID", key))
    return tuple(errors)


def compiled_gate(problem, solve_result, contract: HardpointContract, *,
                  compiler_version: str, catalog_version: str,
                  residual_tolerance_m: float = 1e-7) -> tuple[DesignGateError, ...]:
    errors = []
    if compiler_version != COMPILER_VERSION:
        errors.append(DesignGateError("ERR_COMPILER_VERSION", "compiled artifact uses a stale compiler version"))
    if not catalog_version:
        errors.append(DesignGateError("ERR_CATALOG_VERSION", "compiled artifact has no catalog version"))
    if solve_result.status != "okay":
        errors.append(DesignGateError("ERR_DESIGN_SOLVE", f"constraint solve status is {solve_result.status}"))
    if solve_result.dof != problem.expected_dof:
        errors.append(DesignGateError("ERR_DESIGN_DOF", f"solve DOF {solve_result.dof}, expected {problem.expected_dof}"))
    if solve_result.failed_constraint_ids:
        errors.append(DesignGateError("ERR_DESIGN_CONSTRAINT",
                                      f"failed constraints: {solve_result.failed_constraint_ids}"))
    for constraint in problem.constraints:
        if constraint.kind.value != "distance":
            continue
        a, b = (solve_result.points_m.get(entity) for entity in constraint.entities[:2])
        if a is None or b is None:
            errors.append(DesignGateError("ERR_DESIGN_POINT", f"missing solved point for '{constraint.id}'"))
            continue
        residual = abs(math.dist(a, b) - constraint.value_m)
        if residual > residual_tolerance_m:
            errors.append(DesignGateError("ERR_DESIGN_RESIDUAL",
                                          f"'{constraint.id}' residual {residual:g} m"))
    for message in contract.validate():
        errors.append(DesignGateError("ERR_CONTRACT", message))
    if contract.compiler_version != compiler_version or contract.catalog_version != catalog_version:
        errors.append(DesignGateError("ERR_CONTRACT_VERSION", "contract/compiler version mismatch"))
    sub_ids = {sid for sid, _ in contract.root_transforms}
    covered = {hardpoint.sub_id for hardpoint in contract.hardpoints}
    missing = sorted(sub_ids - covered)
    if missing:
        errors.append(DesignGateError("ERR_INTERFACE_COVERAGE", f"subassemblies without hardpoints: {missing}"))
    return tuple(errors)
