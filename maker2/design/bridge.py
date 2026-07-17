"""Bridge the accepted Boss plan into the authoritative design compiler.

The Boss chooses topology (the ``SubassemblyPlan``); this module derives a
deterministic ``DesignIntentIR`` for a recognized template, runs the
authoritative compiler, and persists the frozen artifacts. The compiler and its
gates -- not the Boss -- remain authoritative over every derived number.
"""
from __future__ import annotations

import json
import os

from .compiler import DesignCompileError, compile_design
from .ir import DesignIntentIR, canonical_json
from .requirements import extract_requirements
from .templates.gear_reducer import TEMPLATE_ID

_REDUCER_ROLES = ("housing", "input_stage", "intermediate_stage", "output_stage")


class TopologyNotCompilable(RuntimeError):
    """The accepted plan does not match any supported compiler template."""


def _plan_sub_ids(plan) -> tuple[str, ...]:
    subs = getattr(plan, "subassemblies", None) or []
    return tuple(getattr(s, "id", "") for s in subs)


def recognize_template(plan) -> str | None:
    """Return the template id for a recognized topology, else None."""
    ids = set(_plan_sub_ids(plan))
    if set(_REDUCER_ROLES) <= ids:
        return TEMPLATE_ID
    return None


def derive_intent(plan, facts, template_id: str) -> DesignIntentIR:
    """Deterministically build a DesignIntentIR from the accepted plan topology.

    Roles bind to same-named subassemblies. No coordinates or derived numbers
    enter the intent; only references and discrete choices.
    """
    roles = tuple((role, role) for role in _REDUCER_ROLES)
    ratio_fact_ids = tuple(f.id for f in facts if f.kind == "ratio")
    return DesignIntentIR(
        template_id=template_id,
        topology_roles=roles,
        requirement_fact_ids=ratio_fact_ids,
        standards_profile_ids=("spur_20deg_full_depth",),
        allowed_component_family_ids=("shaft_metric_light", "bearing_6000"),
        layout="linear",
    )


def compile_from_plan(plan, prompt: str, *, mode: str = "auto",
                      out_dir: str | None = None, log_fn=None):
    """Compile the accepted plan to a frozen hardpoint contract.

    Returns (compiled_design, contract) on success, or (None, None) when the
    topology is unsupported and ``mode`` permits the legacy fallback. Raises
    ``TopologyNotCompilable`` when ``mode == 'required'`` and the topology is not
    recognized, and ``DesignCompileError`` when compilation itself fails.
    """
    def log(message: str) -> None:
        if log_fn is not None:
            log_fn(message)

    if mode == "legacy":
        return None, None

    template_id = recognize_template(plan)
    if template_id is None:
        if mode == "required":
            raise TopologyNotCompilable(
                "geometry_compiler_mode=required but plan topology is not a "
                "supported compiler template")
        log("[compile] topology not recognized; using legacy_hierarchy")
        return None, None

    facts = extract_requirements(prompt)
    intent = derive_intent(plan, facts, template_id)
    compiled, contract = compile_design(intent, facts)
    contract_errors = contract.validate()
    if contract_errors:
        raise DesignCompileError("frozen contract failed validation",
                                 errors=contract_errors)

    log(f"[compile] compiled_hardpoints_v1 froze before managers "
        f"(contract {contract.contract_hash[:19]}, "
        f"{len(contract.hardpoints)} hardpoints, dof={compiled.solve_result.dof})")

    if out_dir:
        persist_artifacts(compiled, contract, intent, facts, out_dir)
        log(f"[compile] wrote design artifacts to {out_dir}")
    return compiled, contract


def persist_artifacts(compiled, contract, intent, facts, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    artifacts = {
        "design_intent.json": intent,
        "hardpoint_contract.json": contract.to_dict(),
        "design_compile_report.json": {
            "compiler_version": compiled.compiler_version,
            "catalog_version": compiled.catalog_version,
            "intent_hash": compiled.intent_hash,
            "design_hash": compiled.design_hash,
            "contract_hash": compiled.contract_hash,
            "dof": compiled.solve_result.dof,
            "status": compiled.solve_result.status,
            "requirement_facts": [f.__dict__ for f in facts],
            "selected_components": [c.__dict__ for c in compiled.selected_components],
        },
        "compiled_parameters.json": {
            p.id: {"value": p.value, "unit": p.unit} for p in compiled.parameters
        },
    }
    for name, payload in artifacts.items():
        with open(os.path.join(out_dir, name), "w", encoding="utf-8") as handle:
            handle.write(canonical_json(payload))
