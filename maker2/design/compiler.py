"""Authoritative deterministic design compiler."""
from __future__ import annotations

from .catalog import Catalog, load_catalog
from .gates import compiled_gate, intent_gate
from .ir import (COMPILER_VERSION, CompiledDesign, CompiledParameter, DesignIntentIR,
                 RequirementFact, canonical_data, fingerprint)
from .templates.base import DesignTemplate
from .templates.gear_reducer import ParallelShaftTwoStageReducerTemplate


class DesignCompileError(RuntimeError):
    def __init__(self, message: str, *, errors=()):
        super().__init__(message)
        self.errors = tuple(errors)


def _templates() -> dict[str, DesignTemplate]:
    items = (ParallelShaftTwoStageReducerTemplate(),)
    return {template.id: template for template in items}


def _catalog_ids(catalog: Catalog) -> set[str]:
    ids = set()
    for section in ("gear_profiles", "gear_face_width_rules", "shaft_families",
                    "bearing_families", "fit_profiles", "clearance_profiles",
                    "housing_wall_profiles"):
        for entry in catalog.entries(section):
            ids.add(entry["id"])
            for variant in entry.get("variants", ()):
                ids.add(variant["id"])
    return ids


def _solve(problem):
    """Use the shared backend when available; temporary lazy fallback during extraction."""
    try:
        from maker2.constraint_solver import solve_problem
    except ImportError:
        try:
            from maker2.slvs_adapter import solve_problem
        except ImportError:
            solve_problem = None
    if solve_problem is not None:
        try:
            return solve_problem(problem)
        except ModuleNotFoundError as exc:
            if exc.name not in {"py_slvs", "py_slvs.slvs"}:
                raise
    raise DesignCompileError("constraint solver unavailable for authoritative design compilation")


def compile_design(intent: DesignIntentIR, facts: tuple[RequirementFact, ...],
                   *, catalog: Catalog | None = None) -> tuple[CompiledDesign, object]:
    catalog = catalog or load_catalog()
    templates = _templates()
    errors = intent_gate(intent, template_ids=set(templates), fact_ids={fact.id for fact in facts},
                         catalog_ids=_catalog_ids(catalog))
    if errors:
        raise DesignCompileError("design intent gate rejected the proposal", errors=errors)
    template = templates[intent.template_id]
    template_errors = template.validate_intent(intent, facts, catalog)
    if template_errors:
        raise DesignCompileError("template rejected the design intent", errors=template_errors)
    candidates = template.enumerate_candidates(intent, facts, catalog)
    if not candidates:
        raise DesignCompileError("no catalog candidate satisfies the design intent")
    candidate = candidates[0]
    values = template.build_parameter_graph(intent, candidate, facts, catalog).evaluate()
    problem = template.build_problem(intent, values)
    solve_result = _solve(problem)
    intent_hash = fingerprint(intent, "intent_v1")
    parameters = tuple(CompiledParameter(key, float(value.value), value.unit, value.provenance)
                       for key, value in values.items() if isinstance(value.value, (int, float)))
    design_payload = {"intent_hash": intent_hash, "candidate": candidate,
                      "parameters": parameters, "compiler_version": COMPILER_VERSION,
                      "catalog_version": catalog.catalog_version,
                      "problem": problem, "solve_result": solve_result}
    design_hash = fingerprint(design_payload, "compiled_design_v1")
    contract = template.project_contract(intent, values, solve_result,
                                         compiler_version=COMPILER_VERSION,
                                         catalog_version=catalog.catalog_version,
                                         design_hash=design_hash)
    gate_errors = compiled_gate(problem, solve_result, contract,
                                compiler_version=COMPILER_VERSION,
                                catalog_version=catalog.catalog_version)
    if gate_errors:
        raise DesignCompileError("compiled design gate rejected the result", errors=gate_errors)
    compiled = CompiledDesign(intent, tuple(facts), candidate.selected_components, parameters,
                              problem, solve_result, COMPILER_VERSION,
                              catalog.catalog_version, intent_hash, design_hash, contract.contract_hash,
                              (("candidate_id", candidate.candidate_id),
                               ("candidate_score", candidate.score)))
    return compiled, contract
