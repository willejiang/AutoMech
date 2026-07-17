"""Versioned, canonical design-compiler IR.

Intent contains only references and discrete choices. Derived numerical authority lives in
``CompiledDesign`` and records units plus provenance for every parameter.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field, is_dataclass
from enum import Enum
from typing import Any

IR_VERSION = "design_intent_v1"
COMPILER_VERSION = "geometry_compiler_v1"


def canonical_data(value: Any) -> Any:
    """Return JSON-safe data with deterministic mapping and set ordering."""
    if is_dataclass(value):
        value = asdict(value)
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(k): canonical_data(v) for k, v in sorted(value.items(), key=lambda x: str(x[0]))}
    if isinstance(value, (list, tuple)):
        return [canonical_data(v) for v in value]
    if isinstance(value, (set, frozenset)):
        return sorted((canonical_data(v) for v in value), key=lambda x: json.dumps(x, sort_keys=True))
    if isinstance(value, float):
        if not (-float("inf") < value < float("inf")):
            raise ValueError("canonical data cannot contain non-finite floats")
        return 0.0 if value == 0.0 else value
    return value


def canonical_json(value: Any) -> str:
    return json.dumps(canonical_data(value), sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def fingerprint(value: Any, prefix: str = "sha256") -> str:
    digest = hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()
    return f"{prefix}:{digest}"


@dataclass(frozen=True)
class RequirementFact:
    id: str
    kind: str
    value: float
    unit: str
    source_text: str
    source_span: tuple[int, int]
    normalized_from: str = ""


@dataclass(frozen=True)
class DesignIntentIR:
    template_id: str
    topology_roles: tuple[tuple[str, str], ...]
    requirement_fact_ids: tuple[str, ...] = ()
    standards_profile_ids: tuple[str, ...] = ()
    allowed_component_family_ids: tuple[str, ...] = ()
    layout: str = "linear"
    priorities: tuple[str, ...] = ("ratio_error", "envelope", "catalog_order")
    discrete_choices: tuple[tuple[str, str], ...] = ()
    version: str = IR_VERSION

    def role_map(self) -> dict[str, str]:
        return dict(self.topology_roles)

    def choice_map(self) -> dict[str, str]:
        return dict(self.discrete_choices)


@dataclass(frozen=True)
class CompiledParameter:
    id: str
    value: float
    unit: str
    provenance: tuple[str, ...]


@dataclass(frozen=True)
class SelectedComponent:
    role: str
    catalog_id: str
    family_id: str
    parameters: tuple[tuple[str, Any], ...] = ()


@dataclass(frozen=True)
class CompiledDesign:
    intent: DesignIntentIR
    requirement_facts: tuple[RequirementFact, ...]
    selected_components: tuple[SelectedComponent, ...]
    parameters: tuple[CompiledParameter, ...]
    problem: Any
    solve_result: Any
    compiler_version: str
    catalog_version: str
    intent_hash: str
    design_hash: str
    contract_hash: str
    diagnostics: tuple[tuple[str, Any], ...] = ()

    def parameter_map(self) -> dict[str, CompiledParameter]:
        return {p.id: p for p in self.parameters}

    def to_dict(self) -> dict:
        return canonical_data(self)
