"""Interfaces for deterministic compiled-design templates."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from ..catalog import Catalog
from ..contracts import HardpointContract
from ..ir import DesignIntentIR, RequirementFact, SelectedComponent
from ..parameter_dag import ParameterDAG, ParameterValue


@dataclass(frozen=True)
class TemplateCandidate:
    selected_components: tuple[SelectedComponent, ...]
    inputs: tuple[tuple[str, Any], ...]
    score: tuple
    candidate_id: str


class DesignTemplate(ABC):
    id: str
    layouts: tuple[str, ...]
    required_roles: tuple[str, ...]

    def matches(self, topology_roles: dict[str, str]) -> bool:
        return set(self.required_roles).issubset(topology_roles)

    @abstractmethod
    def validate_intent(self, intent: DesignIntentIR, facts: tuple[RequirementFact, ...],
                        catalog: Catalog) -> tuple[str, ...]:
        raise NotImplementedError

    @abstractmethod
    def enumerate_candidates(self, intent: DesignIntentIR, facts: tuple[RequirementFact, ...],
                             catalog: Catalog) -> tuple[TemplateCandidate, ...]:
        raise NotImplementedError

    @abstractmethod
    def build_parameter_graph(self, intent: DesignIntentIR, candidate: TemplateCandidate,
                              facts: tuple[RequirementFact, ...], catalog: Catalog) -> ParameterDAG:
        raise NotImplementedError

    @abstractmethod
    def build_problem(self, intent: DesignIntentIR, values: dict[str, ParameterValue]):
        raise NotImplementedError

    @abstractmethod
    def project_contract(self, intent: DesignIntentIR, values: dict[str, ParameterValue],
                         solve_result, *, compiler_version: str, catalog_version: str,
                         design_hash: str) -> HardpointContract:
        raise NotImplementedError
