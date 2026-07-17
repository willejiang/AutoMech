"""Typed, unit-aware deterministic parameter graph."""
from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import Any, Callable, Iterable


class ParameterGraphError(ValueError):
    pass


@dataclass(frozen=True)
class ParameterValue:
    value: Any
    unit: str
    provenance: tuple[str, ...]


@dataclass(frozen=True)
class ParameterNode:
    id: str
    unit: str
    dependencies: tuple[str, ...]
    formula: Callable[..., Any]
    provenance: tuple[str, ...] = ()


class ParameterDAG:
    def __init__(self):
        self._inputs: dict[str, ParameterValue] = {}
        self._nodes: dict[str, ParameterNode] = {}

    def add_input(self, node_id: str, value: Any, unit: str,
                  provenance: Iterable[str] = ()) -> None:
        self._check_new(node_id)
        self._inputs[node_id] = ParameterValue(value, unit, tuple(provenance))

    def add_formula(self, node_id: str, unit: str, dependencies: Iterable[str],
                    formula: Callable[..., Any], provenance: Iterable[str] = ()) -> None:
        self._check_new(node_id)
        self._nodes[node_id] = ParameterNode(node_id, unit, tuple(dependencies), formula,
                                             tuple(provenance))

    def _check_new(self, node_id: str) -> None:
        if not node_id or node_id in self._inputs or node_id in self._nodes:
            raise ParameterGraphError(f"duplicate or empty parameter id '{node_id}'")

    def evaluate(self) -> dict[str, ParameterValue]:
        known = dict(self._inputs)
        pending = dict(self._nodes)
        all_ids = set(known) | set(pending)
        missing = sorted({dep for node in pending.values() for dep in node.dependencies} - all_ids)
        if missing:
            raise ParameterGraphError(f"missing parameter inputs: {missing}")
        while pending:
            ready = sorted(node_id for node_id, node in pending.items()
                           if all(dep in known for dep in node.dependencies))
            if not ready:
                cycle = sorted(pending)
                raise ParameterGraphError(f"parameter dependency cycle: {cycle}")
            for node_id in ready:
                node = pending.pop(node_id)
                args = [known[dep].value for dep in node.dependencies]
                try:
                    value = node.formula(*args)
                except Exception as exc:
                    raise ParameterGraphError(f"formula '{node_id}' failed: {exc}") from exc
                if isinstance(value, float) and not (-float("inf") < value < float("inf")):
                    raise ParameterGraphError(f"formula '{node_id}' produced a non-finite value")
                inherited = tuple(dict.fromkeys(
                    item for dep in node.dependencies for item in known[dep].provenance))
                known[node_id] = ParameterValue(value, node.unit,
                                                 inherited + node.provenance + (f"formula:{node_id}",))
        return {key: known[key] for key in sorted(known)}


def enumerate_candidates(choices: dict[str, Iterable[Any]], score: Callable[[dict], tuple]) -> tuple[dict, ...]:
    """Enumerate a Cartesian product using stable key/value order and score tie-breaking."""
    keys = sorted(choices)
    values = [tuple(choices[key]) for key in keys]
    candidates = [dict(zip(keys, combination)) for combination in itertools.product(*values)]
    return tuple(sorted(candidates, key=lambda c: (tuple(score(c)), tuple((k, repr(c[k])) for k in keys))))
