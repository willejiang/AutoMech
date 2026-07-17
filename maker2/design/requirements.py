"""Deterministic extraction of explicit numerical design requirements."""
from __future__ import annotations

import re

from .ir import RequirementFact, fingerprint

_NUMBER = r"(?P<value>\d+(?:\.\d+)?)"
_PATTERNS = (
    ("ratio", re.compile(rf"(?P<text>{_NUMBER}\s*(?::|to)\s*(?P<den>\d+(?:\.\d+)?)(?:\s+(?:reduction|gear)\s*ratio)?)", re.I)),
    ("ratio", re.compile(rf"(?P<text>{_NUMBER}\s*[x×]\s*(?:reduction|ratio)?)", re.I)),
    ("torque", re.compile(rf"(?P<text>{_NUMBER}\s*(?P<unit>n\s*[·*\-]?\s*m|nm|n\s*mm))", re.I)),
    ("speed", re.compile(rf"(?P<text>{_NUMBER}\s*(?P<unit>rpm|rev(?:olutions?)?\s*/\s*min(?:ute)?))", re.I)),
    ("power", re.compile(rf"(?P<text>{_NUMBER}\s*(?P<unit>kw|w))\b", re.I)),
    ("length", re.compile(rf"(?P<text>{_NUMBER}\s*(?P<unit>mm|cm|m))\b", re.I)),
)


def _normalize(kind: str, value: float, unit: str, den: str | None = None) -> tuple[float, str]:
    u = re.sub(r"[\s·*\-]", "", unit.lower())
    if kind == "ratio":
        return value / float(den or 1.0), "1"
    if kind == "torque":
        return (value / 1000.0, "N*m") if u == "nmm" else (value, "N*m")
    if kind == "speed":
        return value, "rpm"
    if kind == "power":
        return (value * 1000.0, "W") if u == "kw" else (value, "W")
    scale = {"mm": 0.001, "cm": 0.01, "m": 1.0}[u]
    return value * scale, "m"


def extract_requirements(prompt: str) -> tuple[RequirementFact, ...]:
    """Extract only explicit values; inferred design assumptions are intentionally absent."""
    found = []
    occupied: list[tuple[int, int]] = []
    for kind, pattern in _PATTERNS:
        for match in pattern.finditer(prompt):
            span = match.span("text")
            if any(span[0] < b and span[1] > a for a, b in occupied):
                continue
            value, unit = _normalize(kind, float(match.group("value")),
                                     match.groupdict().get("unit") or "",
                                     match.groupdict().get("den"))
            raw = match.group("text")
            fact_id = "req_" + fingerprint((kind, span, raw), prefix="").lstrip(":")[:12]
            found.append(RequirementFact(fact_id, kind, value, unit, raw, span, raw))
            occupied.append(span)
    return tuple(sorted(found, key=lambda f: (f.source_span, f.kind, f.id)))


def facts_by_kind(facts: tuple[RequirementFact, ...]) -> dict[str, tuple[RequirementFact, ...]]:
    out: dict[str, list[RequirementFact]] = {}
    for fact in facts:
        out.setdefault(fact.kind, []).append(fact)
    return {kind: tuple(values) for kind, values in sorted(out.items())}
