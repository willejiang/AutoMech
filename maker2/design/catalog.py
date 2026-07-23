"""Validated, versioned deterministic standards catalog."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping


class CatalogError(ValueError):
    pass


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({k: _freeze(v) for k, v in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(v) for v in value)
    return value


@dataclass(frozen=True)
class Catalog:
    schema_version: str
    catalog_version: str
    data: Mapping[str, Any]

    def entries(self, section: str) -> tuple:
        value = self.data.get(section)
        if not isinstance(value, tuple):
            raise CatalogError(f"catalog section '{section}' is missing or not a list")
        return value

    def by_id(self, section: str, entry_id: str):
        matches = [entry for entry in self.entries(section) if entry.get("id") == entry_id]
        if len(matches) != 1:
            raise CatalogError(f"catalog reference '{section}:{entry_id}' resolved {len(matches)} entries")
        return matches[0]

    def preferred_module(self, minimum_mm: float = 0.0) -> float:
        modules = tuple(float(v) for v in self.entries("gear_modules_mm"))
        return next((v for v in modules if v >= minimum_mm), modules[-1])

    def bearing_for_shaft(self, family_id: str, shaft_diameter_mm: float):
        family = self.by_id("bearing_families", family_id)
        exact = [v for v in family["variants"] if float(v["bore_mm"]) == float(shaft_diameter_mm)]
        if not exact:
            raise CatalogError(f"no {family_id} bearing for {shaft_diameter_mm:g} mm shaft")
        return exact[0]


def validate_catalog(raw: dict) -> None:
    required = {"schema_version", "catalog_version", "gear_profiles", "gear_modules_mm",
                "gear_face_width_rules", "shaft_families", "bearing_families",
                "fit_profiles", "clearance_profiles", "housing_wall_profiles"}
    missing = required - set(raw)
    if missing:
        raise CatalogError(f"catalog missing sections: {sorted(missing)}")
    if raw["schema_version"] != "maker2_catalog_v1":
        raise CatalogError(f"unsupported catalog schema '{raw['schema_version']}'")
    modules = raw["gear_modules_mm"]
    if not modules or any(not isinstance(v, (int, float)) or v <= 0 for v in modules):
        raise CatalogError("gear_modules_mm must contain positive numbers")
    if modules != sorted(set(modules)):
        raise CatalogError("gear_modules_mm must be unique and ascending")
    ids = set()
    for section in required - {"schema_version", "catalog_version", "gear_modules_mm"}:
        values = raw[section]
        if not isinstance(values, list) or not values:
            raise CatalogError(f"catalog section '{section}' must be a non-empty list")
        for entry in values:
            entry_id = entry.get("id") if isinstance(entry, dict) else None
            if not entry_id or entry_id in ids:
                raise CatalogError(f"duplicate or missing catalog id in '{section}': {entry_id!r}")
            ids.add(entry_id)
    for family in raw["bearing_families"]:
        variants = family.get("variants", [])
        if not variants or len({v["id"] for v in variants}) != len(variants):
            raise CatalogError(f"bearing family '{family['id']}' has invalid variants")
        for variant in variants:
            if min(float(variant[k]) for k in ("bore_mm", "outer_mm", "width_mm")) <= 0:
                raise CatalogError(f"bearing '{variant['id']}' has non-positive dimensions")


def load_catalog(path: str | Path | None = None) -> Catalog:
    source = Path(path) if path else Path(__file__).with_name("standards") / "catalog_v1.json"
    with source.open("r", encoding="utf-8") as stream:
        raw = json.load(stream)
    validate_catalog(raw)
    return Catalog(raw["schema_version"], raw["catalog_version"], _freeze(raw))
