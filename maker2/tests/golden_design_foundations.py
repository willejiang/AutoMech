"""Focused golden checks for requirements, catalog, DAG, and frozen contracts.
Run: python -m maker2.tests.golden_design_foundations
"""
from __future__ import annotations

from dataclasses import FrozenInstanceError

from maker2.design.catalog import CatalogError, load_catalog, validate_catalog
from maker2.design.contracts import FunctionalEnvelope, Hardpoint, HardpointContract
from maker2.design.ir import DesignIntentIR, fingerprint
from maker2.design.parameter_dag import ParameterDAG, ParameterGraphError, enumerate_candidates
from maker2.design.requirements import extract_requirements

_I4 = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))


def requirements_checks():
    facts = extract_requirements("Need a 9:1 reducer, 1200 rpm input, 2 N*m torque, under 25 cm long.")
    assert [(f.kind, f.value, f.unit) for f in facts] == [
        ("ratio", 9.0, "1"), ("speed", 1200.0, "rpm"),
        ("torque", 2.0, "N*m"), ("length", 0.25, "m")]
    assert facts == extract_requirements("Need a 9:1 reducer, 1200 rpm input, 2 N*m torque, under 25 cm long.")
    assert len({f.id for f in facts}) == len(facts)


def catalog_checks():
    catalog = load_catalog()
    assert catalog.preferred_module(1.4) == 1.5
    assert catalog.bearing_for_shaft("bearing_6000", 12)["id"] == "bearing_6001_12"
    try:
        catalog.data["catalog_version"] = "tampered"
        raise AssertionError("catalog mapping is mutable")
    except TypeError:
        pass
    try:
        validate_catalog({"schema_version": "maker2_catalog_v1"})
        raise AssertionError("incomplete catalog accepted")
    except CatalogError:
        pass


def dag_checks():
    graph = ParameterDAG()
    graph.add_input("module", 2.0, "mm", ("catalog:m2",))
    graph.add_input("teeth", 20, "1", ("choice:z20",))
    graph.add_formula("pitch_diameter", "mm", ("module", "teeth"), lambda m, z: m * z)
    values = graph.evaluate()
    assert values["pitch_diameter"].value == 40.0
    assert values["pitch_diameter"].provenance[-1] == "formula:pitch_diameter"
    cycle = ParameterDAG()
    cycle.add_formula("a", "1", ("b",), lambda b: b)
    cycle.add_formula("b", "1", ("a",), lambda a: a)
    try:
        cycle.evaluate()
        raise AssertionError("parameter cycle accepted")
    except ParameterGraphError as exc:
        assert "cycle" in str(exc)
    missing = ParameterDAG()
    missing.add_formula("a", "1", ("unknown",), lambda value: value)
    try:
        missing.evaluate()
        raise AssertionError("missing input accepted")
    except ParameterGraphError as exc:
        assert "missing" in str(exc)
    candidates = enumerate_candidates({"b": (2, 1), "a": (1, 2)}, lambda c: (c["a"] + c["b"],))
    assert candidates[0] == {"a": 1, "b": 1}


def ir_contract_checks():
    intent_a = DesignIntentIR("t", (("housing", "h"),), discrete_choices=(("layout", "linear"),))
    intent_b = DesignIntentIR("t", (("housing", "h"),), discrete_choices=(("layout", "linear"),))
    assert fingerprint(intent_a) == fingerprint(intent_b)
    hp = Hardpoint("seat", "housing", "mount", _I4, _I4, (1, 0, 0))
    contract = HardpointContract((("housing", _I4),), (hp,),
                                 (FunctionalEnvelope("housing", (-1, -1, -1), (1, 1, 1)),),
                                 "geometry_compiler_v1", "catalog_v1.0.0", "design:x")
    same = HardpointContract((("housing", _I4),), (hp,), contract.envelopes,
                             "geometry_compiler_v1", "catalog_v1.0.0", "design:x")
    assert contract.contract_hash == same.contract_hash and not contract.validate()
    assert contract.view("housing").hardpoints == (hp,)
    try:
        hp.role = "mesh"
        raise AssertionError("hardpoint is mutable")
    except FrozenInstanceError:
        pass
    try:
        HardpointContract(contract.root_transforms, contract.hardpoints, contract.envelopes,
                          contract.compiler_version, contract.catalog_version, contract.design_hash,
                          "contract_v1:bad")
        raise AssertionError("stale contract hash accepted")
    except ValueError:
        pass


if __name__ == "__main__":
    requirements_checks(); catalog_checks(); dag_checks(); ir_contract_checks()
    print("golden design foundations: PASS")
