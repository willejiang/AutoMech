"""Focused deterministic reducer compiler golden.
Run: python -m maker2.tests.golden_design_reducer_compile
"""
from __future__ import annotations

from maker2.design.compiler import DesignCompileError, compile_design
from maker2.design.gates import intent_gate
from maker2.design.ir import COMPILER_VERSION, DesignIntentIR, canonical_json
from maker2.design.requirements import extract_requirements
from maker2.design.templates.gear_reducer import TEMPLATE_ID


def intent_for(fact_id: str, *, layout="linear"):
    return DesignIntentIR(
        TEMPLATE_ID,
        (("housing", "main_housing"), ("input_stage", "input_shaft_module"),
         ("intermediate_stage", "compound_stage"), ("output_stage", "output_shaft_module")),
        (fact_id,),
        ("spur_20deg_full_depth", "spur_face_10m", "housing_printed_light"),
        ("shaft_metric_light", "bearing_6000"),
        layout,
    )


def compile_checks():
    facts = extract_requirements("Build a parallel-shaft two-stage reducer with a 9:1 reduction ratio.")
    ratio_fact = next(fact for fact in facts if fact.kind == "ratio")
    first, contract_a = compile_design(intent_for(ratio_fact.id), facts)
    second, contract_b = compile_design(intent_for(ratio_fact.id), facts)
    assert first.design_hash == second.design_hash
    assert first.contract_hash == second.contract_hash == contract_a.contract_hash == contract_b.contract_hash
    assert first.compiler_version == COMPILER_VERSION and first.solve_result.status == "okay"
    assert first.solve_result.dof == 0 and not first.solve_result.failed_constraint_ids
    params = first.parameter_map()
    assert abs(params["compiled_ratio"].value - 9.0) < 0.02
    assert params["stage_1_center_distance_mm"].value > 0
    assert params["bearing_plane_rear_mm"].value > params["bearing_plane_front_mm"].value
    assert len(contract_a.root_transforms) == 4
    assert len(contract_a.hardpoints) == 16
    housing = contract_a.view("main_housing")
    assert len(housing.hardpoints) == 6
    for role in ("input_stage", "intermediate_stage", "output_stage"):
        sub_id = intent_for(ratio_fact.id).role_map()[role]
        view = contract_a.view(sub_id)
        expected = 4 if role == "intermediate_stage" else 3
        assert len(view.hardpoints) == expected and len(view.by_role("mount")) == 2
    assert len(contract_a.view("compound_stage").by_role("mesh")) == 2
    problem = first.problem
    assert problem.expected_dof == 0 and len(problem.constraints) == 5
    assert canonical_json(first) == canonical_json(second)


def gate_checks():
    facts = extract_requirements("9:1 reducer")
    fact = facts[0]
    bad = DesignIntentIR(TEMPLATE_ID,
        (("housing", "h"), ("input_stage", "i"), ("intermediate_stage", "m"), ("output_stage", "o")),
        (fact.id,), discrete_choices=(("center_distance_mm", "80"),))
    errors = intent_gate(bad, template_ids={TEMPLATE_ID}, fact_ids={fact.id}, catalog_ids=set())
    assert any(error.code == "ERR_RAW_NUMERIC_AUTHORITY" for error in errors)
    try:
        compile_design(intent_for("req_missing"), facts)
        raise AssertionError("unknown requirement reference accepted")
    except DesignCompileError as exc:
        assert any(getattr(error, "code", "") == "ERR_REQUIREMENT_REF" for error in exc.errors)


if __name__ == "__main__":
    compile_checks(); gate_checks()
    print("golden deterministic reducer compile: PASS")
