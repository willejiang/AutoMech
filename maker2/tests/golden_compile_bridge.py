"""Golden: the plan->compiler bridge freezes a valid zero-DOF contract."""
from maker2.design.bridge import (compile_from_plan, derive_intent,
                                   recognize_template)
from maker2.design.requirements import extract_requirements
from maker2.design.templates.gear_reducer import TEMPLATE_ID


class _Sub:
    def __init__(self, sub_id):
        self.id = sub_id


class _Plan:
    def __init__(self, ids):
        self.subassemblies = [_Sub(i) for i in ids]


REDUCER_IDS = ("housing", "input_stage", "intermediate_stage", "output_stage")
PROMPT = "a two-stage gear reducer with a 12:1 reduction ratio"


def main():
    reducer = _Plan(REDUCER_IDS)
    assert recognize_template(reducer) == TEMPLATE_ID

    # Unrecognized topology is not compilable; auto returns the legacy signal.
    other = _Plan(("frame", "rotor"))
    assert recognize_template(other) is None
    assert compile_from_plan(other, PROMPT, mode="auto") == (None, None)

    # required mode must raise on unsupported topology, not silently pass.
    try:
        compile_from_plan(other, PROMPT, mode="required")
        raise AssertionError("required mode should reject unsupported topology")
    except RuntimeError:
        pass

    facts = extract_requirements(PROMPT)
    assert any(f.kind == "ratio" and abs(f.value - 12.0) < 1e-9 for f in facts)
    intent = derive_intent(reducer, facts, TEMPLATE_ID)
    assert intent.template_id == TEMPLATE_ID
    assert dict(intent.topology_roles) == {r: r for r in REDUCER_IDS}

    compiled, contract = compile_from_plan(reducer, PROMPT, mode="auto")
    assert contract is not None and compiled is not None
    assert compiled.solve_result.dof == 0
    assert contract.validate() == ()
    assert {r[0] for r in contract.root_transforms} == set(REDUCER_IDS)
    assert contract.contract_hash.startswith("contract_v1:")

    # Determinism: identical inputs produce an identical frozen contract.
    _c2, contract2 = compile_from_plan(reducer, PROMPT, mode="auto")
    assert contract2.contract_hash == contract.contract_hash

    # legacy mode skips the compiler entirely.
    assert compile_from_plan(reducer, PROMPT, mode="legacy") == (None, None)

    print("golden compile bridge: PASS")


if __name__ == "__main__":
    main()
