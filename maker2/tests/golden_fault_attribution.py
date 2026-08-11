"""Golden six-domain attribution and regeneration gate."""
from evaluator.attribution import attribute_from_evidence, diagnosis
from maker2.physics_probes import route_refinement


def main():
    d = attribute_from_evidence(manifest_diff={"ok": False,
        "missing": [{"kind": "transmission", "name": "lock"}]})
    assert d["fault_domain"] == "builder_compiler" and d["verified"]
    assert not route_refinement(d)["allow_agent_refinement"]
    for domain in ("runner_scenario", "simulator_numerics", "evaluator"):
        x = diagnosis(domain, "synthetic", "test", verified=True)
        assert not route_refinement(x)["allow_agent_refinement"]
    for domain in ("agent_geometry", "agent_ir"):
        x = diagnosis(domain, "synthetic", "test", verified=True,
                      culprits=[{"kind": "part", "name": "rod"}])
        assert route_refinement(x)["allow_agent_refinement"]
    x = diagnosis("agent_geometry", "guess", "test", verified=False,
                  confidence=0.99, culprits=[{"kind": "part", "name": "rod"}])
    assert not route_refinement(x)["allow_agent_refinement"]
    x = attribute_from_evidence(manifest_diff={"ok": True}, hard_pass=True,
                                evaluator_verdict="FAIL")
    assert x["fault_domain"] == "evaluator" and x["verified"]
    print("golden fault attribution: PASS")

if __name__ == "__main__": main()
