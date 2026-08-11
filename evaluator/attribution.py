"""Deterministic fault-domain attribution and regeneration routing."""
from __future__ import annotations

FAULT_DOMAINS = {"agent_geometry", "agent_ir", "builder_compiler",
                 "runner_scenario", "simulator_numerics", "evaluator"}
AGENT_DOMAINS = {"agent_geometry", "agent_ir"}


def diagnosis(fault_domain: str, fault_code: str, reason: str, *,
              evidence=None, culprits=None, confidence: float = 1.0,
              verified: bool = False, persistence=None) -> dict:
    if fault_domain not in FAULT_DOMAINS:
        fault_domain = "evaluator"
        fault_code = "invalid_fault_domain"
        verified = False
    culprits = list(culprits or [])
    allow = bool(verified and fault_domain in AGENT_DOMAINS and culprits)
    action = "refine_agent" if allow else (
        "retry_harness" if fault_domain in {"runner_scenario", "simulator_numerics"}
        else "halt_harness")
    legacy = "scenario" if fault_domain == "runner_scenario" else "structure"
    return {"diagnosis_version": 2, "fault_domain": fault_domain,
            "fault_code": fault_code, "confidence": max(0.0, min(1.0, confidence)),
            "verified": bool(verified), "reason": str(reason)[:2000],
            "culprit_entities": culprits, "evidence": list(evidence or []),
            "persistence": dict(persistence or {}),
            "routing": {"action": action, "allow_agent_refinement": allow},
            "legacy": {"cause": legacy}}


def compare_authored_ir_compiled(model, manifest: dict) -> dict:
    """Find authored semantic entities absent/duplicated/rejected in a manifest."""
    authored = []
    authored += [("link", x.name) for x in model.links]
    authored += [("motion_joint", x.name) for x in (getattr(model, "motion_joints", None) or [])]
    authored += [("relation", x.name) for x in (getattr(model, "relations", None) or [])]
    authored += [("transmission", x.name) for x in (getattr(model, "transmissions", None) or [])]
    authored += [("planetary_stage", x.name) for x in (getattr(model, "planetary_stages", None) or [])]
    # Agent compiler manifest v3 owns topology decisions directly. Its gate has already
    # required one decision for poses, ports and roles too; this compatibility probe maps
    # the established entity IDs back to the legacy diff shape used by attribution.
    if int(manifest.get("manifest_version") or 0) >= 3:
        counts = {}
        rejected = []
        for row in manifest.get("decisions") or []:
            entity_id = row.get("entity_id")
            if not entity_id:
                continue
            counts[entity_id] = counts.get(entity_id, 0) + 1
            if row.get("action") not in ("emitted", "represented_by"):
                rejected.append({"kind": entity_id.split("/", 1)[0],
                                 "name": entity_id.split("/", 1)[-1],
                                 "reason": row.get("reason", "invalid action")})
        expected = [f"{kind}/{name}" for kind, name in authored]
        missing = [{"kind": item.split("/", 1)[0], "name": item.split("/", 1)[1]}
                   for item in expected if not counts.get(item)]
        duplicate = [{"kind": item.split("/", 1)[0], "name": item.split("/", 1)[1],
                      "count": counts[item]} for item in expected if counts.get(item, 0) > 1]
        return {"ok": not (missing or duplicate or rejected), "missing": missing,
                "duplicate": duplicate, "rejected": rejected}
    records = []
    for key in ("bodies", "constraints", "transmissions", "planetary_stages"):
        records.extend(manifest.get(key) or [])
    by_key = {}
    for rec in records:
        key = (rec.get("source_kind"), rec.get("source_name"))
        by_key.setdefault(key, []).append(rec)
    missing, duplicate, rejected = [], [], []
    for item in authored:
        rows = by_key.get(item, [])
        if not rows:
            missing.append({"kind": item[0], "name": item[1]})
        elif len(rows) > 1:
            duplicate.append({"kind": item[0], "name": item[1], "count": len(rows)})
        elif rows[0].get("rejected") or not rows[0].get("compiled"):
            # A relation represented by an explicit motion edge is intentional, not loss.
            if rows[0].get("reason") != "represented by explicit motion joint":
                rejected.append({"kind": item[0], "name": item[1],
                                 "reason": rows[0].get("reason", "")})
    ok = not (missing or duplicate or rejected)
    return {"ok": ok, "missing": missing, "duplicate": duplicate, "rejected": rejected}


def attribute_from_evidence(*, manifest_diff=None, metrics=None,
                            evaluator_verdict=None, hard_pass=None) -> dict:
    diff = manifest_diff or {}
    if not diff.get("ok", True):
        return diagnosis("builder_compiler", "compiled_semantics_mismatch",
                         "Authored mechanical semantics do not match the builder manifest.",
                         verified=True,
                         evidence=[{"kind": "manifest_diff", "observation": diff}],
                         culprits=(diff.get("missing") or diff.get("duplicate") or
                                   diff.get("rejected") or []))
    health = (metrics or {}).get("numerical_health") or {}
    if health.get("finite") is False or health.get("constraint_residual_max", 0) > 1e-2:
        return diagnosis("simulator_numerics", "unhealthy_simulation",
                         "The simulator reported non-finite state or excessive constraint residual.",
                         verified=True, evidence=[{"kind": "numerical_health",
                                                   "observation": health}])
    if hard_pass is not None and evaluator_verdict is not None:
        ev_pass = str(evaluator_verdict).upper() == "PASS"
        if ev_pass != bool(hard_pass):
            return diagnosis("evaluator", "verdict_contradicts_hard_metrics",
                             "Evaluator verdict contradicts deterministic metrics.", verified=True,
                             evidence=[{"kind": "hard_metric_comparison",
                                        "expected": bool(hard_pass), "actual": ev_pass}])
    return diagnosis("evaluator", "insufficient_verified_evidence",
                     "No fault domain survived deterministic verification.", verified=False)
