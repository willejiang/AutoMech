"""No-regeneration deterministic probes over an immutable generated artifact."""
from __future__ import annotations

import json
from pathlib import Path


def manifest_probe(model, manifest_path: str) -> dict:
    from evaluator.attribution import compare_authored_ir_compiled
    p = Path(manifest_path)
    if not p.exists():
        return {"probe": "manifest_diff", "ok": False,
                "missing_manifest": str(p)}
    manifest = json.loads(p.read_text(encoding="utf-8"))
    return {"probe": "manifest_diff", **compare_authored_ir_compiled(model, manifest)}


def route_refinement(diagnosis: dict) -> dict:
    """Single authoritative gate used by single-agent and hierarchy loops."""
    routing = diagnosis.get("routing") or {}
    allow = bool(routing.get("allow_agent_refinement"))
    return {"allow_agent_refinement": allow,
            "action": routing.get("action") or ("refine_agent" if allow else "halt_harness"),
            "fault_domain": diagnosis.get("fault_domain", "evaluator"),
            "verified": bool(diagnosis.get("verified")),
            "culprits": list(diagnosis.get("culprit_entities") or [])}
