"""Phase 2 — boss_gate: plan-level support + gear-mesh distance (deterministic).

NOTE: currently a STUB that returns [] so the Phase-0 orchestrator wiring imports and the
boss schema gate can run standalone. Phase 2 fills in:
  - weld-graph spans every sub to root_sub (reuse boss._validate_plan BFS) -> ERR_SUP_NOWELD
  - gear mesh center-distance == sum of pitch radii FROM THE PLAN (MountFrame.shaft_dia_mm)
    -> ERR_IFC_MESH_DIST
See .claude/plans/precious-humming-wand.md Phase 2.
"""

from __future__ import annotations

from . import GateError


def boss_gate(plan) -> list[GateError]:
    """Deterministic plan-level checks (support chain + mesh distance). STUB: no-op
    until Phase 2."""
    return []
