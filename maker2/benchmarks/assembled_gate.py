"""Phase 4 — assembled_gate: the ONLY grounding/support check, on the assembled machine.

Per-sub gates deliberately skip grounding (a subassembly hangs off shafts whose bearings
live in another sub — it is not free-standing). Support is a WHOLE-MACHINE property, so it
is checked here, after the assembler stitches everything:

- WELD CHAIN (re-assert): every sub must reach root_sub through weld seams. The boss gate
  already checks this pre-build; re-running against the assembled plan is a cheap
  backstop -> ERR_SUP_FLOAT (severity interface -> boss re-plan).
- GROUNDING (needs realized geometry): the root_sub (the mainplate) must be at the BOTTOM
  of the machine — its lowest world-z should be <= every other sub's lowest z (within a
  small band). If some other sub sits below the base, the machine would topple/rest on the
  wrong part -> ERR_SUP_GROUND. This is FRAME-INDEPENDENT (the authored origin is
  arbitrary — the boss even puts it at the center-wheel axis), so we compare subs to each
  other, not to an absolute z=0.

Reuses precheck.load_robot + precheck._sub_bounds + boss_gate._support_errors. No LLM.
"""

from __future__ import annotations

import numpy as np

from . import GateError
from .boss_gate import _support_errors

# A sub may dip this far below the root before it counts as "below the base" (meters).
_GROUND_BAND_M = 0.005


def assembled_gate(plan, assembled_urdf: str, log_fn=print) -> list[GateError]:
    """Deterministic whole-machine support checks. Returns [] on pass.

    ``assembled_urdf`` is the assembler's stitched URDF (namespaced links). A load
    failure is non-fatal here (precheck reports that separately) -> we skip grounding and
    return only the plan-level weld-chain result."""
    # 1. Weld chain (plan-level; also catches a regression if the assembled plan differs).
    errors = list(_support_errors(plan))
    # Re-tag the weld-chain code as the assembled-stage code for the workbench.
    for e in errors:
        e.code = "ERR_SUP_FLOAT"

    # 2. Grounding — needs realized geometry.
    try:
        from ..precheck import _sub_bounds, load_robot
        robot = load_robot(assembled_urdf)
    except Exception as e:
        log_fn(f"[assembled] grounding check skipped (URDF load: {e})")
        return errors

    bounds = _sub_bounds(robot, plan)
    root_b = bounds.get(plan.root_sub)
    if root_b is None:
        log_fn(f"[assembled] grounding check skipped (no geometry bounds for root "
               f"'{plan.root_sub}')")
        return errors
    root_min_z = float(root_b[0][2])

    below = []
    for sid, b in bounds.items():
        if sid == plan.root_sub or b is None:
            continue
        sub_min_z = float(b[0][2])
        if sub_min_z < root_min_z - _GROUND_BAND_M:
            below.append((sid, (root_min_z - sub_min_z) * 1000.0))
    for sid, drop_mm in sorted(below, key=lambda x: -x[1]):
        errors.append(GateError(
            "boss", "ERR_SUP_GROUND",
            f"subassembly '{sid}' sits {drop_mm:.1f} mm BELOW the base '{plan.root_sub}' "
            "— the machine would rest on it instead of the mainplate; the base must be "
            "the lowest structural part",
            sid))
    return errors
