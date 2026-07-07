"""Phase 2 — boss_gate: plan-level support chain + gear-mesh distance (deterministic).

Runs on the SubassemblyPlan right after parse, BEFORE any manager builds. Two checks the
schema gate can't do (they need cross-seam geometry from the plan):

- SUPPORT CHAIN: the WELD-seam graph must connect every subassembly back to root_sub. A
  sub reachable only via a power/mesh seam (gears touching, nothing structurally holding
  it) is the "castle in the air" case -> ERR_SUP_NOWELD. (Decision: subs are NOT
  free-standing, so support is a BOSS responsibility, checked here before any manager
  runs. Mirrors the BFS boss._validate_plan already does, but emits a routed code instead
  of raising.)
- GEAR-MESH DISTANCE FROM THE PLAN: for each power seam with a mesh_pair, the two frames
  it references carry global xyz_m + shaft_dia_mm (pitch diameter). Their center distance
  must equal the sum of pitch radii (tol _MESH_TOL_FRAC) or the teeth can't engage ->
  ERR_IFC_MESH_DIST. Caught before geometry exists.

Reuses precheck._MESH_TOL_FRAC. No LLM.
"""

from __future__ import annotations

import math

from . import GateError
from ..precheck import _MESH_TOL_FRAC


def _frame_on(sub, frame_name):
    for fr in (sub.frames or []):
        if fr.name == frame_name:
            return fr
    return None


def _support_errors(plan) -> list[GateError]:
    """Every sub must chain to root_sub through WELD seams. A power/mesh-only connection
    is not structural support."""
    sub_ids = {s.id for s in plan.subassemblies}
    if plan.root_sub not in sub_ids:
        return [GateError("boss", "ERR_SUP_NOWELD",
                          f"root_sub '{plan.root_sub}' is not a subassembly", plan.root_sub)]
    adjacency: dict[str, list[str]] = {}
    for seam in plan.seams:
        if seam.kind == "weld":
            adjacency.setdefault(seam.parent_sub, []).append(seam.child_sub)
            adjacency.setdefault(seam.child_sub, []).append(seam.parent_sub)
    visited: set[str] = set()
    stack = [plan.root_sub]
    while stack:
        node = stack.pop()
        if node in visited:
            continue
        visited.add(node)
        stack.extend(adjacency.get(node, []))
    out: list[GateError] = []
    for sid in sorted(sub_ids - visited):
        out.append(GateError(
            "boss", "ERR_SUP_NOWELD",
            f"subassembly '{sid}' is not welded to the machine (no weld-seam path to "
            f"root '{plan.root_sub}') — a gear mesh is not structural support; add a "
            "weld seam so it is held in place",
            sid))
    return out


def _mesh_distance_errors(plan) -> list[GateError]:
    """For each gear-mesh power seam, center distance between the two frames must equal
    the sum of their pitch radii (from shaft_dia_mm)."""
    out: list[GateError] = []
    for seam in plan.seams:
        if seam.kind != "power" or not seam.mesh_pair or len(seam.mesh_pair) != 2:
            continue
        p_sub = plan.sub_by_id(seam.parent_sub)
        c_sub = plan.sub_by_id(seam.child_sub)
        if not p_sub or not c_sub:
            continue
        pf = _frame_on(p_sub, seam.parent_frame)
        cf = _frame_on(c_sub, seam.child_frame)
        if pf is None or cf is None:
            continue                         # frame existence is the schema/frame gate's job
        dia_p = float(getattr(pf, "shaft_dia_mm", 0.0) or 0.0)
        dia_c = float(getattr(cf, "shaft_dia_mm", 0.0) or 0.0)
        if dia_p <= 0 or dia_c <= 0:
            # Boss didn't fix pitch diameters on a mesh seam — can't verify spacing here;
            # the post-build precheck degenerate-gap check remains the backstop.
            continue
        want_m = (dia_p + dia_c) / 2.0 / 1000.0     # sum of pitch radii, mm -> m
        dp = tuple(float(v) for v in (pf.xyz_m or (0, 0, 0)))
        dc = tuple(float(v) for v in (cf.xyz_m or (0, 0, 0)))
        center_d = math.dist(dp, dc)
        if want_m > 0 and abs(center_d - want_m) > _MESH_TOL_FRAC * want_m:
            out.append(GateError(
                "boss", "ERR_IFC_MESH_DIST",
                f"mesh seam '{seam.id}': frames are {center_d*1000:.1f} mm apart but the "
                f"pitch radii sum to {want_m*1000:.1f} mm "
                f"(>{_MESH_TOL_FRAC:.0%} off -> teeth won't engage); move the frames or "
                "fix the pitch diameters",
                seam.id))
    return out


def boss_gate(plan) -> list[GateError]:
    """Deterministic plan-level checks: support chain (ERR_SUP_NOWELD) + gear-mesh
    distance (ERR_IFC_MESH_DIST). Returns [] on pass."""
    return _support_errors(plan) + _mesh_distance_errors(plan)
