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


def _mesh_distance_errors_world(plan, frames_world) -> list[GateError]:
    """For each gear-mesh power seam, center distance between the two frames' SOLVED WORLD
    positions must equal the sum of their pitch radii (from shaft_dia_mm). Reads the
    compiler's `assembly_frames_world` (post-assemble) — the boss no longer authors the
    gear-center coordinates, so this VALIDATES the placement the weld produced rather than
    the boss's guess. `frames_world` is a list of {sub, frame, xyz_m, rpy_rad}; the `sub`
    key is the namespaced sub id, so we match on frame name within the seam's parent/child
    sub (an instanced sub is matched on its base id prefix)."""
    out: list[GateError] = []
    # Index solved world positions by (sub_id_prefix, frame_name). The compiler keys by the
    # namespaced ns_id (== sub id for non-instanced subs), so a direct + prefix match covers
    # both. Store the first world xyz seen for each (sub, frame).
    by_key: dict = {}
    for e in (frames_world or []):
        by_key[(e.get("sub"), e.get("frame"))] = e.get("xyz_m")

    def _world_of(sub_id, frame_name):
        if (sub_id, frame_name) in by_key:
            return by_key[(sub_id, frame_name)]
        # instanced sub: match the first copy whose ns_id starts with "<sub_id>_"
        for (k_sub, k_fr), v in by_key.items():
            if k_fr == frame_name and (k_sub == sub_id or str(k_sub).startswith(sub_id + "_")):
                return v
        return None

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
            continue
        dia_p = float(getattr(pf, "shaft_dia_mm", 0.0) or 0.0)
        dia_c = float(getattr(cf, "shaft_dia_mm", 0.0) or 0.0)
        if dia_p <= 0 or dia_c <= 0:
            continue                         # can't verify without pitch diameters
        wp = _world_of(seam.parent_sub, seam.parent_frame)
        wc = _world_of(seam.child_sub, seam.child_frame)
        if wp is None or wc is None:
            continue                         # frame not solved -> assembler/frame gate's job
        want_m = (dia_p + dia_c) / 2.0 / 1000.0
        center_d = math.dist([float(v) for v in wp], [float(v) for v in wc])
        if want_m > 0 and abs(center_d - want_m) > _MESH_TOL_FRAC * want_m:
            out.append(GateError(
                "boss", "ERR_IFC_MESH_DIST",
                f"mesh seam '{seam.id}': the assembled gear centers are {center_d*1000:.1f} "
                f"mm apart but the pitch radii sum to {want_m*1000:.1f} mm "
                f"(>{_MESH_TOL_FRAC:.0%} off -> teeth won't engage); adjust the weld seam's "
                "frames/offset so the two gear centers land one pitch-center-distance apart, "
                "or fix the pitch diameters",
                seam.id))
    return out


def mesh_distance_errors(plan, frames_world) -> list[GateError]:
    """POST-ASSEMBLE gear-mesh distance gate: validate every mesh seam on the compiler's
    solved world frames (`model.assembly_frames_world`). Public entry for the orchestrator."""
    return _mesh_distance_errors_world(plan, frames_world)


def boss_gate(plan) -> list[GateError]:
    """Deterministic PRE-BUILD plan-level check: support chain (ERR_SUP_NOWELD) only. The
    gear-mesh distance check moved to `mesh_distance_errors` (post-assemble) because the boss
    no longer authors gear-center coordinates — spacing is validated on the solved assembly,
    not the plan. Returns [] on pass."""
    return _support_errors(plan)
