"""Phase 0 — schema/output gates: is the agent's JSON SEMANTICALLY valid?

The existing `boss._validate_plan` / `manager._validate_model` do in-place NORMALIZATION
(slugify names, remap references, assign mesh filenames) and raise on HARD structural
integrity errors (unknown refs, self-loops). They run at parse time and cannot be made
read-only without breaking parsing.

These gates run on the ALREADY-NORMALIZED plan/model and check the SEMANTIC rules the
validators don't — enum validity, driver-on-a-fixed-part, degenerate size_mm (the class
of bug that crashes MjModel.from_xml), dangling mesh_pairs. They are the cheapest checks
in the pipeline (JSON only, no geometry, no render), so they gate FIRST. A failure routes
the specific ERR_SCHEMA_* code back to the agent, never a stack trace.
"""

from __future__ import annotations

from . import GateError

# Reuse the boss's authoritative enum sets so the gate can't drift from the parser.
from ..boss import _VALID_ROLES  # {"mount","power_in","power_out","mesh"}

_VALID_DOF = {"fixed", "spin", "free"}
_VALID_SEAM_KIND = {"weld", "power"}

# Dimension keys that must be strictly positive per shape_hint. Free-text shapes are
# skipped (we cannot know their required dims), and any dim present is checked for
# non-negativity regardless of shape.
_REQUIRED_DIMS = {
    "box": ("x", "y", "z"),
    "cube": ("x", "y", "z"),
    "cylinder": ("radius", "height"),
    "sphere": ("radius",),
}


def _positive(v) -> bool:
    try:
        return float(v) > 0.0
    except (TypeError, ValueError):
        return False


def manager_schema_gate(model) -> list[GateError]:
    """Semantic-schema checks on ONE normalized subassembly model. Returns [] on pass.

    Runs right after decompose, before manager_gate — so the geometry checks can assume
    a valid model (real dof, non-degenerate sizes, resolvable mesh_pairs)."""
    errors: list[GateError] = []
    link_names = {l.name for l in model.links}

    for l in model.links:
        dof = getattr(l, "dof", "fixed")
        if dof not in _VALID_DOF:
            errors.append(GateError(
                "manager", "ERR_SCHEMA_MGR_DOF",
                f"link '{l.name}' has dof '{dof}' (expected one of {sorted(_VALID_DOF)})",
                l.name))
        # driver must sit on a MOVABLE part — a fixed part cannot be actuated.
        if getattr(l, "driver", False) and dof == "fixed":
            errors.append(GateError(
                "manager", "ERR_SCHEMA_MGR_DRIVER_FIXED",
                f"link '{l.name}' is driver=true but dof='fixed'; the driven part must "
                "spin or be free",
                l.name))
        # a spin part needs a real rotation axis.
        if dof == "spin":
            axis = tuple(getattr(l, "spin_axis", (0.0, 0.0, 1.0)) or ())
            if len(axis) != 3 or not any(abs(float(c)) > 1e-9 for c in axis):
                errors.append(GateError(
                    "manager", "ERR_SCHEMA_MGR_NOAXIS",
                    f"spin link '{l.name}' has a zero/invalid spin_axis {axis}",
                    l.name))
        # size_mm must be non-degenerate for a known shape (the MJCF-crash guard), and
        # no declared dimension may be zero/negative for ANY shape.
        size = getattr(l, "size_mm", {}) or {}
        hint = (getattr(l, "shape_hint", "") or "").strip().lower()
        req = _REQUIRED_DIMS.get(hint)
        if req is not None:
            missing = [k for k in req if not _positive(size.get(k))]
            if missing:
                errors.append(GateError(
                    "manager", "ERR_SCHEMA_MGR_DEGENERATE",
                    f"link '{l.name}' (shape '{hint}') needs positive {list(req)}; "
                    f"missing/zero: {missing} (size_mm={size})",
                    l.name))
        else:
            bad = [k for k, v in size.items() if not _positive(v)]
            if bad:
                errors.append(GateError(
                    "manager", "ERR_SCHEMA_MGR_DEGENERATE",
                    f"link '{l.name}' has non-positive size_mm dims {bad} (size_mm={size})",
                    l.name))

    # mesh_pairs must name real links (else the transmission detector + MJCF break).
    for pair in getattr(model, "mesh_pairs", []) or []:
        if len(pair) != 2:
            errors.append(GateError(
                "manager", "ERR_SCHEMA_MGR_MESHREF",
                f"mesh_pair {tuple(pair)} must name exactly two links", str(pair)))
            continue
        for name in pair:
            if name not in link_names:
                errors.append(GateError(
                    "manager", "ERR_SCHEMA_MGR_MESHREF",
                    f"mesh_pair names unknown link '{name}'", name))

    # exactly one driver at most within a sub (the machine has one input overall; a sub
    # may legitimately have zero, but two driven parts in one sub is a modeling error).
    drivers = [l.name for l in model.links if getattr(l, "driver", False)]
    if len(drivers) > 1:
        errors.append(GateError(
            "manager", "ERR_SCHEMA_MGR_DRIVERS",
            f"more than one driver link in this subassembly {drivers}",
            ",".join(drivers)))
    return errors


def boss_schema_gate(plan) -> list[GateError]:
    """Semantic-schema checks on the normalized SubassemblyPlan. Returns [] on pass.

    Runs right after parse_plan, before boss_gate — enum validity, dangling mesh_pairs,
    at-most-one driver, no empty subassembly. Structural integrity (frames exist,
    weld-graph spans root) stays in _validate_plan / boss_gate."""
    errors: list[GateError] = []
    sub_ids = {s.id for s in plan.subassemblies}

    for s in plan.subassemblies:
        if not (s.frames or []):
            errors.append(GateError(
                "boss", "ERR_SCHEMA_BOSS_EMPTY_SUB",
                f"subassembly '{s.id}' declares no interface frames", s.id))
        for fr in s.frames or []:
            if fr.role not in _VALID_ROLES:
                errors.append(GateError(
                    "boss", "ERR_SCHEMA_BOSS_ROLE",
                    f"frame '{fr.name}' on '{s.id}' has role '{fr.role}' "
                    f"(expected one of {sorted(_VALID_ROLES)})",
                    f"{s.id}:{fr.name}"))

    for seam in plan.seams:
        if seam.kind not in _VALID_SEAM_KIND:
            errors.append(GateError(
                "boss", "ERR_SCHEMA_BOSS_KIND",
                f"seam '{seam.id}' has kind '{seam.kind}' "
                f"(expected one of {sorted(_VALID_SEAM_KIND)})",
                seam.id))
        # a power seam that names a mesh_pair: both must be non-empty names (link-level
        # existence is verified post-build; here we only require they were authored).
        mp = tuple(seam.mesh_pair or ())
        if seam.kind == "power" and mp and (len(mp) != 2 or not all(mp)):
            errors.append(GateError(
                "boss", "ERR_SCHEMA_BOSS_MESHREF",
                f"seam '{seam.id}' mesh_pair {mp} must name two links", seam.id))
        if seam.owner_sub and seam.owner_sub not in sub_ids:
            errors.append(GateError(
                "boss", "ERR_SCHEMA_BOSS_OWNER",
                f"seam '{seam.id}' owner_sub '{seam.owner_sub}' is not a subassembly",
                seam.id))

    drivers = [s.id for s in plan.seams if s.driver]
    if len(drivers) > 1:
        errors.append(GateError(
            "boss", "ERR_SCHEMA_BOSS_DRIVERS",
            f"more than one seam has driver=true {drivers} (the machine has ONE input)",
            ",".join(drivers)))
    return errors
