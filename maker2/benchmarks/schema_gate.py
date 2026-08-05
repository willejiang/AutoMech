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

_VALID_DOF = {"fixed", "spin", "slide", "free"}
_VALID_SEAM_KIND = {"weld", "power"}

# Dimension keys that must be strictly positive per shape_hint. Free-text shapes are
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
                "spin, slide, or be free",
                l.name))
        # motion parts need a real axis.
        if dof == "spin":
            axis = tuple(getattr(l, "spin_axis", (0.0, 0.0, 1.0)) or ())
            if len(axis) != 3 or not any(abs(float(c)) > 1e-9 for c in axis):
                errors.append(GateError(
                    "manager", "ERR_SCHEMA_MGR_NOAXIS",
                    f"spin link '{l.name}' has a zero/invalid spin_axis {axis}",
                    l.name))
        if dof == "slide":
            axis = tuple(getattr(l, "slide_axis", (1.0, 0.0, 0.0)) or ())
            if len(axis) != 3 or not any(abs(float(c)) > 1e-9 for c in axis):
                errors.append(GateError(
                    "manager", "ERR_SCHEMA_MGR_NOSLIDEAXIS",
                    f"slide link '{l.name}' has a zero/invalid slide_axis {axis}",
                    l.name))
        # DEGENERACY guard (the MjModel.from_xml crash case), NOT a naming policy.
        # Managers name dims freely (gear_dia, wheel_dia, tube_outer_dia, wheel_thk,
        # length/width/thickness, ...), so we do NOT enumerate keys. We fold the known
        # aliases onto canonical names first (C2) so the semantic match below is robust,
        # then ask the only question that matters: does the part have enough POSITIVE size
        # to form a non-degenerate solid? Flag only a genuinely empty/zero part.
        from .vocab import canonical_size
        hint = (getattr(l, "shape_hint", "") or "").strip().lower()
        size = canonical_size(getattr(l, "size_mm", {}) or {}, hint)

        def _has(pred) -> bool:
            return any(pred(k.lower()) and _positive(v) for k, v in size.items())

        # any radial dim (radius / *_dia / *diameter / *_r) is positive?
        radial = _has(lambda k: k == "radius" or k.endswith("radius")
                      or k.endswith("dia") or k.endswith("diameter"))
        # any axial/extent dim is positive?
        axial = _has(lambda k: k in ("x", "y", "z", "h")
                     or k.endswith(("height", "length", "width", "depth",
                                    "thickness", "thk", "_h", "_z")))
        # NON-size keys (tooth counts, module, ratios) are not extents — exclude them
        # from the "any positive number" fallback so they can't mask a truly empty part.
        _NON_SIZE = ("teeth", "count", "num", "module", "ratio", "pressure_angle",
                     "angle", "pitch_count")
        any_positive_size = _has(
            lambda k: not any(t in k for t in _NON_SIZE))

        if hint in ("cylinder",) and not (radial and axial):
            errors.append(GateError(
                "manager", "ERR_SCHEMA_MGR_DEGENERATE",
                f"cylinder '{l.name}' lacks a positive radial ({'ok' if radial else 'MISSING'}) "
                f"and/or axial ({'ok' if axial else 'MISSING'}) size (size_mm={size})",
                l.name))
        elif hint in ("sphere",) and not radial:
            errors.append(GateError(
                "manager", "ERR_SCHEMA_MGR_DEGENERATE",
                f"sphere '{l.name}' has no positive radius/diameter (size_mm={size})",
                l.name))
        elif hint in ("box", "cube"):
            # a box needs some positive extent; three is ideal but managers vary wildly,
            # so require at least ONE positive extent (a truly empty box is the crash).
            if not any_positive_size:
                errors.append(GateError(
                    "manager", "ERR_SCHEMA_MGR_DEGENERATE",
                    f"box '{l.name}' has no positive extent (size_mm={size})", l.name))
        else:
            # unknown/free-form shape: only flag if it has NO positive size at all AND
            # declared some size (an empty size_mm is allowed — the worker infers).
            if size and not any_positive_size:
                errors.append(GateError(
                    "manager", "ERR_SCHEMA_MGR_DEGENERATE",
                    f"link '{l.name}' has no positive size dimension (size_mm={size})",
                    l.name))

    # A renderable URDF tree permits each link exactly one parent pose. Multiple mates may
    # describe contact, but the compiled KinematicModel placement forest cannot parent one
    # child twice; yourdfpy otherwise fails only after every STL was generated.
    parents={}
    for p in model.poses:
        if not p.parent:continue
        if p.child in parents:
            errors.append(GateError("manager","ERR_SCHEMA_MGR_MULTIPARENT",
              f"link '{p.child}' is placed by multiple poses '{parents[p.child]}' and '{p.name}' — keep exactly one placement parent",
              p.child))
        else:parents[p.child]=p.name

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
    frame_names={s.id:{f.name for f in s.frames or []} for s in plan.subassemblies}

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
        rp=getattr(seam,"rear_parent_frame","");rc=getattr(seam,"rear_child_frame","")
        if bool(rp)!=bool(rc) or (rp and (seam.kind!='weld' or
                rp not in frame_names.get(seam.parent_sub,set()) or
                rc not in frame_names.get(seam.child_sub,set()))):
            errors.append(GateError(
                "boss","ERR_SCHEMA_BOSS_THROUGH_PAIR",
                f"seam '{seam.id}' needs a complete valid rear_parent_frame/rear_child_frame pair",
                seam.id))

    drivers = [s.id for s in plan.seams if s.driver]
    if len(drivers) > 1:
        errors.append(GateError(
            "boss", "ERR_SCHEMA_BOSS_DRIVERS",
            f"more than one seam has driver=true {drivers} (the machine has ONE input)",
            ",".join(drivers)))
    return errors
