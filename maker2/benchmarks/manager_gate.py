"""Phase 1 — manager_gate: per-subassembly overlap + pose-graph connectivity.

Runs on the manager's model BEFORE the worker renders any STL, so a bad LAYOUT is caught
by cheap math instead of a full render + the slow LLM debugger. Per the support decision,
per-sub checks are ONLY connectivity + overlap (NO grounding/tip-over — subs are not
free-standing; they hang off shafts whose bearings live in another sub).

- CONNECTIVITY: every link must be reachable in the pose forest from a real root by
  following parent->child pose edges. An unreachable link = the manager forgot to place
  it -> ERR_CONNECT.
- OVERLAP: build each link's world AABB from shape_hint+size_mm+pose (reusing
  assembler._root_to_link for placement, precheck._transform_aabb for the box), and flag
  any non-mesh, non-pose-adjacent pair whose shared volume exceeds _OVERLAP_FRAC of the
  smaller part -> ERR_OVL. This is the cheap PRE-render check; the post-render subcheck
  (real meshes) remains the backstop.

Reuses: assembler._root_to_link, assembler._mat, precheck._transform_aabb,
precheck._aabb_vol, precheck._OVERLAP_FRAC. No LLM.
"""

from __future__ import annotations

import numpy as np

from . import GateError
from ..assembler import _mat, _root_to_link
from ..precheck import _POS_TOL_M, _aabb_vol, _transform_aabb

# Pre-render WARN threshold (C1): the DECLARED-box AABB over-approximates non-boxy parts
# (a gear/cylinder envelope is a solid disc, but its real geometry may be spokes; a thin rib
# rotated radially yields a huge diagonal AABB), so this pre-render overlap is a WARNING, not
# a block — it CANNOT be made zero-false-positive (a valid 8-rib cage produces 44 near-total
# box overlaps; even an OBB/SAT check still flags 26/28 rib pairs). The AUTHORITATIVE overlap
# gate is the post-render real-mesh subcheck (precheck._OVERLAP_FRAC = 0.30 on REAL meshes,
# which DOES fail the sub up). We raise the warn bar to 0.60 so only egregious declared
# overlaps are surfaced and the log isn't flooded; the continuous overlap magnitude still
# feeds badness() regardless of this cutoff.
_PRE_OVERLAP_FRAC = 0.60


def _local_aabb_m(link) -> tuple | None:
    """A link's LOCAL [min,max] AABB in METERS from shape_hint+size_mm, centered on its
    own origin. Returns None when the shape/size can't yield a box (free text with no
    usable dims) — that link is skipped for overlap (the post-render subcheck covers it).

    size_mm is in MILLIMETERS; we convert to meters to match the pose transforms."""
    size = getattr(link, "size_mm", {}) or {}
    hint = (getattr(link, "shape_hint", "") or "").strip().lower()

    def mm(v):
        try:
            return float(v) / 1000.0
        except (TypeError, ValueError):
            return None

    if hint in ("box", "cube"):
        x, y, z = mm(size.get("x")), mm(size.get("y")), mm(size.get("z"))
        if None in (x, y, z):
            return None
        hx, hy, hz = x / 2, y / 2, z / 2
    elif hint in ("cylinder", "sphere") or "radius" in size:
        r = mm(size.get("radius") or ((size.get("outer_dia") or 0) / 2) or
               ((size.get("pitch_dia") or 0) / 2))
        h = mm(size.get("height")) if hint != "sphere" else (r if r else None)
        if not r:
            return None
        if h is None:
            h = 2 * r
        hx = hy = r
        hz = h / 2
    else:
        # Unknown shape: fall back to the max declared dim as a cube half-extent, if any.
        dims = [mm(v) for v in size.values() if mm(v)]
        if not dims:
            return None
        hx = hy = hz = max(dims) / 2
    return (np.array([-hx, -hy, -hz]), np.array([hx, hy, hz]))


def _world_aabb(link, T) -> tuple | None:
    local = _local_aabb_m(link)
    if local is None:
        return None
    corners = _transform_aabb(local, T)
    if corners is None:
        return None
    return corners.min(axis=0), corners.max(axis=0)


def _overlap_frac(ba, bb) -> float:
    """Shared-AABB volume as a fraction of the SMALLER box's volume (monotonic, robust —
    same measure as precheck._intersection_frac but on declared boxes)."""
    (lo_a, hi_a), (lo_b, hi_b) = ba, bb
    lo_i = np.maximum(lo_a, lo_b)
    hi_i = np.minimum(hi_a, hi_b)
    if np.any(lo_i >= hi_i):
        return 0.0
    vi = _aabb_vol((lo_i, hi_i))
    vs = min(_aabb_vol(ba), _aabb_vol(bb))
    return (vi / vs) if vs > 0 else 0.0


def _connectivity_errors(model) -> list[GateError]:
    """Detect links the manager forgot to place. A pure-contact model is a FOREST, so
    multiple roots are legitimate — we cannot treat "has no parent pose" as floating.
    But a link that participates in NO pose at all (neither parent nor child) and is not
    the declared root_link is an unplaced orphan: it defaults to the origin and would
    overlap/float. That is the reliable connectivity signal."""
    link_names = [l.name for l in model.links]
    touched: set[str] = set()
    for p in model.poses:
        if p.parent:
            touched.add(p.parent)
        if p.child:
            touched.add(p.child)
    out: list[GateError] = []
    for n in link_names:
        if n in touched or n == model.root_link:
            continue
        # A lone link in a single-link model is fine (it IS the whole sub); only flag
        # when there ARE poses (i.e. the sub has a placement structure this link is
        # missing from).
        if model.poses:
            out.append(GateError(
                "manager", "ERR_CONNECT",
                f"link '{n}' is placed by no pose (not connected to the subassembly) — "
                "it would default to the origin and float; add a pose that positions it",
                n))
    return out


def manager_gate(model, sub_frames, frame_contract) -> list[GateError]:
    """Deterministic per-sub geometry checks (connectivity + gross overlap). Returns []
    on pass. Skips mesh_pairs (intended tooth contact) and pose-adjacent pairs
    (shaft-in-bore). Only GROSS overlap (>= _OVERLAP_FRAC) is flagged."""
    errors = _connectivity_errors(model)

    # Pairs that are ALLOWED to overlap: gear meshes + directly pose-linked parts.
    allowed: set = set()
    for a, b in getattr(model, "mesh_pairs", []) or []:
        allowed.add(frozenset((a, b)))
    for p in model.poses:
        if p.parent and p.child:
            allowed.add(frozenset((p.parent, p.child)))

    T = _root_to_link(model)
    boxes: dict[str, tuple] = {}
    for l in model.links:
        t = T.get(l.name)
        if t is None:
            continue
        wb = _world_aabb(l, t)
        if wb is not None:
            boxes[l.name] = wb

    names = list(boxes)
    worst_by_pair: list = []
    for i in range(len(names)):
        for k in range(i + 1, len(names)):
            a, b = names[i], names[k]
            if frozenset((a, b)) in allowed:
                continue
            frac = _overlap_frac(boxes[a], boxes[b])
            if frac >= _PRE_OVERLAP_FRAC:
                worst_by_pair.append((frac, a, b))
    for frac, a, b in sorted(worst_by_pair, reverse=True):
        errors.append(GateError(
            "manager", "ERR_OVL",
            f"parts '{a}' and '{b}' grossly overlap ({frac:.0%} of the smaller part is "
            "inside the other by declared size) — separate them or resize",
            f"{a}~{b}"))
    return errors


def frame_drift_errors(model, frame_contract, is_root: bool = True,
                       realized_frames=None) -> list[GateError]:
    """COLLAPSE check (directional): flag when two contract frames the boss placed APART are
    realized at the SAME point. That is always a bug — a frame marks a distinct interface
    (a bearing hole, a mount seat), and two different holes cannot sit at one spot. When it
    happens, downstream seams weld onto a collapsed frame and the mated subs stack (e.g. three
    shafts all land at the plate origin, floating above their real bearings).

    This is the NARROW, one-directional restoration of the old drift gate. It does NOT check
    absolute positions and does NOT flag the reverse case (contract frames COINCIDENT but
    realized apart) — that reverse case was the real false positive (the boss leaves an input
    gear and output pinion at the same lazy z hint while the manager correctly stacks them, and
    the manager is right). We only flag contract-apart -> realized-coincident.

    ``realized_frames`` (list of {frame, link, local_xyz_m, ...}) overrides model.frames_realized;
    callers pass the fallback-resolved frames the assembler welds with, so a fallback that
    collapses every mount frame onto the root at origin is caught here."""
    frames = list(getattr(frame_contract, "frames", []) or [])
    if len(frames) < 2:
        return []
    if realized_frames is not None:
        realized = {e.get("frame"): e for e in realized_frames}
    else:
        realized = {e.get("frame"): e for e in (getattr(model, "frames_realized", []) or [])}
    T = _root_to_link(model)

    def _world_of(fr):
        entry = realized.get(fr.name)
        if not entry:
            return None
        Tlink = T.get(entry.get("link"))
        if Tlink is None:
            return None
        local = _mat(entry.get("local_xyz_m", (0, 0, 0)),
                     entry.get("local_rpy_rad", (0, 0, 0)))
        return (Tlink @ local)[:3, 3]

    def _want_of(fr):
        return np.array([float(v) for v in (getattr(fr, "xyz_m", None) or (0, 0, 0))])

    realized_frames_l = [fr for fr in frames if _world_of(fr) is not None]
    out: list[GateError] = []
    # For every PAIR the contract places apart (> tolerance), the realization must ALSO place
    # them apart. If the realized pair is coincident, the frames collapsed -> block.
    for i in range(len(realized_frames_l)):
        for j in range(i + 1, len(realized_frames_l)):
            fa, fb = realized_frames_l[i], realized_frames_l[j]
            want_sep = float(np.linalg.norm(_want_of(fa) - _want_of(fb)))
            if want_sep <= _POS_TOL_M:
                continue                      # contract itself coincides them -> nothing to check
            got_sep = float(np.linalg.norm(_world_of(fa) - _world_of(fb)))
            if got_sep <= _POS_TOL_M:
                out.append(GateError(
                    "manager", "ERR_FRAME_DRIFT",
                    f"interface frames '{fa.name}' and '{fb.name}' are realized at the SAME "
                    f"point but the contract places them {want_sep*1000:.1f} mm apart — they "
                    f"collapsed (likely both realized on the root/plate at its origin instead "
                    f"of on their own parts). Realize each frame on the actual part at that "
                    f"interface (its bearing/seat/hole) so they are distinct; otherwise the "
                    f"subs that mate to them stack on top of each other.",
                    fb.name))
                break                         # one collapse report per frame is enough
    return out


def _part_world_in_sub(sub_result, part_name: str):
    """World position (in the sub's own root frame) of a part's link origin, via
    _root_to_link. None if the part isn't a reachable link."""
    model = getattr(sub_result, "model", None)
    if model is None:
        return None
    T = _root_to_link(model)
    Tlink = T.get(part_name)
    return Tlink[:3, 3] if Tlink is not None else None


def _realized_world_in_sub(sub_result, frame_name: str):
    """World position (in the sub's own root frame) of a realized interface frame, from the
    sub's persisted frames (sub_frames.json / model.frames_realized). None if unresolved."""
    model = getattr(sub_result, "model", None)
    entries = getattr(sub_result, "sub_frames", None) or []
    if model is None:
        return None
    entry = next((e for e in entries if e.get("frame") == frame_name), None)
    if not entry:
        return None
    T = _root_to_link(model)
    Tlink = T.get(entry.get("link"))
    if Tlink is None:
        return None
    local = _mat(entry.get("local_xyz_m", (0, 0, 0)), entry.get("local_rpy_rad", (0, 0, 0)))
    return (Tlink @ local)[:3, 3]


def seam_frame_agreement_errors(plan, sub_results: dict) -> list[GateError]:
    """GEOMETRY-BACKED SEAT check: a `mount` frame binds to a real part via `mounts_part`
    (the bearing/bore/seat that physically sits at that hole). The manager must realize the
    frame ON that part's location — not collapse it to the sub's root origin. When it does
    collapse (the reducer bug: three seats all realized on `lower_body` at [0,0,0] while their
    bearings sit at Y=0/80/160), the shaft welded to that seat buries itself in the housing.

    For each seam whose parent/child frame names a `mounts_part`, compare the frame's REALIZED
    world position (in its sub) to where `mounts_part` actually sits (in its sub). A gap
    > _POS_TOL_M means the manager did not realize the frame on its bound part ->
    ERR_SEAM_DISAGREE (route to a targeted rebuild of that sub). This is the deterministic
    checksum the two isolated managers lack."""
    out: list[GateError] = []
    frame_of = {(s.id, fr.name): fr for s in plan.subassemblies for fr in (s.frames or [])}

    def _check(sub_id, frame_name):
        fr = frame_of.get((sub_id, frame_name))
        res = sub_results.get(sub_id)
        if fr is None or res is None:
            return None
        mp = (getattr(fr, "mounts_part", "") or "").strip()
        if not mp:
            return None                       # no bound part -> nothing to verify here
        fw = _realized_world_in_sub(res, frame_name)
        pw = _part_world_in_sub(res, mp)
        if fw is None or pw is None:
            return None                       # unrealized / missing part -> other gates own it
        gap = float(np.linalg.norm(fw - pw))
        return (gap, mp) if gap > _POS_TOL_M else None

    for seam in plan.seams:
        if seam.kind not in ("weld", "power"):
            continue
        for sub_id, frame_name in ((seam.parent_sub, seam.parent_frame),
                                   (seam.child_sub, seam.child_frame)):
            hit = _check(sub_id, frame_name)
            if hit is None:
                continue
            gap, mp = hit
            out.append(GateError(
                "manager", "ERR_SEAM_DISAGREE",
                f"seam '{seam.id}': subassembly '{sub_id}' realized interface frame "
                f"'{frame_name}' {gap*1000:.1f} mm away from its bound part '{mp}' — the frame "
                f"must sit ON '{mp}' (its real bearing/bore/seat), but it was realized elsewhere "
                f"(likely collapsed to the sub's root/origin). Realize '{frame_name}' on '{mp}' "
                f"so the sub welded here does not bury itself in this body.",
                frame_name))
    return out
