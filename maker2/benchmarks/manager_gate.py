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


def frame_drift_errors(model, frame_contract, is_root: bool = True) -> list[GateError]:
    """C6 — self-consistency on frames_realized: check each realized interface frame's
    WORLD position (recomputed from the owning link's pose) against the boss contract.

    The check DIFFERS by the sub's role, because the assembler places subs differently:

    * ROOT sub (is_root=True): the assembler pins its root at the GLOBAL origin, so the
      realized frames must land on the contract's ABSOLUTE xyz_m within _POS_TOL_M. An
      absolute drift -> ERR_FRAME_DRIFT.

    * WELDED CHILD (is_root=False): the assembler RIGIDLY TRANSFORMS the whole sub so its
      weld frame mates to the parent — the sub's own origin is NOT the global origin, so
      the absolute contract coords are the sub's TARGET pose, not where the manager must
      author. A child authored around its own local origin is CORRECT even though every
      frame is offset from the global coord by the (as-yet-unknown) weld transform. What
      MUST hold is INTERNAL CONSISTENCY: the vector between any two realized frames must
      equal the vector between those same two frames in the contract (a rigid transform
      preserves relative offsets). A relative mismatch -> ERR_FRAME_DRIFT; a uniform
      absolute offset (all frames shifted by the same vector) is fine and NOT flagged.

    Reuses assembler._root_to_link / _mat and precheck._POS_TOL_M. Frames the manager did
    not realize at all are NOT flagged here (ERR_FRAME_UNREALIZED owns that)."""
    frames = list(getattr(frame_contract, "frames", []) or [])
    if not frames:
        return []
    realized = {e.get("frame"): e for e in (getattr(model, "frames_realized", []) or [])}
    T = _root_to_link(model)

    def _world_of(fr):
        """Realized WORLD position of a contract frame, or None if not realized/reachable."""
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

    out: list[GateError] = []
    if is_root:
        for fr in frames:
            world = _world_of(fr)
            if world is None:
                continue                      # unrealized/unreachable -> other gate's job
            drift = float(np.linalg.norm(world - _want_of(fr)))
            if drift > _POS_TOL_M:
                out.append(GateError(
                    "manager", "ERR_FRAME_DRIFT",
                    f"interface frame '{fr.name}' is realized on link "
                    f"'{realized[fr.name].get('link')}' but its WORLD position is "
                    f"{drift*1000:.1f} mm from the boss contract's required global position "
                    f"(tolerance {_POS_TOL_M*1000:.1f} mm) — this is the ROOT subassembly, "
                    "pinned at the global origin, so its frames must sit at the contract "
                    "coords. Move the link (or the frame's local offset) onto the contract "
                    "position.",
                    fr.name))
        return out

    # Welded child: compare RELATIVE offsets against a common reference frame. Pick the
    # first realized frame as the reference; every other realized frame's offset FROM it
    # must match the contract's offset within tolerance (a rigid weld preserves this).
    realized_frames = [fr for fr in frames if _world_of(fr) is not None]
    if len(realized_frames) < 2:
        return out                            # 0-1 frames: nothing relative to check
    ref = realized_frames[0]
    ref_world, ref_want = _world_of(ref), _want_of(ref)
    for fr in realized_frames[1:]:
        rel_world = _world_of(fr) - ref_world          # realized offset from the reference
        rel_want = _want_of(fr) - ref_want             # contract offset from the reference
        drift = float(np.linalg.norm(rel_world - rel_want))
        if drift > _POS_TOL_M:
            out.append(GateError(
                "manager", "ERR_FRAME_DRIFT",
                f"interface frame '{fr.name}' is {drift*1000:.1f} mm out of position "
                f"RELATIVE to frame '{ref.name}' (tolerance {_POS_TOL_M*1000:.1f} mm): the "
                f"contract places them {np.round(rel_want*1000, 1)} mm apart but this sub "
                f"realizes them {np.round(rel_world*1000, 1)} mm apart. This welded "
                "subassembly may sit anywhere globally, but its frames' RELATIVE layout "
                "must match the contract. Fix the offending link/frame offset.",
                fr.name))
    return out
