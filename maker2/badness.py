"""Pre-physics BADNESS score: the universal per-loop objective (Part C.bis, C11).

Every retry loop in the pipeline except the boss physics score-loop was a BLIND retry —
a fresh generation plus an error string, with no notion of whether attempt N+1 got
*closer* to buildable. This module gives every loop a single numeric objective it can
MONOTONICALLY reduce, computed entirely from the deterministic gate signals that already
exist, with **no LLM and no physics** — so it works even when the machine never assembles
(which is the situation the whole plan is trying to escape).

``badness(model, gate_errors, context=None) -> float`` : lower = closer to buildable. It
is a weighted sum of:
  * blocking gate errors        — count x per-code severity (schema/connect/frame worst).
  * AABB overlap volume         — summed shared volume of flagged non-adjacent pairs.
  * frame-coordinate drift      — sum of world-pos deltas of realized frames vs the boss
                                  contract (only when ``context`` carries the frame contract).
  * unrealized frames           — contract frames no link realized.
  * non-manifold parts          — from worker results, when ``context`` carries them.

The signature is FROZEN across tracks (Part E): Track 1 may READ a run score for the
memory-append hook but never computes badness. ``context`` is an optional dict so a caller
that only has a model + a list of GateError still gets a meaningful number:
    context = {"fc": frame_contract, "part_results": [WorkerResult, ...]}

Pure function — unit-testable with a hand-built KinematicModel. Reuses
manager_gate._world_aabb / _overlap_frac / _local_aabb_m, precheck._POS_TOL_M,
assembler._root_to_link / _mat. See .claude/plans/precious-humming-wand.md Part C.bis.
"""

from __future__ import annotations

import numpy as np

# Per-code weights: a blocking schema/connectivity/frame fault is a hard "won't build",
# so it dominates a soft geometry warning. Unlisted codes get _DEFAULT_ERR_WEIGHT. These
# are ORDINAL (they set the gradient the loops descend), not physically calibrated.
_ERR_WEIGHTS: dict[str, float] = {
    # schema / structural — the model is not even well-formed.
    "ERR_SCHEMA_MGR_DOF": 10.0,
    "ERR_SCHEMA_MGR_DRIVER_FIXED": 8.0,
    "ERR_SCHEMA_MGR_NOAXIS": 8.0,
    "ERR_SCHEMA_MGR_DEGENERATE": 12.0,     # the MjModel.from_xml crash class
    "ERR_SCHEMA_MGR_MESHREF": 6.0,
    "ERR_SCHEMA_MGR_DRIVERS": 6.0,
    "ERR_CONNECT": 10.0,                   # an unplaced part floats at the origin
    # frames — the assembler cannot weld the sub without these.
    "ERR_FRAME_UNREALIZED": 9.0,
    # compile — the ultimate gate: the sim itself refuses to load it.
    "ERR_COMPILE": 15.0,
    # geometry — softer; the real-mesh subcheck is the authority.
    "ERR_OVL": 3.0,
    # worker — a bad part.
    "ERR_MANIFOLD": 7.0,
    "ERR_DIM": 1.0,                        # size_mm is approximate; a weak signal
}
_DEFAULT_ERR_WEIGHT = 4.0

# Contribution weights for the continuous (non-error-count) signals. Parts are authored
# in mm but transformed to METERS here, so a gross overlap is ~1e-7..1e-6 m^3 and a frame
# drift of a few mm is ~1e-3 m. The weights below lift those tiny magnitudes so one gross
# overlap or a cm of drift lands near a single soft-error weight (~a few units) — i.e. the
# gradient is on the same scale the loops descend, not a rounding-to-zero afterthought.
_W_OVERLAP_VOL = 5.0e6        # 2e-7 m^3 (a 5 mm part fully overlapping) -> ~1.0
_W_UNREALIZED = 9.0           # per contract frame with no realizing link (mirrors the code)
_W_NONMANIFOLD = 7.0          # per non-manifold part (mirrors ERR_MANIFOLD)


def _overlap_volume(model) -> float:
    """Total shared-AABB volume (m^3) over non-adjacent, non-mesh part pairs — the same
    pairs manager_gate flags, but summed as a continuous magnitude so a loop can tell
    "less overlap" from "more overlap" even when the pass/fail verdict is unchanged."""
    from .assembler import _root_to_link
    from .benchmarks.manager_gate import _world_aabb
    from .precheck import _aabb_vol

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
    total = 0.0
    for i in range(len(names)):
        for k in range(i + 1, len(names)):
            a, b = names[i], names[k]
            if frozenset((a, b)) in allowed:
                continue
            (lo_a, hi_a), (lo_b, hi_b) = boxes[a], boxes[b]
            lo_i = np.maximum(lo_a, lo_b)
            hi_i = np.minimum(hi_a, hi_b)
            if np.any(lo_i >= hi_i):
                continue
            total += _aabb_vol((lo_i, hi_i))
    return float(total)


def _unrealized_frames(model, fc) -> int:
    """Count of boss-contract frames the manager never realized (no frames_realized entry,
    or an entry on a link unreachable from the root). Frame POSITION drift is NOT scored
    anymore: the boss authors no placement coordinates (a frame's xyz_m is a rough hint), so
    there is no authoritative position to measure drift against — only whether every required
    frame is realized at all still matters (an unrealized frame breaks the assembler)."""
    frames = list(getattr(fc, "frames", []) or [])
    if not frames:
        return 0
    from .assembler import _root_to_link

    realized = {e.get("frame"): e for e in (getattr(model, "frames_realized", []) or [])}
    T = _root_to_link(model)
    unrealized = 0
    for fr in frames:
        entry = realized.get(fr.name)
        if not entry or T.get(entry.get("link")) is None:
            unrealized += 1
    return unrealized


def _nonmanifold_count(part_results) -> int:
    """Number of worker results that failed to render a usable part (proxy for
    non-manifold / unloadable). Tolerant of the WorkerResult shape."""
    n = 0
    for r in part_results or []:
        if not getattr(r, "success", True):
            n += 1
    return n


def badness_breakdown(model, gate_errors, context=None) -> dict:
    """The per-term contributions behind ``badness`` — used for diff-carrying retry
    feedback (C13) and logging, so a loop can say exactly which term went up or down.
    Returns {"total": float, "terms": {name: value}, "errors": {code: count}}."""
    context = context or {}
    fc = context.get("fc")
    part_results = context.get("part_results")

    err_by_code: dict[str, int] = {}
    err_term = 0.0
    for e in gate_errors or []:
        code = getattr(e, "code", "") or ""
        err_by_code[code] = err_by_code.get(code, 0) + 1
        err_term += _ERR_WEIGHTS.get(code, _DEFAULT_ERR_WEIGHT)

    terms: dict[str, float] = {"gate_errors": round(err_term, 4)}

    try:
        ov = _overlap_volume(model)
        terms["overlap_vol"] = round(_W_OVERLAP_VOL * ov, 4)
    except Exception:
        terms["overlap_vol"] = 0.0

    if fc is not None:
        try:
            terms["unrealized_frames"] = round(_W_UNREALIZED * _unrealized_frames(model, fc), 4)
        except Exception:
            terms["unrealized_frames"] = 0.0

    if part_results is not None:
        terms["nonmanifold"] = round(_W_NONMANIFOLD * _nonmanifold_count(part_results), 4)

    total = round(sum(terms.values()), 4)
    return {"total": total, "terms": terms, "errors": err_by_code}


def badness(model, gate_errors, context=None) -> float:
    """Pre-physics badness of a model + its gate errors: lower = closer to buildable.
    No LLM, no physics — computable before the machine ever assembles. See module
    docstring for the term list and ``badness_breakdown`` for the per-term split.

    FROZEN signature (Part E): ``context`` is an optional dict carrying the frame
    contract (``fc``) and/or worker ``part_results`` so the frame-drift and manifold
    terms can be included; without it, badness is the error + overlap terms only."""
    return badness_breakdown(model, gate_errors, context)["total"]


def format_delta(prev: dict, cur: dict) -> str:
    """A one-line 'gradient' note comparing two badness_breakdown dicts for C13
    diff-carrying feedback: overall direction + the per-term deltas that moved most.
    ``prev``/``cur`` are badness_breakdown() outputs (or empty on the first attempt)."""
    if not prev:
        return f"badness={cur.get('total', 0):.2f} (first attempt; no prior to compare)"
    p_total = prev.get("total", 0.0)
    c_total = cur.get("total", 0.0)
    direction = ("WORSE" if c_total > p_total + 1e-6
                 else "BETTER" if c_total < p_total - 1e-6 else "SAME")
    p_terms = prev.get("terms", {})
    c_terms = cur.get("terms", {})
    deltas = []
    for k in sorted(set(p_terms) | set(c_terms)):
        d = c_terms.get(k, 0.0) - p_terms.get(k, 0.0)
        if abs(d) > 1e-6:
            deltas.append(f"{k} {p_terms.get(k, 0.0):.1f}->{c_terms.get(k, 0.0):.1f}")
    tail = ("; ".join(deltas)) if deltas else "no term changed"
    return f"badness {p_total:.2f}->{c_total:.2f} ({direction}): {tail}"
