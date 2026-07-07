"""Numeric design score for score-gated, keep-best iteration (maker2-mujoco-contact).

The boss loop was pure-boolean (pass/fail) with no notion of "did this iteration get
BETTER" — research (arXiv 2310.01798) says intrinsic self-correction without an
external verifier degrades, so iteration must be gated by an objective signal that
must MEASURABLY improve. This module is that signal: a weighted sum of [0,1] sub-scores
computed from the physics metrics + the geometry pre-check + the appearance judge.

score(phys, precheck_report, judge_verdict, settings) -> (float in [0,1], breakdown dict)

Weights are the plan's calibration DRAFT (0.45/0.25/0.15/0.15); expose via
settings.score_weights so a labeled-run sweep can retune without a code change.
"""
from __future__ import annotations

# Draft weights (transmission / stability / overlap / judge). Calibrate on labeled runs.
_DEFAULT_WEIGHTS = {
    "transmission": 0.45,
    "stability": 0.25,
    "overlap": 0.15,
    "judge": 0.15,
}


def _weights(settings):
    w = dict(_DEFAULT_WEIGHTS)
    override = getattr(settings, "score_weights", None) if settings else None
    if isinstance(override, dict):
        for k in w:
            if k in override:
                try:
                    w[k] = float(override[k])
                except (TypeError, ValueError):
                    pass
    return w


def _transmission_score(m: dict) -> float:
    """How well motion propagated by contact. moved/watched, discounted if the declared
    output was never reached, zeroed if the sim exploded/jammed."""
    if not m:
        return 0.0
    if m.get("exploded"):
        return 0.0
    watched = int(m.get("watched_count") or 0)
    moved = int(m.get("moved_count") or 0)
    if watched <= 0:
        # No downstream parts to move (a single-DOF toy): credit input motion alone.
        return 1.0 if (m.get("input_travel") or 0) > 0.05 else 0.0
    frac = min(1.0, moved / watched)
    reached = m.get("output_reached")
    if reached is False:
        frac *= 0.5              # moved something but the output is dead
    return frac


def _stability_score(m: dict) -> float:
    """Did the assembly stay put (not explode, not topple, not sink)?"""
    if not m:
        return 0.5               # unknown -> neutral
    if m.get("exploded"):
        return 0.0
    tilt = float(m.get("max_tilt_deg") or 0.0)
    # 0 deg -> 1.0, 45 deg -> 0.0 (linear); a toppled machine scores low.
    tilt_term = max(0.0, 1.0 - tilt / 45.0)
    return tilt_term


def _overlap_penalty_score(precheck_report) -> float:
    """1 - worst part-interpenetration fraction from the geometry pre-check. No
    overlaps -> 1.0; a 40%-buried part -> 0.6."""
    if precheck_report is None:
        return 1.0
    worst = 0.0
    for v in getattr(precheck_report, "violations", []) or []:
        if getattr(v, "kind", "") in ("part_overlap", "aabb_overlap"):
            worst = max(worst, float(getattr(v, "value", 0.0) or 0.0))
    return max(0.0, 1.0 - worst)


def _judge_score(judge_verdict) -> float:
    """1.0 if the appearance judge passed, 0.3 if it failed, 0.6 if unknown."""
    if judge_verdict is None:
        return 0.6
    passed = None
    if isinstance(judge_verdict, dict):
        passed = judge_verdict.get("passed")
    else:
        passed = getattr(judge_verdict, "passed", None)
    if passed is True:
        return 1.0
    if passed is False:
        return 0.3
    return 0.6


def score(phys: dict, precheck_report=None, judge_verdict=None,
          settings=None) -> tuple[float, dict]:
    """Weighted design score in [0,1] + a per-term breakdown. `phys` is run_physics'
    result dict (its `metrics` drive transmission/stability); a degraded contact
    decomposition (metrics["contact_degraded"]) caps the transmission term since the
    teeth were only hulls."""
    m = (phys or {}).get("metrics", {}) if isinstance(phys, dict) else {}
    w = _weights(settings)

    terms = {
        "transmission": _transmission_score(m),
        "stability": _stability_score(m),
        "overlap": _overlap_penalty_score(precheck_report),
        "judge": _judge_score(judge_verdict),
    }
    # Contact was degraded to convex hulls (no real teeth) -> transmission is not
    # trustworthy; cap it so a hull-jam doesn't read as a win.
    if m.get("contact_degraded"):
        terms["transmission"] = min(terms["transmission"], 0.5)

    total = sum(w[k] * terms[k] for k in terms)
    breakdown = {"score": round(total, 4),
                 "terms": {k: round(v, 4) for k, v in terms.items()},
                 "weights": w,
                 "constrained_meshes": m.get("constrained_meshes", 0),
                 "contact_degraded": bool(m.get("contact_degraded"))}
    return total, breakdown
