#!/usr/bin/env python3
"""Run the designer-authored functional check against a recorded simulation.

The scenario designer emits `metrics_code`: a small Python program defining
``check(traj, result)`` that says what THIS machine had to achieve — a 12:1 ratio between
two hands, a gripper gap that must close, a ratchet angle that must never decrease. That
cannot be a fixed set of numeric fields, because every mechanism succeeds differently; it
can be code, and this is where that code runs.

Isolated in a subprocess with a timeout, exactly like the CAD authoring sandbox: the check
is LLM-written, so it may loop, raise, or take forever, and none of that may take the run
down with it. A failure here is reported as "the check could not be evaluated", never as a
verdict about the machine.

CLI:  _metrics_runner.py <code.py> <trajectory.json> <sim_result.json> <out.json>
"""
from __future__ import annotations

import json
import math
import re
import sys
import traceback

# How far a measured value may sit from a numeric target before we overrule the designer's
# own "passed". 5% is loose enough for simulation noise and tooth-count rounding, tight
# enough that a missing or extra stage cannot hide inside it.
_TOLERANCE = 0.05

# A target phrased as a BOUND is satisfied by exceeding it. "> 1.0 rad" met by 11.9 rad is a
# pass, not a 1090%-off failure -- reading a bound as an equality target turned two correct
# checks into false failures and buried the one real fault (a 76.9:1 ratio against 12:1) in
# noise the diagnoser then had to reason through.
_LOWER_BOUND_RE = re.compile(
    r"(>=|>|≥|at least|no less than|more than|minimum|min\b|over)", re.I)
_UPPER_BOUND_RE = re.compile(
    r"(<=|<|≤|at most|no more than|less than|under|maximum|max\b|within|below)", re.I)


def _target_number(expected):
    """The number a non-numeric `expected` is really asking for, or None.

    Targets arrive as the designer wrote them: 12.0, "~12:1", "12:1", "approx 12". Ratios
    written "a:b" mean a/b. Anything with no number in it ("closes", "monotonic") returns
    None and that check keeps the designer's verdict.
    """
    if isinstance(expected, bool) or expected is None:
        return None
    if isinstance(expected, (int, float)):
        return float(expected) if math.isfinite(float(expected)) else None
    nums = re.findall(r"-?\d+(?:\.\d+)?", str(expected))
    if not nums:
        return None
    if len(nums) >= 2 and ":" in str(expected):
        a, b = float(nums[0]), float(nums[1])
        return a / b if b else None
    return float(nums[0])


def _bound_kind(expected):
    """"lower" / "upper" if `expected` states a bound rather than a value, else None."""
    if not isinstance(expected, str):
        return None
    # Upper first: "no more than" contains "more than".
    if _UPPER_BOUND_RE.search(expected):
        return "upper"
    if _LOWER_BOUND_RE.search(expected):
        return "lower"
    return None


def main() -> int:
    code_path, traj_path, result_path, out_path = sys.argv[1:5]

    def _write(payload):
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

    try:
        with open(traj_path, encoding="utf-8") as f:
            traj = json.load(f)
    except Exception as e:
        _write({"ok": False, "error": f"no trajectory: {e}", "checks": []})
        return 0
    try:
        with open(result_path, encoding="utf-8") as f:
            result = (json.load(f) or {}).get("metrics", {})
    except Exception:
        result = {}

    try:
        with open(code_path, encoding="utf-8") as f:
            code = f.read()
    except Exception as e:
        _write({"ok": False, "error": f"no metrics code: {e}", "checks": []})
        return 0

    ns: dict = {"math": math}
    try:
        exec(compile(code, "<metrics>", "exec"), ns)
    except Exception:
        _write({"ok": False, "error": "metrics code did not compile",
                "trace": traceback.format_exc()[-1500:], "checks": []})
        return 0

    fn = ns.get("check")
    if not callable(fn):
        _write({"ok": False, "error": "metrics code defines no check()", "checks": []})
        return 0

    try:
        raw = fn(traj, result)
    except Exception:
        _write({"ok": False, "error": "check() raised",
                "trace": traceback.format_exc()[-1500:], "checks": []})
        return 0

    # Normalise whatever it returned. The code is generated, so accept a single dict, a
    # list, or nothing, and coerce each entry to the documented shape rather than trusting
    # it — a malformed check must not crash the caller reading these results.
    if raw is None:
        raw = []
    if isinstance(raw, dict):
        raw = [raw]
    checks = []
    if isinstance(raw, (list, tuple)):
        for item in list(raw)[:20]:
            if not isinstance(item, dict):
                continue
            try:
                passed = bool(item.get("passed"))
            except Exception:
                passed = False
            value = _num_or_str(item.get("value"))
            expected = _num_or_str(item.get("expected"))
            detail = str(item.get("detail") or "")[:300]
            # The designer writes both the measurement AND the band it is judged against,
            # so it can pass itself: one watch reported 9.738 against "~12:1" and called it
            # passed — 19% off, and the hands visibly never changed their angle. Whenever
            # the target is a number we can read, the tolerance is ours, not the
            # designer's. Non-numeric targets ("closes", "never decreases") stay its call.
            target = _target_number(expected)
            bound = _bound_kind(item.get("expected"))
            if target is not None and isinstance(value, float):
                if bound == "lower":
                    # Satisfied by meeting or exceeding it. Only overrule a claimed pass
                    # that does not actually clear the bar.
                    within = value >= target
                elif bound == "upper":
                    within = value <= target
                else:
                    err = abs(value - target) / abs(target) if target else abs(value)
                    within = err <= _TOLERANCE
                if passed and not within:
                    if bound:
                        detail = (detail + " | REJECTED: %.4g does not satisfy %s" % (
                            value, expected))[:300]
                    else:
                        detail = (detail + " | REJECTED: %.4g vs target %.4g is %.1f%% off, "
                                  "over the %.0f%% tolerance" % (
                                      value, target,
                                      abs(value - target) / abs(target) * 100 if target
                                      else 0.0, _TOLERANCE * 100))[:300]
                passed = passed and within
            checks.append({
                "name": str(item.get("name") or "check")[:80],
                "value": value,
                "expected": expected,
                "passed": passed,
                "detail": detail,
            })
    _write({"ok": True, "checks": checks})
    return 0


def _num_or_str(v):
    if v is None:
        return None
    if isinstance(v, bool):
        return v
    try:
        f = float(v)
        return round(f, 6) if math.isfinite(f) else str(v)
    except (TypeError, ValueError):
        return str(v)[:120]


if __name__ == "__main__":
    sys.exit(main())
