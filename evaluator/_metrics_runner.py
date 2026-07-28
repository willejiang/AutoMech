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
import sys
import traceback


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
            checks.append({
                "name": str(item.get("name") or "check")[:80],
                "value": _num_or_str(item.get("value")),
                "expected": _num_or_str(item.get("expected")),
                "passed": passed,
                "detail": str(item.get("detail") or "")[:300],
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
