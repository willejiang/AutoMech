#!/usr/bin/env python3
"""Aggregate per-test physics verdicts into ONE overall verdict + a per-subsystem
cause map + fault localization.

A multi-subsystem machine runs one driven test per subsystem (see the strategy
selector + maker2/physics). This module rolls those N per-test diagnoses up into:
  - passed:      deterministic AND over every test's PASS
  - cause_map:   {subsystem -> {verdict, cause, reason}} so a boss loop can SEE which
                 subsystem failed and why
  - blamed_subs: the subsystems of the failing tests (what to re-run / re-plan)
  - blamed_kind: the dominant fault kind — "interface" when a subsystem's own train
                 works to its last joint but motion doesn't cross INTO the next
                 (a seam/coupling problem), else the worst failing test's cause
                 ("structure" > "scenario" > "framing").

The verdict and the map are 100% DETERMINISTIC. An LLM is used ONLY (and optionally)
to phrase the human-readable `summary`; if the gateway is unavailable the summary
falls back to a joined per-test string. Gateway-injectable via `gw` (the same
{base_url, api_key, model} dict maker2 threads everywhere)."""
from __future__ import annotations

import json
import os


# cause severity for picking the "worst" failing test.
_CAUSE_RANK = {"structure": 3, "scenario": 2, "framing": 1, "none": 0, None: 0}


def _sub_key(t: dict, idx: int) -> str:
    """The subsystem id a test belongs to; fall back to its name, then its index."""
    return t.get("subsystem") or t.get("name") or f"test_{idx}"


def aggregate_verdicts(task, per_test, *, gw=None) -> dict:
    """Roll per-test verdicts into one overall verdict + cause map + blame.

    per_test: the list of test entries run_physics builds, each with at least
              {verdict, cause, reason, subsystem?, name?, metrics?}.
    Returns {passed, summary, cause_map, blamed_subs, blamed_kind}."""
    per_test = per_test or []
    passed = bool(per_test) and all(t.get("verdict") == "PASS" for t in per_test)

    cause_map: dict[str, dict] = {}
    for i, t in enumerate(per_test):
        cause_map[_sub_key(t, i)] = {
            "verdict": t.get("verdict"),
            "cause": t.get("cause", "none"),
            "reason": t.get("reason", ""),
            "output_reached": (t.get("metrics") or {}).get("output_reached"),
        }

    failing = [t for i, t in enumerate(per_test) if t.get("verdict") != "PASS"]
    blamed_subs = [_sub_key(t, i) for i, t in enumerate(per_test)
                   if t.get("verdict") != "PASS"]

    blamed_kind = None
    if failing:
        # "interface" seam fault: a failing test whose OWN train transmitted (its
        # output was reached / gears moved) yet the machine still fails -> the break is
        # at the coupling BETWEEN subsystems, not inside this one. Otherwise take the
        # worst failing test's cause.
        seam = any((t.get("metrics") or {}).get("output_reached")
                   and (t.get("metrics") or {}).get("moved_count", 0) >= 1
                   for t in failing)
        worst = max(failing, key=lambda t: _CAUSE_RANK.get(t.get("cause"), 0))
        blamed_kind = "interface" if seam and len(per_test) > 1 else worst.get("cause", "structure")

    summary = _summary(task, per_test, passed, blamed_subs, blamed_kind, gw)
    return {"passed": passed, "summary": summary, "cause_map": cause_map,
            "blamed_subs": blamed_subs, "blamed_kind": blamed_kind}


def _deterministic_summary(per_test, passed, blamed_subs, blamed_kind) -> str:
    head = "ALL PASS" if passed else f"FAIL ({blamed_kind}): {', '.join(blamed_subs)}"
    body = " | ".join(t.get("summary", t.get("name", "")) for t in per_test)
    return f"{head} — {body}" if body else head


def _summary(task, per_test, passed, blamed_subs, blamed_kind, gw) -> str:
    """Human summary. Try a single small LLM call for prose; fall back deterministically.
    The LLM only PHRASES — it never changes the verdict/map/blame computed above."""
    fallback = _deterministic_summary(per_test, passed, blamed_subs, blamed_kind)
    gw = gw or {}
    base_url = gw.get("base_url")
    api_key = gw.get("api_key")
    if not (base_url or os.environ.get("AZURE_OPENAI_ENDPOINT")):
        return fallback
    try:
        from openai import OpenAI
        c = OpenAI(base_url=(base_url or os.environ["AZURE_OPENAI_ENDPOINT"]).rstrip("/"),
                   api_key=api_key or os.environ.get("AZURE_OPENAI_API_KEY", ""))
        tests_brief = [{"subsystem": t.get("subsystem") or t.get("name"),
                        "verdict": t.get("verdict"), "cause": t.get("cause"),
                        "reason": (t.get("reason") or "")[:200]} for t in per_test]
        msg = (f"TASK: {task}\n"
               f"OVERALL: {'PASS' if passed else 'FAIL'}\n"
               f"PER-TEST: {json.dumps(tests_brief)}\n"
               f"BLAMED SUBSYSTEMS: {blamed_subs} (kind: {blamed_kind})\n\n"
               f"Write ONE plain-English sentence summarizing whether the machine works "
               f"and, if not, which subsystem failed and why. No preamble.")
        r = c.chat.completions.create(
            model=gw.get("model") or os.environ.get("AZURE_VLM_DEPLOYMENT", "claude-opus-4.8"),
            messages=[{"role": "user", "content": msg}],
            max_completion_tokens=160)
        txt = (r.choices[0].message.content or "").strip()
        return txt or fallback
    except Exception:
        return fallback


if __name__ == "__main__":
    # Tiny self-check with no gateway (deterministic path only).
    demo = [
        {"subsystem": "engine", "verdict": "PASS", "cause": "none",
         "summary": "engine: PASS", "metrics": {"output_reached": True, "moved_count": 3}},
        {"subsystem": "steering", "verdict": "FAIL", "cause": "structure",
         "reason": "rack does not mesh with pinion", "summary": "steering: FAIL",
         "metrics": {"output_reached": False, "moved_count": 1}},
    ]
    out = aggregate_verdicts("a car", demo, gw={})
    print(json.dumps(out, indent=2))
