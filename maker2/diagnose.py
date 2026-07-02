#!/usr/bin/env python3
"""maker2 follow-up routing: one small gateway-routed LLM call that lets a refine
chat skip the slow manager when the user isn't asking to redesign the CAD.

classify_followup(): given a user's refine message + the current model summary + the
last test spec, decide the ENTRY STAGE for the next turn:
    rebuild | retest | reframe | revise_scenario
so a follow-up that only wants a different/re-recorded TEST re-enters the evaluator
directly instead of paying a full manager decompose.

Routes through maker2's gateway (Settings.load(), 8313 default) and tolerates a
gateway that ignores strict response_format (parses the JSON out of prose + accepts
renamed keys). The physics VLM verdict engine (diagnose_physics) lives in
evaluator/diagnose.py — this is the maker2-side pipeline glue.
"""
from __future__ import annotations

import json
import os
import re


def _client(gateway):
    from openai import OpenAI
    base = (gateway or {}).get("base_url") or os.environ.get("AZURE_OPENAI_ENDPOINT", "")
    key = (gateway or {}).get("api_key") or os.environ.get("AZURE_OPENAI_API_KEY", "")
    return OpenAI(base_url=base.rstrip("/"), api_key=key)


def _model(gateway):
    return (gateway or {}).get("model") or os.environ.get("AZURE_VLM_DEPLOYMENT",
                                                          "claude-opus-4.8")


def _parse_json(txt: str) -> dict:
    """Parse the model reply into a dict, tolerating prose/fences around the JSON."""
    t = (txt or "").strip()
    if not t:
        return {}
    if not t.lstrip().startswith("{"):
        m = re.search(r"\{.*\}", t, re.DOTALL)
        if m:
            t = m.group(0)
    try:
        return json.loads(t)
    except Exception:
        return {}


def _pick(d: dict, keys, allowed, default):
    """First value under any of `keys` (gateways rename schema keys) that is in
    `allowed`; else scan ALL values for a valid enum member (robust to any key
    name the gateway invents, e.g. 'stage'/'entry_stage'); else default."""
    for k in keys:
        v = d.get(k)
        if isinstance(v, str) and v.strip().lower() in allowed:
            return v.strip().lower()
    for v in d.values():
        if isinstance(v, str) and v.strip().lower() in allowed:
            return v.strip().lower()
    return default


_FOLLOWUP_SYSTEM = """You route a user's follow-up in an automated CAD pipeline. The pipeline
is: manager decomposes the prompt into a CAD model (SLOW), a worker builds geometry, then
an evaluator TESTS the model in physics (drives it, records a video). A model is already
built. Given the user's follow-up message, decide the cheapest correct ENTRY STAGE:

- "rebuild": the user wants to CHANGE THE CAD itself (add/remove/resize/re-shape a part,
  change materials, fix geometry). Only this restarts the slow manager.
- "retest": the model is fine; the user wants it TESTED (again or for the first time),
  or thinks the prior test was inconclusive. Re-run the evaluator on the SAME model.
- "reframe": the user is complaining about the VIDEO/CAMERA (can't see it, bad angle,
  too far). Just re-record with a better camera on the SAME model + SAME test.
- "revise_scenario": the user wants the TEST done DIFFERENTLY (drive a different joint,
  different speed/motion, check something else) but NOT change the CAD.

When unsure, choose "rebuild" (safe default). Respond ONLY with the JSON schema."""

_FOLLOWUP_SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "followup_entry", "strict": True,
        "schema": {
            "type": "object", "additionalProperties": False,
            "properties": {
                "entry": {"type": "string",
                          "enum": ["rebuild", "retest", "reframe", "revise_scenario"]},
                "reason": {"type": "string"},
            },
            "required": ["entry", "reason"],
        },
    },
}


def classify_followup(message, model_summary, last_spec, gateway=None) -> dict:
    """Decide the entry stage for a follow-up. Returns {entry, reason}. Defaults to
    'rebuild' on any failure (never silently skip a real design change)."""
    if not (message or "").strip():
        return {"entry": "rebuild", "reason": "no message"}
    msg = (
        f"CURRENT MODEL: {model_summary}\n"
        f"LAST TEST SPEC (if any): {json.dumps(last_spec) if last_spec else 'none'}\n\n"
        f"USER FOLLOW-UP: {message}\n\n"
        f"Pick the entry stage.")
    try:
        c = _client(gateway)
        r = c.chat.completions.create(
            model=_model(gateway),
            messages=[{"role": "system", "content": _FOLLOWUP_SYSTEM},
                      {"role": "user", "content": msg}],
            response_format=_FOLLOWUP_SCHEMA, max_completion_tokens=300)
        d = _parse_json(r.choices[0].message.content)
    except Exception as e:
        return {"entry": "rebuild", "reason": f"classify failed ({e})"}
    entry = _pick(d, ("entry", "stage", "action", "route", "decision"),
                  {"rebuild", "retest", "reframe", "revise_scenario"}, "rebuild")
    reason = (d.get("reason") or d.get("explanation") or "")
    return {"entry": entry, "reason": str(reason).strip()[:200]}
