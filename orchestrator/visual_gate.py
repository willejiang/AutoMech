#!/usr/bin/env python3
"""
visual_gate.py — the maker subloop's INNER gate. Judges the 6 rendered views on
geometry sanity / prompt-match BEFORE any physics sim runs (no GPU, cheap).

Ports cadam's own reviewer prompt verbatim in spirit from
worker/src/server/evaluateModel.ts (EVAL_SYSTEM_PROMPT). cadam streams prose then
a `===VERDICT===` JSON line for its browser panel; here we don't need the stream,
so we ask for a strict structured verdict instead (same call style as
evaluator/analyze.py). Same question, same 6-view contract.

Env: AZURE_OPENAI_ENDPOINT + AZURE_OPENAI_API_KEY + AZURE_VLM_DEPLOYMENT.
"""
import base64
import json
import os
from pathlib import Path

# Lifted from worker/src/server/evaluateModel.ts EVAL_SYSTEM_PROMPT (the prose-
# review instructions), minus the streaming VERDICT_MARKER mechanics — the schema
# below replaces that.
GATE_SYSTEM = """You are a strict 3D CAD model reviewer. The user gives you their original modeling request plus orthographic renders of the generated model from different viewpoints (ISO, FRONT, BACK, LEFT, RIGHT, TOP). The views are not reliably labeled — infer each view's orientation yourself.

Analyze step by step whether the generated model reasonably satisfies the request: are the main features present, are the shape and proportions correct, is anything missing / intersecting / floating / non-printable / over-simplified, does it match the description overall. This is a GEOMETRY sanity + prompt-match check ONLY — you are NOT judging physics or function, just whether the shape looks like a correct, complete model of what was asked.

Be strict but fair: pass when the model acceptably satisfies the request (main features present, proportions correct, no severe defects). Fail when something is missing, malformed, disconnected, or clearly wrong, and say concretely what to fix."""

GATE_SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "visual_gate_verdict",
        "strict": True,
        "schema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "passed": {"type": "boolean"},
                "reason": {"type": "string",
                           "description": "short explanation of the verdict"},
                "suggestions": {"type": "string",
                                "description": "concrete fixes for the generator; empty string if passed"},
            },
            "required": ["passed", "reason", "suggestions"],
        },
    },
}


def _b64(path):
    return base64.b64encode(Path(path).read_bytes()).decode()


def _build_messages(task, view_pngs, view_names=None):
    content = [{
        "type": "text",
        "text": (f"The user's original modeling request:\n{task or '(none)'}\n\n"
                 f"Below are {len(view_pngs)} orthographic renders. Judge geometry "
                 f"and prompt-match from them."),
    }]
    names = view_names or [None] * len(view_pngs)
    for name, fp in zip(names, view_pngs):
        if name:
            content.append({"type": "text", "text": f"view {name}:"})
        content.append({
            "type": "image_url",
            "image_url": {"url": f"data:image/png;base64,{_b64(fp)}"},
        })
    return [{"role": "system", "content": GATE_SYSTEM},
            {"role": "user", "content": content}]


def judge(task, view_pngs, view_names=None):
    """-> {passed, reason, suggestions}. If no images, fail closed (can't gate)."""
    if not view_pngs:
        return {"passed": False, "reason": "no rendered views to judge",
                "suggestions": "model did not compile to any view"}
    from openai import OpenAI
    client = OpenAI(base_url=os.environ["AZURE_OPENAI_ENDPOINT"].rstrip("/"),
                    api_key=os.environ["AZURE_OPENAI_API_KEY"])
    resp = client.chat.completions.create(
        model=os.environ.get("AZURE_VLM_DEPLOYMENT", "gpt-5.4"),
        messages=_build_messages(task, view_pngs, view_names),
        response_format=GATE_SCHEMA,
        max_completion_tokens=1500,
    )
    return json.loads(resp.choices[0].message.content)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("views", nargs="+", help="PNG view files")
    a = ap.parse_args()
    v = judge(a.task, a.views)
    print(json.dumps(v, indent=2))
