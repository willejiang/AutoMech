#!/usr/bin/env python3
"""
cadam_bridge.py — drive cadam's generation HEADLESSLY.

cadam (worker/) is a Supabase-bound web app; its generation path
(worker/src/server/aiChat.ts:handleAiChatRequest) needs auth + a DB conversation
branch. Rather than stand that up, this bridge replicates cadam's generation by
calling the same LLM with cadam's OWN system prompt (PARAMETRIC_AGENT_PROMPT,
worker/src/server/aiChat.ts) and the same build_parametric_model tool shape
(parametricArtifactSchema, worker/shared/chatAi.ts: {title, version, code}). The
output is raw OpenSCAD text — the artifact seam the rest of the loop consumes.

WORKER_MODE=http is reserved for "boot cadam's Nitro server and POST
/api/parametric-chat"; it's intentionally stubbed (that path needs Supabase).

Default provider = the same gateway the evaluator/worker use (OpenAI-compatible).
Env: AZURE_OPENAI_ENDPOINT + AZURE_OPENAI_API_KEY + AZURE_VLM_DEPLOYMENT (reused),
or override with CADAM_MODEL.
"""
import json
import os
import re

# Faithful condensation of worker/src/server/aiChat.ts PARAMETRIC_AGENT_PROMPT.
# Kept the geometry/parts/BOSL2/parameter rules that matter; dropped the
# browser-UI/answer_user/streaming bits (no UI here). The ONE addition for this
# project: parts must be SEPARATE top-level modules so the evaluator can
# articulate them (a fused union can't have joints).
CADAM_SYSTEM = """You are Adam, an agentic AI CAD engineer that writes OpenSCAD models.

Produce COMPLETE raw OpenSCAD code — no markdown, no code fences, no prose.

Do not rewrite or change the user's intent. Pass the request through faithfully.

Geometry rules:
- Syntax must be correct; the model must be manifold and 3D-printable.
- Write expert code. Use modules for meaningful parts.
- CRITICAL FOR SIMULATION: every distinct mechanical part that can move relative
  to another MUST be its own TOP-LEVEL module named in snake_case
  (e.g. module chassis(), module upright(), module wheel()). A downstream physics
  evaluator renders each part by calling `use <model.scad>; part_name();` and
  synthesizes joints between them — a single fused module cannot articulate.
  Call all the part modules once at the top level so the assembled model previews
  correctly, but keep each part independently renderable.

BOSL2:
- BOSL2 is available when the source includes the literal token BOSL2. Include
  <BOSL2/std.scad> plus the specific file (e.g. <BOSL2/screws.scad> for screw()/
  nut(), <BOSL2/threading.scad>, <BOSL2/skin.scad> for swept/lofted shapes). Set
  $fn = 64; or higher for threads/curves. Use BOSL2 rather than hand-rolled
  helices or stacked primitives.

Parameters:
- Declare every editable dimension as a top-of-file snake_case variable with a
  trailing Customizer comment (e.g. `wheel_radius = 40;  // [20:1:80]`).
- Group with /* [Group Name] */ markers.

Color:
- Wrap each distinct part in color("CssName") so the preview reads clearly; expose
  *_color string parameters.

Output ONLY the OpenSCAD code."""


def _revision_suffix(feedback):
    if not feedback:
        return ""
    return (
        "\n\nThis is a REVISION. A previous version of this model was rejected. "
        "Fix the issues below and output the corrected COMPLETE OpenSCAD script:\n"
        f"{feedback}"
    )


def _strip_fences(text):
    """Models sometimes wrap code despite instructions — strip ``` fences."""
    t = text.strip()
    m = re.match(r"^```(?:openscad|scad)?\s*\n(.*)\n```$", t, re.DOTALL)
    return m.group(1).strip() if m else t


def _client():
    from openai import OpenAI
    return OpenAI(
        base_url=os.environ["AZURE_OPENAI_ENDPOINT"].rstrip("/"),
        api_key=os.environ["AZURE_OPENAI_API_KEY"],
    )


def generate(task, feedback=None):
    """task (+ optional feedback) -> OpenSCAD source string. Replicates cadam's
    build_parametric_model generation without the browser."""
    mode = os.environ.get("WORKER_MODE", "direct")
    if mode == "http":
        raise NotImplementedError(
            "WORKER_MODE=http (POST cadam's /api/parametric-chat) needs Supabase "
            "auth + a DB conversation row. Not wired — use WORKER_MODE=direct.")

    user = (
        f"Create an OpenSCAD CAD model for this request:\n{task}"
        + _revision_suffix(feedback)
    )
    resp = _client().chat.completions.create(
        model=os.environ.get("CADAM_MODEL",
                             os.environ.get("AZURE_VLM_DEPLOYMENT", "gpt-5.4")),
        messages=[{"role": "system", "content": CADAM_SYSTEM},
                  {"role": "user", "content": user}],
        max_completion_tokens=4000,
    )
    return _strip_fences(resp.choices[0].message.content or "")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--feedback", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    scad = generate(a.task, a.feedback)
    if a.out:
        from pathlib import Path
        Path(a.out).write_text(scad, encoding="utf-8")
        print(f"[cadam] wrote {len(scad)} chars -> {a.out}")
    else:
        print(scad)
