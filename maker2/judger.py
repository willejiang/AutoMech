"""Judger (evaluator) agent: one built model -> a pass/fail verdict with feedback.

The judger closes the generate->judge->refine loop. Given the user's request, a
compact text summary of the assembled model, and (when rendering worked) up to
six orthographic views plus an optional reference photo, it asks the LLM for a
strict JSON verdict ``{"pass":bool,"reasons":str,"suggestions":str}``. The verdict
is saved as judge.json; on failure its suggestions are fed back to the manager.

Like the manager, this uses plain-JSON rather than tool-calling and repairs an
unparsable reply by feeding the error back, bounded by Settings.judger_retries.
The judger NEVER builds geometry and never mutates the model -- it only reads.
"""

from __future__ import annotations

import json
import time

from .imageutil import ImageLoadError, load_image_block
from .jsonutil import extract_json_object
from .llm.client import LLMError, LLMTruncationError
from .llm.conversation import Conversation
from .model import JudgeVerdict, KinematicModel, WorkerResult
from .prompts.judger_prompt import (JUDGER_SYSTEM, build_judger_repair,
                                    build_judger_shrink, build_judger_user)

# Joint types whose axis/limits are worth showing the evaluator.
_MOVING = {"revolute", "prismatic", "continuous"}


class JudgeError(RuntimeError):
    """The judger could not produce a usable verdict (parse or LLM failure)."""


def _ascii(text: str) -> str:
    """Coerce LLM-authored text to ASCII so the cp1252 console cannot crash."""
    return text.encode("ascii", "replace").decode("ascii")


def _summarize_model(model: KinematicModel,
                     results: list[WorkerResult]) -> str:
    """Compact ASCII summary of the built model for the evaluator's text channel.

    Lists each link (intended shape/size and the as-built bounding box from its
    STL report) and the joint tree (parent -> child, type, axis/limits for moving
    joints). This is what the judge falls back to when no views are available.
    """
    rep_by_name = {r.link_name: r for r in results}
    lines = [f"product: {model.name}", f"root link: {model.root_link}",
             f"links ({len(model.links)}):"]
    for l in model.links:
        size = ", ".join(f"{k}={v}" for k, v in l.size_mm.items()) or "n/a"
        shape = l.shape_hint or "free-form"
        r = rep_by_name.get(l.name)
        rep = r.stl_report if r else None
        if rep and rep.ok and rep.bbox_mm != (0.0, 0.0, 0.0):
            bx, by, bz = rep.bbox_mm
            built = f"built {bx:.1f}x{by:.1f}x{bz:.1f} mm, {rep.num_faces} faces"
        elif r and not r.success:
            built = "NOT BUILT (worker failed)"
        else:
            built = "not built"
        lines.append(f"  - {l.name}: shape={shape}, size_mm={{{size}}}; {built}")
        if l.description:
            lines.append(f"      desc: {_ascii(l.description)[:100]}")
    lines.append(f"joints ({len(model.joints)}):")
    for j in model.joints:
        extra = ""
        if j.type in _MOVING:
            ax = ",".join(f"{a:g}" for a in j.axis)
            extra = f" axis=({ax})"
            if j.lower is not None and j.upper is not None:
                extra += f" limits=[{j.lower:g},{j.upper:g}]"
        lines.append(f"  - {j.parent} -> {j.child} : {j.type}{extra}")
    return "\n".join(lines)


def parse_verdict(text: str) -> JudgeVerdict:
    """Parse an LLM reply into a JudgeVerdict. Raises ValueError on a bad shape."""
    obj = json.loads(extract_json_object(text))
    if not isinstance(obj, dict):
        raise ValueError("top-level JSON value is not an object")
    if "pass" not in obj:
        raise ValueError("verdict is missing the 'pass' boolean")
    passed = obj["pass"]
    if not isinstance(passed, bool):
        raise ValueError(f"'pass' must be true or false, got {passed!r}")
    reasons = obj.get("reasons") or ""
    suggestions = obj.get("suggestions") or ""
    if not isinstance(reasons, str):
        reasons = str(reasons)
    if not isinstance(suggestions, str):
        suggestions = str(suggestions)
    return JudgeVerdict(passed=passed, reasons=reasons.strip(),
                        suggestions=suggestions.strip(), raw=obj)


def _save_verdict(verdict: JudgeVerdict, path: str) -> None:
    """Write the verdict to judge.json (UTF-8, the user's three requested keys)."""
    payload = {
        "pass": verdict.passed,
        "reasons": verdict.reasons,
        "suggestions": verdict.suggestions,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def judge(product_prompt: str, model: KinematicModel,
          results: list[WorkerResult], view_pngs: dict, settings, *,
          reference_image_path: str | None = None,
          out_json_path: str | None = None, log_fn=None) -> JudgeVerdict:
    """Evaluate the built model and return a JudgeVerdict (also saved to disk).

    ``view_pngs`` maps view name -> PNG path; an empty dict means rendering was
    unavailable and the judge runs text-only. ``reference_image_path`` is the
    run's --image photo (attached FIRST, before the views, so the prompt can call
    it out). Writes the verdict to ``out_json_path`` when given. Raises JudgeError
    only if no attempt yields a parseable verdict.
    """
    client = settings.judger_client()
    summary = _summarize_model(model, results)

    # Reference photo first (the prompt says "the FIRST image is the reference"),
    # then the rendered views. A view that won't load is dropped, not fatal.
    images: list[dict] = []
    has_reference = False
    if reference_image_path:
        try:
            images.append(load_image_block(reference_image_path))
            has_reference = True
        except ImageLoadError as e:
            if log_fn:
                log_fn(f"[judge] reference image skipped ({e})")
    attached_views: list[str] = []
    for name, png in view_pngs.items():
        try:
            images.append(load_image_block(png))
            attached_views.append(name)
        except ImageLoadError as e:
            if log_fn:
                log_fn(f"[judge] view '{name}' skipped ({e})")

    conv = Conversation()
    conv.add_user_message(
        build_judger_user(product_prompt, summary, attached_views, has_reference),
        images=images or None)
    if log_fn:
        kind = (f"{len(attached_views)} view(s)" if attached_views
                else "text-only (no views)")
        ref = " + reference" if has_reference else ""
        log_fn(f"[judge] evaluating with {kind}{ref}...")

    last_err = ""
    attempts = settings.judger_retries + 1
    for attempt in range(1, attempts + 1):
        messages = conv.get_messages_for_api(api_style=client.api_style)
        try:
            text = client.send(messages, system=JUDGER_SYSTEM)
        except LLMTruncationError as e:
            # Verdict overran the output cap; ask for a shorter one and retry.
            last_err = str(e).splitlines()[0]
            if log_fn:
                log_fn(f"[judge] attempt {attempt}/{attempts} over output cap; "
                       f"asking for a shorter verdict and retrying")
            conv.add_user_message(build_judger_shrink())
            time.sleep(min(2 * attempt, 10))
            continue
        except LLMError as e:
            # Transient send failure under load; back off and retry the same prompt.
            last_err = str(e).splitlines()[0]
            if log_fn:
                log_fn(f"[judge] attempt {attempt}/{attempts} LLM request failed; "
                       f"backing off and retrying")
            time.sleep(min(2 * attempt, 10))
            continue
        conv.add_assistant_message(text)
        try:
            verdict = parse_verdict(text)
        except (ValueError, json.JSONDecodeError) as e:
            last_err = str(e)
            if log_fn:
                log_fn(f"[judge] attempt {attempt}/{attempts} unparsable verdict: "
                       f"{last_err}")
            conv.add_user_message(build_judger_repair(last_err))
            continue
        if out_json_path:
            _save_verdict(verdict, out_json_path)
        if log_fn:
            tag = "PASS" if verdict.passed else "FAIL"
            log_fn(f"[judge] verdict: {tag} -- {_ascii(verdict.reasons)[:200]}")
        return verdict

    raise JudgeError(
        f"Judge failed to return a usable verdict after {attempts} attempts. "
        f"Last error:\n{last_err}")
