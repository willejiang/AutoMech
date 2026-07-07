"""Holistic per-subassembly rigid-conflict DEBUGGER.

Invoked by the conflict gate in orchestrator_boss._finish_subassembly when
subcheck.sub_conflicts finds interpenetrating parts. One LLM call is handed the WHOLE
subassembly context — the user's request, the boss brief + immovable interface frames,
the model JSON, the URDF, and EVERY part's CadQuery script — and returns a minimal patch
that MOVES a part (edits a joint origin) and/or RESHAPES a part (edits its CadQuery
script). The patch is applied deterministically: pose edits mutate the model in place
(the caller rebuilds the URDF); script edits re-export just that part via
cq_worker.rebuild_link.

Reuses the manager's LLM plumbing: settings.manager_client() + Conversation +
twophase.stream_two_part (notes->JSON with cap recovery) + jsonutil.extract_json_object.
"""

from __future__ import annotations

import json
from pathlib import Path

from .jsonutil import extract_json_object
from .llm.client import LLMError
from .llm.conversation import Conversation
from .manager import model_to_dict
from .prompts.subdebugger_prompt import (SUBDEBUGGER_SYSTEM,
                                         build_subdebugger_json_from_notes,
                                         build_subdebugger_user)
from .twophase import stream_two_part


class SubDebuggerError(RuntimeError):
    """The debugger could not produce a usable patch."""


def _read_part_scripts(run_dir: str, model) -> dict[str, str]:
    """Read every persisted CadQuery part script for this sub, keyed by link name.
    Missing scripts are simply absent (a part built before the cq/ dir existed)."""
    cq_dir = Path(run_dir) / "cq"
    out: dict[str, str] = {}
    for l in model.links:
        p = cq_dir / f"{l.name}.py"
        if p.exists():
            try:
                out[l.name] = p.read_text(encoding="utf-8")
            except OSError:
                pass
    return out


def _read_text(path: str, limit: int = 20000) -> str:
    try:
        t = Path(path).read_text(encoding="utf-8")
        return t if len(t) <= limit else t[:limit] + "\n… (truncated)"
    except OSError:
        return "(unavailable)"


def _apply_pose_edits(model, pose_edits: list, frozen_links, log_fn) -> int:
    """Mutate matching joints' xyz_m/rpy_rad in place. Returns how many applied. A joint
    whose CHILD is an interface-frame link is FROZEN — moving it off its declared global
    frame would break the seam, so the edit is rejected (the debugger must fix the other
    part instead)."""
    by_name = {j.name: j for j in model.joints}
    n = 0
    for e in pose_edits or []:
        jn = e.get("joint")
        j = by_name.get(jn)
        if j is None:
            log_fn(f"[debugger] pose edit names unknown joint '{jn}'; skipped")
            continue
        if j.child in frozen_links:
            log_fn(f"[debugger] REJECTED pose edit on '{jn}': child '{j.child}' realizes "
                   "an interface frame (immovable)")
            continue
        xyz = e.get("xyz_m")
        rpy = e.get("rpy_rad")
        if isinstance(xyz, (list, tuple)) and len(xyz) == 3:
            j.xyz_m = tuple(float(v) for v in xyz)
        if isinstance(rpy, (list, tuple)) and len(rpy) == 3:
            j.rpy_rad = tuple(float(v) for v in rpy)
        n += 1
        log_fn(f"[debugger] moved joint '{jn}' -> xyz_m={list(j.xyz_m)}")
    return n


def _strip_cq_fences(text: str) -> str:
    t = (text or "").strip()
    if t.startswith("```"):
        body = t.split("\n", 1)[1] if "\n" in t else ""
        if body.rstrip().endswith("```"):
            body = body.rstrip()[:-3]
        return body.strip()
    return t


def _apply_script_edits(model, ctx, run_dir, script_edits: list, frozen_links,
                        log_fn) -> list[str]:
    """Re-export each edited part via cq_worker.rebuild_link. Returns the link names
    whose STL was successfully re-rendered. An interface-frame link is FROZEN — reshaping
    it (e.g. shrinking it to a plug) desyncs the frame the assembler relies on, so the
    edit is rejected and the debugger must fix the other part instead."""
    from .cq_worker import rebuild_link
    by_name = {l.name: l for l in model.links}
    changed: list[str] = []
    for e in script_edits or []:
        ln = e.get("link")
        link = by_name.get(ln)
        script = _strip_cq_fences(e.get("script") or "")
        if link is None:
            log_fn(f"[debugger] script edit names unknown link '{ln}'; skipped")
            continue
        if ln in frozen_links:
            log_fn(f"[debugger] REJECTED script edit on '{ln}': it realizes an interface "
                   "frame (its geometry is immovable)")
            continue
        if "def build_" not in script:
            log_fn(f"[debugger] script edit for '{ln}' has no build_ function; skipped")
            continue
        try:
            r = rebuild_link(link, script, ctx, run_dir, log_fn=log_fn)
        except Exception as ex:
            log_fn(f"[debugger] re-render of '{ln}' errored ({ex}); skipped")
            continue
        if r.success:
            changed.append(ln)
        else:
            log_fn(f"[debugger] re-render of '{ln}' failed to export "
                   f"({(r.error or '')[:80]})")
    return changed


def debug_sub(model, ctx, run_dir, spec, plan, user_prompt, conflicts, settings,
              frame_contract=None, log_fn=print):
    """Run ONE debugging pass over a conflicted subassembly.

    Returns (model, changed_links, moved) where ``changed_links`` are parts re-rendered
    from an edited script and ``moved`` is True if any joint pose changed (so the caller
    must rebuild the URDF). The model is mutated in place for pose edits; on any failure
    the model is returned unchanged so the caller can retry or fail up.
    """
    fc = frame_contract
    if fc is None:
        from .boss import frame_contract_for
        fc = frame_contract_for(plan, spec.id)
    frames = list(getattr(fc, "frames", []) or [])

    # Interface-frame links are immovable AND un-reshapeable: moving/shrinking one desyncs
    # the frame the assembler welds to. The set is every declared realization's link plus
    # any link named exactly like a contract frame (the manager's marker-link convention,
    # which the auto-realize fallback also relies on).
    link_names = {l.name for l in model.links}
    frozen_links = {e.get("link") for e in (getattr(model, "frames_realized", []) or [])
                    if e.get("link")}
    frozen_links |= {getattr(fr, "name", "") for fr in frames
                     if getattr(fr, "name", "") in link_names}

    part_scripts = _read_part_scripts(run_dir, model)
    model_json = json.dumps(model_to_dict(model), indent=2)
    urdf_text = _read_text(ctx.urdf_path)
    conflicts_desc = [c.describe() for c in conflicts]

    client = settings.manager_client()
    conv = Conversation()
    conv.add_user_message(build_subdebugger_user(
        user_prompt, getattr(spec, "brief", "") or "", frames,
        model_json, urdf_text, part_scripts, conflicts_desc,
        frozen_links=frozen_links))

    memory_path = str(Path(run_dir) / "debugger_memory.md")
    tag = f"debug:{spec.id}"
    try:
        text = stream_two_part(client, conv, SUBDEBUGGER_SYSTEM,
                               memory_path=memory_path,
                               regen_msg_fn=build_subdebugger_json_from_notes,
                               log_fn=log_fn, tag=tag)
    except LLMError as e:
        raise SubDebuggerError(f"debugger LLM request failed: {e}") from e

    try:
        patch = json.loads(extract_json_object(text))
    except (ValueError, json.JSONDecodeError) as e:
        raise SubDebuggerError(f"debugger returned no usable JSON patch: {e}") from e

    reason = (patch.get("reason") or "").strip()
    if reason:
        log_fn(f"[debugger] {reason[:200]}")

    moved = _apply_pose_edits(model, patch.get("pose_edits"), frozen_links, log_fn) > 0
    changed = _apply_script_edits(model, ctx, run_dir,
                                  patch.get("script_edits"), frozen_links, log_fn)
    if not moved and not changed:
        log_fn("[debugger] patch made no applicable change")
    return model, changed, moved
