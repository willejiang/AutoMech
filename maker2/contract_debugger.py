"""LLM contract-fault debugger (the gate-fault debugger, tier 2).

When the deterministic `fault_repair` can't cleanly fix a naming/realization fault, this hands
the SPECIFIC gate error plus the two subs' ACTUAL built part/frame names to a small LLM call and
asks for a MINIMAL patch — a mesh_pair rename, a frame re-point — applied in place. No full boss
re-plan, no geometry rebuild for a pure rename. This is the fuzzy fallback between the
deterministic repair (tier 1) and a boss re-plan (tier 3).

`debug_contract_fault(kind, detail, plan, subs, settings, log_fn) -> RepairResult` mutates the
plan/sub_frames in place and re-persists, mirroring fault_repair's contract.
"""

from __future__ import annotations

import json
import os

from .fault_repair import RepairResult, _persist_sub_frames
from .jsonutil import extract_json_object
from .llm.client import LLMError
from .llm.conversation import Conversation
from .prompts.contract_debugger_prompt import (CONTRACT_DEBUGGER_SYSTEM,
                                               build_contract_debugger_user)


def _sub_facts(plan, subs) -> dict:
    """Compact, real facts the debugger needs: each sub's built part names + gear parts, and its
    realized frames (frame -> link). No coordinates — this is a NAMING/realization fixer."""
    facts = {}
    for s in plan.subassemblies:
        res = subs.get(s.id)
        if not res or res.model is None:
            continue
        links = [l.name for l in res.model.links]
        gears = [l.name for l in res.model.links
                 if "gear" in l.name or "pinion" in l.name
                 or (l.size_mm or {}).get("module") or (l.size_mm or {}).get("teeth")]
        realized = {e.get("frame"): e.get("link") for e in (res.sub_frames or [])}
        declared = [fr.name for fr in (s.frames or [])]
        facts[s.id] = {"parts": links, "gears": gears,
                       "realized_frames": realized, "declared_frames": declared}
    return facts


def debug_contract_fault(kind: str, detail: str, plan, subs: dict, settings,
                         log_fn=print) -> RepairResult:
    """LLM fallback for a contract fault. Returns RepairResult; repaired=False -> boss re-plan."""
    out = RepairResult()
    facts = _sub_facts(plan, subs)
    seams = [{"id": s.id, "kind": s.kind, "parent_sub": s.parent_sub,
              "parent_frame": s.parent_frame, "child_sub": s.child_sub,
              "child_frame": s.child_frame, "mesh_pair": list(s.mesh_pair or ())}
             for s in plan.seams]
    try:
        client = settings.make_client(getattr(settings, "judger_max_tokens", 16000),
                                      thinking="off")
    except Exception as e:
        log_fn(f"[contract-debug] no client ({e}); skipping")
        return out
    conv = Conversation()
    conv.add_user_message(build_contract_debugger_user(kind, detail, facts, seams))
    try:
        text, _ = client.send_collect(
            conv.get_messages_for_api(api_style=client.api_style),
            system=CONTRACT_DEBUGGER_SYSTEM)
    except LLMError as e:
        log_fn(f"[contract-debug] LLM request failed ({e}); skipping")
        return out
    try:
        patch = json.loads(extract_json_object(text))
    except (ValueError, json.JSONDecodeError) as e:
        log_fn(f"[contract-debug] no usable JSON patch ({e}); skipping")
        return out

    seam_by_id = {s.id: s for s in plan.seams}
    frame_link = {(s.id): {e.get("frame"): e for e in (subs[s.id].sub_frames or [])}
                  for s in plan.subassemblies if s.id in subs and subs[s.id].model is not None}

    # seam_edits: rename a mesh_pair or a seam frame to a REAL name.
    for e in patch.get("seam_edits", []) or []:
        seam = seam_by_id.get(e.get("seam_id"))
        if seam is None:
            continue
        mp = e.get("mesh_pair")
        if isinstance(mp, (list, tuple)) and len(mp) == 2:
            # only accept names that actually exist in the respective subs
            pgood = mp[0] in (facts.get(seam.parent_sub, {}).get("parts", []))
            cgood = mp[1] in (facts.get(seam.child_sub, {}).get("parts", []))
            if pgood and cgood and tuple(seam.mesh_pair or ()) != tuple(mp):
                old = list(seam.mesh_pair or ())
                seam.mesh_pair = (mp[0], mp[1])
                out.repaired = True
                log_fn(f"[contract-debug] seam '{seam.id}': mesh_pair {old} -> {list(mp)}")

    # frame_edits: re-point a realized frame onto a REAL link in that sub.
    for e in patch.get("frame_edits", []) or []:
        sid = e.get("sub_id"); fr = e.get("frame"); link = e.get("link")
        res = subs.get(sid)
        if not res or res.model is None or not fr or not link:
            continue
        if link not in {l.name for l in res.model.links}:
            continue                          # only real links
        new = {"frame": fr, "link": link,
               "local_xyz_m": e.get("local_xyz_m", [0.0, 0.0, 0.0]),
               "local_rpy_rad": [0.0, 0.0, 0.0]}
        res.sub_frames = [x for x in (res.sub_frames or []) if x.get("frame") != fr]
        res.sub_frames.append(new)
        _persist_sub_frames(res, log_fn)
        out.repaired = True
        out.changed_subs.add(sid)
        log_fn(f"[contract-debug] sub '{sid}': frame '{fr}' -> link '{link}'")

    return out
