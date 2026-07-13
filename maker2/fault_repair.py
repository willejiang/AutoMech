"""Deterministic contract-fault repair (the gate-fault debugger, tier 1).

Most gate failures in the hierarchical pipeline are TRIVIAL cross-agent naming mismatches: the
boss names a gear/part/frame that a manager built under a different name (`inter_gear` vs the
built `large_gear`), or a manager realized a seat frame on the wrong link. The old response was
to throw the whole plan away and BOSS RE-PLAN — an 8-iteration loop to fix a one-token rename.

This module repairs those faults IN PLACE, deterministically, with NO LLM and NO re-plan. It is
the first tier of the gate-fault debugger; the caller falls back to an LLM contract-debugger and
finally a boss re-plan only when no deterministic rule applies.

`repair_contract_fault(kind, plan, subs, detail, log_fn) -> RepairResult` mutates `plan` (seam
fields) and each affected `SubResult.sub_frames` in place, re-persists `sub_frames.json`, and
reports what changed. `repaired=False` means "no clean rule applied — use the fallback."
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

import numpy as np


@dataclass
class RepairResult:
    repaired: bool = False
    changed_subs: set = field(default_factory=set)
    rebuilt_subs: set = field(default_factory=set)   # subs whose URDF must be rebuilt
    note: str = ""


def _persist_sub_frames(sub_result, log_fn) -> None:
    """Re-write a sub's sub_frames.json after an in-place repair so the assembler + any reuse
    read the corrected mapping."""
    ctx = getattr(sub_result, "ctx", None)
    run_dir = getattr(ctx, "run_dir", "") if ctx else ""
    if not run_dir:
        return
    try:
        with open(os.path.join(run_dir, "sub_frames.json"), "w", encoding="utf-8") as f:
            json.dump(sub_result.sub_frames, f, indent=2)
    except OSError as e:
        log_fn(f"[repair] could not rewrite sub_frames.json: {e}")


def _repair_mesh_pair(plan, subs, log_fn) -> RepairResult:
    """A power seam's `mesh_pair` names a gear not built in its sub, but the manager DID realize
    the mesh FRAME on its real gear. Rewrite mesh_pair from the frame-realized gear links (the
    role-based identity), so the stored plan, the module gate, and driver marking all agree with
    the assembler's frame-based solve. Repairs every mismatched power seam it can resolve."""
    from .assembler import _gear_link

    rr = RepairResult()
    for seam in plan.seams:
        if seam.kind != "power":
            continue
        p = subs.get(seam.parent_sub)
        c = subs.get(seam.child_sub)
        if not p or not c or p.model is None or c.model is None:
            continue
        gp = _gear_link(p, seam, 0)
        gc = _gear_link(c, seam, 1)
        if gp is None or gc is None:
            continue                              # can't resolve via frame either -> leave it
        want = (gp.name, gc.name)
        if tuple(seam.mesh_pair or ()) != want:
            old = tuple(seam.mesh_pair or ())
            seam.mesh_pair = want
            rr.repaired = True
            rr.note += (f"mesh_pair {list(old) or '[]'}->{list(want)} on seam "
                        f"'{seam.id}' (from realized frames); ")
            log_fn(f"[repair] seam '{seam.id}': mesh_pair {list(old) or '[]'} -> {list(want)} "
                   f"(resolved from mesh frames '{seam.parent_frame}'/'{seam.child_frame}') — no re-plan")
    return rr


def _repair_collapsed_frames(plan, subs, log_fn) -> RepairResult:
    """A sub realized several seat frames the boss placed APART all at ONE point (the root
    origin). For each such frame, re-point its realization to the sub's built part whose world
    position best matches the boss's declared frame `xyz_m` (relative to the sub origin). Rewrites
    the sub's sub_frames entry + re-persists. No geometry rebuild (only the frame->link mapping
    moves; the parts were already built at the right spots)."""
    from .assembler import _root_to_link

    rr = RepairResult()
    frame_of = {(s.id, fr.name): fr for s in plan.subassemblies for fr in (s.frames or [])}
    # subs that participate in a weld/power seam frame
    seam_frames: dict = {}
    for seam in plan.seams:
        if seam.kind in ("weld", "power"):
            seam_frames.setdefault(seam.parent_sub, set()).add(seam.parent_frame)
            seam_frames.setdefault(seam.child_sub, set()).add(seam.child_frame)

    for s in plan.subassemblies:
        res = subs.get(s.id)
        if not res or res.model is None:
            continue
        used = seam_frames.get(s.id, set())
        frames = [fr for fr in (s.frames or []) if fr.name in used]
        if len(frames) < 2:
            continue
        T = _root_to_link(res.model)
        entries = {e.get("frame"): e for e in (res.sub_frames or [])}

        # world pos of each realized frame (in sub root)
        def _world_of(fr_name):
            e = entries.get(fr_name)
            if not e:
                return None
            Tl = T.get(e.get("link"))
            if Tl is None:
                return None
            loc = np.array([float(v) for v in e.get("local_xyz_m", (0, 0, 0))])
            return (Tl[:3, :3] @ loc) + Tl[:3, 3]

        # are >=2 declared-apart frames realized coincident? (the collapse signature)
        realized = [(fr, _world_of(fr.name)) for fr in frames]
        realized = [(fr, w) for fr, w in realized if w is not None]
        collapsed = False
        for i in range(len(realized)):
            for j in range(i + 1, len(realized)):
                fa, wa = realized[i]; fb, wb = realized[j]
                want = float(np.linalg.norm(
                    np.array([float(v) for v in (fa.xyz_m or (0, 0, 0))])
                    - np.array([float(v) for v in (fb.xyz_m or (0, 0, 0))])))
                if want > 0.002 and float(np.linalg.norm(wa - wb)) <= 0.002:
                    collapsed = True
        if not collapsed:
            continue

        # Re-point each seat frame to the built part whose position best matches its declared
        # xyz_m (relative to the sub origin). Parts are placed in the sub's root frame by _root_to_link.
        # EXCLUDE the root link itself: a seat lives on a feature part (a bore/bearing), not the
        # structural body — realizing it on the root is exactly the collapse we're repairing.
        root = getattr(res.model, "root_link", "")
        part_world = {name: Tl[:3, 3] for name, Tl in T.items() if name != root}
        if not part_world:
            continue
        origin = np.array([0.0, 0.0, 0.0])       # the sub's root frame origin
        changed_here = False
        for fr in frames:
            want_local = np.array([float(v) for v in (fr.xyz_m or (0, 0, 0))]) - origin
            # nearest built (non-root) part to the declared local seat position
            best = min(part_world.items(),
                       key=lambda kv: float(np.linalg.norm(kv[1] - want_local)),
                       default=(None, None))
            bname, bpos = best
            if bname is None:
                continue
            gap = float(np.linalg.norm(bpos - want_local))
            if gap > 0.010:                      # no built part near this seat -> can't repair cleanly
                continue
            cur = entries.get(fr.name)
            if cur and cur.get("link") == bname and cur.get("local_xyz_m", [0, 0, 0]) == [0.0, 0.0, 0.0]:
                continue                         # already on the right part
            new_entry = {"frame": fr.name, "link": bname,
                         "local_xyz_m": [0.0, 0.0, 0.0], "local_rpy_rad": [0.0, 0.0, 0.0]}
            # replace or append
            res.sub_frames = [e for e in (res.sub_frames or []) if e.get("frame") != fr.name]
            res.sub_frames.append(new_entry)
            entries[fr.name] = new_entry
            changed_here = True
            log_fn(f"[repair] sub '{s.id}': seat '{fr.name}' re-pointed onto built part "
                   f"'{bname}' (was collapsed) — no re-plan")
        if changed_here:
            _persist_sub_frames(res, log_fn)
            rr.repaired = True
            rr.changed_subs.add(s.id)
            rr.note += f"re-pointed collapsed seats in '{s.id}'; "
    return rr


def repair_contract_fault(kind: str, plan, subs: dict, detail: str = "", log_fn=print) -> RepairResult:
    """Try a deterministic in-place repair for a CONTRACT (naming/realization) gate fault.

    `kind` selects the rule set:
      - "mesh"          -> rewrite mismatched power-seam mesh_pair from realized mesh frames.
      - "frame_agree"   -> re-point collapsed seat frames onto their real built parts.
      - "assembler"     -> try both (a stitch failure is usually one of these).
    Returns RepairResult(repaired, changed_subs, rebuilt_subs, note). repaired=False -> the caller
    should fall back to the LLM contract-debugger, then a boss re-plan."""
    out = RepairResult()
    rules = {
        "mesh": (_repair_mesh_pair,),
        "frame_agree": (_repair_collapsed_frames,),
        "assembler": (_repair_mesh_pair, _repair_collapsed_frames),
    }.get(kind, ())
    for rule in rules:
        try:
            r = rule(plan, subs, log_fn)
        except Exception as e:
            log_fn(f"[repair] rule {rule.__name__} errored ({type(e).__name__}: {e}); skipping")
            continue
        if r.repaired:
            out.repaired = True
            out.changed_subs |= r.changed_subs
            out.rebuilt_subs |= r.rebuilt_subs
            out.note += r.note
    return out
