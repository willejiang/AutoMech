"""Boss agent: one machine prompt -> a validated SubassemblyPlan.

The boss is the hierarchy's top layer. It asks the LLM for a plain-JSON split of a
big machine into SUBASSEMBLIES (each a single manager's job) plus the SEAMS that
join them, parses it into the dataclass contract, then runs a pure-Python
validation pass the LLM can't be trusted to satisfy: URDF-safe unique ids/frames/
seams, one root subassembly with every other reachable through weld seams, seam
endpoints/frames that exist, and at most one power input (driver).

Validation MUTATES the plan into a normalized form (slugified ids propagated into
seam endpoints; frame/seam names slugified) and raises BossError listing every
problem at once, fed back to the LLM as a repair request bounded by
Settings.manager_retries. Like the manager, we use plain JSON, not tool-calling,
because the local gateway's tool support is unverified and one JSON object is
trivial to validate + repair.

CLI:  python -m maker2.boss "a tunnel boring machine" --out output
      -> writes <out>/<slug>_<ts>/subassembly_plan.json (and prints it).
"""

from __future__ import annotations

import json
import re

from .imageutil import ImageLoadError, load_image_block
from .jsonutil import extract_json_object
from .llm.client import LLMError
from .llm.conversation import Conversation
from .model import (FrameContract, MountFrame, SeamSpec, SubassemblyPlan,
                    SubassemblySpec)
from .prompts.boss_prompt import (BOSS_SYSTEM, build_boss_coarser, build_boss_feedback,
                                  build_boss_json_from_notes, build_boss_prior_plan,
                                  build_boss_refine, build_boss_replan,
                                  build_boss_repair, build_boss_repair_diff, build_boss_user)
from .twophase import stream_two_part


_VALID_JOINT_TYPES = {"fixed", "revolute", "prismatic", "continuous"}
_VALID_SEAM_KINDS = {"weld", "power"}
_VALID_ROLES = {"mount", "power_in", "power_out", "mesh"}
_URDF_SAFE = re.compile(r"^[a-z][a-z0-9_]*$")


class BossError(RuntimeError):
    """The boss could not produce a valid plan (parse or validation)."""


# --------------------------------------------------------------------------- #
# Dataclass parsing (mirrors manager.parse_model)
# --------------------------------------------------------------------------- #

def _as_tuple3(value, default: tuple) -> tuple:
    if value is None:
        return default
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"expected a 3-number list, got {value!r}")
    return tuple(float(x) for x in value)


def _opt_float(value):
    return None if value is None else float(value)


def _frame_from_dict(d: dict, sub_id: str, idx: int) -> MountFrame:
    if not isinstance(d, dict):
        raise ValueError(f"{sub_id}.frames[{idx}] is not an object")
    name = d.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"{sub_id}.frames[{idx}] is missing a non-empty 'name'")
    role = str(d.get("role") or "mount").strip().lower()
    return MountFrame(
        name=name.strip(),
        xyz_m=_as_tuple3(d.get("xyz_m"), (0.0, 0.0, 0.0)),
        rpy_rad=_as_tuple3(d.get("rpy_rad"), (0.0, 0.0, 0.0)),
        axis=_as_tuple3(d.get("axis"), (0.0, 0.0, 1.0)),
        shaft_dia_mm=float(d.get("shaft_dia_mm") or 0.0),
        link=str(d.get("link") or ""),
        role=role,
        mounts_part=str(d.get("mounts_part") or "").strip(),
    )


def _sub_from_dict(d: dict, idx: int) -> SubassemblySpec:
    if not isinstance(d, dict):
        raise ValueError(f"subassemblies[{idx}] is not an object")
    sub_id = d.get("id")
    if not isinstance(sub_id, str) or not sub_id.strip():
        raise ValueError(f"subassemblies[{idx}] is missing a non-empty 'id'")
    sub_id = sub_id.strip()
    frames = d.get("frames") or []
    if not isinstance(frames, list):
        raise ValueError(f"subassemblies[{idx}] '{sub_id}': frames must be an array")
    return SubassemblySpec(
        id=sub_id,
        brief=str(d.get("brief") or ""),
        function=str(d.get("function") or ""),
        frames=[_frame_from_dict(f, sub_id, i) for i, f in enumerate(frames)],
        input_tags=[str(t) for t in (d.get("input_tags") or [])],
        output_tags=[str(t) for t in (d.get("output_tags") or [])],
        est_link_budget=int(d.get("est_link_budget", 30)),
        instances=_instances_from_dict(d.get("instances"), sub_id),
    )


def _instances_from_dict(value, sub_id: str) -> list:
    """Normalize a sub's `instances` (repeated-copy poses) to [{xyz_m, rpy_rad}, ...].
    Tolerant: a missing/empty list means a single instance."""
    if not isinstance(value, list):
        return []
    out = []
    for i, e in enumerate(value):
        if not isinstance(e, dict):
            raise ValueError(f"sub '{sub_id}' instances[{i}] is not an object")
        xyz = e.get("xyz_m") or [0.0, 0.0, 0.0]
        rpy = e.get("rpy_rad") or [0.0, 0.0, 0.0]
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError(f"sub '{sub_id}' instances[{i}] needs 3-number xyz_m/rpy_rad")
        out.append({"xyz_m": [float(x) for x in xyz],
                    "rpy_rad": [float(x) for x in rpy]})
    return out


def _seam_from_dict(d: dict, idx: int) -> SeamSpec:
    if not isinstance(d, dict):
        raise ValueError(f"seams[{idx}] is not an object")
    sid = d.get("id")
    if not isinstance(sid, str) or not sid.strip():
        raise ValueError(f"seams[{idx}] is missing a non-empty 'id'")
    for key in ("kind", "parent_sub", "parent_frame", "child_sub", "child_frame"):
        v = d.get(key)
        if not isinstance(v, str) or not v.strip():
            raise ValueError(f"seams[{idx}] '{sid.strip()}' is missing '{key}'")
    mp = d.get("mesh_pair") or ()
    if mp and (not isinstance(mp, (list, tuple)) or len(mp) != 2):
        raise ValueError(f"seams[{idx}] '{sid.strip()}': mesh_pair must be [drive, driven]")
    kind = str(d.get("kind")).strip().lower()
    mate_type = str(d.get("mate_type") or "").strip().lower()
    if mate_type and mate_type not in ("insert", "seat", "mesh"):
        raise ValueError(f"seams[{idx}] '{sid.strip()}': mate_type must be one of "
                         f"insert|seat|mesh (got '{mate_type}')")
    # mate_type is REQUIRED on a WELD seam: the boss authors the connection graph, and the
    # compiler places each sub by welding its port onto its neighbor's realized port — there
    # is no coordinate fallback anymore, so a weld with no mate_type cannot be placed.
    if kind == "weld" and mate_type not in ("insert", "seat"):
        raise ValueError(
            f"seams[{idx}] '{sid.strip()}': a weld seam needs mate_type 'insert' (a "
            f"shaft/pin end into a bore/hole) or 'seat' (a face resting on a face). The boss "
            f"no longer authors placement coordinates; every weld MUST declare how its two "
            f"frames mate so the compiler can place the child.")
    # extra_pins: an optional list of [parent_port, child_port] anti-spin dowel pairs.
    raw_pins = d.get("extra_pins") or ()
    pins: list = []
    if raw_pins:
        if not isinstance(raw_pins, (list, tuple)):
            raise ValueError(f"seams[{idx}] '{sid.strip()}': extra_pins must be a list of "
                             "[parent_port, child_port] pairs")
        for j, pr in enumerate(raw_pins):
            if not isinstance(pr, (list, tuple)) or len(pr) != 2:
                raise ValueError(f"seams[{idx}] '{sid.strip()}': extra_pins[{j}] must be "
                                 "[parent_port, child_port]")
            pins.append((str(pr[0]).strip(), str(pr[1]).strip()))
    return SeamSpec(
        id=sid.strip(),
        kind=str(d.get("kind")).strip().lower(),
        parent_sub=str(d.get("parent_sub")).strip(),
        parent_frame=str(d.get("parent_frame")).strip(),
        child_sub=str(d.get("child_sub")).strip(),
        child_frame=str(d.get("child_frame")).strip(),
        joint_type=str(d.get("joint_type") or "fixed").strip().lower(),
        axis=_as_tuple3(d.get("axis"), (0.0, 0.0, 1.0)),
        lower=_opt_float(d.get("lower")),
        upper=_opt_float(d.get("upper")),
        effort=float(d.get("effort", 10.0)),
        velocity=float(d.get("velocity", 1.0)),
        driver=bool(d.get("driver", False)),
        owner_sub=str(d.get("owner_sub") or "").strip(),
        mesh_pair=tuple(str(x).strip() for x in mp) if mp else (),
        mate_type=mate_type,
        parent_port=str(d.get("parent_port") or "").strip(),
        child_port=str(d.get("child_port") or "").strip(),
        offset_mm=float(d.get("offset_mm", 0.0)),
        clock_rad=float(d.get("clock_rad", 0.0)),
        extra_pins=tuple(pins),
    )


def parse_plan(text: str) -> SubassemblyPlan:
    """Parse an LLM response into a (not-yet-validated) SubassemblyPlan."""
    obj = json.loads(extract_json_object(text))
    if not isinstance(obj, dict):
        raise ValueError("top-level JSON value is not an object")
    subs = obj.get("subassemblies")
    seams = obj.get("seams")
    if not isinstance(subs, list) or not subs:
        raise ValueError("'subassemblies' must be a non-empty array")
    if not isinstance(seams, list):
        raise ValueError("'seams' must be an array")
    return SubassemblyPlan(
        name=str(obj.get("name") or "machine"),
        root_sub=str(obj.get("root_sub") or ""),
        global_origin_note=str(obj.get("global_origin_note") or ""),
        subassemblies=[_sub_from_dict(d, i) for i, d in enumerate(subs)],
        seams=[_seam_from_dict(d, i) for i, d in enumerate(seams)],
    )


# --------------------------------------------------------------------------- #
# Normalization + validation (mirrors manager._validate_model)
# --------------------------------------------------------------------------- #

def _slugify(name: str, fallback: str) -> str:
    s = re.sub(r"[^a-z0-9_]+", "_", str(name).strip().lower())
    s = re.sub(r"_+", "_", s).strip("_")
    if not _URDF_SAFE.match(s):
        s = re.sub(r"_+", "_", f"{fallback}_{s}").strip("_")
    return s if _URDF_SAFE.match(s) else fallback


def _dedupe(slug: str, used: set) -> str:
    if slug not in used:
        return slug
    i = 2
    while f"{slug}_{i}" in used:
        i += 1
    return f"{slug}_{i}"


def _validate_plan(plan: SubassemblyPlan) -> None:
    """Normalize ids/names + verify the plan is one weld-connected machine.

    Mutates ``plan`` in place: subassembly ids are slugified/deduped and the new
    ids are propagated into seams (parent_sub/child_sub/owner_sub) and into
    root_sub; frame names are slugified/deduped within each sub; seam ids are
    slugified/deduped. Collects all problems and raises BossError together so one
    repair round-trip can fix everything.
    """
    problems: list[str] = []

    # 1. Normalize subassembly ids, building old -> new remap.
    sub_remap: dict[str, str] = {}
    used_subs: set[str] = set()
    for sub in plan.subassemblies:
        slug = _dedupe(_slugify(sub.id, "sub"), used_subs)
        used_subs.add(slug)
        sub_remap[sub.id] = slug
        sub.id = slug
    valid_subs = set(used_subs)

    # 2. Normalize frame names within each sub; collect the sub's frame set.
    frames_by_sub: dict[str, set[str]] = {}
    for sub in plan.subassemblies:
        used_frames: set[str] = set()
        for fr in sub.frames:
            slug = _dedupe(_slugify(fr.name, "frame"), used_frames)
            used_frames.add(slug)
            fr.name = slug
            if fr.role not in _VALID_ROLES:
                problems.append(f"sub '{sub.id}' frame '{fr.name}' has invalid role "
                                f"'{fr.role}' (expected one of {sorted(_VALID_ROLES)})")
        frames_by_sub[sub.id] = used_frames
        if sub.est_link_budget > 25:
            problems.append(f"sub '{sub.id}' est_link_budget {sub.est_link_budget} "
                            f"> 25 (too big for one subassembly — split it into more)")

    # 3. Normalize seam ids; remap seam endpoints through the sub table.
    used_seams: set[str] = set()
    for seam in plan.seams:
        slug = _dedupe(_slugify(seam.id, "seam"), used_seams)
        used_seams.add(slug)
        seam.id = slug
        seam.parent_sub = sub_remap.get(seam.parent_sub, _slugify(seam.parent_sub, "sub"))
        seam.child_sub = sub_remap.get(seam.child_sub, _slugify(seam.child_sub, "sub"))
        if seam.owner_sub:
            seam.owner_sub = sub_remap.get(seam.owner_sub, _slugify(seam.owner_sub, "sub"))

    # 4. root_sub remap + existence.
    plan.root_sub = sub_remap.get(plan.root_sub, _slugify(plan.root_sub, "sub"))
    if plan.root_sub not in valid_subs:
        problems.append(f"root_sub '{plan.root_sub}' is not one of the subassemblies "
                        f"{sorted(valid_subs)}")

    # 5. Seam endpoints must reference real subs + real frames; no self-seam.
    for seam in plan.seams:
        if seam.kind not in _VALID_SEAM_KINDS:
            problems.append(f"seam '{seam.id}' has invalid kind '{seam.kind}' "
                            f"(expected one of {sorted(_VALID_SEAM_KINDS)})")
        if seam.parent_sub not in valid_subs:
            problems.append(f"seam '{seam.id}' references unknown parent_sub '{seam.parent_sub}'")
        elif seam.parent_frame not in frames_by_sub.get(seam.parent_sub, set()):
            problems.append(f"seam '{seam.id}' parent_frame '{seam.parent_frame}' "
                            f"is not a frame on '{seam.parent_sub}'")
        if seam.child_sub not in valid_subs:
            problems.append(f"seam '{seam.id}' references unknown child_sub '{seam.child_sub}'")
        elif seam.child_frame not in frames_by_sub.get(seam.child_sub, set()):
            problems.append(f"seam '{seam.id}' child_frame '{seam.child_frame}' "
                            f"is not a frame on '{seam.child_sub}'")
        if seam.parent_sub == seam.child_sub:
            problems.append(f"seam '{seam.id}' connects sub '{seam.parent_sub}' to itself")
        if seam.joint_type not in _VALID_JOINT_TYPES:
            problems.append(f"seam '{seam.id}' has invalid joint_type '{seam.joint_type}'")
        if seam.joint_type in ("revolute", "prismatic") and (seam.lower is None or seam.upper is None):
            problems.append(f"seam '{seam.id}' ({seam.joint_type}) needs both 'lower' and 'upper'")
        if seam.kind == "power" and seam.owner_sub and seam.owner_sub not in valid_subs:
            problems.append(f"seam '{seam.id}' owner_sub '{seam.owner_sub}' is not a subassembly")

    # 6. At most one power INPUT (driver) in the whole machine.
    drivers = [s.id for s in plan.seams if s.driver]
    if len(drivers) > 1:
        problems.append(f"more than one seam has driver:true {drivers} (the machine "
                        f"has ONE power input)")

    # 7. WELD seams must connect every subassembly into one tree rooted at root_sub.
    if plan.root_sub in valid_subs:
        adjacency: dict[str, list[str]] = {}
        for seam in plan.seams:
            if seam.kind == "weld":
                adjacency.setdefault(seam.parent_sub, []).append(seam.child_sub)
                adjacency.setdefault(seam.child_sub, []).append(seam.parent_sub)
        visited: set[str] = set()
        stack = [plan.root_sub]
        while stack:
            node = stack.pop()
            if node in visited:
                continue
            visited.add(node)
            stack.extend(adjacency.get(node, []))
        unreached = sorted(valid_subs - visited)
        if unreached:
            problems.append(f"subassemblies not reachable from root '{plan.root_sub}' "
                            f"through weld seams: {unreached} (every sub must be welded "
                            f"into the machine)")

    if problems:
        raise BossError("Plan validation failed:\n- " + "\n- ".join(problems))


# --------------------------------------------------------------------------- #
# Persistence (mirrors manager.model_to_dict / save_model / load_model)
# --------------------------------------------------------------------------- #

def _frame_to_dict(fr: MountFrame) -> dict:
    return {"name": fr.name, "xyz_m": list(fr.xyz_m), "rpy_rad": list(fr.rpy_rad),
            "axis": list(fr.axis), "shaft_dia_mm": fr.shaft_dia_mm,
            "link": fr.link, "role": fr.role, "mounts_part": fr.mounts_part}


def plan_to_dict(plan: SubassemblyPlan) -> dict:
    """Serialize a SubassemblyPlan to the same JSON shape the boss emits."""
    return {
        "name": plan.name,
        "root_sub": plan.root_sub,
        "global_origin_note": plan.global_origin_note,
        "subassemblies": [
            {
                "id": s.id,
                "brief": s.brief,
                "function": s.function,
                "est_link_budget": s.est_link_budget,
                "input_tags": list(s.input_tags),
                "output_tags": list(s.output_tags),
                "frames": [_frame_to_dict(f) for f in s.frames],
                "instances": [dict(inst) for inst in (s.instances or [])],
            }
            for s in plan.subassemblies
        ],
        "seams": [
            {
                "id": s.id,
                "kind": s.kind,
                "parent_sub": s.parent_sub,
                "parent_frame": s.parent_frame,
                "child_sub": s.child_sub,
                "child_frame": s.child_frame,
                "mate_type": s.mate_type,
                "parent_port": s.parent_port,
                "child_port": s.child_port,
                "offset_mm": s.offset_mm,
                "clock_rad": s.clock_rad,
                "extra_pins": [list(pr) for pr in (s.extra_pins or ())],
                "joint_type": s.joint_type,
                "axis": list(s.axis),
                "lower": s.lower,
                "upper": s.upper,
                "effort": s.effort,
                "velocity": s.velocity,
                "driver": s.driver,
                "owner_sub": s.owner_sub,
                "mesh_pair": list(s.mesh_pair),
            }
            for s in plan.seams
        ],
    }


def plan_from_dict(obj: dict) -> SubassemblyPlan:
    """Build a SubassemblyPlan from a plain dict (validated separately)."""
    return parse_plan(json.dumps(obj))


def save_plan(plan: SubassemblyPlan, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(plan_to_dict(plan), f, indent=2)


def load_plan(path: str) -> SubassemblyPlan:
    with open(path, "r", encoding="utf-8") as f:
        return parse_plan(f.read())


def frame_contract_for(plan: SubassemblyPlan, sub_id: str,
                       *, appearance_summary: str = "") -> FrameContract:
    """Build the FrameContract Stage B hands to one subassembly's manager.
    ``appearance_summary`` (1c) is the optional coarse whole-machine layout text."""
    sub = plan.sub_by_id(sub_id)
    if sub is None:
        raise BossError(f"no subassembly '{sub_id}' in plan")
    # Fall back to a summary stashed on the plan by run_boss (1c) when the caller
    # doesn't pass one explicitly — keeps build_subassembly's call site unchanged.
    summary = appearance_summary or getattr(plan, "appearance_summary", "") or ""
    return FrameContract(
        sub_id=sub.id,
        frames=list(sub.frames),
        global_origin_note=plan.global_origin_note,
        input_tags=list(sub.input_tags),
        output_tags=list(sub.output_tags),
        neighbors=[{"id": o.id, "function": o.function, "brief": o.brief}
                   for o in plan.subassemblies if o.id != sub_id],
        appearance_summary=summary,
    )


# --------------------------------------------------------------------------- #
# Top-level planning (LLM + repair loop) — mirrors manager.decompose
# --------------------------------------------------------------------------- #

def plan_machine(product_prompt: str, settings, *, image_path: str | None = None,
                 plan_json_path: str | None = None,
                 feedback: str | None = None,
                 refine_message: str | None = None,
                 prior_plan_json: str | None = None,
                 log_fn=None) -> SubassemblyPlan:
    """Split a machine prompt into a validated, persisted SubassemblyPlan.

    ``feedback`` (a later loop pass) delivers an interface/assembly-level fault and
    asks the boss to re-plan. ``refine_message`` + ``prior_plan_json`` (multi-turn)
    hand the boss its previous plan and the user's change so it UPDATES the plan
    (keeping unchanged subs' ids so they can be reused). On truncation the boss is
    asked for FEWER, larger subassemblies; on parse/validation failure the error is
    fed back as a repair request, bounded by ``settings.manager_retries``. Raises
    BossError if no attempt yields a valid plan.
    """
    client = settings.boss_client()
    conv = Conversation()
    try:
        images = [load_image_block(image_path)] if image_path else None
    except ImageLoadError as e:
        raise BossError(str(e)) from e
    conv.add_user_message(
        build_boss_user(product_prompt, has_image=bool(image_path)),
        images=images)
    if image_path and log_fn:
        log_fn(f"[boss] using input image: {image_path}")
    # Show the prior plan FIRST (both refine and fault re-plan build on it), so the
    # boss keeps unchanged subassembly ids and they can be REUSED from disk.
    if prior_plan_json:
        conv.add_user_message(build_boss_prior_plan(prior_plan_json))
    if feedback:
        # A fault re-plan. With a prior plan present, ask the boss to change ONLY what
        # the fault requires and keep every other sub's id/brief/frames EXACTLY
        # (build_boss_replan); without one, fall back to the old from-scratch re-plan.
        conv.add_user_message(
            build_boss_replan(feedback) if prior_plan_json
            else build_boss_feedback(feedback))
        if log_fn:
            log_fn("[boss] re-planning from a fault"
                   + (" (keeping unchanged subassemblies)" if prior_plan_json else ""))
    if refine_message:
        conv.add_user_message(build_boss_refine(refine_message))
        if log_fn:
            log_fn(f"[boss] refining the plan: {refine_message[:80]}")

    # SEAM (Track 1): optional research pre-step.
    _boss_research(client, conv, settings, product_prompt, log_fn=log_fn)

    # Scratch memory path for two-phase cap-cut recovery.
    from pathlib import Path
    memory_path = (str(Path(plan_json_path).parent / "boss_memory.md")
                   if plan_json_path else None)

    # SEAM (Track 3): the attempt/retry control loop.
    return _plan_loop(client, conv, settings, memory_path=memory_path,
                      plan_json_path=plan_json_path, log_fn=log_fn)


def _boss_research(client, conv, settings, product_prompt, *, log_fn=None) -> None:
    """SEAM owned by Track 1 (RAG). Optional web-search / KB research pre-step, gated by
    settings.enable_reference_tools (web) and settings.enable_kb (local KB): look up
    reference designs / typical layouts and decomposition conventions, and inject
    findings into ``conv`` before planning. kb_search is pinned to the boss
    collection."""
    from .tools import maybe_research
    maybe_research(client, conv, settings,
                   f"plan the subassemblies of: {product_prompt}",
                   collection="boss", log_fn=log_fn)


def _plan_gate_badness(plan) -> tuple[float, list, dict]:
    """A pre-build 'badness' for a parsed plan from the deterministic boss gates (schema +
    support-chain + mesh-distance). Returns (badness, errors, breakdown). Lower = closer to
    a valid, buildable plan. Pure-Python; imported lazily to keep boss import light."""
    try:
        from .benchmarks.schema_gate import boss_schema_gate
        from .benchmarks.boss_gate import boss_gate
    except Exception:
        return 0.0, [], {"terms": {}, "count": 0}
    errs = boss_schema_gate(plan) + boss_gate(plan)
    # Weight support/interface faults (structural) above pure schema enum faults.
    w = {"ERR_SUP_NOWELD": 5.0, "ERR_IFC_MESH_DIST": 4.0}
    total = float(sum(w.get(e.code, 2.0) for e in errs))
    by_code: dict = {}
    for e in errs:
        by_code[e.code] = by_code.get(e.code, 0) + 1
    return total, errs, {"total": round(total, 3), "terms": by_code, "count": len(errs)}


def _plan_loop(client, conv, settings, *, memory_path, plan_json_path=None,
               log_fn=None) -> SubassemblyPlan:
    """SEAM owned by Track 3 (badness keep-best + escalation). The attempt/retry control
    loop: stream a NOTES→JSON response, parse+validate, then score the plan with the
    deterministic boss gates. MONOTONIC-IMPROVEMENT contract (Part C.bis):
      * a CLEAN plan (no gate errors) returns immediately.
      * otherwise KEEP the lowest-badness plan; feed the boss DIFF-CARRYING feedback stating
        whether the last plan got closer + which checks moved.
      * at loop end return the BEST plan even if imperfect (partial beats failure).
      * on a plateau, ESCALATE by asking for FEWER/larger subassemblies (build_boss_coarser).
    Bounded by settings.manager_retries."""
    from .badness import format_delta

    attempts = settings.manager_retries + 1
    plateau_k = int(getattr(settings, "loop_plateau_k", 2))
    eps = 1e-3
    best_badness = float("inf")
    best_plan: SubassemblyPlan | None = None
    best_bd: dict = {}
    prev_bd: dict = {}
    last_err = ""
    stall = 0

    for attempt in range(1, attempts + 1):
        if log_fn:
            log_fn(f"[boss] attempt {attempt}/{attempts}: planning subassemblies "
                   f"(streaming)…")
        try:
            text = stream_two_part(client, conv, BOSS_SYSTEM,
                                   memory_path=memory_path,
                                   regen_msg_fn=build_boss_json_from_notes,
                                   log_fn=log_fn, tag="boss")
        except LLMError as e:
            raise BossError(f"Boss LLM request failed: {e}") from e
        conv.add_assistant_message(text)

        # 1. Parse + normalize. A hard structural failure (unknown refs, unspanned weld
        #    graph) raises here -> feed the error back and retry.
        try:
            plan = parse_plan(text)
            _validate_plan(plan)
        except (ValueError, BossError, json.JSONDecodeError) as e:
            last_err = str(e)
            if log_fn:
                log_fn(f"[boss] attempt {attempt}/{attempts} rejected (parse): {last_err}")
            note = format_delta(prev_bd, best_bd) if best_bd else "no scored plan yet"
            conv.add_user_message(build_boss_repair_diff(last_err, note))
            continue

        # 2. Score the plan with the deterministic gates.
        cur_badness, errs, cur_bd = _plan_gate_badness(plan)
        if log_fn:
            log_fn(f"[boss] attempt {attempt}: plan badness={cur_badness:.2f} "
                   f"({len(errs)} gate issue(s))")

        # 2a. CLEAN -> done.
        if not errs:
            if plan_json_path:
                save_plan(plan, plan_json_path)
            if log_fn:
                log_fn(f"[boss] OK on attempt {attempt}: {len(plan.subassemblies)} "
                       f"subassemblies, {len(plan.seams)} seams, root='{plan.root_sub}' "
                       "(clean)")
            return plan

        # 2b. keep-best on badness.
        if cur_badness < best_badness - eps:
            best_badness, best_plan, best_bd = cur_badness, plan, cur_bd
            stall = 0
        else:
            stall += 1
        last_err = ("plan failed automated checks:\n"
                    + "\n".join(f"- {e}" for e in errs[:12]))
        delta_note = format_delta(prev_bd, cur_bd)
        prev_bd = cur_bd

        # 2c. Escalate on plateau: fewer/larger subs.
        if stall >= plateau_k and attempt < attempts:
            stall = 0
            if log_fn:
                log_fn("[boss] plateau -> escalate: requesting FEWER, larger subassemblies")
            conv.add_user_message(build_boss_coarser(
                last_err + "\n\n(this split is not converging — merge related subs.)"))
        else:
            conv.add_user_message(build_boss_repair_diff(last_err, delta_note))

    if best_plan is not None:
        if plan_json_path:
            save_plan(best_plan, plan_json_path)
        if log_fn:
            log_fn(f"[boss] no clean plan in {attempts}; returning BEST "
                   f"(badness {best_badness:.2f})")
        return best_plan
    raise BossError(
        f"Boss failed after {attempts} attempts. Last error:\n{last_err}")


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main() -> int:
    import argparse
    import os
    import sys
    from .config import Settings
    from .orchestrator import make_run_context

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            pass

    ap = argparse.ArgumentParser(description="maker2 boss: machine -> SubassemblyPlan")
    ap.add_argument("prompt", help="natural-language machine description")
    ap.add_argument("--out", default="output")
    ap.add_argument("--model", default=None, help="LLM for the boss (e.g. anthropic/claude-opus-4.8)")
    ap.add_argument("--image", default=None, help="optional reference image path")
    ap.add_argument("--json", action="store_true", help="print the plan JSON as the LAST line")
    a = ap.parse_args()

    settings = Settings.load()
    if a.model:
        settings.model = a.model.split("/", 1)[-1]
    ctx = make_run_context(a.prompt, a.out)
    os.makedirs(ctx.run_dir, exist_ok=True)
    plan_path = os.path.join(ctx.run_dir, "subassembly_plan.json")
    print(f"[boss] model: {settings.model}")
    print(f"[boss] machine: {a.prompt}")
    try:
        plan = plan_machine(a.prompt, settings, image_path=a.image,
                            plan_json_path=plan_path, log_fn=print)
    except BossError as e:
        print(f"[boss] FAILED: {e}")
        return 1
    print("-" * 56)
    print(f"RESULT: {len(plan.subassemblies)} subassemblies, {len(plan.seams)} seams "
          f"-> {plan_path}")
    if a.json:
        print("PLAN_JSON:" + json.dumps(plan_to_dict(plan)))
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
