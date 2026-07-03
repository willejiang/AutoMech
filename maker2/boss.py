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
from .prompts.boss_prompt import (BOSS_SYSTEM, build_boss_feedback,
                                  build_boss_json_from_notes, build_boss_repair,
                                  build_boss_user)
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
        link=str(d.get("link") or ""),
        role=role,
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
    )


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
            "axis": list(fr.axis), "link": fr.link, "role": fr.role}


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


def frame_contract_for(plan: SubassemblyPlan, sub_id: str) -> FrameContract:
    """Build the FrameContract Stage B hands to one subassembly's manager."""
    sub = plan.sub_by_id(sub_id)
    if sub is None:
        raise BossError(f"no subassembly '{sub_id}' in plan")
    return FrameContract(
        sub_id=sub.id,
        frames=list(sub.frames),
        global_origin_note=plan.global_origin_note,
        input_tags=list(sub.input_tags),
        output_tags=list(sub.output_tags),
        neighbors=[{"id": o.id, "function": o.function, "brief": o.brief}
                   for o in plan.subassemblies if o.id != sub_id],
    )


# --------------------------------------------------------------------------- #
# Top-level planning (LLM + repair loop) — mirrors manager.decompose
# --------------------------------------------------------------------------- #

def plan_machine(product_prompt: str, settings, *, image_path: str | None = None,
                 plan_json_path: str | None = None,
                 feedback: str | None = None,
                 log_fn=None) -> SubassemblyPlan:
    """Split a machine prompt into a validated, persisted SubassemblyPlan.

    ``feedback`` (a later loop pass) delivers an interface/assembly-level fault and
    asks the boss to re-plan. On truncation the boss is asked for FEWER, larger
    subassemblies; on parse/validation failure the error is fed back as a repair
    request, bounded by ``settings.manager_retries``. Raises BossError if no attempt
    yields a valid plan.
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
    if feedback:
        conv.add_user_message(build_boss_feedback(feedback))
        if log_fn:
            log_fn("[boss] re-planning from an interface/assembly fault")

    # Scratch memory: the boss writes its plan as NOTES first (saved here) so a JSON
    # cut can regenerate from the plan instead of shrinking it. Next to the plan file.
    from pathlib import Path
    memory_path = (str(Path(plan_json_path).parent / "boss_memory.md")
                   if plan_json_path else None)

    last_err = ""
    attempts = settings.manager_retries + 1
    for attempt in range(1, attempts + 1):
        if log_fn:
            log_fn(f"[boss] attempt {attempt}/{attempts}: planning subassemblies "
                   f"(streaming)…")
        # stream_two_part streams the NOTES-then-JSON response and RECOVERS a cap cut
        # without shrinking: continue the notes if cut mid-notes, else regenerate the
        # JSON from the saved notes. So a truncation is no longer a failure here — any
        # remaining error below is a genuine content/validation error to repair.
        try:
            text = stream_two_part(client, conv, BOSS_SYSTEM,
                                   memory_path=memory_path,
                                   regen_msg_fn=build_boss_json_from_notes,
                                   log_fn=log_fn, tag="boss")
        except LLMError as e:
            raise BossError(f"Boss LLM request failed: {e}") from e
        conv.add_assistant_message(text)
        try:
            plan = parse_plan(text)
            _validate_plan(plan)
        except (ValueError, BossError, json.JSONDecodeError) as e:
            last_err = str(e)
            if log_fn:
                log_fn(f"[boss] attempt {attempt}/{attempts} rejected: {last_err}")
            conv.add_user_message(build_boss_repair(last_err))
            continue
        if plan_json_path:
            save_plan(plan, plan_json_path)
        if log_fn:
            log_fn(f"[boss] OK on attempt {attempt}: {len(plan.subassemblies)} "
                   f"subassemblies, {len(plan.seams)} seams, root='{plan.root_sub}'")
        return plan

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
