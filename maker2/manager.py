"""Manager agent: one product prompt -> a validated KinematicModel.

The manager is the only component that owns geometry RELATIONSHIPS. It asks the
LLM for a plain-JSON decomposition (links + joints), parses it into the dataclass
contract, then runs a pure-Python validation pass that the LLM cannot be trusted
to satisfy on its own: URDF-safe unique names, a single connected tree (one root,
no cycles, no orphans), valid joint types, and sane limits for articulated joints.

Validation MUTATES the model into a normalized, URDF-safe form (slugified/deduped
names propagated into joint endpoints; root_link inferred; mesh_filename assigned)
and raises ManagerError listing every problem at once. On failure the error text
is fed back to the LLM as a repair request, bounded by Settings.manager_retries.

We use plain JSON rather than native tool-calling because the local gateway's
tool support is unverified; a single JSON object is trivial to validate + repair.
"""

from __future__ import annotations

import json
import re

from .imageutil import ImageLoadError, load_image_block
from .jsonutil import extract_json_object
from .llm.client import LLMError
from .llm.conversation import Conversation
from .model import JointSpec, KinematicModel, LinkSpec
from .prompts.manager_prompt import (MANAGER_SYSTEM,
                                     build_manager_evaluator_feedback,
                                     build_manager_json_from_notes,
                                     build_manager_prior_model,
                                     build_manager_refine,
                                     build_manager_repair,
                                     build_manager_subassembly, build_manager_user)
from .twophase import stream_two_part


_VALID_JOINT_TYPES = {"fixed", "revolute", "prismatic", "continuous"}
_NEEDS_LIMITS = {"revolute", "prismatic"}
_NEEDS_AXIS = {"revolute", "prismatic", "continuous"}
_URDF_SAFE = re.compile(r"^[a-z][a-z0-9_]*$")


class ManagerError(RuntimeError):
    """The manager could not produce a valid model (parse or validation)."""


# --------------------------------------------------------------------------- #
# Dataclass parsing
# --------------------------------------------------------------------------- #

def _as_tuple3(value, default: tuple) -> tuple:
    if value is None:
        return default
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"expected a 3-number list, got {value!r}")
    return tuple(float(x) for x in value)


def _opt_float(value):
    return None if value is None else float(value)


def _parse_color(value) -> tuple:
    """Coerce a manager-supplied color to an RGBA tuple in 0..1, else ().

    Accepts [r,g,b] or [r,g,b,a]; values >1 are treated as 0..255 and scaled.
    A bad/missing value yields () so the URDF builder falls back to its palette.
    """
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return ()
    try:
        nums = [float(x) for x in value[:4]]
    except (TypeError, ValueError):
        return ()
    if any(n > 1.0 for n in nums):
        nums = [n / 255.0 for n in nums]
    if len(nums) == 3:
        nums.append(1.0)
    return tuple(min(1.0, max(0.0, n)) for n in nums)


def _link_from_dict(d: dict, idx: int) -> LinkSpec:
    if not isinstance(d, dict):
        raise ValueError(f"links[{idx}] is not an object")
    name = d.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"links[{idx}] is missing a non-empty 'name'")
    desc = d.get("description") or ""
    size = d.get("size_mm") or {}
    if not isinstance(size, dict):
        raise ValueError(f"links[{idx}] '{name}': size_mm must be an object")
    return LinkSpec(
        name=name.strip(),
        description=str(desc),
        shape_hint=str(d.get("shape_hint") or ""),
        size_mm={str(k): v for k, v in size.items()},
        origin_note=str(d.get("origin_note") or ""),
        color=_parse_color(d.get("color")),
    )


def _joint_from_dict(d: dict, idx: int) -> JointSpec:
    if not isinstance(d, dict):
        raise ValueError(f"joints[{idx}] is not an object")
    name = d.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"joints[{idx}] is missing a non-empty 'name'")
    jtype = d.get("type")
    if not isinstance(jtype, str) or not jtype.strip():
        raise ValueError(f"joints[{idx}] '{name}' is missing 'type'")
    parent = d.get("parent")
    child = d.get("child")
    if not isinstance(parent, str) or not parent.strip():
        raise ValueError(f"joints[{idx}] '{name}' is missing 'parent'")
    if not isinstance(child, str) or not child.strip():
        raise ValueError(f"joints[{idx}] '{name}' is missing 'child'")
    return JointSpec(
        name=name.strip(),
        type=jtype.strip().lower(),
        parent=parent.strip(),
        child=child.strip(),
        xyz_m=_as_tuple3(d.get("xyz_m"), (0.0, 0.0, 0.0)),
        rpy_rad=_as_tuple3(d.get("rpy_rad"), (0.0, 0.0, 0.0)),
        axis=_as_tuple3(d.get("axis"), (0.0, 0.0, 1.0)),
        lower=_opt_float(d.get("lower")),
        upper=_opt_float(d.get("upper")),
        effort=float(d.get("effort", 10.0)),
        velocity=float(d.get("velocity", 1.0)),
        driver=bool(d.get("driver", False)),
    )


def parse_model(text: str) -> KinematicModel:
    """Parse an LLM response into a (not-yet-validated) KinematicModel."""
    obj = json.loads(extract_json_object(text))
    if not isinstance(obj, dict):
        raise ValueError("top-level JSON value is not an object")
    links = obj.get("links")
    joints = obj.get("joints")
    if not isinstance(links, list) or not links:
        raise ValueError("'links' must be a non-empty array")
    if not isinstance(joints, list):
        raise ValueError("'joints' must be an array")
    return KinematicModel(
        name=str(obj.get("name") or "product"),
        root_link=str(obj.get("root_link") or ""),
        links=[_link_from_dict(d, i) for i, d in enumerate(links)],
        joints=[_joint_from_dict(d, i) for i, d in enumerate(joints)],
    )


def parse_frames_realized(text: str) -> list[dict]:
    """Pull the optional `frames_realized` block out of a subassembly manager's
    response (the SAME JSON object that carries links/joints). Each entry maps a
    contract frame name to the real link the manager put there + the link-local
    offset of that frame. Returns [] if absent/malformed — the caller (the boss
    orchestrator) machine-checks it separately; parse_model ignores this key."""
    try:
        obj = json.loads(extract_json_object(text))
    except (ValueError, json.JSONDecodeError):
        return []
    fr = obj.get("frames_realized")
    if not isinstance(fr, list):
        return []
    out: list[dict] = []
    for e in fr:
        if not isinstance(e, dict):
            continue
        name = e.get("frame")
        link = e.get("link")
        if not isinstance(name, str) or not isinstance(link, str):
            continue
        out.append({
            "frame": name.strip(),
            "link": link.strip(),
            "local_xyz_m": _as_tuple3(e.get("local_xyz_m"), (0.0, 0.0, 0.0)),
            "local_rpy_rad": _as_tuple3(e.get("local_rpy_rad"), (0.0, 0.0, 0.0)),
        })
    return out


# --------------------------------------------------------------------------- #
# Normalization + tree validation
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


def _validate_model(model: KinematicModel) -> None:
    """Normalize names + verify a single connected tree. Raises on problems.

    Mutates ``model`` in place: link names are slugified/deduped and the new
    names are propagated into joint parent/child; joint names are slugified/
    deduped; on success ``root_link`` is set and each link's ``mesh_filename``
    is assigned. All detected problems are collected and raised together so the
    repair prompt can fix everything in one round-trip.
    """
    problems: list[str] = []

    # 1. Normalize link names, building old -> new remap.
    link_remap: dict[str, str] = {}
    used_links: set[str] = set()
    for link in model.links:
        slug = _dedupe(_slugify(link.name, "link"), used_links)
        used_links.add(slug)
        link_remap[link.name] = slug
        link.name = slug
    valid_links = set(used_links)

    # 2. Remap joint endpoints through the same table; normalize joint names.
    used_joints: set[str] = set()
    for joint in model.joints:
        slug = _dedupe(_slugify(joint.name, "joint"), used_joints)
        used_joints.add(slug)
        joint.name = slug
        joint.parent = link_remap.get(joint.parent, _slugify(joint.parent, "link"))
        joint.child = link_remap.get(joint.child, _slugify(joint.child, "link"))

    # 3. Endpoints must reference real links; no self-loops.
    for j in model.joints:
        if j.parent not in valid_links:
            problems.append(f"joint '{j.name}' references unknown parent '{j.parent}'")
        if j.child not in valid_links:
            problems.append(f"joint '{j.name}' references unknown child '{j.child}'")
        if j.parent == j.child:
            problems.append(f"joint '{j.name}' connects link '{j.parent}' to itself")

    # 4. Joint types + articulated-joint requirements.
    for j in model.joints:
        if j.type not in _VALID_JOINT_TYPES:
            problems.append(
                f"joint '{j.name}' has invalid type '{j.type}' "
                f"(expected one of {sorted(_VALID_JOINT_TYPES)})")
            continue
        if j.type in _NEEDS_AXIS and tuple(j.axis) == (0.0, 0.0, 0.0):
            problems.append(f"joint '{j.name}' ({j.type}) needs a non-zero axis")
        if j.type in _NEEDS_LIMITS:
            if j.lower is None or j.upper is None:
                problems.append(
                    f"joint '{j.name}' ({j.type}) needs both 'lower' and 'upper'")
            elif j.lower >= j.upper:
                problems.append(
                    f"joint '{j.name}' ({j.type}) needs lower < upper "
                    f"(got {j.lower} >= {j.upper})")

    # 5. Each non-root link is the child of exactly one joint.
    child_counts: dict[str, int] = {}
    for j in model.joints:
        if j.child in valid_links:
            child_counts[j.child] = child_counts.get(j.child, 0) + 1
    for name, count in child_counts.items():
        if count > 1:
            problems.append(f"link '{name}' is the child of {count} joints (must be 1)")

    # 6. Exactly one root (a link that is never any joint's child).
    roots = sorted(valid_links - set(child_counts))
    if len(roots) == 0:
        problems.append("no root link -- every link is a child (a cycle, not a tree)")
    elif len(roots) > 1:
        problems.append(
            f"multiple root links {roots} -- the model is a forest, not one tree")

    # 7. DFS from the single root: catch orphans + cycles.
    if len(roots) == 1:
        adjacency: dict[str, list[str]] = {}
        for j in model.joints:
            adjacency.setdefault(j.parent, []).append(j.child)
        visited: set[str] = set()
        stack = [roots[0]]
        while stack:
            node = stack.pop()
            if node in visited:
                problems.append(f"cycle detected at link '{node}'")
                continue
            visited.add(node)
            stack.extend(adjacency.get(node, []))
        orphans = sorted(valid_links - visited)
        if orphans:
            problems.append(f"links not connected to the root: {orphans}")

    if problems:
        raise ManagerError("Model validation failed:\n- " + "\n- ".join(problems))

    # Success: finalize the normalized model.
    model.root_link = roots[0]
    for link in model.links:
        link.mesh_filename = f"meshes/{link.name}.stl"


# --------------------------------------------------------------------------- #
# Persistence
# --------------------------------------------------------------------------- #

def model_to_dict(model: KinematicModel) -> dict:
    """Serialize a KinematicModel to the same JSON shape the manager emits."""
    return {
        "name": model.name,
        "root_link": model.root_link,
        "links": [
            {
                "name": l.name,
                "description": l.description,
                "shape_hint": l.shape_hint,
                "size_mm": l.size_mm,
                "origin_note": l.origin_note,
                "color": list(l.color),
                "mesh_filename": l.mesh_filename,
            }
            for l in model.links
        ],
        "joints": [
            {
                "name": j.name,
                "type": j.type,
                "parent": j.parent,
                "child": j.child,
                "xyz_m": list(j.xyz_m),
                "rpy_rad": list(j.rpy_rad),
                "axis": list(j.axis),
                "lower": j.lower,
                "upper": j.upper,
                "effort": j.effort,
                "velocity": j.velocity,
                "driver": j.driver,
            }
            for j in model.joints
        ],
    }


def save_model(model: KinematicModel, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(model_to_dict(model), f, indent=2)


def load_model(path: str) -> KinematicModel:
    with open(path, "r", encoding="utf-8") as f:
        return parse_model(f.read())


# --------------------------------------------------------------------------- #
# Top-level decomposition (LLM + repair loop)
# --------------------------------------------------------------------------- #

def decompose(product_prompt: str, settings, *, image_path: str | None = None,
              model_json_path: str | None = None,
              evaluator_feedback: str | None = None,
              refine_message: str | None = None,
              prior_model_json: str | None = None,
              frame_contract=None,
              log_fn=None) -> KinematicModel:
    """Decompose a product prompt into a validated, persisted KinematicModel.

    If ``image_path`` is given, the image is attached to the manager's first user
    message and becomes the authoritative source (the text prompt is a hint).

    If ``evaluator_feedback`` is given (a later loop iteration), it is appended as
    a follow-up user message instructing the manager to regenerate strictly
    following that feedback. The MANAGER_SYSTEM prompt already tells the manager
    to obey the evaluator, so this just delivers the verdict.

    Sends the manager prompt, parses + validates, and on any parse/validation
    failure feeds the error back as a repair request — bounded by
    ``settings.manager_retries`` (so initial + retries attempts total). Raises
    ManagerError if no attempt yields a valid model.
    """
    client = settings.manager_client()
    conv = Conversation()
    try:
        images = [load_image_block(image_path)] if image_path else None
    except ImageLoadError as e:
        raise ManagerError(str(e)) from e
    conv.add_user_message(
        build_manager_user(product_prompt, has_image=bool(image_path)),
        images=images)
    if image_path and log_fn:
        log_fn(f"[manager] using input image: {image_path}")
    if evaluator_feedback:
        conv.add_user_message(build_manager_evaluator_feedback(evaluator_feedback))
        if log_fn:
            log_fn("[manager] applying evaluator feedback from the previous "
                   "iteration")
    # Multi-turn refine: show the prior model, then the user's requested change.
    if prior_model_json:
        conv.add_user_message(build_manager_prior_model(prior_model_json))
    if refine_message:
        conv.add_user_message(build_manager_refine(refine_message))
        if log_fn:
            log_fn(f"[manager] applying user refine request: {refine_message[:80]}")
    # Hierarchy: build ONE subassembly under the boss's interface/frame contract.
    if frame_contract is not None:
        conv.add_user_message(build_manager_subassembly(frame_contract))
        if log_fn:
            log_fn(f"[manager] subassembly '{getattr(frame_contract, 'sub_id', '?')}' "
                   f"with {len(getattr(frame_contract, 'frames', []))} interface frame(s)")

    # Optional web-search research pre-step (gated by settings.enable_reference_tools):
    # look up standard dimensions / part specs before decomposing.
    from .tools import maybe_research
    maybe_research(client, conv, settings,
                   f"decompose into parts: {product_prompt}", log_fn=log_fn)

    # Scratch memory: the manager writes its decomposition as NOTES first (saved
    # here) so a JSON cut can regenerate from the notes instead of dropping parts.
    # Next to the model file; tagged by sub id in hierarchy mode.
    from pathlib import Path
    memory_path = (str(Path(model_json_path).parent / "manager_memory.md")
                   if model_json_path else None)
    tag = (f"sub:{getattr(frame_contract, 'sub_id', '?')}"
           if frame_contract is not None else "manager")

    last_err = ""
    attempts = settings.manager_retries + 1
    for attempt in range(1, attempts + 1):
        if log_fn:
            log_fn(f"[manager] attempt {attempt}/{attempts}: decomposing (streaming)…")
        # stream_two_part streams the NOTES-then-JSON response and RECOVERS a cap cut
        # WITHOUT shrinking: continue the notes if cut mid-notes, else regenerate the
        # JSON from the saved notes (so no shafts/bearings get dropped to fit). A
        # truncation is no longer a failure here — any error below is a content error.
        try:
            text = stream_two_part(client, conv, MANAGER_SYSTEM,
                                   memory_path=memory_path,
                                   regen_msg_fn=build_manager_json_from_notes,
                                   log_fn=log_fn, tag=tag)
        except LLMError as e:
            # send() failures (connection, HTTP, empty) aren't fixed by re-sending the
            # identical prompt, so fail fast with a clear message.
            raise ManagerError(f"Manager LLM request failed: {e}") from e
        conv.add_assistant_message(text)
        try:
            model = parse_model(text)
            _validate_model(model)
        except (ValueError, ManagerError, json.JSONDecodeError) as e:
            last_err = str(e)
            if log_fn:
                log_fn(f"[manager] attempt {attempt}/{attempts} rejected: {last_err}")
            conv.add_user_message(build_manager_repair(last_err))
            continue
        if model_json_path:
            save_model(model, model_json_path)
        # Hierarchy: stash the manager's realized interface-frame placements on the
        # model (a side-channel the boss orchestrator reads; not part of the URDF
        # contract, so parse_model/_validate_model ignore it).
        if frame_contract is not None:
            model.frames_realized = parse_frames_realized(text)
            if log_fn:
                log_fn(f"[manager] realized {len(model.frames_realized)} interface "
                       f"frame(s)")
        if log_fn:
            log_fn(f"[manager] OK on attempt {attempt}: "
                   f"{len(model.links)} links, {len(model.joints)} joints, "
                   f"root='{model.root_link}'")
        return model

    raise ManagerError(
        f"Manager failed after {attempts} attempts. Last error:\n{last_err}")


# --------------------------------------------------------------------------- #
# Item 4b — Claude-Code-style minimal editing at the MODEL level:
#   should_rebuild : does this sub even need to change for the fault? (skip if not)
#   decompose_patch: return a STRUCTURED PATCH (add/modify/remove links+joints) against
#                    the prior model, so unchanged parts are kept and only the delta is
#                    rebuilt.
# --------------------------------------------------------------------------- #

def should_rebuild(prior_model_json: str, fault_reason: str, settings,
                   *, frame_contract=None, log_fn=None) -> bool:
    """Ask the manager (cheaply) whether THIS subassembly must change to fix the fault,
    or is unrelated and should be KEPT as-is. Returns True to rebuild, False to keep.
    Defaults to True (rebuild) on any error — never skip on uncertainty."""
    from .prompts.manager_prompt import build_manager_should_rebuild
    try:
        client = settings.manager_client()
        conv = Conversation()
        conv.add_user_message(build_manager_should_rebuild(prior_model_json, fault_reason,
                                                           frame_contract))
        text, _ = client.send_collect(conv.get_messages_for_api(api_style=client.api_style),
                                      system=MANAGER_SYSTEM)
        verdict = (text or "").strip().upper()
        keep = verdict.startswith("KEEP") or "KEEP" in verdict.split()[:3]
        if log_fn:
            log_fn(f"[manager] skip-check: {'KEEP (reuse)' if keep else 'REBUILD'}")
        return not keep
    except Exception as e:
        if log_fn:
            log_fn(f"[manager] skip-check failed ({e}); rebuilding")
        return True


# Structured patch: add/modify/remove links and joints against a prior model.
MODEL_PATCH_SCHEMA = {
    "add_links": "list[LinkSpec]", "modify_links": "list[LinkSpec]",
    "remove_links": "list[str]", "add_joints": "list[JointSpec]",
    "modify_joints": "list[JointSpec]", "remove_joints": "list[str]",
}


def parse_patch(text: str) -> dict:
    """Parse the manager's minimal PATCH JSON into typed add/modify/remove sets."""
    obj = json.loads(extract_json_object(text))
    return {
        "add_links": [_link_from_dict(d, i) for i, d in enumerate(obj.get("add_links") or [])],
        "modify_links": [_link_from_dict(d, i) for i, d in enumerate(obj.get("modify_links") or [])],
        "remove_links": [str(n) for n in (obj.get("remove_links") or [])],
        "add_joints": [_joint_from_dict(d, i) for i, d in enumerate(obj.get("add_joints") or [])],
        "modify_joints": [_joint_from_dict(d, i) for i, d in enumerate(obj.get("modify_joints") or [])],
        "remove_joints": [str(n) for n in (obj.get("remove_joints") or [])],
    }


def apply_patch(prior: KinematicModel, patch: dict) -> tuple[KinematicModel, set, dict]:
    """Apply a patch to a prior model deterministically. Returns (new_model,
    changed_link_names, patch_meta). ``changed`` = added + modified links (the ONLY
    links the worker must (re)build; unchanged links keep their prior STLs).
    ``patch_meta`` splits them so the worker can EDIT a modified part's existing script
    (item 2b) vs freshly generate an added one: {"modify": {names}, "add": {names}}.
    Then _validate_model."""
    links = {l.name: l for l in prior.links}
    joints = {j.name: j for j in prior.joints}
    add_names: set = {l.name for l in patch.get("add_links", [])}
    modify_names: set = {l.name for l in patch.get("modify_links", [])}
    changed: set = set()
    for l in patch.get("add_links", []) + patch.get("modify_links", []):
        links[l.name] = l
        changed.add(l.name)
    for n in patch.get("remove_links", []):
        links.pop(n, None)
    for j in patch.get("add_joints", []) + patch.get("modify_joints", []):
        joints[j.name] = j
    for n in patch.get("remove_joints", []):
        joints.pop(n, None)
    new = KinematicModel(name=prior.name, root_link=prior.root_link,
                         links=list(links.values()), joints=list(joints.values()))
    _validate_model(new)                 # normalize + enforce single-tree invariant
    # A validated model may have renamed/dropped a link; keep only changed links that
    # survived validation.
    survivors = {l.name for l in new.links}
    changed &= survivors
    meta = {"modify": modify_names & survivors, "add": add_names & survivors}
    return new, changed, meta


def decompose_patch(prior_model_json: str, fault_reason: str, settings,
                    *, frame_contract=None, model_json_path=None,
                    log_fn=None) -> tuple[KinematicModel, set, dict]:
    """Return a MINIMAL patched model + the set of changed link names + patch_meta,
    given the prior model + the exact fault. The manager changes as FEW parts as
    possible (Claude-Code style). ``patch_meta`` = {"modify": {names}, "add": {names},
    "fault_by_link": {link: fault}} so the worker can EDIT a modified part's existing
    script vs freshly generate an added one (item 2b). Falls back to a full decompose if
    the patch can't be produced."""
    from .prompts.manager_prompt import build_manager_patch
    prior = parse_model(prior_model_json)
    client = settings.manager_client()
    attempts = settings.manager_retries + 1
    conv = Conversation()
    conv.add_user_message(build_manager_patch(prior_model_json, fault_reason,
                                              frame_contract))
    last_err = ""
    for attempt in range(1, attempts + 1):
        try:
            text, _ = client.send_collect(
                conv.get_messages_for_api(api_style=client.api_style),
                system=MANAGER_SYSTEM)
            patch = parse_patch(text)
            model, changed, meta = apply_patch(prior, patch)
        except (LLMError, ValueError, ManagerError, json.JSONDecodeError) as e:
            last_err = str(e)
            if log_fn:
                log_fn(f"[manager] patch attempt {attempt}/{attempts} rejected: {last_err}")
            conv.add_user_message(build_manager_repair(last_err))
            continue
        if model_json_path:
            save_model(model, model_json_path)
        if frame_contract is not None:
            model.frames_realized = parse_frames_realized(text)
        # The manager doesn't emit a per-link fault; attribute the whole fault to each
        # changed link so a per-part edit (2b) still has the "what's wrong" context.
        meta["fault_by_link"] = {name: fault_reason for name in changed}
        if log_fn:
            log_fn(f"[manager] patched: {len(changed)} link(s) changed "
                   f"(modify={sorted(meta['modify'])}, add={sorted(meta['add'])}), "
                   f"rest kept")
        return model, changed, meta
    raise ManagerError(f"Manager patch failed after {attempts} attempts: {last_err}")

