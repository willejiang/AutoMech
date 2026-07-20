"""Manager agent: one product prompt -> a validated KinematicModel.

The manager is the only component that owns geometry RELATIONSHIPS. It asks the
LLM for a plain-JSON decomposition (links + parent-relative POSES, plus each
link's DOF), parses it into the dataclass contract, then runs a pure-Python
normalization pass that the LLM cannot be trusted to satisfy on its own:
URDF-safe unique names + mesh_filename assignment.

Pure contact (maker2-mujoco-contact): there are NO motors and NO joints between
parts. The manager emits parts + relative poses; motion is a property of each
part (LinkSpec.dof: fixed|spin|free) and transmission happens by real tooth
contact in MuJoCo. So the model is legitimately a FOREST — the old single-tree/
one-root/no-cycle validation is gone; only slug/dedup + weak name-existence
checks remain.

Validation MUTATES the model into a normalized, URDF-safe form (slugified/deduped
names propagated into pose endpoints + mesh_pairs; root_link inferred;
mesh_filename assigned) and raises ManagerError listing every problem at once. On
failure the error text is fed back to the LLM as a repair request, bounded by
Settings.manager_retries.

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
from .model import KinematicModel, LinkSpec, PoseSpec
from .prompts.manager_prompt import (MANAGER_SYSTEM, manager_system,
                                     build_manager_coarser,
                                     build_manager_evaluator_feedback,
                                     build_manager_json_from_notes,
                                     build_manager_prior_model,
                                     build_manager_refine,
                                     build_manager_repair,
                                     build_manager_repair_diff,
                                     build_manager_subassembly, build_manager_user)
from .twophase import stream_two_part


_VALID_DOF = {"fixed", "spin", "free"}
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


def _canonicalize_link_axis(link: LinkSpec) -> LinkSpec:
    """Round/axial geometry, inferred ports and motion share one local +Z truth."""
    from .mate_solver import is_axial_part
    if is_axial_part(link):
        link.spin_axis = (0.0, 0.0, 1.0)
    return link


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
    dof = str(d.get("dof") or "fixed").strip().lower()
    if dof not in _VALID_DOF:
        raise ValueError(f"links[{idx}] '{name}': dof '{dof}' invalid "
                         f"(expected one of {sorted(_VALID_DOF)})")
    return _canonicalize_link_axis(LinkSpec(
        name=name.strip(),
        description=str(desc),
        shape_hint=str(d.get("shape_hint") or ""),
        size_mm={str(k): v for k, v in size.items()},
        origin_note=str(d.get("origin_note") or ""),
        color=_parse_color(d.get("color")),
        dof=dof,
        spin_axis=_as_tuple3(d.get("spin_axis"), (0.0, 0.0, 1.0)),
        driver=bool(d.get("driver", False)),
        material=str(d.get("material") or "steel").strip().lower() or "steel",
    ))


def _pose_from_dict(d: dict, idx: int) -> PoseSpec:
    if not isinstance(d, dict):
        raise ValueError(f"poses[{idx}] is not an object")
    name = d.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"poses[{idx}] is missing a non-empty 'name'")
    child = d.get("child")
    if not isinstance(child, str) or not child.strip():
        raise ValueError(f"poses[{idx}] '{name}' is missing 'child'")
    parent = d.get("parent")
    parent = parent.strip() if isinstance(parent, str) else ""
    return PoseSpec(
        name=name.strip(),
        parent=parent,
        child=child.strip(),
        xyz_m=_as_tuple3(d.get("xyz_m"), (0.0, 0.0, 0.0)),
        rpy_rad=_as_tuple3(d.get("rpy_rad"), (0.0, 0.0, 0.0)),
    )


def _mesh_pairs_from(obj: dict) -> list:
    """Parse the optional `mesh_pairs` list: [[drive, driven], ...] -> [(str,str)]."""
    raw = obj.get("mesh_pairs") or []
    out: list = []
    if isinstance(raw, list):
        for e in raw:
            if isinstance(e, (list, tuple)) and len(e) == 2:
                a, b = e
                if isinstance(a, str) and isinstance(b, str):
                    out.append((a.strip(), b.strip()))
    return out


def parse_model(text: str) -> KinematicModel:
    """Parse an LLM response into a (not-yet-validated) KinematicModel.

    Pure-contact contract: `links` (with dof) + `poses` (parent-relative placements)
    + optional `mesh_pairs`. Accepts the legacy `joints` key as an alias for `poses`
    so a saved pre-migration model still loads (its joint types collapse to poses)."""
    obj = json.loads(extract_json_object(text))
    if not isinstance(obj, dict):
        raise ValueError("top-level JSON value is not an object")
    links = obj.get("links")
    poses = obj.get("poses")
    if poses is None:
        poses = obj.get("joints")            # legacy alias
    if not isinstance(links, list) or not links:
        raise ValueError("'links' must be a non-empty array")
    if not isinstance(poses, list):
        raise ValueError("'poses' must be an array")
    return KinematicModel(
        name=str(obj.get("name") or "product"),
        root_link=str(obj.get("root_link") or ""),
        links=[_link_from_dict(d, i) for i, d in enumerate(links)],
        poses=[_pose_from_dict(d, i) for i, d in enumerate(poses)],
        mesh_pairs=_mesh_pairs_from(obj),
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
    """Normalize names + weakly validate a pure-contact FOREST. Raises on problems.

    Mutates ``model`` in place: link names are slugified/deduped and the new names
    are propagated into pose parent/child, mesh_pairs, and root_link; pose names are
    slugified/deduped; each link's ``mesh_filename`` is assigned. Because a
    pure-contact model is legitimately a forest (parts placed by pose, motion by
    contact — NOT a single joint tree), the old single-root/no-cycle/orphan checks
    are gone. We only guard that every pose child/parent + root_link names a real
    link, and pick a sensible root_link if the manager didn't name one."""
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

    def _remap(name: str) -> str:
        return link_remap.get(name, _slugify(name, "link")) if name else ""

    # 2. Remap pose endpoints through the same table; normalize pose names.
    used_poses: set[str] = set()
    for pose in model.poses:
        slug = _dedupe(_slugify(pose.name, "pose"), used_poses)
        used_poses.add(slug)
        pose.name = slug
        pose.parent = _remap(pose.parent)
        pose.child = _remap(pose.child)

    # 3. Remap mesh_pairs + root_link through the same table.
    model.mesh_pairs = [(_remap(a), _remap(b)) for (a, b) in model.mesh_pairs]
    model.root_link = _remap(model.root_link)

    # 4. Weak guard: pose endpoints must reference real links; no self-loops.
    for p in model.poses:
        if p.parent and p.parent not in valid_links:
            problems.append(f"pose '{p.name}' references unknown parent '{p.parent}'")
        if p.child not in valid_links:
            problems.append(f"pose '{p.name}' references unknown child '{p.child}'")
        if p.parent and p.parent == p.child:
            problems.append(f"pose '{p.name}' places link '{p.child}' relative to itself")

    if problems:
        raise ManagerError("Model validation failed:\n- " + "\n- ".join(problems))

    # Success: finalize. Pick a root_link if unset/invalid — prefer a link that is
    # never a pose child (a forest root); else the first link.
    if model.root_link not in valid_links:
        children = {p.child for p in model.poses}
        roots = [l.name for l in model.links if l.name not in children]
        model.root_link = roots[0] if roots else (model.links[0].name if model.links else "")
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
                "dof": l.dof,
                "spin_axis": list(l.spin_axis),
                "driver": l.driver,
                "material": l.material,
            }
            for l in model.links
        ],
        "poses": [
            {
                "name": p.name,
                "parent": p.parent,
                "child": p.child,
                "xyz_m": list(p.xyz_m),
                "rpy_rad": list(p.rpy_rad),
            }
            for p in model.poses
        ],
        "mesh_pairs": [list(pair) for pair in model.mesh_pairs],
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

    Track-0 refactor (see .claude/plans/precious-humming-wand.md Part E): the body is
    split into three OWNED seams so three parallel tracks each edit a separate function:
      * ``_manager_research``       — the optional web/KB research pre-step (Track 1).
      * ``_parse_manager_output``   — parse the LLM text into a KinematicModel (Track 2).
      * ``_decompose_loop``         — the attempt/retry control loop (Track 3).
    This driver only assembles the conversation and calls them; it is behavior-identical
    to the pre-split version.
    """
    client = settings.manager_client()
    conv = Conversation()
    try:
        images = [load_image_block(image_path)] if image_path else None
    except ImageLoadError as e:
        raise ManagerError(str(e)) from e
    conv.add_user_message(
        build_manager_user(product_prompt, has_image=bool(image_path),
                           manager_ir=getattr(settings, "manager_ir", True)),
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
        conv.add_user_message(build_manager_subassembly(
            frame_contract, manager_ir=getattr(settings, "manager_ir", True),
            manager_py=getattr(settings, "manager_py", False)))
        if log_fn:
            log_fn(f"[manager] subassembly '{getattr(frame_contract, 'sub_id', '?')}' "
                   f"with {len(getattr(frame_contract, 'frames', []))} interface frame(s)")

    # SEAM (Track 1): optional research pre-step.
    _manager_research(client, conv, settings, product_prompt, log_fn=log_fn)

    # Scratch memory path for two-phase cap-cut recovery (tagged by sub id in hierarchy).
    from pathlib import Path
    memory_path = (str(Path(model_json_path).parent / "manager_memory.md")
                   if model_json_path else None)
    tag = (f"sub:{getattr(frame_contract, 'sub_id', '?')}"
           if frame_contract is not None else "manager")

    # SEAM (Track 3): the attempt/retry control loop.
    return _decompose_loop(client, conv, settings, memory_path=memory_path, tag=tag,
                           model_json_path=model_json_path,
                           frame_contract=frame_contract, log_fn=log_fn)


def _manager_research(client, conv, settings, product_prompt, *, log_fn=None) -> None:
    """SEAM owned by Track 1 (RAG). Optional web-search / KB research pre-step, gated by
    settings.enable_reference_tools (web) and settings.enable_kb (local KB): look up
    standard dimensions / part specs and the output-format conventions, and inject
    findings into ``conv`` before decomposing. kb_search is pinned to the manager
    collection."""
    from .tools import maybe_research
    maybe_research(client, conv, settings,
                   f"decompose into parts: {product_prompt}",
                   collection="manager", log_fn=log_fn)


def _extract_python_block(text: str) -> str:
    """The single ```python code block from an LLM reply (方案B manager). Falls back to the
    whole text if the model omitted fences."""
    import re
    m = re.search(r"```(?:python)?\s*(.*?)```", text, re.S)
    return (m.group(1) if m else text).strip()


def _parse_manager_output(text: str, *, settings=None, frame_contract=None,
                          model_json_path=None, log_fn=None) -> KinematicModel:
    """Turn the manager's raw LLM text into a validated KinematicModel.

    Two authoring formats, selected by ``settings.manager_ir`` (default ON):

    * CONNECTION GRAPH (manager_ir on): a single JSON object of PARTS + MATES. Fed to
      ``mate_solver.solve_connection_graph``, which SOLVES every part's pose from the mates
      (no LLM-authored coordinates). See prompts/schema.py IR_SCHEMA_TEXT + Part A.
    * MJCF SKELETON (manager_ir off — the ``--no-manager-ir`` fallback): PARTS JSON, a line
      ``=== MJCF ===``, then an MJCF-style XML skeleton, split + fed to ``mjcf_skeleton_parser``.

    Either path produces a raw KinematicModel that we run through the EXISTING
    ``_validate_model`` (slug/dedup/mesh_filename/forest normalization), so the ENTIRE
    downstream pipeline is byte-identical regardless of authoring format.

    Raises ValueError/ManagerError/JSONDecodeError on bad content (caught by the loop):
    ``MateSolveError`` (connection graph) and ``SkeletonError`` / re-raised ``ET.ParseError``
    (skeleton) are all ValueError subclasses, so the loop's existing
    ``except (ValueError, ManagerError, json.JSONDecodeError)`` feeds the message back as a
    repair request with no change to the retry loop."""
    if getattr(settings, "manager_py", False):
        # 方案B: the manager authored a parametric CadQuery module. Extract the python code
        # block, evaluate it into a KinematicModel with GLOBAL poses, exporting each part's
        # STL along the way (this ABSORBS the worker step). Runs in the sub's run dir so
        # `import params` + meshes/ resolve. Frame realization is validated separately (the
        # authored assembly places parts in global coords; no declarative frames_realized).
        from pathlib import Path as _P

        from .py_manager import evaluate_manager_python, PyManagerError
        code = _extract_python_block(text)
        if not code:
            raise ManagerError("no ```python code block found; the manager must emit ONE "
                               "python block defining build_subassembly() -> cq.Assembly")
        run_dir = str(_P(model_json_path).parent) if model_json_path else "."
        sub_name = getattr(frame_contract, "sub_id", "") or "subassembly"
        params_text = getattr(frame_contract, "params_text", "") or ""
        try:
            model = evaluate_manager_python(code, run_dir, sub_name,
                                            params_text=params_text, log_fn=log_fn)
        except PyManagerError as e:
            raise ManagerError(str(e)) from e
        _validate_model(model)
        return model

    if getattr(settings, "manager_ir", True):
        from .mate_solver import solve_connection_graph_text
        model = solve_connection_graph_text(text)
        _validate_model(model)
        if frame_contract is not None and log_fn:
            log_fn(f"[manager] realized {len(model.frames_realized)} interface frame(s)")
        return model

    import xml.etree.ElementTree as ET

    from .mjcf_skeleton import SkeletonError, mjcf_skeleton_parser
    from .prompts.schema import MJCF_SENTINEL

    if MJCF_SENTINEL not in text:
        raise SkeletonError(
            f"missing the `{MJCF_SENTINEL}` separator: emit the PARTS JSON object, then a "
            f"line with exactly `{MJCF_SENTINEL}`, then the MJCF XML skeleton")
    parts_block, mjcf_block = text.split(MJCF_SENTINEL, 1)
    mjcf_block = _slice_mjcf(mjcf_block)
    try:
        model = mjcf_skeleton_parser(mjcf_block, parts_block)
    except ET.ParseError as e:
        raise SkeletonError(f"the MJCF skeleton is not well-formed XML: {e}") from e
    _validate_model(model)
    if frame_contract is not None and log_fn:
        log_fn(f"[manager] realized {len(model.frames_realized)} interface frame(s)")
    return model


def _slice_mjcf(block: str) -> str:
    """Trim the MJCF half of the payload to just its XML: from the first ``<mujoco`` (or
    ``<worldbody`` if the wrapper is absent) through the matching close tag, dropping any
    stray prose or fences the model added around it. Returns the block unchanged if no
    recognizable root tag is present (letting the XML parser raise a clear error)."""
    for open_tag, close_tag in (("<mujoco", "</mujoco>"), ("<worldbody", "</worldbody>")):
        start = block.find(open_tag)
        if start == -1:
            continue
        end = block.rfind(close_tag)
        if end != -1:
            return block[start:end + len(close_tag)]
        return block[start:]
    return block.strip()


def _manager_gate_errors(model, frame_contract, *, manager_py=False):
    """The PURE-PYTHON deterministic gate errors for a parsed model, used to (a) decide
    whether an attempt is clean and (b) feed badness. No render, no LLM. Mirrors the
    orchestrator's manager-gate block (schema + connectivity/overlap + frames-realized +
    frame-drift) so the loop's notion of 'good' matches the gate that will judge the sub.
    Imported lazily: benchmarks pulls in assembler/precheck (numpy), which we don't want at
    manager import time, and it keeps manager usable without the benchmark deps."""
    if manager_py:
        # 方案B: the structural gates (schema/connectivity/frame-realization) validate the
        # connection-graph JSON contract that a CadQuery-authoring manager no longer produces.
        # These are the stage-2 dismantle targets; geometric correctness is enforced by the
        # evaluator (non-empty solids, built geometry) + the downstream precheck instead.
        return []
    try:
        from .benchmarks.schema_gate import manager_schema_gate
        from .benchmarks.manager_gate import manager_gate, frame_drift_errors
        from .benchmarks import GateError as _GateError
        from .assembler import _gear_pitch_r_mm
    except Exception:
        return []
    errs = manager_schema_gate(model)
    if frame_contract is not None:
        frames = [{"frame": e.get("frame"), "link": e.get("link")}
                  for e in (getattr(model, "frames_realized", []) or [])]
        errs += manager_gate(model, frames, frame_contract)
        realized = {e.get("frame") for e in (getattr(model, "frames_realized", []) or [])}
        for fr in getattr(frame_contract, "frames", []) or []:
            if fr.name not in realized:
                errs.append(_GateError("manager", "ERR_FRAME_UNREALIZED",
                                       f"interface frame '{fr.name}' is not realized", fr.name))
        # A MESH frame must be realized ON A GEAR link (a part with a pitch radius). A mesh
        # frame that is missing, or realized on the shaft/bearing/a plain cylinder, leaves the
        # assembler unable to find the gear for that seam — the two-gears-on-one-shaft case
        # then cannot be resolved. Enforce it here so the manager fixes it, not the assembler.
        realized_link = {e.get("frame"): e.get("link")
                         for e in (getattr(model, "frames_realized", []) or [])}
        links_by_name = {l.name: l for l in (getattr(model, "links", []) or [])}
        for fr in getattr(frame_contract, "frames", []) or []:
            if getattr(fr, "role", "") != "mesh":
                continue
            lk = links_by_name.get(realized_link.get(fr.name))
            if lk is None or not _gear_pitch_r_mm(lk):
                errs.append(_GateError(
                    "manager", "ERR_MESH_FRAME_NOT_ON_GEAR",
                    f"mesh frame '{fr.name}' must be realized on a GEAR part (with teeth / a "
                    f"pitch diameter); it is "
                    + ("not realized at all" if lk is None
                       else f"on non-gear link '{lk.name}'")
                    + " — realize it on the meshing gear so the assembler can pair the teeth.",
                    fr.name))
        errs += frame_drift_errors(model, frame_contract)
    else:
        errs += manager_gate(model, [], None)
    return errs


# Gate codes that are advisory only (do NOT count against a "clean" attempt): the pre-render
# overlap warning (unreliable on non-boxy parts) and the soft dimension-lock warning.
_NONBLOCKING_CODES = {"ERR_OVL", "ERR_DIM"}


def _decompose_loop(client, conv, settings, *, memory_path, tag, model_json_path=None,
                    frame_contract=None, log_fn=None) -> KinematicModel:
    """SEAM owned by Track 3 (badness keep-best + escalation). The attempt/retry control
    loop: stream a NOTES→payload response, parse via _parse_manager_output, then score the
    parsed model with the pure-Python gates + badness() (NO physics). MONOTONIC-IMPROVEMENT
    contract (Part C.bis):
      * a CLEAN attempt (no blocking gate errors) returns immediately (fast path).
      * otherwise KEEP the lowest-badness attempt so far; feed the manager DIFF-CARRYING
        feedback (build_manager_repair_diff) stating whether badness went up/down and which
        checks moved, so retry N+1 is guided, not blind.
      * at loop end, return the BEST attempt even if none was perfect — a partial improvement
        beats a fresh failure (raise only if nothing ever parsed).
      * on a PLATEAU (badness flat for _PLATEAU_K attempts), ESCALATE: ask for a coarser
        decomposition (build_manager_coarser) instead of repeating the same approach.
    Bounded by settings.manager_retries."""
    from .badness import badness_breakdown, format_delta

    attempts = settings.manager_retries + 1
    _mgr_ir = getattr(settings, "manager_ir", True)
    _mgr_py = getattr(settings, "manager_py", False)
    plateau_k = int(getattr(settings, "loop_plateau_k", 2))
    eps = 1e-3
    best_badness = float("inf")
    best_model: KinematicModel | None = None
    best_bd: dict = {}
    prev_bd: dict = {}
    last_err = ""
    stall = 0                              # consecutive attempts with no badness improvement

    for attempt in range(1, attempts + 1):
        if log_fn:
            log_fn(f"[manager] attempt {attempt}/{attempts}: decomposing (streaming)…")
        try:
            text = stream_two_part(client, conv, manager_system(_mgr_ir, _mgr_py),
                                   memory_path=memory_path,
                                   regen_msg_fn=lambda notes: build_manager_json_from_notes(
                                       notes, manager_ir=_mgr_ir, manager_py=_mgr_py),
                                   log_fn=log_fn, tag=tag)
        except LLMError as e:
            raise ManagerError(f"Manager LLM request failed: {e}") from e
        conv.add_assistant_message(text)

        # 1. Parse. A parse/validation failure yields no model to score -> feed the error
        #    back (with the last badness delta if we have one) and retry.
        try:
            model = _parse_manager_output(text, settings=settings,
                                          frame_contract=frame_contract,
                                          model_json_path=model_json_path, log_fn=log_fn)
        except (ValueError, ManagerError, json.JSONDecodeError) as e:
            last_err = str(e)
            if log_fn:
                log_fn(f"[manager] attempt {attempt}/{attempts} rejected (parse): {last_err}")
            note = format_delta(prev_bd, best_bd) if best_bd else "no scored attempt yet"
            conv.add_user_message(build_manager_repair_diff(last_err, note,
                                                            manager_ir=_mgr_ir))
            continue

        # 2. Score the parsed model with the pure gates + badness (no physics).
        gate_errs = _manager_gate_errors(model, frame_contract, manager_py=_mgr_py)
        blocking = [e for e in gate_errs if e.code not in _NONBLOCKING_CODES]
        cur_bd = badness_breakdown(model, gate_errs, context={"fc": frame_contract})
        cur_badness = cur_bd["total"]
        if log_fn:
            log_fn(f"[manager] attempt {attempt}: badness={cur_badness:.2f} "
                   f"({len(blocking)} blocking gate issue(s)) terms={cur_bd['terms']}")

        # 2a. CLEAN -> done. Return this model immediately (the fast path is preserved).
        if not blocking:
            if model_json_path:
                save_model(model, model_json_path)
            if log_fn:
                log_fn(f"[manager] OK on attempt {attempt}: {len(model.links)} links, "
                       f"{len(model.poses)} poses, root='{model.root_link}' "
                       f"(badness {cur_badness:.2f}, clean)")
            return model

        # 2b. Not clean -> keep-best on badness.
        if cur_badness < best_badness - eps:
            best_badness, best_model, best_bd = cur_badness, model, cur_bd
            stall = 0
            if log_fn:
                log_fn(f"[manager] attempt {attempt}: new best badness {cur_badness:.2f} "
                       "(kept)")
        else:
            stall += 1
            if log_fn:
                log_fn(f"[manager] attempt {attempt}: badness {cur_badness:.2f} did not beat "
                       f"best {best_badness:.2f} (revert to best; stall {stall}/{plateau_k})")

        last_err = ("automated checks still failing:\n"
                    + "\n".join(f"- {e}" for e in blocking[:12]))
        delta_note = format_delta(prev_bd, cur_bd)
        prev_bd = cur_bd

        # 2c. Escalate on plateau: switch approach (coarsen) instead of retrying the same way.
        if stall >= plateau_k and attempt < attempts:
            stall = 0
            if log_fn:
                log_fn(f"[manager] plateau -> escalate: requesting a COARSER decomposition")
            conv.add_user_message(build_manager_coarser(
                last_err + "\n\n(the previous approach is not converging — simplify.)"))
        else:
            conv.add_user_message(build_manager_repair_diff(last_err, delta_note,
                                                            manager_ir=_mgr_ir))

    # Loop exhausted. Return the best imperfect model if we have one; else fail.
    if best_model is not None:
        if model_json_path:
            save_model(best_model, model_json_path)
        if log_fn:
            log_fn(f"[manager] no clean attempt in {attempts}; returning BEST "
                   f"(badness {best_badness:.2f}) so downstream gets the closest model")
        return best_model
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


# Structured patch: add/modify/remove links and poses against a prior model.
MODEL_PATCH_SCHEMA = {
    "add_links": "list[LinkSpec]", "modify_links": "list[LinkSpec]",
    "remove_links": "list[str]", "add_poses": "list[PoseSpec]",
    "modify_poses": "list[PoseSpec]", "remove_poses": "list[str]",
}


def parse_patch(text: str) -> dict:
    """Parse the manager's minimal PATCH JSON into typed add/modify/remove sets.
    Accepts legacy `*_joints` keys as aliases for `*_poses`."""
    obj = json.loads(extract_json_object(text))

    def _poses(key_new, key_old):
        raw = obj.get(key_new)
        if raw is None:
            raw = obj.get(key_old) or []
        return [_pose_from_dict(d, i) for i, d in enumerate(raw or [])]

    return {
        "add_links": [_link_from_dict(d, i) for i, d in enumerate(obj.get("add_links") or [])],
        "modify_links": [_link_from_dict(d, i) for i, d in enumerate(obj.get("modify_links") or [])],
        "remove_links": [str(n) for n in (obj.get("remove_links") or [])],
        "add_poses": _poses("add_poses", "add_joints"),
        "modify_poses": _poses("modify_poses", "modify_joints"),
        "remove_poses": [str(n) for n in (obj.get("remove_poses") or obj.get("remove_joints") or [])],
    }


def apply_patch(prior: KinematicModel, patch: dict) -> tuple[KinematicModel, set, dict]:
    """Apply a patch to a prior model deterministically. Returns (new_model,
    changed_link_names, patch_meta). ``changed`` = added + modified links (the ONLY
    links the worker must (re)build; unchanged links keep their prior STLs).
    ``patch_meta`` splits them so the worker can EDIT a modified part's existing script
    (item 2b) vs freshly generate an added one: {"modify": {names}, "add": {names}}.
    Then _validate_model."""
    links = {l.name: l for l in prior.links}
    poses = {p.name: p for p in prior.poses}
    add_names: set = {l.name for l in patch.get("add_links", [])}
    modify_names: set = {l.name for l in patch.get("modify_links", [])}
    changed: set = set()
    for l in patch.get("add_links", []) + patch.get("modify_links", []):
        links[l.name] = l
        changed.add(l.name)
    for n in patch.get("remove_links", []):
        links.pop(n, None)
    for p in patch.get("add_poses", []) + patch.get("modify_poses", []):
        poses[p.name] = p
    for n in patch.get("remove_poses", []):
        poses.pop(n, None)
    new = KinematicModel(name=prior.name, root_link=prior.root_link,
                         links=list(links.values()), poses=list(poses.values()),
                         mesh_pairs=list(prior.mesh_pairs))
    _validate_model(new)                 # normalize + weak forest validation
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
            conv.add_user_message(build_manager_repair(last_err, manager_ir=False))
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

