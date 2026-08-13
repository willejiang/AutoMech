"""Scorer-owned deterministic replay of an imported ``model.mjcf``.

The replay deliberately consumes only MJCF and scorer-owned task profiles.  It does
not read harness controls, submitted metrics, or model-generated judgments.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
from types import MappingProxyType
from typing import Any, Mapping
import xml.etree.ElementTree as ET

from .contract import ContractError, ResourceLimits, validate_relative_path
from .tasks.comfort_v1 import TASKS, get_task

SCORER_REPLAY_VERSION = "comfort-replay/1.0"
MIN_TIMESTEP_S = 1.0e-5
MAX_FINITE_STEPS = 1_000_000
MAX_MODEL_BODIES = 4096
MAX_MODEL_JOINTS = 4096
MAX_MODEL_DOFS = 16384
MAX_MODEL_GEOMS = 16384
MAX_MODEL_EQUALITIES = 8192
MAX_MODEL_ACTUATORS = 4096
MAX_CONTACTS_PER_SAMPLE = 4096

# Explicit rather than deny-list validation: unknown/new executable extension points
# remain unavailable until the scorer intentionally supports them.
_ALLOWED_TAGS = frozenset({
    "mujoco", "compiler", "option", "flag", "size", "statistic",
    "default", "visual", "global", "quality", "headlight", "map", "scale", "rgba",
    "asset", "texture", "material", "mesh", "hfield", "skin",
    "worldbody", "body", "frame", "inertial", "joint", "freejoint", "geom", "site",
    "camera", "light", "composite", "flexcomp",
    "contact", "pair", "exclude",
    "equality", "connect", "weld", "tendon", "distance",
    "spatial", "fixed", "pulley",
    "actuator", "general", "motor", "position", "velocity", "intvelocity", "damper",
    "cylinder", "muscle", "adhesion",
    "sensor", "touch", "accelerometer", "velocimeter", "gyro", "force", "torque",
    "magnetometer", "camprojection", "rangefinder", "jointpos", "jointvel",
    "jointactuatorfrc", "jointlimitpos", "jointlimitvel", "jointlimitfrc", "tendonpos",
    "tendonvel", "tendonactuatorfrc", "tendonlimitpos", "tendonlimitvel",
    "tendonlimitfrc", "actuatorpos", "actuatorvel", "actuatorfrc", "ballquat",
    "ballangvel", "framepos", "framequat", "framexaxis", "frameyaxis", "framezaxis",
    "framelinvel", "frameangvel", "subtreecom", "subtreelinvel", "subtreeangmom",
    "user", "clock", "keyframe", "key", "custom", "numeric", "text", "tuple",
    "flex", "deformable", "edge", "elasticity", "shell", "cable", "rope", "loop",
})
_FORBIDDEN_TAGS = frozenset({"include", "plugin", "extension"})
_FILE_ASSET_TAGS = frozenset({"mesh", "texture", "hfield", "skin"})
_INPUT_HINTS = (
    "input", "driver", "drive", "crankshaft", "crank", "rotor_shaft", "rotor",
    "sun", "minute",
)


@dataclass(frozen=True)
class ReplayProfile:
    task_id: str
    mode: str
    duration_s: float
    sample_hz: float
    target_speed_rad_s: float
    kp: float | None = None
    kd: float | None = None
    max_effort: float | None = None


@dataclass(frozen=True)
class ReplayResult:
    output_dir: str
    trajectory_path: str
    metadata_path: str
    metadata: Mapping[str, Any]


def _make_profile(task_id: str) -> ReplayProfile:
    task = get_task(task_id)
    numeric_prefix = int(task_id.split("_", 1)[0])
    speed = 2.0
    duration = max(4.0, task.input_min_rad / speed + 0.5)
    if 1 <= numeric_prefix <= 6:
        return ReplayProfile(task_id, "exact_kinematic_projection", duration, 60.0, speed)
    if 7 <= numeric_prefix <= 10:
        return ReplayProfile(task_id, "finite_effort_pd", duration, 60.0, speed,
                             kp=40.0, kd=4.0, max_effort=20.0)
    raise ContractError(f"task {task_id!r} is outside Comfort v1")


TASK_PROFILES: Mapping[str, ReplayProfile] = MappingProxyType({
    task.task_id: _make_profile(task.task_id) for task in TASKS
})


def task_profile(task_id: str) -> ReplayProfile:
    """Return the frozen scorer-owned replay policy for one Comfort v1 task."""
    try:
        return TASK_PROFILES[task_id]
    except KeyError as exc:
        raise ContractError(f"unknown Comfort v1 task: {task_id!r}") from exc


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_root(path: str | os.PathLike[str]) -> Path:
    root = Path(path)
    if root.is_symlink():
        raise ContractError("model root must not be a symlink")
    try:
        root = root.resolve(strict=True)
    except FileNotFoundError as exc:
        raise ContractError(f"model root does not exist: {path}") from exc
    if not root.is_dir():
        raise ContractError("model root must be a directory")
    return root


def _safe_relative(value: str, *, label: str) -> Path:
    try:
        return Path(validate_relative_path(value))
    except ContractError as exc:
        raise ContractError(f"invalid {label}: {exc}") from exc


def _safe_existing_file(root: Path, relative: Path, *, label: str) -> Path:
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise ContractError(f"symlink is forbidden in {label}: {relative.as_posix()}")
    try:
        resolved = current.resolve(strict=True)
    except FileNotFoundError as exc:
        raise ContractError(f"missing {label}: {relative.as_posix()}") from exc
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ContractError(f"{label} escapes model root: {relative.as_posix()}") from exc
    if not resolved.is_file():
        raise ContractError(f"{label} is not a regular file: {relative.as_posix()}")
    return resolved


def _read_and_validate_xml(model_path: Path, limits: ResourceLimits) -> tuple[ET.Element, bytes]:
    if model_path.stat().st_size > limits.max_file_bytes:
        raise ContractError("imported MJCF exceeds the replay file-size limit")
    raw = model_path.read_bytes()
    lowered = raw.lower()
    if b"<!doctype" in lowered or b"<!entity" in lowered:
        raise ContractError("DTD and entity declarations are forbidden in imported MJCF")
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise ContractError(f"invalid MJCF XML: {exc}") from exc
    if root.tag != "mujoco":
        raise ContractError("MJCF root tag must be <mujoco>")
    for element in root.iter():
        tag = element.tag
        if not isinstance(tag, str) or "}" in tag:
            raise ContractError("XML namespaces and non-element nodes are forbidden")
        if tag in _FORBIDDEN_TAGS:
            raise ContractError(f"forbidden MJCF tag <{tag}>")
        if tag not in _ALLOWED_TAGS:
            raise ContractError(f"unsupported MJCF tag <{tag}>")
        if "plugin" in element.attrib:
            raise ContractError("MJCF plugin attributes are forbidden")
    return root, raw


def _stage_model(model_path: Path, model_root: Path, output_dir: Path,
                 xml_root: ET.Element, raw_xml: bytes,
                 limits: ResourceLimits) -> tuple[Path, dict[str, str]]:
    try:
        model_relative = model_path.resolve(strict=True).relative_to(model_root)
    except (FileNotFoundError, ValueError) as exc:
        raise ContractError("model.mjcf must be a regular file within model_root") from exc
    _safe_existing_file(model_root, model_relative, label="model file")

    compiler = xml_root.find("compiler")
    meshdir = Path()
    texturedir = Path()
    if compiler is not None:
        if compiler.get("assetdir"):
            shared = _safe_relative(compiler.get("assetdir", ""), label="compiler assetdir")
            meshdir = shared
            texturedir = shared
        if compiler.get("meshdir"):
            meshdir = _safe_relative(compiler.get("meshdir", ""), label="compiler meshdir")
        if compiler.get("texturedir"):
            texturedir = _safe_relative(compiler.get("texturedir", ""),
                                        label="compiler texturedir")

    staged_root = output_dir / "imported"
    staged_root.mkdir(parents=True, exist_ok=True)
    staged_model = staged_root / "model.mjcf"
    staged_model.write_bytes(raw_xml)
    hashes: dict[str, str] = {}
    total_bytes = len(raw_xml)
    asset_count = 0
    for element in xml_root.iter():
        file_value = element.get("file")
        if file_value is None:
            continue
        if element.tag not in _FILE_ASSET_TAGS:
            raise ContractError(f"file attribute is unsupported on <{element.tag}>")
        relative_file = _safe_relative(file_value, label=f"<{element.tag}> file")
        prefix = texturedir if element.tag == "texture" else meshdir
        source_relative = prefix / relative_file
        source = _safe_existing_file(model_root, source_relative,
                                     label=f"<{element.tag}> asset")
        asset_count += 1
        total_bytes += source.stat().st_size
        if asset_count > limits.max_files:
            raise ContractError("replay asset file-count limit exceeded")
        if source.stat().st_size > limits.max_file_bytes:
            raise ContractError("replay asset file-size limit exceeded")
        if total_bytes > limits.max_total_bytes:
            raise ContractError("replay aggregate size limit exceeded")
        destination = staged_root / source_relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        hashes[source_relative.as_posix()] = _sha256(destination)
    return staged_model, hashes


def _joint_name(mj: Any, model: Any, joint_id: int) -> str:
    return mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"


def _body_name(mj: Any, model: Any, body_id: int) -> str:
    return mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"


def _geom_name(mj: Any, model: Any, geom_id: int) -> str:
    return mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"


def _joint_widths(mj: Any, joint_type: int) -> tuple[int, int]:
    if joint_type == int(mj.mjtJoint.mjJNT_FREE):
        return 7, 6
    if joint_type == int(mj.mjtJoint.mjJNT_BALL):
        return 4, 3
    return 1, 1


def _joint_metadata(mj: Any, model: Any, input_id: int) -> tuple[list[dict[str, Any]], dict[str, int]]:
    metadata = []
    ids = {}
    type_names = {
        int(mj.mjtJoint.mjJNT_FREE): "free",
        int(mj.mjtJoint.mjJNT_BALL): "ball",
        int(mj.mjtJoint.mjJNT_SLIDE): "slide",
        int(mj.mjtJoint.mjJNT_HINGE): "hinge",
    }
    for joint_id in range(model.njnt):
        name = _joint_name(mj, model, joint_id)
        ids[name] = joint_id
        joint_type = int(model.jnt_type[joint_id])
        qwidth, dwidth = _joint_widths(mj, joint_type)
        body_id = int(model.jnt_bodyid[joint_id])
        metadata.append({
            "id": joint_id, "name": name, "type": type_names[joint_type],
            "qpos_adr": int(model.jnt_qposadr[joint_id]),
            "qpos_width": qwidth, "dof_adr": int(model.jnt_dofadr[joint_id]),
            "dof_width": dwidth, "body": _body_name(mj, model, body_id),
            "axis": [float(value) for value in model.jnt_axis[joint_id]],
            "limited": bool(model.jnt_limited[joint_id]),
            "range": [float(value) for value in model.jnt_range[joint_id]],
            "is_input": joint_id == input_id,
        })
    return metadata, ids


def _choose_input_joint(mj: Any, model: Any, requested: str | None) -> int:
    scalar = []
    for joint_id in range(model.njnt):
        kind = int(model.jnt_type[joint_id])
        if kind in (int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE)):
            scalar.append(joint_id)
    if not scalar:
        raise ContractError("replay requires at least one hinge or slide joint")
    if requested is not None:
        found = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, requested)
        if found < 0 or found not in scalar:
            raise ContractError(f"input_joint is not a scalar joint: {requested!r}")
        return int(found)
    ranked = []
    for joint_id in scalar:
        name = _joint_name(mj, model, joint_id).lower()
        hint = next((index for index, token in enumerate(_INPUT_HINTS) if token in name),
                    len(_INPUT_HINTS))
        ranked.append((hint, joint_id))
    return min(ranked)[1]


def _equality_relations(xml_root: ET.Element) -> list[tuple[str, str, tuple[float, ...]]]:
    relations = []
    equality = xml_root.find("equality")
    if equality is None:
        return relations
    for element in equality:
        if element.tag != "joint":
            continue
        if str(element.get("active", "true")).lower() in ("false", "0"):
            continue
        first = element.get("joint1")
        second = element.get("joint2")
        if not first:
            raise ContractError("<equality><joint> requires joint1")
        # Omitted joint2 locks joint1 to a constant polynomial. Direct qpos projection
        # cannot safely drive through that constraint, so exact replay fails closed.
        if not second:
            raise ContractError("exact replay does not support joint equalities with omitted joint2")
        try:
            coefficients = tuple(float(item) for item in
                                 element.get("polycoef", "0 1 0 0 0").split())
        except ValueError as exc:
            raise ContractError("invalid equality joint polycoef") from exc
        if len(coefficients) != 5 or not all(math.isfinite(item) for item in coefficients):
            raise ContractError("equality joint polycoef must contain five finite numbers")
        relations.append((first, second, coefficients))
    return relations


def _polynomial(coefficients: tuple[float, ...], value: float) -> float:
    return sum(coefficient * value ** power
               for power, coefficient in enumerate(coefficients))


def _project_equalities(qpos: Any, input_name: str, ids: Mapping[str, int], model: Any,
                        relations: list[tuple[str, str, tuple[float, ...]]]) -> None:
    known = {input_name}
    tolerance = 1.0e-9
    for _ in range(len(relations) + 1):
        changed = False
        for first, second, coefficients in relations:
            if first not in ids or second not in ids:
                raise ContractError(f"equality references unknown joint: {first!r}, {second!r}")
            first_adr = int(model.jnt_qposadr[ids[first]])
            second_adr = int(model.jnt_qposadr[ids[second]])
            if second in known:
                projected = _polynomial(coefficients, float(qpos[second_adr]))
                if first in known:
                    if not math.isclose(float(qpos[first_adr]), projected,
                                        rel_tol=tolerance, abs_tol=tolerance):
                        raise ContractError(
                            f"conflicting equality projection for joint {first!r}")
                else:
                    qpos[first_adr] = projected
                    known.add(first)
                    changed = True
            elif first in known:
                if (any(abs(value) > 1e-14 for value in coefficients[2:])
                        or abs(coefficients[1]) < 1e-14):
                    raise ContractError(
                        f"cannot invert nonlinear equality from driven joint {first!r}")
                projected = (float(qpos[first_adr]) - coefficients[0]) / coefficients[1]
                if second in known:
                    if not math.isclose(float(qpos[second_adr]), projected,
                                        rel_tol=tolerance, abs_tol=tolerance):
                        raise ContractError(
                            f"conflicting equality projection for joint {second!r}")
                else:
                    qpos[second_adr] = projected
                    known.add(second)
                    changed = True
        if not changed:
            break


def _finite_state(data: Any) -> bool:
    import numpy as np
    return bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
                and np.isfinite(data.qacc).all() and np.isfinite(data.xpos).all()
                and np.isfinite(data.xquat).all())


def _record_contact_sample(mj: Any, model: Any, data: Any) -> list[dict[str, Any]]:
    import numpy as np
    contacts = []
    if int(data.ncon) > MAX_CONTACTS_PER_SAMPLE:
        raise ContractError("replay contact count exceeds the per-sample limit")
    for index in range(data.ncon):
        contact = data.contact[index]
        force = np.zeros(6, dtype=float)
        try:
            mj.mj_contactForce(model, data, index, force)
        except Exception:
            force[:] = 0.0
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        body1 = int(model.geom_bodyid[geom1])
        body2 = int(model.geom_bodyid[geom2])
        contacts.append({
            "geom1": _geom_name(mj, model, geom1),
            "geom2": _geom_name(mj, model, geom2),
            "body1": _body_name(mj, model, body1),
            "body2": _body_name(mj, model, body2),
            "distance": float(contact.dist),
            "position": [float(value) for value in contact.pos],
            "force": [float(value) for value in force],
        })
    return contacts


def _module_hash(module: Any) -> str | None:
    location = getattr(module, "__file__", None)
    if not location:
        return None
    path = Path(location)
    return _sha256(path) if path.is_file() else None


def _validate_compiled_model(model: Any, profile: ReplayProfile) -> int:
    dimensions = {
        "bodies": (int(model.nbody), MAX_MODEL_BODIES),
        "joints": (int(model.njnt), MAX_MODEL_JOINTS),
        "dofs": (int(model.nv), MAX_MODEL_DOFS),
        "geoms": (int(model.ngeom), MAX_MODEL_GEOMS),
        "equalities": (int(model.neq), MAX_MODEL_EQUALITIES),
        "actuators": (int(model.nu), MAX_MODEL_ACTUATORS),
    }
    for label, (actual, maximum) in dimensions.items():
        if actual > maximum:
            raise ContractError(
                f"compiled model {label} limit exceeded: {actual} > {maximum}")
    timestep = float(model.opt.timestep)
    if not math.isfinite(timestep) or timestep < MIN_TIMESTEP_S:
        raise ContractError(
            f"MuJoCo timestep must be finite and at least {MIN_TIMESTEP_S:g} s")
    finite_steps = int(math.ceil(profile.duration_s / timestep)) + 1
    if finite_steps > MAX_FINITE_STEPS:
        raise ContractError(
            f"finite replay step budget exceeded: {finite_steps} > {MAX_FINITE_STEPS}")
    return finite_steps


def replay_model(model_path: str | os.PathLike[str], output_dir: str | os.PathLike[str],
                 task_id: str, *, model_root: str | os.PathLike[str] | None = None,
                 input_joint: str | None = None,
                 limits: ResourceLimits | None = None) -> ReplayResult:
    """Validate, stage, and deterministically replay one imported MJCF model.

    ``input_joint`` is an optional scorer-side binding for a later CLI.  If omitted,
    selection is deterministic and based only on joint names in the imported model.
    """
    try:
        import mujoco as mj
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("MuJoCo Python bindings are required for replay") from exc

    limits = limits or ResourceLimits()
    source_model = Path(model_path)
    root = _safe_root(model_root or source_model.parent)
    try:
        source_model = source_model.resolve(strict=True)
    except FileNotFoundError as exc:
        raise ContractError(f"model does not exist: {model_path}") from exc
    xml_root, raw_xml = _read_and_validate_xml(source_model, limits)
    profile = task_profile(task_id)

    output_raw = Path(output_dir)
    if output_raw.exists() and output_raw.is_symlink():
        raise ContractError("replay output directory must not be a symlink")
    out = output_raw.resolve(strict=False)
    try:
        out.relative_to(root)
    except ValueError:
        pass
    else:
        raise ContractError("replay output directory must be outside the imported model root")
    out.mkdir(parents=True, exist_ok=True)
    for reserved in (out / "imported", out / "trajectory.json", out / "replay_metadata.json"):
        if reserved.is_symlink():
            raise ContractError(f"replay output path must not be a symlink: {reserved.name}")
    staged_model, asset_hashes = _stage_model(
        source_model, root, out, xml_root, raw_xml, limits)
    try:
        model = mj.MjModel.from_xml_path(str(staged_model))
    except Exception as exc:
        raise ContractError(f"MuJoCo rejected imported model.mjcf: {exc}") from exc
    finite_step_budget = _validate_compiled_model(model, profile)
    data = mj.MjData(model)
    mj.mj_resetData(model, data)
    mj.mj_forward(model, data)

    input_id = _choose_input_joint(mj, model, input_joint)
    joint_meta, joint_ids = _joint_metadata(mj, model, input_id)
    input_name = _joint_name(mj, model, input_id)
    input_qadr = int(model.jnt_qposadr[input_id])
    input_dadr = int(model.jnt_dofadr[input_id])
    relations = _equality_relations(xml_root)

    # Imported actuators are not trusted as replay policy. Scorer PD writes generalized
    # force directly, and exact projection does not integrate dynamics.
    if model.nu:
        model.actuator_gainprm[:] = 0.0
        model.actuator_biasprm[:] = 0.0

    sample_count = int(math.ceil(profile.duration_s * profile.sample_hz)) + 1
    sample_times = np.linspace(0.0, profile.duration_s, sample_count)
    initial_input = float(data.qpos[input_qadr])
    joints: dict[str, dict[str, list[Any]]] = {
        item["name"]: {"qpos": [], "qvel": []} for item in joint_meta
    }
    bodies: dict[str, dict[str, list[list[float]]]] = {
        _body_name(mj, model, body_id): {"position": [], "quaternion": []}
        for body_id in range(model.nbody)
    }
    finite_samples: list[bool] = []
    contact_samples: list[list[dict[str, Any]]] = []
    equality_samples: dict[str, list[float]] = {
        (mj.mj_id2name(model, mj.mjtObj.mjOBJ_EQUALITY, equality_id)
         or f"equality_{equality_id}"): []
        for equality_id in range(model.neq)
    }
    recorded_times: list[float] = []
    previous_qpos = data.qpos.copy()
    previous_time = 0.0

    next_sample = 0
    if profile.mode == "exact_kinematic_projection":
        step_iterable = sample_times
    else:
        step_iterable = None

    def record(now: float) -> None:
        nonlocal previous_qpos, previous_time
        recorded_times.append(float(now))
        elapsed = now - previous_time
        if profile.mode == "exact_kinematic_projection" and elapsed > 0.0:
            for item in joint_meta:
                qadr = item["qpos_adr"]
                dadr = item["dof_adr"]
                if item["qpos_width"] == 1 and item["dof_width"] == 1:
                    data.qvel[dadr] = (data.qpos[qadr] - previous_qpos[qadr]) / elapsed
            mj.mj_forward(model, data)
        finite_samples.append(_finite_state(data))
        for item in joint_meta:
            name = item["name"]
            qa = item["qpos_adr"]
            da = item["dof_adr"]
            qvalues = [float(value) for value in data.qpos[qa:qa + item["qpos_width"]]]
            dvalues = [float(value) for value in data.qvel[da:da + item["dof_width"]]]
            joints[name]["qpos"].append(qvalues[0] if len(qvalues) == 1 else qvalues)
            joints[name]["qvel"].append(dvalues[0] if len(dvalues) == 1 else dvalues)
        for body_id in range(model.nbody):
            name = _body_name(mj, model, body_id)
            bodies[name]["position"].append([float(value) for value in data.xpos[body_id]])
            bodies[name]["quaternion"].append([float(value) for value in data.xquat[body_id]])
        contact_samples.append(_record_contact_sample(mj, model, data))
        equality_residuals: dict[int, list[float]] = {}
        for row in range(data.nefc):
            if int(data.efc_type[row]) == int(mj.mjtConstraint.mjCNSTR_EQUALITY):
                equality_residuals.setdefault(int(data.efc_id[row]), []).append(
                    abs(float(data.efc_pos[row])))
        for equality_id in range(model.neq):
            name = (mj.mj_id2name(model, mj.mjtObj.mjOBJ_EQUALITY, equality_id)
                    or f"equality_{equality_id}")
            values = equality_residuals.get(equality_id)
            equality_samples[name].append(max(values) if values else 0.0)
        previous_qpos = data.qpos.copy()
        previous_time = now

    if step_iterable is not None:
        for now_value in step_iterable:
            now = float(now_value)
            data.qpos[input_qadr] = initial_input + profile.target_speed_rad_s * now
            data.qvel[:] = 0.0
            data.qvel[input_dadr] = profile.target_speed_rad_s
            _project_equalities(data.qpos, input_name, joint_ids, model, relations)
            mj.mj_forward(model, data)
            record(now)
    else:
        end_time = float(sample_times[-1])
        finite_steps = 0
        while next_sample < sample_count:
            target_sample = float(sample_times[next_sample])
            while data.time + 1e-12 < target_sample:
                target = initial_input + profile.target_speed_rad_s * float(data.time)
                error = target - float(data.qpos[input_qadr])
                effort = profile.kp * error - profile.kd * float(data.qvel[input_dadr])
                effort = float(np.clip(effort, -profile.max_effort, profile.max_effort))
                data.qfrc_applied[:] = 0.0
                data.qfrc_applied[input_dadr] = effort
                if finite_steps >= finite_step_budget:
                    raise ContractError("finite replay exhausted its deterministic step budget")
                mj.mj_step(model, data)
                finite_steps += 1
                if not _finite_state(data):
                    break
            record(float(data.time))
            next_sample += 1
            if not finite_samples[-1] or data.time > end_time + model.opt.timestep:
                break

    trajectory = {
        "schema": "physcad-scorer-trajectory/1.0",
        "task_id": task_id,
        "profile": {
            "mode": profile.mode, "duration_s": profile.duration_s,
            "sample_hz": profile.sample_hz,
            "target_speed_rad_s": profile.target_speed_rad_s,
            "kp": profile.kp, "kd": profile.kd, "max_effort": profile.max_effort,
        },
        "input_joint": input_name,
        "t": recorded_times,
        "joints": joints,
        "joint_meta": joint_meta,
        "bodies": bodies,
        "finite_health": {
            "all_finite": bool(finite_samples) and all(finite_samples),
            "samples": finite_samples,
            "first_failure_index": next((index for index, ok in enumerate(finite_samples)
                                          if not ok), None),
        },
        "contacts": {
            "sample_counts": [len(sample) for sample in contact_samples],
            "samples": contact_samples,
        },
        "equality_residuals": equality_samples,
    }
    trajectory_path = out / "trajectory.json"
    trajectory_path.write_text(json.dumps(trajectory, indent=2, sort_keys=True,
                                          allow_nan=False), encoding="utf-8")

    metadata = {
        "schema": "physcad-scorer-replay/1.0",
        "task_id": task_id,
        "input_joint": input_name,
        "profile_mode": profile.mode,
        "output": {"trajectory": "trajectory.json", "imported_model": "imported/model.mjcf"},
        "versions": {
            "scorer": SCORER_REPLAY_VERSION,
            "mujoco": str(getattr(mj, "__version__", "unknown")),
            "model_format": "MJCF",
        },
        "hashes": {
            "source_model_sha256": hashlib.sha256(raw_xml).hexdigest(),
            "imported_model_sha256": _sha256(staged_model),
            "model_assets_sha256": dict(sorted(asset_hashes.items())),
            "trajectory_sha256": _sha256(trajectory_path),
            "scorer_module_sha256": _sha256(Path(__file__)),
            "mujoco_module_sha256": _module_hash(mj),
        },
        "finite_health": trajectory["finite_health"],
        "sample_count": len(finite_samples),
        "model_complexity": {
            "bodies": int(model.nbody), "joints": int(model.njnt),
            "dofs": int(model.nv), "geoms": int(model.ngeom),
            "equalities": int(model.neq), "actuators": int(model.nu),
            "timestep_s": float(model.opt.timestep),
            "finite_step_budget": finite_step_budget,
        },
    }
    metadata_path = out / "replay_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True,
                                        allow_nan=False), encoding="utf-8")
    return ReplayResult(str(out), str(trajectory_path), str(metadata_path), metadata)


__all__ = [
    "ReplayProfile", "ReplayResult", "SCORER_REPLAY_VERSION", "TASK_PROFILES",
    "replay_model", "task_profile",
]
