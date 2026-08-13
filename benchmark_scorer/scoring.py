"""Deterministic five-layer benchmark scoring.

Raw harness verdicts are never consulted.  Only validated artifacts and numeric
observations produced from them are score-bearing.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from .adapters.automech import AutoMechArtifacts, discover_automech
from .contract import ContractError, Evidence, ResourceLimits
from .ingest import ingest_submission, load_json
from .metrics import (
    axis_drift, circularity_residual, finite_fraction, lateral_drift,
    net_angular_travel, reversal_count, span, stable_ratio_regression,
)
from .tasks.comfort_v1 import FUNCTIONAL_POINTS, TASK_REGISTRY, ComfortTask, get_task

SCORER_VERSION = "physcad-benchmark-scorer/1.0"
LAYER_WEIGHTS = (10, 15, 15, 20, 40)
LAYER_NAMES = ("execution", "assembly", "geometry", "physics_ready", "functional")


@dataclass(frozen=True)
class CheckResult:
    name: str
    status: str
    points_possible: float
    points_awarded: float
    reason: str
    evidence: tuple[Evidence, ...] = ()

    def __post_init__(self) -> None:
        if self.status not in {"PASS", "FAIL", "UNKNOWN"}:
            raise ValueError("check status must be PASS, FAIL, or UNKNOWN")
        if self.points_possible < 0 or not 0 <= self.points_awarded <= self.points_possible:
            raise ValueError("invalid check points")
        object.__setattr__(self, "evidence", tuple(self.evidence))


@dataclass(frozen=True)
class LayerResult:
    index: int
    name: str
    weight: int
    status: str
    points_awarded: float
    prerequisite: str | None
    checks: tuple[CheckResult, ...]


@dataclass(frozen=True)
class ScoreResult:
    task_id: str
    suite_id: str
    input_kind: str
    scorer_version: str
    layers: tuple[LayerResult, ...]
    total_points: float
    unknown_checks: int
    overall_verdict: str
    observed_mechanical_verdict: str

    def to_dict(self) -> dict[str, Any]:
        def evidence(item: Evidence) -> dict[str, Any]:
            return {key: value for key, value in {
                "kind": item.kind, "path": item.path, "sha256": item.sha256,
                "observation": _thaw(item.observation), "source": item.source,
                "media_type": item.media_type,
            }.items() if value is not None}

        return {
            "schema": "physcad-benchmark-score/1.0",
            "scorer_version": self.scorer_version,
            "suite_id": self.suite_id,
            "task_id": self.task_id,
            "input_kind": self.input_kind,
            "layer_weights": list(LAYER_WEIGHTS),
            "layers": [{
                "index": layer.index, "name": layer.name, "weight": layer.weight,
                "status": layer.status, "points_awarded": layer.points_awarded,
                "prerequisite": layer.prerequisite,
                "checks": [{
                    "name": check.name, "status": check.status,
                    "points_possible": check.points_possible,
                    "points_awarded": check.points_awarded, "reason": check.reason,
                    "evidence": [evidence(item) for item in check.evidence],
                } for check in layer.checks],
            } for layer in self.layers],
            "total_points": self.total_points,
            "points_possible": 100,
            "unknown_checks": self.unknown_checks,
            "overall_verdict": self.overall_verdict,
            "observed_mechanical_verdict": self.observed_mechanical_verdict,
        }


@dataclass
class _Context:
    root: Path
    task: ComfortTask
    input_kind: str
    suite_id: str
    documents: dict[str, Any]
    evidence: tuple[Evidence, ...]
    artifact_paths: dict[str, str]
    bindings: dict[str, tuple[str, ...]]
    trajectory: Mapping[str, Any] | None = None
    load_errors: tuple[str, ...] = ()


def _thaw(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _thaw(child) for key, child in value.items()}
    if isinstance(value, tuple):
        return [_thaw(child) for child in value]
    return value


def _ev(ctx: _Context, *kinds: str) -> tuple[Evidence, ...]:
    selected = tuple(item for item in ctx.evidence if item.kind in kinds)
    return selected[:12]


def _check(name: str, status: str, points: float, reason: str,
           evidence: Iterable[Evidence] = ()) -> CheckResult:
    return CheckResult(name, status, points, points if status == "PASS" else 0.0,
                       reason, tuple(evidence))


def _status(checks: Sequence[CheckResult]) -> str:
    if any(item.status == "FAIL" for item in checks):
        return "FAIL"
    if any(item.status == "UNKNOWN" for item in checks):
        return "UNKNOWN"
    return "PASS"


def _layer(index: int, checks: Sequence[CheckResult], prerequisite: str | None = None) -> LayerResult:
    weight = LAYER_WEIGHTS[index - 1]
    return LayerResult(index, LAYER_NAMES[index - 1], weight, _status(checks),
                       round(sum(item.points_awarded for item in checks), 6),
                       prerequisite, tuple(checks))


def _gated(index: int, previous: LayerResult) -> LayerResult:
    weight = LAYER_WEIGHTS[index - 1]
    reason = f"prerequisite layer {previous.index} ({previous.name}) is {previous.status}"
    status = "FAIL" if previous.status == "FAIL" else "UNKNOWN"
    return _layer(index, [_check("prerequisite_gate", status, weight, reason)], reason)


def _load_automech(path: str | Path, task_id: str | None,
                   limits: ResourceLimits) -> _Context:
    artifacts = discover_automech(path, limits=limits)
    if task_id is None:
        task_id = _infer_task_id(Path(artifacts.root), artifacts)
    if task_id is None:
        raise ContractError("AutoMech input requires --task-id when the task cannot be inferred")
    try:
        task = get_task(task_id)
    except KeyError as exc:
        raise ContractError(f"unknown Comfort v1 task: {task_id!r}") from exc
    root = Path(artifacts.root)
    docs: dict[str, Any] = {}
    load_errors: list[str] = []
    optional_bindings = root / "task_bindings.json"
    bindings_relative = "task_bindings.json" if optional_bindings.is_file() and not optional_bindings.is_symlink() else None
    for relative in (artifacts.kinematic_model, artifacts.builder_manifest,
                     artifacts.trajectory, artifacts.contacts, bindings_relative):
        if relative:
            try:
                docs[relative] = load_json(root / relative, limits)
            except (ContractError, OSError) as exc:
                # These files are score-bearing. A malformed present file is corrupt
                # evidence, not equivalent to absent evidence.
                docs[relative] = None
                load_errors.append(f"{relative}: {exc}")
    paths = {
        key: value for key, value in {
            "kinematic_model": artifacts.kinematic_model,
            "model_mjcf": artifacts.model_mjcf, "model_urdf": artifacts.model_urdf,
            "builder_manifest": artifacts.builder_manifest,
            "trajectory": artifacts.trajectory, "contacts": artifacts.contacts,
            "video": artifacts.video, "task_bindings": bindings_relative,
        }.items() if value
    }
    if artifacts.meshes:
        paths["meshes"] = ",".join(artifacts.meshes)
    ctx = _Context(root, task, "automech", "physcad-comfort-v1", docs,
                   artifacts.evidence, paths, {}, load_errors=tuple(load_errors))
    ctx.trajectory = docs.get(artifacts.trajectory) if artifacts.trajectory else None
    ctx.bindings = _infer_bindings(ctx)
    _enrich_automech_primary_roles(ctx, artifacts)
    return ctx


def _enrich_automech_primary_roles(ctx: _Context, artifacts: AutoMechArtifacts) -> None:
    input_role, output_role = _input_role(ctx.task), _output_role(ctx.task)
    if not input_role and not output_role:
        return
    kinematic = ctx.documents.get(artifacts.kinematic_model) if artifacts.kinematic_model else None
    manifest = ctx.documents.get(artifacts.builder_manifest) if artifacts.builder_manifest else None
    coordinate_map = ((manifest.get("topology_plan") or {}).get("coordinate_map")
                      if isinstance(manifest, Mapping) else {}) or {}
    joints = set((ctx.trajectory.get("joints") or {}).keys()) if isinstance(ctx.trajectory, Mapping) else set()
    if isinstance(kinematic, Mapping):
        links = kinematic.get("links") or []
        driver_link = next((row.get("name") for row in links
                            if isinstance(row, Mapping) and row.get("driver") is True), None)
        output_link = kinematic.get("output_link")
        if input_role and driver_link:
            mapped = coordinate_map.get(driver_link, driver_link)
            if isinstance(mapped, str) and mapped in joints:
                ctx.bindings[input_role] = (mapped,)
        if output_role and isinstance(output_link, str):
            mapped = coordinate_map.get(output_link, output_link)
            if isinstance(mapped, str) and mapped in joints:
                ctx.bindings[output_role] = (mapped,)
            elif output_link in ((ctx.trajectory or {}).get("bodies") or {}):
                ctx.bindings[output_role] = (output_link,)


def _load_portable(path: str | Path, task_id: str | None,
                   limits: ResourceLimits) -> _Context:
    submission = ingest_submission(path, limits=limits)
    manifest_task = submission.manifest.task_id
    if task_id is not None and task_id != manifest_task:
        raise ContractError("--task-id must match portable manifest task_id")
    try:
        task = get_task(manifest_task)
    except KeyError as exc:
        raise ContractError(f"unknown Comfort v1 task: {manifest_task!r}") from exc
    score_paths = {
        "assembly.json", "task_bindings.json", "evidence/trajectory.json",
        "evidence/contacts.json", "evidence/execution.json", "evidence/geometry.json",
    }
    # Portable inference is deliberately restricted to the contract's score-bearing
    # files. Audit JSON may be loaded by ingestion but can never influence a score.
    docs = {key: _thaw(value) for key, value in submission.documents.items()
            if key in score_paths}
    paths: dict[str, str] = {}
    for record in submission.manifest.files:
        role = (record.role or "").lower()
        if role:
            paths.setdefault(role, record.path)
        if record.path in {
                "assembly.json", "task_bindings.json", "evidence/trajectory.json",
                "evidence/contacts.json", "evidence/execution.json", "evidence/geometry.json"}:
            paths.setdefault(Path(record.path).stem, record.path)
    ctx = _Context(Path(submission.root), task, "portable", submission.manifest.suite_id,
                   docs, submission.evidence, paths, {})
    trajectory_path = _find_document(docs, ("trajectory",), preferred="evidence/trajectory.json")
    ctx.trajectory = docs.get(trajectory_path) if trajectory_path else None
    ctx.bindings = _infer_bindings(ctx)
    return ctx


def _infer_task_id(root: Path, artifacts: AutoMechArtifacts) -> str | None:
    candidates = [root.name]
    for relative in (artifacts.kinematic_model, artifacts.builder_manifest,
                     artifacts.trajectory):
        if not relative:
            continue
        try:
            raw = json.loads((root / relative).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if isinstance(raw, Mapping):
            for key in ("task_id", "benchmark_task_id", "name", "task"):
                value = raw.get(key)
                if isinstance(value, str):
                    candidates.append(value)
    for candidate in candidates:
        folded = candidate.casefold()
        matches = [task_id for task_id in TASK_REGISTRY
                   if task_id.casefold() in folded or folded in task_id.casefold()]
        if len(matches) == 1:
            return matches[0]
        prefix = next((task_id for task_id in TASK_REGISTRY
                       if folded.startswith(task_id.split("_", 1)[0] + "_")), None)
        if prefix:
            return prefix
    return None


def _find_document(documents: Mapping[str, Any], tokens: Sequence[str],
                   preferred: str | None = None) -> str | None:
    if preferred in documents:
        return preferred
    matches = [name for name in documents if all(token in name.casefold() for token in tokens)]
    return min(matches, key=lambda item: (len(item), item.casefold())) if matches else None


def _flatten_names(value: Any) -> list[str]:
    names: list[str] = []
    stack = [value]
    while stack:
        item = stack.pop()
        if isinstance(item, Mapping):
            for key, child in item.items():
                if key in {"name", "id", "link", "joint", "body", "child", "parent"} and isinstance(child, str):
                    names.append(child)
                stack.append(child)
        elif isinstance(item, (list, tuple)):
            stack.extend(item)
    return names


_ROLE_HINTS: Mapping[str, tuple[str, ...]] = {
    "input_shaft": ("input", "driver", "crank"), "output_shaft": ("output",),
    "compound_intermediate_shaft": ("compound", "intermediate"),
    "idler_shaft": ("idler",), "gear": ("gear", "pinion", "wheel"),
    "hand_crank": ("hand_crank", "handcrank", "crank"),
    "minute_input": ("minute", "input"), "hour_output": ("hour", "output"),
    "coaxial_hand": ("hand",), "fixed_ring": ("ring",), "sun_input": ("sun",),
    "carrier_output": ("carrier",), "planet_gear": ("planet", "gear"),
    "planet_pin_hinge": ("planet", "pin"), "crankshaft_input": ("crankshaft", "input"),
    "rotor_shaft_input": ("rotor", "shaft"), "wind_rotor": ("wind", "rotor"),
    "crank_pin": ("crank", "pin"), "eccentric_pin": ("eccentric", "pin"),
    "connecting_rod": ("connecting", "rod"), "pitman_rod": ("pitman",),
    "horizontal_slider": ("slider",), "vertical_crosshead": ("crosshead",),
    "horizontal_guide": ("guide",), "vertical_guide": ("guide",),
    "pump_rod": ("pump", "rod"), "piston_output": ("piston", "output"),
    "walking_beam": ("walking", "beam"), "beam_pivot": ("beam", "pivot"),
    "polished_rod_output": ("polished", "rod"), "crank_disk": ("crank", "disk"),
}


def _explicit_bindings(documents: Mapping[str, Any]) -> dict[str, tuple[str, ...]]:
    path = _find_document(documents, ("task", "binding"), preferred="task_bindings.json")
    if not path or not isinstance(documents[path], Mapping):
        return {}
    raw = documents[path]
    roles = raw.get("roles", raw.get("bindings", raw))
    result: dict[str, tuple[str, ...]] = {}
    if not isinstance(roles, Mapping):
        return result
    for role, value in roles.items():
        if isinstance(value, str):
            result[str(role)] = (value,)
        elif isinstance(value, (list, tuple)):
            result[str(role)] = tuple(str(item) for item in value if isinstance(item, str))
        elif isinstance(value, Mapping):
            nested = value.get("names", value.get("name", value.get("entities")))
            if isinstance(nested, str):
                result[str(role)] = (nested,)
            elif isinstance(nested, (list, tuple)):
                result[str(role)] = tuple(str(item) for item in nested if isinstance(item, str))
    return result


def _normalized_name(value: str) -> str:
    words = [word for word in "".join(ch if ch.isalnum() else " " for ch in value.casefold()).split()
             if word not in {"joint", "hinge", "spin", "body", "link"}]
    return "_".join(words)


def _role_prefers_joint(role: str) -> bool:
    return any(token in role for token in ("shaft", "input", "output", "hinge", "slider"))


def _infer_bindings(ctx: _Context) -> dict[str, tuple[str, ...]]:
    explicit = _explicit_bindings(ctx.documents)
    typed: dict[str, set[str]] = {"joint": set(), "body": set(), "other": set()}
    for document in ctx.documents.values():
        if document is not None:
            typed["other"].update(_flatten_names(document))
    trajectory = ctx.trajectory
    if isinstance(trajectory, Mapping):
        for key, kind in (("joints", "joint"), ("bodies", "body")):
            value = trajectory.get(key)
            if isinstance(value, Mapping):
                typed[kind].update(str(name) for name in value)
    all_names = set().union(*typed.values())
    result: dict[str, tuple[str, ...]] = {}
    used: set[str] = set()
    for role_cardinality in ctx.task.roles:
        role, count = role_cardinality.role, role_cardinality.count
        supplied = tuple(dict.fromkeys(explicit.get(role, ())))
        if supplied:
            # Explicit names must exist in score-bearing model/trajectory evidence.
            if len(supplied) == count and all(name in all_names for name in supplied):
                result[role] = supplied
                used.update(supplied)
            else:
                result[role] = ()
            continue
        role_norm = _normalized_name(role)
        preferred = typed["joint"] if _role_prefers_joint(role) and typed["joint"] else all_names
        available = sorted((name for name in preferred if name not in used), key=str.casefold)
        exact = [name for name in available if _normalized_name(name) == role_norm]
        hints = tuple(_normalized_name(item) for item in _ROLE_HINTS.get(role, tuple(role.split("_"))))
        # Hints are alternatives. Keep only semantically compatible names; never
        # resolve an overfull set by lexical truncation.
        compatible = [name for name in available
                      if any(hint and hint in _normalized_name(name) for hint in hints)]
        candidates = exact if len(exact) == count else compatible
        if len(candidates) == count:
            result[role] = tuple(candidates)
            used.update(candidates)
        else:
            result[role] = ()  # absent or ambiguous => UNKNOWN in semantics
    if isinstance(trajectory, Mapping):
        driver = trajectory.get("input_joint", trajectory.get("driver"))
        input_roles = [item.role for item in ctx.task.roles if "input" in item.role]
        if isinstance(driver, str) and driver in typed["joint"] and input_roles:
            result[input_roles[0]] = (driver,)
    return result


def _series(trajectory: Mapping[str, Any] | None, name: str) -> tuple[float, ...] | None:
    if not isinstance(trajectory, Mapping):
        return None
    joints = trajectory.get("joints")
    if not isinstance(joints, Mapping):
        return None
    candidates = [name, f"{name}_spin", f"{name}_hinge", f"{name}_joint"]
    key = next((candidate for candidate in candidates if candidate in joints), None)
    if key is None:
        folded = name.casefold()
        fuzzy = [str(candidate) for candidate in joints
                 if folded in str(candidate).casefold() or str(candidate).casefold() in folded]
        key = min(fuzzy, key=lambda item: (len(item), item)) if fuzzy else None
    if key is None:
        return None
    raw = joints[key]
    if isinstance(raw, Mapping):
        raw = raw.get("qpos", raw.get("position"))
    if not isinstance(raw, (list, tuple)) or not raw:
        return None
    if isinstance(raw[0], (list, tuple)):
        if any(not item for item in raw):
            return None
        raw = [item[0] for item in raw]
    try:
        return tuple(float(item) for item in raw)
    except (TypeError, ValueError):
        return None


def _bound_series(ctx: _Context, role: str) -> tuple[float, ...] | None:
    for name in ctx.bindings.get(role, ()):
        found = _series(ctx.trajectory, name)
        if found is not None:
            return found
    return None


def _input_role(task: ComfortTask) -> str | None:
    if task.ratio_denominator and "input" in task.ratio_denominator:
        return task.ratio_denominator
    if task.ratio_numerator and "input" in task.ratio_numerator:
        return task.ratio_numerator
    return next((item.role for item in task.roles if "input" in item.role), None)


def _output_role(task: ComfortTask) -> str | None:
    if task.ratio_numerator and "input" not in task.ratio_numerator:
        return task.ratio_numerator
    if task.ratio_denominator and "input" not in task.ratio_denominator:
        return task.ratio_denominator
    return next((item.role for item in reversed(task.roles)
                 if "output" in item.role or "slider" in item.role), None)


def _body_axis_series(ctx: _Context, role: str | None) -> tuple[float, ...] | None:
    if role is None or not isinstance(ctx.trajectory, Mapping):
        return None
    bodies = ctx.trajectory.get("bodies")
    if not isinstance(bodies, Mapping):
        return None
    axis = 0 if ctx.task.output_axis == "horizontal" else 2
    for bound in ctx.bindings.get(role, ()):
        key = next((name for name in bodies if bound.casefold() in str(name).casefold()
                    or str(name).casefold() in bound.casefold()), None)
        if key is None:
            continue
        raw = bodies[key]
        if isinstance(raw, Mapping):
            raw = raw.get("position")
        if isinstance(raw, (list, tuple)) and raw and all(isinstance(item, (list, tuple)) and len(item) > axis for item in raw):
            try:
                return tuple(float(item[axis]) for item in raw)
            except (TypeError, ValueError):
                pass
    return None


def _execution_layer(ctx: _Context) -> LayerResult:
    if ctx.load_errors:
        return _layer(1, [_check("score_bearing_evidence", "FAIL", 10,
                                "malformed score-bearing evidence: " + "; ".join(ctx.load_errors),
                                _ev(ctx, "trajectory", "contacts", "kinematic_model"))])
    document = ctx.documents.get("evidence/execution.json")
    if isinstance(document, Mapping):
        compile_ok = document.get("model_compiled") is True
        initialized = document.get("initialized") is True
        finite = document.get("all_finite") is True
        source_ok = document.get("source_build_ok") is True
        nonempty = document.get("nonempty_part_set") is True
        status = "PASS" if compile_ok and initialized and finite and source_ok and nonempty else "FAIL"
        reason = ("scorer-owned source reexecution produced a nonempty part set and MJCF replay remained finite"
                  if status == "PASS" else
                  "scorer-owned source build or MJCF compile/initialization/finite checks failed")
        return _layer(1, [_check("scorer_owned_execution", status, 10, reason,
                                _ev(ctx, "execution", "model_mjcf"))])
    source = any(item.kind == "source" for item in ctx.evidence)
    reason = ("source is present but no scorer-owned execution record was submitted"
              if source else "no safely replayable execution evidence was submitted")
    return _layer(1, [_check("scorer_owned_execution", "UNKNOWN", 10, reason,
                            _ev(ctx, "source", "model_mjcf"))])


def _assembly_layer(ctx: _Context) -> LayerResult:
    assembly = (_find_document(ctx.documents, ("assembly",), preferred="assembly.json")
                if ctx.input_kind == "portable" else ctx.artifact_paths.get("kinematic_model"))
    model_ok = bool(assembly or any(key in ctx.artifact_paths
                                   for key in ("model", "model_mjcf", "model_urdf")))
    expected = ctx.task.role_cardinalities
    missing = [role for role, count in expected.items()
               if len(set(ctx.bindings.get(role, ()))) < count]
    input_role, output_role = _input_role(ctx.task), _output_role(ctx.task)
    distinct_known = bool(input_role and output_role and ctx.bindings.get(input_role)
                          and ctx.bindings.get(output_role))
    distinct = distinct_known and set(ctx.bindings[input_role]).isdisjoint(ctx.bindings[output_role])
    checks = [
        _check("assembly_model", "PASS" if model_ok else "UNKNOWN", 5,
               "assembly/simulation model is available" if model_ok else "assembly model is missing",
               _ev(ctx, "assembly", "kinematic_model", "model_mjcf", "model_urdf")),
        _check("required_role_cardinalities", "PASS" if not missing else "UNKNOWN", 7,
               "all required roles are bound" if not missing else
               "missing or ambiguous role bindings: " + ", ".join(missing),
               _ev(ctx, "kinematic_model", "builder_manifest", "assembly", "trajectory")),
        _check("input_output_independence",
               "PASS" if distinct else ("FAIL" if distinct_known else "UNKNOWN"), 3,
               "input and output are distinct" if distinct else
               ("input and output alias" if distinct_known else "input/output binding incomplete"),
               _ev(ctx, "kinematic_model", "builder_manifest", "assembly")),
    ]
    return _layer(2, checks)


def _geometry_layer(ctx: _Context) -> LayerResult:
    mesh_evidence = _ev(ctx, "mesh")
    mesh_ok = bool(mesh_evidence)
    geometry = ctx.documents.get("evidence/geometry.json")
    conflict_value = (geometry.get("non_exempt_conflict_count")
                      if isinstance(geometry, Mapping) else None)
    conflict_known = isinstance(conflict_value, int) and not isinstance(conflict_value, bool)
    warnings = geometry.get("provenance_warnings") if isinstance(geometry, Mapping) else ()
    reason = (f"non-exempt conflict count={conflict_value}"
              + (f"; provenance warnings={len(warnings)}" if warnings else "")
              if conflict_known else "no scorer-owned solid-intersection result was submitted")
    checks = [
        _check("mesh_inventory", "PASS" if mesh_ok else "UNKNOWN", 5,
               "mesh files are hash-addressed" if mesh_ok else "mesh evidence is missing",
               mesh_evidence),
        _check("non_exempt_conflicts",
               "PASS" if conflict_value == 0 else ("FAIL" if conflict_known else "UNKNOWN"),
               10, reason, _ev(ctx, "geometry", "assembly", "mesh")),
    ]
    return _layer(3, checks)


def _semantics_layer(ctx: _Context) -> LayerResult:
    expected = ctx.task.role_cardinalities
    present = sum(1 for role, count in expected.items() if len(set(ctx.bindings.get(role, ()))) >= count)
    missing = [role for role, count in expected.items() if len(set(ctx.bindings.get(role, ()))) < count]
    cardinality_status = "PASS" if not missing else "UNKNOWN"
    cardinality = _check("required_role_cardinalities", cardinality_status, 10,
                         "all required roles have deterministic bindings" if not missing
                         else "missing role bindings: " + ", ".join(missing),
                         _ev(ctx, "kinematic_model", "builder_manifest", "assembly"))
    input_role, output_role = _input_role(ctx.task), _output_role(ctx.task)
    distinct_known = bool(input_role and output_role and ctx.bindings.get(input_role)
                          and ctx.bindings.get(output_role))
    distinct = distinct_known and set(ctx.bindings[input_role]).isdisjoint(ctx.bindings[output_role])
    relation = _check("input_output_independence",
                      "PASS" if distinct else ("FAIL" if distinct_known else "UNKNOWN"), 5,
                      "input and output bind to distinct entities" if distinct
                      else ("input and output are bound to the same entity" if distinct_known
                            else "input/output bindings are incomplete"))
    return _layer(3, (cardinality, relation))


def _trajectory_layer(ctx: _Context) -> LayerResult:
    trajectory = ctx.trajectory
    if trajectory is None:
        return _layer(4, [_check("trajectory_present", "UNKNOWN", 20,
                                "trajectory evidence is missing")])
    if not isinstance(trajectory, Mapping):
        return _layer(4, [_check("trajectory_parse", "FAIL", 20,
                                "trajectory JSON is not an object", _ev(ctx, "trajectory"))])
    times = trajectory.get("t")
    joints = trajectory.get("joints")
    present = isinstance(times, (list, tuple)) and len(times) >= 2 and isinstance(joints, Mapping) and bool(joints)
    checks = [_check("trajectory_shape", "PASS" if present else "FAIL", 5,
                     "trajectory has time samples and joint series" if present
                     else "trajectory lacks usable time samples or joints", _ev(ctx, "trajectory"))]
    if not present:
        checks.append(_check("sample_alignment", "UNKNOWN", 5, "unavailable after shape failure"))
        checks.append(_check("finite_health", "UNKNOWN", 10, "unavailable after shape failure"))
        return _layer(4, checks)
    lengths = []
    values: list[Any] = list(times)
    for raw in joints.values():
        if isinstance(raw, Mapping):
            raw = raw.get("qpos", raw.get("position"))
        if isinstance(raw, (list, tuple)):
            lengths.append(len(raw))
            values.extend(raw)
    aligned = bool(lengths) and all(length == len(times) for length in lengths)
    checks.append(_check("sample_alignment", "PASS" if aligned else "FAIL", 5,
                         "all joint series align with time" if aligned else "joint/time sample counts differ"))
    health = trajectory.get("finite_health")
    explicit_bad = isinstance(health, Mapping) and health.get("all_finite") is False
    fraction = finite_fraction(values)
    finite = fraction == 1.0 and not explicit_bad
    checks.append(_check("finite_health", "PASS" if finite else "FAIL", 10,
                         f"finite sample fraction={fraction:.6g}" if finite
                         else f"non-finite state evidence; finite fraction={fraction:.6g}"))
    return _layer(4, checks)


def _invariant_check(ctx: _Context) -> CheckResult:
    """Evaluate only explicitly named invariant observations.

    Portable/AutoMech producers may provide ``invariants`` as a mapping in the
    trajectory or contacts document. Values must be booleans or objects with a
    boolean ``passed`` field. No generic metric stands in for unrelated invariants.
    """
    observations: dict[str, bool] = {}
    sources: list[str] = []
    for path, document in ctx.documents.items():
        if not isinstance(document, Mapping):
            continue
        raw = document.get("invariants")
        if not isinstance(raw, Mapping):
            continue
        sources.append(path)
        for name, value in raw.items():
            passed = value.get("passed") if isinstance(value, Mapping) else value
            if isinstance(passed, bool):
                observations[str(name)] = passed
    required = tuple(ctx.task.invariants)
    missing = [name for name in required if name not in observations]
    if missing:
        return _check("registered_invariants", "UNKNOWN", FUNCTIONAL_POINTS["invariants"],
                      "missing named invariant evidence: " + ", ".join(missing),
                      _ev(ctx, "trajectory", "contacts", "kinematic_model"))
    failed = [name for name in required if not observations[name]]
    return _check("registered_invariants", "FAIL" if failed else "PASS",
                  FUNCTIONAL_POINTS["invariants"],
                  "failed invariants: " + ", ".join(failed) if failed
                  else "all registered invariants have passing named evidence",
                  _ev(ctx, "trajectory", "contacts", "kinematic_model"))


def _functional_layer(ctx: _Context) -> LayerResult:
    task = ctx.task
    input_role, output_role = _input_role(task), _output_role(task)
    input_values = _bound_series(ctx, input_role) if input_role else None
    try:
        input_travel = (abs(net_angular_travel(input_values))
                        if input_values is not None and len(input_values) >= 2 else None)
    except ValueError:
        input_travel = None
    input_tolerance = max(0.05, task.input_min_rad * 0.01)
    input_ok = (input_travel is not None
                and input_travel + input_tolerance >= task.input_min_rad)
    input_check = _check("input_motion", "PASS" if input_ok else
                         ("FAIL" if input_travel is not None else "UNKNOWN"),
                         FUNCTIONAL_POINTS["input"],
                         f"input travel={input_travel:.6g} rad reaches minimum {task.input_min_rad:.6g}"
                         if input_ok else
                         (f"input travel={input_travel:.6g} rad is below minimum {task.input_min_rad:.6g}"
                          if input_travel is not None else "bound input trajectory is unavailable"))
    if not input_ok:
        unknown = input_travel is None
        gate = "functional prerequisites unavailable" if unknown else "input minimum not reached"
        downstream_status = "UNKNOWN" if unknown else "FAIL"
        return _layer(5, (input_check,
            _check("motion_propagation", downstream_status, FUNCTIONAL_POINTS["propagation"], gate),
            _check("registered_output", downstream_status, FUNCTIONAL_POINTS["output"], gate),
            _check("registered_invariants", downstream_status, FUNCTIONAL_POINTS["invariants"], gate)))

    output_values = _bound_series(ctx, output_role) if output_role else None
    if output_values is None:
        output_values = _body_axis_series(ctx, output_role)
    output_moves = (output_values is not None and len(output_values) >= 2
                    and span(output_values) > 1e-9)
    propagation = _check("motion_propagation",
                         "PASS" if output_moves else ("FAIL" if output_values else "UNKNOWN"),
                         FUNCTIONAL_POINTS["propagation"],
                         "registered output exhibits motion" if output_moves else
                         ("registered output remains static" if output_values else
                          "registered output trajectory is unavailable"))

    output_status, output_reason = "UNKNOWN", "bound output trajectory is unavailable"
    if task.ratio_min is not None and task.ratio_max is not None:
        numerator = _bound_series(ctx, task.ratio_numerator or "")
        denominator = _bound_series(ctx, task.ratio_denominator or "")
        if numerator is not None and denominator is not None:
            # Apply the task minimum to the driven input series. Regress in the
            # registry's declared orientation only after that prerequisite passes.
            driven_is_denominator = task.ratio_denominator == input_role
            regression_input = denominator if driven_is_denominator else numerator
            regression_output = numerator if driven_is_denominator else denominator
            try:
                # Full input travel already passed the preregistered minimum. The stable
                # regression trims transients, so require only a meaningful fraction inside
                # that window rather than the full-task minimum again.
                regression = stable_ratio_regression(
                    regression_input, regression_output,
                    min_input_travel=max(0.5, task.input_min_rad * 0.5))
                measured = abs(regression.slope)
                declared_ratio = measured if driven_is_denominator else 1.0 / measured
                declared_slope = regression.slope if driven_is_denominator else 1.0 / regression.slope
                direction_ok = (task.direction is None or
                                (task.direction == "same" and declared_slope > 0) or
                                (task.direction == "opposite" and declared_slope < 0))
                ratio_ok = task.ratio_min <= declared_ratio <= task.ratio_max
                output_status = "PASS" if ratio_ok and direction_ok else "FAIL"
                output_reason = (f"measured declared ratio={declared_ratio:.6g}, "
                                 f"slope={declared_slope:.6g}, R^2={regression.r_squared:.6g}")
            except (ValueError, ZeroDivisionError) as exc:
                output_status, output_reason = "FAIL", str(exc)
    elif output_values is not None:
        units = ctx.trajectory.get("units", {}) if isinstance(ctx.trajectory, Mapping) else {}
        # Joint slide coordinates are always meters in the portable contract and AutoMech
        # trajectory; body positions use units.bodies (typically mm).
        output_from_joint = bool(output_role and _bound_series(ctx, output_role) is not None)
        if output_from_joint:
            factor = 1000.0
        else:
            body_unit = units.get("bodies") if isinstance(units, Mapping) else None
            factor = 1000.0 if body_unit == "m" else 1.0
        measured_span = span(output_values) * factor
        reversals = reversal_count(output_values,
                                   hysteresis=max(measured_span / factor * 0.05, 1e-9))
        output_ok = measured_span >= (task.output_span_min_mm or 0.0) and reversals >= (task.reversals_min or 0)
        output_status = "PASS" if output_ok else "FAIL"
        output_reason = f"output span={measured_span:.6g} mm, reversals={reversals}"
    output = _check("registered_output", output_status, FUNCTIONAL_POINTS["output"], output_reason)
    return _layer(5, (input_check, propagation, output, _invariant_check(ctx)))


def score_context(ctx: _Context) -> ScoreResult:
    """Score a validated context using exactly five prerequisite-gated layers."""
    layers: list[LayerResult] = [_execution_layer(ctx)]
    builders = (_assembly_layer, _geometry_layer, _trajectory_layer, _functional_layer)
    for index, builder in enumerate(builders, start=2):
        previous = layers[-1]
        # A definitive failed prerequisite blocks credit. UNKNOWN means the folder lacks
        # proof for that layer, but independently observable downstream evidence is still
        # evaluated and reported rather than discarded.
        layers.append(builder(ctx) if previous.status != "FAIL" else _gated(index, previous))
    total = round(sum(layer.points_awarded for layer in layers), 6)
    unknown = sum(check.status == "UNKNOWN" for layer in layers for check in layer.checks)
    statuses = [layer.status for layer in layers]
    overall = "FAIL" if "FAIL" in statuses else ("UNKNOWN" if "UNKNOWN" in statuses else "PASS")
    mechanical = [layers[3].status, layers[4].status]
    observed = ("FAIL" if "FAIL" in mechanical else
                "UNKNOWN" if "UNKNOWN" in mechanical else "PASS")
    return ScoreResult(ctx.task.task_id, ctx.suite_id, ctx.input_kind, SCORER_VERSION,
                       tuple(layers), total, unknown, overall, observed)


def detect_input_kind(path: str | Path) -> str:
    root = Path(path)
    if (root / "benchmark_submission.json").is_file():
        return "portable"
    return "automech"


def validate_input(path: str | Path, *, input_kind: str = "auto",
                   task_id: str | None = None,
                   limits: ResourceLimits | None = None) -> dict[str, Any]:
    """Validate and summarize a portable or AutoMech folder without scoring it."""
    limits = limits or ResourceLimits()
    kind = detect_input_kind(path) if input_kind == "auto" else input_kind
    if kind not in {"portable", "automech"}:
        raise ContractError("input kind must be auto, portable, or automech")
    ctx = _load_portable(path, task_id, limits) if kind == "portable" else _load_automech(path, task_id, limits)
    if ctx.load_errors:
        raise ContractError("malformed score-bearing evidence: " + "; ".join(ctx.load_errors))
    return {"valid": True, "input_kind": kind, "task_id": ctx.task.task_id,
            "suite_id": ctx.suite_id, "evidence_files": len(ctx.evidence),
            "bindings": {key: list(value) for key, value in sorted(ctx.bindings.items())}}


def score_path(path: str | Path, *, input_kind: str = "auto",
               task_id: str | None = None,
               limits: ResourceLimits | None = None) -> ScoreResult:
    """Validate and score one folder.  Missing evidence becomes UNKNOWN with zero points."""
    limits = limits or ResourceLimits()
    kind = detect_input_kind(path) if input_kind == "auto" else input_kind
    if kind not in {"portable", "automech"}:
        raise ContractError("input kind must be auto, portable, or automech")
    ctx = _load_portable(path, task_id, limits) if kind == "portable" else _load_automech(path, task_id, limits)
    return score_context(ctx)


__all__ = [
    "CheckResult", "LAYER_NAMES", "LAYER_WEIGHTS", "LayerResult", "SCORER_VERSION",
    "ScoreResult", "detect_input_kind", "score_context", "score_path", "validate_input",
]
