"""Safe, read-only ingestion of portable benchmark submission folders."""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping

from .contract import (
    CONTRACT_ID, ContractError, Evidence, FileRecord, IngestedSubmission,
    ResourceLimits, SubmissionManifest, evidence_for_files, validate_relative_path,
)

MANIFEST_NAME = "benchmark_submission.json"
_JSON_MEDIA = {"application/json", "text/json"}
_SCORE_BEARING_JSON = {
    "assembly.json", "task_bindings.json", "evidence/trajectory.json",
    "evidence/contacts.json", "evidence/execution.json", "evidence/geometry.json",
}


def file_sha256(path: Path, *, max_bytes: int | None = None) -> str:
    digest = hashlib.sha256()
    total = 0
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            total += len(chunk)
            if max_bytes is not None and total > max_bytes:
                raise ContractError(f"file grew beyond limit while hashing: {path.name}")
            digest.update(chunk)
    return digest.hexdigest()


def _reject_constant(token: str) -> None:
    raise ContractError(f"non-finite JSON number is forbidden: {token}")


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ContractError(f"duplicate JSON object key is forbidden: {key!r}")
        result[key] = value
    return result


def _json_shape(value: Any, *, max_depth: int, max_nodes: int,
                max_samples: int) -> None:
    nodes = 0
    stack = [(value, 1)]
    while stack:
        item, depth = stack.pop()
        nodes += 1
        if nodes > max_nodes:
            raise ContractError("JSON node limit exceeded")
        if depth > max_depth:
            raise ContractError("JSON nesting depth limit exceeded")
        if isinstance(item, float) and not math.isfinite(item):
            raise ContractError("non-finite JSON number is forbidden")
        if isinstance(item, dict):
            stack.extend((child, depth + 1) for child in item.values())
        elif isinstance(item, list):
            if len(item) > max_samples:
                raise ContractError("JSON sample/array limit exceeded")
            stack.extend((child, depth + 1) for child in item)


def load_json_bytes(raw: bytes, *, name: str, limits: ResourceLimits) -> Any:
    if len(raw) > limits.max_json_bytes:
        raise ContractError(f"JSON file exceeds {limits.max_json_bytes} bytes: {name}")
    try:
        value = json.loads(raw.decode("utf-8"), parse_constant=_reject_constant,
                           object_pairs_hook=_reject_duplicate_keys)
    except UnicodeDecodeError as exc:
        raise ContractError(f"JSON is not UTF-8: {name}") from exc
    except json.JSONDecodeError as exc:
        raise ContractError(f"invalid JSON in {name}: {exc.msg}") from exc
    except RecursionError as exc:
        raise ContractError(f"JSON nesting exceeds parser limits: {name}") from exc
    _json_shape(value, max_depth=limits.max_json_depth,
                max_nodes=limits.max_json_nodes, max_samples=limits.max_samples)
    return value


def load_json(path: Path, limits: ResourceLimits) -> Any:
    size = path.stat().st_size
    if size > limits.max_json_bytes:
        raise ContractError(f"JSON file exceeds {limits.max_json_bytes} bytes: {path.name}")
    return load_json_bytes(path.read_bytes(), name=path.name, limits=limits)


def _safe_root(path: str | os.PathLike[str]) -> Path:
    root = Path(path)
    try:
        if root.is_symlink():
            raise ContractError("submission root must not be a symlink")
        root = root.resolve(strict=True)
    except FileNotFoundError as exc:
        raise ContractError(f"submission root does not exist: {path}") from exc
    if not root.is_dir():
        raise ContractError("submission root must be a directory")
    return root


def _safe_file(root: Path, relative: str, limits: ResourceLimits) -> Path:
    relative = validate_relative_path(relative)
    candidate = root.joinpath(*relative.split("/"))
    # lstat each component before resolve: any symlink is forbidden, even one that points
    # back inside the root. This prevents time-of-check path substitution at trust boundaries.
    current = root
    for part in relative.split("/"):
        current = current / part
        if current.is_symlink():
            raise ContractError(f"symlink is forbidden in submission path: {relative}")
    try:
        resolved = candidate.resolve(strict=True)
    except FileNotFoundError as exc:
        raise ContractError(f"declared file is missing: {relative}") from exc
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ContractError(f"path escapes submission root: {relative}") from exc
    if not resolved.is_file():
        raise ContractError(f"declared path is not a regular file: {relative}")
    size = resolved.stat().st_size
    if size > limits.max_file_bytes:
        raise ContractError(f"file exceeds size limit: {relative}")
    return resolved


def _file_record(item: Any) -> FileRecord:
    if not isinstance(item, Mapping):
        raise ContractError("each manifest files entry must be an object")
    try:
        return FileRecord(
            path=item["path"], sha256=item["sha256"], size=item["size"],
            media_type=item.get("media_type", "application/octet-stream"),
            role=item.get("role"),
        )
    except KeyError as exc:
        raise ContractError(f"files entry missing {exc.args[0]!r}") from exc


def _require_strings(mapping: Any, names: tuple[str, ...], label: str) -> None:
    if not isinstance(mapping, Mapping):
        raise ContractError(f"{label} must be an object")
    for name in names:
        value = mapping.get(name)
        if not isinstance(value, str) or not value.strip():
            raise ContractError(f"{label}.{name} must be a non-empty string")


def _validate_metadata(value: Mapping[str, Any]) -> None:
    _require_strings(value["producer"], ("harness", "harness_version", "run_id"),
                     "producer")
    _require_strings(value["evidence_lane"],
                     ("physics_mode", "engine", "engine_version"), "evidence_lane")
    submitted = value["evidence_lane"].get("submitted")
    if submitted is not True:
        raise ContractError("evidence_lane.submitted must be true")
    _require_strings(value["units"], ("length", "angle", "time"), "units")
    allowed_units = {"length": {"m", "mm"}, "angle": {"rad", "deg"}, "time": {"s"}}
    for dimension, allowed in allowed_units.items():
        if value["units"][dimension] not in allowed:
            raise ContractError(f"unsupported units.{dimension}: {value['units'][dimension]!r}")
    _require_strings(value["telemetry_provenance"],
                     ("simulator", "sampling_source", "from"),
                     "telemetry_provenance")


def parse_manifest(value: Any) -> SubmissionManifest:
    if not isinstance(value, Mapping):
        raise ContractError("benchmark_submission.json must contain an object")
    required = (
        "contract", "suite_id", "task_id", "prompt_sha256", "producer",
        "evidence_lane", "files", "units", "telemetry_provenance",
    )
    missing = [name for name in required if name not in value]
    if missing:
        raise ContractError("manifest missing required fields: " + ", ".join(missing))
    _validate_metadata(value)
    try:
        from .tasks.comfort_v1 import TASK_REGISTRY
        task = TASK_REGISTRY.get(value["task_id"])
    except Exception:
        task = None
    if task is not None:
        if value["suite_id"] != "physcad-comfort-v1":
            raise ContractError("Comfort task requires suite_id 'physcad-comfort-v1'")
        if str(value["prompt_sha256"]).lower() != task.prompt_sha256:
            raise ContractError("prompt SHA-256 does not match the registered task prompt")
    files_raw = value["files"]
    if not isinstance(files_raw, list):
        raise ContractError("manifest files must be an array")
    known = set(required) | {"idealizations", "raw_audit"}
    raw_audit = value.get("raw_audit", [])
    if isinstance(raw_audit, Mapping):
        raw_audit = tuple(raw_audit.values())
    if not isinstance(raw_audit, (list, tuple)):
        raise ContractError("raw_audit must be an array or object of relative paths")
    idealizations = value.get("idealizations", [])
    if not isinstance(idealizations, list):
        raise ContractError("idealizations must be an array")
    return SubmissionManifest(
        contract=value["contract"], suite_id=value["suite_id"],
        task_id=value["task_id"], prompt_sha256=value["prompt_sha256"],
        producer=value["producer"], evidence_lane=value["evidence_lane"],
        files=tuple(_file_record(item) for item in files_raw), units=value["units"],
        telemetry_provenance=value["telemetry_provenance"],
        idealizations=tuple(idealizations), raw_audit=tuple(raw_audit),
        extra={key: value[key] for key in value if key not in known},
    )


def _check_references(value: Any, *, manifest_paths: set[str], source: str) -> None:
    """Ensure path-like model/mesh references are hash-addressed by the manifest."""
    path_keys = {"mesh", "mesh_path", "mesh_filename", "geometry", "model",
                 "model_path", "file", "path"}
    stack = [value]
    while stack:
        item = stack.pop()
        if isinstance(item, dict):
            for key, child in item.items():
                if key in path_keys and isinstance(child, str):
                    ref = validate_relative_path(child)
                    if ref not in manifest_paths:
                        raise ContractError(
                            f"{source} references unlisted/unhashed file {ref!r}")
                stack.append(child)
        elif isinstance(item, list):
            stack.extend(item)


def ingest_submission(path: str | os.PathLike[str], *,
                      limits: ResourceLimits | None = None) -> IngestedSubmission:
    """Validate and ingest a folder without writing to it or executing submitted code."""
    limits = limits or ResourceLimits()
    root = _safe_root(path)
    manifest_path = _safe_file(root, MANIFEST_NAME, limits)
    manifest_raw = manifest_path.read_bytes()
    manifest = parse_manifest(load_json_bytes(
        manifest_raw, name=MANIFEST_NAME, limits=limits))
    if manifest.contract != CONTRACT_ID:  # defensive; dataclass already rejects this
        raise ContractError(f"unsupported contract: {manifest.contract}")
    if len(manifest.files) > limits.max_files:
        raise ContractError("manifest file-count limit exceeded")

    total = len(manifest_raw)
    if total > limits.max_total_bytes:
        raise ContractError("submission total-size limit exceeded")
    verified: dict[str, Path] = {}
    verified_bytes: dict[str, bytes] = {}
    for record in manifest.files:
        target = _safe_file(root, record.path, limits)
        raw = target.read_bytes()
        actual_size = len(raw)
        total += actual_size
        if total > limits.max_total_bytes:
            raise ContractError("submission total-size limit exceeded")
        if actual_size != record.size:
            raise ContractError(
                f"size mismatch for {record.path}: declared {record.size}, actual {actual_size}")
        actual_hash = hashlib.sha256(raw).hexdigest()
        if actual_hash != record.sha256:
            raise ContractError(f"SHA-256 mismatch for {record.path}")
        verified[record.path] = target
        verified_bytes[record.path] = raw

    # Score-bearing portable files are useful only if declared and hash-verified. Audit files
    # are intentionally loaded too, but remain named as audit documents and carry no verdict.
    documents: dict[str, Any] = {}
    for record in manifest.files:
        is_json = record.media_type in _JSON_MEDIA or record.path.endswith(".json")
        if not is_json:
            continue
        documents[record.path] = load_json_bytes(
            verified_bytes[record.path], name=record.path, limits=limits)

    manifest_paths = set(verified)
    for name in ("assembly.json", "task_bindings.json"):
        if name in documents:
            _check_references(documents[name], manifest_paths=manifest_paths, source=name)

    evidence = (Evidence(kind="submission_manifest", path=MANIFEST_NAME,
                         sha256=hashlib.sha256(manifest_raw).hexdigest(),
                         media_type="application/json"),
                *evidence_for_files(manifest.files))
    return IngestedSubmission(root=str(root), manifest=manifest,
                              documents=documents, evidence=evidence)


__all__ = ["MANIFEST_NAME", "file_sha256", "ingest_submission", "load_json",
           "parse_manifest"]
