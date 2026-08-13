"""Read-only discovery adapter for legacy AutoMech/maker2 run directories.

The adapter inventories raw evidence; it never imports ``machine.py`` and never treats
``result.json``, ``sim_result.json``, or benchmark telemetry verdicts as score-bearing.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import re
from types import MappingProxyType
from typing import Any, Mapping

from ..contract import ContractError, Evidence, ResourceLimits
from ..ingest import file_sha256, load_json

_AUDIT_NAMES = ("result.json", "benchmark_metrics.json")
_MODEL_NAMES = ("model.mjcf", "model.urdf")
_ITER_RE = re.compile(r"^mujoco_(\d+)$")


@dataclass(frozen=True)
class AutoMechArtifacts:
    root: str
    kinematic_model: str | None = None
    machine_py: str | None = None
    meshes: tuple[str, ...] = ()
    model_mjcf: str | None = None
    model_urdf: str | None = None
    builder_manifest: str | None = None
    physics_dir: str | None = None
    trajectory: str | None = None
    contacts: str | None = None
    video: str | None = None
    audit: Mapping[str, str] = field(default_factory=dict)
    evidence: tuple[Evidence, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "meshes", tuple(self.meshes))
        object.__setattr__(self, "audit", MappingProxyType(dict(self.audit)))
        object.__setattr__(self, "evidence", tuple(self.evidence))


def _root(path: str | os.PathLike[str]) -> Path:
    root = Path(path)
    if root.is_symlink():
        raise ContractError("AutoMech run root must not be a symlink")
    try:
        root = root.resolve(strict=True)
    except FileNotFoundError as exc:
        raise ContractError(f"AutoMech run root does not exist: {path}") from exc
    if not root.is_dir():
        raise ContractError("AutoMech run root must be a directory")
    return root


def _regular(root: Path, candidate: Path | None) -> Path | None:
    if candidate is None or not candidate.exists():
        return None
    current = root
    try:
        rel = candidate.relative_to(root)
    except ValueError:
        return None
    for part in rel.parts:
        current = current / part
        if current.is_symlink():
            raise ContractError(f"symlink is forbidden in AutoMech run: {rel.as_posix()}")
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ContractError(f"AutoMech artifact escapes run root: {candidate}") from exc
    return resolved if resolved.is_file() else None


def _relative(root: Path, candidate: Path | None) -> str | None:
    return candidate.relative_to(root).as_posix() if candidate else None


def _read_result_audit(root: Path, limits: ResourceLimits) -> Mapping[str, Any]:
    result = _regular(root, root / "result.json")
    if not result:
        return {}
    try:
        value = load_json(result, limits)
    except ContractError:
        return {}
    return value if isinstance(value, dict) else {}


def _declared_physics_dir(root: Path, result: Mapping[str, Any]) -> Path | None:
    """Use the final result's evidence path, never a harness pass/fail flag."""
    physics = result.get("physics") if isinstance(result, dict) else None
    candidates: list[str] = []
    if isinstance(physics, dict):
        for key in ("trajectory", "contacts", "video", "frames_dir"):
            value = physics.get(key)
            if isinstance(value, str) and value:
                candidates.append(value)
        for test in physics.get("tests", []) if isinstance(physics.get("tests"), list) else []:
            if isinstance(test, dict):
                for key in ("trajectory", "contacts", "video", "frames_dir"):
                    value = test.get(key)
                    if isinstance(value, str) and value:
                        candidates.append(value)
    for raw in candidates:
        p = Path(raw)
        if p.is_absolute():
            try:
                p = p.resolve(strict=False).relative_to(root)
            except ValueError:
                continue
        elif any(part in ("", ".", "..") for part in p.parts):
            continue
        target = (root / p).resolve(strict=False)
        try:
            target.relative_to(root)
        except ValueError:
            continue
        if target.is_file():
            target = target.parent
        if target.name == "frames":
            target = target.parent
        if target.is_dir() and target.parent.name == "physics":
            return target
    return None


def _physics_candidates(root: Path, limits: ResourceLimits) -> list[Path]:
    physics = root / "physics"
    if not physics.is_dir() or physics.is_symlink():
        return []
    result = []
    scanned = 0
    for child in physics.iterdir():
        scanned += 1
        if scanned > limits.max_files:
            raise ContractError("AutoMech physics directory entry limit exceeded")
        if child.is_dir() and not child.is_symlink() and (child / "trajectory.json").is_file():
            result.append(child)
    return result


def _candidate_order(path: Path) -> tuple[int, int, str]:
    match = _ITER_RE.fullmatch(path.name)
    if match:
        return (2, int(match.group(1)), path.name)
    if path.name == "mujoco":
        return (1, 0, path.name)
    if path.name == "test_0":
        return (0, 0, path.name)
    return (-1, 0, path.name)


def _designated_physics(root: Path, result: Mapping[str, Any],
                        limits: ResourceLimits) -> Path | None:
    del result  # Raw harness metadata is audit-only and may not select score-bearing evidence.
    candidates = _physics_candidates(root, limits)
    if not candidates:
        return None
    # Choose the highest completed standard run deterministically. Corrected/manual/nonstandard
    # directories have lower order and cannot silently replace the designated harness attempt.
    return max(candidates, key=_candidate_order)


def _find_top_or_best(root: Path, name: str) -> Path | None:
    for candidate in (root / name, root / "best" / name):
        found = _regular(root, candidate)
        if found:
            return found
    return None


def discover_automech(path: str | os.PathLike[str], *,
                      limits: ResourceLimits | None = None) -> AutoMechArtifacts:
    limits = limits or ResourceLimits()
    root = _root(path)
    result_doc = _read_result_audit(root, limits)

    kinematic = _find_top_or_best(root, "kinematic_model.json")
    machine = _find_top_or_best(root, "machine.py")
    model_mjcf = _find_top_or_best(root, "model.mjcf")
    model_urdf = _find_top_or_best(root, "model.urdf")
    builder = _find_top_or_best(root, "builder_manifest.json")

    mesh_dirs = [root / "meshes", root / "best" / "meshes"]
    meshes: list[Path] = []
    for meshes_dir in mesh_dirs:
        if not meshes_dir.is_dir() or meshes_dir.is_symlink():
            continue
        scanned = 0
        current_meshes = []
        for candidate in meshes_dir.iterdir():
            scanned += 1
            if scanned > limits.max_files:
                raise ContractError("AutoMech mesh directory entry limit exceeded")
            if candidate.suffix.lower() in {".stl", ".obj", ".ply", ".dae", ".glb",
                                            ".gltf", ".off", ".3mf", ".fbx"}:
                found = _regular(root, candidate)
                if found:
                    current_meshes.append(found)
        if current_meshes:
            meshes = sorted(current_meshes, key=lambda p: p.name.casefold())
            break

    physics_dir = _designated_physics(root, result_doc, limits)
    trajectory = _regular(root, physics_dir / "trajectory.json") if physics_dir else None
    contacts = _regular(root, physics_dir / "contacts.json") if physics_dir else None
    video = None
    if physics_dir:
        video = _regular(root, physics_dir / "model.mp4")
        if video is None:
            mp4s = []
            scanned = 0
            for candidate in physics_dir.iterdir():
                scanned += 1
                if scanned > limits.max_files:
                    raise ContractError("AutoMech physics media entry limit exceeded")
                if candidate.is_file() and candidate.suffix.lower() == ".mp4":
                    mp4s.append(candidate)
            if mp4s:
                video = _regular(root, min(mp4s, key=lambda p: p.name.casefold()))

    audit: dict[str, str] = {}
    for name in _AUDIT_NAMES:
        found = _regular(root, root / name)
        if found:
            audit[name] = found.relative_to(root).as_posix()
    if physics_dir:
        sim_result = _regular(root, physics_dir / "sim_result.json")
        metrics_result = _regular(root, physics_dir / "metrics_result.json")
        if sim_result:
            audit["sim_result.json"] = sim_result.relative_to(root).as_posix()
        if metrics_result:
            audit["metrics_result.json"] = metrics_result.relative_to(root).as_posix()

    located: list[tuple[str, Path | None]] = [
        ("kinematic_model", kinematic), ("source", machine),
        ("model_mjcf", model_mjcf), ("model_urdf", model_urdf),
        ("builder_manifest", builder), ("trajectory", trajectory),
        ("contacts", contacts), ("video", video),
    ]
    evidence = []
    total_bytes = 0
    discovered_files = 0

    def add_evidence(kind: str, item: Path, *, observation=None) -> None:
        nonlocal total_bytes, discovered_files
        size = item.stat().st_size
        if size > limits.max_file_bytes:
            raise ContractError(f"AutoMech artifact exceeds size limit: {item.name}")
        discovered_files += 1
        total_bytes += size
        if discovered_files > limits.max_files:
            raise ContractError("AutoMech artifact file-count limit exceeded")
        if total_bytes > limits.max_total_bytes:
            raise ContractError("AutoMech artifact total-size limit exceeded")
        evidence.append(Evidence(kind=kind, path=item.relative_to(root).as_posix(),
                                 sha256=file_sha256(item, max_bytes=limits.max_file_bytes),
                                 observation=observation))

    for kind, item in located:
        if item:
            add_evidence(kind, item)
    for item in meshes:
        add_evidence("mesh", item)
    for name, relative in audit.items():
        item = root.joinpath(*relative.split("/"))
        add_evidence("audit_only", item,
                     observation={"name": name, "score_bearing": False})

    return AutoMechArtifacts(
        root=str(root), kinematic_model=_relative(root, kinematic),
        machine_py=_relative(root, machine), meshes=tuple(_relative(root, p) for p in meshes),
        model_mjcf=_relative(root, model_mjcf), model_urdf=_relative(root, model_urdf),
        builder_manifest=_relative(root, builder),
        physics_dir=physics_dir.relative_to(root).as_posix() if physics_dir else None,
        trajectory=_relative(root, trajectory), contacts=_relative(root, contacts),
        video=_relative(root, video), audit=audit, evidence=tuple(evidence),
    )


__all__ = ["AutoMechArtifacts", "discover_automech"]
