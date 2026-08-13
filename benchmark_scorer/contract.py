"""Portable benchmark contract primitives.

This package deliberately contains no model calls and no dependency on an AutoMech
verdict.  Contract objects are frozen so ingestion cannot mutate caller-owned input.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import PurePosixPath, PureWindowsPath
from types import MappingProxyType
from typing import Any, Iterable, Mapping
import re
import urllib.parse

CONTRACT_ID = "physcad-benchmark-submission/1.0"
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_WINDOWS_FORBIDDEN = set('<>:"|?*')
_WINDOWS_RESERVED = {"con", "prn", "aux", "nul",
                     *(f"com{i}" for i in range(1, 10)),
                     *(f"lpt{i}" for i in range(1, 10))}


class ContractError(ValueError):
    """A submission is unsafe, corrupt, or not the supported contract version."""


class TriState(str, Enum):
    """A primitive result that never turns missing evidence into a failure."""

    TRUE = "true"
    FALSE = "false"
    UNKNOWN = "unknown"

    @classmethod
    def from_value(cls, value: bool | None | "TriState") -> "TriState":
        if isinstance(value, cls):
            return value
        if value is True:
            return cls.TRUE
        if value is False:
            return cls.FALSE
        if value is None:
            return cls.UNKNOWN
        raise TypeError("tri-state values must be True, False, None, or TriState")

    @property
    def known(self) -> bool:
        return self is not TriState.UNKNOWN


@dataclass(frozen=True)
class Evidence:
    """One immutable, hash-addressed observation supporting a primitive result."""

    kind: str
    path: str | None = None
    sha256: str | None = None
    observation: Any = None
    source: str = "submitted"
    media_type: str | None = None

    def __post_init__(self) -> None:
        if not self.kind:
            raise ContractError("evidence kind must not be empty")
        if self.path is not None:
            object.__setattr__(self, "path", validate_relative_path(self.path))
        if self.sha256 is not None:
            object.__setattr__(self, "sha256", validate_sha256(self.sha256))
        object.__setattr__(self, "observation", freeze_json(self.observation))


@dataclass(frozen=True)
class PrimitiveResult:
    """A deterministic check result plus the evidence used to reach it."""

    name: str
    state: TriState
    evidence: tuple[Evidence, ...] = ()
    reason: str = ""

    def __post_init__(self) -> None:
        if not self.name:
            raise ContractError("primitive result name must not be empty")
        object.__setattr__(self, "state", TriState.from_value(self.state))
        object.__setattr__(self, "evidence", tuple(self.evidence))


@dataclass(frozen=True)
class ResourceLimits:
    """Conservative limits applied before score-bearing content is parsed."""

    max_files: int = 4096
    max_file_bytes: int = 256 * 1024 * 1024
    max_total_bytes: int = 1024 * 1024 * 1024
    max_json_bytes: int = 32 * 1024 * 1024
    max_json_depth: int = 64
    max_json_nodes: int = 2_000_000
    max_samples: int = 5_000_000

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class FileRecord:
    path: str
    sha256: str
    size: int
    media_type: str = "application/octet-stream"
    role: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "path", validate_relative_path(self.path))
        object.__setattr__(self, "sha256", validate_sha256(self.sha256))
        if isinstance(self.size, bool) or not isinstance(self.size, int) or self.size < 0:
            raise ContractError(f"invalid size for {self.path!r}")
        if not isinstance(self.media_type, str) or not self.media_type:
            raise ContractError(f"invalid media type for {self.path!r}")
        if self.role is not None and (not isinstance(self.role, str) or not self.role):
            raise ContractError(f"invalid role for {self.path!r}")


@dataclass(frozen=True)
class SubmissionManifest:
    contract: str
    suite_id: str
    task_id: str
    prompt_sha256: str
    producer: Mapping[str, Any]
    evidence_lane: Mapping[str, Any]
    files: tuple[FileRecord, ...]
    units: Mapping[str, Any]
    telemetry_provenance: Mapping[str, Any]
    idealizations: tuple[Any, ...] = ()
    raw_audit: tuple[str, ...] = ()
    extra: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.contract != CONTRACT_ID:
            raise ContractError(
                f"unsupported contract {self.contract!r}; expected {CONTRACT_ID!r}")
        for name in ("suite_id", "task_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ContractError(f"{name} must be a non-empty string")
        object.__setattr__(self, "prompt_sha256", validate_sha256(self.prompt_sha256))
        object.__setattr__(self, "files", tuple(self.files))
        paths = [item.path for item in self.files]
        portable_keys = [path.casefold() for path in paths]
        if len(portable_keys) != len(set(portable_keys)):
            raise ContractError("manifest contains duplicate or case-colliding file paths")
        object.__setattr__(self, "producer", immutable_mapping(self.producer))
        object.__setattr__(self, "evidence_lane", immutable_mapping(self.evidence_lane))
        object.__setattr__(self, "units", immutable_mapping(self.units))
        object.__setattr__(self, "telemetry_provenance",
                           immutable_mapping(self.telemetry_provenance))
        object.__setattr__(self, "idealizations",
                           tuple(freeze_json(item) for item in self.idealizations))
        object.__setattr__(self, "raw_audit",
                           tuple(validate_relative_path(p) for p in self.raw_audit))
        object.__setattr__(self, "extra", immutable_mapping(self.extra))

    @property
    def files_by_path(self) -> Mapping[str, FileRecord]:
        return MappingProxyType({item.path: item for item in self.files})


@dataclass(frozen=True)
class IngestedSubmission:
    root: str
    manifest: SubmissionManifest
    documents: Mapping[str, Any]
    evidence: tuple[Evidence, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "documents", immutable_mapping(self.documents))
        object.__setattr__(self, "evidence", tuple(self.evidence))


def freeze_json(value: Any) -> Any:
    """Recursively freeze JSON-like data without changing scalar values."""
    if isinstance(value, Mapping):
        return MappingProxyType({key: freeze_json(child) for key, child in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(freeze_json(child) for child in value)
    return value


def immutable_mapping(value: Mapping[str, Any]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ContractError("expected a JSON object")
    return freeze_json(value)


def validate_sha256(value: str) -> str:
    if not isinstance(value, str) or not _SHA256_RE.fullmatch(value.lower()):
        raise ContractError("SHA-256 must be exactly 64 hexadecimal characters")
    return value.lower()


def validate_relative_path(value: str) -> str:
    """Return a canonical portable path or reject traversal/absolute spellings."""
    if not isinstance(value, str) or not value or "\x00" in value:
        raise ContractError("file path must be a non-empty string without NUL")
    if "\\" in value:
        raise ContractError(f"backslashes are not portable in path {value!r}")
    decoded = value
    for _ in range(3):
        newer = urllib.parse.unquote(decoded)
        if newer == decoded:
            break
        decoded = newer
    if decoded != value:
        # Reject encoded separators/dots rather than normalizing an ambiguous manifest.
        if any(token in decoded for token in ("/", "\\", "..")):
            raise ContractError(f"encoded traversal or separator in path {value!r}")
    posix = PurePosixPath(value)
    windows = PureWindowsPath(value)
    if posix.is_absolute() or windows.is_absolute() or windows.drive:
        raise ContractError(f"absolute path is forbidden: {value!r}")
    if any(part in ("", ".", "..") for part in posix.parts):
        raise ContractError(f"non-canonical or traversing path: {value!r}")
    for part in posix.parts:
        if part.endswith((".", " ")) or any(ch in _WINDOWS_FORBIDDEN for ch in part):
            raise ContractError(f"path is not portable to Windows: {value!r}")
        stem = part.split(".", 1)[0].casefold()
        if stem in _WINDOWS_RESERVED:
            raise ContractError(f"reserved Windows path component: {value!r}")
    canonical = posix.as_posix()
    if canonical != value:
        raise ContractError(f"path must be canonical: {value!r}")
    return canonical


def evidence_for_files(records: Iterable[FileRecord]) -> tuple[Evidence, ...]:
    return tuple(Evidence(kind=item.role or "file", path=item.path,
                          sha256=item.sha256, media_type=item.media_type)
                 for item in records)


__all__ = [
    "CONTRACT_ID", "ContractError", "Evidence", "FileRecord", "IngestedSubmission",
    "PrimitiveResult", "ResourceLimits", "SubmissionManifest", "TriState",
    "evidence_for_files", "validate_relative_path", "validate_sha256",
]
