"""Provider-neutral protocol types for deterministic agent collaboration.

The protocol deliberately has no dependency on ``maker2.design``.  Compiler-specific
proposal payloads can be added later as frozen dataclasses and registered with the
state reducer without changing the team runner.
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, fields, is_dataclass, replace
from enum import Enum
from types import MappingProxyType
from typing import Any, Generic, Mapping, Protocol, TypeVar, runtime_checkable


PayloadT = TypeVar("PayloadT", covariant=True)
FrozenValue = Any


class ProtocolError(ValueError):
    """Base error for malformed team-protocol values."""


class InvalidProposalError(ProtocolError):
    """A proposal does not satisfy the collaboration protocol."""


def deep_freeze(value: Any) -> FrozenValue:
    """Return an immutable, deterministic representation while preserving types.

    Dictionaries become read-only mappings and lists become tuples.  Frozen
    dataclass payloads retain their concrete type, which gives extensions typed
    payloads without allowing mutable data to leak into snapshots or proposals.
    """
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ProtocolError("non-finite floats are not deterministic protocol values")
        return value
    if isinstance(value, Enum):
        return value
    if isinstance(value, Mapping):
        frozen: dict[str, FrozenValue] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ProtocolError("protocol mapping keys must be strings")
            frozen[key] = deep_freeze(item)
        return MappingProxyType(frozen)
    if isinstance(value, (list, tuple)):
        return tuple(deep_freeze(item) for item in value)
    if isinstance(value, frozenset):
        # Sets have no protocol ordering; sort their canonical forms explicitly.
        items = sorted((deep_freeze(item) for item in value), key=canonical_json)
        return tuple(items)
    if isinstance(value, set):
        raise ProtocolError("mutable sets are not valid protocol values; use frozenset")
    if is_dataclass(value) and not isinstance(value, type):
        params = getattr(type(value), "__dataclass_params__", None)
        if not params or not params.frozen:
            raise ProtocolError(
                f"proposal dataclass {type(value).__name__} must be declared frozen=True"
            )
        changes = {
            field.name: deep_freeze(getattr(value, field.name))
            for field in fields(value)
            if field.init
        }
        frozen_value = replace(value, **changes)
        # Validate init=False fields even though dataclasses.replace cannot set them.
        for field in fields(value):
            if not field.init:
                deep_freeze(getattr(value, field.name))
        return frozen_value
    raise ProtocolError(
        f"unsupported mutable or non-serializable protocol value: {type(value).__name__}"
    )


def thaw(value: FrozenValue) -> Any:
    """Make an isolated mutable copy suitable for a reducer."""
    if isinstance(value, Mapping):
        return {key: thaw(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [thaw(item) for item in value]
    if isinstance(value, Enum):
        return value
    if is_dataclass(value) and not isinstance(value, type):
        changes = {
            field.name: thaw(getattr(value, field.name))
            for field in fields(value)
            if field.init
        }
        return replace(value, **changes)
    return value


def canonical_value(value: Any) -> Any:
    """Convert a protocol value to a JSON-compatible canonical value."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ProtocolError("non-finite floats are not deterministic protocol values")
        return value
    if isinstance(value, Enum):
        return canonical_value(value.value)
    if isinstance(value, Mapping):
        return {key: canonical_value(value[key]) for key in sorted(value)}
    if isinstance(value, (list, tuple)):
        return [canonical_value(item) for item in value]
    if is_dataclass(value) and not isinstance(value, type):
        return {
            field.name: canonical_value(getattr(value, field.name))
            for field in fields(value)
        }
    raise ProtocolError(f"cannot canonicalize {type(value).__name__}")


def canonical_json(value: Any) -> str:
    """Serialize with stable key ordering and no platform-dependent whitespace."""
    return json.dumps(
        canonical_value(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )


def stable_digest(value: Any) -> str:
    """Return the SHA-256 digest of a canonical protocol value."""
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class TypedProposal(Generic[PayloadT]):
    """An immutable proposal authored against one exact state revision.

    ``payload`` may be a mapping or a frozen dataclass.  It is recursively frozen
    at construction.  ``create`` derives a stable ID; callers may construct the
    dataclass directly when an external audit ID is required.
    """

    proposal_id: str
    author: str
    base_revision: int
    kind: str
    payload: PayloadT

    def __post_init__(self) -> None:
        for label, value in (
            ("proposal_id", self.proposal_id),
            ("author", self.author),
            ("kind", self.kind),
        ):
            if not isinstance(value, str) or not value.strip():
                raise InvalidProposalError(f"{label} must be a non-empty string")
        if not isinstance(self.base_revision, int) or self.base_revision < 0:
            raise InvalidProposalError("base_revision must be a non-negative integer")
        object.__setattr__(self, "payload", deep_freeze(self.payload))

    @classmethod
    def create(
        cls,
        *,
        author: str,
        base_revision: int,
        kind: str,
        payload: PayloadT,
        key: str = "",
    ) -> "TypedProposal[PayloadT]":
        """Create a proposal whose ID is stable for the same semantic input."""
        frozen_payload = deep_freeze(payload)
        digest = stable_digest(
            {
                "author": author,
                "base_revision": base_revision,
                "kind": kind,
                "key": key,
                "payload": frozen_payload,
            }
        )
        safe_author = "".join(c if c.isalnum() or c in "-_" else "_" for c in author)
        return cls(
            proposal_id=f"{safe_author}:{base_revision}:{digest[:20]}",
            author=author,
            base_revision=base_revision,
            kind=kind,
            payload=frozen_payload,
        )

    def canonical_record(self) -> Mapping[str, Any]:
        return {
            "proposal_id": self.proposal_id,
            "author": self.author,
            "base_revision": self.base_revision,
            "kind": self.kind,
            "payload": self.payload,
        }

    @property
    def fingerprint(self) -> str:
        return stable_digest(self.canonical_record())

    @property
    def stable_order_key(self) -> tuple[str, str, str, str]:
        return (self.kind, self.author, self.proposal_id, canonical_json(self.payload))


class PatchOperation(str, Enum):
    SET = "set"
    DELETE = "delete"


@dataclass(frozen=True)
class StatePatch:
    """Generic typed payload useful for tests and non-domain state updates."""

    operation: PatchOperation
    path: tuple[str, ...]
    value: Any = None

    def __post_init__(self) -> None:
        if not self.path or any(not isinstance(part, str) or not part for part in self.path):
            raise InvalidProposalError("patch path must contain non-empty string segments")
        object.__setattr__(self, "path", tuple(self.path))
        object.__setattr__(self, "value", deep_freeze(self.value))


STATE_PATCH_KIND = "state.patch"
StatePatchProposal = TypedProposal[StatePatch]


@runtime_checkable
class ProposalSource(Protocol):
    """Extension point implemented by an agent or deterministic proposal source."""

    agent_id: str

    def propose(self, snapshot: "SnapshotView") -> list[TypedProposal[Any]]:
        ...


@runtime_checkable
class SnapshotView(Protocol):
    """Read-only snapshot surface exposed to proposal sources."""

    revision: int
    commit_id: str
    state: Mapping[str, Any]
