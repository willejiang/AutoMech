"""Append-only revision history for collaborative agent state."""
from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Callable, Iterable, Mapping

from .protocol import TypedProposal, canonical_json, deep_freeze, stable_digest, thaw


class TeamStateError(RuntimeError):
    """Base error for rejected team-state transitions."""


class StaleRevisionError(TeamStateError):
    """A proposal was authored against a revision other than the current head."""

    def __init__(self, proposal_id: str, base_revision: int, current_revision: int):
        self.proposal_id = proposal_id
        self.base_revision = base_revision
        self.current_revision = current_revision
        super().__init__(
            f"proposal {proposal_id!r} targets revision {base_revision}; "
            f"current revision is {current_revision}"
        )


class DuplicateProposalError(TeamStateError):
    """A proposal ID was already committed."""


class UnknownProposalKindError(TeamStateError):
    """No reducer is registered for a proposal kind."""


class InvalidTransitionError(TeamStateError):
    """A reducer returned an invalid next state."""


Reducer = Callable[[dict[str, Any], TypedProposal[Any]], Mapping[str, Any] | None]


@dataclass(frozen=True)
class RevisionSnapshot:
    """Immutable snapshot committed at one monotonically increasing revision."""

    revision: int
    parent_commit_id: str | None
    commit_id: str
    state: Mapping[str, Any]
    proposals: tuple[TypedProposal[Any], ...] = ()

    def __post_init__(self) -> None:
        if self.revision < 0:
            raise InvalidTransitionError("snapshot revision cannot be negative")
        object.__setattr__(self, "state", deep_freeze(self.state))
        object.__setattr__(self, "proposals", tuple(self.proposals))

    def to_dict(self) -> dict[str, Any]:
        return {
            "revision": self.revision,
            "parent_commit_id": self.parent_commit_id,
            "commit_id": self.commit_id,
            "state": thaw(self.state),
            "proposals": [
                {
                    "proposal_id": proposal.proposal_id,
                    "author": proposal.author,
                    "base_revision": proposal.base_revision,
                    "kind": proposal.kind,
                    "payload": thaw(proposal.payload),
                }
                for proposal in self.proposals
            ],
        }


class TeamState:
    """Append-only deterministic state store with optimistic concurrency.

    Each successful ``commit`` appends exactly one immutable snapshot.  Proposals
    for a batch are sorted by their stable protocol key before reduction, so agent
    completion order cannot change the resulting state or commit ID.
    """

    def __init__(
        self,
        initial_state: Mapping[str, Any] | None = None,
        *,
        reducers: Mapping[str, Reducer] | None = None,
    ) -> None:
        self._reducers: dict[str, Reducer] = dict(reducers or {})
        genesis_state = deep_freeze(initial_state or {})
        genesis_id = stable_digest(
            {"revision": 0, "parent_commit_id": None, "state": genesis_state, "proposals": []}
        )
        self._history: list[RevisionSnapshot] = [
            RevisionSnapshot(
                revision=0,
                parent_commit_id=None,
                commit_id=genesis_id,
                state=genesis_state,
            )
        ]
        self._committed_ids: set[str] = set()

    @property
    def head(self) -> RevisionSnapshot:
        return self._history[-1]

    @property
    def revision(self) -> int:
        return self.head.revision

    @property
    def history(self) -> tuple[RevisionSnapshot, ...]:
        return tuple(self._history)

    @property
    def reducers(self) -> Mapping[str, Reducer]:
        return MappingProxyType(dict(self._reducers))

    def snapshot(self, revision: int | None = None) -> RevisionSnapshot:
        if revision is None:
            return self.head
        if not isinstance(revision, int) or revision < 0 or revision >= len(self._history):
            raise IndexError(f"no snapshot at revision {revision!r}")
        return self._history[revision]

    def register_reducer(self, kind: str, reducer: Reducer, *, replace: bool = False) -> None:
        """Register a proposal reducer, leaving domain wiring outside core state."""
        if not isinstance(kind, str) or not kind:
            raise ValueError("proposal kind must be a non-empty string")
        if kind in self._reducers and not replace:
            raise ValueError(f"reducer already registered for {kind!r}")
        self._reducers[kind] = reducer

    def commit(self, proposals: Iterable[TypedProposal[Any]]) -> RevisionSnapshot:
        ordered = tuple(sorted(tuple(proposals), key=lambda proposal: proposal.stable_order_key))
        if not ordered:
            return self.head

        current = self.head
        batch_ids: set[str] = set()
        for proposal in ordered:
            if proposal.base_revision != current.revision:
                raise StaleRevisionError(
                    proposal.proposal_id, proposal.base_revision, current.revision
                )
            if proposal.proposal_id in self._committed_ids or proposal.proposal_id in batch_ids:
                raise DuplicateProposalError(
                    f"proposal ID {proposal.proposal_id!r} has already been committed"
                )
            batch_ids.add(proposal.proposal_id)
            if proposal.kind not in self._reducers:
                raise UnknownProposalKindError(
                    f"no reducer registered for proposal kind {proposal.kind!r}"
                )

        working = thaw(current.state)
        if not isinstance(working, dict):
            raise InvalidTransitionError("team state root must be a mapping")
        for proposal in ordered:
            reduced = self._reducers[proposal.kind](working, proposal)
            if reduced is not None:
                if not isinstance(reduced, Mapping):
                    raise InvalidTransitionError(
                        f"reducer for {proposal.kind!r} returned a non-mapping state"
                    )
                working = dict(reduced)

        frozen_state = deep_freeze(working)
        revision = current.revision + 1
        commit_record = {
            "revision": revision,
            "parent_commit_id": current.commit_id,
            "state": frozen_state,
            "proposals": [proposal.canonical_record() for proposal in ordered],
        }
        snapshot = RevisionSnapshot(
            revision=revision,
            parent_commit_id=current.commit_id,
            commit_id=stable_digest(commit_record),
            state=frozen_state,
            proposals=ordered,
        )
        self._history.append(snapshot)
        self._committed_ids.update(batch_ids)
        return snapshot

    def export_json_lines(self) -> str:
        """Return the append-only log in stable, newline-delimited JSON form."""
        return "\n".join(canonical_json(snapshot.to_dict()) for snapshot in self._history)
