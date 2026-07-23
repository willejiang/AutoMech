"""Deterministic, provider-neutral collaboration primitives for maker2 agents."""

from .protocol import (
    STATE_PATCH_KIND,
    InvalidProposalError,
    PatchOperation,
    ProposalSource,
    ProtocolError,
    StatePatch,
    StatePatchProposal,
    TypedProposal,
    canonical_json,
    deep_freeze,
    stable_digest,
    thaw,
)
from .runner import AgentRoundResult, AgentTeamRunner
from .state import (
    DuplicateProposalError,
    InvalidTransitionError,
    RevisionSnapshot,
    StaleRevisionError,
    TeamState,
    TeamStateError,
    UnknownProposalKindError,
)

__all__ = [
    "STATE_PATCH_KIND",
    "AgentRoundResult",
    "AgentTeamRunner",
    "DuplicateProposalError",
    "InvalidProposalError",
    "InvalidTransitionError",
    "PatchOperation",
    "ProposalSource",
    "ProtocolError",
    "RevisionSnapshot",
    "StaleRevisionError",
    "StatePatch",
    "StatePatchProposal",
    "TeamState",
    "TeamStateError",
    "TypedProposal",
    "UnknownProposalKindError",
    "canonical_json",
    "deep_freeze",
    "stable_digest",
    "thaw",
]
