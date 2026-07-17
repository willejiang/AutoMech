"""Shared-state schema for the boss↔manager team round.

The managers of a machine's subassemblies run as members of an ``AgentTeamRunner``
team (``maker2/team``): each authors ``TypedProposal``s against one revisioned
``TeamState`` snapshot and the runner commits a stable batch per round. Across rounds
a manager sees the interface frames its SIBLINGS have already realized and can
re-author to match — the collaboration the old one-way handoff could not express.

Only FROZEN, deterministic descriptors live in the snapshot (``KinematicModel`` is
mutable and stays in a side dict owned by the driver). The payloads and reducers here
define what managers publish:

  * ``manager.realized`` — a sub's realized interface frames (frame/link/local pose),
    the same shape ``SubResult.sub_frames`` carries, so siblings can read placements.
  * ``manager.result`` — a compact build outcome (ok / digest / error) for auditing.

Every proposal ``kind`` must have a registered reducer or ``TeamState.commit`` raises
``UnknownProposalKindError``; ``build_team_state`` wires both.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .team import TeamState, TypedProposal

MANAGER_REALIZED_KIND = "manager.realized"
MANAGER_RESULT_KIND = "manager.result"


@dataclass(frozen=True)
class RealizedFrame:
    """One realized interface frame, mirroring a ``SubResult.sub_frames`` entry."""

    frame: str
    link: str
    local_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    local_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def from_sub_frame(cls, d: dict) -> "RealizedFrame":
        return cls(
            frame=str(d.get("frame", "")),
            link=str(d.get("link", "")),
            local_xyz_m=tuple(float(v) for v in (d.get("local_xyz_m") or (0.0, 0.0, 0.0))[:3]),
            local_rpy_rad=tuple(float(v) for v in (d.get("local_rpy_rad") or (0.0, 0.0, 0.0))[:3]),
        )


@dataclass(frozen=True)
class ManagerRealized:
    """A subassembly's realized interface frames, published for its siblings to read."""

    sub_id: str
    frames: tuple  # tuple[RealizedFrame, ...]


@dataclass(frozen=True)
class ManagerResult:
    """A subassembly's compact build outcome."""

    sub_id: str
    ok: bool
    model_digest: str = ""
    error: str = ""


def reduce_manager_realized(state: dict, proposal: TypedProposal) -> None:
    """Record a sub's realized frames as plain dicts under ``state['realized'][sub_id]``.

    Mutates ``state`` in place (the thawed working copy); returns None per the reducer
    contract. Only lists/dicts/primitives are stored so the snapshot stays frozen-safe.
    """
    payload = proposal.payload
    realized = state.setdefault("realized", {})
    realized[payload.sub_id] = [
        {"frame": fr.frame, "link": fr.link,
         "local_xyz_m": list(fr.local_xyz_m), "local_rpy_rad": list(fr.local_rpy_rad)}
        for fr in payload.frames
    ]


def reduce_manager_result(state: dict, proposal: TypedProposal) -> None:
    """Record a sub's build outcome under ``state['results'][sub_id]``."""
    payload = proposal.payload
    results = state.setdefault("results", {})
    results[payload.sub_id] = {
        "ok": bool(payload.ok),
        "model_digest": payload.model_digest,
        "error": payload.error,
    }


REDUCERS = {
    MANAGER_REALIZED_KIND: reduce_manager_realized,
    MANAGER_RESULT_KIND: reduce_manager_result,
}


def seed_state(plan) -> dict:
    """The genesis state for a machine's manager team.

    Carries the frozen compiled hardpoint contract (or None for unrecognized topology)
    plus empty maps the reducers fill. The authoritative live objects (the contract, the
    plan) are passed to managers directly; the snapshot copy here is for audit/determinism.
    """
    contract = getattr(plan, "hardpoint_contract", None)
    return {
        "machine": getattr(plan, "name", "") or "",
        "hardpoint_contract": contract.to_dict() if contract is not None else None,
        "realized": {},
        "results": {},
    }


def build_team_state(plan) -> TeamState:
    """A ``TeamState`` seeded from ``plan`` with the manager reducers registered."""
    return TeamState(seed_state(plan), reducers=dict(REDUCERS))
