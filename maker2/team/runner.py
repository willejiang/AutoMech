"""Deterministic orchestration of independent proposal sources."""
from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Any

from .protocol import ProposalSource, TypedProposal
from .state import RevisionSnapshot, TeamState


@dataclass(frozen=True)
class AgentRoundResult:
    agent_id: str
    proposals: tuple[TypedProposal[Any], ...]


class AgentTeamRunner:
    """Collect proposals against one snapshot, then commit one stable batch.

    Proposal sources are invoked in stable ``agent_id`` order.  More elaborate
    execution strategies may supply ``collect_fn`` (for example, a thread pool),
    but its possibly unordered results are normalized before commit.
    """

    def __init__(
        self,
        state: TeamState,
        agents: Iterable[ProposalSource] = (),
        *,
        collect_fn: Callable[
            [RevisionSnapshot, tuple[ProposalSource, ...]], Iterable[AgentRoundResult]
        ]
        | None = None,
    ) -> None:
        self.state = state
        self._agents: dict[str, ProposalSource] = {}
        self._collect_fn = collect_fn
        for agent in agents:
            self.add_agent(agent)

    @property
    def agents(self) -> tuple[ProposalSource, ...]:
        return tuple(self._agents[key] for key in sorted(self._agents))

    def add_agent(self, agent: ProposalSource) -> None:
        agent_id = getattr(agent, "agent_id", "")
        if not isinstance(agent_id, str) or not agent_id:
            raise ValueError("team agents must expose a non-empty agent_id")
        if agent_id in self._agents:
            raise ValueError(f"duplicate team agent ID {agent_id!r}")
        self._agents[agent_id] = agent

    def collect(self, snapshot: RevisionSnapshot | None = None) -> tuple[AgentRoundResult, ...]:
        base = snapshot or self.state.head
        agents = self.agents
        if self._collect_fn is not None:
            results = tuple(self._collect_fn(base, agents))
        else:
            results = tuple(
                AgentRoundResult(agent.agent_id, tuple(agent.propose(base)))
                for agent in agents
            )
        return self._normalize_results(base, results)

    def run_round(self) -> RevisionSnapshot:
        base = self.state.head
        results = self.collect(base)
        proposals = tuple(
            proposal
            for result in results
            for proposal in result.proposals
        )
        return self.state.commit(proposals)

    def run(self, *, max_rounds: int = 1, stop_when_idle: bool = True) -> RevisionSnapshot:
        if max_rounds < 0:
            raise ValueError("max_rounds cannot be negative")
        for _ in range(max_rounds):
            before = self.state.head
            after = self.run_round()
            if stop_when_idle and after is before:
                break
        return self.state.head

    @staticmethod
    def _normalize_results(
        snapshot: RevisionSnapshot,
        results: Sequence[AgentRoundResult],
    ) -> tuple[AgentRoundResult, ...]:
        seen_agents: set[str] = set()
        normalized: list[AgentRoundResult] = []
        for result in results:
            if not isinstance(result, AgentRoundResult):
                raise TypeError("collect_fn must return AgentRoundResult values")
            if result.agent_id in seen_agents:
                raise ValueError(f"duplicate result for agent {result.agent_id!r}")
            seen_agents.add(result.agent_id)
            proposals = tuple(result.proposals)
            for proposal in proposals:
                if not isinstance(proposal, TypedProposal):
                    raise TypeError(
                        f"agent {result.agent_id!r} returned a non-TypedProposal value"
                    )
                if proposal.author != result.agent_id:
                    raise ValueError(
                        f"proposal {proposal.proposal_id!r} author {proposal.author!r} "
                        f"does not match source {result.agent_id!r}"
                    )
                if proposal.base_revision != snapshot.revision:
                    # TeamState performs the authoritative stale check.  Keep the
                    # proposal intact so callers receive StaleRevisionError there.
                    pass
            normalized.append(AgentRoundResult(result.agent_id, proposals))
        return tuple(sorted(normalized, key=lambda result: result.agent_id))
