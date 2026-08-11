"""Focused golden checks for the independent agent-team protocol/runtime."""
from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

from maker2.agent_tool_runtime import run_agent_tool_loop, tool_definition
from maker2.llm.client import LLMResponse, ToolCall
from maker2.llm.conversation import Conversation
from maker2.team import (
    AgentRoundResult,
    AgentTeamRunner,
    StaleRevisionError,
    TeamState,
    TypedProposal,
)


@dataclass(frozen=True)
class PutValue:
    key: str
    value: object


def reduce_put(state, proposal):
    payload = proposal.payload
    assert isinstance(payload, PutValue)
    state[payload.key] = payload.value


def proposal(author: str, revision: int, key: str, value: object):
    return TypedProposal.create(
        author=author,
        base_revision=revision,
        kind="put",
        payload=PutValue(key, value),
    )


def immutable_append_only():
    team = TeamState({"seed": {"values": [1]}}, reducers={"put": reduce_put})
    genesis = team.head
    first = proposal("alpha", 0, "alpha", {"numbers": [2, 3]})
    team.commit([first])
    second = proposal("beta", 1, "beta", 4)
    team.commit([second])

    assert team.revision == 2 and len(team.history) == 3
    assert genesis.state["seed"]["values"] == (1,)
    assert team.snapshot(1).state["alpha"]["numbers"] == (2, 3)
    assert "beta" not in team.snapshot(1).state
    try:
        team.head.state["bad"] = True
        raise AssertionError("snapshot mapping was mutable")
    except TypeError:
        pass
    try:
        first.payload.value["numbers"] += (9,)
        raise AssertionError("proposal payload was mutable")
    except TypeError:
        pass


def stale_rejection_is_atomic():
    team = TeamState(reducers={"put": reduce_put})
    stale = proposal("late", 0, "late", True)
    team.commit([proposal("first", 0, "first", True)])
    before = team.head
    try:
        team.commit([stale])
        raise AssertionError("stale proposal was accepted")
    except StaleRevisionError as error:
        assert error.base_revision == 0 and error.current_revision == 1
    assert team.head is before and len(team.history) == 2


def deterministic_commits():
    proposals = [
        proposal("zeta", 0, "zeta", [3, 2, 1]),
        proposal("alpha", 0, "alpha", {"b": 2, "a": 1}),
    ]
    left = TeamState(reducers={"put": reduce_put})
    right = TeamState(reducers={"put": reduce_put})
    left_snapshot = left.commit(proposals)
    right_snapshot = right.commit(reversed(proposals))
    assert left_snapshot.commit_id == right_snapshot.commit_id
    assert left.export_json_lines() == right.export_json_lines()


class StaticAgent:
    def __init__(self, agent_id: str, key: str):
        self.agent_id = agent_id
        self.key = key

    def propose(self, snapshot):
        return [proposal(self.agent_id, snapshot.revision, self.key, self.agent_id)]


def stable_runner_collection():
    def reversed_collection(snapshot, agents):
        return reversed(
            [
                AgentRoundResult(agent.agent_id, tuple(agent.propose(snapshot)))
                for agent in agents
            ]
        )

    left = TeamState(reducers={"put": reduce_put})
    right = TeamState(reducers={"put": reduce_put})
    AgentTeamRunner(
        left, [StaticAgent("zeta", "z"), StaticAgent("alpha", "a")]
    ).run_round()
    AgentTeamRunner(
        right,
        [StaticAgent("alpha", "a"), StaticAgent("zeta", "z")],
        collect_fn=reversed_collection,
    ).run_round()
    assert left.head.commit_id == right.head.commit_id


class FakeClient:
    def __init__(self, api_style: str):
        self.api_style = api_style
        self.requests = []
        self.responses = [
            LLMResponse(
                "checking",
                [
                    ToolCall("call-b", "double", {"value": 3}),
                    ToolCall("call-a", "missing", {}),
                ],
                "tool_calls",
            ),
            LLMResponse("done", [], "end_turn"),
        ]

    def send_with_tools(self, messages, system="", tools=None):
        self.requests.append((messages, system, tools))
        return self.responses.pop(0)


def provider_neutral_tool_runtime():
    definition = tool_definition(
        "double",
        "Double one integer.",
        {"value": {"type": "integer"}},
        ["value"],
    )
    assert definition["function"]["parameters"]["additionalProperties"] is False
    for style in ("openai", "anthropic"):
        client = FakeClient(style)
        conversation = Conversation()
        conversation.add_user_message("Use the tool.")
        result = run_agent_tool_loop(
            client,
            conversation,
            "test",
            [definition],
            {"double": lambda value: value * 2},
            max_rounds=3,
        )
        assert result.text == "done" and result.rounds == 2
        assert [execution.result for execution in result.executions] == ["6", "(no such tool: missing)"]
        assert result.executions[1].is_error
        followup = client.requests[1][0]
        if style == "openai":
            assert followup[-2]["role"] == "tool" and followup[-1]["role"] == "tool"
        else:
            assert followup[-2]["content"][0]["type"] == "tool_result"
            assert followup[-1]["content"][0]["type"] == "tool_result"


def conversation_compaction_keeps_user_anchor():
    conversation = Conversation()
    conversation.add_user_message("Compile this machine.")
    conversation.add_assistant_message("", tool_calls=[{
        "id": "large-call", "name": "read", "arguments": {},
    }])
    conversation.add_tool_result("large-call", "x" * 200)
    messages = conversation.get_messages_for_api(max_chars=100, api_style="openai")
    assert messages[0] == {"role": "user", "content": "Compile this machine."}
    assert messages[1]["role"] == "assistant" and messages[2]["role"] == "tool"


def main():
    immutable_append_only()
    stale_rejection_is_atomic()
    deterministic_commits()
    stable_runner_collection()
    provider_neutral_tool_runtime()
    conversation_compaction_keeps_user_anchor()
    print("golden agent team protocol: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
