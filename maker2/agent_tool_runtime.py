"""Provider-neutral runtime for bounded LLM tool loops.

This generalizes the behavior currently embedded in ``maker2.tools`` without
changing that module yet.  Callers supply the existing provider-neutral
``Conversation`` and any client exposing ``api_style`` + ``send_with_tools``.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Protocol

from .llm.client import LLMError, LLMResponse, ToolCall


ToolExecutor = Callable[..., Any]


class ToolConversation(Protocol):
    def get_messages_for_api(self, *, api_style: str) -> list[dict]: ...
    def add_user_message(self, content: str) -> None: ...
    def add_assistant_message(
        self, content: str, tool_calls: list[dict] | None = None
    ) -> None: ...
    def add_tool_result(self, tool_call_id: str, content: str) -> None: ...


class ToolClient(Protocol):
    api_style: str

    def send_with_tools(
        self, messages: list[dict], system: str = "", tools: list[dict] | None = None
    ) -> LLMResponse: ...


@dataclass(frozen=True)
class ToolExecution:
    round_index: int
    call_id: str
    name: str
    arguments: Mapping[str, Any]
    result: str
    is_error: bool = False


@dataclass(frozen=True)
class ToolLoopResult:
    text: str
    rounds: int
    executions: tuple[ToolExecution, ...]
    stop_reason: str
    reached_max_rounds: bool = False

    @property
    def did_call_tools(self) -> bool:
        return bool(self.executions)


def tool_definition(
    name: str,
    description: str,
    properties: Mapping[str, Any],
    required: Sequence[str] = (),
) -> dict[str, Any]:
    """Build the OpenAI-function shape relayed by maker2's unified client."""
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": description,
            "parameters": {
                "type": "object",
                "properties": dict(properties),
                "required": list(required),
                "additionalProperties": False,
            },
        },
    }


def execute_tool_call(
    tool_call: ToolCall,
    executors: Mapping[str, ToolExecutor],
    *,
    round_index: int,
    log_fn: Callable[[str], None] | None = None,
) -> ToolExecution:
    """Execute one provider-neutral ToolCall and normalize its result to text."""
    args = tool_call.arguments if isinstance(tool_call.arguments, dict) else {}
    executor = executors.get(tool_call.name)
    if executor is None:
        return ToolExecution(
            round_index,
            tool_call.id,
            tool_call.name,
            args,
            f"(no such tool: {tool_call.name})",
            True,
        )
    if log_fn:
        log_fn(f"[tool] {tool_call.name}({args.get('query', args)})")
    try:
        result = executor(**args)
        if not isinstance(result, str):
            result = str(result)
        return ToolExecution(
            round_index, tool_call.id, tool_call.name, args, result, False
        )
    except Exception as error:
        return ToolExecution(
            round_index,
            tool_call.id,
            tool_call.name,
            args,
            f"(tool {tool_call.name} error: {error})",
            True,
        )


def run_agent_tool_loop(
    client: ToolClient,
    conversation: ToolConversation,
    system: str,
    tools: Sequence[dict[str, Any]],
    executors: Mapping[str, ToolExecutor],
    *,
    max_rounds: int = 4,
    log_fn: Callable[[str], None] | None = None,
    text_only_nudge: str | None = None,
    require_tool_call: bool = True,
) -> ToolLoopResult:
    """Drive a bounded provider-neutral tool loop.

    All calls in one assistant response are executed and returned before the next
    LLM request.  Errors become tool-result text so the model can recover.  Set
    ``require_tool_call`` for agents that must inspect state before answering;
    a single text-only response is then followed by ``text_only_nudge``.
    """
    if max_rounds < 0:
        raise ValueError("max_rounds cannot be negative")
    last_text = ""
    executions: list[ToolExecution] = []
    nudged = False
    rounds = 0
    last_stop_reason = "not_started"

    for round_index in range(max_rounds):
        rounds += 1
        messages = conversation.get_messages_for_api(api_style=client.api_style)
        try:
            response = client.send_with_tools(messages, system=system, tools=list(tools))
        except LLMError as error:
            if log_fn:
                log_fn(f"[tool] tool call failed: {error}")
            return ToolLoopResult(
                last_text,
                rounds,
                tuple(executions),
                "llm_error",
                False,
            )

        last_stop_reason = response.stop_reason
        last_text = response.text or last_text
        if not response.tool_calls:
            if response.text:
                conversation.add_assistant_message(response.text)
            if require_tool_call and not executions and not nudged:
                nudged = True
                conversation.add_user_message(
                    text_only_nudge
                    or "Call an available tool now; do not only describe what you would do."
                )
                continue
            return ToolLoopResult(
                last_text,
                rounds,
                tuple(executions),
                last_stop_reason,
                False,
            )

        conversation.add_assistant_message(
            response.text or "",
            tool_calls=[
                {
                    "id": call.id,
                    "name": call.name,
                    "arguments": call.arguments
                    if isinstance(call.arguments, dict)
                    else {},
                }
                for call in response.tool_calls
            ],
        )
        for call in response.tool_calls:
            execution = execute_tool_call(
                call, executors, round_index=round_index, log_fn=log_fn
            )
            executions.append(execution)
            conversation.add_tool_result(call.id, execution.result)

    if log_fn:
        log_fn(f"[tool] reached max_rounds={max_rounds}; proceeding")
    return ToolLoopResult(
        last_text,
        rounds,
        tuple(executions),
        last_stop_reason,
        bool(max_rounds),
    )


# Explicit migration alias.  Existing maker2.tools.run_tool_loop remains untouched.
run_tool_loop = run_agent_tool_loop


__all__ = [
    "ToolClient",
    "ToolConversation",
    "ToolExecution",
    "ToolExecutor",
    "ToolLoopResult",
    "execute_tool_call",
    "run_agent_tool_loop",
    "run_tool_loop",
    "tool_definition",
]
