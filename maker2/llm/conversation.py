"""Conversation history manager (vendored from freecad-ai, persistence stripped).

Stores chat messages in a provider-neutral internal format and converts them
to OpenAI- or Anthropic-shaped messages on demand. The addon's disk
persistence (save/load/list_saved) and its ``..config`` dependency were
removed — this project only needs the in-memory message model and
``get_messages_for_api``.

Message types in internal (provider-neutral) format:
  - User:        {"role": "user", "content": "..."}
  - Assistant:   {"role": "assistant", "content": "...", "tool_calls": [...]}
  - Tool result: {"role": "tool_result", "tool_call_id": "...", "content": "..."}
  - System:      {"role": "user", "content": "[System] ..."}

The get_messages_for_api() method converts to provider-specific format.
"""

import json
import time
from dataclasses import dataclass, field


@dataclass
class Conversation:
    """Manages a single conversation's message history."""

    messages: list[dict] = field(default_factory=list)
    conversation_id: str = ""
    created_at: float = 0.0
    model: str = ""

    def __post_init__(self):
        if not self.conversation_id:
            self.conversation_id = f"conv_{int(time.time() * 1000)}"
        if not self.created_at:
            self.created_at = time.time()
        self.compaction_enabled = True

    def add_user_message(self, content: str, images: list[dict] | None = None,
                         documents: list[dict] | None = None):
        """Add a user message, optionally with images and/or documents.

        Args:
            content: The text content of the message.
            images: Optional list of image dicts, each with keys:
                    type, source, media_type, data (base64).
            documents: Optional list of document dicts, each with keys:
                       filename, text.
        """
        if images or documents:
            blocks = [{"type": "text", "text": content}]
            # Append document content as labeled text blocks
            for doc in (documents or []):
                blocks.append({
                    "type": "text",
                    "text": f"--- Attached file: {doc['filename']} ---\n{doc['text']}",
                })
            blocks.extend(images or [])
            self.messages.append({"role": "user", "content": blocks})
        else:
            self.messages.append({"role": "user", "content": content})

    def add_assistant_message(self, content: str, tool_calls: list[dict] | None = None):
        """Add an assistant message, optionally with tool calls."""
        msg = {"role": "assistant", "content": content}
        if tool_calls:
            msg["tool_calls"] = tool_calls
        self.messages.append(msg)

    def add_tool_result(self, tool_call_id: str, content: str):
        """Add a tool result message."""
        self.messages.append({
            "role": "tool_result",
            "tool_call_id": tool_call_id,
            "content": content,
        })

    def add_system_message(self, content: str, images: list[dict] | None = None):
        """Add a system-level message (execution results, errors, etc.).

        Optionally attach images (e.g. a viewport capture) so vision-capable
        LLMs can see the state the system message is describing.
        """
        # System messages are stored as user messages with a prefix,
        # since not all LLM APIs support arbitrary system messages mid-conversation
        prefixed = f"[System] {content}"
        if images:
            blocks = [{"type": "text", "text": prefixed}]
            blocks.extend(images)
            self.messages.append({"role": "user", "content": blocks})
        else:
            self.messages.append({"role": "user", "content": prefixed})

    def get_messages_for_api(self, max_chars: int = 100000,
                             api_style: str = "openai",
                             describe_fn=None,
                             strip_images: bool = False,
                             strip_thinking: bool = False) -> list[dict]:
        """Get messages formatted for the LLM API.

        Truncates older messages if the total content exceeds max_chars.
        Converts from internal format to provider-specific format.
        Never splits a tool_call/tool_result pair during truncation.

        Args:
            describe_fn: If given, image blocks in history are replaced with
                text descriptions produced by this callable (vision fallback).
            strip_images: If True (and no describe_fn), image blocks in history
                are replaced with a text placeholder. Use for non-vision models
                so a stale image from earlier in the conversation isn't sent to
                a model that would reject it.
            strip_thinking: If True, remove reasoning_content from assistant
                messages in the history.  Required by models like Gemma that
                reject thinking content in multi-turn conversations.
        """
        if not self.messages:
            return []

        # Walk backwards, collecting messages while respecting max_chars
        # and never splitting tool_call/tool_result pairs
        result = []
        total_chars = 0

        i = len(self.messages) - 1
        while i >= 0:
            msg = self.messages[i]
            content = msg.get("content", "")
            msg_chars = self._content_chars(content)

            # If this is a tool_result, we must also include the preceding assistant
            # message that contains the tool_call. Walk back to find the pair.
            if msg["role"] == "tool_result":
                # Collect all consecutive tool_results
                tool_group = [msg]
                j = i - 1
                while j >= 0 and self.messages[j]["role"] == "tool_result":
                    tool_group.insert(0, self.messages[j])
                    j -= 1
                # The message before should be the assistant with tool_calls
                if j >= 0 and self.messages[j]["role"] == "assistant":
                    tool_group.insert(0, self.messages[j])
                    j -= 1

                group_chars = sum(self._content_chars(m.get("content", "")) for m in tool_group)
                if total_chars + group_chars > max_chars and result:
                    break
                result = tool_group + result
                total_chars += group_chars
                i = j
                continue

            if total_chars + msg_chars > max_chars and result:
                break
            result.insert(0, msg)
            total_chars += msg_chars
            i -= 1

        # API histories must start with a user message. A retained suffix may begin
        # with an assistant tool-call group; dropping that whole group can empty the
        # request even though it is internally complete. Anchor such a suffix to the
        # original user instruction instead. This preserves tool-call/result pairing
        # while giving the model the task that the retained tool calls belong to.
        if result and result[0]["role"] != "user":
            anchor = next((message for message in self.messages
                           if message["role"] == "user"), None)
            if anchor is not None:
                result.insert(0, anchor)
            else:
                result = []

        # Replace image blocks with text descriptions if describe_fn is
        # provided, else drop them to a placeholder when strip_images is set.
        if describe_fn:
            result = self._replace_images_with_descriptions(result, describe_fn)
        elif strip_images:
            result = self._strip_images(result)

        # Convert to provider format
        if api_style == "anthropic":
            return self._to_anthropic_format(result)
        else:
            return self._to_openai_format(result, strip_thinking=strip_thinking)

    def _to_openai_format(self, messages: list[dict],
                          strip_thinking: bool = False) -> list[dict]:
        """Convert internal messages to OpenAI API format."""
        result = []
        for msg in messages:
            if msg["role"] == "tool_result":
                result.append({
                    "role": "tool",
                    "tool_call_id": msg["tool_call_id"],
                    "content": msg["content"],
                })
            elif msg["role"] == "assistant" and msg.get("tool_calls"):
                oai_msg = {
                    "role": "assistant",
                    "content": msg.get("content") or None,
                    "tool_calls": [
                        {
                            "id": tc["id"],
                            "type": "function",
                            "function": {
                                "name": tc["name"],
                                "arguments": json.dumps(tc["arguments"]),
                            },
                        }
                        for tc in msg["tool_calls"]
                    ],
                }
                if msg.get("reasoning_content") and not strip_thinking:
                    oai_msg["reasoning_content"] = msg["reasoning_content"]
                result.append(oai_msg)
            elif isinstance(msg.get("content"), list):
                # Content blocks (text + images)
                oai_blocks = []
                for block in msg["content"]:
                    if block.get("type") == "text":
                        oai_blocks.append({"type": "text", "text": block["text"]})
                    elif block.get("type") == "image":
                        data_uri = f"data:{block['media_type']};base64,{block['data']}"
                        oai_blocks.append({
                            "type": "image_url",
                            "image_url": {"url": data_uri},
                        })
                result.append({"role": msg["role"], "content": oai_blocks})
            else:
                out = {"role": msg["role"], "content": msg["content"]}
                if msg.get("reasoning_content") and not strip_thinking:
                    out["reasoning_content"] = msg["reasoning_content"]
                result.append(out)
        return result

    def _to_anthropic_format(self, messages: list[dict]) -> list[dict]:
        """Convert internal messages to Anthropic API format."""
        result = []
        for msg in messages:
            if msg["role"] == "tool_result":
                result.append({
                    "role": "user",
                    "content": [
                        {
                            "type": "tool_result",
                            "tool_use_id": msg["tool_call_id"],
                            "content": msg["content"],
                        }
                    ],
                })
            elif msg["role"] == "assistant" and msg.get("tool_calls"):
                content_blocks = []
                if msg.get("content"):
                    content_blocks.append({"type": "text", "text": msg["content"]})
                for tc in msg["tool_calls"]:
                    content_blocks.append({
                        "type": "tool_use",
                        "id": tc["id"],
                        "name": tc["name"],
                        "input": tc["arguments"],
                    })
                result.append({"role": "assistant", "content": content_blocks})
            elif isinstance(msg.get("content"), list):
                # Content blocks (text + images)
                anth_blocks = []
                for block in msg["content"]:
                    if block.get("type") == "text":
                        anth_blocks.append({"type": "text", "text": block["text"]})
                    elif block.get("type") == "image":
                        anth_blocks.append({
                            "type": "image",
                            "source": {
                                "type": "base64",
                                "media_type": block["media_type"],
                                "data": block["data"],
                            },
                        })
                result.append({"role": msg["role"], "content": anth_blocks})
            else:
                result.append({"role": msg["role"], "content": msg["content"]})
        return result

    @staticmethod
    def _replace_images_with_descriptions(messages: list[dict],
                                          describe_fn) -> list[dict]:
        """Replace image content blocks with text descriptions from describe_fn.

        Images are processed serially. On failure, an error text block is
        substituted so remaining images can still be processed.
        """
        result = []
        for msg in messages:
            if not isinstance(msg.get("content"), list):
                result.append(msg)
                continue
            new_blocks = []
            for block in msg["content"]:
                if block.get("type") == "image":
                    b64_data = block.get("data", "")
                    mime = block.get("mime_type", "image/png")
                    data_url = f"data:{mime};base64,{b64_data}"
                    try:
                        description = describe_fn(data_url)
                        new_blocks.append({
                            "type": "text",
                            "text": f"[Image described: {description}]",
                        })
                    except Exception as e:
                        new_blocks.append({
                            "type": "text",
                            "text": f"[Image: description unavailable — error: {e}]",
                        })
                else:
                    new_blocks.append(block)
            result.append({**msg, "content": new_blocks})
        return result

    @staticmethod
    def _strip_images(messages: list[dict]) -> list[dict]:
        """Replace image content blocks with a placeholder text block.

        For models without vision support and no describe_image fallback, so
        history images aren't sent raw to a provider that would reject them.
        """
        result = []
        for msg in messages:
            if not isinstance(msg.get("content"), list):
                result.append(msg)
                continue
            new_blocks = []
            for block in msg["content"]:
                if block.get("type") == "image":
                    new_blocks.append({
                        "type": "text",
                        "text": "[Image omitted — the current model has no vision support]",
                    })
                else:
                    new_blocks.append(block)
            result.append({**msg, "content": new_blocks})
        return result

    @staticmethod
    def _content_chars(content) -> int:
        """Return character count for content (str or list of blocks)."""
        if isinstance(content, list):
            total = 0
            for block in content:
                if block.get("type") == "text":
                    total += len(block.get("text", ""))
                elif block.get("type") == "image":
                    total += 1000
            return total
        return len(content) if content else 0

    @staticmethod
    def extract_text(content) -> str:
        """Extract plain text from content (str or list of blocks)."""
        if isinstance(content, list):
            parts = []
            for block in content:
                if block.get("type") == "text":
                    parts.append(block.get("text", ""))
            return "\n".join(parts)
        return content or ""

    def clear(self):
        """Clear all messages."""
        self.messages.clear()

    def estimated_tokens(self) -> int:
        """Rough token estimate (chars / 4)."""
        total_chars = 0
        for m in self.messages:
            content = m.get("content", "")
            if isinstance(content, list):
                for block in content:
                    if block.get("type") == "text":
                        total_chars += len(block.get("text", ""))
                    elif block.get("type") == "image":
                        total_chars += 1000  # rough estimate for image tokens
            else:
                total_chars += len(content)
            # Also count tool call arguments
            for tc in m.get("tool_calls", []):
                total_chars += len(str(tc.get("arguments", {})))
        return total_chars // 4
