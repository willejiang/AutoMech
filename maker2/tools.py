#!/usr/bin/env python3
"""Web search as an LLM tool — a keyless executor + the tool loop that drives it.

The gateway (:8313 Copilot proxy) has NO built-in web search but faithfully RELAYS
tool-calling: offered a `web_search` function, the model calls it. So we define the
tool here and implement the executor ourselves. It is KEYLESS — it scrapes the
DuckDuckGo HTML endpoint (Bing HTML as a fallback) over the stdlib `urllib` the LLM
client already uses, so there is no new dependency and no API key. Best-effort: any
network/parse failure returns a short "search unavailable" string and NEVER raises
into the agent loop.

Gated behind Settings.enable_reference_tools; when on, the boss/manager/worker run a
short research pre-step (run_tool_loop) that enriches the conversation with lookups
before their normal generation.
"""
from __future__ import annotations

import html
import re
import urllib.parse
import urllib.request
from html.parser import HTMLParser

from .llm.client import LLMError


# --------------------------------------------------------------------------- #
# The tool definition (OpenAI function-tool shape; the client relays it verbatim)
# --------------------------------------------------------------------------- #

WEB_SEARCH_TOOL = {
    "type": "function",
    "function": {
        "name": "web_search",
        "description": "Search the web for reference designs, standard part dimensions "
                       "(gear modules, bearing/jewel sizes, screw specs), material "
                       "properties, or typical mechanism layouts. Returns the top result "
                       "titles, snippets, and URLs as text.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "the search query"},
                "k": {"type": "integer",
                      "description": "number of results to return (default 5)"},
            },
            "required": ["query"],
        },
    },
}


# --------------------------------------------------------------------------- #
# Keyless executor
# --------------------------------------------------------------------------- #

_UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
       "(KHTML, like Gecko) Chrome/120.0 Safari/537.36")
_TIMEOUT = 8


class _DDGParser(HTMLParser):
    """Pull (title, url, snippet) from DuckDuckGo HTML results. The lite HTML endpoint
    uses <a class="result__a" href=...>title</a> and <a class="result__snippet">text</a>."""

    def __init__(self):
        super().__init__()
        self.results: list[dict] = []
        self._grab = None            # "title" | "snippet"
        self._cur = {"title": "", "url": "", "snippet": ""}

    def handle_starttag(self, tag, attrs):
        if tag != "a":
            return
        cls = dict(attrs).get("class", "")
        if "result__a" in cls:
            if self._cur["title"]:            # a new result begins -> flush prior
                self._flush()
            self._grab = "title"
            self._cur["url"] = dict(attrs).get("href", "")
        elif "result__snippet" in cls:
            self._grab = "snippet"

    def handle_data(self, data):
        if self._grab == "title":
            self._cur["title"] += data
        elif self._grab == "snippet":
            self._cur["snippet"] += data

    def handle_endtag(self, tag):
        if tag == "a":
            self._grab = None

    def _flush(self):
        if self._cur["title"].strip():
            self.results.append({k: v.strip() for k, v in self._cur.items()})
        self._cur = {"title": "", "url": "", "snippet": ""}

    def close(self):
        self._flush()
        super().close()


def _http_get(url: str) -> str:
    req = urllib.request.Request(url, headers={"User-Agent": _UA})
    with urllib.request.urlopen(req, timeout=_TIMEOUT) as r:
        return r.read().decode("utf-8", errors="replace")


def _clean(s: str) -> str:
    return html.unescape(re.sub(r"\s+", " ", s)).strip()


def _ddg_url(href: str) -> str:
    """DDG wraps result urls as /l/?uddg=<encoded>. Unwrap when present."""
    if "uddg=" in href:
        try:
            q = urllib.parse.parse_qs(urllib.parse.urlparse(href).query)
            return urllib.parse.unquote(q.get("uddg", [href])[0])
        except Exception:
            return href
    return href


def web_search_exec(query: str, k: int = 5) -> str:
    """Search the web (keyless) and return a compact text block of the top-k results.
    Best-effort: returns a short 'unavailable' string on any failure, never raises."""
    k = max(1, min(int(k or 5), 10))
    q = urllib.parse.quote_plus(str(query)[:300])
    # 1) DuckDuckGo HTML.
    try:
        p = _DDGParser()
        p.feed(_http_get(f"https://html.duckduckgo.com/html/?q={q}"))
        p.close()
        hits = p.results[:k]
        if hits:
            lines = [f"Search results for {query!r}:"]
            for i, h in enumerate(hits, 1):
                lines.append(f"{i}. {_clean(h['title'])}\n   {_ddg_url(h['url'])}"
                             + (f"\n   {_clean(h['snippet'])}" if h.get("snippet") else ""))
            return "\n".join(lines)
    except Exception:
        pass
    # 2) Bing HTML fallback (titles + urls only).
    try:
        body = _http_get(f"https://www.bing.com/search?q={q}")
        pairs = re.findall(r'<h2><a href="([^"]+)"[^>]*>(.*?)</a>', body)[:k]
        if pairs:
            lines = [f"Search results for {query!r}:"]
            for i, (url, title) in enumerate(pairs, 1):
                lines.append(f"{i}. {_clean(re.sub('<.*?>', '', title))}\n   {url}")
            return "\n".join(lines)
    except Exception:
        pass
    return f"(web search unavailable for {query!r} — no results / network error)"


EXECUTORS = {"web_search": web_search_exec}


# --------------------------------------------------------------------------- #
# Tool loop
# --------------------------------------------------------------------------- #

def run_tool_loop(client, conv, system: str, tools: list, executors: dict, *,
                  max_rounds: int = 4, log_fn=None) -> str:
    """Drive tool-calling: send the conversation with `tools`; for each ToolCall the
    model makes, run executors[name](**arguments) and feed the result back via
    conv.add_tool_result. Repeat until the model stops calling tools (return its text)
    or max_rounds is hit. Tool/exec failures are swallowed as an error string result so
    one bad call never breaks the loop."""
    def log(m):
        if log_fn:
            log_fn(f"[tool] {m}")

    last_text = ""
    for _ in range(max_rounds):
        messages = conv.get_messages_for_api(api_style=client.api_style)
        try:
            resp = client.send_with_tools(messages, system=system, tools=tools)
        except LLMError as e:
            log(f"tool call failed: {e}")
            return last_text
        last_text = resp.text or last_text
        if not resp.tool_calls:
            if resp.text:
                conv.add_assistant_message(resp.text)
            return last_text
        # Record the assistant turn WITH its tool_calls so each tool_result has a
        # matching tool_use block (Anthropic rejects an orphan tool_result otherwise).
        conv.add_assistant_message(resp.text or "", tool_calls=[
            {"id": tc.id, "name": tc.name,
             "arguments": tc.arguments if isinstance(tc.arguments, dict) else {}}
            for tc in resp.tool_calls])
        for tc in resp.tool_calls:
            fn = executors.get(tc.name)
            args = tc.arguments if isinstance(tc.arguments, dict) else {}
            if fn is None:
                result = f"(no such tool: {tc.name})"
            else:
                log(f"{tc.name}({args.get('query', args)})")
                try:
                    result = fn(**args)
                except Exception as e:
                    result = f"(tool {tc.name} error: {e})"
            conv.add_tool_result(tc.id, result)
    log(f"reached max_rounds={max_rounds}; proceeding")
    return last_text


_RESEARCH_SYSTEM = (
    "You are researching to build a mechanical CAD design well. Use the web_search "
    "tool to look up anything genuinely useful — real reference designs, standard part "
    "dimensions (gear modules, bearing/jewel sizes, thread specs), material choices, or "
    "how a mechanism is typically laid out. Do a FEW focused searches at most, then STOP "
    "(reply with a one-line 'done'). Do not design anything yet; just gather facts.")


def maybe_research(client, conv, settings, what: str, *, log_fn=None) -> None:
    """If reference tools are enabled, run a short web-search research turn on `conv`
    BEFORE the caller's normal generation, so useful facts land in the conversation as
    context. Best-effort; never raises. `what` is a short description of the build task
    (for the research prompt)."""
    if not getattr(settings, "enable_reference_tools", False):
        return
    try:
        conv.add_user_message(
            f"Before you begin, research anything useful for this task: {what}")
        run_tool_loop(client, conv, _RESEARCH_SYSTEM, [WEB_SEARCH_TOOL], EXECUTORS,
                      max_rounds=4, log_fn=log_fn)
    except Exception as e:
        if log_fn:
            log_fn(f"[tool] research skipped: {e}")


def research_findings(client, settings, what: str, *, log_fn=None) -> str:
    """Like maybe_research but on a THROWAWAY conversation, returning the collected
    search-result text (to inject as context into a caller that owns its own per-task
    conversations, e.g. the SCAD worker's per-batch convs). Empty string when disabled
    or on failure."""
    if not getattr(settings, "enable_reference_tools", False):
        return ""
    try:
        from .llm.conversation import Conversation
        conv = Conversation()
        conv.add_user_message(
            f"Research standard dimensions / specs useful to build: {what}")
        run_tool_loop(client, conv, _RESEARCH_SYSTEM, [WEB_SEARCH_TOOL], EXECUTORS,
                      max_rounds=3, log_fn=log_fn)
        # Collect the tool_result contents (the actual search text) from the convo.
        notes = [Conversation.extract_text(m.get("content", ""))
                 for m in conv.messages if m.get("role") == "tool_result"]
        return "\n".join(n for n in notes if n).strip()
    except Exception as e:
        if log_fn:
            log_fn(f"[tool] research skipped: {e}")
        return ""
