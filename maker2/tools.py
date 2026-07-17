#!/usr/bin/env python3
"""Reference tools as LLM tools — keyless web search + a local KB, and the tool loop.

Two retrieval tools are offered to the agents' research pre-step, gated independently:

  * ``web_search`` (Settings.enable_reference_tools) — the gateway (:8313 Copilot
    proxy) has NO built-in web search but faithfully RELAYS tool-calling: offered a
    ``web_search`` function, the model calls it. We implement the executor ourselves,
    KEYLESS — it scrapes the DuckDuckGo HTML endpoint (Bing HTML fallback) over the
    stdlib ``urllib`` the LLM client already uses, so there is no new dependency and
    no API key.
  * ``kb_search`` (Settings.enable_kb) — a local, offline retrieval over this
    project's curated per-agent knowledge base (output format + conventions + worked
    examples) plus a growing memory of passing runs (see maker2/kb).

Both are best-effort: any network/parse/index failure returns a short "unavailable"
string and NEVER raises into the agent loop. When on, the boss/manager/worker run a
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


KB_SEARCH_TOOL = {
    "type": "function",
    "function": {
        "name": "kb_search",
        "description": "Search the LOCAL knowledge base for this project's OUTPUT "
                       "FORMAT and conventions plus worked examples: the exact schema/"
                       "field contract, the interface-frame (<site>) rules, canonical "
                       "dimension names, gear layout math, hand-authored good "
                       "skeletons, and (once it fills) designs from prior PASSING runs. "
                       "Prefer this over web_search for anything about HOW to structure "
                       "your output. Returns the most relevant passages as text.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {"type": "string",
                          "description": "what you want to look up (a concept, a part "
                                         "type, or a brief of the thing you're building)"},
                "k": {"type": "integer",
                      "description": "number of passages to return (default 5)"},
            },
            "required": ["query"],
        },
    },
}


SOLVE_CONSTRAINTS_TOOL = {
    "type": "function",
    "function": {
        "name": "solve_constraints",
        "description": "Solve a small 3D point/distance constraint problem with the "
                       "AUTHORITATIVE solver (libslvs) to get EXACT coordinates and a "
                       "consistency verdict, instead of guessing. Units are METERS. Use "
                       "it to check gear center distances, coaxial (coincident) shaft "
                       "points, or projected offsets before you commit numbers. Returns "
                       "the solved points, the degrees of freedom, the solver status, and "
                       "any constraints it could not satisfy.",
        "parameters": {
            "type": "object",
            "properties": {
                "points": {
                    "type": "array",
                    "description": "the 3D points in the problem",
                    "items": {
                        "type": "object",
                        "properties": {
                            "id": {"type": "string"},
                            "xyz_m": {"type": "array", "items": {"type": "number"},
                                      "description": "initial [x, y, z] in meters"},
                            "fixed": {"type": "boolean",
                                      "description": "true to anchor this point (default false)"},
                        },
                        "required": ["id", "xyz_m"],
                    },
                },
                "constraints": {
                    "type": "array",
                    "description": "the constraints relating the points",
                    "items": {
                        "type": "object",
                        "properties": {
                            "id": {"type": "string"},
                            "kind": {"type": "string",
                                     "enum": ["coincident", "distance",
                                              "projected_distance"]},
                            "refs": {"type": "array", "items": {"type": "string"},
                                     "description": "point ids: 2 for coincident/distance, "
                                                    "3 for projected_distance"},
                            "value_m": {"type": "number",
                                        "description": "target distance in meters "
                                                       "(distance/projected_distance)"},
                        },
                        "required": ["id", "kind", "refs"],
                    },
                },
                "expected_dof": {"type": "integer",
                                 "description": "the DOF you expect the solution to have "
                                                "(default 0 = fully constrained)"},
            },
            "required": ["points", "constraints"],
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
# Local KB executor (settings.enable_kb) — retrieval over the curated per-agent
# collection + its growing memory. Failure-soft: any problem returns a short
# "unavailable" string, never raises into the tool loop.
# --------------------------------------------------------------------------- #

def kb_search_exec(query: str, collection: str = "manager", k: int = 5) -> str:
    """Search the local KB `collection` (+ its memory) and return a compact text
    block of the top-k passages. Returns an 'unavailable' string on any failure."""
    try:
        from . import kb
    except Exception as e:
        return f"(kb unavailable: import failed — {e})"
    try:
        k = max(1, min(int(k or 5), 8))
        if not kb.available():
            return ("(kb unavailable: sentence-transformers / faiss-cpu not installed "
                    "— run `pip install -r maker2/requirements.txt`)")
        hits = kb.search(str(query), collection, k=k)
        if not hits:
            return (f"(kb: no matching passages for {query!r} in {collection!r} — "
                    "the index may be empty or not yet built)")
        return f"Knowledge-base passages for {query!r}:\n\n" + kb.format_hits(hits)
    except Exception as e:
        return f"(kb search unavailable for {query!r} — {e})"


def _kb_bound(collection: str):
    """A kb_search executor pinned to `collection`, so the agent always searches its
    OWN collection even if the model omits/mis-sets the collection argument."""
    def _exec(query: str, collection: str = collection, k: int = 5) -> str:
        # Ignore any model-supplied collection; pin to the caller's agent collection.
        return kb_search_exec(query, collection=_exec.__collection__, k=k)
    _exec.__collection__ = collection
    return _exec


# --------------------------------------------------------------------------- #
# Authoritative constraint solver executor (settings.enable_solver_tool) — lets an
# agent pose a small 3D point/distance problem and get libslvs' exact solution +
# consistency verdict mid-authoring. Failure-soft: any problem (py-slvs absent, bad
# args, solver error) returns a short string and NEVER raises into the tool loop.
# --------------------------------------------------------------------------- #

_CONSTRAINT_ARITY = {"coincident": 2, "distance": 2, "projected_distance": 3}


def solve_constraints_exec(points=None, constraints=None, expected_dof: int = 0) -> str:
    """Solve a point/distance constraint problem (meters) with libslvs and return a
    compact text verdict. Returns an 'unavailable'/'error' string on any failure."""
    try:
        from .constraint_solver import slvs_available, solve_problem
        from .constraint_ir import (AssemblyConstraintProblem, ConstraintKind,
                                     ConstraintSpec, EntityKind, EntitySpec)
    except Exception as e:  # pragma: no cover - import guard
        return f"(solver unavailable: import failed — {e})"

    ok, reason = slvs_available()
    if not ok:
        return (f"(solver unavailable: py-slvs backend not importable — {reason}. "
                "Install it via maker2/requirements.txt.)")

    try:
        pts = list(points or [])
        cons = list(constraints or [])
        if not pts:
            return "(solver: no points given — nothing to solve)"

        entities = []
        seen_ids = set()
        for p in pts:
            pid = str(p.get("id", "")).strip()
            xyz = tuple(float(v) for v in (p.get("xyz_m") or ())[:3])
            if not pid or len(xyz) != 3:
                return f"(solver: each point needs an id and a 3-number xyz_m; bad point {p!r})"
            if pid in seen_ids:
                return f"(solver: duplicate point id {pid!r})"
            seen_ids.add(pid)
            entities.append(EntitySpec(pid, EntityKind.POINT_3D, xyz,
                                       fixed=bool(p.get("fixed", False))))

        kind_map = {"coincident": ConstraintKind.COINCIDENT,
                    "distance": ConstraintKind.DISTANCE,
                    "projected_distance": ConstraintKind.PROJECTED_DISTANCE}
        specs = []
        for i, c in enumerate(cons):
            kind_str = str(c.get("kind", "")).strip().lower()
            if kind_str not in kind_map:
                return (f"(solver: unsupported constraint kind {kind_str!r}; "
                        f"use one of {sorted(kind_map)})")
            refs = tuple(str(r) for r in (c.get("refs") or ()))
            need = _CONSTRAINT_ARITY[kind_str]
            if len(refs) != need:
                return (f"(solver: {kind_str} needs {need} point refs, got {len(refs)})")
            missing = [r for r in refs if r not in seen_ids]
            if missing:
                return f"(solver: constraint refs not defined as points: {missing})"
            cid = str(c.get("id") or f"c{i}")
            specs.append(ConstraintSpec(cid, kind_map[kind_str], refs,
                                        float(c.get("value_m", 0.0) or 0.0)))

        problem = AssemblyConstraintProblem(
            entities=entities, constraints=specs, expected_dof=int(expected_dof or 0),
            units="m", base_id="llm_solver_tool")
        result = solve_problem(problem)
    except Exception as e:
        return f"(solver error: {type(e).__name__}: {e})"

    consistent = (result.status in ("okay", "redundant")
                  and not result.failed_constraint_ids)
    lines = [
        f"Solver result: status={result.status} dof={result.dof} "
        f"(expected_dof={int(expected_dof or 0)})",
        f"verdict: {'CONSISTENT' if consistent else 'NOT CONSISTENT'}",
    ]
    if result.failed_constraint_ids:
        lines.append("unsatisfied constraints: "
                     + ", ".join(map(str, result.failed_constraint_ids)))
    if result.dof != int(expected_dof or 0):
        lines.append(f"note: solved DOF {result.dof} != expected {int(expected_dof or 0)} "
                     "(under- or over-constrained)")
    lines.append("solved points (m):")
    for pid, xyz in result.points_m.items():
        lines.append(f"  {pid} = [{', '.join(f'{v:.6g}' for v in xyz)}]")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# Tool loop
# --------------------------------------------------------------------------- #

def run_tool_loop(client, conv, system: str, tools: list, executors: dict, *,
                  max_rounds: int = 4, log_fn=None, text_only_nudge=None) -> str:
    """Drive tool-calling: send the conversation with `tools`; for each ToolCall the
    model makes, run executors[name](**arguments) and feed the result back via
    conv.add_tool_result. Repeat until the model stops calling tools (return its text)
    or max_rounds is hit. Tool/exec failures are swallowed as an error string result so
    one bad call never breaks the loop."""
    def log(m):
        if log_fn:
            log_fn(f"[tool] {m}")

    last_text = ""
    did_any_call = False
    nudged = False
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
            # The model sometimes returns a text-only PREAMBLE ("I'll search...")
            # before actually emitting the tool call. If it hasn't searched yet and we
            # have budget, nudge it to make the call now instead of bailing (once).
            if not did_any_call and not nudged:
                nudged = True
                conv.add_user_message(text_only_nudge or
                    "You said you would search but did not call the tool. Call "
                    "web_search NOW with a concrete query — do not just describe it.")
                continue
            return last_text
        did_any_call = True
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
    "You are researching to build a mechanical CAD design well. Use the tools to look "
    "up anything genuinely useful. Prefer kb_search (the local knowledge base) for the "
    "OUTPUT FORMAT, conventions, dimension names, gear math, and worked examples; use "
    "web_search for real-world reference designs, standard part dimensions (gear "
    "modules, bearing/jewel sizes, thread specs), or materials. When exact geometry "
    "matters — gear center distances, coaxial shaft points, projected offsets — call "
    "solve_constraints (if offered) to get the solver's authoritative coordinates "
    "instead of guessing. Do a FEW focused "
    "lookups at most, then STOP (reply with a one-line 'done'). Do not design anything "
    "yet; just gather facts.")


def _research_toolset(settings, collection: str):
    """Assemble the (tools, executors) offered in a research turn, per the enabled
    flags: web_search when enable_reference_tools, kb_search (pinned to `collection`)
    when enable_kb. Returns ([], {}) when neither is on."""
    tools: list = []
    executors: dict = {}
    if getattr(settings, "enable_reference_tools", False):
        tools.append(WEB_SEARCH_TOOL)
        executors["web_search"] = web_search_exec
    if getattr(settings, "enable_kb", False):
        tools.append(KB_SEARCH_TOOL)
        executors["kb_search"] = _kb_bound(collection)
    if getattr(settings, "enable_solver_tool", False):
        from .constraint_solver import slvs_available
        if slvs_available()[0]:
            tools.append(SOLVE_CONSTRAINTS_TOOL)
            executors["solve_constraints"] = solve_constraints_exec
    return tools, executors


def maybe_research(client, conv, settings, what: str, *, collection: str = "manager",
                   log_fn=None) -> None:
    """If web and/or KB reference tools are enabled, run a short research turn on
    `conv` BEFORE the caller's normal generation, so useful facts land in the
    conversation as context. `collection` pins kb_search to the calling agent's own
    KB collection. Best-effort; never raises. `what` is a short task description."""
    tools, executors = _research_toolset(settings, collection)
    if not tools:
        return
    try:
        conv.add_user_message(
            f"Before you begin, research anything useful for this task: {what}")
        run_tool_loop(client, conv, _RESEARCH_SYSTEM, tools, executors,
                      max_rounds=4, log_fn=log_fn)
    except Exception as e:
        if log_fn:
            log_fn(f"[tool] research skipped: {e}")


def research_findings(client, settings, what: str, *, collection: str = "worker",
                      log_fn=None) -> str:
    """Like maybe_research but on a THROWAWAY conversation, returning the collected
    tool-result text (to inject as context into a caller that owns its own per-task
    conversations, e.g. the SCAD worker's per-batch convs). Empty string when both
    web and KB are disabled or on failure."""
    tools, executors = _research_toolset(settings, collection)
    if not tools:
        return ""
    try:
        from .llm.conversation import Conversation
        conv = Conversation()
        conv.add_user_message(
            f"Research standard dimensions / specs / conventions useful to build: {what}")
        run_tool_loop(client, conv, _RESEARCH_SYSTEM, tools, executors,
                      max_rounds=3, log_fn=log_fn)
        # Collect the tool_result contents (the actual retrieved text) from the convo.
        notes = [Conversation.extract_text(m.get("content", ""))
                 for m in conv.messages if m.get("role") == "tool_result"]
        return "\n".join(n for n in notes if n).strip()
    except Exception as e:
        if log_fn:
            log_fn(f"[tool] research skipped: {e}")
        return ""
