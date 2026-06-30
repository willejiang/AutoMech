"""Shared helper: pull a strict-JSON object out of a chatty LLM reply.

Both the manager (decomposition JSON) and the judger (verdict JSON) ask the model
for a single bare JSON object but get fences or prose anyway, so they share one
brace-depth scanner instead of each carrying a copy.
"""

from __future__ import annotations


def extract_json_object(text: str) -> str:
    """Pull the first balanced ``{...}`` object out of an LLM response.

    Tolerates ```json fences and surrounding prose by scanning brace depth
    (ignoring braces inside strings) from the first ``{`` to its match.
    """
    t = text.strip()
    start = t.find("{")
    if start == -1:
        raise ValueError("no JSON object found in the response")
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(t)):
        c = t[i]
        if in_str:
            if esc:
                esc = False
            elif c == "\\":
                esc = True
            elif c == '"':
                in_str = False
        else:
            if c == '"':
                in_str = True
            elif c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    return t[start:i + 1]
    raise ValueError("unbalanced braces -- JSON object never closes")
