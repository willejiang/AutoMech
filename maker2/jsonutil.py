"""Shared helper: pull a strict-JSON object out of a chatty LLM reply.

Both the manager (decomposition JSON) and the judger (verdict JSON) ask the model
for a single bare JSON object but get fences or prose anyway, so they share one
brace-depth scanner instead of each carrying a copy.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from typing import Any


def json_safe_value(value: Any) -> Any:
    """Return a JSON-compatible value with non-finite numbers represented as null.

    Python's JSON encoder emits NaN and Infinity by default, even though neither token is
    valid JSON.  CLI events cross a strict ``JSON.parse`` boundary in the frontend, so
    normalize those values before serialization rather than producing an unreadable event.
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(key): json_safe_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe_value(item) for item in value]
    return value


def strict_json_dumps(value: Any, **kwargs: Any) -> str:
    """Serialize standards-compliant JSON, replacing nested NaN/Infinity with null."""
    return json.dumps(json_safe_value(value), allow_nan=False, **kwargs)


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
