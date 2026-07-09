#!/usr/bin/env python3
"""Two-part (NOTES -> JSON) streaming with cap-cut recovery, shared by the boss and
the manager.

Both agents ask a large-output model for a single JSON object. On a big machine that
overruns the gateway output cap, the OLD recovery was to ask for a SMALLER result
(fewer subassemblies / a coarser decomposition) — which folds away shafts, bearings,
and internals: exactly the detail we want to keep.

Instead we make the agent respond in TWO parts:

    <plaintext NOTES: the plan / reasoning>
    === JSON ===
    { ...the single JSON object... }

and recover a cap cut WITHOUT shrinking:

  * cut while still in the NOTES (no ``{`` seen yet)  -> CONTINUE the notes (feed the
    model its own partial, ask it to keep going) until the notes finish or a small
    budget runs out;
  * cut AFTER the JSON started (a ``{`` is present)   -> the notes are complete, so
    DISCARD the partial JSON and REGENERATE the whole JSON from scratch in a FRESH
    conversation primed by the saved notes (so the model spends its entire budget
    emitting JSON, not re-reasoning).

The notes are persisted to a per-agent ``*_memory.md`` scratch file (in the run dir,
NOT the long-term memory system) so the reasoning survives the cut and the regen.

The single ``{`` is the phase boundary: ``jsonutil.extract_json_object`` scans from
the first ``{``, so "is there a ``{`` in the accumulated text" cleanly tells NOTES
from JSON, and the returned notes+json text parses through the existing ``parse_*``
unchanged (they skip the notes prefix).
"""
from __future__ import annotations

from pathlib import Path

from .jsonutil import extract_json_object
from .llm.conversation import Conversation

JSON_SENTINEL = "=== JSON ==="

# Inner budgets so a pathological stream can't loop forever. The OUTER agent retry
# loop (settings.manager_retries) still bounds total attempts + content repair.
_MAX_NOTES_CONTINUE = 3          # continuation rounds for the NOTES phase
_MAX_JSON_CONTINUE = 4           # brace-continuation rounds during the JSON regen

CONTINUE_NOTES = (
    "Your NOTES were cut off before you finished. Continue them from EXACTLY where "
    "they stopped — do NOT restart or repeat what you already wrote, and do NOT begin "
    "the JSON yet. Keep planning until the notes are complete, then emit a line with "
    "exactly `=== JSON ===` followed by the single JSON object.")


def _save(memory_path, notes: str) -> None:
    """Overwrite the scratch memory file with the current notes (best-effort)."""
    if not memory_path or not notes:
        return
    try:
        Path(memory_path).parent.mkdir(parents=True, exist_ok=True)
        Path(memory_path).write_text(notes, encoding="utf-8")
    except OSError:
        pass


def _notes_of(accum: str) -> str:
    """The NOTES portion = everything before the first ``{`` (the JSON start)."""
    i = accum.find("{")
    return accum if i == -1 else accum[:i]


def _heartbeat(log_fn, tag: str, phase: str):
    """A char-counting on_delta callback: logs a throttled progress line every
    ~1200 chars so the UI stage dot spins while the agent streams."""
    state = {"chars": 0, "mark": 0}

    def _beat(delta: str) -> None:
        state["chars"] += len(delta)
        if log_fn and state["chars"] - state["mark"] >= 1200:
            state["mark"] = state["chars"]
            log_fn(f"[{tag}] …{phase} ({state['chars']} chars)")

    return _beat


def stream_two_part(client, conv: Conversation, system: str, *,
                    memory_path=None, regen_msg_fn, log_fn=None, tag: str = "") -> str:
    """Stream a NOTES-then-JSON response from ``conv``, recovering from cap cuts, and
    return the full text (notes + JSON) ready for the caller's ``parse_*``.

    ``regen_msg_fn(notes) -> str`` builds the agent-specific "here are your notes,
    now output ONLY the JSON" message used if the JSON is cut.
    """
    accum = ""
    for _ in range(_MAX_NOTES_CONTINUE + 1):
        prev_len = len(accum)
        messages = conv.get_messages_for_api(api_style=client.api_style)
        text, finish = client.send_collect(
            messages, system=system, on_delta=_heartbeat(log_fn, tag, "writing plan"))
        accum = (accum + text) if accum else text
        has_json = "{" in accum
        made_progress = len(accum) - prev_len > 20   # real new content this round

        if finish == "length" and not has_json:
            if not made_progress:
                # The round burned the cap but produced ~no visible content — the
                # model spent the whole budget on hidden thinking. Continuing would
                # just repeat that. Stop and regenerate the JSON from whatever notes
                # we have (thinking is bounded once effort is set on the client).
                notes = _notes_of(accum)
                _save(memory_path, notes)
                if log_fn:
                    log_fn(f"[{tag}] a notes round produced no content (cap spent on "
                           f"hidden thinking); generating the JSON now")
                return _regen_json(client, system, notes, regen_msg_fn, log_fn, tag)
            # Cut mid-NOTES with real progress: save, feed the model its own partial,
            # and ask it to keep writing the notes. Loop (bounded).
            _save(memory_path, accum)
            conv.add_assistant_message(text)
            if log_fn:
                log_fn(f"[{tag}] notes hit the output cap; continuing the notes")
            conv.add_user_message(CONTINUE_NOTES)
            continue

        if finish == "length" and has_json:
            # Cut mid-JSON: the notes are complete. Discard the partial JSON and
            # regenerate the whole JSON from scratch, primed by the notes.
            notes = _notes_of(accum)
            _save(memory_path, notes)
            if log_fn:
                log_fn(f"[{tag}] JSON hit the output cap; regenerating the JSON "
                       f"from the saved plan")
            return _regen_json(client, system, notes, regen_msg_fn, log_fn, tag)

        # finish == "stop" (or anything non-length): the response completed.
        if not has_json:
            # The model finished politely after writing ONLY notes (or an empty reply) and
            # never emitted the `{` — returning this as-is makes the caller's parse throw
            # "no JSON object found" and burns the whole attempt. The notes ARE the plan, so
            # regenerate the JSON from them (same path as a mid-JSON cap cut) instead.
            notes = _notes_of(accum)
            _save(memory_path, notes)
            if log_fn:
                log_fn(f"[{tag}] response ended with notes but no JSON; generating the "
                       f"JSON from the plan")
            return _regen_json(client, system, notes, regen_msg_fn, log_fn, tag)
        _save(memory_path, _notes_of(accum))
        return accum

    # Notes-continuation budget exhausted without ever reaching the JSON. Treat the
    # notes as done and regenerate the JSON from them so we always make progress.
    notes = _notes_of(accum)
    _save(memory_path, notes)
    if log_fn:
        log_fn(f"[{tag}] notes still open after {_MAX_NOTES_CONTINUE} continuations; "
               f"generating the JSON from the plan so far")
    return _regen_json(client, system, notes, regen_msg_fn, log_fn, tag)


def _regen_json(client, system: str, notes: str, regen_msg_fn, log_fn, tag: str) -> str:
    """Regenerate ONLY the JSON in a FRESH conversation seeded with the saved notes.

    If the JSON is ALSO cut here, fall back to brace completion-continuation (keep the
    partial, ask the model to continue) until ``extract_json_object`` balances or the
    budget runs out — then return whatever we have (the caller's parse will reject an
    incomplete object and the outer retry fires)."""
    conv = Conversation()
    conv.add_user_message(regen_msg_fn(notes))
    accum = ""
    for _ in range(_MAX_JSON_CONTINUE + 1):
        messages = conv.get_messages_for_api(api_style=client.api_style)
        text, finish = client.send_collect(
            messages, system=system, on_delta=_heartbeat(log_fn, tag, "writing JSON"))
        accum = (accum + text) if accum else text
        conv.add_assistant_message(text)
        if _json_complete(accum):
            return accum
        if finish != "length":
            # Model stopped on its own but the JSON isn't balanced — one nudge, then
            # give up and let the caller's parse/validate fail and retry.
            return accum
        if log_fn:
            log_fn(f"[{tag}] JSON output capped again; continuing where it stopped")
        conv.add_user_message(
            "The JSON object was cut off before it closed. Continue it from EXACTLY "
            "where it stopped — output only the remaining JSON text, no repetition, "
            "no prose, no fences, so the object closes correctly.")
    return accum


def _json_complete(text: str) -> bool:
    """True if ``text`` contains a fully-balanced ``{...}`` object."""
    try:
        extract_json_object(text)
        return True
    except ValueError:
        return False
