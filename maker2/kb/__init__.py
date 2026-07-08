"""maker2.kb — local offline RAG for the boss/manager/worker/evaluator agents.

Public surface:
  * COLLECTIONS                 — the valid per-agent collection names.
  * search(query, collection)   — retrieve top-k chunks from the curated collection
                                  AND its growing memory (memory_<collection>).
  * remember_passing_model(...) — the memory-append hook: persist a passing run's
                                  skeleton so future runs can retrieve it.
  * format_hits(hits)           — render hits as a compact text block for a prompt.

Everything is failure-soft: if sentence-transformers / faiss-cpu are not installed
(settings.enable_kb gates the callers), `search` returns [] and `remember_*` is a
no-op. Nothing here raises into an agent loop.
"""
from __future__ import annotations

from . import store

# The four curated agent collections (built by kb.ingest). A run's growing memory
# lives in a parallel "memory_<collection>" index that starts empty.
COLLECTIONS = ("manager", "boss", "worker", "evaluator")


def _memory_name(collection: str) -> str:
    return f"memory_{collection}"


def available() -> bool:
    """True if the RAG deps import (cheap). Callers still work when this is False —
    search returns [] — so this is only for logging / short-circuit."""
    return store.available()


def search(query: str, collection: str, k: int = 5) -> list[dict]:
    """Retrieve the top-k chunks for `query` across the curated `collection` and its
    memory index together, best-first. Unknown collection or missing index -> []."""
    if not query or not query.strip():
        return []
    colls = [collection, _memory_name(collection)]
    return store.search(query, colls, k=k)


def format_hits(hits: list[dict], *, max_chars: int = 2400) -> str:
    """Render retrieved chunks as a compact, source-labeled text block for injection
    into a prompt. Truncates to max_chars so a big example set cannot blow the
    conversation budget (get_messages_for_api truncates oldest at ~100k chars)."""
    if not hits:
        return ""
    out, used = [], 0
    for i, h in enumerate(hits, 1):
        src = (h.get("meta") or {}).get("source") or h.get("collection", "kb")
        body = h.get("text", "").strip()
        block = f"[{i}] ({src})\n{body}"
        if used + len(block) > max_chars and out:
            break
        out.append(block)
        used += len(block)
    return "\n\n".join(out)


def remember_passing_model(model, *, collection: str = "manager", score=None,
                           note: str = "", log_fn=None) -> int:
    """Memory-append hook (plan C9/C10 + B.5). Persist a PASSING run's design so
    future runs retrieve it. Appends, per subassembly-shaped model, a chunk holding
    the model's parts + placement summary + a one-line 'what worked' note to
    memory_<collection>. Returns the number of chunks added (0 on any failure or when
    deps are unavailable). NEVER raises — this runs inside the boss score block.

    `model` is a KinematicModel (or an object exposing name/links/poses); `note` is
    the evaluator's memory_note when available.
    """
    try:
        chunk = _model_to_memory_chunk(model, score=score, note=note)
        if not chunk:
            return 0
        n = store.append_chunks(_memory_name(collection), [chunk])
        if log_fn and n:
            log_fn(f"[kb] remembered a passing design in {_memory_name(collection)} "
                   f"(score={score})")
        return n
    except Exception as e:
        if log_fn:
            log_fn(f"[kb] memory append skipped: {e}")
        return 0


def remember_passing_subs(subs, *, collection: str = "manager", score=None,
                          note: str = "", log_fn=None) -> int:
    """Convenience for the boss score block: remember every subassembly of a passing
    run. `subs` is a {sub_id: KinematicModel} mapping (or an iterable of models).
    Returns the total chunks added. Never raises.

    Track 3 fills the `# TRACK1: memory append hook here` marker after the ACCEPT
    block with, e.g.::

        from . import kb
        if kb.available():
            kb.remember_passing_subs(subs, score=s_val,
                                     note=getattr(judge_verdict, "memory_note", ""),
                                     log_fn=log)
    """
    try:
        models = subs.values() if hasattr(subs, "values") else (subs or [])
        total = 0
        for m in models:
            total += remember_passing_model(m, collection=collection, score=score,
                                            note=note, log_fn=log_fn)
        return total
    except Exception as e:
        if log_fn:
            log_fn(f"[kb] memory append skipped: {e}")
        return 0


def _model_to_memory_chunk(model, *, score=None, note: str = "") -> dict | None:
    """Summarize a KinematicModel into one retrievable memory chunk (text + meta).
    Kept compact (parts + poses + the note) so it stays a small passage."""
    name = getattr(model, "name", "") or "machine"
    links = list(getattr(model, "links", []) or [])
    poses = list(getattr(model, "poses", []) or [])
    if not links:
        return None
    lines = [f"WORKING DESIGN: {name}"]
    if note:
        lines.append(f"what worked: {note}")
    if score is not None:
        lines.append(f"score: {round(float(score), 3)}")
    lines.append("PARTS:")
    for l in links:
        dof = getattr(l, "dof", "fixed")
        shape = getattr(l, "shape_hint", "")
        size = getattr(l, "size_mm", {})
        drv = " driver" if getattr(l, "driver", False) else ""
        lines.append(f"  - {getattr(l, 'name', '?')} [{shape} {dof}{drv}] {size}")
    mesh = list(getattr(model, "mesh_pairs", []) or [])
    if mesh:
        lines.append(f"mesh_pairs: {mesh}")
    if poses:
        lines.append("PLACEMENTS (parent -> child @ xyz_m):")
        for p in poses:
            par = getattr(p, "parent", "") or "(root)"
            lines.append(f"  - {par} -> {getattr(p, 'child', '?')} "
                         f"@ {tuple(round(v, 4) for v in getattr(p, 'xyz_m', ()))}")
    text = "\n".join(lines)
    return {"text": text, "meta": {"source": f"memory/{name}", "kind": "memory",
                                   "score": (round(float(score), 3)
                                             if score is not None else None)}}
