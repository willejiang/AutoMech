"""One-time offline ingest: chunk the authored corpus -> per-collection faiss index.

Run once (after `pip install sentence-transformers faiss-cpu` and one online run to
populate the HF cache for all-MiniLM-L6-v2):

    python -m maker2.kb.ingest              # build every collection
    python -m maker2.kb.ingest manager      # build only the manager collection

Reads markdown docs from kb/corpus/<collection>/*.md, chunks them TIGHT (200-500
tokens; golden skeletons kept whole), embeds with MiniLM, and writes
kb/index/<collection>/{vectors.faiss,chunks.jsonl} via store.build_index.

The memory_<collection> indices are NOT built here — they start empty and grow at
runtime via the memory-append hook (see kb.remember_passing_model).
"""
from __future__ import annotations

import sys
from pathlib import Path

from . import store

_CORPUS_ROOT = Path(__file__).resolve().parent / "corpus"

# Rough chars-per-token; MiniLM caps at 256 tokens/input, so keep chunks well under
# ~500 tokens (~2000 chars) and let the embedder truncate the rare overflow.
_MAX_CHARS = 1800
_MIN_CHARS = 60


def _split_section(header: str, body: str) -> list[str]:
    """Split one markdown section (already header-delimited) into <=_MAX_CHARS pieces
    on paragraph boundaries, re-prefixing the header so each piece keeps its context."""
    body = body.strip()
    if not body:
        return []
    whole = f"{header}\n{body}".strip()
    if len(whole) <= _MAX_CHARS:
        return [whole]
    pieces, cur = [], header
    for para in body.split("\n\n"):
        para = para.strip()
        if not para:
            continue
        if len(cur) + len(para) + 2 > _MAX_CHARS and cur.strip() != header.strip():
            pieces.append(cur.strip())
            cur = f"{header}\n{para}"
        else:
            cur = f"{cur}\n\n{para}" if cur.strip() != header.strip() else f"{header}\n{para}"
    if cur.strip() and cur.strip() != header.strip():
        pieces.append(cur.strip())
    return pieces


def chunk_markdown(text: str, *, source: str, keep_whole: bool) -> list[dict]:
    """Chunk one markdown doc. A `keep_whole` doc (a golden skeleton, a worked
    example) becomes a single chunk so retrieval returns the complete example; other
    docs are split by top-level (`## `) sections, then by paragraph if oversized."""
    text = text.strip()
    if not text:
        return []
    if keep_whole:
        return [{"text": text, "meta": {"source": source, "kind": "example"}}]

    # Split on level-2 headers; everything before the first `## ` is the preamble.
    lines = text.split("\n")
    sections: list[tuple[str, list[str]]] = []
    header, buf = "", []
    for ln in lines:
        if ln.startswith("## "):
            if buf or header:
                sections.append((header, buf))
            header, buf = ln, []
        else:
            buf.append(ln)
    if buf or header:
        sections.append((header, buf))

    chunks: list[dict] = []
    for hdr, body_lines in sections:
        for piece in _split_section(hdr, "\n".join(body_lines)):
            if len(piece) >= _MIN_CHARS:
                chunks.append({"text": piece, "meta": {"source": source,
                                                       "kind": "doc"}})
    return chunks


def _collection_chunks(collection: str) -> list[dict]:
    cdir = _CORPUS_ROOT / collection
    if not cdir.is_dir():
        return []
    chunks: list[dict] = []
    for md in sorted(cdir.glob("*.md")):
        keep_whole = md.name.startswith("golden_")
        chunks.extend(chunk_markdown(md.read_text(encoding="utf-8"),
                                     source=f"{collection}/{md.name}",
                                     keep_whole=keep_whole))
    return chunks


def list_collections() -> list[str]:
    if not _CORPUS_ROOT.is_dir():
        return []
    return sorted(p.name for p in _CORPUS_ROOT.iterdir() if p.is_dir())


def ingest(collections: list[str] | None = None, *, log=print) -> dict[str, int]:
    """Build the named collections (all if None). Returns {collection: n_vectors}."""
    if not store.available():
        log("[kb.ingest] sentence-transformers / faiss-cpu not installed — "
            "install them and re-run. No index written.")
        return {}
    targets = collections or list_collections()
    counts: dict[str, int] = {}
    for coll in targets:
        chunks = _collection_chunks(coll)
        if not chunks:
            log(f"[kb.ingest] {coll}: no corpus docs found; skipped.")
            continue
        n = store.build_index(coll, chunks)
        counts[coll] = n
        log(f"[kb.ingest] {coll}: {n} chunks -> {store.collection_dir(coll)}")
    return counts


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    ingest(args or None)
