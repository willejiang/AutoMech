"""One-time offline ingest: chunk the authored corpus -> per-collection faiss index.

Run once (after `pip install sentence-transformers faiss-cpu` and one online run to
populate the HF cache for all-MiniLM-L6-v2):

    python -m maker2.kb.ingest              # build every collection
    python -m maker2.kb.ingest manager      # build only the manager collection

Reads markdown docs from kb/corpus/<collection>/*.md, chunks them TIGHT (200-500
tokens; golden skeletons kept whole), embeds with MiniLM, and writes
kb/index/<collection>/{vectors.faiss,chunks.jsonl} via store.build_index.

YOUR OWN REFERENCE MATERIAL goes in `kb/corpus_local/<collection>/*.md` instead, which
is gitignored. Both roots are ingested into the same index, so a local doc is retrieved
exactly like a first-party one — it just never enters the repository. That separation is
the point: this is an open-source project, so the committed corpus stays first-party and
license-clean, while whatever you are personally allowed to read (a CC-licensed course,
a textbook you own, your employer's internal standards) stays on your machine. See
`corpus_local/README.md`, written on first run.

The memory_<collection> indices are NOT built here — they start empty and grow at
runtime via the memory-append hook (see kb.remember_passing_model).
"""
from __future__ import annotations

import sys
from pathlib import Path

from . import store

_CORPUS_ROOT = Path(__file__).resolve().parent / "corpus"
# Gitignored twin of _CORPUS_ROOT. Same layout (one directory per collection).
_LOCAL_ROOT = Path(__file__).resolve().parent / "corpus_local"

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


def _docs_of(root: Path, collection: str, origin: str) -> list[dict]:
    """Chunk every *.md under one corpus root's collection directory.

    `origin` ("corpus" / "corpus_local") is recorded in each chunk's source string, so a
    retrieved hit says where it came from. When a local doc turns out to be wrong, that
    label is what lets you find it — a poisoned index is otherwise a haystack (this repo
    already has one `memory_manager.poisoned.bak` from exactly that problem)."""
    cdir = root / collection
    if not cdir.is_dir():
        return []
    chunks: list[dict] = []
    for md in sorted(cdir.rglob("*.md")):
        if md.name.upper() == "README.MD":
            continue                       # the how-to note, not reference material
        rel = md.relative_to(cdir).as_posix()
        chunks.extend(chunk_markdown(md.read_text(encoding="utf-8", errors="replace"),
                                     source=f"{origin}:{collection}/{rel}",
                                     keep_whole=md.name.startswith("golden_")))
    return chunks


def _collection_chunks(collection: str) -> list[dict]:
    """First-party corpus + the gitignored local corpus, in that order."""
    return (_docs_of(_CORPUS_ROOT, collection, "corpus")
            + _docs_of(_LOCAL_ROOT, collection, "corpus_local"))


def list_collections() -> list[str]:
    """Collections present in EITHER root, so a local-only collection still builds."""
    names = set()
    for root in (_CORPUS_ROOT, _LOCAL_ROOT):
        if root.is_dir():
            names |= {p.name for p in root.iterdir()
                      if p.is_dir() and not p.name.startswith((".", "_"))}
    return sorted(names)


_LOCAL_README = """\
# Your own reference material (gitignored — nothing here is committed)

Drop markdown here and `python -m maker2.kb.ingest` folds it into the same index the
agents search. Layout mirrors `../corpus/`: one directory per collection.

    corpus_local/
      manager/     what the machine-authoring agent reads   <- most useful
      boss/        subassembly decomposition
      worker/      geometry idioms
      evaluator/   physics-test design
      analyzer/    failure diagnosis
      compiler/    assembly/solver notes

A PDF is converted first (`python -m maker2.kb.pdf_import book.pdf --collection manager
--local`), which writes the markdown here for you.

A file named `golden_*.md` is kept as ONE chunk, so a complete worked example comes back
whole instead of in fragments. Anything else is split by `## ` section.

## What is worth putting here

The agent writes `build_machine()` — one build123d script. Reference material helps when
it is something the agent can ACT on:

- a mechanism and the numeric relation that defines it ("crank-slider: stroke = 2 x crank
  length"), ideally with a short `build_machine()` skeleton;
- what a real machine of some class actually contains, and what carries what;
- standard sizes and fits you want it to reuse.

Prose it cannot act on mostly adds retrieval noise. Prefer short, checkable statements.

## Please keep it verifiable

A wrong line in here is worse than a missing one: it comes back to the agent looking
authoritative, and it will build to it without question. This repo already carries a
`memory_manager.poisoned.bak` from exactly that failure — cleaning a bad index meant
throwing the whole thing away. Favour claims physics can contradict (a ratio, a stroke, a
centre distance) over unfalsifiable ones ("bearings are usually 6204").

## Licensing is on you

This directory is gitignored precisely so this project's committed corpus stays
first-party and license-clean. What you put here is your own call under whatever terms
you hold the material — note that "open" is not "unconditional": CC BY-SA is viral,
CC BY-NC forbids commercial use, and every CC licence requires attribution.

Facts and ideas are not copyrightable in the first place; the expression of them is. So
writing the mechanism and its numbers in your own words is both safer and more useful
here than pasting paragraphs.
"""


def _ensure_local_root(*, log=print) -> None:
    """Create corpus_local/ with its README the first time ingest runs. Making the
    directory exist is what makes the feature discoverable — a documented path nobody
    has ever seen is a path nobody uses."""
    try:
        readme = _LOCAL_ROOT / "README.md"
        if not readme.exists():
            _LOCAL_ROOT.mkdir(parents=True, exist_ok=True)
            readme.write_text(_LOCAL_README, encoding="utf-8")
            log(f"[kb.ingest] created {_LOCAL_ROOT} (gitignored) — drop your own "
                f"reference markdown there; see its README.md")
    except OSError as e:
        log(f"[kb.ingest] could not create {_LOCAL_ROOT}: {e}")


def ingest(collections: list[str] | None = None, *, log=print) -> dict[str, int]:
    """Build the named collections (all if None). Returns {collection: n_vectors}."""
    if not store.available():
        log("[kb.ingest] sentence-transformers / faiss-cpu not installed — "
            "install them and re-run. No index written.")
        return {}
    _ensure_local_root(log=log)
    targets = collections or list_collections()
    counts: dict[str, int] = {}
    for coll in targets:
        chunks = _collection_chunks(coll)
        if not chunks:
            log(f"[kb.ingest] {coll}: no corpus docs found; skipped.")
            continue
        n = store.build_index(coll, chunks)
        counts[coll] = n
        # Always say how much of this index is NOT first-party. An index is a black box
        # once built, and "how much of what the agent reads did I not write?" is the
        # question you need answered before you trust — or have to clean — it.
        local = sum(1 for c in chunks
                    if str((c.get("meta") or {}).get("source", "")).startswith("corpus_local:"))
        extra = f" ({n - local} first-party + {local} local)" if local else ""
        log(f"[kb.ingest] {coll}: {n} chunks{extra} -> {store.collection_dir(coll)}")
    return counts


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    ingest(args or None)
