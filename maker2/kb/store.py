"""Local offline vector store: MiniLM embeddings + a flat faiss index per collection.

This is the single I/O layer every KB consumer goes through — ingest (writes the
curated indices), kb_search (reads them), and the memory-append hook (writes the
growing memory indices). It is deliberately dependency-lazy and failure-soft:

  * `sentence-transformers` and `faiss-cpu` are imported ONLY when an operation
    actually needs them (settings.enable_kb gates the callers). If they are not
    installed, reads return [] and writes are no-ops — the pipeline runs fine
    without RAG, so a missing dependency must never raise into an agent loop.
  * Everything runs on CPU, offline. `HF_HUB_OFFLINE=1` / `TRANSFORMERS_OFFLINE=1`
    are set on import so the embedder never reaches out to the network at runtime;
    the model is expected to be in the local HF cache (downloaded once at ingest).

On-disk layout (one directory per collection under kb/index/):
    kb/index/<collection>/vectors.faiss   — a flat inner-product index
    kb/index/<collection>/chunks.jsonl    — one {"text","meta"} row per vector,
                                            index-aligned with the faiss rows.

Vectors are L2-normalized before add/search, so inner product == cosine similarity.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

# Keep the embedder fully offline at runtime (the model is cached at ingest time).
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")

# The MiniLM sentence embedder — small (~80MB), CPU-friendly, 384-dim output.
_MODEL_NAME = "sentence-transformers/all-MiniLM-L6-v2"
_EMBED_DIM = 384

# kb/index/ lives beside this file so it moves with the package.
_INDEX_ROOT = Path(__file__).resolve().parent / "index"

# Cache the loaded model + opened indices across calls in one process.
_model = None
_index_cache: dict[str, tuple] = {}   # collection -> (faiss_index, [chunk_dict, ...])


def index_root() -> Path:
    return _INDEX_ROOT


def collection_dir(collection: str) -> Path:
    return _INDEX_ROOT / collection


def available() -> bool:
    """True if both runtime deps import. Cheap to call; used to short-circuit."""
    try:
        import faiss  # noqa: F401
        import sentence_transformers  # noqa: F401
        return True
    except Exception:
        return False


def _get_model():
    """Load (once) the MiniLM embedder. Returns None if unavailable."""
    global _model
    if _model is not None:
        return _model
    try:
        from sentence_transformers import SentenceTransformer
        _model = SentenceTransformer(_MODEL_NAME, device="cpu")
    except Exception:
        _model = None
    return _model


def embed(texts: list[str]):
    """Embed a list of strings to L2-normalized float32 vectors (numpy [N, dim]).
    Returns None if the embedder is unavailable."""
    model = _get_model()
    if model is None or not texts:
        return None
    import numpy as np
    vecs = model.encode(list(texts), convert_to_numpy=True,
                        normalize_embeddings=True, show_progress_bar=False)
    return np.asarray(vecs, dtype="float32")


# --------------------------------------------------------------------------- #
# Write path (used by ingest + memory-append)
# --------------------------------------------------------------------------- #

def build_index(collection: str, chunks: list[dict]) -> int:
    """Create (overwrite) a collection index from `chunks` (each {"text","meta"}).
    Returns the number of vectors written, or 0 if deps/embeddings are unavailable."""
    texts = [c["text"] for c in chunks if c.get("text", "").strip()]
    chunks = [c for c in chunks if c.get("text", "").strip()]
    if not texts:
        return 0
    vecs = embed(texts)
    if vecs is None:
        return 0
    import faiss
    idx = faiss.IndexFlatIP(_EMBED_DIM)
    idx.add(vecs)
    cdir = collection_dir(collection)
    cdir.mkdir(parents=True, exist_ok=True)
    faiss.write_index(idx, str(cdir / "vectors.faiss"))
    with open(cdir / "chunks.jsonl", "w", encoding="utf-8") as f:
        for c in chunks:
            f.write(json.dumps({"text": c["text"], "meta": c.get("meta", {})},
                               ensure_ascii=False) + "\n")
    _index_cache.pop(collection, None)   # invalidate any cached handle
    return len(texts)


def append_chunks(collection: str, chunks: list[dict]) -> int:
    """Append `chunks` to an existing collection (create it if absent). Used by the
    memory-append hook so a passing run's skeletons accumulate. Returns count added,
    or 0 on any failure (never raises — this runs inside the boss score block)."""
    try:
        chunks = [c for c in chunks if c.get("text", "").strip()]
        if not chunks:
            return 0
        vecs = embed([c["text"] for c in chunks])
        if vecs is None:
            return 0
        import faiss
        cdir = collection_dir(collection)
        cdir.mkdir(parents=True, exist_ok=True)
        fpath = cdir / "vectors.faiss"
        if fpath.exists():
            idx = faiss.read_index(str(fpath))
        else:
            idx = faiss.IndexFlatIP(_EMBED_DIM)
        idx.add(vecs)
        faiss.write_index(idx, str(fpath))
        with open(cdir / "chunks.jsonl", "a", encoding="utf-8") as f:
            for c in chunks:
                f.write(json.dumps({"text": c["text"], "meta": c.get("meta", {})},
                                   ensure_ascii=False) + "\n")
        _index_cache.pop(collection, None)
        return len(chunks)
    except Exception:
        return 0


# --------------------------------------------------------------------------- #
# Read path (used by kb_search)
# --------------------------------------------------------------------------- #

def _load(collection: str):
    """Open (and cache) a collection's (faiss_index, chunks). Returns None if the
    index is missing or unreadable."""
    if collection in _index_cache:
        return _index_cache[collection]
    cdir = collection_dir(collection)
    fpath, cpath = cdir / "vectors.faiss", cdir / "chunks.jsonl"
    if not (fpath.exists() and cpath.exists()):
        return None
    try:
        import faiss
        idx = faiss.read_index(str(fpath))
        chunks = []
        with open(cpath, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    chunks.append(json.loads(line))
        handle = (idx, chunks)
        _index_cache[collection] = handle
        return handle
    except Exception:
        return None


def search(query: str, collections: list[str], k: int = 5) -> list[dict]:
    """Search one or more collections for `query`; return the top-k chunks overall as
    [{"text","meta","score","collection"}], best-first. Missing collections are
    skipped. Returns [] if the embedder is unavailable or nothing matches."""
    qv = embed([query])
    if qv is None:
        return []
    hits: list[dict] = []
    for coll in collections:
        loaded = _load(coll)
        if loaded is None:
            continue
        idx, chunks = loaded
        if idx.ntotal == 0:
            continue
        n = min(k, idx.ntotal)
        scores, ids = idx.search(qv, n)
        for score, i in zip(scores[0].tolist(), ids[0].tolist()):
            if 0 <= i < len(chunks):
                hits.append({"text": chunks[i]["text"],
                             "meta": chunks[i].get("meta", {}),
                             "score": float(score), "collection": coll})
    hits.sort(key=lambda h: h["score"], reverse=True)
    return hits[:k]
