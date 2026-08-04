"""PDF -> markdown for the KB corpus.

`kb.ingest` reads markdown only. This converts a PDF into one, so a reference book can be
retrieved alongside the authored docs:

    python -m maker2.kb.pdf_import book.pdf --collection manager
    python -m maker2.kb.pdf_import book.pdf --collection manager --local
    python -m maker2.kb.pdf_import book.pdf --pages 40-120 --title "507 Movements"

then re-run `python -m maker2.kb.ingest <collection>`.

WHERE IT WRITES, AND WHY THAT IS THE IMPORTANT FLAG. Default output is
`kb/corpus/<collection>/` — committed, so it must be first-party or public domain.
`--local` writes to `kb/corpus_local/`, which is gitignored, for material you personally
hold a licence to. The tool cannot tell which is which, so it prints the licence question
every run and requires the answer to be a deliberate flag rather than a default.

Public-domain sources worth having: anything US-published before 1929 is out of copyright,
which covers the classic kinematics atlases — Brown's *507 Mechanical Movements* (1868) and
Reuleaux's *Kinematics of Machinery* (1876). They are also a better fit than a modern design
text: they map MECHANISM -> FUNCTION ("crank and slotted lever produces quick return"),
which is what the machine-authoring agent has to choose between, whereas a modern text
mostly sizes a part you have already chosen.

TEXT LAYER ONLY. A scanned page with no embedded text yields nothing here; this does not
OCR. The report at the end says how many pages came back empty, so a scan-only PDF is
obvious immediately rather than after a silent, useless ingest.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

_KB_ROOT = Path(__file__).resolve().parent


def _parse_pages(spec: str | None, n_pages: int) -> list[int]:
    """"40-120", "5", "1-10,50,60-70" -> zero-based page indices (clamped)."""
    if not spec:
        return list(range(n_pages))
    out: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, _, b = part.partition("-")
            lo, hi = int(a), int(b)
        else:
            lo = hi = int(part)
        out.extend(range(max(1, lo) - 1, min(n_pages, hi)))
    return sorted(set(out))


# A hyphen at end of line is a word split across lines, not punctuation. Left in, the
# chunker embeds "recipro-" and "cating" as separate tokens and neither matches a query
# for "reciprocating".
_DEHYPHEN = re.compile(r"(\w)-\n(\w)")
# Page furniture: a line that is only a number, or "Page 12", or "12 | CHAPTER 3".
_RUNNING_HEAD = re.compile(r"^\s*(page\s+)?\d+\s*(\|.*)?$", re.I)


def clean_page(text: str) -> str:
    """Undo PDF line-wrapping artefacts. Retrieval quality depends on this more than on
    anything else here: a PDF's text layer is laid out for a page, not for a reader."""
    text = _DEHYPHEN.sub(r"\1\2", text)
    lines = [ln.rstrip() for ln in text.split("\n")]
    lines = [ln for ln in lines if not _RUNNING_HEAD.match(ln)]
    # Re-flow: a line that does not end a sentence is a wrapped continuation, so join it
    # to the next. Blank lines stay as paragraph breaks.
    out: list[str] = []
    for ln in lines:
        s = ln.strip()
        if not s:
            out.append("")
            continue
        if out and out[-1] and not out[-1][-1] in ".:;?!" and not s[0].isupper():
            out[-1] = f"{out[-1]} {s}"
        else:
            out.append(s)
    # Collapse runs of blank lines to one.
    joined = "\n".join(out)
    return re.sub(r"\n{3,}", "\n\n", joined).strip()


def pdf_to_markdown(pdf_path: Path, *, pages: str | None = None,
                    title: str | None = None) -> tuple[str, dict]:
    """Return (markdown, stats). Raises RuntimeError if pypdf is missing."""
    try:
        from pypdf import PdfReader
    except ImportError as e:                                  # pragma: no cover
        raise RuntimeError(
            "pypdf is not installed. `python -m pip install pypdf`") from e

    reader = PdfReader(str(pdf_path))
    idxs = _parse_pages(pages, len(reader.pages))
    name = title or pdf_path.stem.replace("_", " ")

    body: list[str] = []
    empty = 0
    for i in idxs:
        try:
            raw = reader.pages[i].extract_text() or ""
        except Exception:
            raw = ""
        page = clean_page(raw)
        if not page:
            empty += 1
            continue
        # One `## ` per page: ingest splits on level-2 headers, so each page becomes its
        # own retrievable chunk (further split if long), and a hit can be traced back to
        # a page number in the original book.
        body.append(f"## p.{i + 1}\n\n{page}")

    md = (f"# {name}\n\n"
          f"Imported from `{pdf_path.name}`"
          + (f", pages {pages}" if pages else "")
          + ".\n\n" + "\n\n".join(body) + "\n")
    return md, {"pages_total": len(reader.pages), "pages_read": len(idxs),
                "pages_empty": empty, "pages_kept": len(body), "chars": len(md)}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Convert a PDF into a KB markdown doc (then run kb.ingest).")
    ap.add_argument("pdf", type=Path)
    ap.add_argument("--collection", default="manager",
                    help="KB collection to write into (default: manager)")
    ap.add_argument("--local", action="store_true",
                    help="write to corpus_local/ (gitignored) instead of corpus/. Use "
                         "this for anything you hold a licence to but may not publish.")
    ap.add_argument("--pages", default=None,
                    help='page range, 1-based, e.g. "40-120" or "1-10,50"')
    ap.add_argument("--title", default=None, help="document title (default: file name)")
    ap.add_argument("--out", type=Path, default=None, help="explicit output .md path")
    a = ap.parse_args(argv)

    if not a.pdf.is_file():
        print(f"no such file: {a.pdf}")
        return 1
    md, stats = ("", {})
    try:
        md, stats = pdf_to_markdown(a.pdf, pages=a.pages, title=a.title)
    except RuntimeError as e:
        print(e)
        return 1

    # An empty page RANGE and an empty TEXT LAYER are different faults with different
    # fixes, and reporting the first as the second sends you off to OCR a PDF that was
    # never the problem.
    if stats["pages_read"] == 0:
        print(f"--pages {a.pages} selected no pages of {a.pdf.name} "
              f"(it has {stats['pages_total']}). Page numbers are 1-based.")
        return 1
    if stats["pages_kept"] == 0:
        print(f"No text extracted from {stats['pages_read']} page(s). This PDF is "
              f"probably a SCAN with no text layer — this tool does not OCR. Run it "
              f"through an OCR pass first (e.g. ocrmypdf) and try again.")
        return 1

    root = _KB_ROOT / ("corpus_local" if a.local else "corpus")
    out = a.out or (root / a.collection / f"{a.pdf.stem.lower()}.md")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(md, encoding="utf-8")

    print(f"wrote {out}")
    print(f"  {stats['pages_kept']} page(s) kept, {stats['pages_empty']} empty, "
          f"{stats['chars']} chars")
    if stats["pages_empty"] > stats["pages_kept"]:
        print("  WARNING: more pages were empty than not — likely a partial scan.")
    if a.local:
        print("  -> corpus_local/ is gitignored; this will not be committed.")
    else:
        print("  -> corpus/ IS COMMITTED. Only public-domain or first-party material "
              "belongs here (US pre-1929 is public domain). Licensed material goes in "
              "with --local.")
    print(f"\nNow run:  python -m maker2.kb.ingest {a.collection}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
