"""Wikipedia -> markdown for the KB corpus.

    python -m maker2.kb.wiki_import Gear "Four-bar linkage" Escapement
    python -m maker2.kb.wiki_import --preset mechanisms
    python -m maker2.kb.wiki_import Gear --keep-all --collection manager
    python -m maker2.kb.ingest manager

WHY WIKIPEDIA AND NOT A TEXTBOOK. Every other source tried here loses exactly the part
that matters. A scanned book's formulas come back as OCR noise ("Diametral pitch =" with
the right-hand side gone); a born-digital lecture PDF keeps the symbols but scrambles the
layout, so a fraction's numerator and denominator arrive as separate fragments. Wikipedia
is plain text with LaTeX markup already in it, so `d = N*m_n/cos(psi)` survives intact.

WHERE IT WRITES, AND WHY. Default is `corpus_local/` (gitignored), NOT the committed
corpus. Wikipedia is CC BY-SA: attribution plus a share-alike condition that can extend to
derivative works. Keeping the text on your machine sidesteps that entirely — the tool is
part of this repo, the text you pull is not. Pass --commit only if you have decided the
share-alike terms are acceptable for a published corpus; each file carries its attribution
header either way.

WHAT THIS IS GOOD FOR, AND WHAT IT IS NOT. These articles explain what a mechanism IS and
which quantities govern it — "the Grashof condition", "pitch diameter is N*m/cos(psi)".
That is real grounding for choosing a mechanism. But they are encyclopedia prose, not
build instructions: the slider-crank article defines stroke in words without ever writing
`stroke = 2 * crank_radius`. Expect this to help the agent PICK a mechanism and name its
governing quantities; expect to still hand-write the numeric relation and the
`build_machine()` skeleton yourself.
"""
from __future__ import annotations

import argparse
import json
import re
import time
import urllib.parse
import urllib.request
from pathlib import Path

_KB_ROOT = Path(__file__).resolve().parent
_API = "https://en.wikipedia.org/w/api.php"
# Wikipedia's API policy asks for a descriptive agent with contact info.
_UA = {"User-Agent": "AutoMech-KB/1.0 (+https://github.com/willejiang/AutoMech)"}

# Sections that are about the topic's past or its appearances in culture rather than how
# it works. Dropping them is most of the retrieval-noise win: "History" alone is a third
# of the Gear and Escapement articles, and it is all dates and names.
_DROP_SECTIONS = {
    "history", "etymology", "see also", "notes", "references", "external links",
    "further reading", "bibliography", "in popular culture", "gallery",
    "examples", "applications", "simulations", "gallery of escapements",
}

PRESETS: dict[str, list[str]] = {
    # Rotation, reduction and the arithmetic that governs them.
    "gears": [
        "Gear", "Gear train", "Spur gear", "Bevel gear", "Worm drive",
        "Epicyclic gearing", "Involute gear", "Rack and pinion",
        "Backlash (engineering)", "Gear ratio",
    ],
    # Turning one kind of motion into another — what to reach for, given a requirement.
    "mechanisms": [
        "Linkage (mechanical)", "Four-bar linkage", "Slider-crank linkage",
        "Crank (mechanism)", "Cam", "Cam follower", "Scotch yoke", "Geneva drive",
        "Ratchet (device)", "Escapement", "Universal joint", "Kinematic pair",
        "Degrees of freedom (mechanics)",
    ],
    # What holds a shaft, and how tightly things sit on each other.
    "shafts_fits": [
        "Bearing (mechanical)", "Rolling-element bearing", "Plain bearing",
        "Engineering fit", "Interference fit", "Axle", "Coupling",
        "Screw thread", "Leadscrew", "Ball screw",
    ],
    # Power transmission that is not a gear.
    "drives": [
        "Belt (mechanical)", "Chain drive", "Pulley", "Flywheel", "Clutch",
        "Torque", "Power transmission",
    ],
}


def _api(params: dict, tries: int = 5) -> dict:
    """GET the MediaWiki API with backoff. 429 is common from a shared IP, and the useful
    response to it is to wait, not to fail the whole run."""
    url = _API + "?" + urllib.parse.urlencode({**params, "format": "json"})
    for k in range(tries):
        try:
            req = urllib.request.Request(url, headers=_UA)
            return json.loads(urllib.request.urlopen(req, timeout=90).read())
        except Exception as e:
            if getattr(e, "code", None) == 429 and k < tries - 1:
                time.sleep(15 * (k + 1))
                continue
            raise
    return {}


def fetch(title: str) -> tuple[str, str]:
    """(canonical_title, plain_text) for one article; ('', '') if it does not exist.

    One title per call on purpose: a multi-title `titles=A|B|C` request returns a full
    extract only for the first page and empty strings for the rest, which silently loses
    most of what you asked for."""
    d = _api({"action": "query", "prop": "extracts", "explaintext": 1,
              "redirects": 1, "titles": title})
    pages = (d.get("query") or {}).get("pages") or {}
    if not pages:
        return "", ""
    page = list(pages.values())[0]
    if "missing" in page:
        return "", ""
    return page.get("title", title), page.get("extract", "") or ""


def to_markdown(title: str, text: str, *, keep_all: bool = False) -> tuple[str, dict]:
    """Wikipedia's plain-text dump -> markdown, one `## ` per kept section.

    The dump marks sections as `== Name ==` / `=== Sub ===`. Ingest splits on `## `, so
    mapping every heading (at any depth) to `## ` makes each section its own retrievable
    chunk instead of one 65k-character blob."""
    lines = text.split("\n")
    out: list[str] = []
    kept, dropped = [], []
    keeping = True
    body: list[str] = []

    def flush():
        while body and not body[-1].strip():
            body.pop()

    for ln in lines:
        m = re.match(r"^(=+)\s*(.+?)\s*\1$", ln.strip())
        if m:
            name = m.group(2).strip()
            keeping = keep_all or name.lower() not in _DROP_SECTIONS
            (kept if keeping else dropped).append(name)
            if keeping:
                flush()
                body.append("")
                body.append(f"## {name}")
                body.append("")
            continue
        if keeping:
            body.append(ln)

    flush()
    # CC BY-SA requires attribution and a licence pointer; put it in the file itself so a
    # doc that gets copied around keeps its provenance.
    url = "https://en.wikipedia.org/wiki/" + urllib.parse.quote(title.replace(" ", "_"))
    head = (f"# {title}\n\n"
            f"Source: Wikipedia, [{title}]({url}) — CC BY-SA 4.0. Retrieved via the\n"
            f"MediaWiki API by `maker2.kb.wiki_import`; section headings preserved,\n"
            f"non-technical sections dropped.\n")
    md = head + "\n".join(body).rstrip() + "\n"
    md = re.sub(r"\n{3,}", "\n\n", md)
    return md, {"kept": kept, "dropped": dropped, "chars": len(md)}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Pull Wikipedia engineering articles into the KB corpus.")
    ap.add_argument("titles", nargs="*", help="article titles")
    ap.add_argument("--preset", choices=sorted(PRESETS),
                    action="append", default=[],
                    help="a curated title list (repeatable); 'all' expands every preset")
    ap.add_argument("--collection", default="manager")
    ap.add_argument("--commit", action="store_true",
                    help="write to the COMMITTED corpus/ instead of the gitignored "
                         "corpus_local/. Wikipedia is CC BY-SA (share-alike); only pass "
                         "this if those terms are acceptable for a published corpus.")
    ap.add_argument("--keep-all", action="store_true",
                    help="keep History/Etymology/See-also too (default drops them)")
    ap.add_argument("--delay", type=float, default=2.0, help="seconds between requests")
    a = ap.parse_args(argv)

    titles = list(a.titles)
    for p in a.preset:
        titles += PRESETS[p]
    if not titles:
        print("Nothing to fetch. Give titles or --preset. Available presets:")
        for k, v in PRESETS.items():
            print(f"  {k:12} {len(v):2} articles: {', '.join(v[:4])}...")
        return 1
    seen, ordered = set(), []
    for t in titles:
        if t.lower() not in seen:
            seen.add(t.lower())
            ordered.append(t)

    root = _KB_ROOT / ("corpus" if a.commit else "corpus_local")
    outdir = root / a.collection
    outdir.mkdir(parents=True, exist_ok=True)

    total, ok, missing = 0, 0, []
    for i, t in enumerate(ordered):
        try:
            canon, text = fetch(t)
        except Exception as e:
            print(f"  {t:34} FAILED ({type(e).__name__}: {e})")
            missing.append(t)
            continue
        if not text:
            print(f"  {t:34} NOT FOUND")
            missing.append(t)
            continue
        md, st = to_markdown(canon, text, keep_all=a.keep_all)
        slug = re.sub(r"[^a-z0-9]+", "_", canon.lower()).strip("_")
        (outdir / f"wiki_{slug}.md").write_text(md, encoding="utf-8")
        total += st["chars"]
        ok += 1
        print(f"  {canon:34} {st['chars']:7,} chars  "
              f"({len(st['kept'])} sections, {len(st['dropped'])} dropped)")
        if i < len(ordered) - 1:
            time.sleep(a.delay)

    print(f"\n{ok}/{len(ordered)} article(s) -> {outdir}")
    print(f"{total:,} characters (~{total // 4:,} tokens)")
    if missing:
        print(f"not retrieved: {', '.join(missing)}")
    if a.commit:
        print("WROTE TO THE COMMITTED CORPUS. Wikipedia is CC BY-SA — attribution is in "
              "each file; satisfy yourself that share-alike is acceptable here.")
    else:
        print("corpus_local/ is gitignored; this text stays on your machine.")
    print(f"\nNow run:  python -m maker2.kb.ingest {a.collection}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
