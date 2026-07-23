"""方案B: thin wrapper around the text-to-cad CAD skill's `scripts/inspect` for deterministic,
single-STEP selector-level precision checks — the "补精量" half of the verification layer.

This does NOT replace geocheck. geocheck owns the cross-subassembly / semantic / boolean-clash
judgements (shaft-through-bore containment, functional-vs-structural interference, sign-aware
realized-axis vs params-axis) that the skill's inspect cannot express. inspect only measures
distance / alignment between two selectors inside ONE STEP file, so it complements geocheck with
per-part precision (a gear face flush to a station, a bore coaxial with a shaft) — it never decides
cross-sub faults.

The skill is an optional local install. When it is not present, every call degrades gracefully to
``{"available": False, ...}`` so the pipeline runs unchanged (geocheck still gates). Nothing here
raises for a missing skill.
"""
from __future__ import annotations

import glob
import json
import os
import subprocess
import sys
from pathlib import Path

_INSPECT_TIMEOUT = 60


def _skill_dir() -> Path | None:
    """Locate the CAD skill's base directory (the one holding `scripts/inspect`).

    Order: explicit CAD_SKILL_DIR env var, then the plugin cache glob (highest version wins).
    Returns None if nothing usable is found — callers then degrade gracefully.
    """
    env = os.environ.get("CAD_SKILL_DIR")
    if env:
        p = Path(env)
        if (p / "scripts" / "inspect").exists():
            return p
    home = Path.home()
    pattern = str(home / ".claude" / "plugins" / "cache" / "text-to-cad" / "cad" / "*" / "skills" / "cad")
    cands = [Path(c) for c in glob.glob(pattern) if (Path(c) / "scripts" / "inspect").exists()]
    if not cands:
        return None
    # highest version dir (…/cad/<version>/skills/cad) sorts correctly by the version segment
    cands.sort(key=lambda c: c.parts)
    return cands[-1]


def available() -> bool:
    """True if the CAD skill's inspect tool can be located."""
    return _skill_dir() is not None


def _run(skill: Path, args: list) -> dict:
    """Run `scripts/inspect <args>` in the skill dir, parse the last JSON line. sys.executable is
    the same interpreter the pipeline runs under (has build123d + cadpy in 方案B)."""
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        r = subprocess.run(
            [sys.executable, str(skill / "scripts" / "inspect"), *args],
            capture_output=True, text=True, timeout=_INSPECT_TIMEOUT,
            cwd=str(skill), env=env)
    except subprocess.TimeoutExpired:
        return {"available": True, "ok": False, "error": "inspect timed out"}
    except Exception as e:
        return {"available": True, "ok": False, "error": f"{type(e).__name__}: {e}"}
    out = (r.stdout or "").strip()
    # inspect emits pretty-printed (multi-line) JSON for refs and compact JSON for measure/align.
    # Try the whole stdout first (covers multi-line), then fall back to the last JSON-looking line.
    if out:
        try:
            d = json.loads(out)
            if isinstance(d, dict):
                d.setdefault("available", True)
                return d
        except Exception:
            pass
    for line in reversed(out.splitlines()):
        line = line.strip()
        if line.startswith("{"):
            try:
                d = json.loads(line)
                d.setdefault("available", True)
                return d
            except Exception:
                continue
    return {"available": True, "ok": False,
            "error": (r.stderr or out or "no JSON from inspect")[-300:]}


def entry_facts(step_path: str) -> dict:
    """Whole-entry facts (bounds, size, center, face/edge counts) for a STEP — no selector needed.
    Returns {available: False} if the skill is absent. This is the cheapest sub-level sanity probe:
    a real non-empty solid with a plausible bounding box."""
    skill = _skill_dir()
    if skill is None:
        return {"available": False}
    d = _run(skill, ["refs", str(step_path), "--facts"])
    if not d.get("ok"):
        return d
    toks = d.get("tokens") or []
    if not toks:
        return {"available": True, "ok": False, "error": "no tokens"}
    t = toks[0]
    return {"available": True, "ok": True,
            "summary": t.get("summary", {}),
            "entryFacts": t.get("entryFacts", {})}


def measure(step_path: str, from_sel: str, to_sel: str, axis: str | None = None) -> dict:
    """Signed coordinate distance between two selectors in ONE STEP (e.g. a gear face to a shaft
    station). Returns {available: False} if the skill is absent."""
    skill = _skill_dir()
    if skill is None:
        return {"available": False}
    args = ["measure", str(step_path), "--from", from_sel, "--to", to_sel, "--format", "json"]
    if axis:
        args += ["--axis", axis]
    return _run(skill, args)


def align(step_path: str, moving_sel: str, target_sel: str,
          mode: str = "center", axis: str | None = None) -> dict:
    """Read-only translation delta for a flush/center alignment of two selectors in ONE STEP
    (e.g. a bore coaxial with a shaft). Returns {available: False} if the skill is absent."""
    skill = _skill_dir()
    if skill is None:
        return {"available": False}
    args = ["align", str(step_path), "--moving", moving_sel, "--target", target_sel,
            "--mode", mode, "--format", "json"]
    if axis:
        args += ["--axis", axis]
    return _run(skill, args)
