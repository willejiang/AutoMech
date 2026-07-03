#!/usr/bin/env python3
"""Render ONE OpenSCAD module to an STL via the native OpenSCAD CLI.

The maker2 SCAD worker emits a single .scad with one top-level `module <link>()`
per link. To get a separate mesh per link we render each module in isolation by
emitting a tiny wrapper that `use`s the model and calls just that module — the
same technique evaluator/run_eval.py:render_parts uses. OpenSCAD discovery mirrors
orchestrator/render_views.py:find_openscad (env override, Nightly, PATH, etc.).
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path
from shutil import which

# top-level `module foo()` declarations (not indented => real parts, not helpers)
_MODULE_RE = re.compile(r"^module\s+([A-Za-z_]\w*)\s*\(", re.MULTILINE)


def find_openscad():
    env = os.environ.get("OPENSCAD_BIN")
    if env and Path(env).exists():
        return env
    for c in (
        "openscad",
        r"C:\Program Files\OpenSCAD (Nightly)\openscad.exe",
        r"C:\Program Files\OpenSCAD\openscad.exe",
        "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD",
    ):
        if Path(c).is_absolute():
            if Path(c).exists():
                return c
        else:
            p = which(c)
            if p:
                return p
    return None


def parse_modules(scad_text):
    return _MODULE_RE.findall(scad_text)


def trim_to_complete_modules(scad_text: str) -> tuple[str, str]:
    """Split a (possibly truncated) .scad into (complete_prefix, trailing_partial).

    Scans top-level brace depth: `complete_prefix` is everything up to and including
    the last point where depth returned to 0 (a fully-closed module); anything after
    is an unfinished module the model was mid-writing when it got cut off. Used for
    completion-continuation so we resume AFTER the last whole module and never keep a
    half-written one. Ignores braces inside // and /* */ comments and "..." strings."""
    depth = 0
    last_complete = 0          # index just past the last depth->0 close
    i, n = 0, len(scad_text)
    in_line_comment = in_block_comment = in_string = False
    while i < n:
        c = scad_text[i]
        two = scad_text[i:i + 2]
        if in_line_comment:
            if c == "\n":
                in_line_comment = False
        elif in_block_comment:
            if two == "*/":
                in_block_comment = False
                i += 2
                continue
        elif in_string:
            if c == "\\":
                i += 2
                continue
            if c == '"':
                in_string = False
        elif two == "//":
            in_line_comment = True
            i += 2
            continue
        elif two == "/*":
            in_block_comment = True
            i += 2
            continue
        elif c == '"':
            in_string = True
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth <= 0:
                depth = 0
                last_complete = i + 1
        i += 1
    return scad_text[:last_complete], scad_text[last_complete:]



def render_module(oscad, scad_path, module_name, stl_path):
    """Render a single module to STL. Returns True on success.

    Emits `use <model.scad>; <module>();` into a wrapper and exports binstl
    (explicit --export-format; 2026 OpenSCAD rejects bare `-o x.stl --render`).
    """
    scad_path = Path(scad_path)
    stl_path = Path(stl_path)
    stl_path.parent.mkdir(parents=True, exist_ok=True)
    wrapper = stl_path.parent / f"_wrap_{module_name}.scad"
    wrapper.write_text(f"use <{scad_path.as_posix()}>\n{module_name}();\n",
                       encoding="utf-8")
    r = subprocess.run(
        [oscad, "-o", str(stl_path), "--export-format", "binstl", str(wrapper)],
        capture_output=True, text=True,
    )
    wrapper.unlink(missing_ok=True)
    return r.returncode == 0 and stl_path.exists() and stl_path.stat().st_size > 0


def render_module_err(oscad, scad_path, module_name, stl_path):
    """Like render_module but returns (ok, stderr) for feeding errors back."""
    scad_path = Path(scad_path)
    stl_path = Path(stl_path)
    stl_path.parent.mkdir(parents=True, exist_ok=True)
    wrapper = stl_path.parent / f"_wrap_{module_name}.scad"
    wrapper.write_text(f"use <{scad_path.as_posix()}>\n{module_name}();\n",
                       encoding="utf-8")
    r = subprocess.run(
        [oscad, "-o", str(stl_path), "--export-format", "binstl", str(wrapper)],
        capture_output=True, text=True,
    )
    wrapper.unlink(missing_ok=True)
    ok = r.returncode == 0 and stl_path.exists() and stl_path.stat().st_size > 0
    return ok, (r.stderr or r.stdout or "").strip()
