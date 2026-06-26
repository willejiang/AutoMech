#!/usr/bin/env python3
"""
render_views.py — .scad -> STL + 6 orthographic PNG views, headless, via the
NATIVE OpenSCAD CLI. Mirrors how cadam previews models in the browser
(worker/benchmarks/render.sh): BOSL2/MCAD on the library path, full --render,
the same camera framing. No browser / WASM.

It also parses the top-level `module NAME(...)` declarations out of the .scad so
the manifest author (urdf_author.py) knows which render_module names exist — the
evaluator renders each part by calling `use <model.scad>; NAME();`, so a fused
single module can't articulate.

Standalone (sidesteps the LLM — good first smoke test):
  python orchestrator/render_views.py worker/benchmarks/03-hex-bolt-and-nut.scad
  python orchestrator/render_views.py model.scad --out runs/x/iter_0
"""
import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from shutil import which

# (name, camera rot x,y,z) — the 6 the evaluator/cadam name set uses, minus ISO
# duplication. cadam's enum is ISO/FRONT/BACK/LEFT/RIGHT/TOP/BOTTOM; we render
# the 6 task-useful orthos + ISO for the judge (7 images, index 0 = ISO).
# Camera angles follow render.sh's --camera="0,0,0,rx,ry,rz,0" convention.
VIEWS = [
    ("ISO", (62, 0, 25)),
    ("FRONT", (90, 0, 0)),
    ("BACK", (90, 0, 180)),
    ("LEFT", (90, 0, 90)),
    ("RIGHT", (90, 0, 270)),
    ("TOP", (0, 0, 0)),
]

# top-level `module foo()` — not indented (nested helper modules are skipped so
# the author only sees real parts).
_MODULE_RE = re.compile(r"^module\s+([A-Za-z_]\w*)\s*\(", re.MULTILINE)


def find_openscad():
    env = os.environ.get("OPENSCAD_BIN")
    if env and Path(env).exists():
        return env
    for c in (
        "openscad",
        "/Applications/OpenSCAD (Nightly).app/Contents/MacOS/OpenSCAD",
        "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD",
        r"C:\Program Files\OpenSCAD\openscad.exe",
    ):
        p = which(c) if not Path(c).is_absolute() else (c if Path(c).exists() else None)
        if p:
            return p
    return None


def parse_modules(scad_text):
    return _MODULE_RE.findall(scad_text)


def _render_one(oscad, scad_path, rot, png_path, size=720):
    cam = f"0,0,0,{rot[0]},{rot[1]},{rot[2]},0"
    cmd = [
        oscad, "-o", str(png_path),
        f"--imgsize={size},{size}",
        "--projection=ortho",
        "--colorscheme=Tomorrow",
        "--render", "--viewall", "--autocenter",
        f"--camera={cam}",
        str(scad_path),
    ]
    return subprocess.run(cmd, capture_output=True, text=True)


def render(scad_path, out_dir):
    """-> dict(stl_path, view_pngs[], view_names[], module_names[],
              compile_ok, compile_err). On compile failure, view_pngs is [] and
    compile_err carries OpenSCAD's stderr so the loop can feed it back."""
    scad_path = Path(scad_path)
    out_dir = Path(out_dir)
    views_dir = out_dir / "views"
    views_dir.mkdir(parents=True, exist_ok=True)

    scad_text = scad_path.read_text(encoding="utf-8", errors="replace")
    modules = parse_modules(scad_text)

    oscad = find_openscad()
    if not oscad:
        return {
            "stl_path": None, "view_pngs": [], "view_names": [],
            "module_names": modules, "compile_ok": False,
            "compile_err": "OpenSCAD CLI not found. Set OPENSCAD_BIN or install openscad.",
        }

    # 1) STL (full render). First failure here == bad geometry -> feed back.
    stl_path = out_dir / "model.stl"
    r = subprocess.run(
        [oscad, "-o", str(stl_path), "--render", str(scad_path)],
        capture_output=True, text=True,
    )
    if r.returncode != 0 or not stl_path.exists():
        return {
            "stl_path": None, "view_pngs": [], "view_names": [],
            "module_names": modules, "compile_ok": False,
            "compile_err": (r.stderr or r.stdout or "openscad STL export failed").strip(),
        }

    # 2) 6+ISO orthographic PNGs.
    pngs, names, errs = [], [], []
    for name, rot in VIEWS:
        png = views_dir / f"rgb_{name.lower()}.png"
        rr = _render_one(oscad, scad_path, rot, png)
        if rr.returncode == 0 and png.exists():
            pngs.append(str(png))
            names.append(name)
        else:
            errs.append(f"{name}: {(rr.stderr or '').strip()[:200]}")

    return {
        "stl_path": str(stl_path),
        "view_pngs": pngs,
        "view_names": names,
        "module_names": modules,
        "compile_ok": len(pngs) > 0,
        "compile_err": "; ".join(errs) if errs else "",
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scad")
    ap.add_argument("--out", default=None, help="output dir (default: alongside the .scad)")
    a = ap.parse_args()
    out = a.out or str(Path(a.scad).resolve().parent / (Path(a.scad).stem + "_views"))
    res = render(a.scad, out)
    # don't dump image bytes — just the summary
    print(json.dumps({k: v for k, v in res.items()}, indent=2))
    if not res["compile_ok"]:
        print(f"\n[render] COMPILE FAILED: {res['compile_err'][:500]}", file=sys.stderr)
        sys.exit(1)
    print(f"\n[render] ok: {len(res['view_pngs'])} views, "
          f"{len(res['module_names'])} modules {res['module_names']} -> {out}")


if __name__ == "__main__":
    main()
