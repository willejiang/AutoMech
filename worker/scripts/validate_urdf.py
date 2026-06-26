#!/usr/bin/env python3
"""Validate a URDF: XML well-formedness, link/joint graph integrity, mesh
file resolution, and physical sanity (positive mass, inertia triangle
inequality). Pure-Python, only needs trimesh (already installed).

Usage:
  python scripts/validate_urdf.py [path/to/robot.urdf]
Default: the .urdf inside the newest *_urdf folder in ~/Downloads.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh


def find_urdf(arg: str | None) -> Path:
    if arg:
        p = Path(arg).expanduser()
        if not p.is_file():
            sys.exit(f"URDF not found: {p}")
        return p
    downloads = Path.home() / "Downloads"
    dirs = sorted(
        downloads.glob("*_urdf"), key=lambda d: d.stat().st_mtime, reverse=True
    )
    for d in dirs:
        urdfs = list(d.glob("*.urdf"))
        if urdfs:
            return urdfs[0]
    sys.exit(f"No *_urdf/*.urdf found in {downloads}")


def main() -> None:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    urdf = find_urdf(sys.argv[1] if len(sys.argv) > 1 else None)
    base = urdf.parent
    print("=" * 56)
    print("URDF check:", urdf)
    print("=" * 56)

    fails = 0

    try:
        root = ET.parse(urdf).getroot()
        print("[OK] XML well-formed | robot =", root.get("name"))
    except Exception as e:
        sys.exit(f"[FAIL] XML parse: {e}")

    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [l.get("name") for l in links]
    name_set = set(link_names)
    print(f"[OK] {len(links)} links, {len(joints)} joints")

    dups = {n for n in link_names if link_names.count(n) > 1}
    if dups:
        print(f"[FAIL] duplicate link names: {dups}")
        fails += 1
    else:
        print("[OK] link names unique")

    dangling = []
    for j in joints:
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        if p not in name_set:
            dangling.append(("parent", j.get("name"), p))
        if c not in name_set:
            dangling.append(("child", j.get("name"), c))
    if dangling:
        print(f"[FAIL] dangling joint refs: {dangling}")
        fails += 1
    else:
        print("[OK] every joint parent/child link exists")

    children = {j.find("child").get("link") for j in joints}
    roots = name_set - children
    if roots == {"base_link"}:
        print("[OK] single root: base_link")
    else:
        print(f"[WARN] root link(s): {roots}")

    # mesh files resolve (URDF filenames are relative to the URDF dir).
    missing = []
    for m in root.iter("mesh"):
        fn = m.get("filename")
        candidate = (base / fn) if not Path(fn).is_absolute() else Path(fn)
        if not candidate.is_file():
            missing.append(fn)
    if missing:
        print(f"[FAIL] missing mesh files: {missing}")
        fails += 1
    else:
        print(f"[OK] all {sum(1 for _ in root.iter('mesh'))} mesh refs resolve")

    # physical sanity
    print("-" * 56)
    masses = []
    for l in links:
        inert = l.find("inertial")
        if inert is None:
            continue
        mass = float(inert.find("mass").get("value"))
        I = inert.find("inertia")
        ixx, iyy, izz = (float(I.get(k)) for k in ("ixx", "iyy", "izz"))
        if l.find("visual") is not None:
            masses.append(mass)
        if mass <= 0:
            print(f"  [FAIL] {l.get('name')}: mass <= 0")
            fails += 1
        if min(ixx, iyy, izz) <= 0:
            print(f"  [FAIL] {l.get('name')}: non-positive inertia")
            fails += 1
        if not (ixx + iyy >= izz and iyy + izz >= ixx and ixx + izz >= iyy):
            print(f"  [WARN] {l.get('name')}: inertia triangle inequality")
    if masses:
        print(
            f"  part mass: {min(masses):.4f} .. {max(masses):.4f} kg "
            f"| total {sum(masses):.3f} kg"
        )
    if fails == 0:
        print("  [OK] all inertia tensors physically valid")

    # mesh geometry
    print("-" * 56)
    nonwater = 0
    total = 0
    for m in root.iter("mesh"):
        fn = m.get("filename")
        candidate = (base / fn) if not Path(fn).is_absolute() else Path(fn)
        try:
            mesh = trimesh.load(candidate, force="mesh")
            total += 1
            if not mesh.is_watertight:
                nonwater += 1
        except Exception as e:
            print(f"  [WARN] couldn't load {fn}: {e}")
    print(
        f"  {nonwater}/{total} parts non-watertight "
        "(open shells; fine for visual, use convex hull for collision)"
    )

    print("=" * 56)
    if fails == 0:
        print("RESULT: PASS — URDF is structurally and physically loadable.")
    else:
        print(f"RESULT: {fails} hard failure(s) — fix before loading.")
        sys.exit(1)


if __name__ == "__main__":
    main()
