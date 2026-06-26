#!/usr/bin/env python3
"""Split a wind-turbine STL into tower + rotor and emit a spinnable URDF.

A horizontal-axis wind turbine is one welded mesh of: a vertical TOWER, a
nacelle/hub, and 3 blades. To make it spin we need exactly two bodies — the
fixed tower (base_link) and the ROTOR (hub + 3 blades as one rigid body) —
joined by a CONTINUOUS joint about the rotor's spin axis (the +X axis through
the hub), so the blades can rotate without limit.

Segmentation is geometric: the tower is the central vertical column (near the
X≈0, Y≈0 axis); everything else — hub, blades, the nacelle/met-mast that sits
off to the +X side — is the rotor. The rotor mesh is translated so the spin
axis passes through the child-frame origin, so driving the joint spins the
blades about the correct centre instead of flinging them off.

Material is inferred by Claude (claude-opus-4.8 via the local gateway).

Usage:
  python scripts/turbine_to_urdf.py [path/to/turbine.stl] [--no-llm]
      [--axis x|y|z] [--input-unit mm]
Default input: newest STL whose name looks like a turbine/windmill in
~/Downloads (matches 风车/风力/turbine/windmill), else the newest *.stl.
Output goes to worker/output/<name>_urdf to avoid other tools overwriting it.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import os
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
import trimesh

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

GATEWAY_DEFAULT = os.environ.get("LLM_GATEWAY_URL", "http://localhost:8313").rstrip("/")
MODEL_ID = "claude-opus-4.8"
SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_ROOT = SCRIPT_DIR.parent / "output"

FALLBACK = {
    "tower": ((0.78, 0.80, 0.82, 1.0), 7850.0, 0.6),   # steel
    "rotor": ((0.92, 0.93, 0.95, 1.0), 1600.0, 0.4),   # composite blades
}


def find_input_stl(cli: str | None) -> Path:
    if cli:
        p = Path(cli).expanduser()
        if not p.is_file():
            sys.exit(f"STL not found: {p}")
        return p
    downloads = Path.home() / "Downloads"
    pat = re.compile(r"风车|风力|风机|turbine|windmill|wind", re.I)
    hits = sorted(
        [f for f in downloads.glob("*.stl") if pat.search(f.name)],
        key=lambda f: f.stat().st_mtime,
        reverse=True,
    )
    if hits:
        return hits[0]
    stls = sorted(downloads.glob("*.stl"), key=lambda f: f.stat().st_mtime, reverse=True)
    if not stls:
        sys.exit(f"No .stl in {downloads}")
    return stls[0]


def cap_holes(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    m = mesh.copy()
    try:
        m.fill_holes()
    except Exception:
        pass
    m.remove_unreferenced_vertices()
    return m


def detect_axis_and_split(mesh: trimesh.Trimesh, axis_override: str | None):
    """Return (axis_index, hub_center, labels) where labels[i] in {'tower',
    'rotor'} for each face.

    The vertical axis is the model's tallest extent (the tower stands up). The
    SPIN axis is horizontal — whichever of the two remaining axes the hub
    pokes out along (the rotor's off-centre mass). The tower is the central
    column near the vertical line through the base centroid.
    """
    v = mesh.vertices
    lo = v.min(0)
    hi = v.max(0)
    size = hi - lo
    vert = int(np.argmax(size))  # vertical axis (tower height)
    horiz = [a for a in range(3) if a != vert]

    fc = mesh.triangles.mean(axis=1)
    # Base centroid in the horizontal plane = tower column centre line.
    base = v[v[:, vert] < lo[vert] + 0.25 * size[vert]]
    col = np.array([base[:, horiz[0]].mean(), base[:, horiz[1]].mean()])

    # Nacelle bottom: a height above which everything is rotor (hub+blades).
    nac_bot = lo[vert] + 0.58 * size[vert]
    # Column half-widths (tower thickness) from the base slab.
    cw0 = max((base[:, horiz[0]].max() - base[:, horiz[0]].min()) * 0.7, size[horiz[0]] * 0.18)
    cw1 = max((base[:, horiz[1]].max() - base[:, horiz[1]].min()) * 0.7, size[horiz[1]] * 0.18)

    labels = np.empty(len(fc), dtype=object)
    for i, c in enumerate(fc):
        h0, h1, z = c[horiz[0]], c[horiz[1]], c[vert]
        in_column = abs(h0 - col[0]) < cw0 and abs(h1 - col[1]) < cw1
        if in_column and z < nac_bot:
            labels[i] = "tower"
        elif z >= nac_bot:
            labels[i] = "rotor"
        elif in_column:
            labels[i] = "tower"
        else:
            labels[i] = "rotor"

    # Hub centre = mean of rotor verts near the spin axis. The spin axis is the
    # horizontal axis along which the rotor's centroid is most offset from the
    # tower column.
    rotor_v = v[np.unique(mesh.faces[np.where(labels == "rotor")[0]])]
    offset = [abs(rotor_v[:, h].mean() - (col[k])) for k, h in enumerate(horiz)]
    spin = horiz[int(np.argmax(offset))] if axis_override is None else "xyz".index(axis_override)
    if axis_override is not None:
        spin = "xyz".index(axis_override)

    # Hub centre: on the spin axis, at the rotor's vertical centre of mass and
    # lateral centre (the other horizontal coord ≈ column line).
    other = [a for a in horiz if a != spin][0]
    hub = np.zeros(3)
    hub[spin] = rotor_v[:, spin].mean()
    hub[other] = col[horiz.index(other)] if other in horiz else 0.0
    # vertical hub height = mid of the dense nacelle band, approximated by the
    # rotor verts' median vertical (robust to a few stray low faces).
    hub[vert] = np.median(rotor_v[rotor_v[:, spin] > rotor_v[:, spin].mean()][:, vert]) \
        if (rotor_v[:, spin] > rotor_v[:, spin].mean()).any() else rotor_v[:, vert].mean()
    return spin, vert, hub, labels


def render_segment_png(mesh, size=300):
    fig = plt.figure(figsize=(size / 100, size / 100), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    n = mesh.face_normals
    shade = 0.45 + 0.55 * ((n[:, 2] + 1.0) / 2.0)
    cols = np.stack([0.3 * shade, 0.55 * shade, 0.85 * shade, np.ones_like(shade)], 1)
    ax.add_collection3d(
        Poly3DCollection(mesh.vertices[mesh.faces], facecolor=cols, edgecolor="none")
    )
    b = mesh.bounds
    ax.set_xlim(b[0, 0], b[1, 0])
    ax.set_ylim(b[0, 1], b[1, 1])
    ax.set_zlim(b[0, 2], b[1, 2])
    try:
        ax.set_box_aspect(b[1] - b[0])
    except Exception:
        pass
    ax.view_init(elev=20, azim=-60)
    ax.set_axis_off()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight", pad_inches=0)
    plt.close(fig)
    return buf.getvalue()


def gateway_chat(gateway, messages, max_tokens=2500, retries=2):
    body = json.dumps(
        {"model": MODEL_ID, "max_tokens": max_tokens, "messages": messages}
    ).encode()
    last = None
    for _ in range(retries + 1):
        req = urllib.request.Request(
            f"{gateway}/v1/chat/completions",
            data=body,
            headers={"Content-Type": "application/json", "Authorization": "Bearer local"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=120) as resp:
            data = json.loads(resp.read())
        ch = data.get("choices") or []
        if ch and ch[0].get("message", {}).get("content"):
            return ch[0]["message"]["content"]
        last = data
    raise KeyError(f"gateway returned no choices: {last}")


def classify_materials(gateway, segments):
    names = list(segments.keys())
    content = [
        {
            "type": "text",
            "text": (
                "This is a wind turbine split into 2 parts for URDF, shown below "
                f"in order: {names} (tower = the mast/base; rotor = hub + blades). "
                "For EACH part give a realistic material as JSON keyed by the exact "
                'name:\n{"tower": {"material": "...", "color": [r,g,b,a], '
                '"density_kg_m3": <n>, "friction": <0..1>}, ...}\n'
                "color 0..1 floats. No prose, no markdown."
            ),
        }
    ]
    for name in names:
        content.append({"type": "text", "text": f"{name}:"})
        b64 = base64.b64encode(render_segment_png(segments[name])).decode()
        content.append(
            {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{b64}"}}
        )
    raw = gateway_chat(gateway, [{"role": "user", "content": content}])
    text = re.sub(r"^```(?:json)?\s*", "", raw.strip())
    text = re.sub(r"\s*```$", "", text).strip()
    try:
        return json.loads(text[text.find("{") :])
    except Exception:
        return {}


def material_for(label, parsed):
    fb_color, fb_density, fb_friction = FALLBACK[label]
    item = parsed.get(label, {}) if isinstance(parsed.get(label), dict) else {}
    color = item.get("color", fb_color)
    try:
        color = [float(c) for c in color][:4]
        if len(color) == 3:
            color.append(1.0)
        assert len(color) == 4
    except Exception:
        color = list(fb_color)
    try:
        density = float(item.get("density_kg_m3", fb_density))
        assert density > 0
    except Exception:
        density = fb_density
    try:
        friction = float(item.get("friction", fb_friction))
    except Exception:
        friction = fb_friction
    return {"material": str(item.get("material", label)), "color": color,
            "density": density, "friction": friction}


def box_inertia(mass, extents):
    x, y, z = (max(float(e), 1e-4) for e in extents)
    return (mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0,
            mass * (x * x + y * y) / 12.0)


def f(x):
    return f"{x:.6g}"


def link_lines(name, mesh_rel, com, mass, extents, color):
    ixx, iyy, izz = box_inertia(mass, extents)
    r, g, b, a = color
    return [
        f'  <link name="{name}">',
        "    <visual>",
        '      <origin xyz="0 0 0" rpy="0 0 0"/>',
        f'      <geometry><mesh filename="{mesh_rel}"/></geometry>',
        f'      <material name="{name}_mat"><color rgba="{f(r)} {f(g)} {f(b)} {f(a)}"/></material>',
        "    </visual>",
        f'    <collision><origin xyz="0 0 0"/><geometry><mesh filename="{mesh_rel}"/></geometry></collision>',
        "    <inertial>",
        f'      <mass value="{f(mass)}"/>',
        f'      <origin xyz="{f(com[0])} {f(com[1])} {f(com[2])}"/>',
        f'      <inertia ixx="{f(ixx)}" ixy="0" ixz="0" iyy="{f(iyy)}" iyz="0" izz="{f(izz)}"/>',
        "    </inertial>",
        "  </link>",
    ]


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="Wind-turbine STL -> spinnable URDF")
    ap.add_argument("stl", nargs="?")
    ap.add_argument("--gateway", default=GATEWAY_DEFAULT)
    ap.add_argument("--no-llm", action="store_true")
    ap.add_argument("--input-unit", choices=["mm", "cm", "m"], default="mm")
    ap.add_argument("--axis", choices=["x", "y", "z"], default=None,
                    help="override the rotor spin axis (default: auto-detect)")
    args = ap.parse_args()

    stl_path = find_input_stl(args.stl)
    robot = re.sub(r"[^A-Za-z0-9_]+", "_", stl_path.stem).strip("_").lower() or "turbine"
    # Non-ASCII stems (e.g. Chinese) collapse to ''; fall back.
    if not re.search(r"[a-z0-9]", robot):
        robot = "wind_turbine"
    print(f"[1/5] Input: {stl_path}")

    mesh = trimesh.load(stl_path, force="mesh")
    to_m = {"mm": 0.001, "cm": 0.01, "m": 1.0}[args.input_unit]
    if to_m != 1.0:
        mesh.apply_scale(to_m)

    spin, vert, hub, labels = detect_axis_and_split(mesh, args.axis)
    axis_vec = [0, 0, 0]
    axis_vec[spin] = 1
    print(f"[2/5] Spin axis = {'XYZ'[spin]} ({axis_vec}), hub at "
          f"({hub[0]:.3g},{hub[1]:.3g},{hub[2]:.3g}) | vertical = {'XYZ'[vert]}")

    segments = {}
    for lab in ("tower", "rotor"):
        idx = np.where(labels == lab)[0]
        if len(idx) == 0:
            sys.exit(f"[FAIL] no faces for '{lab}' — try --axis or check the model")
        segments[lab] = cap_holes(mesh.submesh([idx], append=True))
    print(f"[2/5] tower faces={len(np.where(labels=='tower')[0])} "
          f"rotor faces={len(np.where(labels=='rotor')[0])}")

    if args.no_llm:
        print("[3/5] --no-llm; fallback materials.")
        parsed = {}
    else:
        print(f"[3/5] Classifying materials with {MODEL_ID} ...")
        try:
            parsed = classify_materials(args.gateway, segments)
            for k in ("tower", "rotor"):
                print(f"        {k}: {material_for(k, parsed)['material']}")
        except (urllib.error.URLError, urllib.error.HTTPError, KeyError) as e:
            print(f"      Claude failed ({e}); fallback.")
            parsed = {}

    out_dir = OUTPUT_ROOT / f"{robot}_urdf"
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    lines = ['<?xml version="1.0"?>', f'<robot name="{robot}">', ""]

    # base_link = tower, frame == world (mesh not translated).
    tower = segments["tower"]
    tower.export(mesh_dir / "tower.stl")
    tmat = material_for("tower", parsed)
    tcom = tower.center_mass if tower.is_volume else tower.centroid
    tvol = abs(tower.volume) if tower.is_volume else float(np.prod(tower.extents))
    tmass = max(tvol * tmat["density"], 1e-4)
    lines += link_lines("base_link", "meshes/tower.stl", tcom, tmass, tower.extents, tmat["color"])
    lines.append("")

    # rotor: translate so the hub (spin-axis point) sits at the child origin.
    rotor = segments["rotor"].copy()
    rotor.apply_translation(-hub)
    rotor.export(mesh_dir / "rotor.stl")
    rmat = material_for("rotor", parsed)
    rcom = rotor.center_mass if rotor.is_volume else rotor.centroid
    rvol = abs(rotor.volume) if rotor.is_volume else float(np.prod(rotor.extents))
    rmass = max(rvol * rmat["density"], 1e-4)
    lines += link_lines("rotor", "meshes/rotor.stl", rcom, rmass, rotor.extents, rmat["color"])
    lines.append("")

    # continuous joint about the spin axis, origin at the hub (world coords,
    # since base_link frame == world).
    axis_str = " ".join(str(a) for a in axis_vec)
    lines += [
        '  <joint name="rotor_spin" type="continuous">',
        '    <parent link="base_link"/>',
        '    <child link="rotor"/>',
        f'    <origin xyz="{f(hub[0])} {f(hub[1])} {f(hub[2])}" rpy="0 0 0"/>',
        f'    <axis xyz="{axis_str}"/>',
        '    <dynamics damping="0.05" friction="0.1"/>',
        "  </joint>",
        "",
    ]

    lines.append("</robot>")
    print(f"[4/5] Exported tower + rotor meshes to {mesh_dir}")

    urdf_path = out_dir / f"{robot}.urdf"
    urdf_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[5/5] URDF written: {urdf_path} (1 continuous joint — blades spin)")
    print(f"Done. Bundle: {out_dir}")


if __name__ == "__main__":
    main()
