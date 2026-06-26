#!/usr/bin/env python3
"""Turn a single-mesh quadruped STL (e.g. Robot_Dog.stl) into a walkable URDF.

A connected-component split is useless here: the dog is one welded mesh. So we
segment it by GEOMETRY into a torso + 4 legs, each leg cut into thigh + shank,
giving a hip and a knee revolute joint per leg (8 movable joints total). Claude
(claude-opus-4.8, via the local gateway) confirms the per-segment material.

Kinematics (quadruped, legs swing in the sagittal X-Z plane):
  base_link (torso)
    └─ <leg>_thigh   — hip  joint, revolute about Y, at the thigh top
         └─ <leg>_shank — knee joint, revolute about Y, at the thigh/shank seam

Each child link's mesh is translated so the joint origin sits at (0,0,0) in the
child frame — otherwise driving the joint rotates the mesh about the wrong point
and the leg flies off.

Cut planes are derived from the model's own geometry (front/back split, left/
right symmetry plane, hip line, knee line) but exposed as CLI flags for tuning.

Usage:
  python scripts/dog_to_urdf.py [path/to/dog.stl] [--no-llm]
      [--hip-frac 0.55] [--knee-frac 0.30]
Default input: newest *dog*/*Dog* STL in ~/Downloads, else newest *.stl.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
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

import os

GATEWAY_DEFAULT = os.environ.get("LLM_GATEWAY_URL", "http://localhost:8313").rstrip("/")
MODEL_ID = "claude-opus-4.8"

LEG_KEYS = [
    ("front", "left"),
    ("front", "right"),
    ("back", "left"),
    ("back", "right"),
]

# Per-segment fallback material (link_name_substring -> (color rgba, density,
# friction)). Feet/shanks get grippy rubber; torso a painted metal.
FALLBACK = {
    "torso": ((0.25, 0.35, 0.55, 1.0), 1200.0, 0.5),
    "thigh": ((0.30, 0.55, 0.85, 1.0), 1100.0, 0.6),
    "shank": ((0.12, 0.12, 0.14, 1.0), 1300.0, 1.0),
}


def find_input_stl(cli: str | None) -> Path:
    if cli:
        p = Path(cli).expanduser()
        if not p.is_file():
            sys.exit(f"STL not found: {p}")
        return p
    downloads = Path.home() / "Downloads"
    dog = sorted(
        [f for f in downloads.glob("*.stl") if re.search(r"dog", f.name, re.I)],
        key=lambda f: f.stat().st_mtime,
        reverse=True,
    )
    if dog:
        return dog[0]
    stls = sorted(downloads.glob("*.stl"), key=lambda f: f.stat().st_mtime, reverse=True)
    if not stls:
        sys.exit(f"No .stl in {downloads}")
    return stls[0]


def segment_label(centroid, x_split, y_split, hip_z, knee_z):
    x, y, z = centroid
    if z >= hip_z:
        return "torso"
    fb = "front" if x > x_split else "back"
    lr = "left" if y > y_split else "right"
    seg = "thigh" if z >= knee_z else "shank"
    return f"{fb}_{lr}_{seg}"


def split_by_label(mesh: trimesh.Trimesh, labels: list[str]) -> dict:
    """Build one submesh per label from face assignments, keeping geometry."""
    out = {}
    labels_arr = np.array(labels)
    for lab in sorted(set(labels)):
        face_idx = np.where(labels_arr == lab)[0]
        if len(face_idx) == 0:
            continue
        sub = mesh.submesh([face_idx], append=True)
        out[lab] = sub
    return out


def cap_holes(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Fill open boundaries left by the cut so inertia/collision are sane."""
    m = mesh.copy()
    try:
        m.fill_holes()
    except Exception:
        pass
    m.remove_unreferenced_vertices()
    return m


def render_segment_png(mesh: trimesh.Trimesh, size: int = 300) -> bytes:
    fig = plt.figure(figsize=(size / 100, size / 100), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    n = mesh.face_normals
    shade = 0.45 + 0.55 * ((n[:, 2] + 1.0) / 2.0)
    cols = np.stack(
        [0.25 * shade, 0.5 * shade, 0.85 * shade, np.ones_like(shade)], axis=1
    )
    pc = Poly3DCollection(mesh.vertices[mesh.faces], facecolor=cols, edgecolor="none")
    ax.add_collection3d(pc)
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


def gateway_chat(gateway, messages, max_tokens=3000) -> str:
    body = json.dumps(
        {"model": MODEL_ID, "max_tokens": max_tokens, "messages": messages}
    ).encode()
    req = urllib.request.Request(
        f"{gateway}/v1/chat/completions",
        data=body,
        headers={"Content-Type": "application/json", "Authorization": "Bearer local"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=120) as resp:
        return json.loads(resp.read())["choices"][0]["message"]["content"]


def classify_materials(gateway, segments: dict) -> dict:
    """Ask Claude for a material per segment. Returns {label: {...}}."""
    names = list(segments.keys())
    content = [
        {
            "type": "text",
            "text": (
                "This is a walking quadruped robot dog, segmented for URDF into "
                f"{len(names)} parts shown below in order: {names}. For EACH part, "
                "give a realistic material as JSON.\n\n"
                "Respond with ONLY a JSON object keyed by the exact part name:\n"
                '{"front_left_thigh": {"material": "...", "color": [r,g,b,a], '
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
    raw = gateway_chat(gateway, [{"role": "user", "content": content}], max_tokens=2500)
    text = re.sub(r"^```(?:json)?\s*", "", raw.strip())
    text = re.sub(r"\s*```$", "", text).strip()
    try:
        start = text.find("{")
        parsed = json.loads(text[start:])
    except Exception:
        parsed = {}
    return parsed if isinstance(parsed, dict) else {}


def material_for(label: str, parsed: dict) -> dict:
    seg = "torso" if "torso" in label else ("thigh" if "thigh" in label else "shank")
    fb_color, fb_density, fb_friction = FALLBACK[seg]
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
    return {
        "material": str(item.get("material", seg)),
        "color": color,
        "density": density,
        "friction": friction,
    }


def box_inertia(mass, extents):
    x, y, z = (max(float(e), 1e-4) for e in extents)
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def f(x):
    return f"{x:.6g}"


def link_block(name, mesh_rel, com_local, mass, extents, color):
    ixx, iyy, izz = box_inertia(mass, extents)
    r, g, b, a = color
    return [
        f'  <link name="{name}">',
        "    <visual>",
        '      <origin xyz="0 0 0" rpy="0 0 0"/>',
        f"      <geometry><mesh filename=\"{mesh_rel}\"/></geometry>",
        f'      <material name="{name}_mat">',
        f'        <color rgba="{f(r)} {f(g)} {f(b)} {f(a)}"/>',
        "      </material>",
        "    </visual>",
        "    <collision>",
        '      <origin xyz="0 0 0" rpy="0 0 0"/>',
        f"      <geometry><mesh filename=\"{mesh_rel}\"/></geometry>",
        "    </collision>",
        "    <inertial>",
        f'      <mass value="{f(mass)}"/>',
        f'      <origin xyz="{f(com_local[0])} {f(com_local[1])} {f(com_local[2])}" rpy="0 0 0"/>',
        f'      <inertia ixx="{f(ixx)}" ixy="0" ixz="0" iyy="{f(iyy)}" iyz="0" izz="{f(izz)}"/>',
        "    </inertial>",
        "  </link>",
    ]


def main() -> None:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    ap = argparse.ArgumentParser(description="Quadruped STL -> walkable URDF")
    ap.add_argument("stl", nargs="?")
    ap.add_argument("--gateway", default=GATEWAY_DEFAULT)
    ap.add_argument("--no-llm", action="store_true")
    ap.add_argument("--input-unit", choices=["mm", "cm", "m"], default="mm")
    ap.add_argument("--hip-frac", type=float, default=0.55, help="leg/torso split, frac of height")
    ap.add_argument("--knee-frac", type=float, default=0.30, help="thigh/shank split, frac of height")
    ap.add_argument("--joint-limit-deg", type=float, default=75.0)
    args = ap.parse_args()

    stl_path = find_input_stl(args.stl)
    robot = re.sub(r"[^A-Za-z0-9_]+", "_", stl_path.stem).strip("_").lower() or "robot_dog"
    print(f"[1/6] Input: {stl_path}")

    mesh = trimesh.load(stl_path, force="mesh")
    to_m = {"mm": 0.001, "cm": 0.01, "m": 1.0}[args.input_unit]
    if to_m != 1.0:
        mesh.apply_scale(to_m)
    lo = mesh.vertices.min(0)
    hi = mesh.vertices.max(0)
    size = hi - lo

    # Cut planes from geometry. Symmetry plane Y≈0; front/back split at the
    # X midpoint of the leg span; hip/knee as height fractions.
    x_split = (lo[0] + hi[0]) / 2.0
    y_split = 0.0
    hip_z = lo[2] + args.hip_frac * size[2]
    knee_z = lo[2] + args.knee_frac * size[2]
    print(
        f"[2/6] Cuts: X={x_split:.3g} Y={y_split:.3g} "
        f"hipZ={hip_z:.3g} kneeZ={knee_z:.3g} (unit=m)"
    )

    fc = mesh.triangles.mean(axis=1)
    labels = [segment_label(c, x_split, y_split, hip_z, knee_z) for c in fc]
    raw_segments = split_by_label(mesh, labels)
    segments = {k: cap_holes(v) for k, v in raw_segments.items()}

    expected = ["torso"] + [f"{fb}_{lr}_{seg}" for fb, lr in LEG_KEYS for seg in ("thigh", "shank")]
    missing = [e for e in expected if e not in segments]
    if missing:
        print(f"      [WARN] missing segments: {missing} — try adjusting --hip-frac/--knee-frac")
    print(f"      Segmented into {len(segments)} parts: {sorted(segments)}")

    if args.no_llm:
        print("[3/6] --no-llm; fallback materials.")
        parsed = {}
    else:
        print(f"[3/6] Classifying materials with {MODEL_ID} ...")
        try:
            parsed = classify_materials(args.gateway, segments)
            for k in sorted(segments):
                m = material_for(k, parsed)
                print(f"        {k}: {m['material']}")
        except (urllib.error.URLError, urllib.error.HTTPError, KeyError) as e:
            print(f"      Claude failed ({e}); fallback.")
            parsed = {}

    # ── Build URDF ──────────────────────────────────────────────────────────
    out_dir = Path.home() / "Downloads" / f"{robot}_urdf"
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    limit = np.radians(args.joint_limit_deg)
    lines = ['<?xml version="1.0"?>', f'<robot name="{robot}">', ""]

    def export_and_link(label, joint_origin_world):
        """Translate the segment so joint_origin_world -> child-frame origin,
        export it, and return the link block lines (mesh at child origin)."""
        seg = segments[label].copy()
        seg.apply_translation(-np.asarray(joint_origin_world))
        seg.export(mesh_dir / f"{label}.stl")
        mat = material_for(label, parsed)
        com = seg.center_mass if seg.is_volume else seg.centroid
        vol = abs(seg.volume) if seg.is_volume else float(np.prod(seg.extents))
        mass = max(vol * mat["density"], 1e-4)
        return link_block(label, f"meshes/{label}.stl", com, mass, seg.extents, mat["color"])

    # Torso = base_link, its own frame is the world frame (no parent joint),
    # so its mesh is NOT translated.
    torso = segments["torso"]
    torso.export(mesh_dir / "torso.stl")
    tmat = material_for("torso", parsed)
    tcom = torso.center_mass if torso.is_volume else torso.centroid
    tvol = abs(torso.volume) if torso.is_volume else float(np.prod(torso.extents))
    tmass = max(tvol * tmat["density"], 1e-4)
    lines += [
        '  <link name="base_link">',
        "    <visual>",
        '      <origin xyz="0 0 0" rpy="0 0 0"/>',
        '      <geometry><mesh filename="meshes/torso.stl"/></geometry>',
        '      <material name="base_link_mat">',
        f'        <color rgba="{f(tmat["color"][0])} {f(tmat["color"][1])} {f(tmat["color"][2])} {f(tmat["color"][3])}"/>',
        "      </material>",
        "    </visual>",
        '    <collision><origin xyz="0 0 0"/><geometry><mesh filename="meshes/torso.stl"/></geometry></collision>',
        "    <inertial>",
        f'      <mass value="{f(tmass)}"/>',
        f'      <origin xyz="{f(tcom[0])} {f(tcom[1])} {f(tcom[2])}"/>',
        f'      <inertia ixx="{f(box_inertia(tmass,torso.extents)[0])}" ixy="0" ixz="0" '
        f'iyy="{f(box_inertia(tmass,torso.extents)[1])}" iyz="0" izz="{f(box_inertia(tmass,torso.extents)[2])}"/>',
        "    </inertial>",
        "  </link>",
        "",
    ]

    for fb, lr in LEG_KEYS:
        thigh_label = f"{fb}_{lr}_thigh"
        shank_label = f"{fb}_{lr}_shank"
        if thigh_label not in segments or shank_label not in segments:
            continue
        thigh = segments[thigh_label]
        shank = segments[shank_label]

        # Hip = top of the thigh (where it meets the torso): highest Z of thigh.
        tb = thigh.bounds
        hip_origin = np.array(
            [thigh.centroid[0], thigh.centroid[1], tb[1, 2]]
        )
        # Knee = thigh/shank seam ≈ knee_z plane at the thigh's X,Y.
        knee_origin = np.array([thigh.centroid[0], thigh.centroid[1], knee_z])

        # Thigh link, child of base_link. Joint at hip, in world coords because
        # base_link frame == world.
        lines += export_and_link(thigh_label, hip_origin)
        lines += [
            f'  <joint name="{fb}_{lr}_hip" type="revolute">',
            '    <parent link="base_link"/>',
            f'    <child link="{thigh_label}"/>',
            f'    <origin xyz="{f(hip_origin[0])} {f(hip_origin[1])} {f(hip_origin[2])}" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="{f(-limit)}" upper="{f(limit)}" effort="50" velocity="10"/>',
            "  </joint>",
            "",
        ]

        # Shank link, child of thigh. Its joint origin is the knee, expressed in
        # the THIGH's frame (thigh was shifted by -hip_origin), so subtract.
        knee_in_thigh = knee_origin - hip_origin
        lines += export_and_link(shank_label, knee_origin)
        lines += [
            f'  <joint name="{fb}_{lr}_knee" type="revolute">',
            f'    <parent link="{thigh_label}"/>',
            f'    <child link="{shank_label}"/>',
            f'    <origin xyz="{f(knee_in_thigh[0])} {f(knee_in_thigh[1])} {f(knee_in_thigh[2])}" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="{f(-limit)}" upper="{f(limit)}" effort="50" velocity="10"/>',
            "  </joint>",
            "",
        ]

    lines.append("</robot>")
    print(f"[4/6] Exported {len(segments)} segment meshes to {mesh_dir}")

    urdf_path = out_dir / f"{robot}.urdf"
    urdf_path.write_text("\n".join(lines), encoding="utf-8")
    n_joints = sum(1 for l in lines if "<joint " in l)
    print(f"[5/6] URDF written: {urdf_path} ({n_joints} revolute joints)")
    print(f"[6/6] Done. Bundle: {out_dir}")


if __name__ == "__main__":
    main()
