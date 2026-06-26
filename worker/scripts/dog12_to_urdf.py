#!/usr/bin/env python3
"""Turn a 12-DOF quadruped STL into a walkable URDF (12 revolute joints).

Like dog_to_urdf.py but for a true 12-DOF dog: each leg gets THREE joints —
hip-abduction (roll, about X), hip-pitch (about Y), knee (about Y) — so each
leg is cut into hip_module / thigh / shank (4 legs x 3 = 12 joints).

Kinematic chain per leg:
  base_link (torso)
    └─ <leg>_hip   — hip-abduction joint, revolute about X, at the leg root
         └─ <leg>_thigh — hip-pitch joint, revolute about Y, at the hip module
              └─ <leg>_shank — knee joint, revolute about Y, at the knee bend

Segmentation is geometric. Front and back legs fold in OPPOSITE directions
(front knee points forward, rear knee points backward — measured from the
mesh), so the knee cut is taken at each leg's own knee height. Each child
mesh is translated so its joint origin sits at the child-frame origin, which
the reassembly check in validate confirms is correct.

Materials are inferred by Claude (claude-opus-4.8 via the local gateway).

Usage:
  python scripts/dog12_to_urdf.py [path/to/dog.stl] [--no-llm]
      [--hip-frac 0.84] [--knee-frac 0.55] [--input-unit mm]
Default input: newest *12-DOF*/*Quadruped*/*dog* STL in ~/Downloads.
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

LEG_KEYS = [("front", "left"), ("front", "right"), ("back", "left"), ("back", "right")]
SEGMENTS = ("hip", "thigh", "shank")

FALLBACK = {
    "torso": ((0.25, 0.35, 0.55, 1.0), 1200.0, 0.5),
    "hip": ((0.45, 0.45, 0.48, 1.0), 2700.0, 0.5),
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
    for pat in (r"12.?dof", r"quadruped", r"dog"):
        hits = sorted(
            [f for f in downloads.glob("*.stl") if re.search(pat, f.name, re.I)],
            key=lambda f: f.stat().st_mtime,
            reverse=True,
        )
        if hits:
            return hits[0]
    stls = sorted(downloads.glob("*.stl"), key=lambda f: f.stat().st_mtime, reverse=True)
    if not stls:
        sys.exit(f"No .stl in {downloads}")
    return stls[0]


def leg_quadrant(centroid, x_split):
    """Return (front/back, left/right) for a face centroid, or None for torso."""
    x, y, _ = centroid
    fb = "front" if x > x_split else "back"
    lr = "left" if y > 0 else "right"
    return fb, lr


def cap_holes(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    m = mesh.copy()
    try:
        m.fill_holes()
    except Exception:
        pass
    m.remove_unreferenced_vertices()
    return m


def leg_faces(fc, x_split, fb, lr, hip_z):
    """Boolean face mask for one leg quadrant below the torso line."""
    xc = 1 if fb == "front" else -1
    yc = 1 if lr == "left" else -1
    return (
        (np.sign(fc[:, 0] - x_split) == xc)
        & (np.sign(fc[:, 1]) == yc)
        & (fc[:, 2] < hip_z)
    )


def render_leg_side_png(mesh, fc, x_split, fb, lr, hip_z, size=380):
    """Side view (X-Z) of one leg with a normalized 0..100% vertical scale.
    0% = foot (bottom), 100% = leg top (near body). Used for Claude to mark
    the hip and knee joint heights."""
    from matplotlib.collections import PolyCollection

    sel = leg_faces(fc, x_split, fb, lr, hip_z)
    idx = np.where(sel)[0]
    tri = mesh.vertices[mesh.faces[idx]]
    v = mesh.vertices[np.unique(mesh.faces[idx])]
    zmin, zmax = v[:, 2].min(), v[:, 2].max()
    fig, ax = plt.subplots(figsize=(size / 100, size / 75))
    ax.add_collection(
        PolyCollection(
            [t[:, [0, 2]] for t in tri],
            facecolor="#3a86ff",
            edgecolor="#15408f",
            linewidths=0.1,
            alpha=0.75,
        )
    )
    ax.set_xlim(v[:, 0].min() - 8, v[:, 0].max() + 8)
    ax.set_ylim(zmin - 5, zmax + 5)
    for pct in range(0, 101, 10):
        z = zmin + pct / 100 * (zmax - zmin)
        ax.axhline(z, color="r", lw=0.5, alpha=0.4)
        ax.text(v[:, 0].max() + 2, z, f"{pct}%", color="r", fontsize=8, va="center")
    ax.set_title(f"{fb}_{lr} leg side, 0%=foot 100%=top")
    ax.set_aspect("equal")
    ax.set_yticks([])
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=88, bbox_inches="tight")
    plt.close(fig)
    return buf.getvalue(), zmin, zmax


def annotate_legs(gateway, mesh, fc, x_split, hip_z):
    """Ask Claude for the hip/knee joint heights (% of leg span) on ONE clear
    leg (front_left) and apply the same split to all four legs.

    A symmetric quadruped's legs share the same segment proportions, so a single
    careful annotation is far more stable than four independent ones (which drift
    and sometimes collapse a segment with hip==knee). Returns dict[(fb,lr)] ->
    (hip_pct, knee_pct). Raises on gateway failure.
    """
    png, _, _ = render_leg_side_png(mesh, fc, x_split, "front", "left", hip_z)
    b64 = base64.b64encode(png).decode()
    content = [
        {
            "type": "text",
            "text": (
                "Side view of ONE leg of a 12-DOF quadruped robot dog. Vertical "
                "scale 0%=foot (bottom) to 100%=top (near body). The leg has 3 "
                "segments and 2 joints: HIP (upper round pivot where the leg meets "
                "the body) and KNEE (round pivot mid-leg where the leg bends). The "
                "hip is clearly ABOVE the knee. Report each joint's vertical "
                'position in percent. Respond ONLY JSON: {"hip_pct": <n>, '
                '"knee_pct": <n>}'
            ),
        },
        {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{b64}"}},
    ]
    raw = gateway_chat(gateway, [{"role": "user", "content": content}], max_tokens=1500)
    text = re.sub(r"^```(?:json)?\s*", "", raw.strip())
    text = re.sub(r"\s*```$", "", text).strip()
    try:
        d = json.loads(text[text.find("{") :])
        hip_pct = float(d["hip_pct"])
        knee_pct = float(d["knee_pct"])
    except Exception:
        hip_pct, knee_pct = 75.0, 40.0
    # Guard against a collapsed thigh segment: hip must sit clearly above knee.
    if not (hip_pct - knee_pct >= 15.0 and 0 < knee_pct < hip_pct < 100):
        hip_pct, knee_pct = 75.0, 40.0
    return {key: (hip_pct, knee_pct) for key in LEG_KEYS}


def measure_leg_zs(mesh, fc, x_split, hip_z, annotations=None):
    """Per leg, compute hip-top, hip/thigh seam Z, knee Z, foot Z.

    When `annotations` (dict[(fb,lr)]->(hip_pct,knee_pct)) is given, the hip and
    knee boundaries come from Claude's percentages of the leg's Z span. Otherwise
    falls back to geometric knee detection (extreme-X bend point) plus a fixed
    hip band. Returns dict[(fb,lr)] with hip_top, hip_pitch_z, knee_z, foot,
    knee_pt.
    """
    out = {}
    for fb, lr in LEG_KEYS:
        xc = 1 if fb == "front" else -1
        sel = leg_faces(fc, x_split, fb, lr, hip_z)
        c = fc[sel]
        if len(c) == 0:
            continue
        hip_top = c[:, 2].max()
        foot = c[:, 2].min()
        span = hip_top - foot

        if annotations and (fb, lr) in annotations:
            hip_pct, knee_pct = annotations[(fb, lr)]
            # Hip joint % marks the hip-pitch line (hip module is above it).
            hip_pitch_z = foot + (hip_pct / 100.0) * span
            knee_z = foot + (knee_pct / 100.0) * span
        else:
            hip_pitch_z = hip_top - 0.20 * span
            mid = (c[:, 2] > foot + 0.25 * span) & (c[:, 2] < hip_top - 0.15 * span)
            band = c[mid] if mid.sum() > 0 else c
            kp = band[band[:, 0].argmin()] if xc > 0 else band[band[:, 0].argmax()]
            knee_z = kp[2]

        # knee point (X,Y) at the knee Z, for the knee joint origin.
        near = c[np.abs(c[:, 2] - knee_z) < max(span * 0.06, 1e-4)]
        if len(near) == 0:
            near = c
        knee_pt = np.array([near[:, 0].mean(), near[:, 1].mean(), knee_z])
        out[(fb, lr)] = {
            "hip_top": hip_top,
            "hip_pitch_z": hip_pitch_z,
            "knee_z": knee_z,
            "foot": foot,
            "knee_pt": knee_pt,
        }
    return out


def segment_mesh(mesh, x_split, hip_z, leg_zs):
    """Assign every face to torso or <leg>_<seg>. Returns dict[label]=Trimesh."""
    fc = mesh.triangles.mean(axis=1)
    labels = np.empty(len(fc), dtype=object)
    for i, c in enumerate(fc):
        if c[2] >= hip_z:
            labels[i] = "torso"
            continue
        fb, lr = leg_quadrant(c, x_split)
        z = c[2]
        zinfo = leg_zs.get((fb, lr))
        if zinfo is None:
            labels[i] = "torso"
            continue
        knee_z = zinfo["knee_z"]
        # hip module = top slice (above the hip-pitch line); thigh = down to the
        # knee; shank = below the knee. Boundaries are per-leg (Claude-annotated
        # when available, else geometric).
        hip_pitch_z = zinfo["hip_pitch_z"]
        if z >= hip_pitch_z:
            seg = "hip"
        elif z >= knee_z:
            seg = "thigh"
        else:
            seg = "shank"
        labels[i] = f"{fb}_{lr}_{seg}"

    out = {}
    for lab in sorted(set(labels)):
        idx = np.where(labels == lab)[0]
        if len(idx) == 0:
            continue
        out[lab] = cap_holes(mesh.submesh([idx], append=True))
    return out


def render_segment_png(mesh, size=300):
    fig = plt.figure(figsize=(size / 100, size / 100), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    n = mesh.face_normals
    shade = 0.45 + 0.55 * ((n[:, 2] + 1.0) / 2.0)
    cols = np.stack([0.25 * shade, 0.5 * shade, 0.85 * shade, np.ones_like(shade)], 1)
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


def gateway_chat(gateway, messages, max_tokens=3500, retries=2):
    body = json.dumps(
        {"model": MODEL_ID, "max_tokens": max_tokens, "messages": messages}
    ).encode()
    last = None
    for attempt in range(retries + 1):
        req = urllib.request.Request(
            f"{gateway}/v1/chat/completions",
            data=body,
            headers={"Content-Type": "application/json", "Authorization": "Bearer local"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=150) as resp:
            data = json.loads(resp.read())
        choices = data.get("choices") or []
        if choices and choices[0].get("message", {}).get("content"):
            return choices[0]["message"]["content"]
        # Empty/transient response — retry.
        last = data
    raise KeyError(f"gateway returned no choices after {retries + 1} tries: {last}")


def classify_materials(gateway, segments):
    names = list(segments.keys())
    content = [
        {
            "type": "text",
            "text": (
                "This is a 12-DOF walking quadruped robot dog, segmented for URDF "
                f"into {len(names)} parts shown below in order: {names}. For EACH "
                "part give a realistic material as JSON keyed by the exact part "
                'name:\n{"front_left_thigh": {"material": "...", "color": '
                '[r,g,b,a], "density_kg_m3": <n>, "friction": <0..1>}, ...}\n'
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
    seg = next((s for s in ("torso", "hip", "thigh", "shank") if s in label), "thigh")
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
    return {"material": str(item.get("material", seg)), "color": color,
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


def joint_lines(name, jtype, parent, child, origin, axis, limit):
    return [
        f'  <joint name="{name}" type="{jtype}">',
        f'    <parent link="{parent}"/>',
        f'    <child link="{child}"/>',
        f'    <origin xyz="{f(origin[0])} {f(origin[1])} {f(origin[2])}" rpy="0 0 0"/>',
        f'    <axis xyz="{axis}"/>',
        f'    <limit lower="{f(-limit)}" upper="{f(limit)}" effort="50" velocity="10"/>',
        "  </joint>",
    ]


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="12-DOF quadruped STL -> URDF")
    ap.add_argument("stl", nargs="?")
    ap.add_argument("--gateway", default=GATEWAY_DEFAULT)
    ap.add_argument("--no-llm", action="store_true")
    ap.add_argument("--input-unit", choices=["mm", "cm", "m"], default="mm")
    ap.add_argument("--hip-frac", type=float, default=0.84,
                    help="torso/leg split as frac of height (legs below)")
    ap.add_argument("--joint-limit-deg", type=float, default=75.0)
    args = ap.parse_args()

    stl_path = find_input_stl(args.stl)
    robot = re.sub(r"[^A-Za-z0-9_]+", "_", stl_path.stem).strip("_").lower() or "dog12"
    print(f"[1/6] Input: {stl_path}")

    mesh = trimesh.load(stl_path, force="mesh")
    to_m = {"mm": 0.001, "cm": 0.01, "m": 1.0}[args.input_unit]
    if to_m != 1.0:
        mesh.apply_scale(to_m)
    lo = mesh.vertices.min(0)
    hi = mesh.vertices.max(0)
    size = hi - lo
    x_split = (lo[0] + hi[0]) / 2.0
    hip_z = lo[2] + args.hip_frac * size[2]
    print(f"[2/6] Cuts: X={x_split:.3g} hipZ={hip_z:.3g} (unit=m)")

    fc = mesh.triangles.mean(axis=1)

    # Per-leg joint heights. With LLM enabled, Claude marks the hip/knee on each
    # leg's side view (robust to folded/vertical legs); --no-llm uses geometric
    # detection.
    annotations = None
    if not args.no_llm:
        print(f"[2b/6] Annotating leg joints with {MODEL_ID} ...")
        try:
            annotations = annotate_legs(args.gateway, mesh, fc, x_split, hip_z)
            for (fb, lr), (h, k) in annotations.items():
                print(f"        {fb}_{lr}: hip={h:.0f}% knee={k:.0f}%")
        except (urllib.error.URLError, urllib.error.HTTPError, KeyError) as e:
            print(f"      Claude annotation failed ({e}); geometric fallback.")
            annotations = None

    leg_zs = measure_leg_zs(mesh, fc, x_split, hip_z, annotations)
    segments = segment_mesh(mesh, x_split, hip_z, leg_zs)

    expected = ["torso"] + [f"{fb}_{lr}_{s}" for fb, lr in LEG_KEYS for s in SEGMENTS]
    missing = [e for e in expected if e not in segments]
    if missing:
        print(f"      [WARN] missing: {missing} (adjust --hip-frac)")
    print(f"      {len(segments)} parts: {sorted(segments)}")

    if args.no_llm:
        print("[3/6] --no-llm; fallback materials.")
        parsed = {}
    else:
        print(f"[3/6] Classifying materials with {MODEL_ID} ...")
        try:
            parsed = classify_materials(args.gateway, segments)
            for k in sorted(segments):
                print(f"        {k}: {material_for(k, parsed)['material']}")
        except (urllib.error.URLError, urllib.error.HTTPError, KeyError) as e:
            print(f"      Claude failed ({e}); fallback.")
            parsed = {}

    out_dir = Path.home() / "Downloads" / f"{robot}_urdf"
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    limit = np.radians(args.joint_limit_deg)

    lines = ['<?xml version="1.0"?>', f'<robot name="{robot}">', ""]

    def emit_link(label, joint_origin_world):
        seg = segments[label].copy()
        seg.apply_translation(-np.asarray(joint_origin_world))
        seg.export(mesh_dir / f"{label}.stl")
        mat = material_for(label, parsed)
        com = seg.center_mass if seg.is_volume else seg.centroid
        vol = abs(seg.volume) if seg.is_volume else float(np.prod(seg.extents))
        mass = max(vol * mat["density"], 1e-4)
        return link_lines(label, f"meshes/{label}.stl", com, mass, seg.extents, mat["color"])

    # base_link = torso, frame == world (mesh not translated).
    torso = segments["torso"]
    torso.export(mesh_dir / "torso.stl")
    tmat = material_for("torso", parsed)
    tcom = torso.center_mass if torso.is_volume else torso.centroid
    tvol = abs(torso.volume) if torso.is_volume else float(np.prod(torso.extents))
    tmass = max(tvol * tmat["density"], 1e-4)
    lines += link_lines("base_link", "meshes/torso.stl", tcom, tmass, torso.extents, tmat["color"])
    lines.append("")

    for fb, lr in LEG_KEYS:
        labels = {s: f"{fb}_{lr}_{s}" for s in SEGMENTS}
        if any(labels[s] not in segments for s in SEGMENTS):
            continue
        zinfo = leg_zs[(fb, lr)]
        hip_seg = segments[labels["hip"]]
        thigh_seg = segments[labels["thigh"]]

        # Joint origins (world coords).
        # hip-abduction at leg root (top of hip module, near torso).
        hb = hip_seg.bounds
        hip_root = np.array([hip_seg.centroid[0], hip_seg.centroid[1], hb[1, 2]])
        # hip-pitch at the hip/thigh seam (bottom of hip module).
        hip_pitch = np.array([hip_seg.centroid[0], hip_seg.centroid[1], hb[0, 2]])
        # knee at the measured bend point.
        knee = np.asarray(zinfo["knee_pt"], dtype=float)

        # hip module: child of base_link (frame == world).
        lines += emit_link(labels["hip"], hip_root)
        lines += joint_lines(f"{fb}_{lr}_hip_abduction", "revolute", "base_link",
                             labels["hip"], hip_root, "1 0 0", limit)
        lines.append("")

        # thigh: child of hip module. Origin (hip_pitch) expressed in hip frame.
        lines += emit_link(labels["thigh"], hip_pitch)
        lines += joint_lines(f"{fb}_{lr}_hip_pitch", "revolute", labels["hip"],
                             labels["thigh"], hip_pitch - hip_root, "0 1 0", limit)
        lines.append("")

        # shank: child of thigh. Origin (knee) expressed in thigh frame.
        lines += emit_link(labels["shank"], knee)
        lines += joint_lines(f"{fb}_{lr}_knee", "revolute", labels["thigh"],
                             labels["shank"], knee - hip_pitch, "0 1 0", limit)
        lines.append("")

    lines.append("</robot>")
    print(f"[4/6] Exported {len(segments)} meshes to {mesh_dir}")

    urdf_path = out_dir / f"{robot}.urdf"
    urdf_path.write_text("\n".join(lines), encoding="utf-8")
    n_joints = sum(1 for l in lines if "<joint " in l)
    print(f"[5/6] URDF written: {urdf_path} ({n_joints} revolute joints)")
    print(f"[6/6] Done. Bundle: {out_dir}")


if __name__ == "__main__":
    main()
