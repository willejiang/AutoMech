#!/usr/bin/env python3
"""Split a downloaded STL into its connected parts and emit a URDF.

Pipeline:
  1. Pick the newest *.stl in ~/Downloads (or a path given on the CLI).
  2. Split it into geometrically connected components via trimesh — each
     physically separate mesh becomes one URDF link.
  3. Render each part to a PNG and ask Claude (claude-opus-4.8, via the local
     LLM gateway at localhost:8313) to name it and pick a material (color +
     physical properties). One batched call for all parts.
  4. Export each part as its own mesh and write a URDF where every link is
     attached to a single base_link with a fixed joint (rigid assembly).
  5. Drop the whole bundle (URDF + meshes) into ~/Downloads/<name>_urdf/.

The gateway injects provider credentials itself, so no real ANTHROPIC_API_KEY
is needed — same path the web app's evaluateModel.ts uses.

Usage:
  python scripts/stl_to_urdf.py [path/to/model.stl] [--gateway URL] [--no-llm]
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

# matplotlib's Agg backend renders off-screen with no display server — the
# robust choice on a headless / Windows-bash box where trimesh's OpenGL
# viewer can't grab a context.
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

GATEWAY_DEFAULT = os.environ.get("LLM_GATEWAY_URL", "http://localhost:8313").rstrip("/")
MODEL_ID = "claude-opus-4.8"

# Parts smaller than this fraction of the largest component's volume are
# treated as noise/shards and merged away rather than becoming their own link.
MIN_VOLUME_FRACTION = 0.001

# A named fallback palette so a --no-llm run (or a Claude failure) still yields
# a valid, visually distinct URDF.
FALLBACK_MATERIALS = [
    ("aluminum", (0.8, 0.8, 0.82, 1.0), 2700.0, 0.4),
    ("steel", (0.6, 0.6, 0.62, 1.0), 7850.0, 0.6),
    ("rubber", (0.1, 0.1, 0.1, 1.0), 1100.0, 1.0),
    ("plastic", (0.2, 0.5, 0.9, 1.0), 1100.0, 0.5),
    ("glass", (0.6, 0.8, 0.9, 0.5), 2500.0, 0.2),
    ("copper", (0.72, 0.45, 0.2, 1.0), 8960.0, 0.5),
]


def find_input_stl(cli_path: str | None) -> Path:
    if cli_path:
        p = Path(cli_path).expanduser()
        if not p.is_file():
            sys.exit(f"STL not found: {p}")
        return p
    downloads = Path.home() / "Downloads"
    stls = sorted(
        downloads.glob("*.stl"), key=lambda f: f.stat().st_mtime, reverse=True
    )
    if not stls:
        sys.exit(f"No .stl files found in {downloads}")
    return stls[0]


def safe_slug(name: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_").lower()
    return slug or "part"


def load_and_split(stl_path: Path) -> list[trimesh.Trimesh]:
    """Return connected components, largest first, noise shards dropped."""
    loaded = trimesh.load(stl_path, force="mesh")
    if not isinstance(loaded, trimesh.Trimesh):
        loaded = loaded.dump(concatenate=True)

    # split() walks face adjacency; only_watertight=False keeps open shells
    # (OpenSCAD exports are usually watertight, but parts joined only by a
    # shared edge shouldn't be required to be).
    parts = loaded.split(only_watertight=False)
    if len(parts) <= 1:
        return [loaded]

    parts = sorted(parts, key=lambda m: m.volume, reverse=True)
    largest = max(p.volume for p in parts) or 1.0
    kept = [p for p in parts if (p.volume / largest) >= MIN_VOLUME_FRACTION]
    return kept or [loaded]


def render_part_png(mesh: trimesh.Trimesh, size: int = 320) -> bytes:
    """Render a single part to an isometric PNG (raw color, no lighting)."""
    fig = plt.figure(figsize=(size / 100, size / 100), dpi=100)
    ax = fig.add_subplot(111, projection="3d")

    faces = mesh.vertices[mesh.faces]
    coll = Poly3DCollection(
        faces, alpha=1.0, facecolor="#3a86ff", edgecolor="none"
    )
    # Cheap shading: tint each face by its normal's Z so geometry reads in 3D.
    normals = mesh.face_normals
    shade = 0.45 + 0.55 * ((normals[:, 2] + 1.0) / 2.0)
    colors = np.stack(
        [0.227 * shade, 0.525 * shade, 1.0 * shade, np.ones_like(shade)], axis=1
    )
    coll.set_facecolor(colors)
    ax.add_collection3d(coll)

    b = mesh.bounds
    ax.set_xlim(b[0, 0], b[1, 0])
    ax.set_ylim(b[0, 1], b[1, 1])
    ax.set_zlim(b[0, 2], b[1, 2])
    try:
        ax.set_box_aspect((b[1] - b[0]))
    except Exception:
        pass
    ax.view_init(elev=25, azim=-60)
    ax.set_axis_off()

    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight", pad_inches=0)
    plt.close(fig)
    return buf.getvalue()


def gateway_chat(gateway: str, messages: list, max_tokens: int = 2000) -> str:
    url = f"{gateway}/v1/chat/completions"
    body = json.dumps(
        {"model": MODEL_ID, "max_tokens": max_tokens, "messages": messages}
    ).encode()
    req = urllib.request.Request(
        url,
        data=body,
        headers={
            "Content-Type": "application/json",
            "Authorization": "Bearer local",
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=120) as resp:
        data = json.loads(resp.read())
    return data["choices"][0]["message"]["content"]


def classify_parts(
    gateway: str, parts: list[trimesh.Trimesh], model_hint: str
) -> list[dict]:
    """Ask Claude to name + assign a material to each rendered part."""
    content: list = [
        {
            "type": "text",
            "text": (
                f"This is a 3D model exported as STL (filename hint: "
                f'"{model_hint}"). It was split into {len(parts)} connected '
                "parts, each shown below as an isometric render in order "
                "(part 0, part 1, ...). For EACH part, infer a short "
                "snake_case link name and a realistic material.\n\n"
                "Respond with ONLY a JSON array, one object per part in order:\n"
                '[{"name": "...", "material": "...", '
                '"color": [r,g,b,a], "density_kg_m3": <number>, '
                '"friction": <0..1>}]\n'
                "color components are 0..1 floats. No prose, no markdown."
            ),
        }
    ]
    for i, mesh in enumerate(parts):
        png = render_part_png(mesh)
        b64 = base64.b64encode(png).decode()
        content.append({"type": "text", "text": f"part {i}:"})
        content.append(
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/png;base64,{b64}"},
            }
        )

    # Budget scales with part count — each part's JSON object is ~90 tokens, and
    # a too-small cap truncates the array mid-object so json.loads fails and we
    # silently drop to the fallback palette. Floor of 4000, headroom on top.
    max_tokens = max(4000, 200 * len(parts))
    raw = gateway_chat(
        gateway, [{"role": "user", "content": content}], max_tokens=max_tokens
    )
    return parse_classification(raw, len(parts))


def _load_json_array(text: str):
    """Parse a JSON array, salvaging a response truncated mid-array.

    A token-capped reply can stop partway through, leaving an unterminated
    array (`[ {..}, {..}, {"name": "fo`). Rather than discard every object,
    trim back to the last complete `}` and close the bracket so the parsed
    prefix still drives the URDF.
    """
    start = text.find("[")
    if start == -1:
        return None
    body = text[start:]
    try:
        return json.loads(body)
    except json.JSONDecodeError:
        pass
    last_obj = body.rfind("}")
    if last_obj == -1:
        return None
    try:
        return json.loads(body[: last_obj + 1] + "]")
    except json.JSONDecodeError:
        return None


def parse_classification(raw: str, n: int) -> list[dict]:
    """Extract the JSON array; fall back to the palette on any mismatch."""
    text = raw.strip()
    # Strip a leading ```json / ``` fence if present (the model sometimes adds
    # one despite being told not to).
    text = re.sub(r"^```(?:json)?\s*", "", text)
    text = re.sub(r"\s*```$", "", text).strip()

    parsed = _load_json_array(text)

    result = []
    for i in range(n):
        fb = FALLBACK_MATERIALS[i % len(FALLBACK_MATERIALS)]
        item = parsed[i] if isinstance(parsed, list) and i < len(parsed) else {}
        if not isinstance(item, dict):
            item = {}
        name = safe_slug(str(item.get("name", f"part_{i}")))
        color = item.get("color", fb[1])
        try:
            color = [float(c) for c in color][:4]
            if len(color) == 3:
                color.append(1.0)
            assert len(color) == 4
        except Exception:
            color = list(fb[1])
        density = item.get("density_kg_m3", fb[2])
        try:
            density = float(density)
            assert density > 0
        except Exception:
            density = fb[2]
        friction = item.get("friction", fb[3])
        try:
            friction = float(friction)
        except Exception:
            friction = fb[3]
        result.append(
            {
                "name": name,
                "material": str(item.get("material", fb[0])),
                "color": color,
                "density": density,
                "friction": friction,
            }
        )
    # De-duplicate link names (URDF requires unique names).
    seen: dict[str, int] = {}
    for item in result:
        base = item["name"]
        if base in seen:
            seen[base] += 1
            item["name"] = f"{base}_{seen[base]}"
        else:
            seen[base] = 0
    return result


def box_inertia(mass: float, extents: np.ndarray) -> tuple[float, ...]:
    """Solid-box inertia about the COM — a sane URDF default per link."""
    x, y, z = (max(float(e), 1e-4) for e in extents)
    ixx = mass * (y * y + z * z) / 12.0
    iyy = mass * (x * x + z * z) / 12.0
    izz = mass * (x * x + y * y) / 12.0
    return ixx, iyy, izz


def f(x: float) -> str:
    return f"{x:.6g}"


def build_urdf(
    robot_name: str,
    parts: list[trimesh.Trimesh],
    meta: list[dict],
    mesh_dir_rel: str,
) -> str:
    lines = ['<?xml version="1.0"?>', f'<robot name="{robot_name}">', ""]

    # Materials (one <material> per link, referenced by name).
    for m in meta:
        r, g, b, a = m["color"]
        lines.append(f'  <material name="{m["name"]}_mat">')
        lines.append(f'    <color rgba="{f(r)} {f(g)} {f(b)} {f(a)}"/>')
        lines.append("  </material>")
    lines.append("")

    # base_link: a massless anchor everything attaches to.
    lines += [
        '  <link name="base_link">',
        "    <inertial>",
        '      <mass value="0.0001"/>',
        '      <origin xyz="0 0 0"/>',
        '      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>',
        "    </inertial>",
        "  </link>",
        "",
    ]

    for mesh, m in zip(parts, meta):
        name = m["name"]
        com = mesh.center_mass if mesh.is_volume else mesh.centroid
        extents = mesh.extents
        volume = abs(mesh.volume) if mesh.is_volume else float(np.prod(extents))
        mass = max(volume * m["density"], 1e-4)
        ixx, iyy, izz = box_inertia(mass, extents)
        mesh_file = f"{mesh_dir_rel}/{name}.stl"

        lines.append(f'  <link name="{name}">')
        lines.append("    <visual>")
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append("      <geometry>")
        lines.append(f'        <mesh filename="{mesh_file}"/>')
        lines.append("      </geometry>")
        lines.append(f'      <material name="{name}_mat"/>')
        lines.append("    </visual>")
        lines.append("    <collision>")
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append("      <geometry>")
        lines.append(f'        <mesh filename="{mesh_file}"/>')
        lines.append("      </geometry>")
        lines.append("    </collision>")
        lines.append("    <inertial>")
        lines.append(f'      <mass value="{f(mass)}"/>')
        lines.append(
            f'      <origin xyz="{f(com[0])} {f(com[1])} {f(com[2])}" rpy="0 0 0"/>'
        )
        lines.append(
            f'      <inertia ixx="{f(ixx)}" ixy="0" ixz="0" '
            f'iyy="{f(iyy)}" iyz="0" izz="{f(izz)}"/>'
        )
        lines.append("    </inertial>")
        lines.append("  </link>")

        lines.append(f'  <joint name="base_to_{name}" type="fixed">')
        lines.append('    <parent link="base_link"/>')
        lines.append(f'    <child link="{name}"/>')
        lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append("  </joint>")
        lines.append("")

    lines.append("</robot>")
    return "\n".join(lines)


def main() -> None:
    # Windows consoles default to cp1252 and crash on non-Latin-1 output
    # (Claude part names, status glyphs). Force UTF-8 where supported.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="STL → URDF splitter")
    ap.add_argument("stl", nargs="?", help="Path to STL (default: newest in ~/Downloads)")
    ap.add_argument("--gateway", default=GATEWAY_DEFAULT, help="LLM gateway base URL")
    ap.add_argument(
        "--no-llm",
        action="store_true",
        help="Skip Claude; use the fallback material palette",
    )
    ap.add_argument(
        "--input-unit",
        choices=["mm", "cm", "m"],
        default="mm",
        help="Unit of the STL coordinates (default: mm). URDF is always written "
        "in meters, so meshes/inertia are scaled to SI accordingly.",
    )
    args = ap.parse_args()

    stl_path = find_input_stl(args.stl)
    robot_name = safe_slug(stl_path.stem)
    print(f"[1/5] Input: {stl_path}")

    parts = load_and_split(stl_path)
    print(f"[2/5] Split into {len(parts)} part(s) by connected components.")

    # URDF / robotics simulators expect meters. STL carries no unit, so convert
    # from the declared input unit. Without this, a mm-authored model's
    # volume×density yields masses ~1e9× too large (mm³ vs m³) and the URDF,
    # while valid XML, is physically unusable.
    to_m = {"mm": 0.001, "cm": 0.01, "m": 1.0}[args.input_unit]
    if to_m != 1.0:
        for mesh in parts:
            mesh.apply_scale(to_m)
        print(f"      Scaled {args.input_unit} -> m (x{to_m:g}) for SI URDF.")

    if args.no_llm:
        print("[3/5] --no-llm set; using fallback material palette.")
        meta = parse_classification("[]", len(parts))
    else:
        print(f"[3/5] Classifying parts with {MODEL_ID} via {args.gateway} ...")
        try:
            meta = classify_parts(args.gateway, parts, stl_path.name)
            print("      Claude classification:")
            for i, m in enumerate(meta):
                print(f"        part {i}: {m['name']} ({m['material']})")
        except (urllib.error.URLError, urllib.error.HTTPError, KeyError) as e:
            print(f"      Claude call failed ({e}); falling back to palette.")
            meta = parse_classification("[]", len(parts))

    out_dir = Path.home() / "Downloads" / f"{robot_name}_urdf"
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    for mesh, m in zip(parts, meta):
        mesh.export(mesh_dir / f"{m['name']}.stl")
    print(f"[4/5] Exported {len(parts)} part mesh(es) to {mesh_dir}")

    urdf = build_urdf(robot_name, parts, meta, "meshes")
    urdf_path = out_dir / f"{robot_name}.urdf"
    urdf_path.write_text(urdf, encoding="utf-8")
    print(f"[5/5] URDF written: {urdf_path}")
    print(f"\nDone. Bundle in: {out_dir}")


if __name__ == "__main__":
    main()
