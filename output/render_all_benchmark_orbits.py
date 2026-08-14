from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import imageio_ffmpeg
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from evaluator.diagnose import encode_mp4

OUT = Path("C:/Users/t-zhijjiang/Downloads/all_benchmark_orbit_videos")
WIDTH = 640
HEIGHT = 480
FPS = 24
FRAMES = 96

PALETTE = np.asarray([
    (0.95, 0.28, 0.20, 1.0),
    (0.10, 0.62, 0.92, 1.0),
    (0.98, 0.69, 0.15, 1.0),
    (0.20, 0.78, 0.45, 1.0),
    (0.65, 0.35, 0.92, 1.0),
    (0.95, 0.35, 0.70, 1.0),
    (0.15, 0.78, 0.78, 1.0),
    (0.92, 0.52, 0.12, 1.0),
    (0.45, 0.68, 0.18, 1.0),
    (0.30, 0.42, 0.92, 1.0),
    (0.84, 0.24, 0.44, 1.0),
    (0.15, 0.66, 0.48, 1.0),
], dtype=float)

METHOD_LABELS = {"automech": "AutoMech", "claude-code": "Claude Code", "codex": "Codex"}


def model_rows():
    suite = REPO / "output" / "comfort_benchmark_v1_20260811"
    summary = json.loads((suite / "suite_summary.json").read_text(encoding="utf-8"))
    rows = [("automech", item["name"], Path(item["result"]["run_dir"]) / "model.mjcf")
            for item in summary]
    strict = REPO / "output" / "external_benchmark_strict"
    for method in ("claude-code", "codex"):
        for task in sorted(path for path in (strict / method).iterdir() if path.is_dir()):
            rows.append((method, task.name, task / "models" / "model.mjcf"))
    return rows


def color_model(model):
    for geom_id in range(model.ngeom):
        geom_type = int(model.geom_type[geom_id])
        name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "").casefold()
        if geom_type == int(mujoco.mjtGeom.mjGEOM_PLANE) or "ground" in name:
            model.geom_rgba[geom_id] = (0.10, 0.12, 0.15, 1.0)
            continue
        original_alpha = float(model.geom_rgba[geom_id, 3])
        # Preserve deliberately invisible collision proxies. Visible mesh/primitive geometry
        # receives a consistent high-saturation body color.
        if original_alpha <= 0.05:
            model.geom_rgba[geom_id, 3] = 0.0
            continue
        body_id = int(model.geom_bodyid[geom_id])
        model.geom_rgba[geom_id] = PALETTE[(body_id * 5 + geom_id) % len(PALETTE)]


def framing(model, data):
    centers = []
    radii = []
    for geom_id in range(model.ngeom):
        if int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        if float(model.geom_rgba[geom_id, 3]) <= 0.05:
            continue
        centers.append(np.asarray(data.geom_xpos[geom_id], dtype=float))
        radius = float(model.geom_rbound[geom_id])
        radii.append(radius if np.isfinite(radius) and radius > 0 else 0.005)
    if not centers:
        return np.asarray(model.stat.center, dtype=float), max(float(model.stat.extent), 0.1)
    centers = np.asarray(centers)
    radii = np.asarray(radii)[:, None]
    lower = np.min(centers - radii, axis=0)
    upper = np.max(centers + radii, axis=0)
    center = (lower + upper) * 0.5
    diagonal = float(np.linalg.norm(upper - lower))
    return center, max(diagonal, 0.1)


def caption(image, method, task, frame_index):
    frame = Image.fromarray(image)
    draw = ImageDraw.Draw(frame, "RGBA")
    draw.rectangle((0, 0, WIDTH, 58), fill=(5, 8, 14, 205))
    draw.text((14, 9), f"{METHOD_LABELS[method]} — {task}", fill=(255, 255, 255, 255))
    angle = frame_index * 360.0 / FRAMES
    draw.text((14, 33), f"360-degree final-model orbit | camera azimuth {angle:5.1f} deg",
              fill=(255, 214, 92, 255))
    return frame


def render_one(method, task, model_path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    color_model(model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    center, diagonal = framing(model, data)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = center
    camera.distance = max(diagonal * 1.55, 0.22)
    camera.elevation = -20.0
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    frame_dir = OUT / "frames" / method / task
    if frame_dir.exists():
        shutil.rmtree(frame_dir)
    frame_dir.mkdir(parents=True)
    for index in range(FRAMES):
        camera.azimuth = index * 360.0 / FRAMES
        renderer.update_scene(data, camera=camera)
        image = renderer.render()
        caption(image, method, task, index).save(frame_dir / f"rgb_{index:04d}.png")
    renderer.close()
    video_dir = OUT / method
    video_dir.mkdir(parents=True, exist_ok=True)
    output = video_dir / f"{task}_orbit.mp4"
    result = encode_mp4(str(frame_dir), str(output), fps=FPS)
    if not result:
        raise RuntimeError(f"failed to encode {output}")
    return output


def compile_method(method, videos):
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    concat = OUT / method / "_concat.txt"
    with concat.open("w", encoding="utf-8") as stream:
        for path in videos:
            stream.write(f"file '{path.as_posix()}'\n")
    target = OUT / f"{method}_all_10_orbits.mp4"
    command = [ffmpeg, "-y", "-f", "concat", "-safe", "0", "-i", str(concat),
               "-c", "copy", "-movflags", "+faststart", str(target)]
    subprocess.run(command, check=True, capture_output=True, timeout=300)
    return target


def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    by_method = {method: [] for method in METHOD_LABELS}
    index_rows = []
    for number, (method, task, model_path) in enumerate(model_rows(), start=1):
        output = render_one(method, task, model_path)
        by_method[method].append(output)
        index_rows.append((method, task, output))
        print(f"[{number:02d}/30] {method} {task}: {output.stat().st_size} bytes", flush=True)
    compilations = {method: compile_method(method, videos)
                    for method, videos in by_method.items()}
    lines = ["# All benchmark 360-degree orbit videos", "",
             "Each individual video shows the final submitted MJCF at its initial state with a colorful body palette, a roughly 20-degree elevated camera, and one complete 360-degree orbit.", ""]
    for method in ("automech", "claude-code", "codex"):
        lines.extend([f"## {METHOD_LABELS[method]}", "",
                      f"Compilation: `{compilations[method]}`", ""])
        for _, task, output in [row for row in index_rows if row[0] == method]:
            lines.append(f"- `{task}` — `{output}`")
        lines.append("")
    (OUT / "INDEX.md").write_text("\n".join(lines), encoding="utf-8")
    for method, path in compilations.items():
        print(f"COMPILATION {method}: {path} {path.stat().st_size} bytes", flush=True)
    print(f"INDEX: {OUT / 'INDEX.md'}", flush=True)


if __name__ == "__main__":
    main()
