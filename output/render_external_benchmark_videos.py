from __future__ import annotations

import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from evaluator.diagnose import encode_mp4

STRICT = REPO / "output" / "external_benchmark_strict"
WORK = REPO / "output" / "external_benchmark_work"
OUT = Path("C:/Users/t-zhijjiang/Downloads/external_benchmark_videos")
FPS = 20
WIDTH = 640
HEIGHT = 480

CASES = [
    ("codex", "01_single_stage_4to1", "replay", "Codex 01 — scorer replay: valid collision proxies"),
    ("claude-code", "02_two_stage_9to1", "replay", "Claude 02 — ideal 9:1, moving collisions disabled"),
    ("claude-code", "04_openwork_clock_12to1", "replay", "Claude 04 — ideal 12:1, moving collisions disabled"),
    ("claude-code", "10_wind_rotor_pump", "submitted", "Claude 10 — submitted trajectory writes passive coordinates"),
    ("claude-code", "10_wind_rotor_pump", "replay", "Claude 10 — scorer replay: output remains stationary"),
]


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def series(raw):
    if isinstance(raw, dict):
        raw = raw.get("qpos", raw.get("position"))
    if not isinstance(raw, list) or not raw:
        return None
    values = []
    for value in raw:
        if isinstance(value, list):
            if not value:
                return None
            value = value[0]
        values.append(float(value))
    return values


def trajectory_for(method: str, task: str, lane: str):
    if lane == "submitted":
        return read_json(STRICT / method / task / "evidence" / "trajectory.json")
    return read_json(WORK / method / task / "replay" / "trajectory.json")


def configure_collision_debug(model):
    for geom_id in range(model.ngeom):
        name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "").casefold()
        if "ground" in name or int(model.geom_bodyid[geom_id]) == 0:
            model.geom_rgba[geom_id] = (0.10, 0.11, 0.13, 1.0)
            continue
        active = int(model.geom_contype[geom_id]) != 0 and int(model.geom_conaffinity[geom_id]) != 0
        if active:
            model.geom_rgba[geom_id] = (0.96, 0.08, 0.04, 0.82)
        else:
            model.geom_rgba[geom_id] = (0.55, 0.60, 0.68, 0.28)


def camera_for(model, data):
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    points = np.asarray(data.xpos[1:], dtype=float)
    if len(points):
        lower = points.min(axis=0)
        upper = points.max(axis=0)
        camera.lookat[:] = (lower + upper) * 0.5
        extent = max(float(np.linalg.norm(upper - lower)), 0.10)
    else:
        camera.lookat[:] = model.stat.center
        extent = max(float(model.stat.extent), 0.10)
    camera.distance = max(extent * 2.2, 0.24)
    camera.azimuth = 135.0
    camera.elevation = -22.0
    return camera


def add_caption(image: np.ndarray, title: str, subtitle: str):
    frame = Image.fromarray(image)
    draw = ImageDraw.Draw(frame, "RGBA")
    draw.rectangle((0, 0, WIDTH, 55), fill=(0, 0, 0, 175))
    draw.text((12, 7), title, fill=(255, 255, 255, 255))
    draw.text((12, 30), subtitle, fill=(255, 225, 120, 255))
    return frame


def render_case(method: str, task: str, lane: str, title: str, debug: bool):
    task_root = STRICT / method / task
    model = mujoco.MjModel.from_xml_path(str(task_root / "models" / "model.mjcf"))
    if debug:
        configure_collision_debug(model)
    data = mujoco.MjData(model)
    trajectory = trajectory_for(method, task, lane)
    times = trajectory.get("t") or []
    joints = trajectory.get("joints") or {}
    if not times:
        raise RuntimeError(f"{method}/{task}/{lane}: missing time samples")
    mapped = []
    for name, raw in joints.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
        values = series(raw)
        if joint_id >= 0 and values and len(values) == len(times):
            mapped.append((str(name), int(model.jnt_qposadr[joint_id]), values))
    if not mapped:
        raise RuntimeError(f"{method}/{task}/{lane}: no trajectory joints map to MJCF")

    frame_count = min(140, len(times))
    indices = np.linspace(0, len(times) - 1, frame_count).round().astype(int)
    slug = f"{method}_{task}_{lane}_{'collision_debug' if debug else 'visual'}"
    frames = OUT / slug
    frames.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    mujoco.mj_forward(model, data)
    camera = camera_for(model, data)
    for frame_index, sample_index in enumerate(indices):
        mujoco.mj_resetData(model, data)
        for _, qpos_address, values in mapped:
            data.qpos[qpos_address] = values[sample_index]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        image = renderer.render()
        if debug:
            subtitle = "RED = collision enabled; GRAY = visual only / collision disabled"
        elif lane == "submitted" and task == "10_wind_rotor_pump":
            subtitle = "Submitted telemetry: passive output coordinates are externally written"
        elif lane == "replay" and task == "10_wind_rotor_pump":
            subtitle = "Independent MJCF replay: rotor turns, piston output remains still"
        else:
            subtitle = "Ideal equality motion; this alone does not prove contact physics"
        captioned = add_caption(image, title, subtitle)
        captioned.save(frames / f"rgb_{frame_index:04d}.png")
    renderer.close()
    output = OUT / f"{slug}.mp4"
    result = encode_mp4(str(frames), str(output), fps=FPS)
    if not result:
        raise RuntimeError(f"failed to encode {output}")
    return output


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    outputs = []
    for method, task, lane, title in CASES:
        outputs.append(render_case(method, task, lane, title, debug=False))
        if lane == "replay" and task != "10_wind_rotor_pump":
            outputs.append(render_case(method, task, lane, title, debug=True))
    (OUT / "README.md").write_text(
        "# External benchmark videos\n\n"
        "- `visual`: normal visible geometry.\n"
        "- `collision_debug`: red geometry participates in collision; gray geometry is visual-only.\n"
        "- Claude task 10 `submitted` shows the harness-provided passive-coordinate motion.\n"
        "- Claude task 10 `replay` shows the submitted MJCF under independent replay; its output is stationary.\n\n"
        "A visually correct moving model can still fail strict physics when important moving solids are gray in the collision-debug video.\n",
        encoding="utf-8")
    for output in outputs:
        print(output, output.stat().st_size, flush=True)


if __name__ == "__main__":
    main()
