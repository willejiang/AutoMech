"""Golden slide-DOF smoke: the single-agent + MuJoCo path must emit and drive a real
translational joint.

Why this exists: before P1, single-agent only knew `fixed | spin | free`. A carriage marked
`dof=slide` would either be rejected, silently degrade to `fixed`, or be misreported as a
rotary `_spin` joint in the runtime. This test locks down the whole narrow path:

- authoring label `dof=slide|slide_axis=x|driver=True`
- `LinkSpec` / `KinematicModel.joints` synthesis (`prismatic`)
- MJCF emission (`<joint type="slide" name="carriage_slide" axis="1 0 0">`)
- runtime recognition (`input_joint == carriage_slide`, `input_unit == m`)
- trajectory recording (`joint_meta[carriage_slide] == {kind: slide, unit: m}`)
- actual motion (`qpos` changes by about the commanded 0.02 m)

It intentionally does NOT check mechanism semantics yet (moved_count stays 0 because there
is only the driver slider and a fixed payload). P1's first target is the primitive itself;
crank-slider comes later.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _ROOT)

from maker2.manager import save_model
from maker2.model import RunContext
from maker2.mjcf_builder import build_mjcf
from maker2.single_agent import evaluate_machine_python
from maker2.urdf_builder import build_urdf


def main() -> int:
    code = '''
def build_machine():
    from build123d import Box, Align, Location
    from cadpy.assembly import AssemblyHelper
    a = AssemblyHelper("slider_demo")
    base = Box(120, 40, 8, align=(Align.CENTER, Align.CENTER, Align.MIN))
    a.add(base, "base|dof=fixed")
    rail_left = Box(80, 4, 12, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, -12, 8)))
    rail_right = Box(80, 4, 12, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 12, 8)))
    a.add(rail_left, "rail_left|dof=fixed|mount=base")
    a.add(rail_right, "rail_right|dof=fixed|mount=base")
    carriage = Box(20, 20, 10, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((-20, 0, 10)))
    a.add(carriage, "carriage|dof=slide|slide_axis=x|driver=True|mount=rail_left,rail_right")
    payload = Box(8, 8, 20, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((-20, 0, 20)))
    a.add(payload, "payload|dof=fixed|mount=carriage")
    return a.build()
'''
    run = tempfile.mkdtemp(prefix="golden_slide_")
    model = evaluate_machine_python(code, run, "slider_demo")
    save_model(model, os.path.join(run, "kinematic_model.json"))
    joints = model.joints
    # rail_left, rail_right, carriage, payload -> 4 non-root placement joints
    assert len(joints) == 4, f"expected 4 placement joints, got {len(joints)}"
    slider = next((j for j in joints if j.child == "carriage"), None)
    assert slider is not None, "missing carriage placement joint"
    assert slider.type == "prismatic", f"expected prismatic, got {slider.type}"
    assert tuple(round(x, 6) for x in slider.axis) == (1.0, 0.0, 0.0), slider.axis

    ctx = RunContext(project_slug="slider_demo", run_dir=run,
                     urdf_path=os.path.join(run, "model.urdf"),
                     meshes_dir=os.path.join(run, "meshes"),
                     logs_dir=run,
                     model_json_path=os.path.join(run, "kinematic_model.json"))
    build_urdf(model, ctx)
    mjcf = build_mjcf(model, ctx, settings=None, metrics={}, log_fn=lambda *_: None)
    text = open(mjcf, encoding="utf-8").read()
    assert 'name="carriage_slide"' in text and 'type="slide"' in text and 'axis="1 0 0"' in text
    # A fixed payload mounted on a slider must not be a top-level jointless body:
    # that would fix it to the world, and its weld would anchor the carriage at qpos=0.
    assert 'name="payload_carried_free"' in text, text
    assert '<weld body1="payload" body2="carriage"' in text, text

    spec = {"run_dir": run, "duration_s": 1.0,
            "drive": {"mode": "velocity", "target_velocity": 0.02}}
    spec_p = os.path.join(run, "spec.json")
    json.dump(spec, open(spec_p, "w", encoding="utf-8"), indent=2)
    out = os.path.join(run, "physics")
    subprocess.run([sys.executable, os.path.join(_ROOT, "evaluator", "run_scenario_mujoco.py"),
                    "--mjcf", mjcf, "--spec", spec_p, "--out", out, "--task", "move the carriage"],
                   check=True, capture_output=True, text=True)

    tr = json.load(open(os.path.join(out, "trajectory.json"), encoding="utf-8"))
    sim = json.load(open(os.path.join(out, "sim_result.json"), encoding="utf-8"))
    assert tr["joint_meta"]["carriage_slide"]["kind"] == "slide"
    assert tr["joint_meta"]["carriage_slide"]["unit"] == "m"
    m = sim["metrics"]
    assert m["input_joint"] == "carriage_slide", m["input_joint"]
    assert m["input_unit"] == "m", m.get("input_unit")
    travel = float(tr["joints"]["carriage_slide"][-1]) - float(tr["joints"]["carriage_slide"][0])
    assert 0.015 <= travel <= 0.0215, travel

    shutil.rmtree(run, ignore_errors=True)
    print("golden slide runtime: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
