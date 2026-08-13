"""Executable golden for scorer-owned deterministic MuJoCo replay."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile

from benchmark_scorer import ContractError
from benchmark_scorer.replay import replay_model, task_profile


EXACT_MJCF = """<mujoco model="exact_gear">
  <option timestep="0.002" gravity="0 0 0" integrator="RK4"/>
  <worldbody>
    <body name="driver_body"><joint name="input_shaft" type="hinge" axis="0 0 1"/>
      <geom type="cylinder" size="0.02 0.005" mass="1"/>
    </body>
    <body name="output_body"><joint name="output_shaft" type="hinge" axis="0 0 1"/>
      <geom type="cylinder" size="0.04 0.005" mass="1"/>
    </body>
  </worldbody>
  <equality><joint joint1="output_shaft" joint2="input_shaft"
                   polycoef="0 -0.25 0 0 0"/></equality>
</mujoco>
"""

CONFLICT_MJCF = """<mujoco model="conflicting_gear">
  <worldbody>
    <body><joint name="input_shaft" type="hinge"/><geom type="sphere" size="0.01"/></body>
    <body><joint name="output_shaft" type="hinge"/><geom type="sphere" size="0.01"/></body>
  </worldbody>
  <equality>
    <joint joint1="output_shaft" joint2="input_shaft" polycoef="0 -0.25 0 0 0"/>
    <joint joint1="output_shaft" joint2="input_shaft" polycoef="0 -0.5 0 0 0"/>
  </equality>
</mujoco>
"""

FINITE_MJCF = """<mujoco model="finite_hinge">
  <option timestep="0.002" gravity="0 0 0" integrator="RK4"/>
  <worldbody>
    <body name="flywheel"><joint name="crankshaft_input" type="hinge" axis="0 0 1"
      damping="0.2"/>
      <geom type="cylinder" size="0.03 0.01" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""


def _write(root: Path, text: str) -> Path:
    path = root / "model.mjcf"
    path.write_text(text, encoding="utf-8")
    return path


def _expect_reject(xml: str, needle: str) -> None:
    with tempfile.TemporaryDirectory(prefix="scorer_replay_reject_") as temp, \
            tempfile.TemporaryDirectory(prefix="scorer_replay_output_") as output:
        root = Path(temp)
        path = _write(root, xml)
        try:
            replay_model(path, Path(output) / "out", "01_single_stage_4to1", model_root=root)
        except ContractError as exc:
            assert needle in str(exc), str(exc)
        else:
            raise AssertionError(f"unsafe MJCF accepted: {needle}")


def main() -> int:
    try:
        import mujoco  # noqa: F401
    except ImportError:
        print("golden benchmark scorer replay: SKIP (mujoco unavailable)")
        return 0

    assert task_profile("01_single_stage_4to1").mode == "exact_kinematic_projection"
    assert task_profile("06_four_planet_4to1").mode == "exact_kinematic_projection"
    assert task_profile("07_horizontal_slider_crank").mode == "finite_effort_pd"
    assert task_profile("10_wind_rotor_pump").mode == "finite_effort_pd"

    with tempfile.TemporaryDirectory(prefix="scorer_replay_exact_") as temp, \
            tempfile.TemporaryDirectory(prefix="scorer_replay_exact_out_") as output:
        root = Path(temp)
        path = _write(root, EXACT_MJCF)
        first = replay_model(path, Path(output) / "out1", "01_single_stage_4to1", model_root=root)
        second = replay_model(path, Path(output) / "out2", "01_single_stage_4to1", model_root=root)
        a = json.loads(Path(first.trajectory_path).read_text(encoding="utf-8"))
        b = json.loads(Path(second.trajectory_path).read_text(encoding="utf-8"))
        assert a == b
        input_q = a["joints"]["input_shaft"]["qpos"]
        output_q = a["joints"]["output_shaft"]["qpos"]
        assert len(input_q) == len(a["t"]) == len(output_q)
        assert max(abs(output + 0.25 * input_value)
                   for input_value, output in zip(input_q, output_q)) < 1e-10
        assert a["finite_health"]["all_finite"]
        assert a["profile"]["mode"] == "exact_kinematic_projection"
        assert first.metadata["hashes"]["source_model_sha256"]
        assert first.metadata["hashes"]["trajectory_sha256"]
        assert Path(first.metadata_path).is_file()
        assert Path(first.output_dir, "imported", "model.mjcf").is_file()

    with tempfile.TemporaryDirectory(prefix="scorer_replay_finite_") as temp, \
            tempfile.TemporaryDirectory(prefix="scorer_replay_finite_out_") as output:
        root = Path(temp)
        path = _write(root, FINITE_MJCF)
        result = replay_model(path, Path(output) / "out", "07_horizontal_slider_crank",
                              model_root=root)
        trajectory = json.loads(Path(result.trajectory_path).read_text(encoding="utf-8"))
        qpos = trajectory["joints"]["crankshaft_input"]["qpos"]
        assert trajectory["profile"]["mode"] == "finite_effort_pd"
        assert trajectory["finite_health"]["all_finite"]
        assert max(qpos) - min(qpos) > 0.1
        assert trajectory["profile"]["max_effort"] == 20.0
        assert abs(trajectory["t"][-1] - trajectory["profile"]["duration_s"]) <= 0.0021
        assert any(abs((later - earlier) - 1.0 / 60.0) > 1e-6
                   for earlier, later in zip(trajectory["t"], trajectory["t"][1:]))

    _expect_reject(CONFLICT_MJCF, "conflicting equality projection")
    _expect_reject(
        FINITE_MJCF.replace('timestep="0.002"', 'timestep="0.000001"'),
        "timestep")
    _expect_reject("<mujoco><include file=\"other.xml\"/></mujoco>", "forbidden")
    _expect_reject("<mujoco><extension><plugin plugin=\"x\"/></extension></mujoco>",
                   "forbidden")
    _expect_reject(
        "<mujoco><asset><mesh name=\"m\" file=\"C:/escape.stl\"/></asset></mujoco>",
        "absolute")
    _expect_reject(
        "<mujoco><asset><mesh name=\"m\" file=\"../escape.stl\"/></asset></mujoco>",
        "traversing")

    print("golden benchmark scorer replay: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
