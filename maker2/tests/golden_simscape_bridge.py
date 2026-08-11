#!/usr/bin/env python3
"""Golden Simscape bridge smoke.

Locks down the post-scaffold contract:

- `engine='simscape'` dispatches through maker2.physics.run_physics(...)
- the backend exports a rich `simscape_bundle.json`
- `build_simscape_model.m` contains the project-runner handoff hook
- an external runner can write `sim_result.json` and maker2 ingests it back into the
  normal result shape (`passed/verdict/metrics/tests/frames_dir/video`)

This test does NOT require MATLAB. It injects a tiny fake runner through
SIMSCAPE_RUNNER_JSON and verifies that the bridge behaves like a real backend seam.
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _ROOT)

from maker2.config import Settings
from maker2.manager import save_model
from maker2.physics import run_physics
from maker2.single_agent import evaluate_machine_python


def main() -> int:
    code = '''
shaft_z = 24
wheel_z = shaft_z

MECHANISM = {
    "output_link": "wheel",
    "watch_links": ["wheel"],
    "ports_by_link": {
        "shaft": [
            {"name": "journal", "type": "shaft", "xyz_mm": [0, 0, shaft_z / 2], "axis": [0, 0, 1], "diameter_mm": 6},
            {"name": "top_face", "type": "flat_face", "xyz_mm": [0, 0, shaft_z], "axis": [0, 0, 1], "normal_sign": 1},
        ],
        "wheel": [
            {"name": "bore", "type": "bore", "xyz_mm": [0, 0, wheel_z / 2], "axis": [0, 0, 1], "diameter_mm": 6.2, "depth_mm": wheel_z},
            {"name": "bottom_face", "type": "flat_face", "xyz_mm": [0, 0, 0], "axis": [0, 0, -1], "normal_sign": 1},
        ],
    },
    "relations": [
        {
            "name": "wheel_on_shaft",
            "mate_type": "coaxial_face",
            "base_part": "shaft",
            "base_port": "top_face",
            "incoming_part": "wheel",
            "incoming_port": "bottom_face",
            "offset_mm": 0,
            "flip": True,
        }
    ],
}

def build_machine():
    from build123d import Cylinder, Location, Align
    from cadpy.assembly import AssemblyHelper
    a = AssemblyHelper("simscape_demo")
    base = Cylinder(18, 6, align=(Align.CENTER, Align.CENTER, Align.MIN))
    a.add(base, "base|dof=fixed")
    shaft = Cylinder(3, shaft_z, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, 6)))
    a.add(shaft, "shaft|dof=spin|spin_axis=z|driver=True|mount=base|material=steel")
    wheel = Cylinder(11, wheel_z, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, 6)))
    a.add(wheel, "wheel|dof=fixed|mount=shaft|material=brass")
    return a.build()
'''
    run = tempfile.mkdtemp(prefix="golden_simscape_")
    old_runner = os.environ.get("SIMSCAPE_RUNNER_JSON")
    try:
        model = evaluate_machine_python(code, run, "simscape_demo")
        save_model(model, os.path.join(run, "kinematic_model.json"))

        fake_runner = r"""
import json, sys
from pathlib import Path
bundle = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
out = Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
driver_joint = next((j['name'] for j in bundle['mechanism']['joints'] if j.get('driver')), None)
driver_part = next((b['name'] for b in bundle['mechanism']['bodies'] if b.get('driver')), None)
watch_count = len(bundle['mechanism'].get('watch_links') or [])
summary = f"Simscape fake runner PASS for {bundle['name']}"
metrics = {
    'verdict': 'PASS',
    'test_kind': 'driven_mechanism',
    'input_joint': driver_joint,
    'input_part': driver_part,
    'input_travel': 6.2832,
    'input_unit': 'rad',
    'moved_count': watch_count,
    'watched_count': watch_count,
    'output_reached': True,
    'output_travel': 0.5236,
    'output_unit': 'rad',
    'ratio_in_out': 12.0,
    'exploded': False,
    'end_z': 0.0,
    'max_tilt_deg': 0.0,
    'max_drift': 0.0,
}
res = {
    'passed': True,
    'verdict': 'PASS',
    'summary': summary,
    'metrics': metrics,
    'frames_dir': None,
    'n_frames': 0,
    'tests': [{
        'name': 'watch_train',
        'verdict': 'PASS',
        'summary': summary,
        'metrics': metrics,
        'frames_dir': None,
        'video': None,
        'cause': 'none',
        'reason': 'fake runner',
    }],
    'cause': 'none',
    'reason': 'fake runner',
    'log': ['fake simscape runner'],
}
(out / 'sim_result.json').write_text(json.dumps(res, indent=2), encoding='utf-8')
"""
        os.environ["SIMSCAPE_RUNNER_JSON"] = json.dumps([
            sys.executable, "-c", fake_runner, "{bundle}", "{out_dir}"
        ])

        result = run_physics("ignored.urdf", "simscape bridge smoke", run,
                             settings=Settings(engine="simscape"))
        assert result["passed"] is True, result
        assert result["verdict"] == "PASS", result
        assert result["metrics"]["test_kind"] == "driven_mechanism", result["metrics"]
        assert result["metrics"]["ratio_in_out"] == 12.0, result["metrics"]
        assert result["tests"] and result["tests"][0]["design"]["bundle"].endswith("simscape_bundle.json")

        out_dir = Path(run) / "physics" / "simscape"
        bundle = json.loads((out_dir / "simscape_bundle.json").read_text(encoding="utf-8"))
        matlab = (out_dir / "build_simscape_model.m").read_text(encoding="utf-8")
        sim_result = json.loads((out_dir / "sim_result.json").read_text(encoding="utf-8"))

        assert bundle["version"] == 2, bundle
        assert bundle["mechanism"]["summary"]["body_count"] == 3, bundle["mechanism"]["summary"]
        assert bundle["mechanism"]["summary"]["relation_count"] == 1, bundle["mechanism"]["summary"]
        assert bundle["mechanism"]["output_link"] == "wheel", bundle["mechanism"]
        assert bundle["mechanism"]["watch_links"] == ["wheel"], bundle["mechanism"]
        shaft_body = next(b for b in bundle["mechanism"]["bodies"] if b["name"] == "shaft")
        assert shaft_body["world_pose"]["xyz_m"] == [0.0, 0.0, 0.006], shaft_body["world_pose"]
        mesh_path = Path(shaft_body["geometry"]["mesh_stl"])
        assert mesh_path.exists(), shaft_body["geometry"]["mesh_stl"]
        assert mesh_path.name == "shaft.stl", shaft_body["geometry"]["mesh_stl"]
        shaft_journal = next(p for p in bundle["mechanism"]["ports_by_link"]["shaft"] if p["name"] == "journal")
        assert shaft_journal["world_xyz_m"] == [0.0, 0.0, 0.018], shaft_journal
        rel = bundle["mechanism"]["relations"][0]
        assert rel["constraint_class"] == "fixed", rel
        assert rel["base_frame"]["name"] == "top_face", rel
        assert rel["incoming_frame"]["name"] == "bottom_face", rel
        assert "maker2_simscape_run(bundlePath, outDir)" in matlab, matlab
        assert "simscape_report.json" in matlab, matlab
        assert sim_result["passed"] is True and sim_result["verdict"] == "PASS", sim_result
        assert sim_result["tests"][0]["design"]["readme"].endswith("README.txt"), sim_result["tests"][0]

        print("golden simscape bridge: PASS")
        return 0
    finally:
        if old_runner is None:
            os.environ.pop("SIMSCAPE_RUNNER_JSON", None)
        else:
            os.environ["SIMSCAPE_RUNNER_JSON"] = old_runner
        shutil.rmtree(run, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
