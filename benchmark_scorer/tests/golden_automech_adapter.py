"""Executable golden for deterministic AutoMech legacy artifact discovery."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile

from benchmark_scorer.adapters.automech import discover_automech


def _write(path: Path, data: bytes = b"fixture") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="scorer_automech_") as temp:
        root = Path(temp)
        _write(root / "kinematic_model.json", b"{}")
        _write(root / "machine.py", b"raise RuntimeError('must never execute')\n")
        _write(root / "meshes" / "part.stl")
        _write(root / "model.mjcf", b"<mujoco/>")
        _write(root / "model.urdf", b"<robot name='x'/>")
        _write(root / "builder_manifest.json", b"{}")
        for iteration in (0, 2):
            base = root / "physics" / f"mujoco_{iteration}"
            _write(base / "trajectory.json", b'{"t":[0,1]}')
            _write(base / "contacts.json", b'{"contacts":[]}')
            _write(base / "model.mp4")
            _write(base / "sim_result.json", b'{"passed":true}')
        (root / "result.json").write_text(json.dumps({
            "ok": False,
            "physics": {"passed": False, "video": "physics/mujoco_0/model.mp4"},
        }), encoding="utf-8")
        _write(root / "benchmark_metrics.json", b'{"outcome":{"ok":true}}')

        found = discover_automech(root)
        assert found.kinematic_model == "kinematic_model.json"
        assert found.machine_py == "machine.py"
        assert found.meshes == ("meshes/part.stl",)
        assert found.model_mjcf == "model.mjcf"
        assert found.model_urdf == "model.urdf"
        assert found.builder_manifest == "builder_manifest.json"
        # Raw result metadata is audit-only; highest completed standard attempt wins.
        assert found.physics_dir == "physics/mujoco_2"
        assert found.trajectory == "physics/mujoco_2/trajectory.json"
        assert found.contacts == "physics/mujoco_2/contacts.json"
        assert found.video == "physics/mujoco_2/model.mp4"
        assert found.audit["result.json"] == "result.json"
        assert found.audit["benchmark_metrics.json"] == "benchmark_metrics.json"
        audit_evidence = [e for e in found.evidence if e.kind == "audit_only"]
        assert audit_evidence and all(e.observation["score_bearing"] is False
                                      for e in audit_evidence)

    with tempfile.TemporaryDirectory(prefix="scorer_automech_fallback_") as temp:
        root = Path(temp)
        for iteration in (1, 5, 3):
            _write(root / "physics" / f"mujoco_{iteration}" / "trajectory.json", b"{}")
        found = discover_automech(root)
        assert found.physics_dir == "physics/mujoco_5"

    print("golden AutoMech adapter: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
