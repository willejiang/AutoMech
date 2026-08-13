"""Executable golden for five-layer scoring and authoritative reporting."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import tempfile

from benchmark_scorer.contract import CONTRACT_ID
from benchmark_scorer.report import load_score_json, write_score_json, write_score_markdown
from benchmark_scorer.scoring import LAYER_WEIGHTS, score_path
from benchmark_scorer.tasks.comfort_v1 import get_task


def _hash(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _portable(root: Path) -> None:
    task = get_task("01_single_stage_4to1")
    assembly = {"links": [{"name": "input_shaft"}, {"name": "output_shaft"},
                            {"name": "gear_a"}, {"name": "gear_b"},
                            {"name": "hand_crank"}]}
    bindings = {"roles": {"input_shaft": "input_shaft", "output_shaft": "output_shaft",
                            "gear": ["gear_a", "gear_b"], "hand_crank": "hand_crank"}}
    samples = 81
    inputs = [8.0 * index / (samples - 1) for index in range(samples)]
    trajectory = {"t": [index * 0.05 for index in range(samples)],
                  "joints": {"input_shaft": inputs,
                             "output_shaft": [-value / 4 for value in inputs],
                             "gear_a": inputs, "gear_b": [-value / 4 for value in inputs]},
                  "finite_health": {"all_finite": True},
                  "invariants": {name: True for name in task.invariants}}
    mesh = b"solid fixture\nendsolid fixture\n"
    execution = {"model_compiled": True, "initialized": True, "all_finite": True,
                 "source_build_ok": True, "nonempty_part_set": True, "n_parts": 5}
    geometry = {"non_exempt_conflict_count": 0, "provenance_warnings": []}
    payloads = {"assembly.json": json.dumps(assembly).encode(),
                "task_bindings.json": json.dumps(bindings).encode(),
                "evidence/trajectory.json": json.dumps(trajectory).encode(),
                "evidence/execution.json": json.dumps(execution).encode(),
                "evidence/geometry.json": json.dumps(geometry).encode(),
                "meshes/fixture.stl": mesh}
    files = []
    for path, raw in payloads.items():
        target = root / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(raw)
        role = "mesh" if path.endswith(".stl") else Path(path).stem
        files.append({"path": path, "sha256": _hash(raw), "size": len(raw),
                      "media_type": "model/stl" if role == "mesh" else "application/json",
                      "role": role})
    manifest = {"contract": CONTRACT_ID, "suite_id": "physcad-comfort-v1",
                "task_id": task.task_id, "prompt_sha256": task.prompt_sha256,
                "producer": {"harness": "fixture", "harness_version": "1", "run_id": "x"},
                "evidence_lane": {"submitted": True, "physics_mode": "fixture",
                                  "engine": "fixture", "engine_version": "1"},
                "files": files, "units": {"length": "mm", "angle": "rad", "time": "s"},
                "telemetry_provenance": {"simulator": "fixture", "sampling_source": "state",
                                         "from": "fixture"}}
    (root / "benchmark_submission.json").write_text(json.dumps(manifest), encoding="utf-8")


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="scorer_pipeline_") as temp:
        root = Path(temp)
        _portable(root)
        result = score_path(root)
        assert tuple(layer.weight for layer in result.layers) == LAYER_WEIGHTS
        assert len(result.layers) == 5
        assert result.total_points == 100
        assert all(layer.status == "PASS" for layer in result.layers)
        score_json = write_score_json(result, root / "out" / "score.json")
        score_md = write_score_markdown(score_json)
        authoritative = load_score_json(score_json)
        assert authoritative["total_points"] == 100
        assert "**Total:** 100" in score_md.read_text(encoding="utf-8")

        corrupt = json.loads(score_json.read_text())
        corrupt["total_points"] = 99
        score_json.write_text(json.dumps(corrupt), encoding="utf-8")
        try:
            load_score_json(score_json)
        except Exception as exc:
            assert "disagrees" in str(exc)
        else:
            raise AssertionError("inconsistent score total accepted")

        # Insufficient input gates propagation/output/invariants even if other joints move.
        _portable(root)
        manifest = json.loads((root / "benchmark_submission.json").read_text())
        trajectory_doc = json.loads((root / "evidence" / "trajectory.json").read_text())
        trajectory_doc["joints"]["input_shaft"] = [value * 0.1 for value in trajectory_doc["joints"]["input_shaft"]]
        raw = json.dumps(trajectory_doc).encode()
        (root / "evidence" / "trajectory.json").write_bytes(raw)
        for item in manifest["files"]:
            if item["path"] == "evidence/trajectory.json":
                item["sha256"], item["size"] = _hash(raw), len(raw)
        (root / "benchmark_submission.json").write_text(json.dumps(manifest), encoding="utf-8")
        short = score_path(root)
        functional = short.layers[4]
        assert [check.status for check in functional.checks] == ["FAIL", "FAIL", "FAIL", "FAIL"]
        assert functional.points_awarded == 0

        # Missing trajectory is UNKNOWN and gates all later layers with zero points.
        trajectory = root / "evidence" / "trajectory.json"
        trajectory.unlink()
        manifest = json.loads((root / "benchmark_submission.json").read_text())
        manifest["files"] = [item for item in manifest["files"]
                             if item["path"] != "evidence/trajectory.json"]
        (root / "benchmark_submission.json").write_text(json.dumps(manifest), encoding="utf-8")
        missing = score_path(root)
        assert missing.layers[3].status == "UNKNOWN"
        assert missing.layers[4].status == "UNKNOWN"
        assert missing.layers[4].points_awarded == 0

    print("golden scoring pipeline: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
