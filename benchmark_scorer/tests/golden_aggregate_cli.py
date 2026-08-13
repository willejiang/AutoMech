"""Executable golden for fixed-denominator aggregation and CLI surfaces."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile

from benchmark_scorer.aggregate import aggregate_scores
from benchmark_scorer.cli import main
from benchmark_scorer.contract import ContractError
from benchmark_scorer.tasks.comfort_v1 import TASKS


def _score(path: Path, task_id: str, points: float, unknown: int = 0) -> None:
    layers = []
    for index, weight in enumerate((10, 15, 15, 20, 40), start=1):
        award = min(float(weight), max(0.0, points - sum(x["points_awarded"] for x in layers)))
        if unknown and index == 5:
            checks = [
                {"name": "known", "status": "PASS", "points_possible": award,
                 "points_awarded": award, "reason": "fixture", "evidence": []},
                {"name": "unknown", "status": "UNKNOWN", "points_possible": weight - award,
                 "points_awarded": 0, "reason": "fixture", "evidence": []},
            ]
            status = "UNKNOWN"
        elif award == weight:
            checks = [{"name": "fixture", "status": "PASS", "points_possible": weight,
                       "points_awarded": award, "reason": "fixture", "evidence": []}]
            status = "PASS"
        else:
            checks = [
                {"name": "earned", "status": "PASS", "points_possible": award,
                 "points_awarded": award, "reason": "fixture", "evidence": []},
                {"name": "lost", "status": "FAIL", "points_possible": weight - award,
                 "points_awarded": 0, "reason": "fixture", "evidence": []},
            ]
            status = "FAIL"
        layers.append({"index": index, "name": str(index), "weight": weight,
                       "status": status, "points_awarded": award,
                       "prerequisite": None, "checks": checks})
    value = {"schema": "physcad-benchmark-score/1.0", "scorer_version": "fixture",
             "suite_id": "physcad-comfort-v1", "task_id": task_id, "input_kind": "portable",
             "layer_weights": [10, 15, 15, 20, 40], "layers": layers,
             "total_points": points, "points_possible": 100, "unknown_checks": unknown}
    path.write_text(json.dumps(value), encoding="utf-8")


def main_test() -> int:
    with tempfile.TemporaryDirectory(prefix="scorer_aggregate_") as temp:
        root = Path(temp)
        first, second = root / "first.json", root / "second.json"
        _score(first, TASKS[0].task_id, 80, 1)
        _score(second, TASKS[1].task_id, 60)
        value = aggregate_scores((first, second))
        assert value["denominator_tasks"] == 10
        assert value["total_points"] == 140
        assert value["mean_score"] == 14
        assert len(value["missing_tasks"]) == 8
        assert value["unknown_evidence_tasks"] == [TASKS[0].task_id]
        # The unknown scalar is ignored; checks are authoritative.
        first_doc = json.loads(first.read_text())
        first_doc["unknown_checks"] = 999
        first.write_text(json.dumps(first_doc))
        assert aggregate_scores((first, second))["tasks"][0]["unknown_checks"] == 1
        wrong_suite = root / "wrong-suite.json"
        _score(wrong_suite, TASKS[2].task_id, 100)
        wrong_doc = json.loads(wrong_suite.read_text())
        wrong_doc["suite_id"] = "other-suite"
        wrong_suite.write_text(json.dumps(wrong_doc))
        try:
            aggregate_scores((wrong_suite,))
        except ContractError as exc:
            assert "physcad-comfort-v1" in str(exc)
        else:
            raise AssertionError("wrong suite score accepted")
        duplicate = root / "duplicate.json"
        _score(duplicate, TASKS[0].task_id, 1)
        try:
            aggregate_scores((first, duplicate))
        except ContractError as exc:
            assert "duplicate" in str(exc)
        else:
            raise AssertionError("duplicate task score accepted")
        output = root / "suite.json"
        assert main(["aggregate", str(first), str(second), "--output", str(output)]) == 0
        assert json.loads(output.read_text())["mean_score"] == 14
    print("golden aggregate CLI: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main_test())
