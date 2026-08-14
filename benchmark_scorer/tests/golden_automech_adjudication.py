"""Golden checks for reviewed AutoMech task-1/task-2 adjudications."""
from __future__ import annotations

import json
from pathlib import Path

from benchmark_scorer.automech_adjudication import apply_reviewed_adjudication
from benchmark_scorer.report import _validate_score

REPO = Path(__file__).resolve().parents[2]
PORTABLE = REPO / "output" / "comfort_benchmark_v1_rescored_portable"
SCORES = REPO / "benchmark_results" / "comfort_v1_20260811" / "automech_scores"


def adjudicate(task_id: str):
    score_path = SCORES / task_id / "score.json"
    score = json.loads(score_path.read_text(encoding="utf-8"))
    return apply_reviewed_adjudication(
        score, PORTABLE / task_id, score_path.relative_to(REPO))


def main() -> None:
    task1, record1 = adjudicate("01_single_stage_4to1")
    assert task1["total_points"] == 90
    assert task1["overall_verdict"] == "FAIL"
    invariant1 = task1["layers"][4]["checks"][3]
    assert invariant1["status"] == "FAIL"
    assert "45.00/44.86 mm" in invariant1["reason"]
    assert record1["decisions"][0]["evidence"]

    task2, record2 = adjudicate("02_two_stage_9to1")
    assert task2["total_points"] == 90
    assert task2["overall_verdict"] == "FAIL"
    assert task2["layers"][2]["points_awarded"] == 15
    assert task2["layers"][3]["points_awarded"] == 20
    invariant2 = task2["layers"][4]["checks"][3]
    assert invariant2["status"] == "FAIL"
    assert "0.0036 rad" in invariant2["reason"]
    assert "direct-qpos" in invariant2["reason"]
    assert record2["result"] == {"total_points": 90, "overall_verdict": "FAIL"}

    _validate_score(task1)
    _validate_score(task2)
    print("golden AutoMech adjudication: PASS")


if __name__ == "__main__":
    main()
