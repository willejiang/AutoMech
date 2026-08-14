from __future__ import annotations

import hashlib
import json
from pathlib import Path

from benchmark_scorer.aggregate import aggregate_scores
from benchmark_scorer.automech_adjudication import apply_reviewed_adjudication
from benchmark_scorer.report import write_score_json, write_score_markdown

REPO = Path(__file__).resolve().parents[1]
PORTABLE = REPO / "output" / "comfort_benchmark_v1_rescored_portable"
SCORES = REPO / "benchmark_results" / "comfort_v1_20260811" / "automech_scores"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def strict_dump(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
                    encoding="utf-8")


def main() -> None:
    for task_id in ("01_single_stage_4to1", "02_two_stage_9to1"):
        task_root = SCORES / task_id
        score_path = task_root / "score.json"
        pre = task_root / "score.pre_adjudication.json"
        original_path = pre if pre.exists() else score_path
        original = json.loads(original_path.read_text(encoding="utf-8"))
        if not pre.exists():
            strict_dump(pre, original)
        reviewed, record = apply_reviewed_adjudication(
            original, PORTABLE / task_id, pre.relative_to(REPO))
        record["pre_adjudication_score_sha256"] = sha(pre)
        strict_dump(task_root / "adjudication.json", record)
        write_score_json(reviewed, score_path)
        write_score_markdown(score_path, task_root / "score.md")

    aggregate = aggregate_scores(sorted(SCORES.glob("*/score.json")))
    strict_dump(SCORES / "suite_score.json", aggregate)
    assert aggregate["total_points"] == 860.0, aggregate
    scores = [json.loads(path.read_text()) for path in SCORES.glob("*/score.json")]
    assert sum(score["overall_verdict"] == "PASS" for score in scores) == 5
    print("AutoMech reviewed aggregate: 860/1000, 5/10 strict PASS")


if __name__ == "__main__":
    main()
