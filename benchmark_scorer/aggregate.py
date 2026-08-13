"""Fixed-denominator aggregation for the ten-task Comfort v1 suite."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Mapping

from .contract import ContractError
from .report import load_score_json
from .tasks.comfort_v1 import TASKS

SUITE_TASK_IDS = tuple(task.task_id for task in TASKS)


def aggregate_scores(paths: Iterable[str | Path]) -> dict[str, Any]:
    """Aggregate score files with a fixed 10-task denominator.

    Duplicates are rejected, missing tasks contribute zero, and unknown checks are
    reported explicitly while their already-zero points remain in each task score.
    """
    by_task: dict[str, tuple[str, Mapping[str, Any]]] = {}
    duplicates: dict[str, list[str]] = {}
    unexpected: list[str] = []
    for raw_path in paths:
        path = str(Path(raw_path))
        score = load_score_json(raw_path)
        if score.get("suite_id") != "physcad-comfort-v1":
            raise ContractError(f"score is not from physcad-comfort-v1: {path}")
        task_id = score.get("task_id")
        if task_id not in SUITE_TASK_IDS:
            unexpected.append(path)
            continue
        if task_id in by_task:
            duplicates.setdefault(str(task_id), [by_task[str(task_id)][0]]).append(path)
        else:
            by_task[str(task_id)] = (path, score)
    if duplicates:
        detail = "; ".join(f"{task}: {', '.join(items)}"
                           for task, items in sorted(duplicates.items()))
        raise ContractError(f"duplicate suite task scores: {detail}")
    missing = [task_id for task_id in SUITE_TASK_IDS if task_id not in by_task]
    entries = []
    total = 0.0
    unknown_tasks = []
    for task_id in SUITE_TASK_IDS:
        if task_id not in by_task:
            entries.append({"task_id": task_id, "status": "MISSING", "score": 0.0,
                            "unknown_checks": None, "path": None})
            continue
        path, score = by_task[task_id]
        points = float(score["total_points"])
        unknown = int(score.get("unknown_checks", 0))
        total += points
        if unknown:
            unknown_tasks.append(task_id)
        entries.append({"task_id": task_id,
                        "status": "UNKNOWN_EVIDENCE" if unknown else "SCORED",
                        "score": points, "unknown_checks": unknown, "path": path})
    return {
        "schema": "physcad-benchmark-suite-score/1.0",
        "suite_id": "physcad-comfort-v1",
        "denominator_tasks": 10,
        "points_possible": 1000,
        "total_points": round(total, 6),
        "mean_score": round(total / 10.0, 6),
        "submitted_tasks": len(by_task),
        "missing_tasks": missing,
        "unknown_evidence_tasks": unknown_tasks,
        "unexpected_score_files": sorted(unexpected),
        "duplicates": {},
        "tasks": entries,
    }


def discover_score_files(path: str | Path) -> tuple[Path, ...]:
    root = Path(path)
    if root.is_file():
        return (root,)
    if not root.is_dir():
        raise ContractError(f"aggregate input does not exist: {path}")
    return tuple(sorted(root.rglob("score.json"), key=lambda item: item.as_posix().casefold()))


def write_aggregate_json(value: Mapping[str, Any], path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(dict(value), indent=2, sort_keys=True,
                                 allow_nan=False) + "\n", encoding="utf-8")
    return target


__all__ = ["SUITE_TASK_IDS", "aggregate_scores", "discover_score_files",
           "write_aggregate_json"]
