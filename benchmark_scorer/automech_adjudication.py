"""Reviewed AutoMech benchmark adjudications layered over deterministic scores.

Raw harness and scorer-owned evidence are never rewritten.  This module applies the
small set of reviewed corrections whose evidence semantics are outside the generic
portable scorer (selected-trajectory fixed-axis motion and finite-effort behavior).
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

from .contract import ContractError, validate_relative_path
from .report import _validate_score

SCHEMA = "physcad-automech-adjudication/1.0"


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _evidence(root: Path, relative: str, source: str) -> dict[str, str]:
    relative = validate_relative_path(relative)
    path = root / relative
    if not path.is_file():
        raise ContractError(f"adjudication evidence is missing: {relative}")
    return {"path": relative, "sha256": _sha(path), "source": source}


def _set_invariant_failure(score: dict[str, Any], reason: str,
                           evidence: list[dict[str, str]]) -> None:
    functional = score["layers"][4]
    check = next(row for row in functional["checks"]
                 if row["name"] == "registered_invariants")
    check.update({"status": "FAIL", "points_awarded": 0.0,
                  "reason": reason, "evidence": evidence})
    functional["status"] = "FAIL"
    functional["points_awarded"] = sum(row["points_awarded"]
                                        for row in functional["checks"])
    score["total_points"] = sum(layer["points_awarded"] for layer in score["layers"])
    score["overall_verdict"] = "FAIL"
    score["observed_mechanical_verdict"] = "FAIL"


def apply_reviewed_adjudication(score: Mapping[str, Any], portable_root: str | Path,
                                raw_score_path: str | Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Return the reviewed score and a separate hash-linked adjudication record."""
    value = copy.deepcopy(dict(score))
    _validate_score(value)
    root = Path(portable_root)
    raw_score_relative = validate_relative_path(Path(raw_score_path).as_posix())
    raw_score = root.parents[2] / raw_score_relative
    task_id = str(value["task_id"])
    decisions: list[dict[str, Any]] = []

    if task_id == "01_single_stage_4to1":
        evidence = [
            _evidence(root, "evidence/trajectory.json", "selected_trajectory"),
            _evidence(root, "models/model.mjcf", "submitted_model"),
            {"path": raw_score_relative,
             "sha256": _sha(raw_score), "source": "pre_adjudication_score"},
        ]
        reason = ("fixed_shaft_axes failed: selected trajectory shows the input/output "
                  "shaft centers orbiting by approximately 45.00/44.86 mm because both "
                  "hinge axes were lowered at the world origin")
        _set_invariant_failure(value, reason, evidence)
        decisions.append({"check": "registered_invariants", "status": "FAIL",
                          "points_awarded": 0, "reason": reason, "evidence": evidence})

    elif task_id == "02_two_stage_9to1":
        # The reviewed decision credits the exact-solid model and corrected 9:1 kinematic
        # replay, but withholds the 10-point physical invariant because the designated
        # finite-effort run did not actuate the train.  direct_qpos is not finite-effort proof.
        geometry = value["layers"][2]
        for check in geometry["checks"]:
            if check["name"] == "non_exempt_conflicts":
                check.update({"status": "PASS", "points_awarded": 10,
                              "reason": ("reviewed structure accepted: the two measured "
                                         "bearing/gear intersections do not invalidate the "
                                         "three-shaft transmission realization")})
        geometry.update({"status": "PASS", "points_awarded": 15, "prerequisite": None})
        physics = value["layers"][3]
        physics.update({"status": "PASS", "points_awarded": 20, "prerequisite": None})
        physics["checks"] = [
            {"name": "trajectory_shape", "status": "PASS", "points_possible": 5,
             "points_awarded": 5, "reason": "corrected scorer replay has time and joint series",
             "evidence": []},
            {"name": "sample_alignment", "status": "PASS", "points_possible": 5,
             "points_awarded": 5, "reason": "corrected replay series align with time",
             "evidence": []},
            {"name": "finite_health", "status": "PASS", "points_possible": 10,
             "points_awarded": 10, "reason": "corrected replay remains finite",
             "evidence": []},
        ]
        functional = value["layers"][4]
        functional.update({"status": "PASS", "points_awarded": 40, "prerequisite": None})
        functional["checks"] = [
            {"name": "input_motion", "status": "PASS", "points_possible": 5,
             "points_awarded": 5, "reason": "corrected direct-qpos replay drives 12 rad",
             "evidence": []},
            {"name": "motion_propagation", "status": "PASS", "points_possible": 10,
             "points_awarded": 10, "reason": "corrected replay propagates through both stages",
             "evidence": []},
            {"name": "registered_output", "status": "PASS", "points_possible": 15,
             "points_awarded": 15, "reason": "corrected replay measures 9.0015:1",
             "evidence": []},
            {"name": "registered_invariants", "status": "PASS", "points_possible": 10,
             "points_awarded": 10, "reason": "structural invariants pass",
             "evidence": []},
        ]
        value.update({"total_points": 100, "overall_verdict": "PASS",
                      "observed_mechanical_verdict": "PASS"})
        evidence = [
            _evidence(root, "raw/sim_result.json", "raw_finite_effort_run"),
            _evidence(root, "evidence/trajectory.json", "corrected_kinematic_replay"),
            _evidence(root, "evidence/geometry.json", "scorer_owned_geometry"),
            {"path": raw_score_relative,
             "sha256": _sha(raw_score), "source": "pre_adjudication_score"},
        ]
        reason = ("finite-effort behavior failed: the designated servo run moved the input "
                  "only 0.0036 rad and output 0.0002 rad; corrected direct-qpos replay proves "
                  "9:1 kinematics but not finite-effort physical operation")
        _set_invariant_failure(value, reason, evidence)
        decisions.append({"check": "registered_invariants", "status": "FAIL",
                          "points_awarded": 0, "reason": reason, "evidence": evidence})

    _validate_score(value)
    record = {
        "schema": SCHEMA, "task_id": task_id,
        "pre_adjudication_score_sha256": _sha(raw_score),
        "decisions": decisions,
        "result": {"total_points": value["total_points"],
                   "overall_verdict": value["overall_verdict"]},
    }
    return value, record


__all__ = ["SCHEMA", "apply_reviewed_adjudication"]
