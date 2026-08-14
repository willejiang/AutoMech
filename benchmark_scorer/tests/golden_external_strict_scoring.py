"""Golden checks for canonical external strict scoring and reporting."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile

from benchmark_scorer.contract import ContractError
from benchmark_scorer.external_scoring import (
    aggregate_external_scores, generate_external_suite, render_report_fragment,
    validate_external_score,
)

REPO = Path(__file__).resolve().parents[2]
STRICT = REPO / "output" / "external_benchmark_strict"
STAGED = REPO / "output" / "external_benchmark_staged"
WORK = REPO / "output" / "external_benchmark_work"
REALIZATION = REPO / "output" / "external_declared_realization_audit"
SUMMARY = REPO / "benchmark_results" / "comfort_v1_20260811" / "external_strict" / "summary.json"
EXPECTED = {"claude-code": (310, 0), "codex": (390, 1)}


def fail(action, needle: str) -> None:
    try:
        action()
    except ContractError as exc:
        assert needle in str(exc), exc
    else:
        raise AssertionError(f"expected ContractError containing {needle!r}")


def load_scores(root: Path, method: str):
    return [json.loads(path.read_text(encoding="utf-8"))
            for path in sorted((root / method).glob("*/score.json"))]


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="external_strict_golden_") as temp:
        first = Path(temp) / "first"
        second = Path(temp) / "second"
        kwargs = dict(repo=REPO, strict_root=STRICT, staged_root=STAGED,
                      work_root=WORK, realization_root=REALIZATION,
                      summary_path=SUMMARY)
        generate_external_suite(output=first, **kwargs)
        generate_external_suite(output=second, **kwargs)

        first_files = {path.relative_to(first): path.read_bytes()
                       for path in first.rglob("*") if path.is_file()}
        second_files = {path.relative_to(second): path.read_bytes()
                        for path in second.rglob("*") if path.is_file()}
        assert first_files == second_files, "external strict output is not deterministic"

        suites = {}
        scores = {}
        for method, (expected_points, expected_passes) in EXPECTED.items():
            suite = json.loads((first / method / "suite_score.json").read_text())
            assert suite["total_points"] == expected_points, suite
            assert suite["strict_passes"] == expected_passes, suite
            values = load_scores(first, method)
            assert len(values) == 10
            for value in values:
                validate_external_score(value, repo=REPO)
                assert len([check for layer in value["layers"] for check in layer["checks"]]) == 13
            assert aggregate_external_scores(values, method) == suite
            suites[method], scores[method] = suite, values

        codex_pass = next(value for value in scores["codex"]
                          if value["task_id"] == "01_single_stage_4to1")
        assert codex_pass["total_points"] == 100
        assert codex_pass["strict_verdict"] == "PASS"

        gated = next(value for value in scores["claude-code"]
                     if value["task_id"] == "02_two_stage_9to1")
        assert gated["total_points"] == 40
        physics = gated["layers"][3]
        functional = gated["layers"][4]
        assert physics["status"] == "FAIL" and physics["points_awarded"] == 0
        assert functional["status"] == "GATED" and functional["points_awarded"] == 0
        assert all(check["raw_status"] == "PASS" for check in physics["checks"])
        assert all(check["raw_status"] == "PASS" for check in functional["checks"])

        partial = next(value for value in scores["claude-code"]
                       if value["task_id"] == "01_single_stage_4to1")
        assert partial["layers"][2]["points_awarded"] == 5
        assert partial["total_points"] == 30

        malformed = copy.deepcopy(codex_pass)
        malformed["layers"][0]["checks"][0]["status"] = "UNKNOWN"
        malformed["layers"][0]["checks"][0]["points_awarded"] = 10
        fail(lambda: validate_external_score(malformed), "non-PASS")

        unknown_field = copy.deepcopy(codex_pass)
        unknown_field["invented"] = True
        fail(lambda: validate_external_score(unknown_field), "unknown or missing")

        tampered = copy.deepcopy(codex_pass)
        tampered["layers"][0]["checks"][0]["evidence"][0]["sha256"] = "0" * 64
        fail(lambda: validate_external_score(tampered, repo=REPO), "hash mismatch")

        fail(lambda: aggregate_external_scores(scores["codex"][:-1], "codex"),
             "exactly ten")
        fail(lambda: aggregate_external_scores(scores["codex"] + [scores["codex"][0]], "codex"),
             "duplicate")
        fail(lambda: aggregate_external_scores(scores["claude-code"], "codex"),
             "mixes methods")

        fragment = render_report_fragment(suites, scores)
        assert fragment == (first / "report_fragment.md").read_text(encoding="utf-8")
        assert "310/1000; 0/10 PASS" in fragment
        assert "390/1000; 1/10 PASS" in fragment
        assert "640/1000" not in fragment and "510/1000" not in fragment

    print("golden external strict scoring: PASS")


if __name__ == "__main__":
    main()
