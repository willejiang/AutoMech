"""Regression checks for designer-authored functional metric normalization."""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    runner = root / "evaluator" / "_metrics_runner.py"
    checks = [
        ("planet_range", 0.25, "0.01 <= ratio <= 0.95", True),
        ("below_range", 0.001, "0.01 <= ratio <= 0.95", True),
        ("above_range", 1.0, "0.01 <= ratio <= 0.95", True),
        ("lower_endpoint", 0.01, "0.01 <= ratio <= 0.95", True),
        ("upper_endpoint", 0.95, "0.01 <= ratio <= 0.95", True),
        ("strict_endpoint", 0.01, "0.01 < ratio < 0.95", True),
        ("descending", -2.0, "1 >= value >= -3", True),
        ("scientific", 0.25, "1e-2 <= ratio <= 9.5e-1", True),
        ("between", 12.0, "between 10 and 20", True),
        ("ratio_target", 12.0, "12:1", True),
        ("one_sided", 2.0, "at least 1.0", True),
        ("designer_fail_stays_fail", 0.25, "0.01 <= ratio <= 0.95", False),
        ("nonnumeric", "closed", "closes", True),
    ]
    code = "def check(traj, result):\n    return " + repr([
        {"name": name, "value": value, "expected": expected,
         "passed": passed, "detail": "fixture"}
        for name, value, expected, passed in checks
    ])

    with tempfile.TemporaryDirectory(prefix="golden_metrics_") as temp:
        temp_path = Path(temp)
        code_path = temp_path / "metrics.py"
        trajectory_path = temp_path / "trajectory.json"
        result_path = temp_path / "sim_result.json"
        output_path = temp_path / "metrics_result.json"
        code_path.write_text(code, encoding="utf-8")
        trajectory_path.write_text("{}", encoding="utf-8")
        result_path.write_text('{"metrics": {}}', encoding="utf-8")
        completed = subprocess.run(
            [sys.executable, str(runner), str(code_path), str(trajectory_path),
             str(result_path), str(output_path)],
            check=False, capture_output=True, text=True, timeout=10,
        )
        assert completed.returncode == 0, completed.stderr
        payload = json.loads(output_path.read_text(encoding="utf-8"))

    assert set(payload) == {"ok", "checks"}
    assert payload["ok"] is True
    by_name = {item["name"]: item for item in payload["checks"]}
    for item in by_name.values():
        assert set(item) == {"name", "value", "expected", "passed", "detail"}

    for name in ("planet_range", "lower_endpoint", "upper_endpoint", "descending",
                 "scientific", "between", "ratio_target", "one_sided", "nonnumeric"):
        assert by_name[name]["passed"] is True, by_name[name]
    for name in ("below_range", "above_range", "strict_endpoint",
                 "designer_fail_stays_fail"):
        assert by_name[name]["passed"] is False, by_name[name]
    assert "outside" in by_name["below_range"]["detail"]
    print("golden metrics runner: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
