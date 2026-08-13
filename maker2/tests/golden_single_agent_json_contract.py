"""Golden checks for the single-agent CLI JSON and persistence contract."""
from __future__ import annotations

import json
import math
import tempfile
from pathlib import Path

from maker2.jsonutil import strict_json_dumps
from maker2.single_agent import _artifact_json, _iter_score, persist_single_agent_run


def main() -> int:
    harness_fault = {
        "passed": False,
        "diagnosis": {
            "fault_domain": "simulator_numerics",
            "routing": {"allow_agent_refinement": False},
        },
    }
    score = _iter_score(harness_fault)
    assert math.isfinite(score), score
    assert score < -100_000.0, score
    successful_stop = {
        "passed": True,
        "diagnosis": {"routing": {"allow_agent_refinement": False}},
    }
    assert _iter_score(successful_stop) == 10_000.0

    event_line = _artifact_json({
        "kind": "physics",
        "iter": 0,
        "score": float("-inf"),
        "physics": {"metrics": {"ratio": float("nan"), "travel": float("inf")}},
    })
    assert event_line.startswith("ARTIFACT_JSON:")
    event_text = event_line.removeprefix("ARTIFACT_JSON:")
    assert all(token not in event_text for token in ("NaN", "Infinity")), event_text
    event = json.loads(event_text)
    assert event["score"] is None
    assert event["physics"]["metrics"] == {"ratio": None, "travel": None}

    with tempfile.TemporaryDirectory(prefix="golden_single_agent_json_") as temp:
        run_dir = Path(temp) / "run"
        raw_result = {
            "ok": False,
            "run_dir": str(run_dir),
            "render_dir": str(run_dir),
            "iterations": 1,
            "hierarchy": False,
            "single_agent": True,
            "physics": {"metrics": {"ratio": float("nan")}},
        }
        result = persist_single_agent_run(
            raw_result,
            prompt="fixture mechanism",
            model="fixture-model",
            max_iters=3,
            refine_message=None,
            thread="fixture-thread",
        )
        assert result is not raw_result
        assert result["physics"]["metrics"]["ratio"] is None

        result_text = (run_dir / "result.json").read_text(encoding="utf-8")
        run_text = (run_dir / "run.json").read_text(encoding="utf-8")
        assert all(token not in result_text + run_text for token in ("NaN", "Infinity"))
        assert json.loads(result_text) == result
        run_doc = json.loads(run_text)
        assert run_doc["prompt"] == "fixture mechanism"
        assert run_doc["model"] == "fixture-model"
        assert run_doc["max_iters"] == 3
        assert run_doc["thread"] == "fixture-thread"
        assert run_doc["single_agent"] is True

        result_line = "RESULT_JSON:" + strict_json_dumps(
            result, separators=(",", ":"))
        assert json.loads(result_line.removeprefix("RESULT_JSON:")) == result

    print("golden single-agent JSON contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
