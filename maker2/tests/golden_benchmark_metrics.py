"""Golden tests for additive benchmark telemetry and aggregation."""
from __future__ import annotations

import json
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace

from maker2.benchmarks.aggregate_metrics import aggregate
from maker2.benchmarks.telemetry import (
    normalize_usage, record_openai_response, start_recorder, stop_recorder,
)


def main() -> int:
    openai = normalize_usage("openai", {
        "prompt_tokens": 100, "completion_tokens": 25, "total_tokens": 125,
        "prompt_tokens_details": {"cached_tokens": 40},
        "completion_tokens_details": {"reasoning_tokens": 10},
    })
    assert openai == {
        "input_tokens": 100, "output_tokens": 25,
        "cache_read_input_tokens": 40, "reasoning_tokens": 10,
        "total_tokens": 125,
    }
    anthropic = normalize_usage("anthropic", {
        "input_tokens": 80, "output_tokens": 20,
        "cache_read_input_tokens": 30, "cache_creation_input_tokens": 10,
    })
    assert anthropic == {
        "input_tokens": 80, "output_tokens": 20,
        "cache_read_input_tokens": 30, "cache_write_input_tokens": 10,
        "total_tokens": 100,
    }
    assert normalize_usage("openai", None) is None
    assert normalize_usage("anthropic", {
        "input_tokens": 3, "output_tokens": 4, "reasoning_tokens": 2,
    })["reasoning_tokens"] == 2

    with tempfile.TemporaryDirectory(prefix="golden_benchmark_") as temp:
        root = Path(temp)
        run_a = root / "run_a"
        recorder = start_recorder(task="fixture A", out_dir=temp,
                                  pipeline="single_agent", cold_requested=True)
        recorder.configure(provider="local_gateway", model="fixture", engine="mujoco")
        response = SimpleNamespace(
            model="fixture",
            usage=SimpleNamespace(
                prompt_tokens=10, completion_tokens=5, total_tokens=15,
                input_tokens=None, output_tokens=None,
                cache_read_input_tokens=None, cache_creation_input_tokens=None,
                prompt_tokens_details=SimpleNamespace(cached_tokens=0),
                completion_tokens_details=SimpleNamespace(reasoning_tokens=2),
            ),
        )
        record_openai_response(response, model="fixture", duration_s=0.01)
        recorder.record_cache("mjcf_compiler", "miss")
        recorder.record_compiler_submission()
        recorder.record_compiler_candidate()
        recorder.record_first_attempt(True, stage="physics")

        def tools():
            for _ in range(25):
                recorder.record_tool("query_pair_geometry", duration_s=0.001, error=False)

        workers = [threading.Thread(target=tools) for _ in range(4)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()
        sidecar = recorder.finalize({
            "ok": True, "run_dir": str(run_a), "iterations": 1,
            "physics": {"passed": True, "metrics": {"functional_ok": True}},
        })
        stop_recorder(recorder)
        doc = json.loads(Path(sidecar).read_text(encoding="utf-8"))
        assert doc["schema_version"] == 1
        assert doc["outcome"]["pass_at_1"]["passed"] is True
        assert doc["cold_run"]["qualified"] is True
        assert doc["tools"]["calls"] == 100
        assert doc["usage"]["tokens"]["total_tokens"] == 15
        assert doc["usage"]["cost_usd"] is None
        assert doc["mjcf_compiler"] == {"candidates": 1, "submissions": 1}

        run_b = root / "run_b"
        recorder_b = start_recorder(task="fixture B", out_dir=temp,
                                    pipeline="simple", cold_requested=False)
        recorder_b.record_cache("mjcf_compiler", "hit")
        recorder_b.finalize({"ok": False, "run_dir": str(run_b), "iterations": 2})
        stop_recorder(recorder_b)

        summary = aggregate([temp])
        assert summary["runs"] == 2
        assert summary["final_success"] == {"known": 2, "passed": 1, "rate": 0.5}
        assert summary["pass_at_1"] == {"known": 2, "passed": 1,
                                             "unknown": 0, "rate": 0.5}
        assert summary["cold_run"] == {"qualified": 1, "not_qualified": 0,
                                             "unknown": 1}
        assert summary["tools"]["calls"] == 100
        assert summary["usage"]["cost_usd"] is None

        run_c = root / "run_c"
        recorder_c = start_recorder(task="fixture C", out_dir=temp,
                                    pipeline="single_agent", cold_requested=True)
        recorder_c.record_cache("mjcf_compiler", "rejected_hit")
        recorder_c.record_llm(provider="fixture", model="fixture",
                              usage={"input_tokens": 1, "output_tokens": 1,
                                     "cache_read_input_tokens": 0})
        recorder_c.finalize({"ok": False, "run_dir": str(run_c), "iterations": 1})
        stop_recorder(recorder_c)
        doc_c = json.loads((run_c / "benchmark_metrics.json").read_text(encoding="utf-8"))
        assert doc_c["cold_run"]["qualified"] is False

        run_d = root / "run_d"
        recorder_d = start_recorder(task="fixture D", out_dir=temp,
                                    pipeline="single_agent", cold_requested=False)
        recorder_d.record_llm(provider="fixture", model="fixture",
                              usage={"input_tokens": 2, "output_tokens": 1,
                                     "cost_usd": 0.0125})
        recorder_d.record_compiler_submission()
        recorder_d.record_compiler_candidate()
        recorder_d.record_compiler_submission()
        recorder_d.record_compiler_candidate()
        recorder_d.finalize({"ok": True, "run_dir": str(run_d), "iterations": 1})
        stop_recorder(recorder_d)
        doc_d = json.loads((run_d / "benchmark_metrics.json").read_text(encoding="utf-8"))
        assert doc_d["usage"]["cost_usd"] == 0.0125
        assert doc_d["usage"]["cost_coverage"] == "complete"
        assert doc_d["outcome"]["pass_at_1"]["passed"] is False
        assert doc_d["outcome"]["pass_at_1"]["stage"] == "mjcf_compiler_replacement"

    print("golden benchmark metrics: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
