"""Best-effort run telemetry for reproducible cross-harness benchmarks.

The recorder is process-global on purpose: maker2 runs one logical benchmark per process, while
hierarchy workers may call the shared LLM/tool clients from several threads. Existing result.json
contracts stay untouched; this module writes an additive benchmark_metrics.json sidecar.
"""
from __future__ import annotations

import hashlib
import json
import os
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1
_ACTIVE = None
_ACTIVE_LOCK = threading.RLock()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    try:
        return max(0, int(value))
    except (TypeError, ValueError, OverflowError):
        return None


def _reported_cost_usd(usage: Any) -> float | None:
    """Return only a provider-reported monetary cost; never apply remembered pricing."""
    if usage is None:
        return None
    for name in ("cost_usd", "total_cost", "cost"):
        value = usage.get(name) if isinstance(usage, dict) else getattr(usage, name, None)
        if isinstance(value, bool) or value is None:
            continue
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError):
            continue
        if value >= 0 and value < float("inf"):
            return value
    return None


def normalize_usage(provider: str, usage: Any) -> dict[str, int] | None:
    """Normalize OpenAI-compatible or Anthropic usage without estimating missing data."""
    if usage is None:
        return None
    if not isinstance(usage, dict):
        usage = {name: getattr(usage, name, None) for name in (
            "prompt_tokens", "completion_tokens", "total_tokens", "input_tokens",
            "output_tokens", "cache_read_input_tokens", "cache_creation_input_tokens",
            "prompt_tokens_details", "completion_tokens_details", "reasoning_tokens",
        )}
    prompt_details = usage.get("prompt_tokens_details") or {}
    completion_details = usage.get("completion_tokens_details") or {}
    if not isinstance(prompt_details, dict):
        prompt_details = {
            "cached_tokens": getattr(prompt_details, "cached_tokens", None),
        }
    if not isinstance(completion_details, dict):
        completion_details = {
            "reasoning_tokens": getattr(completion_details, "reasoning_tokens", None),
        }
    values = {
        "input_tokens": _integer(usage.get("input_tokens")),
        "output_tokens": _integer(usage.get("output_tokens")),
        "cache_read_input_tokens": _integer(usage.get("cache_read_input_tokens")),
        "cache_write_input_tokens": _integer(usage.get("cache_creation_input_tokens")),
        "reasoning_tokens": _integer(
            usage.get("reasoning_tokens")
            if usage.get("reasoning_tokens") is not None
            else completion_details.get("reasoning_tokens")),
    }
    if values["input_tokens"] is None:
        values["input_tokens"] = _integer(usage.get("prompt_tokens"))
    if values["output_tokens"] is None:
        values["output_tokens"] = _integer(usage.get("completion_tokens"))
    if values["cache_read_input_tokens"] is None:
        values["cache_read_input_tokens"] = _integer(prompt_details.get("cached_tokens"))
    result = {key: value for key, value in values.items() if value is not None}
    total = _integer(usage.get("total_tokens"))
    if total is None and ("input_tokens" in result or "output_tokens" in result):
        total = result.get("input_tokens", 0) + result.get("output_tokens", 0)
    if total is not None:
        result["total_tokens"] = total
    return result or None


class BenchmarkRecorder:
    def __init__(self, *, task: str, out_dir: str, pipeline: str,
                 cold_requested: bool = False):
        self.run_id = uuid.uuid4().hex
        self.task_digest = hashlib.sha256(task.encode("utf-8")).hexdigest()
        self.out_dir = str(out_dir)
        self.pipeline = pipeline
        self.cold_requested = bool(cold_requested)
        self.started_at = _utc_now()
        self._started_perf = time.perf_counter()
        self._lock = threading.RLock()
        self._run_dir: str | None = None
        self._metadata: dict[str, Any] = {}
        self._requests: list[dict[str, Any]] = []
        self._tools: dict[str, dict[str, int]] = {}
        self._cache_events: list[dict[str, str]] = []
        self._compiler = {"candidates": 0, "submissions": 0}
        self._first_attempt: dict[str, Any] | None = None
        self._finalized = False

    @property
    def cold_cache_dir(self) -> str:
        return str(Path(self.out_dir, ".benchmark_cold", self.run_id, "mjcf_agent"))

    def configure(self, **metadata: Any) -> None:
        with self._lock:
            self._metadata.update({key: value for key, value in metadata.items()
                                   if value is not None})

    def bind_run_dir(self, run_dir: str | os.PathLike[str] | None) -> None:
        if run_dir:
            with self._lock:
                self._run_dir = os.fspath(run_dir)

    def record_llm(self, *, provider: str, model: str, usage: Any,
                   duration_s: float | None = None, error: bool = False) -> None:
        normalized = normalize_usage(provider, usage)
        with self._lock:
            self._requests.append({
                "provider": provider or "unknown",
                "model": model or "unknown",
                "duration_s": round(max(0.0, float(duration_s)), 6)
                if duration_s is not None else None,
                "usage": normalized,
                "cost_usd": _reported_cost_usd(usage),
                "error": bool(error),
            })

    def record_tool(self, name: str, *, duration_s: float, error: bool) -> None:
        with self._lock:
            row = self._tools.setdefault(name or "unknown", {"calls": 0, "errors": 0})
            row["calls"] += 1
            row["errors"] += int(bool(error))
            row["duration_ms"] = row.get("duration_ms", 0) + round(
                max(0.0, float(duration_s)) * 1000)

    def record_cache(self, name: str, outcome: str) -> None:
        if outcome not in {"hit", "miss", "rejected_hit"}:
            return
        with self._lock:
            self._cache_events.append({"name": name, "outcome": outcome})

    def record_compiler_candidate(self) -> None:
        with self._lock:
            self._compiler["candidates"] += 1
            if self._compiler["candidates"] > 1 and self._first_attempt is None:
                self._first_attempt = {
                    "known": True, "passed": False,
                    "stage": "mjcf_compiler_refinement", "fault_domain": "builder_compiler",
                    "reason": "the first MJCF compiler candidate did not pass its gate",
                }

    def record_compiler_submission(self) -> None:
        with self._lock:
            self._compiler["submissions"] += 1
            if self._compiler["submissions"] > 1 and self._first_attempt is None:
                self._first_attempt = {
                    "known": True, "passed": False,
                    "stage": "mjcf_compiler_replacement", "fault_domain": "builder_compiler",
                    "reason": "the first MJCF compiler source required replacement",
                }

    def record_first_attempt(self, passed: bool, *, stage: str,
                             fault_domain: str | None = None, reason: str = "") -> None:
        with self._lock:
            if self._first_attempt is None:
                self._first_attempt = {
                    "known": True,
                    "passed": bool(passed),
                    "stage": stage,
                    "fault_domain": fault_domain,
                    "reason": str(reason)[:300],
                }

    def _usage_summary(self) -> dict[str, Any]:
        totals = {name: 0 for name in (
            "input_tokens", "output_tokens", "cache_read_input_tokens",
            "cache_write_input_tokens", "reasoning_tokens", "total_tokens",
        )}
        known = 0
        for request in self._requests:
            usage = request.get("usage")
            if not usage:
                continue
            known += 1
            for name in totals:
                totals[name] += int(usage.get(name, 0))
        reported_costs = [row["cost_usd"] for row in self._requests
                          if row.get("cost_usd") is not None]
        if reported_costs and len(reported_costs) == len(self._requests):
            cost_coverage = "complete"
        elif reported_costs:
            cost_coverage = "partial"
        else:
            cost_coverage = "unavailable"
        return {
            "requests": len(self._requests),
            "errors": sum(int(row["error"]) for row in self._requests),
            "usage_known_requests": known,
            "usage_unknown_requests": len(self._requests) - known,
            "tokens": totals,
            "cost_usd": round(sum(reported_costs), 8) if reported_costs else None,
            "cost_coverage": cost_coverage,
        }

    def _cold_summary(self) -> dict[str, Any]:
        outcomes = [event["outcome"] for event in self._cache_events]
        cache_reads = [
            (request.get("usage") or {}).get("cache_read_input_tokens")
            for request in self._requests
        ]
        if not self.cold_requested:
            qualified = None
            reasons = ["cold mode was not requested"]
        elif ({"hit", "rejected_hit"} & set(outcomes)
              or any((value or 0) > 0 for value in cache_reads)):
            qualified = False
            reasons = ["a project or provider cache hit was observed"]
        elif (not self._cache_events or not self._requests
              or any(value is None for value in cache_reads)):
            qualified = None
            reasons = ["cache evidence is incomplete"]
        else:
            qualified = True
            reasons = ["all observed project caches missed and provider cache reads were zero"]
        return {
            "requested": self.cold_requested,
            "qualified": qualified,
            "reasons": reasons,
        }

    def finalize(self, result: dict[str, Any] | None = None,
                 run_dir: str | os.PathLike[str] | None = None) -> str | None:
        with self._lock:
            if self._finalized:
                return None
            self._finalized = True
            self.bind_run_dir(run_dir or ((result or {}).get("run_dir")))
            target = self._run_dir
            if not target:
                return None
            result = result or {}
            if self._first_attempt is None and result:
                self._first_attempt = {
                    "known": True,
                    "passed": False,
                    "stage": "run_ended_before_terminal_gate",
                    "fault_domain": None,
                    "reason": str(result.get("error") or "run ended before a complete first candidate")[:300],
                }
            physics = result.get("physics") or {}
            tools_total = sum(row["calls"] for row in self._tools.values())
            tools_errors = sum(row["errors"] for row in self._tools.values())
            payload = {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.run_id,
                "task_digest": self.task_digest,
                "started_at": self.started_at,
                "ended_at": _utc_now(),
                "duration_s": round(time.perf_counter() - self._started_perf, 6),
                "pipeline": {"mode": self.pipeline, **self._metadata},
                "outcome": {
                    "ok": result.get("ok"),
                    "iterations": result.get("iterations"),
                    "physics_passed": physics.get("passed"),
                    "functional_ok": (physics.get("metrics") or {}).get("functional_ok"),
                    "pass_at_1": self._first_attempt or {
                        "known": False, "passed": None, "stage": None,
                        "fault_domain": None, "reason": "first-attempt event unavailable",
                    },
                },
                "cold_run": self._cold_summary(),
                "usage": self._usage_summary(),
                "tools": {"calls": tools_total, "errors": tools_errors,
                          "by_name": dict(sorted(self._tools.items()))},
                "cache": {
                    "hits": outcomes_count(self._cache_events, "hit"),
                    "misses": outcomes_count(self._cache_events, "miss"),
                    "rejected_hits": outcomes_count(self._cache_events, "rejected_hit"),
                    "events": list(self._cache_events),
                },
                "mjcf_compiler": dict(self._compiler),
            }
            try:
                path = Path(target, "benchmark_metrics.json")
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
                return str(path)
            except Exception:
                return None


def outcomes_count(events: list[dict[str, str]], outcome: str) -> int:
    return sum(1 for event in events if event.get("outcome") == outcome)


def start_recorder(*, task: str, out_dir: str, pipeline: str,
                   cold_requested: bool = False) -> BenchmarkRecorder:
    global _ACTIVE
    recorder = BenchmarkRecorder(task=task, out_dir=out_dir, pipeline=pipeline,
                                 cold_requested=cold_requested)
    with _ACTIVE_LOCK:
        _ACTIVE = recorder
    return recorder


def active_recorder() -> BenchmarkRecorder | None:
    with _ACTIVE_LOCK:
        return _ACTIVE


def stop_recorder(recorder: BenchmarkRecorder | None = None) -> None:
    global _ACTIVE
    with _ACTIVE_LOCK:
        if recorder is None or _ACTIVE is recorder:
            _ACTIVE = None


def record_openai_response(response: Any, *, provider: str = "openai",
                           model: str = "", duration_s: float | None = None) -> None:
    recorder = active_recorder()
    if recorder is not None:
        recorder.record_llm(provider=provider, model=model or getattr(response, "model", ""),
                            usage=getattr(response, "usage", None), duration_s=duration_s)


def record_first_attempt(passed: bool, *, stage: str,
                         fault_domain: str | None = None, reason: str = "") -> None:
    recorder = active_recorder()
    if recorder is not None:
        recorder.record_first_attempt(passed, stage=stage, fault_domain=fault_domain,
                                      reason=reason)


__all__ = [
    "BenchmarkRecorder", "SCHEMA_VERSION", "active_recorder", "normalize_usage",
    "record_first_attempt", "record_openai_response", "start_recorder", "stop_recorder",
]
