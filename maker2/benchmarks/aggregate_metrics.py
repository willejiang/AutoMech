"""Aggregate additive benchmark_metrics.json sidecars across artifact directories."""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Iterable


def discover(paths: Iterable[str]) -> list[Path]:
    found = set()
    for raw in paths:
        path = Path(raw)
        if path.is_file() and path.name == "benchmark_metrics.json":
            found.add(path.resolve())
        elif path.is_dir():
            found.update(item.resolve() for item in path.rglob("benchmark_metrics.json"))
    return sorted(found)


def aggregate(paths: Iterable[str]) -> dict:
    files = discover(paths)
    docs = []
    errors = []
    for path in files:
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(doc, dict) or "schema_version" not in doc:
                raise ValueError("not a benchmark metrics object")
            docs.append((path, doc))
        except Exception as error:
            errors.append({"path": str(path), "error": str(error)})

    pass_known = pass_count = pass_unknown = 0
    cold_true = cold_false = cold_unknown = 0
    final_known = final_pass = 0
    durations = []
    iterations = []
    token_totals = Counter()
    requests = tool_calls = tool_errors = 0
    reported_cost = 0.0
    cost_complete_runs = cost_partial_runs = 0
    cache_totals = Counter()
    candidates = submissions = 0
    digests = Counter()
    for _, doc in docs:
        digests[doc.get("task_digest")] += 1
        outcome = doc.get("outcome") or {}
        if outcome.get("ok") is not None:
            final_known += 1
            final_pass += int(outcome.get("ok") is True)
        first = outcome.get("pass_at_1") or {}
        if first.get("known"):
            pass_known += 1
            pass_count += int(first.get("passed") is True)
        else:
            pass_unknown += 1
        qualified = (doc.get("cold_run") or {}).get("qualified")
        if qualified is True:
            cold_true += 1
        elif qualified is False:
            cold_false += 1
        else:
            cold_unknown += 1
        duration = doc.get("duration_s")
        if isinstance(duration, (int, float)):
            durations.append(float(duration))
        iteration = outcome.get("iterations")
        if isinstance(iteration, (int, float)):
            iterations.append(float(iteration))
        usage = doc.get("usage") or {}
        requests += int(usage.get("requests") or 0)
        for name, value in (usage.get("tokens") or {}).items():
            token_totals[name] += int(value or 0)
        if usage.get("cost_usd") is not None:
            reported_cost += float(usage["cost_usd"])
        if usage.get("cost_coverage") == "complete":
            cost_complete_runs += 1
        elif usage.get("cost_coverage") == "partial":
            cost_partial_runs += 1
        tools = doc.get("tools") or {}
        tool_calls += int(tools.get("calls") or 0)
        tool_errors += int(tools.get("errors") or 0)
        cache = doc.get("cache") or {}
        for name in ("hits", "misses", "rejected_hits"):
            cache_totals[name] += int(cache.get(name) or 0)
        compiler = doc.get("mjcf_compiler") or {}
        candidates += int(compiler.get("candidates") or 0)
        submissions += int(compiler.get("submissions") or 0)

    duplicates = {digest: count for digest, count in digests.items()
                  if digest and count > 1}
    return {
        "schema_version": 1,
        "runs": len(docs),
        "files_found": len(files),
        "schema_errors": errors,
        "duplicate_task_digests": duplicates,
        "final_success": {
            "known": final_known, "passed": final_pass,
            "rate": final_pass / final_known if final_known else None,
        },
        "pass_at_1": {
            "known": pass_known, "passed": pass_count, "unknown": pass_unknown,
            "rate": pass_count / pass_known if pass_known else None,
        },
        "cold_run": {"qualified": cold_true, "not_qualified": cold_false,
                     "unknown": cold_unknown},
        "runtime_s": {
            "mean": sum(durations) / len(durations) if durations else None,
            "min": min(durations) if durations else None,
            "max": max(durations) if durations else None,
        },
        "iterations_mean": sum(iterations) / len(iterations) if iterations else None,
        "usage": {"requests": requests, "tokens": dict(token_totals),
                  "cost_usd": round(reported_cost, 8)
                  if (cost_complete_runs or cost_partial_runs) else None,
                  "cost_coverage": "complete" if docs and cost_complete_runs == len(docs)
                  else ("partial" if (cost_complete_runs or cost_partial_runs)
                        else "unavailable")},
        "tools": {"calls": tool_calls, "errors": tool_errors},
        "cache": dict(cache_totals),
        "mjcf_compiler": {"candidates": candidates, "submissions": submissions},
    }


def markdown(summary: dict) -> str:
    final = summary["final_success"]
    first = summary["pass_at_1"]
    runtime = summary["runtime_s"]
    return "\n".join([
        "| Metric | Value |",
        "|---|---:|",
        f"| Runs | {summary['runs']} |",
        f"| Final success | {final['passed']}/{final['known']} |",
        f"| Pass@1 | {first['passed']}/{first['known']} ({first['unknown']} unknown) |",
        f"| Cold-qualified | {summary['cold_run']['qualified']} |",
        f"| Mean runtime (s) | {runtime['mean'] if runtime['mean'] is not None else 'unknown'} |",
        f"| LLM requests | {summary['usage']['requests']} |",
        f"| Tool calls | {summary['tools']['calls']} |",
        f"| MJCF candidates | {summary['mjcf_compiler']['candidates']} |",
    ])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", help="sidecars or directories to scan")
    parser.add_argument("--json-out", default="", help="optional aggregate JSON path")
    parser.add_argument("--markdown", action="store_true", help="print a Markdown summary")
    args = parser.parse_args()
    summary = aggregate(args.paths)
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(markdown(summary) if args.markdown else json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
