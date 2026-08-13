"""Command-line interface for validation, scoring, and suite aggregation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Sequence

from .aggregate import aggregate_scores, discover_score_files, write_aggregate_json
from .contract import ContractError
from .report import write_score_json, write_score_markdown
from .scoring import score_path, validate_input


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m benchmark_scorer")
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("validate", "score"):
        command = sub.add_parser(name)
        command.add_argument("input", help="portable submission or AutoMech run folder")
        command.add_argument("--input-kind", choices=("auto", "portable", "automech"),
                             default="auto")
        command.add_argument("--task-id", help="explicit Comfort v1 task (AutoMech)")
        if name == "score":
            command.add_argument("--output-dir", required=True,
                                 help="scorer-owned destination outside the imported folder")
    aggregate = sub.add_parser("aggregate")
    aggregate.add_argument("inputs", nargs="+", help="score.json files or directories")
    aggregate.add_argument("--output", help="suite score JSON path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "validate":
            result = validate_input(args.input, input_kind=args.input_kind,
                                    task_id=args.task_id)
            print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
            return 0
        if args.command == "score":
            input_root = Path(args.input).resolve(strict=True)
            output_dir = Path(args.output_dir)
            if output_dir.exists() and output_dir.is_symlink():
                raise ContractError("score output directory must not be a symlink")
            output_resolved = output_dir.resolve(strict=False)
            try:
                output_resolved.relative_to(input_root)
            except ValueError:
                pass
            else:
                raise ContractError("score output directory must be outside the imported folder")
            result = score_path(args.input, input_kind=args.input_kind,
                                task_id=args.task_id)
            score_json = write_score_json(result, output_resolved / "score.json")
            score_md = write_score_markdown(score_json, output_resolved / "score.md")
            print(json.dumps({"score_json": str(score_json), "score_md": str(score_md),
                              "task_id": result.task_id,
                              "total_points": result.total_points}, sort_keys=True))
            return 0
        files = []
        for value in args.inputs:
            files.extend(discover_score_files(value))
        aggregate = aggregate_scores(files)
        if args.output:
            target = write_aggregate_json(aggregate, args.output)
            print(str(target))
        else:
            print(json.dumps(aggregate, indent=2, sort_keys=True, allow_nan=False))
        return 0
    except (ContractError, OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


__all__ = ["main"]
