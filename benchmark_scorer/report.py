"""Canonical JSON score output and Markdown rendering from score.json only."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

from .contract import ContractError
from .scoring import ScoreResult


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ContractError(f"score.json {label} must be a finite number")
    return float(value)


def _validate_score(value: Any) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or value.get("schema") != "physcad-benchmark-score/1.0":
        raise ContractError("score.json has an unsupported schema")
    for field in ("scorer_version", "suite_id", "task_id", "input_kind"):
        if not isinstance(value.get(field), str) or not value[field]:
            raise ContractError(f"score.json {field} must be a non-empty string")
    if value.get("points_possible") != 100 or value.get("layer_weights") != [10, 15, 15, 20, 40]:
        raise ContractError("score.json maxima or layer_weights are invalid")
    layers = value.get("layers")
    if not isinstance(layers, list) or len(layers) != 5:
        raise ContractError("score.json must contain exactly five layers")
    expected_weights = [10, 15, 15, 20, 40]
    layer_total = 0.0
    unknown_count = 0
    for offset, (layer, weight) in enumerate(zip(layers, expected_weights), start=1):
        if not isinstance(layer, Mapping) or layer.get("index") != offset or layer.get("weight") != weight:
            raise ContractError("score.json layer indices or weights are invalid")
        if not isinstance(layer.get("name"), str) or not layer["name"]:
            raise ContractError("score.json layer name must be non-empty")
        if layer.get("status") not in {"PASS", "FAIL", "UNKNOWN"}:
            raise ContractError("score.json layer status is invalid")
        checks = layer.get("checks")
        if not isinstance(checks, list) or not checks:
            raise ContractError("score.json each layer must contain checks")
        check_possible = check_awarded = 0.0
        statuses = []
        for check in checks:
            if not isinstance(check, Mapping) or not isinstance(check.get("name"), str) or not check["name"]:
                raise ContractError("score.json check is malformed")
            status = check.get("status")
            if status not in {"PASS", "FAIL", "UNKNOWN"}:
                raise ContractError("score.json check status is invalid")
            possible = _number(check.get("points_possible"), "check points_possible")
            awarded = _number(check.get("points_awarded"), "check points_awarded")
            if possible < 0 or awarded < 0 or awarded > possible:
                raise ContractError("score.json check points exceed maxima")
            if status != "PASS" and awarded != 0:
                raise ContractError("FAIL/UNKNOWN checks must award zero points")
            if not isinstance(check.get("reason"), str) or not isinstance(check.get("evidence"), list):
                raise ContractError("score.json check reason/evidence is malformed")
            check_possible += possible
            check_awarded += awarded
            statuses.append(status)
            unknown_count += status == "UNKNOWN"
        expected_status = ("FAIL" if "FAIL" in statuses else
                           "UNKNOWN" if "UNKNOWN" in statuses else "PASS")
        if layer["status"] != expected_status:
            raise ContractError("score.json layer status disagrees with its checks")
        if check_possible != weight:
            raise ContractError("score.json check maxima do not equal layer weight")
        layer_awarded = _number(layer.get("points_awarded"), "layer points_awarded")
        if layer_awarded != check_awarded:
            raise ContractError("score.json layer points disagree with check totals")
        layer_total += layer_awarded
    total = _number(value.get("total_points"), "total_points")
    if total != layer_total or not 0 <= total <= 100:
        raise ContractError("score.json total_points disagrees with layer totals")
    # Never trust a submitted scalar; expose the derived count to consumers.
    if isinstance(value, dict):
        value["unknown_checks"] = unknown_count
    return value


def write_score_json(result: ScoreResult | Mapping[str, Any], path: str | Path) -> Path:
    """Write the authoritative score artifact deterministically."""
    value = result.to_dict() if isinstance(result, ScoreResult) else dict(result)
    _validate_score(value)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
                      encoding="utf-8")
    return target


def load_score_json(path: str | Path) -> Mapping[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ContractError(f"cannot read score.json: {exc}") from exc
    return _validate_score(value)


def render_score_markdown(score: Mapping[str, Any]) -> str:
    """Render Markdown from an already parsed authoritative score document."""
    score = _validate_score(score)
    lines = [
        f"# Benchmark score: {score['task_id']}", "",
        f"**Total:** {score['total_points']}/100  ",
        f"**Suite:** `{score['suite_id']}`  ",
        f"**Input:** `{score['input_kind']}`  ",
        f"**Unknown checks:** {score.get('unknown_checks', 0)}", "",
        "| Layer | Weight | Status | Points |", "|---|---:|---|---:|",
    ]
    for layer in score["layers"]:
        lines.append(f"| {layer['index']}. {layer['name']} | {layer['weight']} | "
                     f"{layer['status']} | {layer['points_awarded']} |")
    for layer in score["layers"]:
        lines.extend(["", f"## {layer['index']}. {layer['name']}", ""])
        if layer.get("prerequisite"):
            lines.append(f"Prerequisite gate: {layer['prerequisite']}")
            lines.append("")
        for check in layer["checks"]:
            lines.append(f"- **{check['status']}** `{check['name']}` — "
                         f"{check['points_awarded']}/{check['points_possible']}: "
                         f"{check['reason']}")
    return "\n".join(lines) + "\n"


def write_score_markdown(score_json_path: str | Path,
                         markdown_path: str | Path | None = None) -> Path:
    """Generate score.md by re-reading score.json; no in-memory score is accepted."""
    source = Path(score_json_path)
    score = load_score_json(source)
    target = Path(markdown_path) if markdown_path else source.with_name("score.md")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(render_score_markdown(score), encoding="utf-8")
    return target


__all__ = ["load_score_json", "render_score_markdown", "write_score_json",
           "write_score_markdown"]
