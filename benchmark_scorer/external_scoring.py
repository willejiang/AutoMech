"""Canonical strict scoring for normalized Claude Code and Codex submissions.

The older external score artifacts were produced before the strict geometry and
MJCF audits existed.  This module deliberately consumes the scorer-owned replay
and strict audit outputs instead and preserves raw diagnostic results separately
from prerequisite-gated awarded points.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PureWindowsPath
from typing import Any, Mapping, Sequence

from .contract import ContractError, validate_relative_path
from .external_evidence import derive_external_invariants
from .scoring import _Context, _assembly_layer, _functional_layer, _trajectory_layer
from .tasks.comfort_v1 import TASKS, get_task

SCHEMA = "physcad-external-strict-score/1.0"
SUITE_SCHEMA = "physcad-external-strict-suite-score/1.0"
METHODS = ("claude-code", "codex")
LAYER_WEIGHTS = (10, 15, 15, 20, 40)
CHECKS = (
    ("execution", "scorer_owned_execution", 10),
    ("assembly", "assembly_model", 5),
    ("assembly", "required_role_cardinalities", 7),
    ("assembly", "input_output_independence", 3),
    ("geometry", "mesh_inventory", 5),
    ("geometry", "non_exempt_conflicts", 10),
    ("physics_ready", "trajectory_shape", 5),
    ("physics_ready", "sample_alignment", 5),
    ("physics_ready", "finite_health", 10),
    ("functional", "input_motion", 5),
    ("functional", "motion_propagation", 10),
    ("functional", "registered_output", 15),
    ("functional", "registered_invariants", 10),
)


def _read(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ContractError(f"cannot read strict evidence {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ContractError(f"strict evidence must be a JSON object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, value: Mapping[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dict(value), indent=2, sort_keys=True,
                               allow_nan=False) + "\n", encoding="utf-8")
    return path


def _relative(path: Path, repo: Path) -> str:
    try:
        value = path.resolve().relative_to(repo.resolve()).as_posix()
    except ValueError as exc:
        raise ContractError(f"strict evidence is outside repository: {path}") from exc
    return validate_relative_path(value)


def _evidence(path: Path, repo: Path, *, kind: str,
              source: str = "scorer_owned") -> dict[str, str]:
    return {"kind": kind, "path": _relative(path, repo), "sha256": _sha256(path),
            "source": source}


def _status(value: Any) -> str:
    return value if value in {"PASS", "FAIL", "UNKNOWN"} else "UNKNOWN"


def _check(name: str, raw_status: str, possible: int, reason: str,
           evidence: Mapping[str, str] | Sequence[Mapping[str, str]], *,
           gate: str | None = None) -> dict[str, Any]:
    raw_status = _status(raw_status)
    status = "GATED" if gate else raw_status
    return {
        "name": name,
        "raw_status": raw_status,
        "status": status,
        "points_possible": possible,
        "points_awarded": possible if status == "PASS" else 0,
        "reason": reason,
        "gate": gate,
        "evidence": ([dict(item) for item in evidence]
                     if isinstance(evidence, (list, tuple)) else [dict(evidence)]),
    }


def _raw_checks(layer) -> dict[str, Any]:
    return {item.name: item for item in layer.checks}


def _first_failure(checks: Sequence[Mapping[str, Any]]) -> str | None:
    for check in checks:
        if check["status"] in {"FAIL", "UNKNOWN"}:
            return f"{check['name']}: {check['reason']}"
    return None


def _strict_model_gate(model: Mapping[str, Any]) -> tuple[bool, str]:
    required = ("collision_coverage_ok", "actuator_policy_ok", "closure_ok",
                "transmission_binding_ok")
    failed = [name for name in required if model.get(name) is not True]
    if model.get("passed") is not True and not failed:
        failed.append("model_audit.passed")
    errors = [str(value) for value in model.get("errors", ())]
    detail = ", ".join(failed + errors)
    return not failed and model.get("passed") is True, detail or "strict model audit passed"


def score_external_task(*, repo: str | Path, method: str, task_id: str,
                        strict_root: str | Path, staged_root: str | Path,
                        work_root: str | Path, realization_root: str | Path,
                        summary: Mapping[str, Any] | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    """Score one external task from strict, scorer-owned evidence."""
    repo = Path(repo)
    if method not in METHODS:
        raise ContractError(f"unsupported external method: {method!r}")
    task = get_task(task_id)
    strict = Path(strict_root) / method / task_id
    staged = Path(staged_root) / method / task_id
    work = Path(work_root) / method / task_id
    realization_path = Path(realization_root) / method / f"{task_id}.json"

    paths = {
        "assembly": staged / "assembly.json",
        "bindings": staged / "task_bindings.json",
        "contacts": staged / "evidence" / "contacts.json",
        "source_execution": work / "source" / "source_execution.json",
        "replay_status": work / "replay_status.json",
        "trajectory": work / "replay" / "trajectory.json",
        "geometry": strict / "strict" / "geometry.json",
        "model_audit": strict / "strict" / "model_audit.json",
        "anchor_audit": strict / "strict" / "anchor_audit.json",
        "realization": realization_path,
    }
    documents = {name: _read(path) for name, path in paths.items()}
    assembly = documents["assembly"]
    bindings_doc = documents["bindings"]
    bindings_raw = bindings_doc.get("roles")
    if not isinstance(bindings_raw, Mapping):
        raise ContractError(f"task bindings lack roles: {paths['bindings']}")
    bindings = {str(name): tuple(str(value) for value in values)
                for name, values in bindings_raw.items() if isinstance(values, list)}
    trajectory = documents["trajectory"]
    contacts = documents["contacts"]
    invariants = derive_external_invariants(strict, assembly, bindings_raw,
                                             trajectory, trajectory, contacts)
    trajectory_with_invariants = dict(trajectory)
    trajectory_with_invariants["invariants"] = invariants
    context = _Context(
        strict, task, "external_strict", "physcad-comfort-v1",
        {"assembly.json": assembly, "task_bindings.json": bindings_doc,
         "evidence/trajectory.json": trajectory_with_invariants,
         "evidence/contacts.json": contacts},
        (), {"model_mjcf": "models/model.mjcf"}, bindings,
        trajectory=trajectory_with_invariants)
    assembly_raw = _raw_checks(_assembly_layer(context))
    physics_raw = _raw_checks(_trajectory_layer(context))
    functional_raw = _raw_checks(_functional_layer(context))

    source = documents["source_execution"]
    replay = documents["replay_status"]
    replay_meta = replay.get("metadata") if isinstance(replay.get("metadata"), Mapping) else {}
    finite = ((replay_meta.get("finite_health") or {}).get("all_finite")
              if isinstance(replay_meta, Mapping) else None)
    execution_ok = (source.get("build_ok") is True
                    and source.get("nonempty_part_set") is True
                    and replay.get("ok") is True and finite is True)
    execution_status = "PASS" if execution_ok else "FAIL"
    execution_reason = ("scorer-owned source build produced a nonempty part set; MJCF replay compiled, initialized, and stayed finite"
                        if execution_ok else "source execution or scorer replay failed")

    geometry = documents["geometry"]
    mesh_hashes = geometry.get("input_hashes")
    mesh_ok = (isinstance(mesh_hashes, Mapping) and bool(mesh_hashes)
               and int(geometry.get("links_scanned", 0)) == len(mesh_hashes))
    conflict_count = geometry.get("non_exempt_conflict_count")
    unavailable = geometry.get("scan_unavailable")
    conflict_known = isinstance(conflict_count, int) and not isinstance(conflict_count, bool)
    geometry_ok = conflict_known and conflict_count == 0 and unavailable == []

    model = documents["model_audit"]
    model_ok, model_reason = _strict_model_gate(model)
    submitted_kinds = {"assembly", "bindings", "contacts"}
    provenance = {
        name: _evidence(path, repo, kind=name,
                        source="normalized_submission" if name in submitted_kinds else "scorer_owned")
        for name, path in paths.items()
    }
    evidence_summary = {
        "schema": "physcad-external-strict-evidence-summary/1.0",
        "suite_id": "physcad-comfort-v1", "method": method, "task_id": task_id,
        "source_execution": source,
        "replay": {"ok": replay.get("ok"), "metadata": replay_meta},
        "geometry": {
            "links_scanned": geometry.get("links_scanned"),
            "mesh_count": len(mesh_hashes) if isinstance(mesh_hashes, Mapping) else None,
            "non_exempt_conflict_count": conflict_count,
            "scan_unavailable": unavailable,
        },
        "model_audit": {
            "passed": model.get("passed"),
            "collision_coverage_ok": model.get("collision_coverage_ok"),
            "actuator_policy_ok": model.get("actuator_policy_ok"),
            "closure_ok": model.get("closure_ok"),
            "transmission_binding_ok": model.get("transmission_binding_ok"),
            "errors": model.get("errors", []),
        },
        "anchor_audit": {
            "anchored_coverage": documents["anchor_audit"].get("anchored_coverage"),
            "floating_physical_links": documents["anchor_audit"].get("floating_physical_links"),
        },
        "declared_realization": documents["realization"],
        "derived_invariants": invariants,
        "raw_physics_checks": {name: {"status": item.status, "reason": item.reason}
                               for name, item in physics_raw.items()},
        "raw_functional_checks": {name: {"status": item.status, "reason": item.reason}
                                  for name, item in functional_raw.items()},
        "inputs": provenance,
    }

    checks: list[dict[str, Any]] = []
    checks.append(_check("scorer_owned_execution", execution_status, 10,
                         execution_reason, provenance["source_execution"]))
    gate = _first_failure(checks)
    for name, points in (("assembly_model", 5), ("required_role_cardinalities", 7),
                         ("input_output_independence", 3)):
        raw = assembly_raw[name]
        checks.append(_check(name, raw.status, points, raw.reason,
                             provenance["assembly"], gate=gate))
    gate = gate or _first_failure(checks[1:4])
    checks.append(_check("mesh_inventory", "PASS" if mesh_ok else "FAIL", 5,
                         "strict geometry hashes every scanned usable mesh" if mesh_ok else
                         "strict geometry mesh inventory is incomplete",
                         provenance["geometry"], gate=gate))
    checks.append(_check("non_exempt_conflicts",
                         "PASS" if geometry_ok else ("FAIL" if conflict_known else "UNKNOWN"),
                         10,
                         f"strict non-exempt conflicts={conflict_count}; unavailable={len(unavailable) if isinstance(unavailable, list) else 'unknown'}",
                         provenance["geometry"], gate=gate))
    gate = gate or _first_failure(checks[4:6])
    upstream_gate = gate
    physics_failure = (None if model_ok else f"strict_model_audit: {model_reason}")
    physics_gate = upstream_gate or physics_failure
    for name, points in (("trajectory_shape", 5), ("sample_alignment", 5),
                         ("finite_health", 10)):
        raw = physics_raw[name]
        checks.append(_check(name, raw.status, points, raw.reason,
                             (provenance["trajectory"], provenance["model_audit"]),
                             gate=physics_gate))
    gate = physics_gate or _first_failure(checks[6:9])
    for name, points in (("input_motion", 5), ("motion_propagation", 10),
                         ("registered_output", 15), ("registered_invariants", 10)):
        raw = functional_raw[name]
        checks.append(_check(name, raw.status, points, raw.reason,
                             provenance["trajectory"], gate=gate))

    by_layer: dict[str, list[dict[str, Any]]] = {}
    for check in checks:
        layer = next(layer for layer, check_name, _ in CHECKS if check_name == check["name"])
        by_layer.setdefault(layer, []).append(check)
    layers = []
    for index, (name, weight) in enumerate(zip(
            ("execution", "assembly", "geometry", "physics_ready", "functional"),
            LAYER_WEIGHTS), start=1):
        layer_checks = by_layer[name]
        statuses = [check["status"] for check in layer_checks]
        status = ("GATED" if "GATED" in statuses else "FAIL" if "FAIL" in statuses
                  else "UNKNOWN" if "UNKNOWN" in statuses else "PASS")
        if name == "physics_ready" and not model_ok and not upstream_gate:
            status = "FAIL"
        layers.append({
            "index": index, "name": name, "weight": weight, "status": status,
            "points_awarded": sum(check["points_awarded"] for check in layer_checks),
            "checks": layer_checks,
            "strict_gate": ({"passed": model_ok, "reason": model_reason}
                            if name == "physics_ready" else None),
        })
    total = sum(layer["points_awarded"] for layer in layers)
    strict_pass = all(layer["status"] == "PASS" for layer in layers)
    summary_reason = None
    if isinstance(summary, Mapping):
        method_summary = ((summary.get("methods") or {}).get(method) or {})
        row = next((item for item in method_summary.get("tasks", ())
                    if item.get("task_id") == task_id), None)
        if row:
            summary_reason = row.get("reason")
    first_blocker = next((check["gate"] or f"{check['name']}: {check['reason']}"
                          for check in checks if check["status"] != "PASS"), None)
    score = {
        "schema": SCHEMA,
        "scorer_version": "physcad-external-strict-scorer/1.0",
        "suite_id": "physcad-comfort-v1", "method": method, "task_id": task_id,
        "layer_weights": list(LAYER_WEIGHTS), "layers": layers,
        "total_points": total, "points_possible": 100,
        "strict_verdict": "PASS" if strict_pass else "FAIL",
        "first_blocker": first_blocker,
        "summary_reason": summary_reason or first_blocker,
        "provenance": provenance,
    }
    validate_external_score(score)
    return score, evidence_summary


def validate_external_score(value: Any, *, repo: str | Path | None = None) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or value.get("schema") != SCHEMA:
        raise ContractError("unsupported external strict score schema")
    allowed_top = {"schema", "scorer_version", "suite_id", "method", "task_id",
                   "layer_weights", "layers", "total_points", "points_possible",
                   "strict_verdict", "first_blocker", "summary_reason", "provenance"}
    if set(value) != allowed_top:
        raise ContractError("external strict score has unknown or missing top-level fields")
    if value.get("method") not in METHODS or value.get("task_id") not in {t.task_id for t in TASKS}:
        raise ContractError("external strict score method/task is invalid")
    if value.get("layer_weights") != list(LAYER_WEIGHTS) or value.get("points_possible") != 100:
        raise ContractError("external strict score maxima are invalid")
    layers = value.get("layers")
    if not isinstance(layers, list) or len(layers) != 5:
        raise ContractError("external strict score must have five layers")
    expected_checks = [name for _, name, _ in CHECKS]
    observed_checks = []
    total = 0
    for index, (layer, weight) in enumerate(zip(layers, LAYER_WEIGHTS), start=1):
        if not isinstance(layer, Mapping) or layer.get("index") != index or layer.get("weight") != weight:
            raise ContractError("external strict layer index/weight mismatch")
        checks = layer.get("checks")
        if not isinstance(checks, list) or not checks:
            raise ContractError("external strict layer lacks checks")
        layer_total = 0
        for check in checks:
            observed_checks.append(check.get("name"))
            if check.get("status") not in {"PASS", "FAIL", "UNKNOWN", "GATED"}:
                raise ContractError("external strict check status is invalid")
            if check.get("raw_status") not in {"PASS", "FAIL", "UNKNOWN"}:
                raise ContractError("external strict raw status is invalid")
            possible, awarded = check.get("points_possible"), check.get("points_awarded")
            if not isinstance(possible, int) or not isinstance(awarded, int) or not 0 <= awarded <= possible:
                raise ContractError("external strict check points are invalid")
            if check.get("status") != "PASS" and awarded != 0:
                raise ContractError("non-PASS external strict check awarded points")
            evidence = check.get("evidence")
            if not isinstance(evidence, list) or not evidence:
                raise ContractError("external strict PASS/FAIL/UNKNOWN check lacks evidence")
            for item in evidence:
                path = validate_relative_path(item.get("path"))
                if PureWindowsPath(path).is_absolute() or not isinstance(item.get("sha256"), str):
                    raise ContractError("external strict evidence path/hash is invalid")
                if repo is not None:
                    target = Path(repo) / path
                    if not target.is_file() or _sha256(target) != item["sha256"]:
                        raise ContractError(f"external strict evidence hash mismatch: {path}")
            layer_total += awarded
        if layer.get("points_awarded") != layer_total:
            raise ContractError("external strict layer total mismatch")
        total += layer_total
    if observed_checks != expected_checks:
        raise ContractError("external strict criterion set/order mismatch")
    if value.get("total_points") != total:
        raise ContractError("external strict total mismatch")
    strict_pass = all(layer.get("status") == "PASS" for layer in layers)
    if value.get("strict_verdict") != ("PASS" if strict_pass else "FAIL"):
        raise ContractError("external strict verdict mismatch")
    return value


def render_external_score(score: Mapping[str, Any]) -> str:
    score = validate_external_score(score)
    lines = [f"# External strict score: {score['method']} / {score['task_id']}", "",
             f"**Total:** {score['total_points']}/100", f"**Strict verdict:** {score['strict_verdict']}",
             f"**First blocker:** {score['first_blocker'] or 'none'}", "",
             "| Layer | Status | Points |", "|---|---|---:|"]
    for layer in score["layers"]:
        lines.append(f"| {layer['name']} | {layer['status']} | {layer['points_awarded']}/{layer['weight']} |")
    for layer in score["layers"]:
        lines.extend(["", f"## {layer['name']}", ""])
        if layer.get("strict_gate"):
            gate = layer["strict_gate"]
            lines.append(f"Strict gate: {'PASS' if gate['passed'] else 'FAIL'} — {gate['reason']}")
            lines.append("")
        for check in layer["checks"]:
            raw = f"; raw={check['raw_status']}" if check["status"] == "GATED" else ""
            lines.append(f"- **{check['status']}** `{check['name']}` — {check['points_awarded']}/{check['points_possible']}{raw}: {check['reason']}")
    return "\n".join(lines) + "\n"


def aggregate_external_scores(scores: Sequence[Mapping[str, Any]], method: str) -> dict[str, Any]:
    if method not in METHODS:
        raise ContractError("invalid aggregate method")
    by_task: dict[str, Mapping[str, Any]] = {}
    for value in scores:
        score = validate_external_score(value)
        if score["method"] != method:
            raise ContractError("external aggregate mixes methods")
        task_id = str(score["task_id"])
        if task_id in by_task:
            raise ContractError(f"duplicate external strict task: {task_id}")
        by_task[task_id] = score
    expected = [task.task_id for task in TASKS]
    missing = [task for task in expected if task not in by_task]
    if missing or len(by_task) != len(expected):
        raise ContractError("publishable external aggregate requires exactly ten tasks: " + ", ".join(missing))
    rows = [{"task_id": task, "score": by_task[task]["total_points"],
             "strict_verdict": by_task[task]["strict_verdict"],
             "first_blocker": by_task[task]["first_blocker"]} for task in expected]
    total = sum(row["score"] for row in rows)
    return {"schema": SUITE_SCHEMA, "suite_id": "physcad-comfort-v1", "method": method,
            "tasks_total": 10, "points_possible": 1000, "total_points": total,
            "mean_score": total / 10, "strict_passes": sum(row["strict_verdict"] == "PASS" for row in rows),
            "tasks": rows}


def render_external_suite(value: Mapping[str, Any]) -> str:
    lines = [f"# External strict suite: {value['method']}", "",
             f"**Total:** {value['total_points']}/1000", f"**Strict passes:** {value['strict_passes']}/10", "",
             "| Task | Score | Verdict | First blocker |", "|---|---:|---|---|"]
    for row in value["tasks"]:
        lines.append(f"| {row['task_id']} | {row['score']} | {row['strict_verdict']} | {row['first_blocker'] or 'none'} |")
    return "\n".join(lines) + "\n"


def render_report_fragment(suites: Mapping[str, Mapping[str, Any]],
                           scores: Mapping[str, Sequence[Mapping[str, Any]]]) -> str:
    lines = [
        "### Detailed strict criterion scores",
        "",
        "Cells show awarded points. Maxima are `E=10`; Assembly `model/roles/I-O=5/7/3`; "
        "Geometry `mesh/conflicts=5/10`; Physics `shape/alignment/finite=5/5/10`; "
        "Functional `input/propagation/output/invariants=5/10/15/10`. `G` means the "
        "criterion had raw diagnostic evidence but received zero points because a strict prerequisite gate was closed.",
        "",
    ]
    for method in METHODS:
        suite = suites[method]
        lines.extend([f"#### {method}", "",
                      f"Strict aggregate: **{suite['total_points']}/1000; {suite['strict_passes']}/10 PASS**.", "",
                      "| # | Task | E | A M/R/I | G M/C | P S/A/F | F I/P/O/V | F total | Total | Strict | Blocker |",
                      "|---:|---|---:|---:|---:|---:|---:|---:|---:|---|---|"])
        for number, score in enumerate(scores[method], start=1):
            checks = {check["name"]: check for layer in score["layers"] for check in layer["checks"]}
            def cell(name: str) -> str:
                check = checks[name]
                return "G" if check["status"] == "GATED" else str(check["points_awarded"])
            asm = "/".join(cell(name) for name in ("assembly_model", "required_role_cardinalities", "input_output_independence"))
            geo = "/".join(cell(name) for name in ("mesh_inventory", "non_exempt_conflicts"))
            phys = "/".join(cell(name) for name in ("trajectory_shape", "sample_alignment", "finite_health"))
            fun_names = ("input_motion", "motion_propagation", "registered_output", "registered_invariants")
            fun = "/".join(cell(name) for name in fun_names)
            fun_total = next(layer["points_awarded"] for layer in score["layers"] if layer["name"] == "functional")
            lines.append(f"| {number} | {score['task_id']} | {cell('scorer_owned_execution')} | {asm} | {geo} | {phys} | {fun} | {fun_total}/40 | {score['total_points']} | {score['strict_verdict']} | {score['summary_reason']} |")
        lines.append("")
    return "\n".join(lines)


def generate_external_suite(*, repo: str | Path, output: str | Path,
                            strict_root: str | Path, staged_root: str | Path,
                            work_root: str | Path, realization_root: str | Path,
                            summary_path: str | Path) -> dict[str, Path]:
    repo, output = Path(repo), Path(output)
    summary = _read(Path(summary_path))
    outputs: dict[str, Path] = {}
    all_scores: dict[str, list[dict[str, Any]]] = {}
    suites: dict[str, dict[str, Any]] = {}
    for method in METHODS:
        values = []
        for task in TASKS:
            score, evidence = score_external_task(
                repo=repo, method=method, task_id=task.task_id,
                strict_root=strict_root, staged_root=staged_root,
                work_root=work_root, realization_root=realization_root, summary=summary)
            task_root = output / method / task.task_id
            _write_json(task_root / "evidence.json", evidence)
            score_path = _write_json(task_root / "score.json", score)
            (task_root / "score.md").write_text(render_external_score(score), encoding="utf-8")
            values.append(score)
            outputs[f"{method}/{task.task_id}"] = score_path
        suite = aggregate_external_scores(values, method)
        _write_json(output / method / "suite_score.json", suite)
        (output / method / "suite_score.md").write_text(render_external_suite(suite), encoding="utf-8")
        all_scores[method] = values
        suites[method] = suite
    fragment = render_report_fragment(suites, all_scores)
    (output / "report_fragment.md").write_text(fragment, encoding="utf-8")
    manifest_files = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            manifest_files.append({"path": path.relative_to(output).as_posix(),
                                   "sha256": _sha256(path), "bytes": path.stat().st_size})
    _write_json(output / "manifest.json", {
        "schema": "physcad-external-strict-score-manifest/1.0",
        "suite_id": "physcad-comfort-v1", "methods": list(METHODS),
        "withdrawn_scores_used": False, "files": manifest_files})
    outputs["report_fragment"] = output / "report_fragment.md"
    return outputs


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--strict-root", required=True)
    parser.add_argument("--staged-root", required=True)
    parser.add_argument("--work-root", required=True)
    parser.add_argument("--realization-root", required=True)
    parser.add_argument("--summary", required=True)
    args = parser.parse_args(argv)
    generate_external_suite(repo=args.repo, output=args.output,
                            strict_root=args.strict_root, staged_root=args.staged_root,
                            work_root=args.work_root, realization_root=args.realization_root,
                            summary_path=args.summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["SCHEMA", "SUITE_SCHEMA", "aggregate_external_scores", "generate_external_suite",
           "render_external_score", "render_report_fragment", "score_external_task",
           "validate_external_score"]
