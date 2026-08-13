"""Executable golden for portable-contract integrity and resource checks."""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

from benchmark_scorer import CONTRACT_ID, ContractError, ResourceLimits, TriState
from benchmark_scorer.ingest import ingest_submission


def _hash(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _submission(root: Path, *, trajectory: bytes = b'{"t":[0,1],"joints":{}}',
                declared_hash: str | None = None, declared_path: str = "evidence/trajectory.json"):
    target = root / "evidence" / "trajectory.json"
    target.parent.mkdir(parents=True)
    target.write_bytes(trajectory)
    assembly = b'{"bodies":[],"joints":[]}'
    (root / "assembly.json").write_bytes(assembly)
    files = [
        {"path": "assembly.json", "sha256": _hash(assembly), "size": len(assembly),
         "media_type": "application/json", "role": "assembly"},
        {"path": declared_path, "sha256": declared_hash or _hash(trajectory),
         "size": len(trajectory), "media_type": "application/json", "role": "trajectory"},
    ]
    manifest = {
        "contract": CONTRACT_ID, "suite_id": "fixture", "task_id": "fixture-1",
        "prompt_sha256": "0" * 64,
        "producer": {"harness": "fixture", "harness_version": "1", "run_id": "r1"},
        "evidence_lane": {"submitted": True, "physics_mode": "finite_effort",
                          "engine": "fixture", "engine_version": "1"},
        "files": files,
        "units": {"length": "m", "angle": "rad", "time": "s"},
        "telemetry_provenance": {"simulator": "fixture", "sampling_source": "run",
                                 "from": "harness_run"},
        "idealizations": [], "raw_audit": [],
    }
    (root / "benchmark_submission.json").write_text(json.dumps(manifest), encoding="utf-8")


def _must_reject(mutator, text: str) -> None:
    with tempfile.TemporaryDirectory(prefix="scorer_contract_") as temp:
        root = Path(temp)
        mutator(root)
        try:
            ingest_submission(root)
        except ContractError:
            return
        raise AssertionError(f"unsafe submission accepted: {text}")


def main() -> int:
    assert TriState.from_value(True) is TriState.TRUE
    assert TriState.from_value(False) is TriState.FALSE
    assert TriState.from_value(None) is TriState.UNKNOWN

    with tempfile.TemporaryDirectory(prefix="scorer_contract_ok_") as temp:
        root = Path(temp)
        _submission(root)
        before = {p.relative_to(root).as_posix(): p.read_bytes()
                  for p in root.rglob("*") if p.is_file()}
        ingested = ingest_submission(root)
        after = {p.relative_to(root).as_posix(): p.read_bytes()
                 for p in root.rglob("*") if p.is_file()}
        assert before == after, "ingestion mutated its input"
        assert ingested.manifest.contract == CONTRACT_ID
        assert "evidence/trajectory.json" in ingested.documents

    def invalid_units(root: Path):
        _submission(root)
        path = root / "benchmark_submission.json"
        doc = json.loads(path.read_text(encoding="utf-8"))
        doc["units"]["angle"] = "turns"
        path.write_text(json.dumps(doc), encoding="utf-8")

    _must_reject(invalid_units, "unsupported units")
    _must_reject(lambda root: _submission(root, declared_path="../escape.json"), "traversal")
    _must_reject(lambda root: _submission(root, declared_path="C:/escape.json"), "absolute path")
    _must_reject(lambda root: _submission(root, declared_hash="f" * 64), "hash mismatch")
    _must_reject(lambda root: _submission(root, declared_path="evidence/CON.json"),
                 "Windows reserved name")
    _must_reject(lambda root: _submission(root, trajectory=b'{"x":NaN}'), "non-finite JSON")
    _must_reject(lambda root: _submission(root, trajectory=b'{"x":1,"x":2}'),
                 "duplicate JSON key")

    def oversized(root: Path):
        _submission(root, trajectory=b"{}")
    with tempfile.TemporaryDirectory(prefix="scorer_contract_limit_") as temp:
        root = Path(temp)
        oversized(root)
        try:
            ingest_submission(root, limits=ResourceLimits(max_file_bytes=1))
        except ContractError:
            pass
        else:
            raise AssertionError("file resource limit was not enforced")

    if hasattr(os, "symlink"):
        with tempfile.TemporaryDirectory(prefix="scorer_contract_link_") as temp:
            root = Path(temp)
            _submission(root)
            actual = root / "evidence" / "actual.json"
            target = root / "evidence" / "trajectory.json"
            target.rename(actual)
            try:
                os.symlink(actual, target)
            except OSError:
                pass  # Windows developer mode/privilege may not permit symlink creation.
            else:
                try:
                    ingest_submission(root)
                except ContractError:
                    pass
                else:
                    raise AssertionError("symlink submission artifact was accepted")

    print("golden contract safety: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
