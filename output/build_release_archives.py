from __future__ import annotations

import hashlib
import json
from pathlib import Path
import zipfile

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "output" / "release_v0.5.0_mjcf_agent_benchmark"

BUNDLES = (
    (
        "physcad-comfort-v1-automech-rescored-portable.zip",
        REPO / "output" / "comfort_benchmark_v1_rescored_portable",
        "automech-rescored-portable",
        "Ten submitted AutoMech portable artifacts plus scorer-owned evidence. Raw and derived files retain their original names.",
    ),
    (
        "physcad-comfort-v1-external-strict-audit.zip",
        REPO / "output" / "external_benchmark_strict",
        "external-strict-audit",
        "Twenty Claude Code/Codex portable submissions with strict geometry, model, anchor, and replay audits. Withdrawn provisional aggregate scores are not included.",
    ),
    (
        "physcad-comfort-v1-representation-ablation-task05.zip",
        REPO / "output" / "representation_ablation_05_three_planet_4to1",
        "representation-ablation-task05",
        "One fresh independent-values task-05 pilot, including raw generation artifacts and scorer-owned geometry/dependency evidence. This pilot does not alter the ten-task aggregate.",
    ),
)

SKIP_PARTS = {"__pycache__", ".pytest_cache", ".mypy_cache"}
SKIP_SUFFIXES = {".pyc", ".pyo"}


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def include(path: Path) -> bool:
    if any(part in SKIP_PARTS for part in path.parts):
        return False
    if path.suffix.lower() in SKIP_SUFFIXES:
        return False
    if path.name.casefold() == ".env" or path.suffix.casefold() in {".key", ".pem"}:
        return False
    return True


def archive(name: str, source: Path, bundle_id: str, description: str) -> Path:
    files = [path for path in sorted(source.rglob("*")) if path.is_file() and include(path)]
    if not files:
        raise RuntimeError(f"empty bundle source: {source}")
    manifest = {
        "schema": "physcad-release-evidence-bundle/1.0",
        "release_tag": "v0.5.0-mjcf-agent-benchmark",
        "bundle_id": bundle_id,
        "description": description,
        "source_commit_before_release_merge": "d409d782129de4ab842675f59246d49db034b69f",
        "path_policy": "Raw submitted files are preserved. Some raw records contain original machine-local paths; derived manifests and the committed report use relative paths.",
        "files": [
            {
                "path": path.relative_to(source).as_posix(),
                "sha256": digest(path),
                "bytes": path.stat().st_size,
            }
            for path in files
        ],
    }
    destination = OUT / name
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=6, allowZip64=True) as target:
        target.writestr("RELEASE_EVIDENCE_MANIFEST.json",
                        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n")
        for path in files:
            target.write(path, path.relative_to(source).as_posix())
    return destination


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    outputs = [archive(*bundle) for bundle in BUNDLES]
    sums = "".join(f"{digest(path)}  {path.name}\n" for path in outputs)
    (OUT / "SHA256SUMS.txt").write_text(sums, encoding="utf-8")
    for path in outputs:
        print(f"{path.name}: {path.stat().st_size / 1024 / 1024:.2f} MiB")
    print(sums, end="")


if __name__ == "__main__":
    main()
