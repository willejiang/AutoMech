from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import zipfile

ROOT = Path(__file__).resolve().parent / "release_v0.5.0_mjcf_agent_benchmark"
FORBIDDEN_NAMES = {".env", "id_rsa", "id_ed25519"}
FORBIDDEN_PARTS = {"__pycache__", ".git", ".claude", ".venv", ".venv312", "node_modules"}
SECRET_PATTERN = re.compile(rb"(?:api[_-]?key|authorization|bearer)\s*[:=]\s*['\"]?[A-Za-z0-9_\-]{20,}", re.I)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    expected = {}
    for line in (ROOT / "SHA256SUMS.txt").read_text(encoding="utf-8").splitlines():
        digest, name = line.split("  ", 1)
        expected[name] = digest
    for path in sorted(ROOT.glob("*.zip")):
        if sha256(path) != expected[path.name]:
            raise RuntimeError(f"checksum mismatch: {path.name}")
        with zipfile.ZipFile(path) as source:
            names = source.namelist()
            if "RELEASE_EVIDENCE_MANIFEST.json" not in names:
                raise RuntimeError(f"missing manifest: {path.name}")
            manifest = json.loads(source.read("RELEASE_EVIDENCE_MANIFEST.json"))
            if len(manifest["files"]) + 1 != len(names):
                raise RuntimeError(f"manifest count mismatch: {path.name}")
            for name in names:
                pure = PurePosixPath(name)
                if pure.name.casefold() in FORBIDDEN_NAMES:
                    raise RuntimeError(f"forbidden file: {path.name}:{name}")
                if FORBIDDEN_PARTS.intersection(pure.parts):
                    raise RuntimeError(f"forbidden path: {path.name}:{name}")
                if pure.suffix.casefold() in {".key", ".pem", ".pyc", ".pyo"}:
                    raise RuntimeError(f"forbidden suffix: {path.name}:{name}")
                if name == "RELEASE_EVIDENCE_MANIFEST.json":
                    continue
                raw = source.read(name)
                if SECRET_PATTERN.search(raw):
                    raise RuntimeError(f"possible secret in {path.name}:{name}")
            print(f"validated {path.name}: {len(names) - 1} evidence files")


if __name__ == "__main__":
    main()
