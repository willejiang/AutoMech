"""Backend-neutral physics result and artifact identity contract."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

CONTRACT_VERSION = 1
VALID_ENGINES = {"pybullet", "mujoco", "simscape", "chrono"}
VALID_MODES = {"kinematic", "ideal_dynamic", "contact_dynamic"}


def file_sha256(path) -> str:
    p = Path(path)
    if not p.exists() or not p.is_file():
        return ""
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def artifact_identity(run_dir: str, model=None) -> dict:
    root = Path(run_dir)
    model_path = root / "kinematic_model.json"
    geometry = {}
    if model is not None:
        for link in model.links:
            mesh = root / (link.mesh_filename or f"meshes/{link.name}.stl")
            geometry[link.name] = file_sha256(mesh)
    return {"authored_ir_sha256": file_sha256(model_path), "geometry_sha256": geometry}


def normalize_result(result: dict | None, *, engine: str, mode: str,
                     run_dir: str = "", model=None, status: str | None = None,
                     manifest_path: str | None = None) -> dict:
    """Add the canonical envelope without breaking legacy UI/readers."""
    out = dict(result or {})
    if mode not in VALID_MODES:
        mode = "ideal_dynamic"
    inferred = status or ("completed" if out else "runtime_failed")
    out.setdefault("passed", None if inferred == "unavailable" else False)
    if out["passed"] is True:
        out.setdefault("verdict", "PASS")
    elif out["passed"] is None:
        out.setdefault("verdict", "UNAVAILABLE")
    else:
        out.setdefault("verdict", "FAIL")
    out.setdefault("summary", "")
    out.setdefault("metrics", {})
    out.setdefault("tests", [])
    out.setdefault("cause", "none")
    out.setdefault("reason", "")
    out["contract_version"] = CONTRACT_VERSION
    out["engine"] = engine
    out["mode"] = mode
    out["status"] = inferred
    artifact = dict(out.get("artifact") or {})
    artifact.update({k: v for k, v in artifact_identity(run_dir, model).items() if v})
    if manifest_path:
        artifact["builder_manifest"] = str(manifest_path)
    out["artifact"] = artifact
    out.setdefault("phases", {})
    health = (out["metrics"].setdefault("numerical_health", {}))
    health.setdefault("finite", True)
    out.setdefault("diagnosis", None)
    return out


def write_json(path, data: dict) -> str:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, indent=2), encoding="utf-8")
    return str(p)
