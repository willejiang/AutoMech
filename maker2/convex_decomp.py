"""Convex decomposition of part STLs for MuJoCo collision (maker2-mujoco-contact).

MuJoCo convexifies every mesh geom for collision, so a concave part (gear teeth,
a bore, a bracket) only collides as its convex hull unless it is first broken into
convex PIECES. This module turns one part STL into N convex-piece STLs, cached by
content hash so a re-run is free.

Fallback chain (coacd is NOT installed on this machine, so the fallback IS the
default path — see the plan's resolved decision):
  1. coacd            — best quality, if importable.
  2. trimesh V-HACD   — trimesh.decomposition.convex_decomposition (needs a VHACD
                        binary; on many boxes this just returns the hull).
  3. convex hull      — a single hull geom for the part, and set the caller's
                        metrics["contact_degraded"]=True (collision fidelity reduced;
                        teeth won't interlock, so transmission may not be faithful).

Only ever imported when settings.engine == "mujoco"; the PyBullet default never
touches it.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path

import numpy as np
import trimesh


# Bump when the decomposition logic changes so stale cached pieces are ignored.
_CACHE_VERSION = "v2"

# CoACD defaults tuned for mechanical parts: a lower threshold keeps tooth concavity.
_COACD_THRESHOLD = 0.05

# Convex pieces below this volume (STL units = mm^3) are dropped: a near-zero sliver from
# VHACD/CoACD (e.g. a tooth-tip splinter on a small pinion) has no usable inertia and makes
# MuJoCo reject the whole model ("mesh volume is too small"). Losing such a splinter is
# harmless — the surviving pieces still cover the part's real volume.
_MIN_PIECE_VOLUME_MM3 = 1e-3


def _drop_slivers(pieces: list) -> list:
    """Drop convex pieces whose volume is below the MuJoCo floor, keeping order. Never
    empties the list: if every piece is sub-threshold (a genuinely tiny part), keep the
    single largest so the part still has geometry."""
    if not pieces:
        return pieces
    def _vol(p):
        try:
            return abs(float(p.volume))
        except Exception:
            return 0.0
    kept = [p for p in pieces if _vol(p) >= _MIN_PIECE_VOLUME_MM3]
    if kept:
        return kept
    return [max(pieces, key=_vol)]


def _part_hash(stl_path: str, backend: str) -> str:
    """Content hash of the STL + backend + version, so a changed mesh or a different
    decomposer invalidates the cache."""
    h = hashlib.sha256()
    h.update(_CACHE_VERSION.encode())
    h.update(backend.encode())
    with open(stl_path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


def _cache_paths(meshes_dir: str, part_name: str, n: int) -> list[str]:
    return [os.path.join(meshes_dir, f"{part_name}_cvx_{k}.stl") for k in range(n)]


def _existing_cache(meshes_dir: str, part_name: str, tag: str) -> list[str] | None:
    """Return the cached piece paths for this part+tag if a manifest matches, else
    None. The manifest (`<part>_cvx.manifest`) records the hash tag + piece count so
    we know the cache is current without re-decomposing."""
    manifest = os.path.join(meshes_dir, f"{part_name}_cvx.manifest")
    if not os.path.exists(manifest):
        return None
    try:
        cached_tag, count = Path(manifest).read_text(encoding="utf-8").strip().split()
        count = int(count)
    except (ValueError, OSError):
        return None
    if cached_tag != tag:
        return None
    paths = _cache_paths(meshes_dir, part_name, count)
    if all(os.path.exists(p) and os.path.getsize(p) > 0 for p in paths):
        return paths
    return None


def _write_manifest(meshes_dir: str, part_name: str, tag: str, count: int) -> None:
    manifest = os.path.join(meshes_dir, f"{part_name}_cvx.manifest")
    Path(manifest).write_text(f"{tag} {count}", encoding="utf-8")


def _coacd_pieces(mesh: "trimesh.Trimesh") -> list["trimesh.Trimesh"] | None:
    """CoACD decomposition, or None if coacd is unavailable/errors."""
    try:
        import coacd
    except ImportError:
        return None
    try:
        try:
            coacd.set_log_level("error")
        except Exception:
            pass
        cmesh = coacd.Mesh(mesh.vertices, mesh.faces)
        parts = coacd.run_coacd(cmesh, threshold=_COACD_THRESHOLD)
        out = []
        for verts, faces in parts:
            out.append(trimesh.Trimesh(vertices=np.asarray(verts),
                                       faces=np.asarray(faces), process=False))
        return out or None
    except Exception:
        return None


def _vhacd_pieces(mesh: "trimesh.Trimesh") -> list["trimesh.Trimesh"] | None:
    """trimesh V-HACD decomposition, or None if it errors or yields nothing usable.
    Returns >1 piece only when a real VHACD backend is present; a hull-only result
    (len==1) is treated as 'no real decomposition' so the caller flags degraded."""
    try:
        parts = mesh.convex_decomposition()
    except Exception:
        return None
    if isinstance(parts, trimesh.Trimesh):
        parts = [parts]
    if not isinstance(parts, list) or len(parts) <= 1:
        return None
    return [p for p in parts if isinstance(p, trimesh.Trimesh) and len(p.faces) > 0] or None


def decompose_part(stl_path: str, meshes_dir: str, part_name: str,
                   *, metrics: dict | None = None, log_fn=print) -> list[str]:
    """Decompose one part STL into convex-piece STLs (cached). Returns the list of
    piece STL paths (absolute). On the hull fallback, sets metrics["contact_degraded"]
    = True. If the STL is missing/unloadable, returns [] and the caller should treat
    the part as a single hull geom on its own mesh.

    `metrics` is the sim metrics dict the score reads; we only ever SET
    contact_degraded (never clear it), so one degraded part flags the whole run."""
    if not os.path.exists(stl_path) or os.path.getsize(stl_path) == 0:
        return []

    # Pick the tag off the best backend we could use, so switching backends re-caches.
    have_coacd = False
    try:
        import coacd  # noqa: F401
        have_coacd = True
    except ImportError:
        have_coacd = False
    backend = "coacd" if have_coacd else "vhacd_or_hull"
    tag = _part_hash(stl_path, backend)

    cached = _existing_cache(meshes_dir, part_name, tag)
    if cached is not None:
        return cached

    try:
        mesh = trimesh.load(stl_path, force="mesh")
    except Exception as e:
        log_fn(f"[convex] load failed for {part_name} ({e}); caller uses raw hull")
        return []
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        return []

    pieces = _coacd_pieces(mesh)
    method = "coacd"
    if pieces is None:
        pieces = _vhacd_pieces(mesh)
        method = "vhacd"
    if pieces is None:
        # Hull fallback — one convex piece = the part's convex hull. Contact fidelity
        # is reduced (no interlocking teeth), so flag the run.
        pieces = [mesh.convex_hull]
        method = "hull"
        if metrics is not None:
            metrics["contact_degraded"] = True
    else:
        # Drop sub-floor slivers a real decomposition can emit; MuJoCo rejects them.
        pieces = _drop_slivers(pieces)

    paths = _cache_paths(meshes_dir, part_name, len(pieces))
    os.makedirs(meshes_dir, exist_ok=True)
    for piece, p in zip(pieces, paths):
        piece.export(p)
    _write_manifest(meshes_dir, part_name, tag, len(pieces))
    log_fn(f"[convex] {part_name}: {len(pieces)} piece(s) via {method}")
    return paths


def decompose_model(model, meshes_dir: str, *, metrics: dict | None = None,
                    log_fn=print) -> dict[str, list[str]]:
    """Decompose every movable/meshing part of a model. Returns {link_name: [piece
    paths]}; a link absent from the map (or mapped to []) means 'use the part's own
    mesh as a single hull geom'. Only parts that MOVE (dof != fixed) or appear in a
    mesh_pair are decomposed — fixed structure collides fine as its own hull, and
    decomposition is the expensive step."""
    movers = {l.name for l in model.links if getattr(l, "dof", "fixed") != "fixed"}
    for pair in getattr(model, "mesh_pairs", []) or []:
        movers.update(pair)
    out: dict[str, list[str]] = {}
    for link in model.links:
        if link.name not in movers:
            continue
        stl = os.path.join(meshes_dir, f"{link.name}.stl")
        log_fn(f"[convex] decomposing {link.name} ...")
        pieces = decompose_part(stl, meshes_dir, link.name,
                                metrics=metrics, log_fn=log_fn)
        if pieces:
            out[link.name] = pieces
    return out
