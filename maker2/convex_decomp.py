"""Convex decomposition of part STLs for MuJoCo collision (maker2-mujoco-contact).

MuJoCo convexifies every mesh geom for collision, so a concave part (gear teeth,
a bore, a bracket) only collides as its convex hull unless it is first broken into
convex PIECES. This module turns one part STL into N convex-piece STLs, cached by
content hash so a re-run is free.

Fallback chain (coacd IS installed here, on the python3 interpreter the pipeline runs
under; the fallbacks only matter on an interpreter that lacks it, and taking one is
LOGGED so a slow run is never a mystery):
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
import sys
from pathlib import Path

import numpy as np
import trimesh

import re

# A movable part whose name/shape matches this is a GEAR and keeps the fine (interlocking-
# teeth) decomposition; every other mover gets a single-hull collision proxy.
_GEAR_NAME_RE = re.compile(r"gear|pinion|wheel|cog|sprocket", re.I)


# Bump when the decomposition logic changes so stale cached pieces are ignored.
_CACHE_VERSION = "v2"

# CoACD concavity threshold, in TWO tiers, because the cost of a tight threshold is
# superlinear and only gear teeth actually need it.
#
# Measured on this watch (hour_wheel, 2136 faces):
#     threshold 0.05 -> 48 pieces, 180.5s
#     threshold 0.10 ->  4 pieces,  37.5s
#     threshold 0.15 ->  1 piece,    0.9s   <- collapsed to the hull: bore FILLED, the
#                                              exact failure _is_hollow exists to prevent
# So 0.10 is the knee, and 0.15 is over the cliff.
#
# GEARS keep 0.05: interlocking teeth ARE the transmission test, and a tooth flank that
# gets merged into a neighbouring hull stops meshing. Everything else takes 0.10, which
# on the bored parts was verified to KEEP THE BORE — piece volume tracked the part's real
# volume, not its hull (hour_pipe 45.6 vs part 42.1, hull 74.5) — while cutting hour_pipe
# from 84.7s to 23.9s. Fewer pieces also makes every sim step cheaper, since MuJoCo's
# contact cost scales with geom count.
_COACD_THRESHOLD = 0.05
_COACD_THRESHOLD_COARSE = 0.10
# Voxel resolution for coacd's preprocessing pass. The default is far finer than these
# millimetre-scale parts need and was pure overhead (144.3s vs 180.5s at the same result).
_COACD_PREPROCESS_RES = 20

# One-shot guard so the "no coacd" notice is logged once per process, not per part.
_COACD_MISSING_LOGGED = False

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


def _coacd_pieces(mesh: "trimesh.Trimesh", *, fine: bool = True,
                  log_fn=print) -> list["trimesh.Trimesh"] | None:
    """CoACD decomposition, or None if coacd is unavailable/errors. On a RUNTIME error
    (coacd present but the run failed) log it — a silent None here masqueraded as 'no
    coacd' and let a hollow part degrade to a solid hull without any trace."""
    try:
        import coacd
    except ImportError:
        # Say so ONCE. A silent None here is indistinguishable from "coacd ran and found
        # nothing", so a run on an interpreter without coacd quietly took the much slower
        # VHACD path and nothing in the log said why.
        global _COACD_MISSING_LOGGED
        if not _COACD_MISSING_LOGGED:
            _COACD_MISSING_LOGGED = True
            log_fn(f"[convex] coacd not importable on {sys.executable} — "
                   f"falling back to VHACD/hull (slower, coarser)")
        return None
    try:
        try:
            coacd.set_log_level("error")
        except Exception:
            pass
        cmesh = coacd.Mesh(mesh.vertices, mesh.faces)
        parts = coacd.run_coacd(
            cmesh,
            threshold=_COACD_THRESHOLD if fine else _COACD_THRESHOLD_COARSE,
            preprocess_resolution=_COACD_PREPROCESS_RES)
        out = []
        for verts, faces in parts:
            out.append(trimesh.Trimesh(vertices=np.asarray(verts),
                                       faces=np.asarray(faces), process=False))
        return out or None
    except Exception as e:
        log_fn(f"[convex] coacd run FAILED ({type(e).__name__}: {e}); falling back")
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
                   *, fine: bool = True, metrics: dict | None = None,
                   log_fn=print) -> list[str]:
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
    backend += "" if fine else "_coarse"     # a tier change must invalidate the cache
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

    pieces = _coacd_pieces(mesh, fine=fine, log_fn=log_fn)
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


def _is_hollow(stl_path: str, *, ratio: float = 1.6) -> bool:
    """True if a part has a cavity/bore/opening its CONVEX HULL would FILL — so the hull
    is a solid plug that jams whatever is supposed to pass through it (the housing_ring
    failure: 1000+ phantom contacts, the sim can't run).

    TWO independent tests, because either alone misses real cases:

    * TOPOLOGY (euler_number != 2). A closed solid with no through-hole has euler 2; a
      part with one bore has 0, with two holes -2, and so on. This is exact, and it is
      what catches the cases the volume ratio cannot see: a THIN plate with pivot holes
      (skeleton_bridge measured vol 1406 vs hull 1460 -> ratio 1.04) and a THIN-WALLED
      tube (hour_pipe: 150 vs 190 -> ratio 1.27). Both have real through-holes that
      arbors pass through, both sailed under a 1.6 threshold, and hulling them welded
      the whole movement solid — arbors buried 6mm inside the "plate", the driven pinion
      swept 0.14 of 12 commanded rad.
    * VOLUME RATIO. Keeps catching open frames / C-shapes whose opening is not a
      topological hole at all, so euler stays 2 while the hull still fills the mouth.

    Cheap and deterministic (one hull + two volumes + a face/edge count), so it gates
    the expensive decomposition to only the parts that actually need it."""
    try:
        m = trimesh.load(stl_path, force="mesh")
        if not isinstance(m, trimesh.Trimesh) or len(m.faces) == 0:
            return False
        # Topology first: exact, and independent of how thin the walls are.
        try:
            if m.is_watertight and int(m.euler_number) != 2:
                return True
        except Exception:
            pass          # a non-manifold mesh has no meaningful euler; fall through
        vol = float(m.volume)
        hull_vol = float(m.convex_hull.volume)
        if vol <= 1e-9 or hull_vol <= 1e-9:
            return False
        return (hull_vol / vol) >= ratio
    except Exception:
        return False


def decompose_model(model, meshes_dir: str, *, metrics: dict | None = None,
                    log_fn=print) -> dict[str, list[str]]:
    """Decompose every movable/meshing part of a model, PLUS any fixed part that is
    hollow (a housing/ring/frame whose convex hull would fill its cavity). Returns
    {link_name: [piece paths]}; a link absent from the map (or mapped to []) means 'use
    the part's own mesh as a single hull geom'. Solid fixed structure collides fine as
    its own hull, and decomposition is the expensive step — so it is gated.

    A/B collision geometry: MuJoCo's per-step contact cost scales with total geom
    count, and a fine decomposition emits ~40-64 hulls PER part. Only GEARS need that
    fidelity (interlocking teeth are the transmission test). Every other movable part
    — a shaft, sleeve, hand, collar — is a simple convex-ish solid that only needs to
    occupy space without interpenetrating, so it collides fine as a SINGLE convex hull.
    So: gears (name gear/pinion/wheel or a mesh_pair member) get the full decomposition;
    all other movers get one hull. The VISUAL mesh is untouched — this is the low-poly
    'B' collision proxy, the high-poly 'A' mesh still renders. Module/teeth/geometry of
    every gear are unchanged; only NON-gear collision detail drops.

    HOLLOW FIXED PARTS are the exception to 'fixed structure collides fine as its own
    hull': a ring/housing/open frame has a cavity, and its convex hull is a SOLID PLUG
    that fills that cavity and interpenetrates everything mounted inside it. Those get
    the full decomposition (which preserves the hole) even though they don't move."""
    movers = {l.name for l in model.links if getattr(l, "dof", "fixed") != "fixed"}
    mesh_members: set = set()
    for pair in getattr(model, "mesh_pairs", []) or []:
        movers.update(pair)
        mesh_members.update(pair)

    # PASS 1 — classify every link (cheap: a hull + two volumes + an euler count each).
    # Kept separate from the work so the expensive pass below is a flat list of
    # independent jobs, which is what makes it parallelisable.
    jobs: list[tuple[str, str, str]] = []          # (link_name, stl, kind)
    for link in model.links:
        stl = os.path.join(meshes_dir, f"{link.name}.stl")
        is_mover = link.name in movers
        # A fixed part is skipped UNLESS it is hollow (its hull would plug its own cavity).
        if not is_mover:
            if _is_hollow(stl):
                jobs.append((link.name, stl, "hollow_fixed"))
            continue
        is_gear = (link.name in mesh_members
                   or _GEAR_NAME_RE.search(link.name or "")
                   or _GEAR_NAME_RE.search(getattr(link, "shape_hint", "") or ""))
        if is_gear:
            jobs.append((link.name, stl, "gear"))
        # B-proxy: one convex hull for a non-gear mover (shaft/sleeve/hand/collar) —
        # UNLESS it is hollow. A pipe, sleeve, bored hand or collar exists precisely
        # so a shaft can pass THROUGH it, and a single hull fills that bore: the
        # shaft is then embedded in a solid rod and the train welds itself shut.
        # (hour_pipe shipped as vol=hullvol=189.92, its bore gone, minute_arbor
        # buried 3.3mm inside it.) Those get the real decomposition, which keeps
        # the hole; only genuinely solid movers take the cheap hull.
        elif _is_hollow(stl):
            jobs.append((link.name, stl, "hollow_mover"))
        else:
            jobs.append((link.name, stl, "hull"))

    _LABEL = {
        "hollow_fixed": "hollow fixed part {} (its hull would fill its cavity)",
        "gear": "gear {} (fine)",
        "hollow_mover": "hollow mover {} (a single hull would fill its bore)",
    }

    def _work(job):
        name, stl, kind = job
        if kind == "hull":
            return name, _hull_only_piece(stl, meshes_dir, name, log_fn=lambda *_: None)
        local: dict = {}
        # Only a GEAR needs the fine threshold — its teeth have to interlock. A bored
        # bearing/washer/pipe just has to keep its hole, which the coarse tier does.
        pieces = decompose_part(stl, meshes_dir, name, fine=(kind == "gear"),
                                metrics=local, log_fn=lambda *_: None)
        return name, pieces, local

    # PASS 2 — do the work. Parts are INDEPENDENT (each reads one STL and writes its own
    # `<part>_cvx_*.stl` + manifest, cached by content hash), so decomposing them one at a
    # time just serialises a pile of CPU-bound jobs. A watch movement is 20+ parts and the
    # gated-in hollow ones each take a real decomposition, which is what made a cold run
    # slow. Threads are enough: coacd/VHACD do their work in native code and release the
    # GIL, and the per-part I/O is a file write.
    out: dict[str, list[str]] = {}
    heavy = [j for j in jobs if j[2] != "hull"]
    for name, stl, kind in heavy:
        log_fn(f"[convex] decomposing {_LABEL[kind].format(name)} ...")
    if len(jobs) > 1:
        from concurrent.futures import ThreadPoolExecutor
        workers = min(len(jobs), (os.cpu_count() or 4))
        with ThreadPoolExecutor(max_workers=workers) as ex:
            results = list(ex.map(_work, jobs))
    else:
        results = [_work(j) for j in jobs]

    for res in results:
        name, pieces = res[0], res[1]
        # Merge each part's metrics centrally: `metrics` is shared run state and the
        # workers must not race on it. contact_degraded is sticky — one degraded part
        # flags the whole run — which is exactly what an OR-merge preserves.
        if metrics is not None and len(res) > 2 and res[2].get("contact_degraded"):
            metrics["contact_degraded"] = True
        if pieces:
            out[name] = pieces
    return out


def _hull_only_piece(stl_path: str, meshes_dir: str, part_name: str,
                     *, log_fn=print) -> list[str]:
    """Write a SINGLE convex-hull STL for a non-gear movable part (its low-poly collision
    proxy), cached. Returns [path] or [] (caller then uses the part's own mesh hull)."""
    if not os.path.exists(stl_path) or os.path.getsize(stl_path) == 0:
        return []
    tag = _part_hash(stl_path, "hull1")
    cached = _existing_cache(meshes_dir, part_name, tag)
    if cached is not None:
        return cached
    try:
        mesh = trimesh.load(stl_path, force="mesh")
        hull = mesh.convex_hull
    except Exception as e:
        log_fn(f"[convex] hull proxy failed for {part_name} ({e}); caller uses raw hull")
        return []
    if not isinstance(hull, trimesh.Trimesh) or len(hull.faces) == 0:
        return []
    os.makedirs(meshes_dir, exist_ok=True)
    paths = _cache_paths(meshes_dir, part_name, 1)
    hull.export(paths[0])
    _write_manifest(meshes_dir, part_name, tag, 1)
    log_fn(f"[convex] {part_name}: 1 piece (hull proxy, non-gear)")
    return paths

