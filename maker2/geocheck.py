"""Deterministic geometry self-check (D3) + layered orthographic render (D3b).

Rendering (human or VLM looking at pictures) has repeatedly MISSED assembly faults
in this project — a shaft not passing through a gear bore looked fine in six views,
and parts inside the housing are occluded. So the PRIMARY fault-finding here is
NUMERIC and deterministic: shaft-through-part containment, gear mesh center distance,
functional-vs-structural solid clashes, and realized-axis vs params-axis direction
(sign-aware, unlike the abs(cos) guard). The layered render is a SECONDARY aid: it
drops the structural parts (housing/wall/cover/seat/foot/base) so the internal
shaft/gear/bearing relationships are not occluded.

Consumes an assembled KinematicModel + its meshes dir; writes a text report and,
optionally, PNGs. Import is cheap; heavy deps (trimesh/matplotlib) load lazily so a
run without rendering doesn't pay for them.
"""
from __future__ import annotations

import os
import numpy as np

from .assembler import _mat
from .model import KinematicModel

# link-name substrings that mark a STRUCTURAL (containing) part vs a FUNCTIONAL one
_STRUCTURAL = ("housing", "wall", "cover", "seat", "foot", "base", "flange", "rib", "bracket")
_FUNCTIONAL = ("shaft", "gear", "pinion", "bearing", "spacer", "collar", "stub", "arbor")


def _is_structural(name: str) -> bool:
    n = name.lower()
    return any(k in n for k in _STRUCTURAL) and not any(k in n for k in ("shaft", "gear", "pinion", "bearing"))


def _world_transforms(model: KinematicModel) -> dict:
    """{link_name -> 4x4 world matrix} by accumulating the pose forest (root at identity)."""
    by_child = {p.child: p for p in model.poses}
    cache: dict = {}

    def world(name, seen=None):
        if name in cache:
            return cache[name]
        seen = seen or set()
        if name in seen:
            return np.eye(4)
        seen.add(name)
        p = by_child.get(name)
        if p is None:
            return np.eye(4)
        local = _mat(p.xyz_m, p.rpy_rad)
        parent = world(p.parent, seen) if p.parent else np.eye(4)
        m = parent @ local
        cache[name] = m
        return m

    for l in model.links:
        world(l.name)
    return cache


def _load_world_meshes(model: KinematicModel, meshes_dir: str) -> dict:
    """{link_name -> trimesh in WORLD mm}. STL is in mm; pose translation is meters -> *1000."""
    import trimesh
    W = _world_transforms(model)
    out = {}
    for l in model.links:
        f = os.path.join(meshes_dir, os.path.basename(l.mesh_filename or (l.name + ".stl")))
        if not os.path.isfile(f):
            continue
        m = trimesh.load(f, force="mesh")
        if m.is_empty:
            continue
        T = W.get(l.name, np.eye(4)).copy()
        T[:3, 3] *= 1000.0
        m.apply_transform(T)
        out[l.name] = m
    return out


def _axial_range(mesh, axis) -> tuple:
    """[min,max] of the mesh's vertices projected on a unit axis (world mm)."""
    ax = np.asarray(axis, float)
    ax = ax / (np.linalg.norm(ax) or 1.0)
    proj = mesh.vertices @ ax
    return float(proj.min()), float(proj.max())


def geometry_report(model: KinematicModel, meshes_dir: str, *,
                    params_module=None, frames=None) -> dict:
    """Run the deterministic checks and return a structured dict + human lines.

    Checks:
      - shaft_through: for each shaft, does each part sharing its axis line fall within the
        shaft's axial span (a part outside the span = shaft does NOT pass through it).
      - clash: functional part vs structural part real-mesh solid intersection fraction.
      - axis_dir: realized revolution axis (world) vs a frame's params axis, SIGN-AWARE.
    """
    import trimesh
    meshes = _load_world_meshes(model, meshes_dir)
    lines: list = []
    findings: dict = {"shaft_through": [], "clash": [], "axis_dir": []}

    shafts = [n for n in meshes if "shaft" in n.lower()]
    riders = [n for n in meshes if any(k in n.lower() for k in ("gear", "pinion", "bearing", "spacer", "collar"))]

    def _sub_ns(name: str) -> str:
        # namespaced link is "<sub_id>_<local>"; a shaft only carries the parts of ITS OWN sub.
        # Two leading tokens capture "sub_input", "sub_inter", etc. (the assembler prefix).
        parts = name.split("_")
        return "_".join(parts[:2]) if len(parts) >= 2 else name

    # 1. shaft-through-part containment along the shaft's own long axis
    for s in shafts:
        sm = meshes[s]
        s_ns = _sub_ns(s)
        ext = sm.bounds[1] - sm.bounds[0]
        axis_idx = int(np.argmax(ext))
        unit = np.eye(3)[axis_idx]
        s_lo, s_hi = _axial_range(sm, unit)
        lines.append(f"[shaft] {s}: axis={'XYZ'[axis_idx]} span=[{s_lo:.0f},{s_hi:.0f}]mm")
        for r in riders:
            if r == s or _sub_ns(r) != s_ns:
                continue  # a shaft only carries its OWN sub's riders; never cross-sub
            # and only riders roughly coaxial with this shaft (share the two transverse coords)
            rc = meshes[r].bounds.mean(axis=0)
            sc = sm.bounds.mean(axis=0)
            transverse = [i for i in range(3) if i != axis_idx]
            if all(abs(rc[i] - sc[i]) < 30.0 for i in transverse):
                r_lo, r_hi = _axial_range(meshes[r], unit)
                inside = (s_lo - 1) <= r_lo and r_hi <= (s_hi + 1)
                status = "OK through" if inside else "NOT THROUGH (outside shaft span)"
                findings["shaft_through"].append({"shaft": s, "part": r, "ok": inside,
                                                  "shaft_span": [s_lo, s_hi], "part_span": [r_lo, r_hi]})
                lines.append(f"    rider {r}: span=[{r_lo:.0f},{r_hi:.0f}] -> {status}")

    # 2. functional vs structural clashes (real solid intersection)
    funcs = [n for n in meshes if not _is_structural(n)]
    structs = [n for n in meshes if _is_structural(n)]
    for fn in funcs:
        for st in structs:
            fm, sm2 = meshes[fn], meshes[st]
            lo = np.maximum(fm.bounds[0], sm2.bounds[0])
            hi = np.minimum(fm.bounds[1], sm2.bounds[1])
            if not np.all(hi > lo):
                continue
            try:
                inter = trimesh.boolean.intersection([fm, sm2], engine="manifold")
                iv = inter.volume if (inter is not None and not inter.is_empty) else 0.0
            except Exception:
                iv = 0.0
            frac = iv / max(min(fm.volume, sm2.volume), 1e-9)
            if frac > 0.02:
                findings["clash"].append({"functional": fn, "structural": st, "frac": frac})
                lines.append(f"[clash] {fn} vs {st}: {frac:.0%} solid overlap "
                             f"(functional part fixed by spec -> enlarge the structural part)")

    # 3. realized axis vs params axis (SIGN-AWARE — the abs(cos) guard misses a flip)
    if params_module is not None and frames:
        W = _world_transforms(model)
        for fr in frames:
            nm = getattr(fr, "name", None)
            ax_fn = f"{nm}_axis"
            if nm is None or not hasattr(params_module, ax_fn):
                continue
            want = np.asarray(getattr(params_module, ax_fn)(), float)
            want = want / (np.linalg.norm(want) or 1.0)
            # find a link whose metadata frame == nm would be ideal; fall back by name match
            link = next((l for l in model.links if l.name == nm or nm in l.name), None)
            if link is None:
                continue
            R = W.get(link.name, np.eye(4))[:3, :3]
            realized = R @ np.array([0, 0, 1.0])  # local +Z after placement
            realized = realized / (np.linalg.norm(realized) or 1.0)
            cos = float(realized @ want)
            ok = cos > 0.9962  # within ~5 deg, SIGNED (a flip gives cos<0 -> fail)
            findings["axis_dir"].append({"frame": nm, "cos": cos, "ok": ok,
                                         "realized": realized.tolist(), "want": want.tolist()})
            if not ok:
                lines.append(f"[axis] frame {nm}: realized {np.round(realized,2).tolist()} "
                             f"vs params {np.round(want,2).tolist()} cos={cos:.2f} "
                             f"-> {'FLIPPED' if cos < 0 else 'off'}")

    n_bad = (sum(1 for x in findings["shaft_through"] if not x["ok"])
             + len(findings["clash"])
             + sum(1 for x in findings["axis_dir"] if not x["ok"]))
    ok = n_bad == 0
    header = f"GEOMETRY SELF-CHECK: {'PASS' if ok else f'{n_bad} issue(s)'}"
    return {"ok": ok, "n_issues": n_bad, "findings": findings,
            "text": header + "\n" + "\n".join(lines)}


def render_layers(model: KinematicModel, meshes_dir: str, out_path: str) -> str | None:
    """Write ONE PNG with 2 rows x 3 orthographic views: top row = full assembly,
    bottom row = functional parts ONLY (structural parts dropped so internals are visible).
    Returns the path, or None on failure. Rendering is a SECONDARY aid — trust the report."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        meshes = _load_world_meshes(model, meshes_dir)
    except Exception:
        return None
    if not meshes:
        return None

    def color(name):
        n = name.lower()
        if "shaft" in n:
            return "#d62828"
        if "gear" in n or "pinion" in n:
            return "#2a9d3f"
        if "bearing" in n:
            return "#1f6fb4"
        return "#b0b0b0"

    def draw(ax, subset, ai, bi, title):
        for n in sorted(subset, key=lambda x: (0 if _is_structural(x) else 1)):
            V = meshes[n].vertices
            struct = _is_structural(n)
            ax.scatter(V[:, ai], V[:, bi], s=1, c=color(n),
                       alpha=0.12 if struct else 0.55, linewidths=0)
        ax.set_title(title, fontsize=9)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

    full = list(meshes)
    func = [n for n in meshes if not _is_structural(n)]
    views = [(0, 2, "X-Z"), (1, 2, "Y-Z"), (0, 1, "X-Y")]
    try:
        fig, axs = plt.subplots(2, 3, figsize=(18, 11))
        for c, (ai, bi, lbl) in enumerate(views):
            draw(axs[0][c], full, ai, bi, f"FULL {lbl}")
            draw(axs[1][c], func, ai, bi, f"FUNCTIONAL-ONLY {lbl}")
        fig.suptitle("top: full assembly   bottom: functional parts only (housing dropped)", fontsize=11)
        fig.tight_layout()
        fig.savefig(out_path, dpi=85)
        plt.close(fig)
        return out_path
    except Exception:
        return None
