"""Per-subassembly rigid-conflict detection.

Runs RIGHT AFTER a subassembly's worker finishes (in orchestrator_boss.
_finish_subassembly), on that ONE sub's own model.urdf — long before the whole-machine
precheck. It reuses precheck's world-mesh + AABB-overlap primitives, but on the sub's
UN-namespaced link names (precheck._part_overlaps works on the assembled, namespaced
robot; here every link is just its own `l.name`).

Why this exists: the manager authors every part's placement (JointSpec.xyz_m/rpy_rad) in
one blind LLM call with no geometric feedback, while the worker decides the actual solid
geometry at the part's local origin. When the two disagree, parts the manager thought
were clear interpenetrate. This detector gives the debugger the exact offending pair so
it can fix the pose or the geometry before the sub is accepted.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .precheck import _OVERLAP_FRAC, _intersection_frac, _link_world_mesh
from .viz import load_robot


# Two parts are "coaxial" when the guest's center, projected onto the plane perpendicular
# to the bore axis, sits within the bore radius of the host center (m added as slack). A
# shaft threaded through a bore is near-perfectly centered; a part crashing in from the
# side lands well outside the bore footprint.
_COAX_SLACK_M = 3e-3


@dataclass
class Conflict:
    """Two rigid parts in one subassembly that grossly interpenetrate."""

    part_a: str
    part_b: str
    frac: float                                     # shared/smaller-part AABB volume, [0,1]

    def describe(self) -> str:
        return (f"parts '{self.part_a}' and '{self.part_b}' interpenetrate "
                f"({self.frac:.0%} of the smaller part is inside the other)")


def _bore_dia_mm(link) -> float | None:
    """The declared through-bore diameter of a ring/tube/bearing/housing-bore part, in mm,
    or None if it has none. A part with a bore is a HOLLOW cylinder — its AABB is filled
    but its interior legitimately accepts a coaxial mate (shaft/bearing/cap)."""
    size = getattr(link, "size_mm", {}) or {}
    for k in ("bore_dia", "bore", "inner_dia", "id", "hole_dia"):
        v = size.get(k)
        try:
            if v is not None and float(v) > 0:
                return float(v)
        except (TypeError, ValueError):
            pass
    return None


def _ring_axis(mesh) -> int:
    """The through-bore axis of a ring/bearing/disc = its SHORTEST AABB extent. A bearing
    or bore-plate is a flat disc (width << diameter), so the bore runs along the thin
    dimension — the AABB's smallest extent, NOT its largest."""
    lo, hi = np.asarray(mesh.bounds[0]), np.asarray(mesh.bounds[1])
    return int(np.argmin(hi - lo))


def _is_concentric_nest(link_a, mesh_a, link_b, mesh_b) -> bool:
    """True when one part legitimately nests INSIDE the other's declared bore, coaxially —
    a shaft in a bearing bore, a bearing outer race in a housing bore, a cap over a shaft.
    Deterministic from the manager's DECLARED bore_dia + the real mesh placement:

      * one part (the host) declares a bore (hollow cylinder), AND
      * the guest passes THROUGH the host: its center lies within the host's axial span
        along the bore axis, AND
      * they are CONCENTRIC: the guest center sits within the bore radius (+slack) of the
        host center, measured in the plane perpendicular to the bore axis.

    This is the class of pair the AABB overlap metric CANNOT judge (a ring's bounding box
    reads as solid, so any coaxial mate looks ~50-100% overlapped). We do NOT compare
    outer diameters (a STEPPED shaft is Ø40 at a shoulder and Ø25 at the seat — its AABB
    cross-section is the shoulder, not the seat, so a diameter-fit test breaks). Instead we
    require the guest to be centered in the bore and threaded through it — a part crashing
    into the ring from the side is off-center or beside it, so real conflicts still fail.

    Special case — TWO coaxial bored rings (e.g. front + rear bearing on one shaft axis):
    both declare a bore and are radially concentric, but sit at different points ALONG the
    axis. Their Ø-wide-but-thin discs never interpenetrate, yet their bounding CUBES clip
    (~40%). When both parts are bored and concentric, exempt regardless of axial offset —
    two concentric rings on a shaft are an intended layout, and a genuine crash would put
    them non-concentric."""
    a_bore = _bore_dia_mm(link_a)
    b_bore = _bore_dia_mm(link_b)
    # Both bored + concentric on a shared axis -> coaxial rings (front/rear bearings).
    if a_bore is not None and b_bore is not None:
        axis = _ring_axis(mesh_a)
        ca = (np.asarray(mesh_a.bounds[0]) + np.asarray(mesh_a.bounds[1])) / 2.0
        cb = (np.asarray(mesh_b.bounds[0]) + np.asarray(mesh_b.bounds[1])) / 2.0
        perp = [i for i in (0, 1, 2) if i != axis]
        radial_off = float(np.hypot(ca[perp[0]] - cb[perp[0]], ca[perp[1]] - cb[perp[1]]))
        if radial_off <= (max(a_bore, b_bore) / 2.0) / 1000.0 + _COAX_SLACK_M:
            return True
    for host_link, host_mesh, guest_mesh in (
            (link_a, mesh_a, mesh_b), (link_b, mesh_b, mesh_a)):
        bore_mm = _bore_dia_mm(host_link)
        if bore_mm is None:
            continue
        axis = _ring_axis(host_mesh)
        hlo, hhi = np.asarray(host_mesh.bounds[0]), np.asarray(host_mesh.bounds[1])
        glo, ghi = np.asarray(guest_mesh.bounds[0]), np.asarray(guest_mesh.bounds[1])
        host_c = (hlo + hhi) / 2.0
        guest_c = (glo + ghi) / 2.0
        # The guest must pass THROUGH the ring: the ring's axial span must fall within the
        # guest's axial span (the shaft is longer than the bearing is wide).
        if not (glo[axis] <= host_c[axis] <= ghi[axis]):
            continue
        # Concentric: radial offset of centers within the bore radius (+ slack).
        perp = [i for i in (0, 1, 2) if i != axis]
        radial_off = float(np.hypot(host_c[perp[0]] - guest_c[perp[0]],
                                    host_c[perp[1]] - guest_c[perp[1]]))
        if radial_off <= (bore_mm / 2.0) / 1000.0 + _COAX_SLACK_M:
            return True
    return False


def sub_conflicts(model, urdf_path: str, log_fn=print) -> list[Conflict]:
    """Load the sub's own URDF and flag non-adjacent link pairs that grossly overlap.

    Skips pairs that are (a) directly joined by a pose edge (parent<->child), or (b) a
    concentric NEST (shaft-in-bore / bearing-in-housing) detected from declared bore_dia +
    coaxial mesh placement — both are INTENDED interpenetration. Flags a remaining pair
    only when its AABB intersection fraction is >= _OVERLAP_FRAC; returns them worst-first.
    Returns [] if the URDF can't be loaded or no mesh bounds are available."""
    try:
        robot = load_robot(urdf_path)
    except Exception as e:
        log_fn(f"[conflict] could not load sub URDF ({type(e).__name__}: {e}); skipping check")
        return []

    # Pose-adjacent link pairs -> intended nesting (shaft in a bearing), never flagged.
    # Read model.poses DIRECTLY (not the lossy model.joints view, which DROPS forest-root
    # poses with an empty parent — so a bearing placed as a root would never be exempted).
    adj = {frozenset((p.parent, p.child))
           for p in model.poses if p.parent and p.child}
    links_by_name = {l.name: l for l in model.links}

    names = [l.name for l in model.links]
    meshes = {}
    for n in names:
        m = _link_world_mesh(robot, n)
        if m is not None:
            meshes[n] = m
    present = list(meshes)

    found: list[Conflict] = []
    for i in range(len(present)):
        for k in range(i + 1, len(present)):
            a, b = present[i], present[k]
            if frozenset((a, b)) in adj:
                continue
            # Concentric shaft-in-bore / bearing-in-housing: intended, AABB can't see it.
            if _is_concentric_nest(links_by_name.get(a), meshes[a],
                                   links_by_name.get(b), meshes[b]):
                log_fn(f"[conflict] exempt concentric nest {a}~{b} (bore fit + coaxial)")
                continue
            frac = _intersection_frac(meshes[a], meshes[b])
            if frac >= _OVERLAP_FRAC:
                found.append(Conflict(part_a=a, part_b=b, frac=frac))
            elif frac > 0.05:
                log_fn(f"[conflict] drop small overlap {a}~{b} "
                       f"({frac:.0%} < {_OVERLAP_FRAC:.0%})")

    found.sort(key=lambda c: c.frac, reverse=True)
    return found
