"""Per-subassembly rigid-conflict detection.

Runs RIGHT AFTER a subassembly's worker finishes (in orchestrator_boss.
_finish_subassembly), on that ONE sub's own model.urdf — long before the whole-machine
precheck. It reuses precheck's world-mesh primitives, but on the sub's UN-namespaced link
names (precheck._part_overlaps works on the assembled, namespaced robot; here every link
is just its own `l.name`).

Why this exists: the manager authors every part's placement (PoseSpec.xyz_m/rpy_rad) in
one blind LLM call with no geometric feedback, while the worker decides the actual solid
geometry at the part's local origin. When the two disagree, parts the manager thought
were clear interpenetrate. This detector gives the debugger the exact offending pair so
it can fix the pose or the geometry before the sub is accepted.

Overlap is measured on the REAL mesh solids (precheck._solid_intersection_frac, a
manifold boolean), NOT bounding boxes: a hollow part's bore/keyway/cut is empty, so a
shaft in a bearing bore, a key in a keyway, a bearing pressed into a housing bore, a plug
through a hole all read ~0 overlap — only genuinely interpenetrating metal scores high.
This is what lets the gate be strict (fail a sub on a real conflict) without mis-flagging
the many intended nestings that fill a machine.
"""

from __future__ import annotations

from dataclasses import dataclass

from .precheck import _OVERLAP_FRAC, _link_world_mesh, _solid_intersection_frac
from .viz import load_robot


@dataclass
class Conflict:
    """Two rigid parts in one subassembly that grossly interpenetrate."""

    part_a: str
    part_b: str
    frac: float                                     # shared/smaller-part SOLID volume, [0,1]

    def describe(self) -> str:
        return (f"parts '{self.part_a}' and '{self.part_b}' interpenetrate "
                f"({self.frac:.0%} of the smaller part's solid is inside the other)")


def sub_conflicts(model, urdf_path: str, log_fn=print) -> list[Conflict]:
    """Load the sub's own URDF and flag non-adjacent link pairs whose SOLIDS grossly
    interpenetrate. Returns worst-first; [] if the URDF can't be loaded.

    Skips pairs directly joined by a pose edge (parent<->child) as intended nesting. Every
    other pair is scored by REAL mesh-solid overlap (_solid_intersection_frac): a pair is
    a conflict only when its solid intersection is >= _OVERLAP_FRAC of the smaller part.
    Because the measure sees hollow interiors, coaxial shaft-in-bore / key-in-keyway /
    bearing-in-housing nestings read ~0 and need no special-case exemption."""
    try:
        robot = load_robot(urdf_path)
    except Exception as e:
        log_fn(f"[conflict] could not load sub URDF ({type(e).__name__}: {e}); skipping check")
        return []

    # Pose-adjacent link pairs -> intended nesting, never flagged. Read model.poses
    # DIRECTLY (not the lossy model.joints view, which DROPS forest-root poses with an
    # empty parent — so a part placed as a root would never be exempted).
    adj = {frozenset((p.parent, p.child))
           for p in model.poses if p.parent and p.child}

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
            frac = _solid_intersection_frac(meshes[a], meshes[b], log_fn=log_fn)
            if frac >= _OVERLAP_FRAC:
                found.append(Conflict(part_a=a, part_b=b, frac=frac))
            elif frac > 0.05:
                log_fn(f"[conflict] drop small overlap {a}~{b} "
                       f"({frac:.0%} < {_OVERLAP_FRAC:.0%})")

    found.sort(key=lambda c: c.frac, reverse=True)
    return found
