"""KinematicModel -> model.urdf via yourdfpy, plus mesh scaffolding + validation.

VISUAL-ONLY (maker2-mujoco-contact): the real simulation is MuJoCo contact under
gravity (see maker2/mjcf_builder.py). This URDF is retained ONLY for the appearance
judge render + the precheck AABB load — it maps every part to a placement in one
tree so yourdfpy can assemble a scene. Because a pure-contact model is a FOREST
(parts placed by pose, no single root), we inject a synthetic ``world`` base link
and weld every forest-root part to it at its declared pose; every non-root part
hangs off its pose-parent via a fixed joint (placement only — DOF is ignored here;
the MJCF builder is what honors dof/contact).

Units: workers build in mm; each <mesh> carries scale="0.001 0.001 0.001" so the
mm geometry renders at meter scale. Pose origins are authored by the manager in
meters and encoded into the 4x4 origin matrix with the URDF fixed-axis XYZ (sxyz)
rpy convention — verified to round-trip through yourdfpy.
"""

from __future__ import annotations

import os

import numpy as np
import trimesh.transformations as tf
from yourdfpy import (URDF, Robot, Link, Joint, Visual, Collision, Geometry,
                      Mesh, Material, Color)

from .model import KinematicModel, LinkSpec, PoseSpec, RunContext


# mm geometry -> meter URDF
_MM_TO_M_SCALE = (0.001, 0.001, 0.001)

# Synthetic base link that every forest-root part welds to (a pure-contact model
# has no single root, but a URDF needs exactly one).
_WORLD_LINK = "world"

# Fallback palette (RGB 0..1) so a link the manager left uncolored still renders
# as a distinct solid instead of the default gray. Indexed by link order.
_FALLBACK_PALETTE = [
    (0.82, 0.27, 0.24), (0.27, 0.51, 0.78), (0.35, 0.71, 0.35), (0.90, 0.67, 0.20),
    (0.59, 0.39, 0.78), (0.24, 0.75, 0.75), (0.86, 0.47, 0.67), (0.63, 0.63, 0.31),
    (0.47, 0.43, 0.39), (0.78, 0.78, 0.82), (0.71, 0.35, 0.24), (0.31, 0.59, 0.47),
]


def _rel_mesh(link: LinkSpec) -> str:
    """URDF-relative mesh path (forward slashes, resolved against the URDF dir)."""
    return link.mesh_filename or f"meshes/{link.name}.stl"


def _origin_matrix(xyz_m, rpy_rad) -> np.ndarray:
    """4x4 homogeneous transform from xyz (m) + rpy (rad, sxyz)."""
    rx, ry, rz = rpy_rad
    m = tf.euler_matrix(rx, ry, rz, axes="sxyz")
    m[:3, 3] = list(xyz_m)
    return m


def _mesh_geometry(rel_filename: str) -> Geometry:
    return Geometry(mesh=Mesh(filename=rel_filename,
                              scale=np.array(_MM_TO_M_SCALE, dtype=float)))


def _link_material(link: LinkSpec, idx: int) -> Material:
    """A URDF <material> for the link: its manager color, else a palette color.

    yourdfpy applies this rgba to the loaded mesh, so the judge's render shows
    each part in its own solid color instead of a uniform gray."""
    rgba = link.color if len(link.color) == 4 else (
        *_FALLBACK_PALETTE[idx % len(_FALLBACK_PALETTE)], 1.0)
    return Material(name=f"{link.name}_mat",
                    color=Color(rgba=np.array(rgba, dtype=float)))


def _build_link(link: LinkSpec, idx: int) -> Link:
    rel = _rel_mesh(link)
    return Link(
        name=link.name,
        visuals=[Visual(name=f"{link.name}_visual", geometry=_mesh_geometry(rel),
                        material=_link_material(link, idx))],
        collisions=[Collision(name=f"{link.name}_collision",
                              geometry=_mesh_geometry(rel))],
    )


def _placement_joint(name: str, parent: str, child: str,
                     xyz_m, rpy_rad) -> Joint:
    """A fixed URDF joint that only PLACES the child under the parent (no DOF —
    this URDF is visual/AABB-only; MuJoCo owns the real motion)."""
    return Joint(name=name, type="fixed", parent=parent, child=child,
                 origin=_origin_matrix(xyz_m, rpy_rad))


def build_urdf(model: KinematicModel, ctx: RunContext) -> str:
    """Build a VISUAL-ONLY URDF from the model and write it to ``ctx.urdf_path``.

    Every part is placed by a fixed joint (DOF is ignored — see module docstring).
    A synthetic ``world`` base link is injected and each forest-root part (any link
    that is never a pose child) is welded to it at its declared pose, so the
    otherwise-rootless forest becomes the single tree yourdfpy requires."""
    links = [_build_link(l, i) for i, l in enumerate(model.links)]
    joints: list[Joint] = []
    child_of = {p.child: p for p in model.poses}

    # Non-root parts: place under their pose-parent (skip poses with no real parent —
    # those parts are forest roots, handled by the world weld below).
    for p in model.poses:
        if p.parent and p.parent != _WORLD_LINK:
            joints.append(_placement_joint(p.name, p.parent, p.child,
                                           p.xyz_m, p.rpy_rad))

    # Forest roots: any link with no incoming parented pose. Weld each to `world` at
    # its own (empty-parent) pose if it has one, else at the origin.
    root_names = [l.name for l in model.links
                  if not (child_of.get(l.name) and child_of[l.name].parent
                          and child_of[l.name].parent != _WORLD_LINK)]
    for rn in root_names:
        p = child_of.get(rn)
        xyz = p.xyz_m if p else (0.0, 0.0, 0.0)
        rpy = p.rpy_rad if p else (0.0, 0.0, 0.0)
        joints.append(_placement_joint(f"world_to_{rn}", _WORLD_LINK, rn, xyz, rpy))

    robot = Robot(
        name=model.name,
        links=[Link(name=_WORLD_LINK)] + links,
        joints=joints,
    )
    urdf = URDF(robot=robot, build_scene_graph=False, load_meshes=False)
    os.makedirs(os.path.dirname(ctx.urdf_path), exist_ok=True)
    urdf.write_xml_file(ctx.urdf_path)
    return ctx.urdf_path


def scaffold_meshes(model: KinematicModel, ctx: RunContext) -> dict[str, str]:
    """Create the meshes dir and a 0-byte placeholder per link.

    Returns {link_name: absolute_native_path}. Existing non-empty files are
    left untouched (so a partial re-run doesn't clobber good meshes).
    """
    os.makedirs(ctx.meshes_dir, exist_ok=True)
    paths: dict[str, str] = {}
    for link in model.links:
        rel = _rel_mesh(link).replace("/", os.sep)
        abs_path = os.path.join(ctx.run_dir, rel)
        os.makedirs(os.path.dirname(abs_path), exist_ok=True)
        if not os.path.exists(abs_path):
            open(abs_path, "wb").close()
        paths[link.name] = abs_path
    return paths


def validate_urdf(path: str, require_meshes: bool = False) -> tuple[bool, str]:
    """Validate the URDF by loading it.

    With ``require_meshes=False`` only the topology is parsed (pre-fill check).
    With ``require_meshes=True`` yourdfpy is asked to resolve + load every mesh
    and assemble the scene (post-fill check) — this catches a broken path or
    malformed STL.

    CAVEAT: yourdfpy only *warns* on a missing or 0-byte mesh; it does not
    raise. So this returning True does NOT prove every mesh was actually filled.
    The authoritative per-link gate is ``validation.check_stl``; treat this as a
    secondary "does the whole thing assemble" sanity check.
    """
    try:
        URDF.load(path, load_meshes=require_meshes,
                  build_scene_graph=require_meshes,
                  force_mesh=require_meshes)
        return True, ""
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"
