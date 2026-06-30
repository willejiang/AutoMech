"""KinematicModel -> model.urdf via yourdfpy, plus mesh scaffolding + validation.

The URDF is the integration contract: it is written *before* any geometry is
built (so workers have empty mesh files to fill), and re-validated *after* the
build (so we know every mesh resolves and loads).

Units: workers build in mm; each <mesh> carries scale="0.001 0.001 0.001" so the
mm geometry renders at meter scale. Joint origins are authored by the manager in
meters and encoded into the 4x4 origin matrix with the URDF fixed-axis XYZ (sxyz)
rpy convention — verified to round-trip through yourdfpy.
"""

from __future__ import annotations

import os

import numpy as np
import trimesh.transformations as tf
from yourdfpy import (URDF, Robot, Link, Joint, Visual, Collision, Geometry,
                      Mesh, Limit, Material, Color)

from .model import KinematicModel, JointSpec, LinkSpec, RunContext


# mm geometry -> meter URDF
_MM_TO_M_SCALE = (0.001, 0.001, 0.001)
_NONFIXED = {"revolute", "prismatic", "continuous"}

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


def _origin_matrix(joint: JointSpec) -> np.ndarray:
    """4x4 homogeneous transform from a joint's xyz (m) + rpy (rad, sxyz)."""
    rx, ry, rz = joint.rpy_rad
    m = tf.euler_matrix(rx, ry, rz, axes="sxyz")
    m[:3, 3] = list(joint.xyz_m)
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


def _build_joint(j: JointSpec) -> Joint:
    kwargs = dict(name=j.name, type=j.type, parent=j.parent, child=j.child,
                  origin=_origin_matrix(j))
    if j.type in _NONFIXED:
        kwargs["axis"] = np.array(list(j.axis), dtype=float)
        if j.type in ("revolute", "prismatic"):
            kwargs["limit"] = Limit(effort=j.effort, velocity=j.velocity,
                                    lower=j.lower, upper=j.upper)
    return Joint(**kwargs)


def build_urdf(model: KinematicModel, ctx: RunContext) -> str:
    """Build the URDF from the model and write it to ``ctx.urdf_path``."""
    robot = Robot(
        name=model.name,
        links=[_build_link(l, i) for i, l in enumerate(model.links)],
        joints=[_build_joint(j) for j in model.joints],
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
