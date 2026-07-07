"""KinematicModel -> MJCF (MuJoCo XML) for pure-contact simulation.

This is the REAL simulation path (maker2-mujoco-contact): the assembly is a set of
rigid bodies that settle and interact under GRAVITY, with transmission ONLY by mesh
contact — no motors, no joints between parts, nothing floating. Contrast urdf_builder
(visual/AABB-only) and the legacy PyBullet path (joint motors, fixed base).

Build rules:
  * <option> gravity + a small timestep + a stiff Newton/elliptic solver so gear
    teeth actually push instead of tunneling.
  * <worldbody> holds a ground <geom type="plane"> + a light. The assembly is NOT
    fixed-base: its root body rests on the plane by mass (or is pinned to the world
    only when settings.base_rests_on_plane is False).
  * Each link becomes a nested <body> under its POSE parent, at the pose's relative
    transform. dof: fixed -> no joint (welded to parent), spin -> <joint type="hinge"
    axis=spin_axis>, free -> <freejoint>.
  * Geometry: one <geom mesh=...> per CoACD piece of a movable/meshing part (so
    concavity collides), else one geom for the part's own mesh. STL mm -> meters via
    <mesh scale="0.001 0.001 0.001">.

Only imported when settings.engine == "mujoco".
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh.transformations as tf

from .convex_decomp import decompose_model


# Contact/solver tuning for meshing gear teeth. These are the knobs the golden
# 2-gear test exists to validate — if teeth tunnel or jitter, tune here.
_TIMESTEP = 2e-4
_SOLVER_ITERS = 150
_GEOM_FRICTION = "1.0 0.05 0.005"     # sliding, torsional, rolling
_GEOM_SOLREF = "0.002 1"              # stiff, ~critically damped contact
_GEOM_MARGIN = 0.0002                 # 0.2 mm — teeth engage just before touching


def _mat(xyz, rpy) -> np.ndarray:
    m = tf.euler_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]), axes="sxyz")
    m[:3, 3] = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    return m


def _pose_children(model) -> dict:
    """{parent_link: [PoseSpec...]} for parented poses only."""
    out: dict = {}
    for p in model.poses:
        if p.parent:
            out.setdefault(p.parent, []).append(p)
    return out


def _roots(model) -> list[str]:
    """Forest roots: links never placed as a parented pose's child."""
    parented = {p.child for p in model.poses if p.parent}
    roots = [l.name for l in model.links if l.name not in parented]
    if model.root_link and model.root_link not in roots:
        roots.append(model.root_link)
    return roots


def _rel_pose_of(model, child: str):
    """The (xyz, rpy) of a link relative to its pose-parent; identity if it's a root
    (roots are positioned by the <body pos> the caller assigns)."""
    for p in model.poses:
        if p.child == child and p.parent:
            return p.xyz_m, p.rpy_rad
    # An empty-parent pose gives a root its base offset.
    for p in model.poses:
        if p.child == child and not p.parent:
            return p.xyz_m, p.rpy_rad
    return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)


def _quat_from_rpy(rpy) -> str:
    """MuJoCo quaternion string (w x y z) from fixed-axis XYZ rpy."""
    q = tf.quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]), axes="sxyz")
    return f"{q[0]:.9g} {q[1]:.9g} {q[2]:.9g} {q[3]:.9g}"


def _add_geoms(body_el, link, piece_map, meshes_dir, mesh_names, asset_el):
    """Attach one <geom> per convex piece (movable/meshing parts) or one for the
    part's own mesh. Registers each referenced <mesh> in <asset> once."""
    pieces = piece_map.get(link.name)
    if pieces:
        sources = pieces
    else:
        own = os.path.join(meshes_dir, f"{link.name}.stl")
        sources = [own] if os.path.exists(own) and os.path.getsize(own) > 0 else []
    for i, src in enumerate(sources):
        mesh_name = f"{link.name}_m{i}"
        if mesh_name not in mesh_names:
            ET.SubElement(asset_el, "mesh", attrib={
                "name": mesh_name, "file": os.path.abspath(src),
                "scale": "0.001 0.001 0.001"})   # mm -> m
            mesh_names.add(mesh_name)
        rgba = ""
        if len(getattr(link, "color", ()) or ()) == 4:
            c = link.color
            rgba = f"{c[0]:.3g} {c[1]:.3g} {c[2]:.3g} {c[3]:.3g}"
        attrib = {"type": "mesh", "mesh": mesh_name,
                  "friction": _GEOM_FRICTION, "solref": _GEOM_SOLREF,
                  "margin": f"{_GEOM_MARGIN}", "condim": "4"}
        if rgba:
            attrib["rgba"] = rgba
        ET.SubElement(body_el, "geom", attrib=attrib)


def _emit_body(parent_el, link_name, model, links_by_name, children, piece_map,
               meshes_dir, mesh_names, asset_el, *, is_root: bool,
               base_height: float, pin_base: bool):
    link = links_by_name[link_name]
    xyz, rpy = _rel_pose_of(model, link_name)
    if is_root:
        # Roots spawn slightly above the plane so they settle down onto it (mirrors
        # the PyBullet base_height auto-settle) unless the base is pinned to world.
        pos = (float(xyz[0]), float(xyz[1]), float(xyz[2]) + base_height)
    else:
        pos = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    body = ET.SubElement(parent_el, "body", attrib={
        "name": link_name,
        "pos": f"{pos[0]:.9g} {pos[1]:.9g} {pos[2]:.9g}",
        "quat": _quat_from_rpy(rpy)})

    dof = getattr(link, "dof", "fixed")
    if is_root:
        if not pin_base:
            ET.SubElement(body, "freejoint", attrib={"name": f"{link_name}_free"})
        # pinned base: no joint => welded to world.
    elif dof == "spin":
        ax = getattr(link, "spin_axis", (0.0, 0.0, 1.0))
        ET.SubElement(body, "joint", attrib={
            "name": f"{link_name}_spin", "type": "hinge",
            "axis": f"{ax[0]:.6g} {ax[1]:.6g} {ax[2]:.6g}", "pos": "0 0 0"})
    elif dof == "free":
        ET.SubElement(body, "freejoint", attrib={"name": f"{link_name}_free"})
    # dof == "fixed": no joint (welded to its parent body).

    ET.SubElement(body, "inertial", attrib={
        "pos": "0 0 0", "mass": "0.05",
        "diaginertia": "1e-4 1e-4 1e-4"})  # placeholder; MuJoCo also infers from geom
    _add_geoms(body, link, piece_map, meshes_dir, mesh_names, asset_el)

    for p in children.get(link_name, []):
        _emit_body(body, p.child, model, links_by_name, children, piece_map,
                   meshes_dir, mesh_names, asset_el, is_root=False,
                   base_height=0.0, pin_base=False)


def _pitch_radius_m(meshes_dir: str, link_name: str) -> float | None:
    """Estimate a gear's pitch radius (meters) from the XY extent of its STL (mm)."""
    stl = os.path.join(meshes_dir, f"{link_name}.stl")
    if not (os.path.exists(stl) and os.path.getsize(stl) > 0):
        return None
    try:
        import trimesh
        mesh = trimesh.load(stl, force="mesh")
        ext = mesh.bounding_box.extents  # mm
        return float(max(ext[0], ext[1]) / 2.0) / 1000.0
    except Exception:
        return None


def _add_gear_constraints(mujoco_el, model, meshes_dir, links_by_name, *,
                          metrics: dict | None, log_fn) -> int:
    """ESCAPE HATCH (settings.allow_gear_constraint): for each mesh_pair, add a MuJoCo
    <equality><joint> gear-ratio constraint so the driven gear tracks the driver even
    when crude tooth contact would jam. Ratio = -(r_drive / r_driven) (meshed spur
    gears counter-rotate). Because the constraint REPLACES contact as the transmission
    mechanism, the two gears are also added to <contact><exclude> so their teeth don't
    collide and fight the constraint. Returns how many constraints were added and
    records metrics["constrained_meshes"]."""
    pairs = getattr(model, "mesh_pairs", []) or []
    if not pairs:
        return 0
    eq = ET.SubElement(mujoco_el, "equality")
    contact = ET.SubElement(mujoco_el, "contact")
    n = 0
    for (drive, driven) in pairs:
        da = links_by_name.get(drive)
        db = links_by_name.get(driven)
        if not da or not db or da.dof != "spin" or db.dof != "spin":
            continue
        rd = _pitch_radius_m(meshes_dir, drive)
        rn = _pitch_radius_m(meshes_dir, driven)
        ratio = -(rd / rn) if (rd and rn) else -1.0
        ET.SubElement(eq, "joint", attrib={
            "joint1": f"{driven}_spin", "joint2": f"{drive}_spin",
            "polycoef": f"0 {ratio:.6g} 0 0 0"})
        ET.SubElement(contact, "exclude", attrib={"body1": drive, "body2": driven})
        n += 1
    if n == 0:
        mujoco_el.remove(eq)
        mujoco_el.remove(contact)
        return 0
    if metrics is not None:
        metrics["constrained_meshes"] = n
    log_fn(f"[mjcf] added {n} gear-ratio constraint(s) + contact-exclude "
           f"(allow_gear_constraint ON)")
    return n


def build_mjcf(model, ctx, *, settings=None, metrics: dict | None = None,
               base_height: float = 0.01, log_fn=print) -> str:
    """Build an MJCF for the model and write it next to the URDF (model.mjcf).
    Returns the MJCF path. Decomposes movable/meshing parts first (cached)."""
    meshes_dir = ctx.meshes_dir
    piece_map = decompose_model(model, meshes_dir, metrics=metrics, log_fn=log_fn)

    pin_base = not getattr(settings, "base_rests_on_plane", True) if settings else False

    mujoco_el = ET.Element("mujoco", attrib={"model": model.name or "assembly"})
    ET.SubElement(mujoco_el, "option", attrib={
        "gravity": "0 0 -9.81", "timestep": f"{_TIMESTEP}",
        "iterations": f"{_SOLVER_ITERS}", "solver": "Newton", "cone": "elliptic"})
    asset_el = ET.SubElement(mujoco_el, "asset")
    world = ET.SubElement(mujoco_el, "worldbody")
    ET.SubElement(world, "light", attrib={"pos": "0 0 3", "dir": "0 0 -1",
                                          "diffuse": "0.8 0.8 0.8"})
    ET.SubElement(world, "geom", attrib={
        "name": "ground", "type": "plane", "size": "5 5 0.1",
        "friction": _GEOM_FRICTION, "condim": "4"})

    links_by_name = {l.name: l for l in model.links}
    children = _pose_children(model)
    mesh_names: set = set()
    roots = _roots(model)
    for rn in roots:
        if rn not in links_by_name:
            continue
        _emit_body(world, rn, model, links_by_name, children, piece_map,
                   meshes_dir, mesh_names, asset_el, is_root=True,
                   base_height=(0.0 if pin_base else base_height), pin_base=pin_base)

    # Optional gear-ratio escape hatch (off by default; pure contact is the intent).
    if settings is not None and getattr(settings, "allow_gear_constraint", False):
        _add_gear_constraints(mujoco_el, model, meshes_dir, links_by_name,
                              metrics=metrics, log_fn=log_fn)

    mjcf_path = os.path.splitext(ctx.urdf_path)[0] + ".mjcf"
    ET.indent(mujoco_el)
    Path(mjcf_path).write_text(ET.tostring(mujoco_el, encoding="unicode"),
                               encoding="utf-8")
    log_fn(f"[mjcf] wrote {mjcf_path} ({len(roots)} root body(ies), "
           f"{len(mesh_names)} mesh geom(s))")
    return mjcf_path
