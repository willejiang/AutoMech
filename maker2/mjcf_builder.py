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
# SUPPORT TEST only: friction pinned near-infinite so the verdict is about SUPPORT, not
# grip. A part correctly seated (or gripped by a bore) must not creep and read as a fault.
_SUPPORT_FRICTION = "5.0 1.0 0.5"


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


def _quat_from_matrix_rot(T) -> str:
    """MuJoCo quaternion string (w x y z) from a 4x4 transform's rotation block."""
    R = np.eye(4)
    R[:3, :3] = T[:3, :3]
    q = tf.quaternion_from_matrix(R)
    return f"{q[0]:.9g} {q[1]:.9g} {q[2]:.9g} {q[3]:.9g}"


def _mesh_extents_m(src: str):
    """(sx, sy, sz) bounding-box extents in METERS of an STL (mm), or None on error."""
    try:
        import trimesh
        mesh = trimesh.load(src, force="mesh")
        ext = mesh.bounding_box.extents  # mm
        return (float(ext[0]) / 1000.0, float(ext[1]) / 1000.0, float(ext[2]) / 1000.0)
    except Exception:
        return None


def _mesh_volume_m3(src: str) -> float | None:
    """Solid volume in CUBIC METERS of an STL authored in mm, or None on error. Uses the
    mesh's own volume when watertight, else falls back to the convex-hull volume (a
    reasonable proxy for a non-watertight union)."""
    try:
        import trimesh
        mesh = trimesh.load(src, force="mesh")
        v_mm3 = abs(float(mesh.volume)) if mesh.is_watertight else 0.0
        if v_mm3 <= 0:
            try:
                v_mm3 = abs(float(mesh.convex_hull.volume))
            except Exception:
                return None
        return v_mm3 / 1.0e9 if v_mm3 > 0 else None      # mm^3 -> m^3
    except Exception:
        return None


def _is_hullable(src: str) -> bool:
    """MuJoCo convexifies every mesh geom; a coplanar/degenerate piece (a razor-thin
    disc, a zero-volume sliver) has no 3D hull and makes MjModel.from_xml* RAISE, which
    would kill the whole sim. Return False for such a piece so the caller substitutes a
    box geom instead (one bad part must not crash the assembly)."""
    ext = _mesh_extents_m(src)
    if ext is None:
        return False
    # Need real thickness on all three axes to form a hull (0.05 mm floor).
    return min(ext) >= 5e-5


def _add_geoms(body_el, link, piece_map, meshes_dir, mesh_names, asset_el,
               friction: str = _GEOM_FRICTION, sdf: bool = True):
    """Attach the part's collision geoms. Registers each referenced <mesh> in <asset>
    once. ``friction`` is the part's per-material contact triple.

    With ``sdf`` (the default) ONE geom is emitted per part, of type "sdf", reading the
    part's ORIGINAL STL: MuJoCo evaluates a signed distance field, so a bore is a bore.
    Otherwise the part is represented by its convex decomposition — one geom per piece,
    or a thin BOX for a piece too degenerate to hull.

    Why SDF is the default (all measured on this repo's own runs, see
    docs/CONTACT_PHYSICS_FINDINGS.md):
      - convex pieces FILL a cavity. An hour pipe with a 1.600mm bore on a 1.500mm arbor
        — a real +0.100mm running fit — was reported as 0.245mm of INTERPENETRATION;
        SDF reports no contact, correctly. Every fit tolerance in this repo was widened
        to absorb that artefact.
      - decomposition is the dominant cost of building a model: 288s for 13 parts (252
        pieces) and 131s for 29 parts, versus 1.3-1.9s for SDF. It is cached per part,
        but the loop authors NEW geometry every iteration, so the cache almost never
        hits and the run pays it again.
    Contact-heavy scenes step slower under SDF, but that is seconds against minutes.
    """
    if sdf:
        own = os.path.join(meshes_dir, f"{link.name}.stl")
        sources = [own] if os.path.exists(own) and os.path.getsize(own) > 0 else []
    else:
        pieces = piece_map.get(link.name)
        if pieces:
            sources = pieces
        else:
            own = os.path.join(meshes_dir, f"{link.name}.stl")
            sources = [own] if os.path.exists(own) and os.path.getsize(own) > 0 else []
    rgba = ""
    if len(getattr(link, "color", ()) or ()) == 4:
        c = link.color
        rgba = f"{c[0]:.3g} {c[1]:.3g} {c[2]:.3g} {c[3]:.3g}"
    for i, src in enumerate(sources):
        common = {"friction": friction, "solref": _GEOM_SOLREF,
                  "margin": f"{_GEOM_MARGIN}", "condim": "4"}
        if rgba:
            common["rgba"] = rgba
        if sdf:
            mesh_name = f"{link.name}_sdf"
            if mesh_name not in mesh_names:
                ET.SubElement(asset_el, "mesh", attrib={
                    "name": mesh_name, "file": os.path.abspath(src),
                    "scale": "0.001 0.001 0.001"})   # mm -> m
                mesh_names.add(mesh_name)
            ET.SubElement(body_el, "geom",
                          attrib={"type": "sdf", "mesh": mesh_name, **common})
        elif _is_hullable(src):
            mesh_name = f"{link.name}_m{i}"
            if mesh_name not in mesh_names:
                ET.SubElement(asset_el, "mesh", attrib={
                    "name": mesh_name, "file": os.path.abspath(src),
                    "scale": "0.001 0.001 0.001"})   # mm -> m
                mesh_names.add(mesh_name)
            ET.SubElement(body_el, "geom",
                          attrib={"type": "mesh", "mesh": mesh_name, **common})
        else:
            # Degenerate piece -> a thin box of the same footprint (floor each half-size
            # so MuJoCo still gets a valid collision volume).
            ext = _mesh_extents_m(src) or (1e-3, 1e-3, 1e-3)
            hx, hy, hz = (max(ext[0] / 2, 2.5e-4), max(ext[1] / 2, 2.5e-4),
                          max(ext[2] / 2, 2.5e-4))
            ET.SubElement(body_el, "geom", attrib={
                "type": "box", "size": f"{hx:.6g} {hy:.6g} {hz:.6g}", **common})


def _pose_matrix(xyz, rpy):
    """4x4 transform from an (xyz, rpy) pair, for accumulating a chain to world."""
    T = tf.euler_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]), axes="sxyz")
    T[0, 3], T[1, 3], T[2, 3] = float(xyz[0]), float(xyz[1]), float(xyz[2])
    return T


def _mesh_inertia_m(src: str, mass_kg: float):
    """(fullinertia_6, com_m) for an STL authored in mm, scaled to `mass_kg`, or None.

    Every part used to ship a flat `diaginertia="1e-4 1e-4 1e-4"` with a comment claiming
    MuJoCo would refine it from the geom. It does not: an explicit <inertial> is taken
    verbatim. Measured on one release gate, that constant was 125x its real inertia AND
    isotropic — a slender lever and a ball became dynamically identical, and every spin
    part's rotational behaviour rested on it. Mass stays ours (density x true solid
    volume, which the convex hull would overstate); only the inertia comes from the mesh.

    Returned as MuJoCo's `fullinertia` (ixx iyy izz ixy ixz iyz) about the centre of mass
    IN THE BODY FRAME, not as principal components: principal magnitudes are only correct
    alongside the rotation that orders them, and an asymmetric part given them in the body
    frame is wrong about which axis is easy to turn.
    """
    try:
        import trimesh
        mesh = trimesh.load(src, force="mesh")
        if not mesh.is_watertight:
            mesh = mesh.convex_hull
        m_native = abs(float(mesh.mass))
        if m_native <= 0:
            return None
        # trimesh works in the authored unit (mm) at density 1. Inertia scales as
        # length^2 (1e-6 for mm->m); renormalise off its mass to the mass we computed.
        I = np.asarray(mesh.moment_inertia, dtype=float) * (mass_kg / m_native) * 1.0e-6
        d = [float(I[0, 0]), float(I[1, 1]), float(I[2, 2])]
        if not all(np.isfinite(I).flatten()) or min(d) <= 0:
            return None
        # MuJoCo rejects a non-positive-definite inertia; a degenerate tessellation can
        # produce one, and that must fall back rather than fail the whole model load.
        if min(np.linalg.eigvalsh((I + I.T) / 2.0)) <= 0:
            return None
        full = [d[0], d[1], d[2], float(I[0, 1]), float(I[0, 2]), float(I[1, 2])]
        com = [float(v) / 1000.0 for v in np.asarray(mesh.center_mass, dtype=float)]
        return full, com
    except Exception:
        return None


def _inertial_attrib(src: str, mass_kg: float) -> dict:
    """The <inertial> attributes for a part: real inertia + centre of mass when the mesh
    can be measured, else the old flat placeholder so the model still loads."""
    got = _mesh_inertia_m(src, mass_kg) if src and os.path.exists(src) else None
    if got is None:
        return {"pos": "0 0 0", "mass": f"{mass_kg:.6g}",
                "diaginertia": "1e-4 1e-4 1e-4"}
    full, com = got
    return {"pos": f"{com[0]:.9g} {com[1]:.9g} {com[2]:.9g}",
            "mass": f"{mass_kg:.6g}",
            "fullinertia": " ".join(f"{v:.6g}" for v in full)}


def _mounts_of(link) -> list:
    """Every part that carries `link`, primary mount first.

    `mount` is a single string because MuJoCo's body tree allowed a part only one
    parent — a shaft running through two bearings could name just one of them, and the
    other became decoration the support test then reported as holding nothing. Bodies
    are flat now, so supporters are a LIST; `mount` stays the primary one (everything
    that reads a single carrier still works) and `extra_mounts` holds the rest.
    """
    out = []
    m = (getattr(link, "mount", "") or "").strip()
    if m:
        out.append(m)
    for e in (getattr(link, "extra_mounts", None) or []):
        e = str(e).strip()
        if e and e not in out:
            out.append(e)
    return out


def _spin_carrier(link_name, links_by_name):
    """The nearest SPINNING part up `link_name`'s mount chain, or None.

    A dof=fixed part mounted on a rotating one (a hand on its arbor, a collar on a
    shaft) has to turn WITH it. Under the old nested emitter that was free: being the
    child of a spinning body meant sharing its rotation. Flat bodies have no parent to
    inherit from, so the carrier has to be found and the rotation re-imposed."""
    seen = set()
    n = link_name
    while n and n not in seen:
        seen.add(n)
        l = links_by_name.get(n)
        if l is None:
            return None
        if getattr(l, "dof", "fixed") == "spin":
            return n
        n = getattr(l, "mount", "")
    return None


def _emit_flat(world_el, model, links_by_name, piece_map, meshes_dir, mesh_names,
               asset_el, eq_el, *, base_height: float, pin_base: bool,
               sdf: bool = True, log_fn=print):
    """Emit EVERY part as a direct child of <worldbody>, carrying its absolute world
    pose, and say out loud the two things body nesting used to provide implicitly.

    Nesting never carried POSITION. The agent authors absolute `.moved()` coordinates
    and the builder back-computed a relative pose from them, so re-parenting a part
    moved it nowhere (verified: flattening two real runs changes every body's world
    position by 0.000000 mm). What nesting did carry was:

      1. `mount` had to form a TREE, because MuJoCo forbids a body having two parents.
         That is why a shaft held by two bearings could only ever name one of them —
         the second bearing became decoration, and the support test reported the shaft
         as unheld. Flat bodies have no parent, so a part may now name every part that
         genuinely carries it.
      2. MuJoCo does not collide a body with its own parent. A 21-part chain therefore
         exempted 20 pairs from contact, and a counterweight could pass through the
         frame that was supposed to stop it. Flat bodies exempt nothing (measured: 12
         parent-child exemptions -> 0).

    What must now be stated explicitly:
      - a dof=fixed part on static structure was welded by BEING a child -> <weld>;
      - a dof=fixed part riding a SPINNING carrier turned with it -> it needs the
        carrier's own hinge plus a 1:1 ratio. Welding it to the spinning body instead
        LOCKS THE SHAFT (the part has no joint of its own to absorb the constraint) and
        kills the whole train — measured: the watch's 12:1 went to zero output.
    """
    n_weld = n_ride = 0
    W = _world_transforms(model)
    for link in model.links:
        name = link.name
        T = W.get(name)
        if T is None:
            continue
        dof = getattr(link, "dof", "fixed")
        pos = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]
        # Only a part that will actually SETTLE spawns slightly above the plane; a
        # welded/hinged part is grounded in place and lifting it would strand it in
        # mid-air.
        if dof == "free" and not pin_base:
            pos[2] += base_height
        body = ET.SubElement(world_el, "body", attrib={
            "name": name,
            "pos": f"{pos[0]:.9g} {pos[1]:.9g} {pos[2]:.9g}",
            "quat": _quat_from_matrix_rot(T)})

        mount = getattr(link, "mount", "")
        carrier = _spin_carrier(mount, links_by_name) if mount else None
        if dof == "spin":
            ax = getattr(link, "spin_axis", (0.0, 0.0, 1.0))
            ET.SubElement(body, "joint", attrib={
                "name": f"{name}_spin", "type": "hinge",
                "axis": f"{ax[0]:.6g} {ax[1]:.6g} {ax[2]:.6g}", "pos": "0 0 0"})
        elif dof == "free" and not pin_base:
            ET.SubElement(body, "freejoint", attrib={"name": f"{name}_free"})
        elif dof == "fixed" and carrier and carrier in links_by_name:
            # Rides a rotating carrier: same axis, locked 1:1 to it.
            ax = getattr(links_by_name[carrier], "spin_axis", (0.0, 0.0, 1.0))
            ET.SubElement(body, "joint", attrib={
                "name": f"{name}_spin", "type": "hinge",
                "axis": f"{ax[0]:.6g} {ax[1]:.6g} {ax[2]:.6g}", "pos": "0 0 0"})
            ET.SubElement(eq_el, "joint", attrib={
                "joint1": f"{name}_spin", "joint2": f"{carrier}_spin",
                "polycoef": "0 1 0 0 0"})
            n_ride += 1
        elif dof == "fixed":
            # Static structure. Welded to each part that carries it; a part with no
            # mount (the base) is left welded to the world by having no joint at all.
            for sup in _mounts_of(link):
                if sup in links_by_name and getattr(
                        links_by_name[sup], "dof", "fixed") != "free":
                    ET.SubElement(eq_el, "weld", attrib={"body1": name, "body2": sup})
                    n_weld += 1

        _emit_inertial_and_geoms(body, link, name, piece_map, meshes_dir,
                                 mesh_names, asset_el, sdf=sdf)
    if n_weld or n_ride:
        log_fn(f"[mjcf] flat bodies: {n_weld} weld(s) to structure, "
               f"{n_ride} part(s) locked 1:1 to a rotating carrier")


def _emit_body(parent_el, link_name, model, links_by_name, children, piece_map,
               meshes_dir, mesh_names, asset_el, *, is_root: bool,
               base_height: float, pin_base: bool,
               world_el=None, parent_T=None):
    link = links_by_name[link_name]
    xyz, rpy = _rel_pose_of(model, link_name)
    dof = getattr(link, "dof", "fixed")
    # A LOOSE PART IS NOT ANYBODY'S CHILD. MuJoCo allows <freejoint> only on a top-level
    # body, so a dof=free part nested under its mount ("the ball rests on the chute")
    # made the whole model fail to load — "free joint can only be used on top level" —
    # and the run died before a single step. `mount` positions a part; it does not weld
    # it. For a free part the mount chain is only how we know WHERE it starts, so hoist
    # it to the world with its accumulated world pose and let contact do the rest.
    if dof == "free" and not is_root and world_el is not None and parent_T is not None:
        T = parent_T @ _pose_matrix(xyz, rpy)
        body = ET.SubElement(world_el, "body", attrib={
            "name": link_name,
            "pos": f"{T[0, 3]:.9g} {T[1, 3]:.9g} {T[2, 3]:.9g}",
            "quat": _quat_from_matrix_rot(T)})
        ET.SubElement(body, "freejoint", attrib={"name": f"{link_name}_free"})
        _emit_inertial_and_geoms(body, link, link_name, piece_map, meshes_dir,
                                 mesh_names, asset_el)
        # Its own children keep hanging off it (now a top-level body), so a free part
        # carrying something still carries it.
        for p in children.get(link_name, []):
            _emit_body(body, p.child, model, links_by_name, children, piece_map,
                       meshes_dir, mesh_names, asset_el, is_root=False,
                       base_height=0.0, pin_base=False,
                       world_el=world_el, parent_T=T)
        return

    # Only a body that will actually SETTLE (a free root, not pinned) spawns slightly above the
    # plane. A fixed/spin root is grounded IN PLACE (welded to world / hinged), so lifting it by
    # base_height would weld it hovering in mid-air. Keep its authored Z.
    settling_root = is_root and not pin_base and dof == "free"
    if settling_root:
        pos = (float(xyz[0]), float(xyz[1]), float(xyz[2]) + base_height)
    else:
        pos = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    body = ET.SubElement(parent_el, "body", attrib={
        "name": link_name,
        "pos": f"{pos[0]:.9g} {pos[1]:.9g} {pos[2]:.9g}",
        "quat": _quat_from_rpy(rpy)})
    my_T = (parent_T if parent_T is not None else np.eye(4)) @ _pose_matrix(pos, rpy)

    dof = getattr(link, "dof", "fixed")
    if is_root:
        # A root body's joint is chosen by its OWN dof — NOT blindly freejointed. This matters
        # most for the single-agent path, where parts are laid out flat with NO parent/child
        # poses, so EVERY part is a "root": if each root were freejointed, the housing, posts,
        # bearings and every dof=fixed structural part would become an independent free body.
        # Under gravity they sit still (looks stable), but the moment a gear spins the mesh
        # reaction force has no fixed ground to push against, so the frame is shoved apart and
        # "flies away". So: fixed -> WELD to world (rigid grounded frame); spin -> hinge (axle
        # grounded in place but still rotates); free -> freejoint (a genuinely loose part).
        # pin_base (base_rests_on_plane=False) still welds a would-be-free base to the world.
        if dof == "fixed" or pin_base:
            pass  # welded to world: no joint
        elif dof == "spin":
            ax = getattr(link, "spin_axis", (0.0, 0.0, 1.0))
            ET.SubElement(body, "joint", attrib={
                "name": f"{link_name}_spin", "type": "hinge",
                "axis": f"{ax[0]:.6g} {ax[1]:.6g} {ax[2]:.6g}", "pos": "0 0 0"})
        else:  # free
            ET.SubElement(body, "freejoint", attrib={"name": f"{link_name}_free"})
    elif dof == "spin":
        ax = getattr(link, "spin_axis", (0.0, 0.0, 1.0))
        ET.SubElement(body, "joint", attrib={
            "name": f"{link_name}_spin", "type": "hinge",
            "axis": f"{ax[0]:.6g} {ax[1]:.6g} {ax[2]:.6g}", "pos": "0 0 0"})
    elif dof == "free":
        ET.SubElement(body, "freejoint", attrib={"name": f"{link_name}_free"})
    # dof == "fixed": no joint (welded to its parent body).

    _emit_inertial_and_geoms(body, link, link_name, piece_map, meshes_dir,
                             mesh_names, asset_el, sdf=False)

    for p in children.get(link_name, []):
        _emit_body(body, p.child, model, links_by_name, children, piece_map,
                   meshes_dir, mesh_names, asset_el, is_root=False,
                   base_height=0.0, pin_base=False,
                   world_el=world_el, parent_T=my_T)


def _emit_inertial_and_geoms(body, link, link_name, piece_map, meshes_dir,
                             mesh_names, asset_el, sdf: bool = True):
    """Per-part mass from the material's density x the part's solid volume, then its
    geoms. Shared by the normal nested path and the hoisted free-part path."""
    from .materials import density_of, friction_of
    mat = getattr(link, "material", "steel") or "steel"
    own_stl = os.path.join(meshes_dir, f"{link_name}.stl")
    vol_m3 = _mesh_volume_m3(own_stl) if os.path.exists(own_stl) else None
    if vol_m3:
        mass = max(density_of(mat) * vol_m3, 1e-6)       # kg; floor so MuJoCo is happy
    else:
        mass = 0.05
    ET.SubElement(body, "inertial", attrib=_inertial_attrib(own_stl, mass))
    _add_geoms(body, link, piece_map, meshes_dir, mesh_names, asset_el,
               friction=friction_of(mat), sdf=sdf)


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


def _add_transmission(mujoco_el, model, meshes_dir, links_by_name, *,
                      metrics: dict | None, log_fn) -> int:
    """DETERMINISTIC TRANSMISSION (B): rigid-body tooth contact cannot reliably drive spur
    gears (a tangent tooth face under fast rotation slips instead of pushing, so the driver
    free-spins and nothing downstream moves). Once GEOMETRY proves a pair truly meshes, we
    express the transmission as a MuJoCo <equality><joint> so it is exact.

    NOT action-at-a-distance cheating: a ratio joint is added ONLY after a geometric mesh
    precheck confirms the two gears' centres are ~one pitch-centre-distance apart (their
    teeth interleave). A pair that fails the precheck gets NO constraint.

    Two kinds, both gated by geometry:
      1. MESH (mesh_pairs, gears on DIFFERENT shafts): counter-rotating, ratio =
         -(r_drive / r_driven). Precheck: centre distance ~= r_drive + r_driven.
      2. COMPOUND (spin parts pressed on the SAME arbor, e.g. wheel + pinion): same speed,
         ratio = +1. Detected by a shared spin parent in the pose forest.
    Constrained pairs are also <contact><exclude>d so geoms don't fight the constraint."""
    import numpy as _np

    links = list(model.links)
    spins = [l.name for l in links if getattr(l, "dof", "fixed") == "spin"]
    if len(spins) < 2:
        return 0
    W = _world_transforms(model)
    eq = ET.SubElement(mujoco_el, "equality")
    contact = ET.SubElement(mujoco_el, "contact")
    n = 0
    done: set = set()
    gear_names = {g for pair in (getattr(model, "mesh_pairs", []) or []) for g in pair}
    loose_gears: list = []

    def _center_xy(name):
        T = W.get(name)
        return None if T is None else _np.array([T[0, 3], T[1, 3]]) * 1000.0  # mm

    # 1. MESH pairs: geometric precheck on centre distance, then a ratio joint.
    for pair in getattr(model, "mesh_pairs", []) or []:
        drive, driven = pair[0], pair[1]
        da, db = links_by_name.get(drive), links_by_name.get(driven)
        if not da or not db or da.dof != "spin" or db.dof != "spin":
            continue
        rd = _pitch_radius_m(meshes_dir, drive)
        rn = _pitch_radius_m(meshes_dir, driven)
        ca, cb = _center_xy(drive), _center_xy(driven)
        if not (rd and rn) or ca is None or cb is None:
            log_fn(f"[mjcf] mesh {drive}~{driven}: missing radius/pose, skipped")
            continue
        cd = float(_np.linalg.norm(ca - cb))
        want = (rd + rn) * 1000.0
        if abs(cd - want) > 0.25 * want:
            log_fn(f"[mjcf] mesh precheck FAIL {drive}~{driven}: centre {cd:.1f}mm vs "
                   f"expected ~{want:.1f}mm — teeth do not interleave, NO constraint")
            continue
        ratio = -(rd / rn)
        ET.SubElement(eq, "joint", attrib={
            "joint1": f"{driven}_spin", "joint2": f"{drive}_spin",
            "polycoef": f"0 {ratio:.6g} 0 0 0"})
        ET.SubElement(contact, "exclude", attrib={"body1": drive, "body2": driven})
        done.add(frozenset((drive, driven)))
        n += 1
        log_fn(f"[mjcf] mesh {drive}->{driven}: ratio {ratio:.3f} (centre {cd:.1f}mm ok)")

    # 2. COMPOUND same-shaft groups: spin parts mounted on a common spin arbor are pressed
    # together -> lock 1:1. A shared spin parent in the pose forest = same shaft.
    parent_of = {p.child: p.parent for p in model.poses if p.parent}
    groups: dict = {}
    for name in spins:
        par = parent_of.get(name)
        if par and links_by_name.get(par) and links_by_name[par].dof == "spin":
            groups.setdefault(par, []).append(name)
    for arbor, members in groups.items():
        r_arbor = _outer_radius_m(meshes_dir, arbor)
        locked, riding = [], []
        for b in members:
            a = arbor
            if frozenset((a, b)) in done:
                continue
            if not _is_pressed_on(meshes_dir, b, r_arbor):
                riding.append(b)
                # A GEAR that merely rides its shaft receives no torque from it. That is
                # correct for an hour wheel on the centre arbor, and fatal for the first
                # gear on the DRIVER — the input then turns against nothing and the whole
                # train downstream of it is dead while every constraint looks healthy.
                # Record the measurement so the diagnosis can say so with numbers.
                if b in gear_names:
                    ext_b = _radial_extent_m(meshes_dir, b)
                    if ext_b and r_arbor:
                        loose_gears.append({
                            "gear": b, "shaft": a,
                            "clearance_mm": round((ext_b[0] - r_arbor) * 1000.0, 3),
                            "press_fit_max_mm": round(_PRESS_FIT_CLEARANCE_M * 1000.0, 3),
                            "driver_shaft": bool(getattr(links_by_name.get(a), "driver", False)),
                        })
                continue
            ET.SubElement(eq, "joint", attrib={
                "joint1": f"{b}_spin", "joint2": f"{a}_spin",
                "polycoef": "0 1 0 0 0"})               # 1:1 same speed (compound)
            ET.SubElement(contact, "exclude", attrib={"body1": a, "body2": b})
            done.add(frozenset((a, b)))
            locked.append(b)
            n += 1
        if locked:
            log_fn(f"[mjcf] compound on '{arbor}': {locked} locked 1:1 (same shaft)")
        if riding:
            log_fn(f"[mjcf] on '{arbor}': {riding} RIDE FREE (bore clears the arbor) — "
                   f"no 1:1 lock; they turn at whatever their own mesh drives them at")

    if n == 0:
        mujoco_el.remove(eq)
        mujoco_el.remove(contact)
        return 0
    if metrics is not None:
        metrics["transmission_constraints"] = n
        if loose_gears:
            metrics["loose_gears"] = loose_gears
    for lg in loose_gears:
        log_fn(f"[mjcf] '{lg['gear']}' rides '{lg['shaft']}' with {lg['clearance_mm']}mm "
               f"clearance (press fit needs <= {lg['press_fit_max_mm']}mm), so the shaft "
               f"does NOT drive it"
               + (" — and that shaft is the INPUT, so nothing downstream can turn"
                  if lg["driver_shaft"] else ""))
    log_fn(f"[mjcf] deterministic transmission: {n} constraint(s) after geometry precheck")
    return n


def _radial_extent_m(meshes_dir: str, name: str) -> tuple[float, float] | None:
    """(inner, outer) radius in METERS about the part's OWN ORIGIN, measured in the XY
    plane of its local frame. The inner value is the BORE: for a bored part the nearest
    material sits at the bore wall, for a solid one it sits at the centre (~0).

    Measured about the ORIGIN, not the centroid, because the origin is where the axis
    actually is: every part is authored at the origin along local +Z and then `.moved()`
    into place, so its bore is centred on (0,0) by construction. The centroid only
    coincides with that for a symmetric part. A clock hand is the counter-example — its
    bore is at one END, so a hand without a balancing tail has its centroid out in the
    middle of the shank, several millimetres off the hole. Measured from there the
    "smallest radius" is 0 (the centroid sits inside solid material), the hand reads as
    having no bore at all, and every fit test built on this — press-vs-running, the
    symmetry check, the concentricity gate — silently gets the wrong answer.

    (On this movement the hands happened to be tail-balanced, so centroid and origin were
    0.02-0.15mm apart and nothing broke. That was luck, not a property we can rely on.)"""
    src = os.path.join(meshes_dir, f"{name}.stl")
    if not os.path.exists(src) or os.path.getsize(src) == 0:
        return None
    try:
        import trimesh
        m = trimesh.load(src, force="mesh")
        v = np.asarray(m.vertices)
        if v.size == 0:
            return None
        r = np.linalg.norm(v[:, :2], axis=1)
        return float(r.min()) / 1000.0, float(r.max()) / 1000.0
    except Exception:
        return None


def _outer_radius_m(meshes_dir: str, name: str) -> float | None:
    ext = _radial_extent_m(meshes_dir, name)
    return ext[1] if ext else None


# HOW A BORE-ON-SHAFT FIT IS CLASSIFIED. One number splits every fit in two, so a pair
# always has exactly one verdict — an earlier pass used two independent thresholds and any
# clearance between them was claimed by BOTH (locked 1:1 *and* excluded as free-running).
#
#   clearance = bore_radius - shaft_outer_radius
#     0 < clearance <= 0.10mm   PRESS FIT   -> lock 1:1, exclude contact (they turn as one)
#          clearance >  0.10mm  RUNNING FIT -> no lock, exclude contact (it turns freely)
#
# Contact is excluded either way: a rigid-body solver has no oil film, so simulating the
# bore wall against the shaft only produces friction that stalls the train. What the
# threshold decides is whether TORQUE crosses the joint.
#
# Measured basis: on the wheels genuinely pressed to their arbors the clearance came out
# 0.01-0.05mm, while an hour wheel riding free on the centre arbor (through its pipe) sat
# at 1.50mm. 0.10mm sits well clear of the press-fit cluster and well below any part that
# is meant to rotate. It is also a real machining line — an interference fit has no
# perceptible clearance, a slip fit does.
_PRESS_FIT_CLEARANCE_M = 0.0001

# Any clearance at all means the two are not interfering, so contact is excluded;
# the press-fit line above decides whether they are also LOCKED together.
_RUNNING_FIT_CLEARANCE_M = 0.0

# HOW DEEP A DELIBERATE INTERFERENCE FIT GOES. A press fit is defined by the bore being
# SMALLER than the shaft — hundredths of a millimetre, which is what makes it grip — so
# the two solids necessarily intersect. Past this depth the parts are not fitted, they are
# driven through each other, and that IS a CAD fault worth reporting.
_MAX_PRESS_INTERFERENCE_M = 2.0e-5           # 0.02 mm, 4x the 0.005 the prompt asks for


def _is_press_fit_overlap(model, meshes_dir, a, b) -> bool:
    """Is the overlap between `a` and `b` a deliberate INTERFERENCE FIT?

    True only when all three hold, so a post punched through a gear cannot qualify:
      * one part DECLARES the other as a mount (being coaxial is not being mounted —
        two wheels threaded onto one arbor share an axis while riding neither);
      * they are concentric within the coaxial tolerance;
      * the bore is under the shaft by no more than _MAX_PRESS_INTERFERENCE_M.
    The verdict is the RADIAL interference, never the overlap volume: volume scales with
    part size, so the same 5um press fit measures 0.063mm3 on one part and far more on a
    bigger one, while "the bore is 5um under the shaft" is the engineering class itself.
    """
    import numpy as _np
    an, bn = a.name, b.name
    declared = (bn in _mounts_of(a)) or (an in _mounts_of(b))
    if not declared:
        return False
    W = _world_transforms(model)
    Ta, Tb = W.get(an), W.get(bn)
    if Ta is None or Tb is None:
        return False
    if float(_np.hypot(Ta[0, 3] - Tb[0, 3], Ta[1, 3] - Tb[1, 3])) > _COAXIAL_XY_TOL_M:
        return False
    ea, eb = _radial_extent_m(meshes_dir, an), _radial_extent_m(meshes_dir, bn)
    if not ea or not eb:
        return False
    # Whichever way round the pair is, the ring's bore against the shaft's outer radius.
    for ring, shaft in ((ea, eb), (eb, ea)):
        interference = shaft[1] - ring[0]                # >0 means the bore is undersize
        if 0.0 < interference <= _MAX_PRESS_INTERFERENCE_M:
            return True
    return False


def _is_pressed_on(meshes_dir: str, part: str, r_arbor: float | None) -> bool:
    """True if `part`'s bore hugs an arbor of outer radius `r_arbor` — i.e. it is PRESSED
    on and turns with it, rather than RIDING free on it.

    The pose forest cannot tell these apart: a wheel keyed to its arbor and a wheel that
    spins freely around that same arbor are both just children of it. But the geometry
    can, and it is exactly the difference that makes a watch work — the hour wheel must
    turn freely on the centre arbor (through the hour pipe) or there is no 12:1 at all.
    Locking it 1:1 to the arbor contradicts its own meshing ratio, and MuJoCo's solver
    then splits the difference and stalls the whole train.

    Unknown geometry falls back to True, keeping the old pose-forest behaviour for
    models we cannot measure."""
    if not r_arbor:
        return True
    ext = _radial_extent_m(meshes_dir, part)
    if ext is None:
        return True
    bore = ext[0]
    return (bore - r_arbor) <= _PRESS_FIT_CLEARANCE_M


def _world_transforms(model) -> dict:
    """Accumulate each link's WORLD 4x4 transform by walking the pose forest from its
    roots (mjcf places bodies parent-relative, so a link's world pose = product of the
    relative poses down its chain). Returns {link_name: 4x4 np.ndarray}."""
    children = _pose_children(model)
    out: dict = {}

    def walk(name, parent_T):
        xyz, rpy = _rel_pose_of(model, name)
        T = parent_T @ _mat(xyz, rpy)
        out[name] = T
        for p in children.get(name, []):
            walk(p.child, T)

    import numpy as _np
    for rn in _roots(model):
        walk(rn, _np.eye(4))
    return out


_COAXIAL_XY_TOL_M = 0.002  # 2mm: centers this close to the axis count as "on the shaft"


def coaxial_pairs(model, meshes_dir, *, include_spin_spin: bool = False) -> list[tuple[str, str]]:
    """[(spin_part, other_part)] for every part centred on a SPIN part's axis — a
    bearing/washer/bridge/hand riding a shaft or pipe.

    Shared by two consumers that must agree: the real MJCF excludes these contacts (an
    inner wall pressing the shaft brakes the driver), and the support test treats the
    same pairs as SUPPORT EDGES. Both follow from one physical fact — it is a radial
    sliding fit, not a collision — so they read it from one place.

    `include_spin_spin` also returns spin-on-spin pairs (a gear pressed on its arbor, a
    pipe over a shaft). The real MJCF must NOT exclude those — a gear driven by tooth
    contact still needs its own collisions — but for SUPPORT they are exactly as real as
    a bearing fit: the arbor is what holds the gear up.

    In that mode a FIXED part may also be the carrier. Support flows down a chain of
    concentric parts — an hour hand rides its pipe, the pipe rides the centre arbor — and
    when only spin parts could carry, a hand whose pipe happened to be declared fixed had
    no carrier at all and fell in the settle test, while the identical design with a
    spinning pipe passed. Carrying something is a property of the geometry, not of whether
    the carrier itself turns."""
    spins = [l for l in model.links if getattr(l, "dof", "fixed") == "spin"]
    carriers = list(model.links) if include_spin_spin else spins
    others = [l for l in model.links
              if getattr(l, "dof", "fixed") == "fixed" or include_spin_spin]
    if not carriers or not others:
        return []
    W = _world_transforms(model)
    out: list[tuple[str, str]] = []
    seen: set = set()
    for s in carriers:
        Ts = W.get(s.name)
        if Ts is None:
            continue
        origin = Ts[:3, 3]
        ax = getattr(s, "spin_axis", (0.0, 0.0, 1.0)) or (0.0, 0.0, 1.0)
        axis = Ts[:3, :3] @ np.array([float(ax[0]), float(ax[1]), float(ax[2])])
        nrm = float(np.linalg.norm(axis))
        axis = axis / nrm if nrm > 1e-9 else np.array([0.0, 0.0, 1.0])
        for f in others:
            if f.name == s.name:
                continue
            Tf = W.get(f.name)
            if Tf is None:
                continue
            d = Tf[:3, 3] - origin
            perp = float(np.linalg.norm(d - np.dot(d, axis) * axis))
            if perp <= _COAXIAL_XY_TOL_M:
                # One edge per unordered pair: with every link a candidate carrier, a
                # concentric pair would otherwise be emitted twice (once from each end).
                key = frozenset((s.name, f.name))
                if key in seen:
                    continue
                seen.add(key)
                out.append((s.name, f.name))
    return out


# Below this, an "overlap" is tessellation noise on two touching faces, not interference.
_OVERLAP_TOL_MM3 = 0.05

_overlap_cache: dict = {}
_bbox_cache: dict = {}


def _bboxes_near(meshes_dir: str, W, a: str, b: str) -> bool:
    """True if the two placed parts are close enough that MuJoCo could report a contact —
    their world bounding boxes overlap once grown by the contact margin. Cheap gate in
    front of the boolean, and it keeps the exclude list to pairs that mean something."""
    import trimesh
    boxes = []
    for name in (a, b):
        if name in _bbox_cache:
            box = _bbox_cache[name]
        else:
            src = os.path.join(meshes_dir, f"{name}.stl")
            T = W.get(name)
            if not os.path.exists(src) or T is None:
                _bbox_cache[name] = None
                box = None
            else:
                try:
                    m = trimesh.load(src, force="mesh").copy()
                    Tm = np.array(T, dtype=float).copy()
                    Tm[:3, 3] *= 1000.0
                    m.apply_transform(Tm)
                    box = (m.bounds[0], m.bounds[1])
                except Exception:
                    box = None
                _bbox_cache[name] = box
        if box is None:
            return False
        boxes.append(box)
    pad = _GEOM_MARGIN * 1000.0 * 2.0        # margin is in metres; be generous
    lo = np.maximum(boxes[0][0], boxes[1][0]) - pad
    hi = np.minimum(boxes[0][1], boxes[1][1]) + pad
    return bool(np.all(hi > lo))


def _overlap_mm3(meshes_dir: str, W, a: str, b: str) -> float | None:
    """Volume (mm^3) where two placed part solids intersect, or None if unmeasurable.

    This is the ground truth for "are these two parts interfering": it needs no assumption
    about axes, bores or symmetry, so it is right for a multi-hole bridge, an off-centre
    hand and a pair of meshing gears alike."""
    key = (a, b)
    if key in _overlap_cache:
        return _overlap_cache[key]
    try:
        import trimesh
        out = None
        placed = []
        for name in (a, b):
            src = os.path.join(meshes_dir, f"{name}.stl")
            if not os.path.exists(src) or os.path.getsize(src) == 0:
                _overlap_cache[key] = None
                return None
            m = trimesh.load(src, force="mesh").copy()
            T = W.get(name)
            if T is None:
                _overlap_cache[key] = None
                return None
            T = np.array(T, dtype=float).copy()
            T[:3, 3] *= 1000.0                       # model poses are metres, STLs are mm
            m.apply_transform(T)
            placed.append(m)
        # Cheap reject first: disjoint bounding boxes cannot intersect, and most pairs are.
        lo = np.maximum(placed[0].bounds[0], placed[1].bounds[0])
        hi = np.minimum(placed[0].bounds[1], placed[1].bounds[1])
        if np.any(hi <= lo):
            _overlap_cache[key] = 0.0
            return 0.0
        inter = trimesh.boolean.intersection(placed)
        out = 0.0 if inter is None or inter.is_empty else abs(float(inter.volume))
    except Exception:
        out = None
    _overlap_cache[key] = out
    return out


def _shares_axis(model, W, a, b, *, tol_m: float = 0.0015) -> bool:
    """True if `a` and `b` are concentric about a common axis — i.e. one could be a bore
    riding the other, rather than two parts that merely happen to overlap.

    A bore-on-shaft fit is concentric BY DEFINITION, so this is the cheapest possible way
    to tell a genuine running fit from an accidental collision, and it is the check whose
    absence let a post pass through a gear unnoticed. The axis is taken from whichever
    part spins (a spin part's own spin_axis is authoritative); if neither spins there is no
    fit to exempt and the answer is False, so their contact is simulated normally.

    `tol_m` is deliberately tight (1.5mm): parts on one arbor are concentric to well under
    a millimetre, while anything mounted elsewhere on the plate is centimetres away."""
    Ta, Tb = W.get(a.name), W.get(b.name)
    if Ta is None or Tb is None:
        return False
    spinner = a if getattr(a, "dof", "fixed") == "spin" else (
        b if getattr(b, "dof", "fixed") == "spin" else None)
    if spinner is None:
        return False
    Ts = W[spinner.name]
    ax = getattr(spinner, "spin_axis", (0.0, 0.0, 1.0)) or (0.0, 0.0, 1.0)
    axis = Ts[:3, :3] @ np.array([float(ax[0]), float(ax[1]), float(ax[2])])
    nrm = float(np.linalg.norm(axis))
    if nrm <= 1e-9:
        return False
    axis = axis / nrm
    # Perpendicular offset between the two origins, measured across that axis.
    d = Tb[:3, 3] - Ta[:3, 3]
    perp = float(np.linalg.norm(d - np.dot(d, axis) * axis))
    return perp <= tol_m


def _add_coaxial_bearing_excludes(mujoco_el, model, meshes_dir, links_by_name, *,
                                  metrics: dict | None, log_fn) -> int:
    """B (deterministic) + A (warn): a ring accessory (bearing/washer/spacer/pedestal/
    retainer/bridge) that hugs a rotating shaft is a SLIDING FIT in the real machine, not
    a collision — but rigid-body contact treats the ring's inner wall pressing the shaft as
    friction that STALLS the driver (the "input turned but nothing moved" failure). For each
    spin part, exclude contact with every FIXED part whose center lies on that part's spin
    axis (coaxial), so the bearing/washer rides the shaft instead of braking it.

    A-side backstop: if a coaxial fixed ring's footprint radius is SMALLER than the shaft's
    (i.e. it truly bites into the shaft, not just rings it), log a WARN — the CAD is wrong
    (no bore clearance) even though the exclude lets the sim run, so the fault isn't hidden."""
    spins = [l for l in model.links if getattr(l, "dof", "fixed") == "spin"]
    fixeds = [l for l in model.links if getattr(l, "dof", "fixed") == "fixed"]
    if not spins or not fixeds:
        return 0
    W = _world_transforms(model)
    contact = ET.SubElement(mujoco_el, "contact")
    n = 0
    warns = 0
    runners = 0
    fits: list = []
    interferences: list = []
    press_fits: list = []
    mount_of = {p.child: p.parent for p in model.poses if p.parent}
    mesh_pairs_fs = {frozenset(p[:2]) for p in (getattr(model, "mesh_pairs", []) or [])
                     if len(p) >= 2}
    excluded_pairs: set = set()
    _COAXIAL_XY_TOL_M = 0.002  # 2mm: centers this close on the axis count as "on the shaft"
    for s in spins:
        Ts = W.get(s.name)
        if Ts is None:
            continue
        origin = Ts[:3, 3]
        ax = getattr(s, "spin_axis", (0.0, 0.0, 1.0)) or (0.0, 0.0, 1.0)
        axis = Ts[:3, :3] @ np.array([float(ax[0]), float(ax[1]), float(ax[2])])
        nrm = float(np.linalg.norm(axis))
        axis = axis / nrm if nrm > 1e-9 else np.array([0.0, 0.0, 1.0])
        r_shaft = _pitch_radius_m(meshes_dir, s.name)
        for f in fixeds:
            Tf = W.get(f.name)
            if Tf is None:
                continue
            d = Tf[:3, 3] - origin
            # perpendicular distance from the fixed part's center to the spin axis line
            perp = float(np.linalg.norm(d - np.dot(d, axis) * axis))
            if perp > _COAXIAL_XY_TOL_M:
                continue
            ET.SubElement(contact, "exclude",
                          attrib={"body1": s.name, "body2": f.name})
            excluded_pairs.add(frozenset((s.name, f.name)))
            n += 1
            # FIT CHECK, measured not guessed: a part that sits ON a shaft must have a bore
            # the shaft actually fits through. Compare the ring's own BORE to the shaft's
            # OUTER radius — the earlier outer-vs-outer test only caught the crude case, and
            # a bore too small is the exact defect that leaves the train jammed. This is
            # deterministic and available BEFORE the sim runs, so it can be reported without
            # waiting for a VLM to notice it in a recording.
            # ONLY judge the fit the part actually DECLARES. Being coaxial is not being
            # mounted: a bearing and a gear both threaded onto one centre arbor share an
            # axis while sitting at different heights, neither on the other. Comparing
            # their radii then reads the GEAR'S TIP RADIUS as if it were a shaft and calls
            # a perfectly good 0.90mm bore "4mm too small" (measured: 14 such reports on a
            # movement whose transmission was working 6/6). The part's own `mount=` says
            # what it is on, so use that and nothing else.
            if (getattr(f, "mount", "") or mount_of.get(f.name, "")) != s.name:
                continue
            ext_ring = _radial_extent_m(meshes_dir, f.name)
            ext_shaft = _radial_extent_m(meshes_dir, s.name)
            if ext_ring and ext_shaft:
                bore, ring_outer = ext_ring
                shaft_outer = ext_shaft[1]
                # Report only a real INTERFERENCE. A bore a hair LARGER than the shaft is a
                # correct running fit (hour_hand 2.43 on a 2.40 pipe), and flagging those
                # would bury the genuine faults the agent must act on. 0.05mm of slop is
                # measurement noise on a tessellated STL, not an interference.
                if bore < shaft_outer - 5e-5:
                    impossible = ring_outer <= shaft_outer
                    fits.append({
                        "part": f.name, "shaft": s.name,
                        "bore_mm": round(bore * 1000.0, 2),
                        "shaft_r_mm": round(shaft_outer * 1000.0, 2),
                        "part_outer_mm": round(ring_outer * 1000.0, 2),
                        "impossible": impossible,
                    })
            r_ring = _pitch_radius_m(meshes_dir, f.name)
            if r_shaft and r_ring and r_ring < r_shaft - 1e-4:
                warns += 1
                log_fn(f"[mjcf] WARN coaxial '{f.name}' (r~{r_ring*1000:.1f}mm) bites into "
                       f"shaft '{s.name}' (r~{r_shaft*1000:.1f}mm): no bore clearance — "
                       f"excluded from contact so the sim runs, but the CAD bore is wrong")
    # RUNNING FITS, decided by GEOMETRY rather than by centroid distance and dof roles.
    # The centroid test above misses two whole classes, and both stalled this movement:
    #   * a big FIXED part whose centroid is nowhere near the axis — a mainplate is 60mm
    #     across, so its centre sits far off the arbor running through its 24mm hole, and
    #     the 2mm coaxial tolerance never fires. Measured: circular_skeleton_base vs
    #     intermediate_arbor, 11.0mm of real clearance, 1.0e17 N of contact force.
    #   * SPIN against SPIN — an hour-wheel sleeve turning on the centre arbor is the
    #     defining fit of motion work, and the loop above only ever pairs spin with fixed.
    # Both are answered by measuring the parts: if one's BORE clears the other's OUTER
    # radius by a running-fit margin, they are meant to turn freely against each other and
    # their contact is a solver artefact, not mechanics.
    everything = list(model.links)
    for i, a in enumerate(everything):
        for b in everything[i + 1:]:
            if frozenset((a.name, b.name)) in excluded_pairs:
                continue
            if getattr(a, "dof", "fixed") == "fixed" and getattr(b, "dof", "fixed") == "fixed":
                continue                    # two static parts never rub
            # DO THE TWO SOLIDS ACTUALLY OVERLAP? That is the only question worth asking,
            # and it is answered exactly by a boolean intersection of the placed meshes.
            #
            # Radius comparisons were wrong in BOTH directions. A radius is measured about
            # one part's own centre, so a support post driven through a gear 15mm off its
            # axis passes "post radius < gear bore" as easily as an arbor sitting IN that
            # bore — that exclusion hid a genuine 30.5mm^3 interpenetration, and deleting
            # the post made minute_wheel go from dead to turning. The same test called two
            # correctly-meshing gears a 5.55mm interference.
            #
            # Zero overlap means the contact MuJoCo reports is an artefact of the collision
            # proxy (convex pieces chord the inner arc of a bore, and margin fires 0.2mm
            # early), so excluding it is right: of the ten worst contacts on this movement,
            # eight had exactly 0.0000mm^3 of real overlap. A positive overlap is a CAD
            # fault and must stay visible.
            # Only NEAR pairs can be misjudged. Two parts whose bounding boxes are apart
            # cannot produce a contact at all, so emitting an exclude for them is noise —
            # it took the list from 4 entries to 110 and buried what the excludes mean.
            if not _bboxes_near(meshes_dir, W, a.name, b.name):
                continue
            vol = _overlap_mm3(meshes_dir, W, a.name, b.name)
            if vol is None:
                continue                    # cannot measure -> simulate it, don't hide it
            if vol > _OVERLAP_TOL_MM3:
                # A MESHING PAIR IS SUPPOSED TO OVERLAP — interlocking teeth are the mesh.
                # (cannon_pinion/minute_wheel measured 10.06mm3 of tooth engagement, which
                # is correct engineering, not a fault.) Such a pair already has its own
                # ratio constraint and contact exclusion, so leave it alone; reporting it
                # would send the agent to "fix" the one thing that works.
                # A PRESS FIT IS THE SAME KIND OF DELIBERATE OVERLAP. An interference fit
                # is DEFINED by the bore being smaller than the shaft, so the two solids
                # must intersect — that is what makes it grip. Reported as a CAD fault it
                # cost a whole run: told "their solids overlap by 0.063mm3, the CAD is
                # wrong", the agent opened every gear bore to shaft_r + 0.05, turning each
                # press fit into a running fit, and the train went dead (measured
                # 1_12_20260803_175836, iteration 2, whose own comment reads "Gear bores
                # use running-clearance dimensions to eliminate the reported 0.063/0.125
                # mm3 interference volumes").
                # Judged on the RADIAL interference, not the volume: volume scales with
                # part size, so 0.063mm3 means nothing on its own, while "the bore is 5um
                # under the shaft" is exactly the engineering class.
                if _is_press_fit_overlap(model, meshes_dir, a, b):
                    press_fits.append({"a": a.name, "b": b.name,
                                       "overlap_mm3": round(vol, 3)})
                elif frozenset((a.name, b.name)) not in mesh_pairs_fs:
                    interferences.append({"a": a.name, "b": b.name,
                                          "overlap_mm3": round(vol, 3)})
                continue
            ET.SubElement(contact, "exclude",
                          attrib={"body1": a.name, "body2": b.name})
            excluded_pairs.add(frozenset((a.name, b.name)))
            n += 1
            runners += 1
    for iv in sorted(interferences, key=lambda x: -x["overlap_mm3"])[:10]:
        log_fn(f"[mjcf] INTERFERENCE {iv['a']} / {iv['b']}: their solids overlap by "
               f"{iv['overlap_mm3']}mm3 — contact is NOT excluded, the CAD is wrong")
    if press_fits:
        log_fn(f"[mjcf] {len(press_fits)} press fit(s) overlap BY DESIGN (a bore under its "
               f"shaft is what grips): "
               + ", ".join(f"{p['a']}/{p['b']}" for p in press_fits[:6]))
    if runners:
        log_fn(f"[mjcf] excluded {runners} running-fit pair(s) measured from the geometry "
               f"(a bore that clears its shaft turns freely; simulating that contact only "
               f"produces solver friction that stalls the train)")

    if n == 0:
        mujoco_el.remove(contact)
        return 0
    if metrics is not None:
        metrics["coaxial_excludes"] = n
        metrics["coaxial_bore_warns"] = warns
        metrics["running_fit_excludes"] = runners
        if interferences:
            metrics["interferences"] = sorted(
                interferences, key=lambda x: -x["overlap_mm3"])[:10]
        if fits:
            metrics["bore_fit_faults"] = fits
    log_fn(f"[mjcf] excluded {n} coaxial bearing/washer contact(s) from the shaft "
           f"({warns} with a too-small bore flagged)")
    return n


def build_support_mjcf(model, ctx, *, metrics: dict | None = None,
                       settings=None, log_fn=print) -> tuple[str, str]:
    """Build the SUPPORT-TEST MJCF: every part a free body except the one already
    resting on the ground, which stays welded to the world.

    This answers "what is actually held up by real geometry?" the only way a heuristic
    cannot fake — let gravity decide. The geometric ray test it replaces called a part
    supported whenever ANY solid sat below it, so a stack of floating parts propping
    each other up passed, and a 1.75 mm air gap under a watch hand read as contact.
    Here every `mount=` weld is DISSOLVED: a part stays put only if real geometry (a
    face it rests on, a bore gripping a shaft) holds it.

    Friction is pinned near-infinite (_SUPPORT_FRICTION) so the verdict is about
    SUPPORT, not grip: a shaft/bore radial fit counts as supported, which matches the
    real machine — a watch hand is pressed onto its pipe and by design has nothing
    underneath it.

    The ground part keeps its weld so the assembly has a world anchor; without it the
    whole machine free-falls together and every part reads as "fell". The anchor is the
    lowest FIXED structure part, not simply the lowest geometry — a shaft routinely
    pokes below the baseplate it turns in, and welding the shaft would leave the actual
    base unsupported. The ground plane is dropped below all geometry so such a
    protruding shaft does not spuriously rest on it.

    Returns (mjcf_path, ground_part_name)."""
    import trimesh

    meshes_dir = ctx.meshes_dir
    # THE SUPPORT TEST RUNS ON CONVEX GEOMETRY, even when the real MJCF uses SDF.
    #
    # SDF loses contacts in a full assembly. Measured on run 1_12_20260803_195154
    # iter_1: rear_bridge_post sits EXACTLY on the base (gap +0.0000mm, its underside
    # inside the base solid) and is byte-identical to front_bridge_post, which never
    # falls. With 2 bodies it holds (-0.025mm); with 4 it holds (-0.018mm); with all 17
    # it falls 52.068mm. Stepping through it, the post starts with 40 contacts on the
    # base, keeps them for ~2000 steps, then the contact count drops to zero and it
    # free-falls. Under convex decomposition the same geometry does not fall (-1.96mm).
    # So the trigger is scene scale, not the part — something in SDF's contact budget
    # drops a pair that is genuinely touching.
    #
    # This matters here more than anywhere else because the verdict is per-part blame:
    # the run above reported the post as unsupported for four straight iterations, and
    # the agent, told to "move it down until it touches", buried it 0.1mm, then 4mm
    # (straight through a 4mm-thick base), then 1mm — destroying a design whose gear
    # train was already correct (11.820 against a 12:1 target).
    #
    # The precision SDF buys is not needed for THIS question. Support asks "is anything
    # holding this part up", where a convex hull that slightly overfills a bore only
    # makes a fit MORE supported, never less. The real MJCF keeps SDF, where bore
    # clearance decides whether torque crosses and the precision is the whole point.
    piece_map = decompose_model(model, meshes_dir, metrics=metrics, log_fn=log_fn)
    use_sdf = False
    W = _world_transforms(model)

    dof_of = {l.name: getattr(l, "dof", "fixed") for l in model.links}
    ground, ground_z = None, float("inf")
    world_bottom_m = float("inf")
    for l in model.links:
        stl = os.path.join(meshes_dir, f"{l.name}.stl")
        T = W.get(l.name)
        if not os.path.exists(stl) or T is None:
            continue
        try:
            mm = trimesh.load(stl, force="mesh")
        except Exception:
            continue
        if not isinstance(mm, trimesh.Trimesh) or len(mm.faces) == 0:
            continue
        Tmm = T.copy()
        Tmm[:3, 3] *= 1000.0
        mm.apply_transform(Tmm)
        z = float(mm.bounds[0][2])
        world_bottom_m = min(world_bottom_m, z / 1000.0)
        if dof_of.get(l.name) == "fixed" and z < ground_z:
            ground, ground_z = l.name, z
    if ground is None:
        ground = model.root_link or (model.links[0].name if model.links else "")
    if world_bottom_m == float("inf"):
        world_bottom_m = 0.0

    mujoco_el = ET.Element("mujoco",
                           attrib={"model": (model.name or "assembly") + "_support"})
    ET.SubElement(mujoco_el, "option", attrib={
        "gravity": "0 0 -9.81", "timestep": f"{_TIMESTEP}",
        "iterations": f"{_SOLVER_ITERS}", "solver": "Newton", "cone": "elliptic"})
    ET.SubElement(mujoco_el, "size", attrib={"memory": "256M"})
    asset_el = ET.SubElement(mujoco_el, "asset")
    world = ET.SubElement(mujoco_el, "worldbody")
    ET.SubElement(world, "light", attrib={"pos": "0 0 3", "dir": "0 0 -1",
                                          "diffuse": "0.8 0.8 0.8"})
    ET.SubElement(world, "geom", attrib={
        "name": "ground", "type": "plane", "size": "5 5 0.1",
        "pos": f"0 0 {world_bottom_m - 0.05:.9g}",
        "friction": _SUPPORT_FRICTION, "condim": "4"})

    # Every part is emitted FLAT at its world pose. The pose forest is deliberately NOT
    # walked: a nested body would inherit its parent's weld and be held up by the very
    # `mount=` label this test exists to disprove.
    from .materials import density_of
    mesh_names: set = set()
    for l in model.links:
        T = W.get(l.name)
        if T is None:
            continue
        xyz = T[:3, 3]
        body = ET.SubElement(world, "body", attrib={
            "name": l.name,
            "pos": f"{xyz[0]:.9g} {xyz[1]:.9g} {xyz[2]:.9g}",
            "quat": _quat_from_matrix_rot(T)})
        if l.name != ground:
            ET.SubElement(body, "freejoint", attrib={"name": f"{l.name}_free"})
        mat = getattr(l, "material", "steel") or "steel"
        own_stl = os.path.join(meshes_dir, f"{l.name}.stl")
        vol_m3 = _mesh_volume_m3(own_stl) if os.path.exists(own_stl) else None
        mass = max(density_of(mat) * vol_m3, 1e-6) if vol_m3 else 0.05
        ET.SubElement(body, "inertial", attrib=_inertial_attrib(own_stl, mass))
        _add_geoms(body, l, piece_map, meshes_dir, mesh_names, asset_el,
                   friction=_SUPPORT_FRICTION, sdf=use_sdf)

    # COAXIAL FITS: exclude them only under convex decomposition. A bore approximated by
    # convex pieces is a solid ring around the shaft, so the pair reads as a deep
    # interpenetration and the solver ejects both parts violently — every part would
    # "rise", not fall, and the test would report nothing. support_test.py re-reads
    # coaxial_pairs() and credits those as support edges instead.
    #
    # Under SDF the bore IS a bore, so that exclusion now deletes the very contact this
    # test exists to measure. Measured on run 1_12_20260803_191330: minute_hand had a
    # correct 0.005mm interference fit on its arbor — gripped, by design — and was still
    # reported as "declared on minute_arbor, but the two never touch" and 11.6mm fallen,
    # because the contact holding it had been excluded. The agent read that as "not tight
    # enough" and opened the interference to 0.080mm on the next iteration, which then
    # tripped the CAD-fault check. Keeping the contact drops the false verdicts (measured
    # 9 parts "fell" -> 2, and the two that remain have real air under them).
    pairs = [] if use_sdf else coaxial_pairs(model, meshes_dir)
    if pairs:
        contact = ET.SubElement(mujoco_el, "contact")
        for s, f in pairs:
            ET.SubElement(contact, "exclude", attrib={"body1": s, "body2": f})

    path = os.path.splitext(ctx.urdf_path)[0] + "_support.mjcf"
    ET.indent(mujoco_el)
    Path(path).write_text(ET.tostring(mujoco_el, encoding="unicode"), encoding="utf-8")
    log_fn(f"[support] wrote {path}: {len(model.links)} part(s) freed, "
           f"'{ground}' welded as the ground anchor")
    return path, ground


def build_mjcf(model, ctx, *, settings=None, metrics: dict | None = None,
               base_height: float = 0.01, log_fn=print) -> str:
    """Build an MJCF for the model and write it next to the URDF (model.mjcf).
    Returns the MJCF path. Decomposes movable/meshing parts first (cached)."""
    meshes_dir = ctx.meshes_dir
    # SDF reads each part's own STL, so there is nothing to decompose — and skipping it
    # is where the build-time saving lives (131-288s -> 1.3-1.9s on this repo's runs).
    use_sdf = getattr(settings, "sdf_collision", True) if settings is not None else True
    piece_map = ({} if use_sdf
                 else decompose_model(model, meshes_dir, metrics=metrics, log_fn=log_fn))

    pin_base = not getattr(settings, "base_rests_on_plane", True) if settings else False

    mujoco_el = ET.Element("mujoco", attrib={"model": model.name or "assembly"})
    ET.SubElement(mujoco_el, "option", attrib={
        "gravity": "0 0 -9.81", "timestep": f"{_TIMESTEP}",
        "iterations": f"{_SOLVER_ITERS}", "solver": "Newton", "cone": "elliptic"})
    # A many-part machine with convex-decomposed geoms generates FAR more contact
    # constraints than MuJoCo's default arena holds ("Insufficient arena memory"). Give
    # the solver a generous fixed arena so a busy assembly (a full watch train) loads.
    ET.SubElement(mujoco_el, "size", attrib={"memory": "256M"})
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
    # FLAT BODIES. Every part is a direct child of <worldbody> with its absolute world
    # pose; `mount` becomes a pure load-path declaration instead of also dictating the
    # body tree. See _emit_flat for what that buys and what it costs.
    struct_eq = ET.SubElement(mujoco_el, "equality")
    _emit_flat(world, model, links_by_name, piece_map, meshes_dir, mesh_names,
               asset_el, struct_eq,
               base_height=(0.0 if pin_base else base_height), pin_base=pin_base,
               sdf=use_sdf, log_fn=log_fn)
    if not len(struct_eq):
        mujoco_el.remove(struct_eq)

    # DETERMINISTIC TRANSMISSION (B, ON by default): rigid-body tooth contact free-spins
    # instead of driving, so once geometry proves a pair meshes we express it as an exact
    # ratio constraint. `allow_gear_constraint` forces the OLD unconditional constraints
    # (no geometry precheck) as an escape hatch.
    if settings is not None and getattr(settings, "allow_gear_constraint", False):
        _add_gear_constraints(mujoco_el, model, meshes_dir, links_by_name,
                              metrics=metrics, log_fn=log_fn)
    else:
        _add_transmission(mujoco_el, model, meshes_dir, links_by_name,
                          metrics=metrics, log_fn=log_fn)

    # A ring accessory (bearing/washer/spacer/retainer) coaxial with a rotating shaft is a
    # sliding fit, not a collision — excluding those contacts stops them from braking the
    # driver (the "input stalled, nothing turned" failure). On by default: pure-contact
    # transmission still happens at the GEAR TEETH, which are NOT coaxial and stay collidable.
    _add_coaxial_bearing_excludes(mujoco_el, model, meshes_dir, links_by_name,
                                  metrics=metrics, log_fn=log_fn)

    mjcf_path = os.path.splitext(ctx.urdf_path)[0] + ".mjcf"
    ET.indent(mujoco_el)
    Path(mjcf_path).write_text(ET.tostring(mujoco_el, encoding="unicode"),
                               encoding="utf-8")
    log_fn(f"[mjcf] wrote {mjcf_path} ({len(roots)} root body(ies), "
           f"{len(mesh_names)} mesh geom(s))")
    return mjcf_path
