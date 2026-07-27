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
               friction: str = _GEOM_FRICTION):
    """Attach one <geom> per convex piece (movable/meshing parts) or one for the
    part's own mesh. Registers each referenced <mesh> in <asset> once. A degenerate
    (coplanar) piece that MuJoCo cannot hull is replaced by a thin BOX geom of the same
    bounding size, so a razor-thin part collides approximately instead of crashing the
    model load. ``friction`` is the part's per-material contact triple."""
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
        if _is_hullable(src):
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


def _emit_body(parent_el, link_name, model, links_by_name, children, piece_map,
               meshes_dir, mesh_names, asset_el, *, is_root: bool,
               base_height: float, pin_base: bool):
    link = links_by_name[link_name]
    xyz, rpy = _rel_pose_of(model, link_name)
    dof = getattr(link, "dof", "fixed")
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

    # Per-part mass from the material's density x the part's solid volume; fall back to
    # the old flat placeholder when the STL/volume is unavailable so the model still loads.
    from .materials import density_of, friction_of
    mat = getattr(link, "material", "steel") or "steel"
    own_stl = os.path.join(meshes_dir, f"{link_name}.stl")
    vol_m3 = _mesh_volume_m3(own_stl) if os.path.exists(own_stl) else None
    if vol_m3:
        mass = max(density_of(mat) * vol_m3, 1e-6)       # kg; floor so MuJoCo is happy
        ET.SubElement(body, "inertial", attrib={
            "pos": "0 0 0", "mass": f"{mass:.6g}",
            "diaginertia": "1e-4 1e-4 1e-4"})            # MuJoCo refines from geom
    else:
        ET.SubElement(body, "inertial", attrib={
            "pos": "0 0 0", "mass": "0.05",
            "diaginertia": "1e-4 1e-4 1e-4"})
    _add_geoms(body, link, piece_map, meshes_dir, mesh_names, asset_el,
               friction=friction_of(mat))

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
        for b in members:
            a = arbor
            if frozenset((a, b)) in done:
                continue
            ET.SubElement(eq, "joint", attrib={
                "joint1": f"{b}_spin", "joint2": f"{a}_spin",
                "polycoef": "0 1 0 0 0"})               # 1:1 same speed (compound)
            ET.SubElement(contact, "exclude", attrib={"body1": a, "body2": b})
            done.add(frozenset((a, b)))
            n += 1
        if members:
            log_fn(f"[mjcf] compound on '{arbor}': {members} locked 1:1 (same shaft)")

    if n == 0:
        mujoco_el.remove(eq)
        mujoco_el.remove(contact)
        return 0
    if metrics is not None:
        metrics["transmission_constraints"] = n
    log_fn(f"[mjcf] deterministic transmission: {n} constraint(s) after geometry precheck")
    return n


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
    a bearing fit: the arbor is what holds the gear up."""
    spins = [l for l in model.links if getattr(l, "dof", "fixed") == "spin"]
    others = [l for l in model.links
              if getattr(l, "dof", "fixed") == "fixed" or include_spin_spin]
    if not spins or not others:
        return []
    W = _world_transforms(model)
    out: list[tuple[str, str]] = []
    for s in spins:
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
                out.append((s.name, f.name))
    return out


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
            n += 1
            r_ring = _pitch_radius_m(meshes_dir, f.name)
            if r_shaft and r_ring and r_ring < r_shaft - 1e-4:
                warns += 1
                log_fn(f"[mjcf] WARN coaxial '{f.name}' (r~{r_ring*1000:.1f}mm) bites into "
                       f"shaft '{s.name}' (r~{r_shaft*1000:.1f}mm): no bore clearance — "
                       f"excluded from contact so the sim runs, but the CAD bore is wrong")
    if n == 0:
        mujoco_el.remove(contact)
        return 0
    if metrics is not None:
        metrics["coaxial_excludes"] = n
        metrics["coaxial_bore_warns"] = warns
    log_fn(f"[mjcf] excluded {n} coaxial bearing/washer contact(s) from the shaft "
           f"({warns} with a too-small bore flagged)")
    return n


def build_support_mjcf(model, ctx, *, metrics: dict | None = None,
                       log_fn=print) -> tuple[str, str]:
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
    piece_map = decompose_model(model, meshes_dir, metrics=metrics, log_fn=log_fn)
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
        ET.SubElement(body, "inertial", attrib={
            "pos": "0 0 0", "mass": f"{mass:.6g}", "diaginertia": "1e-4 1e-4 1e-4"})
        _add_geoms(body, l, piece_map, meshes_dir, mesh_names, asset_el,
                   friction=_SUPPORT_FRICTION)

    # The coaxial sliding fits must be excluded here for the SAME reason as in the real
    # MJCF: a bore modelled as a solid ring around a shaft reads as a deep interpenetration,
    # and the solver ejects both parts violently — every part would "rise", not fall, and
    # the test would report nothing. support_test.py re-reads coaxial_pairs() and credits
    # these as support edges instead, which is what makes a hand on its pipe count as held.
    pairs = coaxial_pairs(model, meshes_dir)
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
    piece_map = decompose_model(model, meshes_dir, metrics=metrics, log_fn=log_fn)

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
    for rn in roots:
        if rn not in links_by_name:
            continue
        _emit_body(world, rn, model, links_by_name, children, piece_map,
                   meshes_dir, mesh_names, asset_el, is_root=True,
                   base_height=(0.0 if pin_base else base_height), pin_base=pin_base)

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
