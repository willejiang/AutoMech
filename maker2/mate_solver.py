"""Deterministic compiler: the manager's connection graph (PARTS + MATES) -> KinematicModel.

Part A (see .claude/plans/precious-humming-wand.md). Instead of authoring each part's
absolute pose, the manager emits a CONNECTION GRAPH — typed parts joined by MATES ("part
A's bore mates coaxially to part B's shaft") — and this module SOLVES every part's pose in
closed form, emitting an ordinary (not-yet-validated) KinematicModel. It is a drop-in
sibling of `mjcf_skeleton.mjcf_skeleton_parser`: same input role, same output type, so the
ENTIRE downstream pipeline (manager._validate_model, the gates, assembler, build_mjcf,
physics) is unchanged.

The mate math mirrors AssemCAD's closed-form resolve `T = L_b · R_flip · R_alpha · L_c^-1`
and the boss's already-proven subassembly-level mate compiler `assembler._bridge_pose_from_ports`
(assembler.py:212-254). Ports are usually INFERRED from a part's shape_hint+size_mm
(`infer_ports`); a PortSpec in the IR overrides inference for that one part.

CRITICAL: this produces a DESIGN KinematicModel at each part's true pose. `mjcf_builder.
build_mjcf` remains the sole SIMULATION compiler (CoACD, mm->m, mass, solver tuning). We
reuse `assembler._mat/_decompose/_root_to_link` (the `sxyz` convention build_mjcf/urdf share)
so solved poses round-trip through build_mjcf byte-consistently.
"""
from __future__ import annotations

import json
import math

import numpy as np
import trimesh.transformations as tf

from .assembler import _decompose, _mat, _root_to_link
from .jsonutil import extract_json_object
from .manager import _link_from_dict, _mesh_pairs_from
from .model import KinematicModel, MateSpec, PortSpec, PoseSpec

# Two independent constraint paths that place the same part must agree within this (m);
# reuses the same tolerance the frame/precheck code uses (precheck._POS_TOL_M = 2mm).
_POS_TOL_M = 2e-3
# Meshing-gear axes must be parallel / perpendicular within this (rad) to accept the mate.
_AXIS_TOL_RAD = math.radians(2.0)

# Mate types whose resolver is the coaxial family (align axes, slide, roll, no flip).
_COAXIAL_MATES = {
    "coaxial", "revolute", "cylindrical", "press_fit", "shrink_fit", "pin", "dowel",
    "taper_pin", "spline", "set_screw", "circlip", "retaining_ring", "taper_lock_bushing",
    "journal_bearing", "bushing", "ball_bearing", "thrust_bearing", "key", "keyway",
}
# Mate types whose resolver is the planar/face family (coincide faces, anti-align normals).
_FACE_MATES = {
    "face_to_face", "snap_fit", "snap_to_face", "bolted", "flanged", "welded", "planar",
    "riveted",
}
# Mate types resolved coaxially THEN seated on a face plane.
_COAXIAL_FACE_MATES = {"coaxial_face", "threaded", "thread_engage"}
# Parallel-axis gears / parallel-axis contact transmission (center distance = r_a + r_b).
_PARALLEL_GEAR_MATES = {
    "gear_spur_external", "gear_helical", "gear_spur_internal", "rack_pinion",
    "belt_pulley", "chain_sprocket", "friction_wheel", "gear_mesh",
}
# Perpendicular / intersecting / skew gears (NEW capability; see _resolve_gear).
_ANGLED_GEAR_MATES = {"gear_bevel", "miter", "worm", "gear_crossed_helical", "gear_hypoid"}
# Internal-mesh gears use C = |r_a - r_b| instead of r_a + r_b.
_INTERNAL_GEAR_MATES = {"gear_spur_internal"}
# Tangential CONTACT (no pitch radius): a pawl/click/detent/cam-follower TOUCHES a wheel's
# rim. Positioned tangent to the base part's OUTER radius; the incoming part needs no radius
# (a click is a lever, not a gear). Couples by contact at sim time like a gear does.
_CONTACT_MATES = {"ratchet", "pawl", "click", "detent", "cam_follower", "contact",
                  "escapement", "tangent"}
# Every gear/contact mate auto-registers a mesh_pair for the transmission detector.
_MESH_PAIR_MATES = _PARALLEL_GEAR_MATES | _ANGLED_GEAR_MATES | _CONTACT_MATES


class MateSolveError(ValueError):
    """The connection graph could not be solved into a KinematicModel. A ValueError
    subclass so `manager._decompose_loop`'s existing `except (ValueError, ...)` catches it
    and feeds it back as a repair request, exactly like a JSON-parse error."""


# --------------------------------------------------------------------------- #
# Small linear-algebra helpers (build on trimesh.transformations, like assembler._mat)
# --------------------------------------------------------------------------- #

def _unit(v) -> np.ndarray:
    a = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        raise MateSolveError(f"a port/separation axis is zero-length: {tuple(v)}")
    return a / n


def _frame_from_axis(axis, origin_m) -> np.ndarray:
    """A 4x4 whose +Z maps to `axis` (unit) with translation `origin_m` (meters). The in-plane
    (x/y) orientation is an arbitrary but DETERMINISTIC completion (no Math.random) — mates
    that care about roll set `angle_rad` explicitly. This is a port's local frame L."""
    z = _unit(axis)
    # Pick a reference not parallel to z, then Gram-Schmidt -> a stable right-handed basis.
    ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = _unit(np.cross(ref, z))
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
    T[:3, 3] = np.asarray(origin_m, dtype=float)
    return T


def _rot_about(axis, angle_rad) -> np.ndarray:
    if abs(angle_rad) < 1e-12:
        return np.eye(4)
    return tf.rotation_matrix(float(angle_rad), _unit(axis))


# --------------------------------------------------------------------------- #
# Port inference — a part's connection points from its shape_hint + size_mm
# --------------------------------------------------------------------------- #

def _mm(v):
    try:
        f = float(v)
        return f if f > 0 else None
    except (TypeError, ValueError):
        return None


def _gear_pitch_radius_mm(link) -> float | None:
    """Pitch radius (MM) of a gear link from size_mm: pitch_radius, or module*teeth/2, or a
    pitch/outer diameter. None if not a gear / no usable dims."""
    sz = getattr(link, "size_mm", {}) or {}
    if _mm(sz.get("pitch_radius")):
        return float(sz["pitch_radius"])
    if _mm(sz.get("pitch_radius_mm")):
        return float(sz["pitch_radius_mm"])
    mod, teeth = _mm(sz.get("module")), sz.get("teeth")
    try:
        if mod and teeth and int(teeth) > 0:
            return mod * int(teeth) / 2.0
    except (TypeError, ValueError):
        pass
    for k in ("pitch_dia", "pitch_diameter"):
        if _mm(sz.get(k)):
            return float(sz[k]) / 2.0
    return None


def _radius_mm(link) -> float | None:
    """Outer radius (MM) of a cylindrical part from size_mm (radius, or */2 of a diameter)."""
    sz = getattr(link, "size_mm", {}) or {}
    if _mm(sz.get("radius")):
        return float(sz["radius"])
    for k in ("outer_dia", "diameter", "od", "outer_diameter"):
        if _mm(sz.get(k)):
            return float(sz[k]) / 2.0
    pr = _gear_pitch_radius_mm(link)
    return pr


def _height_mm(link) -> float | None:
    sz = getattr(link, "size_mm", {}) or {}
    for k in ("height", "length", "thickness", "face_width", "depth", "width"):
        if _mm(sz.get(k)):
            return float(sz[k])
    return None


def _bore_dia_mm(link) -> float | None:
    sz = getattr(link, "size_mm", {}) or {}
    for k in ("bore_dia", "bore", "inner_dia", "id", "hole_dia", "bore_diameter"):
        if _mm(sz.get(k)):
            return float(sz[k])
    return None


def _has_box_dims(link) -> bool:
    """True if size_mm carries prismatic x/y/z extents (so an unknown-hint part is a box)."""
    sz = getattr(link, "size_mm", {}) or {}
    return any(_mm(sz.get(k)) for k in ("x", "y", "z"))


_CYL_HINTS = ("cylinder", "cyl", "disc", "disk", "shaft", "rod", "tube", "ring",
              "bearing", "annulus", "arbor", "pinion", "pin", "post", "pillar",
              "staff", "jewel", "washer", "spacer", "bushing", "collar", "wheel",
              "barrel", "drum", "cap", "hub", "boss", "stem", "spring", "hairspring")
_BOX_HINTS = ("box", "cube", "block", "plate", "slab", "bridge", "cock", "click",
              "pallet", "lever", "arm", "bracket", "beam", "bar", "fork", "finger",
              "tab", "lug", "clip", "spline", "key", "cover", "base")


def is_axial_part(link) -> bool:
    """Whether the worker builds this part around the canonical local +Z primary axis."""
    hint = (getattr(link, "shape_hint", "") or "").strip().lower()
    is_gear = "gear" in hint or _gear_pitch_radius_mm(link) is not None
    hint_is_box = any(w in hint for w in _BOX_HINTS)
    hint_is_cyl = any(w in hint for w in _CYL_HINTS)
    is_cyl = (is_gear or hint_is_cyl or _radius_mm(link) is not None
              or (not hint_is_box and not _has_box_dims(link)))
    if hint_is_box and not is_gear and _radius_mm(link) is None:
        is_cyl = False
    return is_cyl


def infer_ports(link) -> dict:
    """Convention ports for a part from shape_hint+size_mm. Cylinder/gear: `outer` + `bore`
    (both +Z axis at origin) + `end_a`/`end_b` (the two flat faces at -/+ half-height) + a
    `teeth` port on a gear (carrying pitch radius). Box: six `face_{px,nx,py,ny,pz,nz}` +
    `center`. The workers' convention (model.py) is the part's primary axis = local +Z, so
    the bore/outer/teeth axis is +Z. Explicit PortSpec in the IR overrides these by name.

    Shape recognition is FORGIVING: managers write descriptive shape_hints ("bridge",
    "cock", "click", "pallet lever") that are geometrically a box or a cylinder. Rather than
    collapse those to a bare `center` port (which then makes every mate that references a
    face/end fail), we (a) recognize a broad set of prismatic/round synonyms, and (b) fall
    back to the SIZE keys — x/y/z -> box, radius/diameter -> cylinder — so a part keeps its
    real ports even under a free-text hint. Cylinders ALSO expose `face_*` aliases (a round
    plate's top face is intuitively `face_pz`, but the canonical name is `end_b`)."""
    hint = (getattr(link, "shape_hint", "") or "").strip().lower()
    ports: dict = {}
    z = (0.0, 0.0, 1.0)
    is_gear = "gear" in hint or _gear_pitch_radius_mm(link) is not None

    if is_axial_part(link):
        r = _radius_mm(link) or 0.0
        h = _height_mm(link) or (2 * r if r else 0.0)
        half = h / 2.0 / 1000.0 if h else 0.0
        ports["outer"] = PortSpec(name="outer", type="shaft", axis=z, diameter_mm=2 * r)
        bore = _bore_dia_mm(link)
        if bore:
            ports["bore"] = PortSpec(name="bore", type="bore", axis=z, diameter_mm=bore)
        ports["end_a"] = PortSpec(name="end_a", type="flat_face",
                                  xyz_mm=(0.0, 0.0, -h / 2.0 if h else 0.0),
                                  axis=z, normal_sign=-1.0)
        ports["end_b"] = PortSpec(name="end_b", type="flat_face",
                                  xyz_mm=(0.0, 0.0, h / 2.0 if h else 0.0),
                                  axis=z, normal_sign=1.0)
        # face_* aliases: a manager reaching for a box face on a round plate should resolve.
        # +Z/-Z faces map to the two flat ends; the four radial faces map to the outer wall.
        ports["face_pz"] = ports["end_b"]
        ports["face_nz"] = ports["end_a"]
        for _fn in ("face_px", "face_nx", "face_py", "face_ny"):
            ports[_fn] = ports["outer"]
        if is_gear:
            pr = _gear_pitch_radius_mm(link) or 0.0
            ports["teeth"] = PortSpec(name="teeth", type="gear_mesh", axis=z,
                                      pitch_radius_mm=pr)
        del half  # (kept the mm form in xyz_mm above; half unused)
        return ports

    if any(w in hint for w in _BOX_HINTS) or _has_box_dims(link):
        sz = getattr(link, "size_mm", {}) or {}
        x, y, zt = _mm(sz.get("x")) or 0.0, _mm(sz.get("y")) or 0.0, _mm(sz.get("z")) or 0.0
        faces = {
            "face_px": ((x / 2.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            "face_nx": ((-x / 2.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
            "face_py": ((0.0, y / 2.0, 0.0), (0.0, 1.0, 0.0)),
            "face_ny": ((0.0, -y / 2.0, 0.0), (0.0, -1.0, 0.0)),
            "face_pz": ((0.0, 0.0, zt / 2.0), (0.0, 0.0, 1.0)),
            "face_nz": ((0.0, 0.0, -zt / 2.0), (0.0, 0.0, -1.0)),
        }
        for nm, (pos, nrm) in faces.items():
            ports[nm] = PortSpec(name=nm, type="flat_face", xyz_mm=pos, axis=nrm,
                                 normal_sign=1.0)
        ports["center"] = PortSpec(name="center", type="flat_face", axis=z)
        # end_a/end_b aliases: a box's ±Z faces answer to the cylinder end names too, so a
        # mate authored either way resolves symmetrically with the cylinder branch.
        ports["end_b"] = ports["face_pz"]
        ports["end_a"] = ports["face_nz"]
        bore = _bore_dia_mm(link)
        if bore:
            ports["bore"] = PortSpec(name="bore", type="bore", axis=z, diameter_mm=bore)
        return ports

    # Unknown shape with NO usable dims: give it a center port + an optional bore so it can
    # still be mated (rare — most parts hit the box/cylinder branch above via dims).
    ports["center"] = PortSpec(name="center", type="flat_face", axis=z)
    bore = _bore_dia_mm(link)
    if bore:
        ports["bore"] = PortSpec(name="bore", type="bore", axis=z, diameter_mm=bore)
    return ports


def _ports_for(link, override_dicts) -> dict:
    """Inferred ports for a part, with any authored PortSpec dicts merged over by name."""
    ports = infer_ports(link)
    for d in override_dicts or []:
        if not isinstance(d, dict) or not d.get("name"):
            raise MateSolveError(f"part '{link.name}' has a port with no name: {d!r}")
        ports[str(d["name"])] = PortSpec(
            name=str(d["name"]),
            type=str(d.get("type") or "flat_face"),
            xyz_mm=tuple(d.get("xyz_mm") or (0.0, 0.0, 0.0)),
            axis=tuple(d.get("axis") or (0.0, 0.0, 1.0)),
            diameter_mm=float(d.get("diameter_mm") or 0.0),
            depth_mm=float(d.get("depth_mm") or 0.0),
            pitch_radius_mm=float(d.get("pitch_radius_mm") or 0.0),
            normal_sign=float(d.get("normal_sign") or 1.0),
        )
    return ports


def _port_local_frame(port: PortSpec) -> np.ndarray:
    """A port's LOCAL frame L (4x4): +Z along the port axis, origin at the port (mm -> m)."""
    origin_m = np.asarray(port.xyz_mm, dtype=float) / 1000.0
    return _frame_from_axis(port.axis, origin_m)


# --------------------------------------------------------------------------- #
# Per-mate resolvers — T_incoming given T_base (both 4x4 world) and the two ports
# --------------------------------------------------------------------------- #

_SEP_NAMED = {
    "+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0), "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0), "-z": (0.0, 0.0, -1.0),
}


def _separation_dir(mate: MateSpec, base_axis_world) -> np.ndarray:
    """World direction from base center to incoming center for a gear mate. Uses
    mate.separation_axis (a named dir like '+x' or a 3-vector); if absent, defaults to a
    canonical perpendicular of the base axis (deterministic)."""
    sep = mate.separation_axis
    if isinstance(sep, str) and sep.strip().lower() in _SEP_NAMED:
        return _unit(_SEP_NAMED[sep.strip().lower()])
    if isinstance(sep, (list, tuple)) and len(sep) == 3:
        return _unit(sep)
    # No separation axis given: pick a stable perpendicular to the base axis.
    z = _unit(base_axis_world)
    ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    return _unit(np.cross(ref, z))


def _resolve_coaxial(T_base, Lb, Lc, mate: MateSpec) -> np.ndarray:
    """Align the incoming port axis onto the base port axis, coincident origins, then slide
    `offset_mm` along the shared axis and roll `angle_rad`. Shafts point the SAME way as the
    bore (no flip). T_incoming = (T_base·Lb) · slide · roll · Lc^-1."""
    Lb_w = T_base @ Lb
    slide = np.eye(4)
    slide[2, 3] = float(mate.offset_mm) / 1000.0        # +Z is the shared axis in port frame
    roll = _rot_about((0, 0, 1), mate.angle_rad)
    return Lb_w @ slide @ roll @ tf.inverse_matrix(Lc)


def _resolve_face(T_base, Lb, Lc, mate: MateSpec) -> np.ndarray:
    """Coincide the two faces, ANTI-align their normals (flip 180° about the in-plane x) so
    they seat front-to-front, with a gap of `offset_mm` along the normal and an optional
    in-plane roll `angle_rad`."""
    Lb_w = T_base @ Lb
    flip = tf.rotation_matrix(math.pi, (1.0, 0.0, 0.0)) if mate.flip else np.eye(4)
    gap = np.eye(4)
    gap[2, 3] = float(mate.offset_mm) / 1000.0
    roll = _rot_about((0, 0, 1), mate.angle_rad)
    return Lb_w @ gap @ flip @ roll @ tf.inverse_matrix(Lc)


def _resolve_gear(T_base, base_link, base_port, incoming_link, incoming_port,
                  mate: MateSpec) -> np.ndarray:
    """Place the incoming gear so its teeth mesh the base gear.

    Parallel (axis_angle_deg==0): incoming axis parallel to base axis; incoming CENTER at
    C = r_base +/- r_incoming (internal uses the difference) from the base center along the
    separation direction; roll by angle_rad (tooth phase, best-effort).

    Perpendicular/angled (axis_angle_deg!=0): rotate the incoming axis by that angle so the
    pitch cones share the apex at the axis intersection (bevel/miter) or the axes cross at
    a center distance (worm). NEW capability — only 0 and 90 are solved; other angles raise
    MateSolveError (so the menu grows without silent mis-solves)."""
    base_axis_w = _unit((T_base @ _port_local_frame(base_port))[:3, 2])
    base_center_w = (T_base @ _port_local_frame(base_port))[:3, 3]
    r_base = base_port.pitch_radius_mm or _gear_pitch_radius_mm(base_link) or 0.0
    r_in = incoming_port.pitch_radius_mm or _gear_pitch_radius_mm(incoming_link) or 0.0
    if r_base <= 0 or r_in <= 0:
        raise MateSolveError(
            f"gear mate '{mate.name}': missing pitch radius (base '{base_link.name}' "
            f"r={r_base}mm, incoming '{incoming_link.name}' r={r_in}mm) — give each gear a "
            f"module+teeth or a pitch diameter")

    ang = float(mate.axis_angle_deg)
    sep = _separation_dir(mate, base_axis_w)
    # Make sep exactly perpendicular to the base axis (project out any axial component).
    sep = _unit(sep - np.dot(sep, base_axis_w) * base_axis_w)

    if abs(ang) < 1e-6:
        internal = mate.mate_type in _INTERNAL_GEAR_MATES
        C = (r_base - r_in if internal else r_base + r_in) / 1000.0
        center_in_w = base_center_w + C * sep
        incoming_axis_w = base_axis_w
    elif abs(ang - 90.0) < 1e-6:
        # Bevel/worm: incoming axis is perpendicular to the base axis, in the plane spanned
        # by (base_axis, sep). The two pitch circles are tangent at the axis intersection.
        incoming_axis_w = _unit(np.cross(base_axis_w, sep))
        # Place the incoming center one (r_base + r_in) along sep from the base center; the
        # pure-contact sim re-seats teeth, so this positions them tangent well enough.
        C = (r_base + r_in) / 1000.0
        center_in_w = base_center_w + C * sep + (mate.offset_e_mm / 1000.0) * base_axis_w
    else:
        raise MateSolveError(
            f"gear mate '{mate.name}': axis_angle_deg={ang} is not supported yet "
            f"(only 0 = parallel spur and 90 = bevel/worm). Restrict to those, or extend "
            f"mate_solver + precheck.gear_center_distance.")

    # Build T_incoming: rotate the incoming part's gear-port axis onto incoming_axis_w, then
    # translate so the gear-port origin lands at center_in_w. The incoming gear port's local
    # frame is Lc; we want (T_incoming @ Lc) to have +Z = incoming_axis_w and origin center.
    target = _frame_from_axis(incoming_axis_w, center_in_w)
    roll = _rot_about((0, 0, 1), mate.angle_rad)
    Lc = _port_local_frame(incoming_port)
    return target @ roll @ tf.inverse_matrix(Lc)


def _resolve_contact(T_base, base_link, base_port, incoming_link, incoming_port,
                     mate: MateSpec) -> np.ndarray:
    """Place a pawl/click/detent/follower TANGENT to the wheel's rim (no pitch radius).

    Unlike a gear mesh (which needs BOTH parts' pitch radii), a contact mate only needs the
    WHEEL's outer radius: the other part is set so its contact port sits on the wheel rim,
    one wheel-radius from the wheel center along the separation direction. The pawl needs no
    radius (it is a lever). `offset_mm` nudges the contact point radially (a slight preload);
    `angle_rad` rolls the incoming part.

    ORDER-AGNOSTIC: the BFS may traverse this edge from either side, so whichever of the two
    parts actually has a radius is treated as the wheel (kept at T_base's port), and the
    other is seated on its rim. This keeps a pawl tangent to the wheel regardless of which
    part the graph reached first."""
    r_base = base_port.pitch_radius_mm or _radius_mm(base_link) or 0.0
    r_in = incoming_port.pitch_radius_mm or _radius_mm(incoming_link) or 0.0
    if r_base <= 0 and r_in <= 0:
        raise MateSolveError(
            f"contact mate '{mate.name}': neither '{base_link.name}' nor "
            f"'{incoming_link.name}' has an outer radius — give the WHEEL a radius/diameter "
            f"(or module+teeth) so the pawl/follower can be seated on its rim")
    # The already-placed base part is at T_base; we only ever return the INCOMING part's
    # world transform. If the base is the wheel (has radius), seat the incoming on its rim.
    # If instead the INCOMING is the wheel and the base is the pawl, seat the wheel so its
    # rim touches the base pawl's contact port (the pawl is the fixed reference here).
    wheel_is_base = r_base > 0
    r_wheel = r_base if wheel_is_base else r_in
    ref_center_w = (T_base @ _port_local_frame(base_port))[:3, 3]
    ref_axis_w = _unit((T_base @ _port_local_frame(base_port))[:3, 2])
    sep = _separation_dir(mate, ref_axis_w)
    sep = _unit(sep - np.dot(sep, ref_axis_w) * ref_axis_w)   # perpendicular to the axis
    C = (r_wheel + float(mate.offset_mm)) / 1000.0
    contact_w = ref_center_w + C * sep
    target = _frame_from_axis(ref_axis_w, contact_w)
    roll = _rot_about((0, 0, 1), mate.angle_rad)
    Lc = _port_local_frame(incoming_port)
    return target @ roll @ tf.inverse_matrix(Lc)


def _resolve_mate(mate: MateSpec, T_base, base_link, base_ports, incoming_link,
                  incoming_ports) -> np.ndarray:
    """Dispatch to the resolver for this mate's family and return the incoming part's world
    transform (4x4)."""
    bp = base_ports.get(mate.base_port)
    ip = incoming_ports.get(mate.incoming_port)
    if bp is None:
        raise MateSolveError(
            f"mate '{mate.name}': part '{base_link.name}' has no port '{mate.base_port}' "
            f"(known: {sorted(base_ports)})")
    if ip is None:
        raise MateSolveError(
            f"mate '{mate.name}': part '{incoming_link.name}' has no port "
            f"'{mate.incoming_port}' (known: {sorted(incoming_ports)})")

    mt = mate.mate_type
    if mt in _PARALLEL_GEAR_MATES or mt in _ANGLED_GEAR_MATES:
        return _resolve_gear(T_base, base_link, bp, incoming_link, ip, mate)
    if mt in _CONTACT_MATES:
        return _resolve_contact(T_base, base_link, bp, incoming_link, ip, mate)
    Lb, Lc = _port_local_frame(bp), _port_local_frame(ip)
    if mt in _COAXIAL_MATES:
        return _resolve_coaxial(T_base, Lb, Lc, mate)
    if mt in _FACE_MATES:
        return _resolve_face(T_base, Lb, Lc, mate)
    if mt in _COAXIAL_FACE_MATES:
        # Coaxial orientation, then seat the incoming face on the base face plane: reuse the
        # coaxial resolve (offset_mm sets the axial seat).
        return _resolve_coaxial(T_base, Lb, Lc, mate)
    raise MateSolveError(
        f"mate '{mate.name}': unknown mate_type '{mt}'. Use one of the catalog types "
        f"(coaxial, face_to_face, coaxial_face, gear_spur_external, gear_bevel, worm, "
        f"press_fit, bolted, welded, ...).")


# --------------------------------------------------------------------------- #
# Parse the IR + solve the forest
# --------------------------------------------------------------------------- #

def _parse_mate(d: dict, idx: int) -> MateSpec:
    if not isinstance(d, dict):
        raise MateSolveError(f"mates[{idx}] is not an object")
    for k in ("mate_type", "base_part", "base_port", "incoming_part", "incoming_port"):
        v = d.get(k)
        if not isinstance(v, str) or not v.strip():
            raise MateSolveError(f"mates[{idx}] is missing '{k}'")
    sep = d.get("separation_axis")
    return MateSpec(
        name=str(d.get("name") or f"mate_{idx}"),
        mate_type=str(d["mate_type"]).strip(),
        base_part=str(d["base_part"]).strip(),
        base_port=str(d["base_port"]).strip(),
        incoming_part=str(d["incoming_part"]).strip(),
        incoming_port=str(d["incoming_port"]).strip(),
        offset_mm=float(d.get("offset_mm") or 0.0),
        angle_rad=float(d.get("angle_rad") or 0.0),
        flip=bool(d.get("flip", True)),
        axis_angle_deg=float(d.get("axis_angle_deg") or 0.0),
        separation_axis=tuple(sep) if isinstance(sep, (list, tuple)) else (sep or ()),
        offset_e_mm=float(d.get("offset_e_mm") or 0.0),
    )


def solve_connection_graph(ir: dict) -> KinematicModel:
    """Compile a connection-graph IR (parts + mates) into a (not-yet-validated)
    KinematicModel. Raises MateSolveError (a ValueError) on any un/over-constrained graph,
    an unknown port/mate, or a gear missing its pitch radius — caught by the manager retry
    loop and fed back as a repair request. The caller runs `manager._validate_model`."""
    if not isinstance(ir, dict):
        raise MateSolveError("connection graph must be a JSON object")
    part_dicts = ir.get("parts")
    if not isinstance(part_dicts, list) or not part_dicts:
        raise MateSolveError("'parts' must be a non-empty array")

    links = [_link_from_dict(d, i) for i, d in enumerate(part_dicts)]
    by_name: dict = {}
    ports_by_part: dict = {}
    for link, d in zip(links, part_dicts):
        if link.name in by_name:
            raise MateSolveError(f"duplicate part name: '{link.name}'")
        by_name[link.name] = link
        ports_by_part[link.name] = _ports_for(link, d.get("ports"))

    mates = [_parse_mate(d, i) for i, d in enumerate(ir.get("mates") or [])]
    for m in mates:
        for pname in (m.base_part, m.incoming_part):
            if pname not in by_name:
                raise MateSolveError(f"mate '{m.name}' names unknown part '{pname}'")

    root = str(ir.get("root_part") or "").strip() or links[0].name
    if root not in by_name:
        raise MateSolveError(f"root_part '{root}' is not a declared part")

    # Which gears mesh with >1 peer -> each such mate MUST name a separation_axis (no guess).
    gear_peer_count: dict = {}
    for m in mates:
        if m.mate_type in _MESH_PAIR_MATES:
            gear_peer_count[m.base_part] = gear_peer_count.get(m.base_part, 0) + 1
    for m in mates:
        if (m.mate_type in _MESH_PAIR_MATES
                and gear_peer_count.get(m.base_part, 0) > 1
                and not m.separation_axis):
            raise MateSolveError(
                f"gear mate '{m.name}': part '{m.base_part}' meshes with more than one gear, "
                f"so this mate MUST give a 'separation_axis' (e.g. '+x') to fix which "
                f"direction '{m.incoming_part}' sits — otherwise placement is ambiguous.")

    # BFS the undirected mate graph from root, placing each newly-reached part.
    adj: dict = {name: [] for name in by_name}
    for m in mates:
        adj[m.base_part].append((m, True))       # traversed base->incoming
        adj[m.incoming_part].append((m, False))  # traversed incoming->base (swapped)

    T_world: dict = {root: np.eye(4)}
    placement_parent: dict = {}
    order = [root]
    stack = [root]
    while stack:
        cur = stack.pop()
        for m, forward in adj[cur]:
            other = m.incoming_part if forward else m.base_part
            # Orient the mate so `cur` is the already-placed base and `other` is incoming.
            base_p, in_p = (cur, other)
            if base_p == m.base_part:
                base_link, in_link = by_name[m.base_part], by_name[m.incoming_part]
                base_ports, in_ports = ports_by_part[m.base_part], ports_by_part[m.incoming_part]
                eff = m
            else:
                # Traversed backward: swap base/incoming and invert the direction-carrying
                # params (offset, angle, AND the gear separation axis, which points
                # base->incoming) so the SAME geometric constraint resolves from the other side.
                base_link, in_link = by_name[m.incoming_part], by_name[m.base_part]
                base_ports, in_ports = ports_by_part[m.incoming_part], ports_by_part[m.base_part]
                sep = m.separation_axis
                sep_rev: tuple = ()
                if isinstance(sep, str) and sep.strip().lower() in _SEP_NAMED:
                    sep_rev = tuple(-c for c in _SEP_NAMED[sep.strip().lower()])
                elif isinstance(sep, (list, tuple)) and len(sep) == 3:
                    sep_rev = tuple(-float(c) for c in sep)
                eff = MateSpec(
                    name=m.name, mate_type=m.mate_type,
                    base_part=m.incoming_part, base_port=m.incoming_port,
                    incoming_part=m.base_part, incoming_port=m.base_port,
                    offset_mm=-m.offset_mm, angle_rad=-m.angle_rad, flip=m.flip,
                    axis_angle_deg=m.axis_angle_deg, separation_axis=sep_rev,
                    offset_e_mm=-m.offset_e_mm)
            T_other = _resolve_mate(eff, T_world[base_p], base_link, base_ports,
                                    in_link, in_ports)
            if other in T_world:
                # Closing edge: the graph already placed `other` via another path. A redundant
                # mate is a CONSTRAINT, not necessarily a conflict — the AssemCAD principle is
                # that a mate REMOVES dof, it doesn't fully pin a part. A COAXIAL mate (shaft in
                # a bore) removes the 2 perpendicular translations + 2 perpendicular rotations
                # but LEAVES the axial slide + spin free. So a shaft coaxial to TWO colinear
                # bearings is compatible: the two mates agree on the AXIS LINE; they may disagree
                # on where along it the part sits (the free dof), and that is fine. We therefore
                # reject only the PERPENDICULAR component of the drift (axes not colinear = a real
                # geometric conflict) for a coaxial-family mate; along-axis drift is allowed.
                # Face / gear mates fully locate the mate point, so they keep the strict check.
                delta = T_world[other][:3, 3] - T_other[:3, 3]
                if eff.mate_type in _COAXIAL_MATES:
                    # Shared axis in world = the base port's +Z (the coaxial axis).
                    axis_w = _unit((T_world[base_p] @ _port_local_frame(base_ports.get(eff.base_port)))[:3, 2])
                    perp = delta - np.dot(delta, axis_w) * axis_w   # component off the axis line
                    conflict = float(np.linalg.norm(perp)) > _POS_TOL_M
                    detail_axis = (" — the two mate paths put it on DIFFERENT axis lines "
                                   f"({np.linalg.norm(perp)*1000:.1f} mm apart perpendicular to "
                                   "the shared axis)")
                else:
                    conflict = float(np.linalg.norm(delta)) > _POS_TOL_M
                    detail_axis = ""
                if conflict:
                    raise MateSolveError(
                        f"over-constrained: part '{other}' is placed {np.linalg.norm(delta)*1000:.1f} "
                        f"mm apart by two conflicting mate paths (one via mate '{m.name}')"
                        f"{detail_axis}. Every part must be positioned consistently. This is often "
                        f"a shaft mated to two bearings whose bores are NOT colinear, or a part "
                        f"mated to two things that fix its position differently: align the two "
                        f"bores on one axis, or remove/re-root one of the mates positioning "
                        f"'{other}'.")
                continue
            T_world[other] = T_other
            placement_parent[other] = base_p
            order.append(other)
            stack.append(other)

    unreached = [name for name in by_name if name not in T_world]
    if unreached:
        raise MateSolveError(
            f"these parts are connected by NO mate (they would float): {unreached}. Every "
            f"part must mate to the rest of the subassembly, forming one connected graph "
            f"rooted at '{root}'.")

    # World transforms -> a parent-relative PoseSpec forest (root carries no pose).
    poses: list[PoseSpec] = []
    for name in order:
        if name == root:
            continue
        parent = placement_parent[name]
        T_rel = tf.inverse_matrix(T_world[parent]) @ T_world[name]
        xyz, rpy = _decompose(T_rel)
        poses.append(PoseSpec(name=f"place_{name}", parent=parent, child=name,
                              xyz_m=xyz, rpy_rad=rpy))

    # mesh_pairs: authored ones + every gear/contact mate.
    mesh_pairs = list(_mesh_pairs_from(ir))
    seen = {frozenset(p) for p in mesh_pairs}
    for m in mates:
        if m.mate_type in _MESH_PAIR_MATES:
            key = frozenset((m.base_part, m.incoming_part))
            if key not in seen:
                mesh_pairs.append((m.base_part, m.incoming_part))
                seen.add(key)

    # frames_realized: expose an interface frame at a named port (subassembly mode).
    frames_realized: list[dict] = []
    for fr in ir.get("frames") or []:
        if not isinstance(fr, dict):
            raise MateSolveError(f"frames entry is not an object: {fr!r}")
        fname, pname, portname = fr.get("frame"), fr.get("part"), fr.get("port")
        if not (fname and pname and portname):
            raise MateSolveError(
                f"frames entry needs 'frame', 'part', 'port': {fr!r}")
        if pname not in by_name:
            raise MateSolveError(f"frame '{fname}' names unknown part '{pname}'")
        port = ports_by_part[pname].get(str(portname))
        if port is None:
            raise MateSolveError(
                f"frame '{fname}': part '{pname}' has no port '{portname}'")
        _, rpy = _decompose(_port_local_frame(port))
        frames_realized.append({
            "frame": str(fname),
            "link": str(pname),
            "local_xyz_m": tuple(np.asarray(port.xyz_mm, dtype=float) / 1000.0),
            "local_rpy_rad": rpy,
        })

    model = KinematicModel(
        name=str(ir.get("name") or "product"),
        root_link=root,
        links=links,
        poses=poses,
        mesh_pairs=mesh_pairs,
    )
    model.frames_realized = frames_realized
    return model


def solve_connection_graph_text(text: str) -> KinematicModel:
    """Convenience: extract the single JSON object from a manager response (NOTES already
    stripped by the two-phase split) and solve it. Mirrors how `parse_model` is called."""
    try:
        ir = json.loads(extract_json_object(text))
    except (ValueError, json.JSONDecodeError) as e:
        raise MateSolveError(f"connection graph is not valid JSON: {e}") from e
    return solve_connection_graph(ir)
