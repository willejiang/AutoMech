"""Deterministic parser: the manager's PARTS-list + MJCF-style skeleton -> KinematicModel.

Track 2 (see .claude/plans/precious-humming-wand.md Part A). The manager no longer emits
a single comment-less `KinematicModel` JSON. It emits TWO blocks:

  1. PARTS — a JSON array of geometry params per part (the fields `manager._link_from_dict`
     already parses: name, shape_hint, size_mm, origin_note, color, material, driver, plus
     spin_axis + dof), and a top-level `mesh_pairs`.
  2. MJCF — an authoring skeleton `<mujoco><worldbody>` with one nested `<body name pos quat>`
     per part, each carrying an XML COMMENT on its role/frame/meshing, a `<joint>`/`<freejoint>`
     for its dof, and `<site name="frame_...">` elements for the interface frames it realizes.

This parser is the DESIGN-AUTHORING inverse of `mjcf_builder._emit_body`
(mjcf_builder.py:169-197): geometry (pose, nesting, sites) comes from the MJCF; per-part
attributes (dof, spin_axis, size, color, material, driver) come from PARTS, joined by name. It
produces an ordinary, not-yet-validated `KinematicModel` — the caller feeds it through the
EXISTING `manager._validate_model`, so slug/dedup/mesh_filename/forest normalization is
unchanged, and the ENTIRE downstream pipeline keeps consuming `KinematicModel` untouched.

CRITICAL: the skeleton is a DESIGN document authored at each part's TRUE pose. It is NOT the
compiled simulation MJCF — `mjcf_builder.build_mjcf` remains the sole simulation compiler (it
re-applies CoACD, mm->m scale, base_height lift, solver tuning, per-part mass). So the parser
reads `<body pos>` VERBATIM: it does not subtract the root's `base_height` (that lift only
exists in the compiled MJCF, never in the manager's design skeleton).
"""
from __future__ import annotations

import json
import xml.etree.ElementTree as ET

import trimesh.transformations as tf

from .manager import _link_from_dict, _mesh_pairs_from
from .model import KinematicModel, LinkSpec, PoseSpec


class SkeletonError(ValueError):
    """The PARTS+MJCF skeleton could not be parsed into a KinematicModel. A ValueError
    subclass so `manager._decompose_loop`'s existing `except (ValueError, ...)` catches it
    and feeds it back as a repair request, exactly like the old JSON-parse errors."""


def _rpy_from_quat(quat_str: str) -> tuple:
    """Inverse of `mjcf_builder._quat_from_rpy`: a MuJoCo quaternion string "w x y z" back to
    fixed-axis XYZ rpy radians (axes='sxyz'). An absent/empty quat is identity (0,0,0). A
    malformed quat raises SkeletonError."""
    s = (quat_str or "").strip()
    if not s:
        return (0.0, 0.0, 0.0)
    parts = s.split()
    if len(parts) != 4:
        raise SkeletonError(f"body quat must be 4 numbers 'w x y z', got {quat_str!r}")
    try:
        q = [float(x) for x in parts]
    except ValueError as e:
        raise SkeletonError(f"body quat has a non-number: {quat_str!r}") from e
    r, p, y = tf.euler_from_quaternion(q, axes="sxyz")
    return (float(r), float(p), float(y))


def _xyz_from_pos(pos_str: str, *, what: str) -> tuple:
    """A MuJoCo "x y z" position string -> (x, y, z) floats in meters. Absent/empty is the
    origin (0,0,0). Read VERBATIM — no base_height adjustment (design skeleton, not sim MJCF)."""
    s = (pos_str or "").strip()
    if not s:
        return (0.0, 0.0, 0.0)
    parts = s.split()
    if len(parts) != 3:
        raise SkeletonError(f"{what} pos must be 3 numbers 'x y z', got {pos_str!r}")
    try:
        return tuple(float(x) for x in parts)
    except ValueError as e:
        raise SkeletonError(f"{what} pos has a non-number: {pos_str!r}") from e


def _extract_parts_json(text: str) -> str:
    """Pull the first balanced JSON value — an ARRAY `[...]` or an OBJECT `{...}` — out of the
    PARTS block, tolerating ```json fences and prose (brace/bracket depth, ignoring delimiters
    inside strings). The PARTS block is authored as a bare array, but a `{"parts": [...]}`
    wrapper is also accepted, so we scan for whichever of `[`/`{` appears first."""
    t = text.strip()
    starts = [i for i in (t.find("["), t.find("{")) if i != -1]
    if not starts:
        raise SkeletonError("PARTS block contains no JSON array or object")
    start = min(starts)
    open_c = t[start]
    close_c = "]" if open_c == "[" else "}"
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(t)):
        c = t[i]
        if in_str:
            if esc:
                esc = False
            elif c == "\\":
                esc = True
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == open_c:
            depth += 1
        elif c == close_c:
            depth -= 1
            if depth == 0:
                return t[start:i + 1]
    raise SkeletonError("PARTS block JSON is unbalanced (never closes)")


def _parse_parts(parts_json: str) -> tuple[list[LinkSpec], dict, list]:
    """Parse the PARTS block into (links, links_by_name, mesh_pairs). Accepts either a bare
    JSON array of parts, or a `{"parts": [...], "mesh_pairs": [...]}` / `{"links": [...]}`
    wrapper (so mesh_pairs can ride alongside). Reuses `manager._link_from_dict` so the field
    contract (dof, spin_axis, color, material, driver, size_mm) is IDENTICAL to the old path."""
    try:
        raw = json.loads(_extract_parts_json(parts_json))
    except (SkeletonError,):
        raise
    except (ValueError, json.JSONDecodeError) as e:
        raise SkeletonError(f"PARTS block is not valid JSON: {e}") from e

    if isinstance(raw, list):
        part_dicts = raw
        mesh_src: dict = {}
    elif isinstance(raw, dict):
        part_dicts = raw.get("parts")
        if part_dicts is None:
            part_dicts = raw.get("links")
        mesh_src = raw
    else:
        raise SkeletonError("PARTS block must be a JSON array of parts (or an object "
                            "with a 'parts' array)")
    if not isinstance(part_dicts, list) or not part_dicts:
        raise SkeletonError("PARTS must be a non-empty array of part objects")

    links = [_link_from_dict(d, i) for i, d in enumerate(part_dicts)]
    by_name: dict = {}
    for link in links:
        if link.name in by_name:
            raise SkeletonError(f"duplicate part name in PARTS: '{link.name}'")
        by_name[link.name] = link
    return links, by_name, _mesh_pairs_from(mesh_src)


def _iter_bodies(worldbody: ET.Element):
    """Yield (body_el, parent_body_name_or_empty) for every <body> under <worldbody>, in a
    pre-order walk. A body directly under <worldbody> has parent "" (a forest root)."""
    stack = [(child, "") for child in worldbody if child.tag == "body"]
    # Preserve document order for a stable, human-matching root ordering.
    stack.reverse()
    while stack:
        body, parent_name = stack.pop()
        name = (body.get("name") or "").strip()
        if not name:
            raise SkeletonError("a <body> in the skeleton is missing its name attribute")
        yield body, parent_name
        nested = [(c, name) for c in body if c.tag == "body"]
        for item in reversed(nested):
            stack.append(item)


def _dof_from_body(body: ET.Element, link: LinkSpec) -> None:
    """Cross-check (do NOT override) the part's dof against the MJCF joint element. The
    authoritative dof is the PART's (from PARTS JSON) — `_emit_body` forces a <freejoint> on
    EVERY unpinned ROOT regardless of its LinkSpec.dof, so a root's joint element is not a
    reliable dof signal. For a non-root body a <joint type="hinge"> means spin and its `axis`
    is copied onto the link when the part omitted one. Raises only on a hard contradiction that
    the manager must fix (a hinge axis given for a part the PARTS list marks fixed/free)."""
    hinge = None
    for j in body:
        if j.tag == "joint" and (j.get("type") or "hinge").strip() == "hinge":
            hinge = j
            break
    if hinge is None:
        return
    ax = hinge.get("axis")
    if ax and link.dof == "spin":
        parts = ax.split()
        if len(parts) == 3:
            try:
                link.spin_axis = tuple(float(v) for v in parts)
            except ValueError:
                pass


def _sites_of(body: ET.Element, body_name: str) -> list[dict]:
    """Every <site name pos rpy> in a body -> a frames_realized entry. The site's parent body
    is the realized link; its pos/rpy (link-local meters/radians) are the frame offset. The
    contract frame name is the site name with an optional `frame_` prefix stripped (the emit
    convention writes `frame_<contractname>`)."""
    out: list[dict] = []
    for s in body:
        if s.tag != "site":
            continue
        raw = (s.get("name") or "").strip()
        if not raw:
            raise SkeletonError(f"a <site> in body '{body_name}' is missing its name")
        frame = raw[len("frame_"):] if raw.startswith("frame_") else raw
        out.append({
            "frame": frame,
            "link": body_name,
            "local_xyz_m": _xyz_from_pos(s.get("pos"), what=f"site '{raw}'"),
            "local_rpy_rad": _rpy_from_quat(s.get("quat"))
            if s.get("quat") else _rpy_of_site(s),
        })
    return out


def _rpy_of_site(site: ET.Element) -> tuple:
    """A site orients via `quat` (handled by the caller) or an `rpy`/`euler` attr; default
    identity. MuJoCo's own <site> takes `euler` (radians, matching the model rpy), so accept
    both `rpy` and `euler` spellings for the design skeleton."""
    val = site.get("rpy") or site.get("euler")
    if not val:
        return (0.0, 0.0, 0.0)
    parts = val.split()
    if len(parts) != 3:
        raise SkeletonError(f"site rpy/euler must be 3 numbers, got {val!r}")
    try:
        return tuple(float(v) for v in parts)
    except ValueError as e:
        raise SkeletonError(f"site rpy/euler has a non-number: {val!r}") from e


def mjcf_skeleton_parser(mjcf_xml: str, parts_json: str) -> KinematicModel:
    """Turn the manager's PARTS-list + MJCF-style skeleton into a (not-yet-validated)
    KinematicModel, invertible against `mjcf_builder._emit_body`.

      * <body name pos quat> -> a PoseSpec (xyz_m from pos, rpy_rad from quat, parent from XML
        nesting; a body directly under <worldbody> is a root, parent "").
      * per-part geometry/dof/color/material/driver -> joined BY NAME from PARTS.
      * <joint type="hinge" axis> cross-checks dof + supplies spin_axis; <freejoint>/none carry
        no geometry (dof comes from PARTS).
      * <site name="frame_<x>" pos rpy> -> a frames_realized entry (parent body = realized link).
      * root_link = the FIRST top-level body under <worldbody>; mesh_pairs from PARTS.

    A body with no matching part, or a part with no body, raises SkeletonError naming it (a
    ValueError subclass, caught by the manager retry loop and fed back as a repair request).
    Malformed XML raises ET.ParseError (also caught by the loop)."""
    links, links_by_name, mesh_pairs = _parse_parts(parts_json)

    root = ET.fromstring(mjcf_xml)          # may raise ET.ParseError -> caught by the loop
    worldbody = root.find("worldbody") if root.tag == "mujoco" else None
    if worldbody is None:
        # Tolerate a bare <worldbody> root, or a <mujoco> without the wrapper tag.
        worldbody = root if root.tag == "worldbody" else root.find(".//worldbody")
    if worldbody is None:
        raise SkeletonError("MJCF skeleton has no <worldbody> element")

    poses: list[PoseSpec] = []
    frames_realized: list[dict] = []
    seen_bodies: set = set()
    root_link = ""

    for body, parent_name in _iter_bodies(worldbody):
        name = (body.get("name") or "").strip()
        if name in seen_bodies:
            raise SkeletonError(f"duplicate <body name='{name}'> in the skeleton")
        seen_bodies.add(name)
        link = links_by_name.get(name)
        if link is None:
            raise SkeletonError(
                f"<body name='{name}'> has no matching part in the PARTS list "
                f"(known parts: {sorted(links_by_name)})")
        if not parent_name and not root_link:
            root_link = name

        _dof_from_body(body, link)
        from .manager import _canonicalize_link_axis
        _canonicalize_link_axis(link)
        xyz = _xyz_from_pos(body.get("pos"), what=f"body '{name}'")
        rpy = _rpy_from_quat(body.get("quat"))
        # A ROOT (parent "") at the origin with identity orientation needs NO pose — that
        # matches the JSON path, where a base/root part simply has no pose. A root placed at a
        # NON-origin pose keeps an empty-parent pose so `mjcf_builder._rel_pose_of` applies its
        # offset. Non-root bodies always get a pose (it carries the parent-relative transform).
        is_origin = (all(v == 0.0 for v in xyz) and all(v == 0.0 for v in rpy))
        if parent_name or not is_origin:
            poses.append(PoseSpec(
                name=f"place_{name}",
                parent=parent_name,
                child=name,
                xyz_m=xyz,
                rpy_rad=rpy,
            ))
        frames_realized.extend(_sites_of(body, name))

    missing = [l.name for l in links if l.name not in seen_bodies]
    if missing:
        raise SkeletonError(
            f"these parts have no <body> in the MJCF skeleton: {missing}")

    model = KinematicModel(
        name="product",                     # _validate_model / caller may override
        root_link=root_link,
        links=links,
        poses=poses,
        mesh_pairs=mesh_pairs,
    )
    model.frames_realized = frames_realized
    return model
