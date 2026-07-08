"""Golden round-trip test for the manager's PARTS+MJCF skeleton (Track 2, Part A).

Gates the WHOLE of Part A: the manager's new PARTS-list + MJCF-style skeleton, when fed through
`mjcf_skeleton_parser` + `manager._validate_model`, must yield a `KinematicModel` IDENTICAL
(links / poses / dof / spin_axis / mesh_pairs / frames_realized) to what today's JSON few-shot
yields through `manager.parse_model` + `_validate_model`. Only after this passes do we flip the
manager prompt to author MJCF.

It also asserts the negative path: malformed XML raises ET.ParseError (so the manager retry loop
catches it and asks for a repair), and a part/body mismatch raises SkeletonError.

Run:  python -m maker2.tests.golden_mjcf_roundtrip
Exit 0 = round-trip identical + error paths correct; exit 1 = a mismatch (block the prompt flip).
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _ROOT)


# The BASELINE: the turntable in the OLD comment-less `links`/`poses` JSON form (what the
# pre-Track-2 manager emitted and `manager.parse_model` consumed). Kept here verbatim so this
# test pins the skeleton parser against the ORIGINAL contract, independent of the now-rewritten
# shipped few-shot (which is asserted separately, step 10).
_BASELINE_JSON = """\
{
  "name": "motorized_turntable",
  "root_link": "base",
  "links": [
    {"name": "base", "description": "flat square base plate", "shape_hint": "box",
     "size_mm": {"x": 200, "y": 200, "z": 15},
     "origin_note": "top-face center at local origin; slab extends -Z (0..-15mm), centered in X and Y (-100..100mm)",
     "color": [0.30, 0.30, 0.32], "dof": "fixed"},
    {"name": "bearing_block", "description": "pillow-block bearing", "shape_hint": "box",
     "size_mm": {"x": 50, "y": 50, "z": 50, "bore_dia": 12},
     "origin_note": "bottom-face center at local origin; block extends +Z (0..50mm); the 12mm bore runs vertically through the center",
     "color": [0.20, 0.22, 0.25], "dof": "fixed"},
    {"name": "shaft", "description": "vertical drive shaft", "shape_hint": "cylinder",
     "size_mm": {"radius": 6, "height": 90},
     "origin_note": "bottom face center at local origin; cylinder extends +Z (0..90mm), coaxial with the bearing bore",
     "color": [0.75, 0.76, 0.78], "dof": "spin", "spin_axis": [0.0, 0.0, 1.0], "driver": true},
    {"name": "platter", "description": "round turntable platter", "shape_hint": "cylinder",
     "size_mm": {"radius": 80, "height": 8},
     "origin_note": "bottom-face center at local origin; disc extends +Z (0..8mm)",
     "color": [0.10, 0.10, 0.11], "dof": "spin", "spin_axis": [0.0, 0.0, 1.0]}
  ],
  "poses": [
    {"name": "place_bearing_block", "parent": "base", "child": "bearing_block",
     "xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
    {"name": "place_shaft", "parent": "bearing_block", "child": "shaft",
     "xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
    {"name": "place_platter", "parent": "shaft", "child": "platter",
     "xyz_m": [0.0, 0.0, 0.090], "rpy_rad": [0.0, 0.0, 0.0]}
  ],
  "mesh_pairs": []
}"""


# The SAME turntable as prompts/schema.FEWSHOT_JSON, hand-authored as PARTS + an MJCF skeleton.
# Poses are parent-relative (chain base->bearing_block->shaft->platter), mirroring the JSON, so
# each <body pos> equals that part's pose xyz_m. A <site> on the shaft exercises frames_realized.
GOLDEN_PARTS = """\
[
  {
    "name": "base",
    "description": "A flat square base plate, 200 x 200 mm, 15 mm thick, that everything mounts to and that rests on the ground.",
    "shape_hint": "box",
    "size_mm": {"x": 200, "y": 200, "z": 15},
    "origin_note": "top-face center at local origin; slab extends -Z (0..-15mm), centered in X and Y (-100..100mm)",
    "color": [0.30, 0.30, 0.32],
    "dof": "fixed"
  },
  {
    "name": "bearing_block",
    "description": "A pillow-block bearing: a 50 mm cube with a 12 mm vertical bore through its center that the shaft rotates inside. A REAL fixed part.",
    "shape_hint": "box",
    "size_mm": {"x": 50, "y": 50, "z": 50, "bore_dia": 12},
    "origin_note": "bottom-face center at local origin; block extends +Z (0..50mm); the 12mm bore runs vertically through the center",
    "color": [0.20, 0.22, 0.25],
    "dof": "fixed"
  },
  {
    "name": "shaft",
    "description": "A vertical drive shaft, 12 mm diameter, 90 mm long, that turns inside the bearing bore and carries the platter on top. This is the driver the test spins.",
    "shape_hint": "cylinder",
    "size_mm": {"radius": 6, "height": 90},
    "origin_note": "bottom face center at local origin; cylinder extends +Z (0..90mm), coaxial with the bearing bore",
    "color": [0.75, 0.76, 0.78],
    "dof": "spin",
    "spin_axis": [0.0, 0.0, 1.0],
    "driver": true
  },
  {
    "name": "platter",
    "description": "A round turntable platter, 160 mm diameter, 8 mm thick, fixed on top of the shaft (rotates with it).",
    "shape_hint": "cylinder",
    "size_mm": {"radius": 80, "height": 8},
    "origin_note": "bottom-face center at local origin; disc extends +Z (0..8mm)",
    "color": [0.10, 0.10, 0.11],
    "dof": "spin",
    "spin_axis": [0.0, 0.0, 1.0]
  }
]
"""

GOLDEN_MJCF = """\
<mujoco model="motorized_turntable">
  <worldbody>
    <!-- base: structural root plate resting on the ground; everything mounts to its top face -->
    <body name="base" pos="0 0 0" quat="1 0 0 0">
      <!-- bearing_block: fixed pillow block bolted to the base; its bore carries the shaft -->
      <body name="bearing_block" pos="0 0 0" quat="1 0 0 0">
        <!-- shaft: the driver; spins on +Z inside the bearing bore -->
        <body name="shaft" pos="0 0 0" quat="1 0 0 0">
          <joint name="shaft_spin" type="hinge" axis="0 0 1" pos="0 0 0"/>
          <!-- frame where the platter seats on top of the shaft (90mm up the shaft) -->
          <site name="frame_platter_seat" pos="0 0 0.090"/>
          <!-- platter: rides on the shaft top and turns with it -->
          <body name="platter" pos="0 0 0.090" quat="1 0 0 0">
            <joint name="platter_spin" type="hinge" axis="0 0 1" pos="0 0 0"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def _fail(msg: str) -> None:
    print(f"FAIL: {msg}")
    sys.exit(1)


def _cmp_link(a, b, field: str) -> None:
    av, bv = getattr(a, field), getattr(b, field)
    if av != bv:
        _fail(f"link '{a.name}' field '{field}': skeleton={av!r} != json={bv!r}")


def main() -> None:
    from maker2.manager import parse_model, _validate_model
    from maker2.mjcf_skeleton import mjcf_skeleton_parser, SkeletonError

    # 1. Baseline: what the OLD comment-less links/poses JSON yields.
    json_model = parse_model(_BASELINE_JSON)
    _validate_model(json_model)

    # 2. The new path: PARTS + MJCF skeleton -> KinematicModel.
    sk_model = mjcf_skeleton_parser(GOLDEN_MJCF, GOLDEN_PARTS)
    _validate_model(sk_model)

    # 3. root_link + name-set identical.
    if sk_model.root_link != json_model.root_link:
        _fail(f"root_link: skeleton={sk_model.root_link!r} != json={json_model.root_link!r}")
    sk_names = [l.name for l in sk_model.links]
    js_names = [l.name for l in json_model.links]
    if sk_names != js_names:
        _fail(f"link name order differs: skeleton={sk_names} != json={js_names}")

    # 4. Each link: dof, spin_axis, driver, size_mm, color, shape_hint identical.
    js_by_name = {l.name: l for l in json_model.links}
    for sl in sk_model.links:
        jl = js_by_name[sl.name]
        for field in ("dof", "spin_axis", "driver", "size_mm", "color", "shape_hint",
                      "material", "mesh_filename"):
            _cmp_link(sl, jl, field)

    # 5. Poses: same (parent, child, xyz_m, rpy_rad) set. Pose NAMES may differ (the skeleton
    #    derives place_<child>), so compare by child.
    def pose_key(p):
        return (p.parent, p.child,
                tuple(round(v, 9) for v in p.xyz_m),
                tuple(round(v, 9) for v in p.rpy_rad))
    sk_poses = {p.child: pose_key(p) for p in sk_model.poses}
    js_poses = {p.child: pose_key(p) for p in json_model.poses}
    if sk_poses != js_poses:
        _fail(f"poses differ by child:\n  skeleton={sk_poses}\n  json={js_poses}")

    # 6. mesh_pairs identical.
    if sorted(sk_model.mesh_pairs) != sorted(json_model.mesh_pairs):
        _fail(f"mesh_pairs: skeleton={sk_model.mesh_pairs} != json={json_model.mesh_pairs}")

    # 7. frames_realized: the site parsed correctly (the JSON few-shot has none; this is the
    #    NEW capability the skeleton adds — assert it landed on the right link at the right pos).
    frs = sk_model.frames_realized
    if len(frs) != 1:
        _fail(f"expected exactly 1 realized frame from the <site>, got {len(frs)}: {frs}")
    fr = frs[0]
    if fr["frame"] != "platter_seat" or fr["link"] != "shaft":
        _fail(f"site->frame mapping wrong: {fr}")
    if tuple(round(v, 9) for v in fr["local_xyz_m"]) != (0.0, 0.0, 0.09):
        _fail(f"site local_xyz_m wrong: {fr['local_xyz_m']}")

    # 8. Negative path: malformed XML -> ET.ParseError (so the retry loop catches + repairs).
    try:
        mjcf_skeleton_parser("<mujoco><worldbody><body name='x' ", GOLDEN_PARTS)
        _fail("malformed XML did not raise")
    except ET.ParseError:
        pass
    except Exception as e:
        _fail(f"malformed XML raised {type(e).__name__}, expected ET.ParseError: {e}")

    # 9. Negative path: a body with no matching part -> SkeletonError naming it.
    bad_mjcf = GOLDEN_MJCF.replace('name="platter"', 'name="ghost_part"')
    try:
        mjcf_skeleton_parser(bad_mjcf, GOLDEN_PARTS)
        _fail("body/part mismatch did not raise")
    except SkeletonError as e:
        if "ghost_part" not in str(e):
            _fail(f"mismatch error did not name the offending body: {e}")

    print("OK: PARTS+MJCF skeleton round-trips identically to the JSON few-shot "
          f"({len(sk_model.links)} links, {len(sk_model.poses)} poses, "
          f"{len(frs)} realized frame); malformed XML + body/part mismatch both raise.")

    # 10. The SHIPPED few-shot (prompts/schema.FEWSHOT_JSON) must itself parse — it is the
    #     golden example the manager copies, so a broken one silently teaches a broken format.
    from maker2.prompts.schema import FEWSHOT_JSON, MJCF_SENTINEL
    if MJCF_SENTINEL not in FEWSHOT_JSON:
        _fail("FEWSHOT_JSON is missing the MJCF sentinel")
    fs_parts, fs_mjcf = FEWSHOT_JSON.split(MJCF_SENTINEL, 1)
    fs_model = mjcf_skeleton_parser(fs_mjcf, fs_parts)
    _validate_model(fs_model)
    fs_names = [l.name for l in fs_model.links]
    if fs_names != js_names:
        _fail(f"shipped FEWSHOT_JSON links differ from baseline: {fs_names} != {js_names}")
    print(f"OK: shipped FEWSHOT_JSON parses to {len(fs_model.links)} links, "
          f"root='{fs_model.root_link}'.")
    sys.exit(0)


if __name__ == "__main__":
    main()
