"""Stage-1 of the Python-authoring pipeline (方案B): evaluate a MANAGER-authored
parametric CadQuery module into a KinematicModel with GLOBAL poses.

The manager no longer emits a connection-graph JSON that mate_solver solves. Instead it
writes ONE Python module that:
  * imports the boss parameter module (``params``) for all dimensions,
  * defines ``build_subassembly() -> cq.Assembly`` where every part is added to a
    ``cq.Assembly`` with a NAME, a global ``cq.Location`` (the part's placement in the
    subassembly's own frame), and a metadata dict carrying the KinematicModel fields the
    downstream needs: ``dof`` (fixed|spin|free), ``spin_axis``, ``driver``, and, for a
    gear, ``mesh_role``/``mesh_id`` so mesh_pairs can be recovered.

This module runs that authored Python in the SAME sandboxed subprocess pattern the cq
worker uses (never importing cadquery in-process), extracts each assembly child's world
transform + metadata + exports its STL, and assembles a ``KinematicModel`` whose poses are
GLOBAL (parent="" root-relative). Because the manager authored global coordinates, the
downstream libslvs cross-sub solve becomes unnecessary (see plan 方案B, assembler shrink).

The parent process only ever touches JSON + STL paths; cadquery/OCCT stays in the child.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from .model import KinematicModel, LinkSpec, PoseSpec

_EXEC_TIMEOUT = 240  # a whole-subassembly assembly solve is heavier than one part


class PyManagerError(ValueError):
    """The authored manager Python failed to evaluate into a valid subassembly."""


# Subprocess body: exec the authored module, build the cq.Assembly, solve it, then for
# every leaf child emit {name, world 4x4, metadata} + export its STL. Emits ONE json line.
_EVAL_RUNNER = r'''
import sys, json, math
src_path, out_json, meshes_dir = sys.argv[1], sys.argv[2], sys.argv[3]
import os
os.makedirs(meshes_dir, exist_ok=True)
ns = {}
try:
    import cadquery as cq
    # 方案B params access: `import params` binds to a proxy that, on a name the boss's params
    # module does NOT define, raises AttributeError carrying the list of names it DOES define —
    # so a manager that calls a functional-connection name wrong (e.g. params.inter_gear_pitch
    # vs inter_gear_pitch_dia) gets a corrective hint. It stays a plain AttributeError (NOT a
    # hard error) on purpose: per the 骨牌 design, params owns only functional-connection parts
    # (gear mesh pairs, bearing seats); subordinate parts (shaft body, spacer, collar) are
    # derived LOCALLY in the manager module and must NOT be forced through params. A loud
    # non-AttributeError here would wrongly punish that legitimate local derivation.
    import importlib.util as _ilu, os as _os
    _params_path = _os.path.join(_os.path.dirname(_os.path.abspath(src_path)), "params.py")
    if _os.path.exists(_params_path):
        _spec = _ilu.spec_from_file_location("_params_real", _params_path)
        _real = _ilu.module_from_spec(_spec)
        _spec.loader.exec_module(_real)

        class _ParamsProxy:
            """Read-through proxy over the real params module. Defined names resolve normally;
            an undefined name raises AttributeError with the available-names list so the
            manager can correct a mistyped functional-connection name. It does NOT force
            subordinate-part dimensions through params — those are derived locally."""
            def __getattr__(self, name):
                if name.startswith("__") and name.endswith("__"):
                    raise AttributeError(name)
                if hasattr(_real, name):
                    return getattr(_real, name)
                _pub = sorted(n for n in dir(_real) if not n.startswith("_"))
                raise AttributeError(
                    "module 'params' has no attribute '" + name + "'. params owns only "
                    "functional-connection quantities; it defines: " + ", ".join(_pub)
                    + ". Derive subordinate parts (shaft body, spacer, collar) LOCALLY.")
        sys.modules["params"] = _ParamsProxy()

    # 方案B 路B: inject a CANONICAL placement primitive so the manager NEVER hand-writes a
    # rotating cq.Location (which corrupts the params translation) NOR translates the geometry
    # itself (which leaves asm loc at origin, so pose extraction reads [0,0,0]). Build every
    # rotating part with its axis of revolution along LOCAL +Z, then call:
    #     place_part(asm, part, name="inter_gear1", axis=params.inter_gear1_axis(),
    #                xyz=params.inter_gear1(), metadata={...})
    # place_part rotates the part AT THE ORIGIN so local +Z aligns to `axis`, then adds it with a
    # PURE-TRANSLATION cq.Location(xyz). Orientation lives in the geometry, translation lives in
    # the assembly loc — exactly what pose extraction reads — so params coordinates land verbatim.
    def _axis_angle_z_to(axis):
        # rotation (unit axis, degrees) taking local +Z onto `axis`
        import math as _m
        ax = [float(axis[0]), float(axis[1]), float(axis[2])]
        n = _m.sqrt(sum(a*a for a in ax)) or 1.0
        ax = [a/n for a in ax]
        dot = max(-1.0, min(1.0, ax[2]))
        rx, ry, rz = (-ax[1], ax[0], 0.0)          # z(0,0,1) × axis = (-ax_y, ax_x, 0)
        rn = _m.sqrt(rx*rx + ry*ry + rz*rz)
        if rn < 1e-9:                               # parallel / antiparallel
            return (1.0, 0.0, 0.0), (0.0 if dot >= 0.0 else 180.0)
        return (rx/rn, ry/rn, rz/rn), _m.degrees(_m.acos(dot))

    def _place_part(asm, part, *, name, axis, xyz, metadata=None):
        # cq.Location(t, ax, angle) rotates about `ax` through the ORIGIN by `angle`, THEN
        # translates by `t`. So orientation and the params translation are independent: local +Z
        # is oriented onto `axis`, and the part lands exactly at the params coordinate `xyz`.
        rax, ang = _axis_angle_z_to(axis)
        loc = cq.Location(cq.Vector(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                          cq.Vector(*rax), ang)
        asm.add(part, name=name, loc=loc, metadata=(metadata or {}))
        return asm
    ns["place_part"] = _place_part

    # 方案B basis contract: EVERY revolution part is built along local +Z with its origin at the
    # -Z END FACE (cq's `extrude(length)` does exactly this). `place_axial` then places it by a
    # SEMANTIC anchor instead of a hand-computed `- axis*offset`: the manager declares WHICH point
    # of the part (base / center / top) coincides with the params frame, and the axial back-off is
    # derived deterministically from `length`. This kills the whole class of "manager mis-computed
    # the basis" bugs (e.g. a shaft placed by its center but sized as if placed by its base ->
    # sticks out past the wall). `length` MUST be the part's true extent along +Z, so a wrong
    # length surfaces immediately as an out-of-envelope part rather than a silent shift.
    def _place_axial(asm, part, *, name, axis, frame_xyz, length, anchor="base", metadata=None):
        import math as _m
        ax = [float(a) for a in axis]
        n = _m.sqrt(sum(a * a for a in ax)) or 1.0
        ax = [a / n for a in ax]
        try:
            back = {"base": 0.0, "center": float(length) / 2.0, "top": float(length)}[anchor]
        except KeyError:
            raise ValueError("place_axial anchor must be 'base', 'center', or 'top', got %r" % (anchor,))
        xyz = [float(frame_xyz[i]) - ax[i] * back for i in range(3)]
        md = dict(metadata or {})
        md["_axial"] = {"anchor": anchor, "length": float(length)}
        return _place_part(asm, part, name=name, axis=axis, xyz=xyz, metadata=md)
    ns["place_axial"] = _place_axial

    # Canonical gear generator. Managers kept emitting a plain `circle().extrude()` disk with NO
    # teeth (a smooth cylinder can't mesh and reads as a "gearless" reducer). make_gear builds a
    # real toothed spur gear DETERMINISTICALLY so the LLM never has to hand-roll involute math:
    # a root disk at the dedendum radius, then `teeth` trapezoidal teeth arrayed on the pitch
    # circle (a standard involute approximation that meshes correctly in a contact sim), a center
    # bore, extruded `face_width` along +Z with its origin at the -Z end face (so it drops straight
    # into place_axial with anchor="center"). module/teeth come from params; face_width/bore are
    # the manager's local call.
    def _make_gear(module, teeth, face_width, bore, *, pressure_angle_deg=20.0):
        import math as _m
        module = float(module); teeth = int(teeth)
        face_width = float(face_width); bore = float(bore)
        if teeth < 4 or module <= 0 or face_width <= 0:
            raise ValueError("make_gear needs module>0, teeth>=4, face_width>0 (got "
                             f"module={module}, teeth={teeth}, face_width={face_width})")
        pitch_r = module * teeth / 2.0
        addendum = module               # tip above pitch
        dedendum = 1.25 * module        # root below pitch
        tip_r = pitch_r + addendum
        root_r = max(pitch_r - dedendum, bore / 2.0 + 0.5)
        # tooth angular width at the pitch circle: half the circular pitch is tooth, half is gap
        tooth_ang = (_m.pi / teeth)     # radians of tooth arc at pitch circle (~half the pitch)
        half = tooth_ang / 2.0
        # trapezoid narrows toward the tip (approximates the involute flank)
        tip_half = half * 0.65
        # build the root cylinder, then union one tooth per position
        gear = cq.Workplane("XY").circle(root_r).extrude(face_width)
        for i in range(teeth):
            a = 2.0 * _m.pi * i / teeth
            poly = []
            for r, h in ((root_r - 0.01, half), (pitch_r, half),
                         (tip_r, tip_half), (tip_r, -tip_half),
                         (pitch_r, -half), (root_r - 0.01, -half)):
                poly.append((r * _m.cos(a + h), r * _m.sin(a + h)))
            tooth = cq.Workplane("XY").polyline(poly).close().extrude(face_width)
            gear = gear.union(tooth)
        if bore > 0:
            gear = gear.faces(">Z").workplane().hole(bore)
        return gear
    ns["make_gear"] = _make_gear

    with open(src_path, "r", encoding="utf-8") as f:
        code = f.read()
    exec(compile(code, src_path, "exec"), ns)
    fn = ns.get("build_subassembly")
    if fn is None:
        print(json.dumps({"ok": False, "error": "build_subassembly() not defined"}))
        sys.exit(0)
    asm = fn()
    if not hasattr(asm, "traverse"):
        print(json.dumps({"ok": False, "error": "build_subassembly() must return a cq.Assembly"}))
        sys.exit(0)
    # NB: do NOT call asm.solve(). Every part is positioned by place_part's loc= (absolute
    # params coordinates) with NO declared constraints. On a constraint-free multi-part
    # assembly cq's solver does not no-op — it RE-LAYS-OUT the parts (collapsing them toward a
    # default arrangement), which wiped the params Y/X coordinates (e.g. inter_gear1 y=112 -> 0).
    # The loc= placement is already the final global pose, so we read it directly.

    def _mat(loc):
        # world 4x4 (mm) of a cq.Location
        t = loc.toTuple() if hasattr(loc, "toTuple") else None
        # robust path: use the transform matrix
        m = loc.wrapped.Transformation()
        R = [[m.Value(r, c) for c in range(1, 4)] for r in range(1, 4)]
        T = [m.Value(r, 4) for r in range(1, 4)]
        return R, T

    parts = []
    root_name = None
    for child in asm.traverse():
        # asm.traverse() yields (name, Assembly) pairs; the top object has obj None-ish
        name, sub = child
        obj = getattr(sub, "obj", None)
        if obj is None:
            continue  # a grouping node with no solid
        loc = sub.loc
        R, Tv = _mat(loc)
        meta = dict(getattr(sub, "metadata", {}) or {})
        stl = os.path.join(meshes_dir, name + ".stl")
        shape = obj.val() if hasattr(obj, "val") else (obj.toCompound() if hasattr(obj, "toCompound") else obj)
        shape.exportStl(stl)
        bb = shape.BoundingBox()
        vol = 0.0
        try:
            vol = shape.Volume()
        except Exception:
            pass
        parts.append({"name": name, "R": R, "T": Tv, "metadata": meta,
                      "stl": os.path.relpath(stl, meshes_dir),
                      "volume_mm3": vol,
                      "bbox": [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax]})
        if root_name is None:
            root_name = name
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump({"ok": True, "parts": parts, "root": root_name}, f)
    print(json.dumps({"ok": True, "n_parts": len(parts)}))
except Exception as e:
    import traceback
    print(json.dumps({"ok": False, "error": (type(e).__name__ + ": " + str(e))[:400],
                      "trace": traceback.format_exc()[-800:]}))
'''


def _params_public_names(params_text: str) -> list:
    """Top-level public names (functions + constants) the params module defines, so an
    AttributeError can tell the manager exactly what it may call. Skips private `_` names."""
    import re as _re
    names = []
    for m in _re.finditer(r"^(?:def\s+([A-Za-z]\w*)\s*\(|([A-Za-z]\w*)\s*=)", params_text,
                          _re.MULTILINE):
        nm = m.group(1) or m.group(2)
        if nm and not nm.startswith("_") and nm not in names:
            names.append(nm)
    return names


def _rot_to_rpy(R):
    """3x3 rotation -> (roll, pitch, yaw) XYZ, radians. Best-effort, gimbal-safe enough."""
    import math
    sy = math.hypot(R[0][0], R[1][0])
    if sy > 1e-9:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0.0
    return (roll, pitch, yaw)


def evaluate_manager_python(script_text: str, run_dir: str, sub_name: str,
                            *, params_text: str = "", frames=None, log_fn=print) -> KinematicModel:
    """Run the manager-authored CadQuery module in a sandbox, export each part's STL, and
    build a KinematicModel whose poses are GLOBAL (root-relative, meters). Raises
    PyManagerError on any failure so the caller's retry/debug loop can react.

    ``frames`` (v3, optional): the boss's interface frames (list of MountFrame with
    ``name``/``xyz_m`` in global meters). When a built part's metadata tags a ``frame`` that
    matches one, we VERIFY the manager's params-derived location coincides with the boss's
    frame coordinate (a pure consistency guard — it does NOT overwrite the coordinate, per the
    'precheck backstops, does not override' choice). A drift beyond tolerance raises
    PyManagerError so the debugger rewrites the offending params call."""
    run = Path(run_dir)
    run.mkdir(parents=True, exist_ok=True)
    (run / "meshes").mkdir(exist_ok=True)
    # Persist params + the authored module side by side so `import params` resolves (cwd=run).
    # In 方案B the manager module always `import params`; an empty params_text means the boss
    # failed to emit the shared params block. Fail fast with a clear message instead of letting
    # every manager attempt collapse into an opaque `ModuleNotFoundError: No module named
    # 'params'` (which burns all retries). The boss-side gate (ERR_PARAMS_MISSING) should catch
    # this first; this is the backstop.
    if not (params_text or "").strip():
        raise PyManagerError(
            "no params module provided — the boss did not emit a ```python params block, so "
            "`import params` in the manager module would fail. Re-plan the boss to author params.")
    (run / "params.py").write_text(params_text, encoding="utf-8")
    src = run / "manager_sub.py"
    src.write_text(script_text, encoding="utf-8")
    runner = run / "_cq_eval_runner.py"
    runner.write_text(_EVAL_RUNNER, encoding="utf-8")
    out_json = run / "sub_eval.json"

    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        r = subprocess.run(
            [sys.executable, str(runner), str(src), str(out_json), str(run / "meshes")],
            capture_output=True, text=True, timeout=_EXEC_TIMEOUT, cwd=str(run), env=env)
    except subprocess.TimeoutExpired:
        raise PyManagerError(f"manager CadQuery eval timed out after {_EXEC_TIMEOUT}s")
    except Exception as e:
        raise PyManagerError(f"eval subprocess failed: {type(e).__name__}: {e}")

    payload = None
    for line in reversed((r.stdout or "").strip().splitlines()):
        line = line.strip()
        if line.startswith("{"):
            try:
                payload = json.loads(line)
                break
            except Exception:
                continue
    if payload is None or not payload.get("ok"):
        err = (payload or {}).get("error") if payload else None
        tail = (r.stderr or r.stdout or "").strip()[-400:]
        msg = f"manager CadQuery eval failed: {err or tail}"
        # If the manager called a params name that doesn't exist, tell it EXACTLY what params
        # DOES define so it fixes the call in one shot instead of guessing across retries.
        if err and "has no attribute" in err and params_text:
            names = _params_public_names(params_text)
            if names:
                msg += ("\nThe `params` module defines ONLY these names — call one of these (or "
                        "compose them with params.add/params.mul), never invent a params name: "
                        + ", ".join(names))
        raise PyManagerError(msg)
    if not out_json.exists():
        raise PyManagerError("manager eval produced no sub_eval.json")

    spec = json.loads(out_json.read_text(encoding="utf-8"))
    parts = spec.get("parts") or []
    if not parts:
        raise PyManagerError("manager assembly has no parts with solids")

    # v3 consistency guard: index the boss's interface frames by name so a part that tags a
    # `frame` in its metadata can be checked against the boss's authoritative coordinate.
    frames_by_name: dict = {}
    for fr in (frames or []):
        nm = getattr(fr, "name", None)
        if nm:
            frames_by_name[str(nm)] = fr
    _GUARD_TOL_M = 0.002  # 2 mm: params-derived loc must coincide with the boss frame

    links: list[LinkSpec] = []
    poses: list[PoseSpec] = []
    mesh_by_id: dict = {}
    coord_log: list = []
    root = spec.get("root") or parts[0]["name"]
    for p in parts:
        name = p["name"]
        meta = p.get("metadata") or {}
        if float(p.get("volume_mm3", 0.0)) <= 0.0:
            raise PyManagerError(f"part '{name}' built an empty/zero-volume solid")
        # basis contract: a part placed with place_axial MUST be modelled along local +Z with its
        # origin at the -Z END FACE, so its LOCAL bbox is z in [0, length]. If the manager instead
        # centered it (`.translate((0,0,-len/2))`) or built it off-origin, the anchor back-off is
        # computed against the wrong reference and the part shifts. Catch that here rather than as a
        # downstream overlap. (Only enforced for place_axial parts; place_part/non-axial are exempt.)
        _ax = meta.get("_axial")
        if _ax:
            bb = p.get("bbox") or [0, 0, 0, 0, 0, 0]
            zmin, zmax = float(bb[2]), float(bb[5])
            L = float(_ax.get("length", 0.0)) or (zmax - zmin)
            tol = max(0.5, 0.02 * L)         # 0.5 mm or 2% of length
            if abs(zmin) > tol or abs(zmax - L) > tol:
                raise PyManagerError(
                    f"part '{name}' uses place_axial(length={L:.1f}) but its LOCAL z-extent is "
                    f"[{zmin:.1f}, {zmax:.1f}] instead of [0, {L:.1f}] — build it along local +Z "
                    f"with its origin at the -Z end face (a bare `extrude(length)`; do NOT "
                    f"`.translate((0,0,-length/2))` to center it). place_axial derives the anchor "
                    f"back-off from `length`, so the geometry MUST start at z=0.")
        links.append(LinkSpec(
            name=name, description=meta.get("description", ""),
            shape_hint=meta.get("shape_hint", ""),
            mesh_filename=f"meshes/{name}.stl",
            dof=str(meta.get("dof", "fixed")),
            spin_axis=tuple(meta.get("spin_axis", (0.0, 0.0, 1.0))),
            driver=bool(meta.get("driver", False)),
            material=str(meta.get("material", "steel"))))
        # GLOBAL pose: root-relative. T is mm -> meters.
        T = [float(v) / 1000.0 for v in p["T"]]
        rpy = _rot_to_rpy(p["R"])
        # v3 guard: if the part declares which interface frame it realizes, verify the manager's
        # params-derived coordinate matches the boss's frame — a params call gone wrong (wrong
        # function, wrong axis) surfaces HERE as a clear per-sub error, not later as a precheck
        # weld gap. Pure check: the coordinate is NOT overwritten.
        fr_tag = meta.get("frame")
        if fr_tag and str(fr_tag) in frames_by_name:
            bf = frames_by_name[str(fr_tag)]
            bx = tuple(float(v) for v in getattr(bf, "xyz_m", (0.0, 0.0, 0.0)))
            drift = sum((a - b) ** 2 for a, b in zip(T, bx)) ** 0.5
            if drift > _GUARD_TOL_M:
                raise PyManagerError(
                    f"part '{name}' claims frame '{fr_tag}' but its params-derived location "
                    f"{tuple(round(v, 4) for v in T)} m is {drift*1000:.1f} mm from the boss "
                    f"frame {tuple(round(v, 4) for v in bx)} m — recompute its `loc` from "
                    f"`params.{fr_tag}()` (do not type a coordinate).")
            # 路B axis guard: the part is built with its revolution axis along local +Z, so its
            # REALIZED world axis is R·[0,0,1] = the 3rd column of R. It must match the frame's
            # axis (place_part orients from params.<frame>_axis()). A mismatch means the manager
            # oriented the part by hand / with a rotating Location instead of place_part.
            bax = getattr(bf, "axis", None)
            if bax is not None:
                bv = [float(v) for v in bax]
                bn = sum(v * v for v in bv) ** 0.5
                if bn > 1e-9:
                    bv = [v / bn for v in bv]
                    R = p["R"]
                    realized = [R[0][2], R[1][2], R[2][2]]
                    rn = sum(v * v for v in realized) ** 0.5 or 1.0
                    realized = [v / rn for v in realized]
                    cosang = sum(a * b for a, b in zip(realized, bv))
                    import math as _m
                    # axis of revolution is UNSIGNED: +v and -v are the same physical axis, so
                    # compare parallelism via |cos| (0° or 180° both mean aligned).
                    align_deg = _m.degrees(_m.acos(max(-1.0, min(1.0, abs(cosang)))))
                    if align_deg > 5.0:      # 5° tolerance
                        raise PyManagerError(
                            f"part '{name}' claims frame '{fr_tag}' but its realized spin axis "
                            f"{tuple(round(v,3) for v in realized)} is {align_deg:.0f}° off the "
                            f"frame axis {tuple(round(v,3) for v in bv)} — build the part with "
                            f"its axis along local +Z and place it with `place_part(asm, part, "
                            f"axis=params.{fr_tag}_axis(), xyz=params.{fr_tag}(), ...)`; do NOT "
                            f"hand-rotate it or pass a rotating cq.Location.")
        poses.append(PoseSpec(name=f"place_{name}", parent="", child=name,
                              xyz_m=tuple(T), rpy_rad=tuple(rpy)))
        coord_log.append(f"{name}@{tuple(round(v*1000, 1) for v in T)}mm")
        mid = meta.get("mesh_id")
        if mid:
            mesh_by_id.setdefault(str(mid), []).append(name)

    mesh_pairs = [tuple(v[:2]) for v in mesh_by_id.values() if len(v) >= 2]

    model = KinematicModel(name=sub_name, root_link=root, links=links, poses=poses,
                           mesh_pairs=mesh_pairs)
    if log_fn:
        log_fn(f"[py-manager] {sub_name}: {len(links)} part(s), {len(mesh_pairs)} mesh pair(s), "
               f"global poses from params: {', '.join(coord_log)}")
    return model
