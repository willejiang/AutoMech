import sys, json, math
src_path, out_json, meshes_dir = sys.argv[1], sys.argv[2], sys.argv[3]
import os
os.makedirs(meshes_dir, exist_ok=True)
ns = {}
try:
    import build123d as b3d
    from build123d import (BuildPart, BuildSketch, Cylinder, Box, Circle, Polygon,
                           extrude, Location, Plane, Align, Mode, export_stl)
    from cadpy.assembly import AssemblyHelper
    # 方案B params access: proxy raises AttributeError (with available names) on an undefined
    # params name; subordinate parts are derived locally and must NOT be forced through params.
    import importlib.util as _ilu, os as _os
    _params_path = _os.path.join(_os.path.dirname(_os.path.abspath(src_path)), "params.py")
    if _os.path.exists(_params_path):
        _spec = _ilu.spec_from_file_location("_params_real", _params_path)
        _real = _ilu.module_from_spec(_spec)
        _spec.loader.exec_module(_real)

        class _ParamsProxy:
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

    # make_gear: real toothed spur gear along local +Z, origin at -Z end face (so a cadpy
    # rigid_frame at Location((0,0,0)) is its base, ((0,0,fw/2)) its mid-plane).
    def _make_gear(module, teeth, face_width, bore, pressure_angle_deg=20.0):
        module = float(module); teeth = int(teeth)
        face_width = float(face_width); bore = float(bore)
        if teeth < 4 or module <= 0 or face_width <= 0:
            raise ValueError("make_gear needs module>0, teeth>=4, face_width>0 (got "
                             "module=%r teeth=%r face_width=%r)" % (module, teeth, face_width))
        pitch_r = module * teeth / 2.0
        tip_r = pitch_r + module
        root_r = max(pitch_r - 1.25 * module, bore / 2.0 + 0.5)
        half = (math.pi / teeth) / 2.0
        tip_half = half * 0.65
        with BuildPart() as gp:
            with BuildSketch(Plane.XY):
                Circle(root_r)
                for i in range(teeth):
                    a = 2.0 * math.pi * i / teeth
                    pts = []
                    for r, h in ((root_r - 0.01, half), (pitch_r, half),
                                 (tip_r, tip_half), (tip_r, -tip_half),
                                 (pitch_r, -half), (root_r - 0.01, -half)):
                        pts.append((r * math.cos(a + h), r * math.sin(a + h)))
                    Polygon(*pts, mode=Mode.ADD)
            extrude(amount=face_width)
            if bore > 0:
                with BuildSketch(Plane.XY) as _bh:
                    Circle(bore / 2.0)
                extrude(to_extrude=_bh.sketch, amount=face_width, mode=Mode.SUBTRACT)
        return gp.part
    ns["make_gear"] = _make_gear
    ns["b3d"] = b3d
    for _n in ("BuildPart", "BuildSketch", "Cylinder", "Box", "Circle", "Polygon",
               "extrude", "Location", "Plane", "Align", "Mode"):
        ns[_n] = getattr(b3d, _n)
    ns["AssemblyHelper"] = AssemblyHelper

    with open(src_path, "r", encoding="utf-8") as f:
        code = f.read()
    exec(compile(code, src_path, "exec"), ns)
    fn = ns.get("build_subassembly")
    if fn is None:
        print(json.dumps({"ok": False, "error": "build_subassembly() not defined"}))
        sys.exit(0)
    comp = fn()
    if hasattr(comp, "build"):
        comp = comp.build()
    if not hasattr(comp, "children"):
        print(json.dumps({"ok": False,
                          "error": "build_subassembly() must return a cadpy AssemblyHelper or a build123d Compound"}))
        sys.exit(0)

    def _leaves(c):
        kids = list(getattr(c, "children", []) or [])
        if kids:
            for ch in kids:
                for x in _leaves(ch):
                    yield x
        else:
            yield c

    def _RT(loc):
        try:
            tr = loc.wrapped.Transformation()
            R = [[tr.Value(r, c) for c in range(1, 4)] for r in range(1, 4)]
            T = [tr.Value(r, 4) for r in range(1, 4)]
            return R, T
        except Exception:
            p = tuple(loc.position)
            return [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [p[0], p[1], p[2]]

    parts = []
    root_name = None
    for leaf in _leaves(comp):
        name = (getattr(leaf, "label", "") or "").strip() or ("part_%d" % len(parts))
        # WORLD pose: a manager may wrap the whole sub in a Compound and `.moved(...)` it to its
        # global params frame; a leaf's own `.location` is then only relative to that parent, so we
        # MUST read the ACCUMULATED world transform. build123d exposes it as `global_location`.
        wloc = getattr(leaf, "global_location", None) or leaf.location
        R, Tv = _RT(wloc)
        meta = dict(getattr(leaf, "cadpy_metadata", {}) or {})
        stl = os.path.join(meshes_dir, name + ".stl")
        # The pose (R,Tv) above is the FULL local->world transform for this leaf. The STL and bbox
        # must therefore be the leaf's PURE LOCAL geometry (its own placement stripped), so the
        # downstream `pose applied to local mesh` gives the correct world position ONCE. Exporting
        # the leaf as-is bakes its in-sub station (e.g. a gear at local z=70) into the mesh AND
        # leaves it in the pose — double-counted, so every part drifts along the axis by its station
        # distance (the planetary sun/planets/spacers "not through the shaft" bug). Strip it here.
        try:
            local_leaf = leaf.moved(leaf.location.inverse())
        except Exception:
            local_leaf = leaf
        try:
            export_stl(local_leaf, stl)
        except Exception:
            export_stl(b3d.Solid(local_leaf.wrapped) if hasattr(local_leaf, "wrapped") else local_leaf, stl)
        try:
            vol = float(leaf.volume)
        except Exception:
            vol = 0.0
        if vol <= 0.0:
            continue
        bb = local_leaf.bounding_box()
        parts.append({"name": name, "R": R, "T": Tv, "metadata": meta,
                      "stl": os.path.relpath(stl, meshes_dir),
                      "volume_mm3": vol,
                      "bbox": [bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z]})
        if root_name is None:
            root_name = name
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump({"ok": True, "parts": parts, "root": root_name}, f)
    # Export the WHOLE sub as one STEP next to sub_eval.json, so the skill's inspect tool can run
    # single-file selector-level precision checks on it. Best-effort: a STEP failure must not fail
    # the eval (geocheck still gates), so swallow and report step:null.
    step_out = os.path.join(os.path.dirname(out_json), "sub.step")
    step_ok = False
    try:
        from build123d import export_step as _export_step
        _export_step(comp, step_out)
        step_ok = os.path.exists(step_out)
    except Exception:
        step_ok = False
    print(json.dumps({"ok": True, "n_parts": len(parts),
                      "step": step_out if step_ok else None}))
except Exception as e:
    import traceback
    print(json.dumps({"ok": False, "error": (type(e).__name__ + ": " + str(e))[:400],
                      "trace": traceback.format_exc()[-800:]}))
