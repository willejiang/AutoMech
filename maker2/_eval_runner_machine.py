import sys, json, math, os
src_path, out_json, meshes_dir = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(meshes_dir, exist_ok=True)
ns = {"__file__": os.path.abspath(src_path), "__name__": "__physcad_machine__"}
try:
    import build123d as b3d
    from build123d import (BuildPart, BuildSketch, Cylinder, Box, Circle, Polygon,
                           extrude, Location, Plane, Align, Mode, export_stl)
    from cadpy.assembly import AssemblyHelper

    # make_gear: real toothed spur gear along local +Z, origin at -Z end face.
    def _make_gear(module, teeth, face_width, bore, pressure_angle_deg=20.0):
        module = float(module); teeth = int(teeth)
        face_width = float(face_width); bore = float(bore)
        if teeth < 4 or module <= 0 or face_width <= 0:
            raise ValueError("make_gear needs module>0, teeth>=4, face_width>0")
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
               "extrude", "Location", "Plane", "Align", "Mode", "add"):
        ns[_n] = getattr(b3d, _n)
    ns["AssemblyHelper"] = AssemblyHelper

    with open(src_path, "r", encoding="utf-8") as f:
        code = f.read()
    exec(compile(code, src_path, "exec"), ns)
    mechanism = ns.get("MECHANISM") if isinstance(ns.get("MECHANISM"), dict) else {}
    # Single-agent path: the WHOLE machine is one build_machine() (no boss/sub split).
    fn = ns.get("build_machine") or ns.get("build_subassembly")
    if fn is None:
        print(json.dumps({"ok": False, "error": "build_machine() not defined"}))
        sys.exit(0)
    comp = fn()
    if hasattr(comp, "build"):
        comp = comp.build()
    if not hasattr(comp, "children"):
        print(json.dumps({"ok": False,
                          "error": "build_machine() must return a cadpy AssemblyHelper or a build123d Compound"}))
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
        raw = (getattr(leaf, "label", "") or "").strip()
        segs = [s.strip() for s in raw.split("|") if s.strip()]
        name = (segs[0] if segs else "") or ("part_%d" % len(parts))
        label_meta = {}
        _AXIS_VEC = {"x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "z": [0.0, 0.0, 1.0]}
        for seg in segs[1:]:
            if "=" in seg:
                k, v = seg.split("=", 1)
                k = k.strip(); v = v.strip()
                if k in ("spin_axis", "slide_axis") and v.lower() in _AXIS_VEC:
                    label_meta[k] = list(_AXIS_VEC[v.lower()])
                elif v in ("True", "true", "False", "false"):
                    label_meta[k] = v.lower() == "true"
                else:
                    label_meta[k] = v
        wloc = getattr(leaf, "global_location", None) or leaf.location
        R, Tv = _RT(wloc)
        meta = dict(label_meta)
        meta.update(dict(getattr(leaf, "cadpy_metadata", {}) or {}))
        stl = os.path.join(meshes_dir, name + ".stl")
        # Strip the leaf's own placement so the STL is PURE LOCAL geometry (pose carries world).
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
    payload = {"ok": True, "parts": parts, "root": root_name}
    mech = {}
    for k in ("ports_by_link", "relations", "motion_joints", "transmissions",
              "planetary_stages", "output_link", "watch_links"):
        if k in mechanism:
            mech[k] = mechanism[k]
    if mech:
        payload["mechanism"] = mech
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(payload, f)
    # Export the WHOLE machine as one STEP so the text-to-cad inspect tool can run
    # selector-level precision checks on it (the self-check loop).
    step_out = os.path.join(os.path.dirname(out_json), "machine.step")
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
