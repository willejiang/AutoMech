"""Isolation test #3: prove the URDF contract end (hand-built, minimal LLM-free).

Builds a 2-link table KinematicModel by hand, writes the URDF, scaffolds empty
meshes, then fills them with hardcoded FreeCAD primitives via freecad_runner and
checks the assembled URDF loads with every mesh resolved.

Run:  py -3.14 tests/isolation_urdf_table.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from workflow4freecad.model import LinkSpec, JointSpec, KinematicModel, RunContext
from workflow4freecad.urdf_builder import build_urdf, scaffold_meshes, validate_urdf
from workflow4freecad.freecad_runner import find_freecadcmd, run_body


TOP_BODY = """\
box = doc.addObject("Part::Box", "Top")
box.Length = 400.0
box.Width = 400.0
box.Height = 20.0
doc.recompute()
__result_obj__ = box
"""

LEG_BODY = """\
cyl = doc.addObject("Part::Cylinder", "Leg")
cyl.Radius = 20.0
cyl.Height = 500.0
doc.recompute()
__result_obj__ = cyl
"""


def make_ctx(base: str) -> RunContext:
    run_dir = os.path.join(base, "table_run")
    return RunContext(
        project_slug="table",
        run_dir=run_dir,
        urdf_path=os.path.join(run_dir, "model.urdf"),
        meshes_dir=os.path.join(run_dir, "meshes"),
        logs_dir=os.path.join(run_dir, "logs"),
        model_json_path=os.path.join(run_dir, "kinematic_model.json"),
    )


def main() -> int:
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_isolation_out")
    ctx = make_ctx(base)
    os.makedirs(ctx.logs_dir, exist_ok=True)

    model = KinematicModel(
        name="table", root_link="top",
        links=[
            LinkSpec("top", "square table top", shape_hint="box",
                     size_mm={"x": 400, "y": 400, "z": 20},
                     mesh_filename="meshes/top.stl"),
            LinkSpec("leg", "central cylindrical leg", shape_hint="cylinder",
                     size_mm={"radius": 20, "height": 500},
                     mesh_filename="meshes/leg.stl"),
        ],
        joints=[JointSpec("top_to_leg", "fixed", "top", "leg",
                          xyz_m=(0.0, 0.0, -0.5))],
    )

    ok = True

    # 1. build URDF + XML content checks
    build_urdf(model, ctx)
    xml = open(ctx.urdf_path, encoding="utf-8").read()
    has_scale = "0.001 0.001 0.001" in xml
    has_leg = 'filename="meshes/leg.stl"' in xml
    print("1. URDF written: scale=%s leg_mesh=%s" % (has_scale, has_leg))
    ok = ok and has_scale and has_leg

    # 2. scaffold placeholders
    paths = scaffold_meshes(model, ctx)
    placeholders_ok = all(os.path.isfile(p) and os.path.getsize(p) == 0
                          for p in paths.values())
    print("2. scaffold: %d placeholders, all 0-byte=%s"
          % (len(paths), placeholders_ok))
    ok = ok and placeholders_ok and len(paths) == 2

    # 3. pre-fill topology validation (no meshes required)
    topo_ok, topo_err = validate_urdf(ctx.urdf_path, require_meshes=False)
    print("3. pre-fill topology valid=%s %s" % (topo_ok, topo_err))
    ok = ok and topo_ok

    # 4. empty meshes should NOT pass a require_meshes load (observational)
    empty_ok, empty_err = validate_urdf(ctx.urdf_path, require_meshes=True)
    print("4. empty-mesh require_meshes load passed=%s (expect False) %s"
          % (empty_ok, empty_err[:120]))

    # 5. fill the two meshes via freecadcmd (no LLM)
    freecadcmd = find_freecadcmd()
    for link_name, body in (("top", TOP_BODY), ("leg", LEG_BODY)):
        stl = paths[link_name]
        res = run_body(
            freecadcmd, body, stl_path=stl,
            result_path=os.path.join(ctx.logs_dir, f"{link_name}.result.json"),
            script_path=os.path.join(ctx.logs_dir, f"{link_name}.script.py"),
            timeout=120,
        )
        size = os.path.getsize(stl) if os.path.isfile(stl) else 0
        print("5. fill %-4s ok=%s size=%d %s"
              % (link_name, res.ok, size, res.error[:120]))
        ok = ok and res.ok and size > 0

    # 6. post-fill: every mesh must resolve + load
    full_ok, full_err = validate_urdf(ctx.urdf_path, require_meshes=True)
    print("6. post-fill require_meshes load valid=%s %s" % (full_ok, full_err[:200]))
    ok = ok and full_ok

    # 7. best-effort PNG render (needs OpenGL; never fails the test)
    try:
        from workflow4freecad.viz import render_png
        png = render_png(ctx.urdf_path, os.path.join(ctx.run_dir, "preview.png"))
        print("7. render_png OK ->", png, "(%d bytes)" % os.path.getsize(png))
    except Exception as e:
        print("7. render_png skipped (%s: %s)" % (type(e).__name__, str(e)[:100]))

    print("\n%s: URDF contract end" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
