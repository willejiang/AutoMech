"""Golden optional Chrono behavior: explicit unavailable, never PyBullet fallback."""
from __future__ import annotations
import tempfile
from pathlib import Path
from build123d import Align, Box, export_stl
from maker2.chrono_backend import run_chrono_backend
from maker2.config import Settings
from maker2.manager import save_model
from maker2.model import KinematicModel, LinkSpec, PoseSpec


def main():
    run = Path(tempfile.mkdtemp(prefix="golden_chrono_optional_")); (run/"meshes").mkdir()
    export_stl(Box(10,10,10,align=(Align.CENTER,Align.CENTER,Align.MIN)),run/"meshes/base.stl")
    model=KinematicModel("chrono_optional","base",[LinkSpec("base","base")],
                         [PoseSpec("place_base","","base")])
    save_model(model,run/"kinematic_model.json")
    res=run_chrono_backend(model,str(run/"model.urdf"),"settle",str(run),
                           Settings(engine="chrono"),log_fn=lambda *_:None)
    assert res["engine"]=="chrono" and res["status"]=="unavailable"
    assert res["passed"] is None and res["verdict"]=="UNAVAILABLE"
    assert (run/"physics/chrono/builder_manifest.json").exists()
    assert res.get("cause") == "backend"
    print("golden chrono optional: PASS")

if __name__=="__main__": main()
