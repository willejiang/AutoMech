"""Golden fixed-ring planetary lowering and mesh-exemption completeness.

Run: python -m maker2.tests.golden_planetary_stage
"""
from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from build123d import Align, Cylinder, export_stl

from maker2.manager import save_model
from maker2.mjcf_builder import build_mjcf
from maker2.model import (KinematicModel, LinkSpec, MotionJointSpec, PlanetaryStageSpec,
                          PoseSpec, RunContext)


def main():
    run=Path(tempfile.mkdtemp(prefix="golden_planetary_")); (run/"meshes").mkdir()
    names=["base","sun","ring","carrier","p1","p2","p3"]
    for n in names: export_stl(Cylinder(8 if n.startswith('p') else 12,5,align=(Align.CENTER,Align.CENTER,Align.MIN)),run/"meshes"/f"{n}.stl")
    links=[LinkSpec("base","base"),LinkSpec("sun","sun",dof="spin",driver=True),
           LinkSpec("ring","ring"),LinkSpec("carrier","carrier",dof="spin")]+[
           LinkSpec(f"p{i}",f"p{i}",dof="spin") for i in range(1,4)]
    poses=[PoseSpec("b","","base"),PoseSpec("s","","sun"),PoseSpec("r","","ring"),PoseSpec("c","","carrier")]
    coords=[(.018,0,0),(-.009,.015588,0),(-.009,-.015588,0)]
    poses += [PoseSpec(f"pp{i}","carrier",f"p{i}",xyz) for i,xyz in enumerate(coords,1)]
    joints=[MotionJointSpec("sun_h","","sun"),MotionJointSpec("carrier_h","","carrier")]+[
        MotionJointSpec(f"p{i}_h","carrier",f"p{i}") for i in range(1,4)]
    stage=PlanetaryStageSpec("stage","sun","ring","carrier",
        [{"gear":f"p{i}","pin":""} for i in range(1,4)],18,18,54)
    model=KinematicModel("planetary","base",links,poses,motion_joints=joints,planetary_stages=[stage])
    save_model(model,run/"kinematic_model.json")
    ctx=RunContext(model.name,str(run),str(run/"model.urdf"),str(run/"meshes"),str(run),str(run/"kinematic_model.json"))
    metrics={}; root=ET.parse(build_mjcf(model,ctx,metrics=metrics,log_fn=lambda *_:None)).getroot()
    carrier=next(b for b in root.iter("body") if b.get("name")=="carrier")
    assert all(any(b.get("name")==f"p{i}" for b in carrier.iter("body")) for i in range(1,4))
    planetary=[j for j in root.findall("./equality/joint") if j.get("joint1") in {"carrier_h","p1_h","p2_h","p3_h"}]
    assert len(planetary)==4,[(j.get("joint1"),j.get("joint2")) for j in planetary]
    pairs={frozenset((e.get("body1"),e.get("body2"))) for e in root.findall("./contact/exclude")}
    expected={frozenset((x,f"p{i}")) for x in ("sun","ring") for i in range(1,4)}
    assert expected <= pairs and metrics["planetary_mesh_pairs"]==6
    print("golden planetary stage: PASS")


if __name__=="__main__": main()
