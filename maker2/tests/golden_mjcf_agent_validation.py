"""Golden acceptance gates never repair or infer agent topology decisions."""
from __future__ import annotations

import tempfile
from pathlib import Path

from build123d import Align, Box, export_stl

from maker2.mjcf_emitter import MJCFEmitter
from maker2.mjcf_validation import execute_compiler, validate_candidate


def facts(tmp):
    mesh=Path(tmp)/"meshes/a.stl";mesh.parent.mkdir()
    export_stl(Box(10,10,10,align=(Align.CENTER,Align.CENTER,Align.MIN)),mesh)
    frame={"xyz_m":[0,0,0],"quat_wxyz":[1,0,0,0],"matrix":[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}
    return {"run_dir":str(tmp),"entity_ids":["link/a"],
      "model":{"name":"x","root_link":"a","links":[{"name":"a"}]},
      "links":{"a":{"world_frame":frame,"mesh_path":"meshes/a.stl","mass_kg":1,
        "com_m":[0,0,0],"inertia_kg_m2":[[.1,0,0],[0,.1,0],[0,0,.1]],
        "friction":[1,.05,.005],"dof":"fixed"}},
      "simulation":{"gravity":[0,0,-9.81],"timestep":.001,"solver":"Newton","iterations":10}}


def main():
    tmp=Path(tempfile.mkdtemp(prefix="golden_mjcf_validation_")); f=facts(tmp)
    source='''
def compile_mjcf(facts,out):
    out.topology_plan({"support_ground":"a","coordinate_map":{},"tree_edges":[],"closure_edges":[],"rigid_carried":[],"independent_coaxial":[],"transmissions":[],"contact_decisions":[],"support_strategy":[]})
    out.body("a")
    out.decision("link/a","emitted",["a"],"root body",["link/a"])
'''
    xml,manifest=execute_compiler(source,f)
    report=validate_candidate(xml,manifest,f,tmp/"ok.mjcf",run_smoke=False)
    assert report["ok"],report
    before=xml
    bad=xml.replace('</worldbody>','<body name="extra"/></worldbody>')
    rejected=validate_candidate(bad,manifest,f,tmp/"bad.mjcf",run_smoke=False)
    assert not rejected["ok"] and not (tmp/"bad.mjcf").exists()
    assert xml==before and '<exclude' not in xml and '<equality' not in xml
    # Emitter ABI is driving -> driven, while MuJoCo's XML polynomial solves
    # joint1 from joint2. The emitted references must therefore be reversed.
    out=MJCFEmitter(f)
    out.body("a")
    out.joint("a","driver")
    out.joint("a","driven")
    out.joint_equality("reduction","driver","driven",-.25,
                       reason="fixture ratio",sources=["link/a"],fact_ids=["link/a"])
    equality=out.constraints["reduction"]
    assert equality.get("joint1")=="driven"
    assert equality.get("joint2")=="driver"
    assert equality.get("polycoef").split()[:2]==["0", "-0.25"]
    # World-frame anchors/axes are converted into the emitted body's local frame.
    world_frame=[[0,-1,0,1],[1,0,0,2],[0,0,1,3],[0,0,0,1]]
    f["links"]["a"]["world_frame"]={"xyz_m":[1,2,3],"quat_wxyz":[.70710678,0,0,.70710678],"matrix":world_frame}
    framed=MJCFEmitter(f)
    framed.topology_plan({"support_ground":"a","coordinate_map":{},"tree_edges":[],
                          "closure_edges":[],"rigid_carried":[],
                          "independent_coaxial":[],"transmissions":[],
                          "contact_decisions":[],"support_strategy":[]})
    framed.body("a")
    framed.joint("a","world_joint",axis=(1,0,0),pos_mm=(1000,2000,4000),frame="world")
    joint=framed.joints["world_joint"]
    assert [round(float(x),6) for x in joint.get("pos").split()]==[0.0,0.0,1.0]
    assert [round(float(x),6) for x in joint.get("axis").split()]==[0.0,-1.0,0.0]
    framed.exclude_ground("a","fixture may travel below plane",["link/a"],
                          ["dynamic/contact/a/world"])
    assert framed.geoms["a"].get("contype")=="1"
    assert framed.geoms["a"].get("conaffinity")=="2"
    _, framed_manifest=framed.finish()
    assert framed_manifest["ground_excludes"][0]["body"]=="a"
    unsafe='''
def compile_mjcf(facts,out):
    import os
'''
    try: execute_compiler(unsafe,f)
    except Exception as exc: assert "forbidden" in str(exc)
    else: raise AssertionError("unsafe compiler accepted")
    print("golden mjcf agent validation: PASS")


if __name__=="__main__":main()
