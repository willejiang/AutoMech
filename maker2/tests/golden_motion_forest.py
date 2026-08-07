"""Golden generic moving-coordinate forest plus slider-crank non-regression.

Run: python -m maker2.tests.golden_motion_forest
"""
from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from build123d import Align, Box, export_stl

from maker2.manager import load_model, save_model
from maker2.mjcf_builder import build_mjcf
from maker2.model import (KinematicModel, LinkSpec, MotionJointSpec, PoseSpec,
                          RunContext)


def _ctx(model, run):
    return RunContext(model.name, str(run), str(run / "model.urdf"), str(run / "meshes"),
                      str(run), str(run / "kinematic_model.json"))


def _mesh(run, name, size=(8, 8, 8)):
    export_stl(Box(*size, align=(Align.CENTER, Align.CENTER, Align.MIN)),
               run / "meshes" / f"{name}.stl")


def main():
    run = Path(tempfile.mkdtemp(prefix="golden_motion_forest_")); (run / "meshes").mkdir()
    links = [LinkSpec("base", "base"), LinkSpec("shoulder", "shoulder", dof="spin"),
             LinkSpec("elbow", "elbow", dof="spin"), LinkSpec("wrist", "wrist", dof="spin")]
    for l in links: _mesh(run, l.name)
    model = KinematicModel("arm", "base", links, [
        PoseSpec("p0", "", "base"), PoseSpec("p1", "base", "shoulder", (0,0,.008)),
        PoseSpec("p2", "shoulder", "elbow", (.03,0,0)),
        PoseSpec("p3", "elbow", "wrist", (.025,0,0))], motion_joints=[
        MotionJointSpec("shoulder_hinge", "base", "shoulder", "hinge", (0,0,1)),
        MotionJointSpec("elbow_hinge", "shoulder", "elbow", "hinge", (0,1,0)),
        MotionJointSpec("wrist_hinge", "elbow", "wrist", "hinge", (1,0,0))])
    save_model(model, run / "kinematic_model.json")
    loaded = load_model(run / "kinematic_model.json")
    assert len(loaded.motion_joints) == 3
    xml = ET.parse(build_mjcf(loaded, _ctx(loaded, run), metrics={}, log_fn=lambda *_: None))
    world = xml.getroot().find("worldbody")
    shoulder = next(b for b in world.iter("body") if b.get("name") == "shoulder")
    elbow = next(b for b in shoulder.iter("body") if b.get("name") == "elbow")
    wrist = next(b for b in elbow.iter("body") if b.get("name") == "wrist")
    assert elbow is not shoulder and wrist is not elbow
    assert elbow.find("joint").get("name") == "elbow_hinge"
    assert wrist.find("joint").get("axis") == "1 0 0"
    print("golden motion forest: PASS")


if __name__ == "__main__": main()
