"""Golden bounded agent MJCF compiler: topology-specific source, repair, manifest and cache."""
from __future__ import annotations

import json
import tempfile
from pathlib import Path

from build123d import Align, Cylinder, export_stl

from maker2.config import Settings
from maker2.llm.client import LLMResponse, ToolCall
from maker2.manager import save_model
from maker2.mjcf_agent_compiler import compile_agent_mjcf
from maker2.model import KinematicModel, LinkSpec, MotionJointSpec, PoseSpec, RunContext


class FakeClient:
    api_style = "openai"
    model = "fake-mjcf-agent"

    def __init__(self, source):
        self.source = source
        self.calls = 0
        self.step = 0

    def send_with_tools(self, messages, system="", tools=None):
        self.calls += 1
        sequence = [
            ToolCall("t1", "read_mjcf_facts", {"section": "index", "offset": 0, "limit": 25}),
            ToolCall("t2", "submit_compiler_source", {"source": self.source,
                                                       "expected_revision": 0}),
            ToolCall("t3", "run_mjcf_gate", {}),
        ]
        if self.step < len(sequence):
            call = sequence[self.step]; self.step += 1
            return LLMResponse("", [call], "tool_use")
        return LLMResponse("accepted", [], "end_turn")


def _source(entity_ids):
    return '''
def compile_mjcf(facts, out):
    out.topology_plan({"coordinate_map":{"shaft":"shaft_hinge"},"tree_edges":[],"closure_edges":[],"rigid_carried":[],"independent_coaxial":[],"transmissions":[],"contact_decisions":[],"support_ground":"base","support_strategy":[]})
    out.body("base")
    out.body("shaft")
    out.joint("shaft", "shaft_hinge", "hinge", (0,0,1), (0,0,0))
    for entity_id in facts["entity_ids"]:
        generated = ["base"] if entity_id in ("link/base", "pose/place_base") else ["shaft_hinge"] if entity_id in ("link/shaft", "pose/place_shaft", "motion_joint/shaft_hinge", "role/driver/shaft") else ["shaft"]
        out.decision(entity_id, "emitted" if entity_id.startswith(("link/", "motion_joint/")) else "represented_by", generated, "fixture topology decision", [entity_id])
'''


def main():
    run = Path(tempfile.mkdtemp(prefix="golden_mjcf_agent_")); (run/"meshes").mkdir()
    export_stl(Cylinder(8, 4, align=(Align.CENTER,Align.CENTER,Align.MIN)), run/"meshes/base.stl")
    export_stl(Cylinder(2, 12, align=(Align.CENTER,Align.CENTER,Align.MIN)), run/"meshes/shaft.stl")
    model = KinematicModel("agent_fixture", "base", [
        LinkSpec("base","base"), LinkSpec("shaft","shaft",dof="spin",driver=True)],
        [PoseSpec("place_base","","base"), PoseSpec("place_shaft","","shaft")],
        motion_joints=[MotionJointSpec("shaft_hinge","","shaft","hinge",(0,0,1))])
    save_model(model, run/"kinematic_model.json")
    ctx = RunContext(model.name,str(run),str(run/"model.urdf"),str(run/"meshes"),str(run),str(run/"kinematic_model.json"))
    client = FakeClient(_source([])); settings = Settings(
        mjcf_compiler_mode="agent", mjcf_compiler_cache_dir=str(run/"cache"))
    path = compile_agent_mjcf(model,ctx,settings=settings,client=client,log_fn=lambda *_:None)
    assert Path(path).exists() and client.calls >= 3
    manifest=json.loads((run/"builder_manifest.json").read_text())
    assert manifest["topology_plan"]["coordinate_map"]["shaft"]=="shaft_hinge"
    assert (run/"mjcf_gate_report.json").exists()
    # Same artifact hits the accepted compiler cache without an LLM request.
    cached = FakeClient(_source([]))
    compile_agent_mjcf(model,ctx,settings=settings,client=cached,log_fn=lambda *_:None)
    assert cached.calls == 0
    print("golden mjcf agent compiler: PASS")


if __name__ == "__main__": main()
