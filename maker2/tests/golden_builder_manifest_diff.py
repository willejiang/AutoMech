"""Golden authored IR versus builder manifest fidelity probe."""
from evaluator.attribution import compare_authored_ir_compiled
from maker2.model import KinematicModel, LinkSpec, MotionJointSpec, PoseSpec


def main():
    model=KinematicModel("x","base",[LinkSpec("base","base"),LinkSpec("arm","arm",dof="spin")],
        [PoseSpec("b","","base"),PoseSpec("a","base","arm")],
        motion_joints=[MotionJointSpec("arm_hinge","base","arm")])
    good={"bodies":[{"source_kind":"link","source_name":"base","compiled":True},
                     {"source_kind":"link","source_name":"arm","compiled":True}],
          "constraints":[{"source_kind":"motion_joint","source_name":"arm_hinge","compiled":True}],
          "transmissions":[],"planetary_stages":[]}
    assert compare_authored_ir_compiled(model,good)["ok"]
    bad={**good,"constraints":[]}
    diff=compare_authored_ir_compiled(model,bad)
    assert not diff["ok"] and diff["missing"]==[{"kind":"motion_joint","name":"arm_hinge"}]
    print("golden builder manifest diff: PASS")

if __name__=="__main__": main()
