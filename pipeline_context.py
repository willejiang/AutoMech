"""Single source of the pipeline diagram, prepended to EVERY sub-agent's system
prompt so each agent knows where it sits in the two-loop flow and what runs before
and after it. Import: `from pipeline_context import PIPELINE` (both evaluator/ and
orchestrator/ insert the repo root on sys.path)."""

PIPELINE = """\
=== PhysCAD pipeline (you are ONE stage; know the whole flow) ===
USER TASK
  -> [worker] prompt -> OpenSCAD geometry
  -> [visual gate] 6-view VLM: looks right & matches prompt?  (no->back to worker)
  -> [urdf author] VLM reasons geometry+renders -> joints/axes/density -> manifest
  -> [strategy selector] pick strategy (static/scripted/RL) + sim backend
        (pybullet=CPU rigid · isaac_sim=GPU robotics · openfoam=CFD/aero)
  -> [scenario designer] task+robot -> sim spec (pose, control, pass criteria)
  -> [simulate] chosen backend -> frames + metrics (sim_result.json)
  -> [VLM judge] watch frames under the task -> PASS, or FAIL+fix_hint -> worker
  -> PASS -> deliverable. The camera/VLM judge is the trusted gate, not raw metrics.
Make your local decision so the NEXT stage gets clean input.
================================================================
"""
