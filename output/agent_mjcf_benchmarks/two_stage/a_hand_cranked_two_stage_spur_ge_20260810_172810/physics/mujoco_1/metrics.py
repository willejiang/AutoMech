def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_shaft_rotation")
    out = joints.get("output_shaft_rotation")
    missing = []
    if inp is None:
        missing.append("input_shaft_rotation")
    if out is None:
        missing.append("output_shaft_rotation")
    if missing:
        return [{"name": "two_stage_reduction", "value": None, "expected": "nonzero output travel smaller than input travel", "passed": False, "detail": "Missing trajectory key(s): " + ", ".join(missing)}]
    if len(inp) < 2 or len(out) < 2:
        return [{"name": "two_stage_reduction", "value": None, "expected": "nonzero output travel smaller than input travel", "passed": False, "detail": "Insufficient trajectory samples"}]
    input_travel = abs(float(inp[-1]) - float(inp[0]))
    output_travel = abs(float(out[-1]) - float(out[0]))
    ratio = input_travel / output_travel if output_travel > 1e-9 else None
    passed = input_travel > 1.0 and output_travel > 0.03 and output_travel < 0.95 * input_travel
    value = {"input_travel_rad": input_travel, "output_travel_rad": output_travel, "observed_reduction": ratio}
    detail = "Input motion reached the output coupling with reduced angular travel." if passed else "The output did not show clear nonzero reduced motion relative to the hand-crank input."
    return [{"name": "two_stage_reduction", "value": value, "expected": "input travel > 1 rad and 0.03 rad < output travel < 0.95 times input travel; no exact reduction ratio was specified", "passed": passed, "detail": detail}]