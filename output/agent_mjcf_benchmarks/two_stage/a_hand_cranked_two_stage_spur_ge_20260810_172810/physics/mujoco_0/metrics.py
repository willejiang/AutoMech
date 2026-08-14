def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_shaft_rotation")
    out = joints.get("output_shaft_rotation")
    if inp is None or out is None:
        missing = []
        if inp is None:
            missing.append("input_shaft_rotation")
        if out is None:
            missing.append("output_shaft_rotation")
        return [{"name": "two_stage_output_reduction", "value": 0.0, "expected": "output coupling rotates with less angular travel than the hand-crank input", "passed": False, "detail": "Missing trajectory joint(s): " + ", ".join(missing)}]
    if len(inp) < 2 or len(out) < 2:
        return [{"name": "two_stage_output_reduction", "value": 0.0, "expected": "output coupling rotates with less angular travel than the hand-crank input", "passed": False, "detail": "Insufficient trajectory samples"}]
    input_travel = abs(float(inp[-1]) - float(inp[0]))
    output_travel = abs(float(out[-1]) - float(out[0]))
    ratio = input_travel / output_travel if output_travel > 1e-9 else 0.0
    passed = input_travel > 0.1 and output_travel > 0.05 and ratio > 1.001
    detail = "input travel %.4f rad, output-coupling travel %.4f rad, observed reduction %.4f:1; no exact ratio was specified" % (input_travel, output_travel, ratio)
    return [{"name": "two_stage_output_reduction", "value": ratio, "expected": "output coupling rotates and input/output angular-travel ratio is greater than 1", "passed": passed, "detail": detail}]