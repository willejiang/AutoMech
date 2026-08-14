def check(traj, result):
    import math
    joints = traj.get("joints", {})
    inp = joints.get("input_shaft_rotation")
    out = joints.get("output_shaft_rotation")
    if inp is None or out is None:
        missing = []
        if inp is None:
            missing.append("input_shaft_rotation")
        if out is None:
            missing.append("output_shaft_rotation")
        return [{"name": "output_reduction", "value": None, "expected": "nonzero output rotation slower than input", "passed": False, "detail": "Missing trajectory joint(s): " + ", ".join(missing)}]
    n = min(len(inp), len(out))
    if n < 2:
        return [{"name": "output_reduction", "value": None, "expected": "nonzero output rotation slower than input", "passed": False, "detail": "Insufficient trajectory samples"}]
    input_travel = 0.0
    output_travel = 0.0
    for i in range(1, n):
        di = float(inp[i]) - float(inp[i - 1])
        do = float(out[i]) - float(out[i - 1])
        input_travel += abs(math.atan2(math.sin(di), math.cos(di)))
        output_travel += abs(math.atan2(math.sin(do), math.cos(do)))
    ratio = input_travel / output_travel if output_travel > 1e-9 else None
    passed = input_travel > 1.0 and output_travel > 0.05 and ratio is not None and ratio > 1.05
    return [{"name": "output_reduction", "value": {"input_travel_rad": input_travel, "output_travel_rad": output_travel, "input_to_output_ratio": ratio}, "expected": "output travel > 0.05 rad and input/output travel ratio > 1.05; no exact reduction ratio was specified", "passed": passed, "detail": "Measures the output coupling through output_shaft_rotation against the hand crank through input_shaft_rotation."}]