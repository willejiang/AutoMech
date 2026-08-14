def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_shaft_hinge")
    out = joints.get("output_sleeve_hinge")
    if inp is None or out is None:
        return [{"name": "planetary_reduction_ratio", "value": None, "expected": "4.0:1 input-to-carrier, same direction", "passed": False, "detail": "Missing input_shaft_hinge or output_sleeve_hinge trajectory."}]
    if len(inp) < 2 or len(out) != len(inp):
        return [{"name": "planetary_reduction_ratio", "value": None, "expected": "4.0:1 input-to-carrier, same direction", "passed": False, "detail": "Trajectory is absent, too short, or length-mismatched."}]
    pi = 3.141592653589793
    two_pi = 2.0 * pi
    input_travel = 0.0
    output_travel = 0.0
    for i in range(1, len(inp)):
        di = inp[i] - inp[i - 1]
        do = out[i] - out[i - 1]
        while di > pi:
            di -= two_pi
        while di < -pi:
            di += two_pi
        while do > pi:
            do -= two_pi
        while do < -pi:
            do += two_pi
        input_travel += di
        output_travel += do
    if abs(input_travel) < two_pi or abs(output_travel) < 0.25:
        return [{"name": "planetary_reduction_ratio", "value": None, "expected": "4.0:1 input-to-carrier, same direction", "passed": False, "detail": "Insufficient sustained input or carrier-output motion to establish transmission."}]
    ratio = input_travel / output_travel
    passed = ratio > 0.0 and abs(ratio - 4.0) <= 0.08
    return [{"name": "planetary_reduction_ratio", "value": ratio, "expected": "4.0 +/- 0.08 input-to-carrier, same direction", "passed": passed, "detail": "Computed from total input_shaft_hinge travel divided by total output_sleeve_hinge travel."}]