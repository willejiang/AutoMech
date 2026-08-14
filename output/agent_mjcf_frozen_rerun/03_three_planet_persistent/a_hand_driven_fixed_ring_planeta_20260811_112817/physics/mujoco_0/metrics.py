def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_hinge")
    out = joints.get("carrier_hinge")
    expected = "0.01 <= signed carrier travel / signed input travel <= 0.95"
    if inp is None or out is None or len(inp) < 2 or len(out) < 2:
        return [{"name": "carrier_reduction_ratio", "value": None, "expected": expected, "passed": False, "detail": "Missing or insufficient input_hinge or carrier_hinge trajectory data."}]
    n = min(len(inp), len(out))
    def travel(values):
        total = 0.0
        for i in range(1, n):
            delta = float(values[i]) - float(values[i - 1])
            while delta > 3.141592653589793:
                delta -= 6.283185307179586
            while delta < -3.141592653589793:
                delta += 6.283185307179586
            total += delta
        return total
    input_travel = travel(inp)
    carrier_travel = travel(out)
    if abs(input_travel) < 0.1:
        return [{"name": "carrier_reduction_ratio", "value": None, "expected": expected, "passed": False, "detail": "Input travel was below 0.1 rad, so transmission could not be evaluated."}]
    ratio = carrier_travel / input_travel
    passed = ratio >= 0.01 and ratio <= 0.95 and abs(carrier_travel) >= 0.05
    return [{"name": "carrier_reduction_ratio", "value": ratio, "expected": expected, "passed": passed, "detail": "input_travel=%.6f rad, carrier_travel=%.6f rad, signed_ratio=%.6f. A positive ratio denotes same-direction motion; a ratio below one denotes reduction. The task specifies no exact tooth-count ratio." % (input_travel, carrier_travel, ratio)}]