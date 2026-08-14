def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_spin")
    out = joints.get("output_spin")
    if inp is None or out is None or len(inp) < 2 or len(out) < 2:
        return [{"name": "overall_reduction", "value": None, "expected": "9:1 input-to-output angular travel", "passed": False, "detail": "Missing input_spin or output_spin trajectory."}]

    def travel(values):
        total = 0.0
        for a, b in zip(values[:-1], values[1:]):
            delta = b - a
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            total += delta
        return total

    input_travel = travel(inp)
    output_travel = travel(out)
    if abs(input_travel) < 2.0 or abs(output_travel) < 0.05:
        return [{"name": "overall_reduction", "value": None, "expected": "9:1 input-to-output angular travel", "passed": False, "detail": "Insufficient transmitted motion to measure the reducer ratio."}]

    signed_ratio = input_travel / output_travel
    passed = signed_ratio > 0.0 and abs(signed_ratio - 9.0) <= 0.27
    return [{"name": "overall_reduction", "value": signed_ratio, "expected": "9.0 with input and output rotating in the same direction after two external meshes (tolerance +/-0.27)", "passed": passed, "detail": "Measured from input_spin angular travel divided by the delivered output_spin angular travel."}]