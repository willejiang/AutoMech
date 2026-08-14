def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_shaft_rotation")
    out = joints.get("output_shaft_rotation")
    if inp is None or out is None or len(inp) < 2 or len(out) < 2:
        return [{"name": "output_reduction", "value": None, "expected": "output coupling rotates more slowly in the same direction as the crank", "passed": False, "detail": "Missing or insufficient input_shaft_rotation/output_shaft_rotation trajectory data."}]
    n = min(len(inp), len(out))
    def travel(values, count):
        total = 0.0
        for i in range(1, count):
            delta = float(values[i]) - float(values[i - 1])
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            total += delta
        return total
    di = travel(inp, n)
    do = travel(out, n)
    ai = abs(di)
    ao = abs(do)
    passed = ai > 0.1 and ao > 0.001 and ao < 0.98 * ai and di * do > 0.0
    ratio = ai / ao if ao > 1e-12 else None
    return [{"name": "output_reduction", "value": {"input_travel_rad": di, "output_travel_rad": do, "observed_reduction": ratio}, "expected": "nonzero output-coupling travel in the input direction after two external meshes, with output angular travel smaller than input; no exact reduction ratio was specified", "passed": passed, "detail": "Uses the simulator trajectory keys input_shaft_rotation and output_shaft_rotation; the latter carries the user-visible output coupling."}]