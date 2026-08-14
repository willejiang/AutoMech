def check(traj, result):
    joints = traj.get("joints", {})
    required = ["input_shaft_hinge", "idler_shaft_hinge", "output_shaft_hinge"]
    missing = [name for name in required if name not in joints or len(joints[name]) < 2]
    if missing:
        return [{"name": "signed_output_input_ratio", "value": None, "expected": "+1.0 (output same direction, equal angular travel)", "passed": False, "detail": "Missing trajectory data for: " + ", ".join(missing)}]

    def travel(values):
        total = 0.0
        for a, b in zip(values[:-1], values[1:]):
            step = b - a
            while step > math.pi:
                step -= 2.0 * math.pi
            while step < -math.pi:
                step += 2.0 * math.pi
            total += step
        return total

    inp = travel(joints["input_shaft_hinge"])
    idle = travel(joints["idler_shaft_hinge"])
    out = travel(joints["output_shaft_hinge"])
    if abs(inp) < 1.0:
        return [{"name": "signed_output_input_ratio", "value": None, "expected": "+1.0 (output same direction, equal angular travel)", "passed": False, "detail": "Input shaft travel was too small for a valid transmission test: %.4f rad." % inp}]

    ratio = out / inp
    passed = abs(out) >= 0.8 and 0.92 <= ratio <= 1.08
    detail = "input_shaft_hinge=%.4f rad, idler_shaft_hinge=%.4f rad, output_shaft_hinge=%.4f rad; signed output/input ratio=%.4f. Two external meshes should make the idler oppose the input while the output returns to the input direction." % (inp, idle, out, ratio)
    return [{"name": "signed_output_input_ratio", "value": ratio, "expected": "+1.0 within [0.92, 1.08] with observable output motion", "passed": passed, "detail": detail}]