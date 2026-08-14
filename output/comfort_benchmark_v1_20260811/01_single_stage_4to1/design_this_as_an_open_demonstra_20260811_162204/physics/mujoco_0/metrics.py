def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_shaft_hinge")
    out = joints.get("output_shaft_hinge")
    name = "output_shaft_4_to_1_reduction"
    if inp is None or out is None:
        return [{"name": name, "value": None, "expected": -0.25, "passed": False, "detail": "Missing input_shaft_hinge or output_shaft_hinge trajectory."}]
    n = min(len(inp), len(out))
    if n < 3:
        return [{"name": name, "value": None, "expected": -0.25, "passed": False, "detail": "Insufficient shaft trajectory samples."}]
    start = max(0, n // 5)
    def angular_travel(values):
        total = 0.0
        previous = float(values[start])
        for value in values[start + 1:n]:
            current = float(value)
            delta = current - previous
            while delta > 3.141592653589793:
                delta -= 6.283185307179586
            while delta < -3.141592653589793:
                delta += 6.283185307179586
            total += delta
            previous = current
        return total
    input_travel = angular_travel(inp)
    output_travel = angular_travel(out)
    if abs(input_travel) < 2.0:
        return [{"name": name, "value": None, "expected": -0.25, "passed": False, "detail": "The hand-crank input did not complete enough travel to test transmission."}]
    ratio = output_travel / input_travel
    passed = abs(ratio + 0.25) <= 0.0125
    return [{"name": name, "value": ratio, "expected": -0.25, "passed": passed, "detail": "Signed output-shaft travel divided by hand-crank input travel; -0.25 means opposite rotation at an exact 4:1 reduction within 5% simulation tolerance."}]