def check(traj, result):
    name = "vertical_guided_output"
    joints = traj.get("joints", {})
    q = joints.get(name)
    t = traj.get("t", [])
    if q is None or len(q) < 4 or len(q) != len(t):
        return [{"name": "polished_rod_reciprocation", "value": 0.0, "expected": "vertical output moves and reverses direction", "passed": False, "detail": "Missing or incomplete vertical_guided_output trajectory."}]
    vals = [float(x) for x in q]
    span = max(vals) - min(vals)
    eps = max(1e-6, span * 0.01)
    signs = []
    for a, b in zip(vals, vals[1:]):
        d = b - a
        if d > eps:
            signs.append(1)
        elif d < -eps:
            signs.append(-1)
    reversed_direction = 1 in signs and -1 in signs
    passed = span > 1e-4 and reversed_direction
    return [{"name": "polished_rod_reciprocation", "value": span, "expected": "travel span > 0.0001 in the joint's declared unit and motion in both directions", "passed": passed, "detail": "Measured only at the delivered output vertical_guided_output; direction reversal proves reciprocation."}]