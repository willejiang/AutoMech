def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("world_rotor_shaft_hinge")
    out = joints.get("world_crosshead_vertical_slide")
    if inp is None or out is None:
        missing = []
        if inp is None:
            missing.append("world_rotor_shaft_hinge")
        if out is None:
            missing.append("world_crosshead_vertical_slide")
        return [{"name": "piston_vertical_reciprocation", "value": 0.0, "expected": "driven rotor produces nonzero piston stroke with a reversal", "passed": False, "detail": "Missing trajectory key(s): " + ", ".join(missing)}]
    n = min(len(inp), len(out))
    if n < 3:
        return [{"name": "piston_vertical_reciprocation", "value": 0.0, "expected": "driven rotor produces nonzero piston stroke with a reversal", "passed": False, "detail": "Insufficient trajectory samples"}]
    x = [float(v) for v in inp[:n]]
    y = [float(v) for v in out[:n]]
    input_travel = sum(abs(x[i] - x[i - 1]) for i in range(1, n))
    stroke = max(y) - min(y)
    eps = max(1e-7, stroke * 0.01)
    rose = any((y[i] - y[i - 1]) > eps for i in range(1, n))
    fell = any((y[i] - y[i - 1]) < -eps for i in range(1, n))
    passed = input_travel >= 6.0 and stroke > 1e-5 and rose and fell
    detail = "input_travel=%.6g rad, piston_stroke=%.6g m, upward=%s, downward=%s" % (input_travel, stroke, rose, fell)
    return [{"name": "piston_vertical_reciprocation", "value": stroke, "expected": "after at least about one rotor revolution, the pump piston has nonzero vertical stroke and reverses direction", "passed": passed, "detail": detail}]