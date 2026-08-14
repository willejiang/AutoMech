def check(traj, result):
    joints = traj.get("joints", {})
    t = traj.get("t", [])
    inp = joints.get("crankshaft_revolute")
    out = joints.get("crosshead_vertical_slide")
    if inp is None or out is None:
        missing = []
        if inp is None:
            missing.append("crankshaft_revolute")
        if out is None:
            missing.append("crosshead_vertical_slide")
        return [{"name": "crosshead_reciprocation", "value": 0, "expected": "driven crank produces reversing vertical output motion", "passed": False, "detail": "Missing trajectory key(s): " + ", ".join(missing)}]
    n = min(len(t), len(inp), len(out))
    if n < 3:
        return [{"name": "crosshead_reciprocation", "value": 0, "expected": "driven crank produces reversing vertical output motion", "passed": False, "detail": "Insufficient trajectory samples"}]
    x = [float(v) for v in out[:n]]
    q = [float(v) for v in inp[:n]]
    span = max(x) - min(x)
    input_travel = max(q) - min(q)
    eps = max(1e-7, span * 1e-3)
    signs = []
    for i in range(1, n):
        dx = x[i] - x[i - 1]
        if dx > eps:
            s = 1
        elif dx < -eps:
            s = -1
        else:
            continue
        if not signs or s != signs[-1]:
            signs.append(s)
    reversals = max(0, len(signs) - 1)
    passed = input_travel >= 6.0 and span > 1e-6 and reversals >= 1
    detail = "input travel %.6g rad; output span %.6g m; direction reversals %d" % (input_travel, span, reversals)
    return [{"name": "crosshead_reciprocation", "value": {"output_span_m": span, "reversals": reversals}, "expected": "at least one crank revolution reaches the crosshead and causes nonzero vertical travel with a direction reversal", "passed": passed, "detail": detail}]