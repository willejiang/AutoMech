def check(traj, result):
    name = "crosshead_vertical_slide"
    joints = traj.get("joints", {})
    if name not in joints:
        return [{"name": "piston_reciprocation", "value": None, "expected": "guided piston output repeatedly reverses direction", "passed": False, "detail": "Missing trajectory key crosshead_vertical_slide"}]
    raw = joints.get(name, [])
    q = [float(x) for x in raw if x is not None]
    if len(q) < 5:
        return [{"name": "piston_reciprocation", "value": 0, "expected": "guided piston output repeatedly reverses direction", "passed": False, "detail": "Insufficient piston trajectory samples"}]
    span = max(q) - min(q)
    stride = max(1, len(q) // 80)
    sampled = q[::stride]
    if sampled[-1] != q[-1]:
        sampled.append(q[-1])
    eps = max(1e-7, span * 0.005)
    signs = []
    for i in range(1, len(sampled)):
        delta = sampled[i] - sampled[i - 1]
        sign = 1 if delta > eps else (-1 if delta < -eps else 0)
        if sign and (not signs or sign != signs[-1]):
            signs.append(sign)
    reversals = max(0, len(signs) - 1)
    passed = span > 1e-4 and 1 in signs and -1 in signs and reversals >= 2
    return [{"name": "piston_reciprocation", "value": {"stroke_span_m": span, "direction_reversals": reversals}, "expected": "more than 0.0001 m output span with at least two clear direction reversals", "passed": passed, "detail": "Measures the user-visible pump piston carrier through crosshead_vertical_slide; rotation alone does not pass."}]