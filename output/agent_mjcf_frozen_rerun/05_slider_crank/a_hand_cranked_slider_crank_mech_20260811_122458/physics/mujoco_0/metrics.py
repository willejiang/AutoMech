def check(traj, result):
    joints = traj.get("joints", {})
    crank = joints.get("crankshaft_world_hinge")
    slider = joints.get("slider_world_prismatic")
    if crank is None or slider is None:
        missing = []
        if crank is None:
            missing.append("crankshaft_world_hinge")
        if slider is None:
            missing.append("slider_world_prismatic")
        return [{"name": "piston_reciprocation", "value": 0.0, "expected": "nonzero bidirectional slider travel during at least one crank revolution", "passed": False, "detail": "Missing trajectory key(s): " + ", ".join(missing)}]
    n = min(len(crank), len(slider))
    if n < 3:
        return [{"name": "piston_reciprocation", "value": 0.0, "expected": "nonzero bidirectional slider travel during at least one crank revolution", "passed": False, "detail": "Insufficient trajectory samples."}]
    c = [float(v) for v in crank[:n]]
    s = [float(v) for v in slider[:n]]
    crank_travel = max(c) - min(c)
    span = max(s) - min(s)
    eps = max(1e-7, span * 0.01)
    positive = any((s[i] - s[i-1]) > eps for i in range(1, n))
    negative = any((s[i] - s[i-1]) < -eps for i in range(1, n))
    passed = crank_travel >= 6.0 and span > 1e-4 and positive and negative
    detail = "crank travel=%.6g rad, slider span=%.6g m, positive_motion=%s, negative_motion=%s" % (crank_travel, span, positive, negative)
    return [{"name": "piston_reciprocation", "value": span, "expected": "slider_world_prismatic has visible nonzero travel in both directions while crankshaft_world_hinge turns at least one revolution", "passed": passed, "detail": detail}]