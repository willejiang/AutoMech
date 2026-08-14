def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_crankshaft_hinge")
    out = joints.get("horizontal_slider_joint")
    if inp is None or out is None:
        missing = []
        if inp is None:
            missing.append("input_crankshaft_hinge")
        if out is None:
            missing.append("horizontal_slider_joint")
        return [{"name": "slider_crank_reciprocation", "value": None, "expected": "wrist_pin/slider reciprocates after at least one crank revolution", "passed": False, "detail": "Missing trajectory key(s): " + ", ".join(missing)}]
    n = min(len(inp), len(out))
    if n < 3:
        return [{"name": "slider_crank_reciprocation", "value": None, "expected": "wrist_pin/slider reciprocates after at least one crank revolution", "passed": False, "detail": "Insufficient trajectory samples"}]
    crank_travel = abs(float(inp[n - 1]) - float(inp[0]))
    vals = [float(v) for v in out[:n]]
    stroke = max(vals) - min(vals)
    eps = max(1e-7, stroke * 0.02)
    deltas = [vals[i + 1] - vals[i] for i in range(n - 1)]
    forward = any(d > eps for d in deltas)
    backward = any(d < -eps for d in deltas)
    passed = crank_travel >= 6.0 and stroke > 1e-5 and forward and backward
    detail = "crank_travel_rad=%.6g, slider_stroke_m=%.6g, forward=%s, backward=%s" % (crank_travel, stroke, forward, backward)
    return [{"name": "slider_crank_reciprocation", "value": stroke, "expected": "nonzero cyclic slider stroke with both travel directions during at least one complete input revolution", "passed": passed, "detail": detail}]