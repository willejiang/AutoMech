def check(traj, result):
    joints = traj.get("joints", {}) if isinstance(traj, dict) else {}
    if "crosshead_slide" not in joints:
        return [{"name": "piston_reciprocation", "value": None, "expected": "nonzero cyclic stroke with at least four direction reversals", "passed": False, "detail": "Missing output trajectory key crosshead_slide."}]
    try:
        values = [float(v) for v in joints["crosshead_slide"]]
    except Exception:
        return [{"name": "piston_reciprocation", "value": None, "expected": "nonzero cyclic stroke with at least four direction reversals", "passed": False, "detail": "Output trajectory is not numeric."}]
    values = [v for v in values if v == v and abs(v) < 1e100]
    if len(values) < 3:
        return [{"name": "piston_reciprocation", "value": None, "expected": "nonzero cyclic stroke with at least four direction reversals", "passed": False, "detail": "Insufficient output samples."}]
    stroke = max(values) - min(values)
    threshold = max(stroke * 0.002, 1e-9)
    last_sign = 0
    reversals = 0
    travel = 0.0
    for i in range(1, len(values)):
        delta = values[i] - values[i - 1]
        travel += abs(delta)
        sign = 1 if delta > threshold else (-1 if delta < -threshold else 0)
        if sign and last_sign and sign != last_sign:
            reversals += 1
        if sign:
            last_sign = sign
    cyclic_travel = travel / stroke if stroke > 0.0 else 0.0
    passed = stroke > 0.0001 and reversals >= 4 and cyclic_travel >= 4.0
    detail = "stroke=%.6g m, direction_reversals=%d, total_travel/stroke=%.3f" % (stroke, reversals, cyclic_travel)
    return [{"name": "piston_reciprocation", "value": {"stroke_m": stroke, "direction_reversals": reversals, "travel_per_stroke": cyclic_travel}, "expected": "stroke > 0.0001 m, at least four direction reversals, and total travel at least four stroke lengths", "passed": passed, "detail": detail}]