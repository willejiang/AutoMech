def check(traj, result):
    key = "vertical_crosshead_guide"
    joints = traj.get("joints", {})
    if key not in joints:
        return [{"name": "pump_output_reciprocation", "value": "missing", "expected": "vertical_crosshead_guide trajectory", "passed": False, "detail": "The pump_piston and pump_rod output carrier vertical_crosshead_guide was not recorded."}]
    try:
        q = [float(v) for v in joints[key]]
    except Exception:
        return [{"name": "pump_output_reciprocation", "value": "invalid", "expected": "numeric output trajectory", "passed": False, "detail": "The recorded vertical_crosshead_guide values are not numeric."}]
    if len(q) < 4:
        return [{"name": "pump_output_reciprocation", "value": len(q), "expected": "at least four output samples", "passed": False, "detail": "Too few pump-piston samples were recorded to demonstrate reciprocation."}]
    stroke = max(q) - min(q)
    eps = max(1e-9, stroke * 1e-4)
    signs = []
    rise = 0.0
    fall = 0.0
    for a, b in zip(q, q[1:]):
        d = b - a
        if d > eps:
            rise += d
            signs.append(1)
        elif d < -eps:
            fall += -d
            signs.append(-1)
    reversals = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    passed = stroke > 1e-7 and rise > 0.25 * stroke and fall > 0.25 * stroke and reversals >= 2
    if passed:
        detail = "pump_piston completed repeated guided rise and fall: stroke %.6g m, cumulative rise %.6g m, cumulative fall %.6g m, %d direction reversals." % (stroke, rise, fall, reversals)
    else:
        detail = "Motion did not reach the pump_piston as sustained reciprocation through place_crankshaft -> place_connecting_rod -> place_piston_slider -> place_pump_rod: stroke %.6g m, rise %.6g m, fall %.6g m, %d reversals." % (stroke, rise, fall, reversals)
    return [{"name": "pump_output_reciprocation", "value": {"stroke_m": stroke, "rise_m": rise, "fall_m": fall, "direction_reversals": reversals}, "expected": "nonzero pump-piston stroke with repeated upward and downward travel", "passed": passed, "detail": detail}]