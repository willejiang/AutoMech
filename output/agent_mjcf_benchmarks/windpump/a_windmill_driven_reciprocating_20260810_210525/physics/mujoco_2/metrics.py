def check(traj, result):
    joints = traj.get("joints", {})
    crank = joints.get("crank_rotation")
    output = joints.get("crosshead_slide")
    if crank is None or output is None:
        missing = []
        if crank is None:
            missing.append("crank_rotation")
        if output is None:
            missing.append("crosshead_slide")
        return [{"name": "pump_rod_reciprocation", "value": 0, "expected": "crosshead_slide repeatedly reverses while crank_rotation turns", "passed": False, "detail": "Missing trajectory key(s): " + ", ".join(missing)}]
    n = min(len(crank), len(output))
    if n < 3:
        return [{"name": "pump_rod_reciprocation", "value": 0, "expected": "crosshead_slide repeatedly reverses while crank_rotation turns", "passed": False, "detail": "Too few trajectory samples to observe a crank cycle."}]
    c = [float(v) for v in crank[:n]]
    y = [float(v) for v in output[:n]]
    unwrapped = [c[0]]
    pi = 3.141592653589793
    for i in range(1, n):
        delta = c[i] - c[i - 1]
        while delta > pi:
            delta -= 2.0 * pi
        while delta < -pi:
            delta += 2.0 * pi
        unwrapped.append(unwrapped[-1] + delta)
    input_travel = abs(unwrapped[-1] - unwrapped[0])
    span = max(y) - min(y)
    hysteresis = max(1.0e-7, 0.02 * span)
    direction = 0
    anchor = y[0]
    extreme = y[0]
    reversals = 0
    for value in y[1:]:
        if direction == 0:
            if value > anchor + hysteresis:
                direction = 1
                extreme = value
            elif value < anchor - hysteresis:
                direction = -1
                extreme = value
        elif direction > 0:
            if value > extreme:
                extreme = value
            elif extreme - value > hysteresis:
                reversals += 1
                direction = -1
                extreme = value
        else:
            if value < extreme:
                extreme = value
            elif value - extreme > hysteresis:
                reversals += 1
                direction = 1
                extreme = value
    passed = input_travel >= 2.0 * pi and span > 1.0e-6 and reversals >= 2
    detail = "crank_rotation traveled %.3f rad; pump-piston carrier crosshead_slide spanned %.6f m and reversed direction %d times." % (input_travel, span, reversals)
    if not passed:
        if input_travel < 2.0 * pi:
            detail += " The driven wind rotor did not complete one revolution, so propagation could not be established."
        elif span <= 1.0e-6:
            detail += " Motion dies before the pump_rod/pump_piston output because crosshead_slide remains effectively stationary."
        else:
            detail += " The pump_rod/pump_piston output moves but does not repeatedly reciprocate, indicating failed crank-to-slider closure or guidance."
    return [{"name": "pump_rod_reciprocation", "value": {"input_travel_rad": input_travel, "output_span_m": span, "direction_reversals": reversals}, "expected": "at least one rotor revolution and at least two pump-piston direction reversals with nonzero stroke", "passed": passed, "detail": detail}]