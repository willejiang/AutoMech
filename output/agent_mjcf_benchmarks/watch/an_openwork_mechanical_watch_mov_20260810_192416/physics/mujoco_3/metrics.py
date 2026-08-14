def check(traj, result):
    joints = traj.get("joints", {})
    minute = joints.get("minute_rotation")
    hour = joints.get("hour_rotation")
    if minute is None or hour is None or len(minute) < 2 or len(hour) < 2:
        return [{"name": "hand_travel_ratio", "value": None, "expected": "absolute minute/hour travel ratio = 12:1", "passed": False, "detail": "Missing or insufficient minute_rotation or hour_rotation trajectory."}]

    def travel(values):
        total = 0.0
        previous = float(values[0])
        for value in values[1:]:
            current = float(value)
            step = current - previous
            while step > 3.141592653589793:
                step -= 6.283185307179586
            while step < -3.141592653589793:
                step += 6.283185307179586
            total += step
            previous = current
        return total

    dm = travel(minute)
    dh = travel(hour)
    if abs(dm) < 0.5:
        return [{"name": "hand_travel_ratio", "value": None, "expected": "absolute minute/hour travel ratio = 12:1", "passed": False, "detail": "The minute-hand carrier did not complete enough measurable travel."}]
    if abs(dh) < 0.05:
        return [{"name": "hand_travel_ratio", "value": None, "expected": "absolute minute/hour travel ratio = 12:1", "passed": False, "detail": "Motion did not reach the hour-hand carrier."}]
    ratio = abs(dm / dh)
    coupled = abs(abs(dm) - abs(dh)) <= max(0.02, 0.01 * max(abs(dm), abs(dh)))
    passed = (not coupled) and 11.4 <= ratio <= 12.6
    detail = "Measured from the visible hand carriers minute_rotation and hour_rotation."
    if coupled:
        detail = "The hand carriers moved approximately 1:1, indicating unintended rigid coupling."
    return [{"name": "hand_travel_ratio", "value": ratio, "expected": "12:1 within 5%, and not rigidly coupled 1:1", "passed": passed, "detail": detail}]