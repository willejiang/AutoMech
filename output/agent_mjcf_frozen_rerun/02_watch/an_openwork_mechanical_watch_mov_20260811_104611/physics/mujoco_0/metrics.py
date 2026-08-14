def check(traj, result):
    joints = traj.get("joints", {})
    minute = joints.get("minute_shaft_spin")
    hour = joints.get("hour_pipe_spin")
    if minute is None or hour is None:
        return [{"name": "hand_travel_ratio", "value": None, "expected": "minute hand and hour hand trajectories present; same direction at 12:1", "passed": False, "detail": "Missing minute_shaft_spin or hour_pipe_spin trajectory."}]
    n = min(len(minute), len(hour))
    if n < 2:
        return [{"name": "hand_travel_ratio", "value": None, "expected": "same direction at 12:1", "passed": False, "detail": "Insufficient hand trajectory samples."}]
    dm = float(minute[n - 1]) - float(minute[0])
    dh = float(hour[n - 1]) - float(hour[0])
    if abs(dh) < 1e-6:
        return [{"name": "hand_travel_ratio", "value": None, "expected": "same direction at 12:1", "passed": False, "detail": "The hour hand did not receive observable motion."}]
    ratio = abs(dm / dh)
    same_direction = dm * dh > 0.0
    moved = abs(dm) >= 2.0 and abs(dh) >= 0.15
    passed = moved and same_direction and 11.4 <= ratio <= 12.6
    detail = "minute travel=%.4f rad, hour travel=%.4f rad, ratio=%.3f, same_direction=%s" % (dm, dh, ratio, same_direction)
    return [{"name": "hand_travel_ratio", "value": ratio, "expected": "12.0 +/- 5%, with both hands moving in the same direction", "passed": passed, "detail": detail}]