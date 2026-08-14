def check(traj, result):
    joints = traj.get("joints", {})
    minute = joints.get("minute_rotation")
    hour = joints.get("hour_rotation")
    if minute is None or hour is None:
        missing = []
        if minute is None:
            missing.append("minute_rotation")
        if hour is None:
            missing.append("hour_rotation")
        return [{"name": "hand_motion_ratio", "value": None, "expected": "minute_rotation/hour_rotation angular travel = 12:1", "passed": False, "detail": "Missing simulator trajectory: " + ", ".join(missing)}]
    if len(minute) < 2 or len(hour) < 2:
        return [{"name": "hand_motion_ratio", "value": None, "expected": "minute_rotation/hour_rotation angular travel = 12:1", "passed": False, "detail": "Insufficient trajectory samples."}]

    def travel(values):
        total = 0.0
        prev = float(values[0])
        for value in values[1:]:
            cur = float(value)
            delta = cur - prev
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            total += delta
            prev = cur
        return total

    dm = travel(minute)
    dh = travel(hour)
    am = abs(dm)
    ah = abs(dh)
    if am < 1.0:
        return [{"name": "hand_motion_ratio", "value": am, "expected": "minute hand travel >= 1 rad", "passed": False, "detail": "The driven minute member did not accumulate enough motion for a meaningful ratio test."}]
    if ah < 0.05:
        return [{"name": "hand_motion_ratio", "value": ah, "expected": "hour hand travel >= 0.05 rad", "passed": False, "detail": "Motion did not reach the hour-hand output."}]
    ratio = am / ah
    passed = 11.4 <= ratio <= 12.6
    return [{"name": "hand_motion_ratio", "value": ratio, "expected": "12.0 within [11.4, 12.6]", "passed": passed, "detail": "Computed from the simulator's visible hand-carrier trajectories minute_rotation and hour_rotation."}]