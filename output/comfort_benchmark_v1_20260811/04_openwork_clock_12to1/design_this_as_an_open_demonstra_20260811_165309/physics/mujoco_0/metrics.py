def check(traj, result):
    joints = traj.get("joints", {})
    t = traj.get("t", [])
    minute = joints.get("minute_spin")
    hour = joints.get("hour_spin")
    expected = "minute_hand/hour_hand angular-speed ratio = 12:1 within 3%, with both hands moving independently"
    if minute is None or hour is None:
        return [{"name": "hand_angular_speed_ratio", "value": None, "expected": expected, "passed": False, "detail": "Missing minute_spin or hour_spin trajectory."}]
    n = min(len(t), len(minute), len(hour))
    if n < 5:
        return [{"name": "hand_angular_speed_ratio", "value": None, "expected": expected, "passed": False, "detail": "Insufficient trajectory samples."}]
    start = max(0, int(0.4 * n))
    tt = t[start:n]
    mm = minute[start:n]
    hh = hour[start:n]
    if len(tt) < 3:
        return [{"name": "hand_angular_speed_ratio", "value": None, "expected": expected, "passed": False, "detail": "Insufficient post-settle samples."}]
    mt = sum(tt) / len(tt)
    den = sum((x - mt) * (x - mt) for x in tt)
    if den <= 0.0:
        return [{"name": "hand_angular_speed_ratio", "value": None, "expected": expected, "passed": False, "detail": "Trajectory has no measurable time span."}]
    mm_mean = sum(mm) / len(mm)
    hh_mean = sum(hh) / len(hh)
    minute_speed = sum((tt[i] - mt) * (mm[i] - mm_mean) for i in range(len(tt))) / den
    hour_speed = sum((tt[i] - mt) * (hh[i] - hh_mean) for i in range(len(tt))) / den
    if abs(minute_speed) < 0.2 or abs(hour_speed) < 0.01:
        return [{"name": "hand_angular_speed_ratio", "value": None, "expected": expected, "passed": False, "detail": "The driven minute hand or final hour hand did not sustain observable motion."}]
    ratio = abs(minute_speed / hour_speed)
    coupled = abs(abs(minute_speed / hour_speed) - 1.0) < 0.02
    passed = (not coupled) and abs(ratio - 12.0) <= 0.36
    detail = "Measured from the carriers of minute_hand and hour_hand after settling."
    if coupled:
        detail = "The two hand carriers rotate at effectively 1:1, indicating an unintended rigid coupling or welded coaxial fit."
    elif not passed:
        detail = "Motion reached hour_spin, but the measured hand speed ratio is not 12:1 within simulation tolerance."
    return [{"name": "hand_angular_speed_ratio", "value": ratio, "expected": expected, "passed": passed, "detail": detail}]