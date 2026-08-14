def check(traj, result):
    joints = traj.get("joints", {})
    ikey = "input_shaft_spin"
    okey = "output_shaft_spin"
    if ikey not in joints or okey not in joints:
        return [{"name": "fixed_ring_planetary_reduction", "value": None, "expected": "0 < output/input angular travel < 1", "passed": False, "detail": "Missing input_shaft_spin or output_shaft_spin trajectory."}]
    iv = joints.get(ikey, [])
    ov = joints.get(okey, [])
    n = min(len(iv), len(ov))
    if n < 2:
        return [{"name": "fixed_ring_planetary_reduction", "value": None, "expected": "0 < output/input angular travel < 1", "passed": False, "detail": "Insufficient trajectory samples."}]
    try:
        di = float(iv[n - 1]) - float(iv[0])
        do = float(ov[n - 1]) - float(ov[0])
    except Exception:
        return [{"name": "fixed_ring_planetary_reduction", "value": None, "expected": "numeric input and output trajectories", "passed": False, "detail": "Trajectory values were not numeric."}]
    if abs(di) < 0.5:
        return [{"name": "fixed_ring_planetary_reduction", "value": None, "expected": "at least 0.5 rad of input travel", "passed": False, "detail": "The driven input did not travel far enough."}]
    ratio = do / di
    passed = bool(ratio > 0.0 and ratio < 1.0 and abs(do) >= 0.05)
    detail = "Motion reached the visible output in the input direction at a reduced angular rate." if passed else "The visible output must move at least 0.05 rad in the input direction and travel less than the input."
    return [{"name": "fixed_ring_planetary_reduction", "value": ratio, "expected": "0 < output/input angular travel < 1, with at least 0.05 rad output travel", "passed": passed, "detail": detail}]