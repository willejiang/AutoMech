def check(traj, result):
    joints = traj.get("joints", {})
    inp = joints.get("input_rotation")
    out = joints.get("carrier_output_rotation")
    if inp is None or out is None or len(inp) < 2 or len(out) < 2:
        return [{"name": "reduced_carrier_transmission", "value": None, "expected": "input and carrier trajectories present", "passed": False, "detail": "Missing input_rotation or carrier_output_rotation trajectory."}]
    di = inp[-1] - inp[0]
    do = out[-1] - out[0]
    if abs(di) < 1e-6:
        return [{"name": "reduced_carrier_transmission", "value": None, "expected": "driven input travel", "passed": False, "detail": "The input shaft did not accumulate measurable angular travel."}]
    ratio = abs(di / do) if abs(do) > 1e-6 else None
    same_direction = di * do > 0.0
    passed = ratio is not None and abs(do) >= 0.05 and same_direction and ratio > 1.05
    detail = "Carrier received motion in the input direction at reduced angular speed." if passed else "Carrier must move measurably in the input direction and travel less than the input; no exact reduction ratio was specified."
    return [{"name": "reduced_carrier_transmission", "value": {"input_travel_rad": di, "carrier_travel_rad": do, "input_per_output": ratio}, "expected": "carrier travel nonzero, same sign as input, and |input travel / carrier travel| > 1.05", "passed": passed, "detail": detail}]