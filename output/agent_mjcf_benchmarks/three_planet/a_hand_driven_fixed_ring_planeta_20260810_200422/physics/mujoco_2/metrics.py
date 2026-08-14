def check(traj, result):
    joints = traj.get("joints", {})
    ikey = "sun_world_hinge"
    okey = "carrier_world_hinge"
    missing = [k for k in (ikey, okey) if k not in joints]
    if missing:
        return [{"name": "fixed_ring_planetary_reduction", "value": None, "expected": "sun input and carrier output trajectories present", "passed": False, "detail": "Missing trajectory keys: " + ", ".join(missing)}]
    qi = joints[ikey]
    qo = joints[okey]
    if len(qi) < 2 or len(qo) < 2:
        return [{"name": "fixed_ring_planetary_reduction", "value": None, "expected": "enough samples to measure input and output travel", "passed": False, "detail": "Insufficient trajectory samples."}]
    din = float(qi[-1]) - float(qi[0])
    dout = float(qo[-1]) - float(qo[0])
    ratio = abs(din / dout) if abs(dout) > 1e-9 else None
    passed = abs(din) >= 1.0 and abs(dout) >= 0.05 and din * dout > 0.0 and abs(dout) < 0.95 * abs(din)
    value = {"input_travel_rad": din, "carrier_travel_rad": dout, "input_to_carrier_ratio": ratio}
    return [{"name": "fixed_ring_planetary_reduction", "value": value, "expected": "nonzero carrier output in the input direction with less angular travel than the input", "passed": passed, "detail": "For a fixed-ring planetary reducer, the carrier must rotate in the sun-input direction at reduced angular travel; no exact tooth-count ratio was specified."}]