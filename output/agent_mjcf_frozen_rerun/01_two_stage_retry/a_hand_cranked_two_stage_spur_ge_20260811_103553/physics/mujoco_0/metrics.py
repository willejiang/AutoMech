def check(traj, result):
    joints = traj.get("joints", {})
    if "input_spin" not in joints or "output_spin" not in joints:
        return [{"name": "reduced_output_transmission", "value": None, "expected": "input_spin and output_spin trajectories present", "passed": False, "detail": "Required input or output trajectory is missing."}]
    iv = joints["input_spin"]
    ov = joints["output_spin"]
    if len(iv) < 2 or len(ov) < 2:
        return [{"name": "reduced_output_transmission", "value": None, "expected": "nonzero output rotation slower than input", "passed": False, "detail": "Trajectory is too short."}]
    ni = min(len(iv), len(ov))
    input_travel = 0.0
    output_travel = 0.0
    for k in range(1, ni):
        di = (float(iv[k]) - float(iv[k - 1]) + 3.141592653589793) % 6.283185307179586 - 3.141592653589793
        do = (float(ov[k]) - float(ov[k - 1]) + 3.141592653589793) % 6.283185307179586 - 3.141592653589793
        input_travel += di
        output_travel += do
    ai = abs(input_travel)
    ao = abs(output_travel)
    ratio = ai / ao if ao > 1e-9 else None
    passed = ai > 1.0 and ao > 0.05 and ao < 0.99 * ai
    return [{"name": "reduced_output_transmission", "value": {"input_travel_rad": ai, "output_travel_rad": ao, "observed_reduction": ratio}, "expected": "The output must rotate measurably and through less angular travel than the input; no exact ratio was specified.", "passed": passed, "detail": "Checks motion propagation to the delivered output shaft and qualitative reduction without inventing a numerical gear ratio."}]