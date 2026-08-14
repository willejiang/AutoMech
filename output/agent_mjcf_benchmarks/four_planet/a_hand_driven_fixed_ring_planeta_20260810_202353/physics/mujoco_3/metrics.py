def check(traj, result):
    joints = traj.get("joints", {})
    required = ["input_shaft_hinge", "output_shaft_hinge"]
    missing = [name for name in required if name not in joints]
    if missing:
        return [{"name": "carrier_output_reduction", "value": None, "expected": "input and output trajectories present", "passed": False, "detail": "Missing trajectory keys: " + ", ".join(missing)}]

    def signed_travel(values):
        if not values or len(values) < 2:
            return 0.0
        total = 0.0
        previous = float(values[0])
        for value in values[1:]:
            current = float(value)
            delta = current - previous
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            total += delta
            previous = current
        return total

    input_travel = signed_travel(joints["input_shaft_hinge"])
    output_travel = signed_travel(joints["output_shaft_hinge"])
    if abs(input_travel) < 0.5:
        return [{"name": "carrier_output_reduction", "value": {"input_travel_rad": input_travel, "output_travel_rad": output_travel}, "expected": "the driven input travels enough to test transmission", "passed": False, "detail": "Input motion was too small for a valid reducer test."}]

    propagated = abs(output_travel) >= 0.05
    same_direction = input_travel * output_travel > 0.0
    reduced = abs(output_travel) < 0.95 * abs(input_travel)
    ratio = abs(input_travel / output_travel) if propagated else None
    passed = propagated and same_direction and reduced
    return [{"name": "carrier_output_reduction", "value": {"input_travel_rad": input_travel, "output_travel_rad": output_travel, "reduction_ratio": ratio}, "expected": "carrier output moves in the input direction with input_travel/output_travel > 1; no exact ratio was specified", "passed": passed, "detail": "Checks motion propagation to the delivered output, fixed-ring planetary direction, and reduction without inventing an exact ratio."}]