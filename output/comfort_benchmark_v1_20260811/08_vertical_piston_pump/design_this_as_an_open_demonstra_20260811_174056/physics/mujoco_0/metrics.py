def check(traj, result):
    joints = traj.get("joints", {})
    bodies = traj.get("bodies", {})
    xs = joints.get("crosshead_vertical_slide")
    piston = bodies.get("pump_piston")
    if not xs or len(xs) < 4:
        return [{"name": "vertical_piston_reciprocation", "value": 0, "expected": "repeated vertical output motion", "passed": False, "detail": "Missing or insufficient crosshead_vertical_slide trajectory."}]
    vals = [float(v) for v in xs]
    span = max(vals) - min(vals)
    eps = max(1e-6, span * 0.01)
    direction = 0
    reversals = 0
    for i in range(1, len(vals)):
        dv = vals[i] - vals[i - 1]
        new_direction = 1 if dv > eps else (-1 if dv < -eps else 0)
        if new_direction and direction and new_direction != direction:
            reversals += 1
        if new_direction:
            direction = new_direction
    vertical_ok = True
    lateral = 0.0
    zspan = 0.0
    if piston and len(piston) >= 2:
        x0, y0 = float(piston[0][0]), float(piston[0][1])
        lateral = max((((float(p[0]) - x0) ** 2 + (float(p[1]) - y0) ** 2) ** 0.5) for p in piston)
        zs = [float(p[2]) for p in piston]
        zspan = max(zs) - min(zs)
        vertical_ok = zspan > 0.5 and lateral <= max(0.5, 0.05 * zspan)
    passed = span > 1e-4 and reversals >= 2 and vertical_ok
    detail = "crosshead span=%.6g m, direction reversals=%d, piston vertical span=%.3f mm, maximum lateral drift=%.3f mm" % (span, reversals, zspan, lateral)
    return [{"name": "vertical_piston_reciprocation", "value": span, "expected": "crosshead/pump piston has nonzero stroke, at least two reversals, and predominantly vertical motion", "passed": passed, "detail": detail}]