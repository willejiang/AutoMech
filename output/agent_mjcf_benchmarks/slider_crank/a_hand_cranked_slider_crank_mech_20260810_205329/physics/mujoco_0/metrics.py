def check(traj, result):
    joints = traj.get("joints", {})
    crank = joints.get("crankshaft_rotation")
    slider = joints.get("slider_translation")
    if crank is None or slider is None:
        return [{"name": "visible_slider_reciprocation", "value": None, "expected": "slider_translation reverses direction during at least one full crank revolution", "passed": False, "detail": "Missing crankshaft_rotation or slider_translation trajectory."}]
    n = min(len(crank), len(slider))
    if n < 4:
        return [{"name": "visible_slider_reciprocation", "value": None, "expected": "slider_translation reverses direction during at least one full crank revolution", "passed": False, "detail": "Trajectory is too short."}]
    c = [float(x) for x in crank[:n]]
    s = [float(x) for x in slider[:n]]
    crank_travel = sum(abs(c[i] - c[i - 1]) for i in range(1, n))
    stroke = max(s) - min(s)
    tol = max(stroke * 0.001, 1e-9)
    ds = [s[i] - s[i - 1] for i in range(1, n)]
    forward = any(x > tol for x in ds)
    backward = any(x < -tol for x in ds)
    passed = crank_travel >= 6.283185307179586 and stroke > 1e-6 and forward and backward
    return [{"name": "visible_slider_reciprocation", "value": {"crank_travel_rad": crank_travel, "slider_stroke_m": stroke, "moved_both_directions": forward and backward}, "expected": "At least one full crank revolution produces nonzero slider travel in both directions", "passed": passed, "detail": "Measures the delivered output_marker/piston_slider motion through slider_translation; the connecting rod is observed but never driven."}]