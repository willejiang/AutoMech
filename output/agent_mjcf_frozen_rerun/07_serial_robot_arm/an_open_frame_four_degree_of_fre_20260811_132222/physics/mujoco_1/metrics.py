def check(traj, result):
    joints = traj.get("joints", {})
    bodies = traj.get("bodies", {})
    yaw = joints.get("base_yaw_revolute")
    pointer = bodies.get("end_effector_pointer")
    if yaw is None or len(yaw) < 2:
        return [{"name": "yaw_carries_serial_chain", "value": 0.0, "expected": "base_yaw_revolute travel >= 0.3 rad with visible pointer motion", "passed": False, "detail": "Missing or incomplete base_yaw_revolute trajectory."}]
    if pointer is None or len(pointer) < 2:
        return [{"name": "yaw_carries_serial_chain", "value": 0.0, "expected": "end_effector_pointer displacement while base_yaw_revolute moves", "passed": False, "detail": "Missing or incomplete end_effector_pointer body trajectory."}]
    n = min(len(yaw), len(pointer))
    try:
        ys = [float(v) for v in yaw[:n]]
        yaw_travel = max(ys) - min(ys)
        p0 = pointer[0]
        max_disp = 0.0
        for p in pointer[:n]:
            dx = float(p[0]) - float(p0[0])
            dy = float(p[1]) - float(p0[1])
            dz = float(p[2]) - float(p0[2])
            max_disp = max(max_disp, (dx * dx + dy * dy + dz * dz) ** 0.5)
    except (TypeError, ValueError, IndexError):
        return [{"name": "yaw_carries_serial_chain", "value": 0.0, "expected": "valid base-yaw and pointer trajectories", "passed": False, "detail": "Trajectory data are malformed."}]
    passed = yaw_travel >= 0.3 and max_disp >= 10.0
    return [{"name": "yaw_carries_serial_chain", "value": max_disp, "expected": "pointer displacement >= 10 mm while base_yaw_revolute travels >= 0.3 rad", "passed": passed, "detail": "Pointer displacement is %.3f mm and base-yaw travel is %.3f rad. The visual judge must confirm that yaw_carriage, shoulder_link, elbow_link, wrist_link, and end_effector_pointer remain attached and move as one downstream serial chain without gears or transmission coupling." % (max_disp, yaw_travel)}]