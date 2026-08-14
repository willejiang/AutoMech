def check(traj, result):
    import math
    joints = traj.get('joints', {})
    names = ('minute_rotation', 'hour_rotation')
    missing = [name for name in names if name not in joints or len(joints[name]) < 2]
    if missing:
        return [{'name': 'visible_hand_ratio', 'value': None, 'expected': 'minute/hour angular travel = 12:1', 'passed': False, 'detail': 'Missing or insufficient simulator trajectories: ' + ', '.join(missing)}]

    def travel(values):
        total = 0.0
        prev = float(values[0])
        for raw in values[1:]:
            cur = float(raw)
            delta = cur - prev
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            total += delta
            prev = cur
        return total

    minute = travel(joints['minute_rotation'])
    hour = travel(joints['hour_rotation'])
    if abs(hour) < 0.2:
        return [{'name': 'visible_hand_ratio', 'value': None, 'expected': 'minute/hour angular travel = 12:1', 'passed': False, 'detail': 'The minute member moved %.4f rad, but motion did not reach the visible hour member.' % minute}]
    ratio = abs(minute / hour)
    passed = 10.8 <= ratio <= 13.2
    return [{'name': 'visible_hand_ratio', 'value': ratio, 'expected': '12:1, tolerance 10.8 to 13.2', 'passed': passed, 'detail': 'Measured from simulator trajectories minute_rotation and hour_rotation, with continuous angular increments unwrapped.'}]