def check(traj, result):
    joints = traj.get("joints", {})
    inp = "input_shaft_rotation"
    out = "output_shaft_rotation"
    missing = [name for name in (inp, out) if name not in joints]
    if missing:
        return [{"name": "delivered_reduction", "value": None, "expected": "observable reduced motion at the output shaft", "passed": False, "detail": "Missing trajectory joint(s): " + ", ".join(missing)}]
    qi = joints[inp]
    qo = joints[out]
    if len(qi) < 2 or len(qo) < 2:
        return [{"name": "delivered_reduction", "value": None, "expected": "observable reduced motion at the output shaft", "passed": False, "detail": "Insufficient trajectory samples."}]
    di = float(qi[-1] - qi[0])
    do = float(qo[-1] - qo[0])
    if abs(di) < 0.5 or abs(do) < 0.02:
        return [{"name": "delivered_reduction", "value": abs(do), "expected": "input travel at least 0.5 rad and output travel at least 0.02 rad", "passed": False, "detail": "Motion did not propagate observably: input travel=%.4f rad, output travel=%.4f rad." % (di, do)}]
    ratio = di / do
    passed = ratio > 1.02
    return [{"name": "delivered_reduction", "value": ratio, "expected": "signed input/output travel ratio greater than 1.02; two external spur meshes produce the same input/output direction", "passed": passed, "detail": "Input travel=%.4f rad, delivered output travel=%.4f rad, signed reduction ratio=%.4f." % (di, do, ratio)}]