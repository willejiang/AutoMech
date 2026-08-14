def check(traj, result):
    names = ["sun_input_hinge", "carrier_output_hinge", "planet_1_carrier_hinge", "planet_2_carrier_hinge", "planet_3_carrier_hinge"]
    joints = traj.get("joints", {})
    missing = [n for n in names if n not in joints or len(joints[n]) < 2]
    if missing:
        return [{"name": "planetary_4_to_1_semantics", "value": None, "expected": "sun/carrier = 4:1 with three spinning planets", "passed": False, "detail": "Missing trajectory keys: " + ", ".join(missing)}]
    def travel(name):
        values = joints[name]
        return values[-1] - values[0]
    sun = travel("sun_input_hinge")
    carrier = travel("carrier_output_hinge")
    planets = [travel("planet_1_carrier_hinge"), travel("planet_2_carrier_hinge"), travel("planet_3_carrier_hinge")]
    if abs(sun) < 1.0 or abs(carrier) < 1e-6:
        return [{"name": "planetary_4_to_1_semantics", "value": {"sun_travel": sun, "carrier_travel": carrier}, "expected": "substantial input travel and carrier gain +0.25", "passed": False, "detail": "Insufficient input or output motion."}]
    carrier_gain = carrier / sun
    planet_gains = [p / sun for p in planets]
    ratio_ok = abs(carrier_gain - 0.25) <= 0.0125
    planets_ok = all(abs(g + 0.75) <= 0.075 for g in planet_gains)
    synchronized = max(planet_gains) - min(planet_gains) <= 0.05
    passed = ratio_ok and planets_ok and synchronized
    return [{"name": "planetary_4_to_1_semantics", "value": {"sun_to_carrier_ratio": abs(sun / carrier), "carrier_gain": carrier_gain, "planet_relative_gains": planet_gains}, "expected": {"sun_to_carrier_ratio": 4.0, "carrier_gain": 0.25, "each_planet_relative_gain": -0.75}, "passed": passed, "detail": "Carrier must rotate with the sun at one quarter speed; all three dedicated planet hinges must spin consistently relative to the carrier."}]