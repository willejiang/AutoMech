from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
EXPOSE = (
    "Design this as an open demonstration mechanism: expose the complete mechanical "
    "structure and motion path, and avoid covers, housings, bridges, or plates that "
    "obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output "
    "from the simulation camera. "
)
TASKS = [
    ("01_single_stage_4to1", EXPOSE + "Build an open-frame hand-cranked single-stage spur gear reducer with an exact 4:1 reduction. Use one input shaft with a visible hand crank and one parallel output shaft. Support both shafts in clearly visible running bearings, rigidly attach each gear to its own shaft, expose the complete tooth mesh, and include a stable bench-mounted base. The input must be the hand crank only; the output shaft must not be independently driven. Author complete mechanism semantics for the shaft hinges, press fits, running fits, gear mesh, driver, output, and watched links."),
    ("02_two_stage_9to1", EXPOSE + "Build an open-frame hand-cranked two-stage spur gear reducer with an exact overall 9:1 reduction, using two 3:1 stages. Use three parallel shafts: an input shaft with one visible hand crank, a compound intermediate shaft carrying both its driven gear and the second-stage pinion, and one output shaft. Support every shaft in visible running bearings, expose both tooth meshes, and mount the machine on a stable base. Only the input crank is driven. Author complete mechanism semantics, including the compound rigid carrying, both transmissions, driver, output, and watched links."),
    ("03_idler_reverser_1to1", EXPOSE + "Build an open-frame three-shaft spur gear reversing train with one input gear, one freely rotating idler gear, and one output gear. Use equal tooth counts for the input and output so the magnitude of the overall ratio is exactly 1:1. All three shaft axes must be parallel and fixed in the world, all two gear meshes must be fully visible, and only the input shaft has a hand crank. The idler must have its own independent hinge and must not be welded to either neighboring gear. Include a stable bench-mounted base and complete mechanism semantics for both meshes, all bearings, driver, output, and watched links."),
    ("04_openwork_clock_12to1", EXPOSE + "Build an openwork mechanical clock display with two clearly visible coaxial hands whose angular speed ratio is exactly 12:1. Use a visible spur gear train, independent coaxial shafts or sleeves with running clearances, rigidly attach each hand to its intended shaft, and expose the gears and both hands from the camera side. Mount the frame rigidly on a stable base. The minute-side input is the only driver; the hour hand is the final output. Coaxial members with different speeds must remain independent and must not be welded or forced to 1:1. Author complete mechanism semantics for every transmission, bearing, press fit, driver, output, and watched link."),
    ("05_three_planet_4to1", EXPOSE + "Build an open-frame hand-driven planetary reducer with a fixed ring gear, a sun gear input, exactly three equally spaced planet gears on a rigid carrier, and the carrier as output. Choose tooth counts that give an exact 4:1 sun-to-carrier reduction with the ring fixed. Every planet gear must be carried around the sun by the carrier while also spinning on its own dedicated carrier pin hinge. Expose the sun, all three planets, the ring, and the carrier; do not hide them behind a full cover. Include a stable base, a visible input crank, and complete planetary-stage semantics, meshes, bearings, driver, output, and watched links."),
    ("06_four_planet_4to1", EXPOSE + "Build an open-frame hand-driven planetary reducer with a fixed ring gear, a sun gear input, exactly four equally spaced planet gears on a rigid carrier, and the carrier as output. Choose tooth counts that give an exact 4:1 sun-to-carrier reduction with the ring fixed. Every planet gear must be carried by the carrier and must also spin on its own dedicated carrier pin hinge. Keep all four planets and their pins visibly exposed, use a stable base and visible input crank, and author complete planetary-stage semantics, meshes, bearings, driver, output, and watched links."),
    ("07_horizontal_slider_crank", EXPOSE + "Build an open-frame horizontal hand-cranked slider-crank mechanism. Fix the crankshaft axis rigidly to the base, attach one crank disk or crank web and one dedicated eccentric crank pin, connect it through a single rigid connecting rod to a slider constrained to one horizontal linear guide, and expose the complete crank, both rod ends, and slider. Use only one crankshaft input and do not directly actuate the slider. Author explicit revolute, slide, pin, closure, driver, output, and watched-link semantics. Keep rod collisions against the main shaft, web, frame, and guide physically meaningful; do not broadly exclude the whole rod from the crank body."),
    ("08_vertical_piston_pump", EXPOSE + "Build an open-frame bench-mounted vertical reciprocating piston-pump mechanism driven by a horizontal hand crank. Use one fixed crankshaft, one eccentric crank pin, one connecting rod, one guided vertical crosshead or piston slider, and one visible pump rod/piston moving only vertically inside an open cylinder frame. Keep the mechanism above the ground plane, expose the crank and rod linkage, and use only the crankshaft as input. This is a mechanical motion benchmark; do not claim or simulate fluid pressure. Author complete revolute, slide, closure, rigid-carrying, driver, output, and watched-link semantics."),
    ("09_open_pumpjack", EXPOSE + "Build an open-frame hand-cranked pumpjack mechanism on a stable base. Use one fixed horizontal crankshaft with a hand crank, one rotating crank disk and dedicated crank pin, one pitman connecting rod, one pivoted walking beam on a fixed central support, and one vertical polished-rod output guided to reciprocate. Keep the crank, pitman, beam pivot, and output rod fully visible. Only the crankshaft is driven. Author explicit hinge, pin, closure, slide or guided-output, driver, output, and watched-link semantics. This benchmark tests mechanical motion only, not underground fluid extraction."),
    ("10_wind_rotor_pump", EXPOSE + "Build an open-frame wind-rotor-driven reciprocating pump on a stable tower or bench frame. Use one fixed horizontal rotor shaft, a clearly visible wind rotor rigidly attached to it, one crank disk with a dedicated eccentric pin, one connecting rod, one guided vertical crosshead, and a visible pump rod/piston output that moves only vertically. Keep the entire rotor-to-crank-to-piston motion path exposed and use the wind rotor shaft as the only input. Author explicit world-frame shaft hinge, crank, pin, closure, slide, rigid-carrying, driver, output, and watched-link semantics. The benchmark tests imposed rotor-driven mechanical transmission, not aerodynamic power generation or fluid pressure."),
]

summary = []
for index, (name, prompt) in enumerate(TASKS, 1):
    task_out = OUT / name
    task_out.mkdir(parents=True, exist_ok=False)
    log_path = OUT / f"{name}.log"
    command = [
        sys.executable, "-m", "maker2.run", prompt,
        "--single-agent", "--physics", "--engine", "mujoco",
        "--max-iters", "3", "--benchmark-cold", "--json",
        "--out", str(task_out),
    ]
    print(f"[{index:02d}/10] START {name}", flush=True)
    started = time.time()
    with log_path.open("w", encoding="utf-8", errors="replace") as log:
        completed = subprocess.run(
            command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
            text=True, check=False,
        )
    duration = time.time() - started
    result_line = ""
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("RESULT_JSON:"):
            result_line = line[len("RESULT_JSON:"):]
    result = None
    if result_line:
        try:
            result = json.loads(result_line)
        except Exception:
            pass
    row = {
        "index": index, "name": name, "returncode": completed.returncode,
        "duration_s": round(duration, 3), "log": str(log_path),
        "result": result,
    }
    summary.append(row)
    (OUT / "suite_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    passed = ((result or {}).get("physics") or {}).get("passed")
    print(f"[{index:02d}/10] END {name} rc={completed.returncode} physics={passed} duration={duration/60:.1f}m", flush=True)
print("SUITE COMPLETE", flush=True)
