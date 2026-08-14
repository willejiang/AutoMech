### Detailed strict criterion scores

Cells show awarded points. Maxima are `E=10`; Assembly `model/roles/I-O=5/7/3`; Geometry `mesh/conflicts=5/10`; Physics `shape/alignment/finite=5/5/10`; Functional `input/propagation/output/invariants=5/10/15/10`. `G` means the criterion had raw diagnostic evidence but received zero points because a strict prerequisite gate was closed.

#### claude-code

Strict aggregate: **310/1000; 0/10 PASS**.

| # | Task | E | A M/R/I | G M/C | P S/A/F | F I/P/O/V | F total | Total | Strict | Blocker |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 01_single_stage_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Five non-exempt conflicts and most links are not connected to the base. |
| 2 | 02_two_stage_9to1 | 10 | 5/7/3 | 5/10 | G/G/G | G/G/G/G | 0/40 | 40 | FAIL | Moving collisions disabled, fragmented gears, and eight detached links. |
| 3 | 03_idler_reverser_1to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Moving collisions disabled and three fragmented gear compounds. |
| 4 | 04_openwork_clock_12to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Two undeclared material clashes and all moving collision disabled. |
| 5 | 05_three_planet_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Six conflicts and unverified ring-tooth realization. |
| 6 | 06_four_planet_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Seventeen geometry/unavailable failures and invalid transmission lowering. |
| 7 | 07_horizontal_slider_crank | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Three conflicts and five detached physical links. |
| 8 | 08_vertical_piston_pump | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Two conflicts, invalid closure semantics, and fourteen detached links. |
| 9 | 09_open_pumpjack | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Two conflicts and ten detached physical links. |
| 10 | 10_wind_rotor_pump | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | No authored transmission, inactive closure, stationary replay output, and nine detached links. |

#### codex

Strict aggregate: **390/1000; 1/10 PASS**.

| # | Task | E | A M/R/I | G M/C | P S/A/F | F I/P/O/V | F total | Total | Strict | Blocker |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 01_single_stage_4to1 | 10 | 5/7/3 | 5/10 | 5/5/10 | 5/10/15/10 | 40/40 | 100 | PASS | Source, geometry, one input actuator, ratio binding, collision proxies, and replay pass. |
| 2 | 02_two_stage_9to1 | 10 | 5/7/3 | 5/10 | G/G/G | G/G/G/G | 0/40 | 40 | FAIL | Incomplete collision coverage and geometrically detached declared assembly. |
| 3 | 03_idler_reverser_1to1 | 10 | 5/7/3 | 5/10 | G/G/G | G/G/G/G | 0/40 | 40 | FAIL | Incomplete gear/crank collision coverage and detached physical islands. |
| 4 | 04_openwork_clock_12to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Invalid bearing solids and incomplete realization/collision semantics. |
| 5 | 05_three_planet_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Non-exempt conflict, detached structure, and unverified integrated ring teeth. |
| 6 | 06_four_planet_4to1 | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Non-exempt conflict, detached structure, and unverified integrated ring teeth. |
| 7 | 07_horizontal_slider_crank | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Collision conflicts and a persistent geometrically detached link. |
| 8 | 08_vertical_piston_pump | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Seven non-exempt conflicts and eight detached physical links. |
| 9 | 09_open_pumpjack | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Eleven non-exempt conflicts and eleven detached physical links. |
| 10 | 10_wind_rotor_pump | 10 | 5/7/3 | 5/0 | G/G/G | G/G/G/G | 0/40 | 30 | FAIL | Eight conflicts, incomplete closure/transmission semantics, and eleven detached links. |
