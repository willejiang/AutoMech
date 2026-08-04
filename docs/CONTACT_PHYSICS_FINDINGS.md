# Contact-physics limits, measured (2026-07-30)

A sandbox session, run against real STLs from `output/1_12_20260729_144630/iter_4/meshes/`
(a watch movement) with MuJoCo 3.10.0. No pipeline code was changed. The point was to
answer one question honestly: **how much of what `mount=` declares could be handed back to
the physics engine instead?**

Short answer: the *geometric* half can, the *kinematic* half cannot, and the reason is
numerical rather than a modelling shortcut we took.

## 1. SDF collision works, and it fixes the precision problem

MuJoCo's `type="sdf"` accepts our own STL files — no convex decomposition, no plugin
needed for a file mesh. The hand/tube pair (bore 2.15mm on a 2.10mm tube, i.e. a real
0.05mm radial clearance, which must read as NOT touching):

| collision geometry | ncon | reported penetration |
|---|---|---|
| convex decomposition | 4 | **-0.5500 mm** (false contact, 11x the design clearance) |
| SDF | 0 | **0.0000 mm** (correct) |

It was also **17x faster** on the meshing-gear case (2.9 s vs 50.0 s), which was the
opposite of what I expected.

Verification note: `m.geom_type[0]` is the *floor* in these scenes, and `mjGEOM_PLANE == 0`
is indistinguishable from "unset". Reading index 0 made SDF look like a silent downgrade
and led me to the wrong conclusion twice. Name the geoms and read them by name —
`mjGEOM_MESH == 7`, `mjGEOM_SDF == 8`.

This single change would retire the tolerances that exist only to absorb mesh error:
`_FIT_GAP_MM = 0.3` (support_test) and `_PRESS_FIT_CLEARANCE_M = 0.10` (mjcf_builder).
The real transition measured on a 1.200mm bore is between **0.001 and 0.005 mm** of
clearance — two orders of magnitude tighter than that threshold.

> **Resolved for `_PRESS_FIT_CLEARANCE_M` (2026-08-04).** The constant is deleted, and
> not by retuning it: the classification is now the SIGN of the interference
> (`shaft_r - bore > 0` = pressed), which is what `_is_press_fit_overlap` already used.
> Any absolute millimetre threshold is scale-dependent and therefore wrong for *some*
> machine — 0.10mm is a tight fit on a 20mm gearbox shaft and wider than the entire fit
> on a 1mm watch arbor. It cost a whole 7-iteration watch run: `hour_pipe` cleared its
> arbor by 0.050mm (a correct running fit), was classified pressed, and got welded 1:1 to
> the minute shaft while its own gear mesh demanded 0.122 — two equality constraints in
> contradiction, so the solver split the difference and returned 1.90:1 where the design
> said 12:1. The gears were correct in every iteration; the agent was told the ratio was
> wrong and kept re-cutting teeth that were already right. `_FIT_SIGN_EPS_M` is the
> float32 noise floor that keeps a nominal `bore == shaft` on the pressed side.

## 2. Interference fits blow the contact force up to ~1e15 N

Driving a shaft inside a bore and probing the contact forces:

| shaft r | clearance | mu | ratio | mean normal force |
|---|---|---|---|---|
| 1.352 | 2 um | 0.0 | -4.80 | 9.0e-01 N |
| 1.352 | 2 um | 1.0 | +1.0001 | 1.09e+14 N |
| 1.360 | 10 um | 0.0 | +1.0027 | 4.71e+14 N |
| 1.400 | 50 um | 1.0 | +1.7244 | 2.73e+15 N |

1e15 N is roughly a hundred million times the weight of the Earth, so every non-zero ratio
in these runs is an artifact of that blow-up welding the bodies together, not transmission.

**The cause is penetration depth, not friction.** My first reading of the table above was
that "introducing friction takes the force from 0.9 N to 1e14 N" — that was wrong. Those
two rows are different sampling accidents (the mu=0 run happened to have ncon=0 at every
sample point, so its mean is over an empty set). Sweeping mu on *identical* geometry:

| shaft r | interference | mu | mean Fn | relative |
|---|---|---|---|---|
| 1.210 | 0.010 | 0.0 | 5.93e+14 | 1.00 |
| 1.210 | 0.010 | 3.0 | 1.03e+15 | 1.74 |
| 1.230 | 0.030 | 0.0 | 2.63e+15 | 1.00 |
| 1.230 | 0.030 | 3.0 | 2.13e+15 | 0.81 |

Normal force varies by less than 2x across a 6x change in mu — i.e. it is essentially
independent of it, exactly as physics requires. At mu=0, a 0.010mm interference already
produces 5.93e14 N. The contact spring (`solref="0.002 1"`) is simply far too stiff for
micron-scale overlap; friction merely multiplies an already-pathological normal force.

This matters because it points somewhere different: the lever to try is contact
*compliance*, not the friction model.

## 2b. The 1:1 lock is numerical adhesion, not static friction

Under light load an interference fit does turn the part 1:1 (ratio 0.97-0.99), which looks
like correct press-fit behaviour. It is not. Three tests distinguish real static friction
from a solver artifact, and it fails all three.

Breakaway sweep — resisting torque on the driven part, ratio ~1.0 = still gripped,
~0 = slipped free:

| interference | mu | 1e-9 | 1e-7 | 1e-5 | 1e-3 | 1e-1 |
|---|---|---|---|---|---|---|
| 0.005 | 0.5 | 0.987 | 0.922 | -3.111 | -408 | -285442 |
| 0.005 | 3.0 | 0.966 | 0.904 | -3.158 | -399 | -270964 |
| 0.010 | 1.0 | 0.941 | 1.075 | -2.488 | -355 | -145316 |
| 0.030 | 3.0 | 4.991 | -18.844 | -15.006 | -320 | -113065 |

1. **Holding torque does not scale with mu.** 6x the friction coefficient leaves the grip
   unchanged or slightly weaker. Real static friction holds at mu*Fn*r.
2. **It does not scale with interference either.** 0.005 -> 0.030 mm changes nothing.
3. **Breaking it is not slipping.** A real fit that lets go leaves the driven part behind
   (ratio -> 0). Here it is flung backwards to -285442, i.e. energy is being created.

Worth recording because the light-load column on its own reads as a correct press fit. It
is the parameter dependence, not the nominal value, that tells you whether a number is
physics or coincidence.

## 2c. Root cause: too few contact points, and a contact spring far too stiff

Resolving each contact into a torque about the axis — `tau = (r x F)_z`, where a purely
radial force contributes nothing because `r` and `F` are parallel — shows what is actually
wrong. On a 0.010mm interference fit while spinning:

    contact 0: r=1.208mm  F_radial=+1.069e+15 N  F_tangential=+3.331e+04 N  tau=+40.23 N.m

Two things are absurd. The radial force is 3.2e10 times the tangential one, so Coulomb's
bound (`Ft <= mu*Fn`) is satisfied at 0.000000003% — the friction cone is never engaged;
the solver is fully occupied with the pathological normal force. And 40 N.m acting on a
gear with inertia ~1e-11 kg.m^2 gives 4e12 rad/s^2, which reaches 4e8 rad/s within one
timestep. That is the origin of the +524 / -3.3 / +42.2 ratios: not friction transmission,
but one lopsided contact flinging the part away.

**Where the 1e15 N comes from — the contact spring, NOT the interference depth.**
Static probe, no motion:

| interference | ncon | measured penetration | Fn |
|---|---|---|---|
| 0.0005 mm | 1 | -0.0009 mm | 2.335e+14 |
| 0.0010 mm | 2 | -0.0008 mm | 2.098e+14 |
| 0.0100 mm | 3 | -0.0062 mm | 1.537e+15 |
| 0.0300 mm | 6 | -0.0167 mm | 4.174e+15 |

Half a micron of interference already produces 2.3e14 N. Force is linear in penetration —
this is an enormous spring constant, not deep pressing. `solref="0.002 1"` asks the solver
to erase any overlap within 2 ms, which at micron scale is brutally stiff.

**Where the bad torque comes from — the collision detector returns a few lopsided points.**
At 0.010mm interference the three contacts sit at -132 deg, -82 deg, +149 deg. A real
cylindrical fit is a line contact all the way round; GJK/MPR (and SDF) return only deepest
local minima. With no points opposite them, the radial forces cannot cancel, so a huge net
force remains and the torque depends violently on where those few points happen to land.

**Softening does not fix the second problem — and what it does depends on the geometry.**
Sweeping solref over 500x on the shaft-in-bore:

| solref | ncon | max Fn | net radial force | contact angles |
|---|---|---|---|---|
| 0.002 1 | 3 | 1.537e+15 | 2.089e+15 | -132, -82, 149 |
| 0.05 1 | 3 | 2.459e+12 | 3.342e+12 | -132, -82, 149 |
| 1.0 1 | 3 | 6.147e+09 | 8.354e+09 | -132, -82, 149 |

Force drops 250000x; `ncon` and the angles are **identical throughout**. Collision detection
(where things overlap) and constraint solving (how hard to push) are separate stages, and
`solref` only touches the second. On a smooth cylinder there are no extra local minima to
find, so the forces get smaller but stay just as unbalanced — which is why softening never
made mu matter here.

Gear teeth behave the opposite way, because the geometry is different: several tooth pairs
already sit near the contact zone, so allowing deeper overlap does recruit more of them.

| solref | ratio | err vs -0.3333 | ncon | mean Fn |
|---|---|---|---|---|
| 0.002 1 | -0.2888 | 13.4% | 5.3 | 2.83e+16 |
| 0.005 1 | -0.2884 | 13.5% | 11.3 | 2.14e+15 |
| 0.01 1 | -0.0680 | 79.6% | 28.0 | 4.23e+14 |
| 0.02 1 | +0.0207 | 106.2% | 32.1 | 1.95e+14 |
| 0.05 1 | +0.0229 | 106.9% | 31.3 | 2.20e+13 |

`ncon` rises 6x — and transmission gets *worse*, all the way to a sign flip (no drive at
all). Gears push through tooth normal force and need the contact to be STIFF; softened,
the teeth sink into each other's spaces and the push leaks away. Under a light output load
no setting is stable (134.5 / -0.017 / 89837 at solref 0.002 / 0.01 / 0.05).

So the default stiff contact is already the best available setting for gears; more contact
points do not buy accuracy.


## 2d. Things that were tried and do NOT help

Recorded so the same ground is not covered twice. All tested on the same shaft-in-bore rig:

- **`impratio`** — the source shows that under an elliptic cone mu does not multiply the
  Jacobian; it enters through regularization as `mu_eff = mu*sqrt(1/impratio)`, and our
  impratio defaults to 1. Raising it to 10/100/1000 across mu = 0.2/1.0/5.0 gave 12 results,
  all negative, with no monotonic trend in either variable.
- **Pyramidal cone** — here mu *does* multiply the Jacobian directly
  (`mju_addScl(jacdifp, jac, jac + k*NV, con->friction[k-1], NV)`), a completely different
  code path. It is worse: mu = 0.2 -> ratio 42.2, mu = 1.0 -> -3.3, mu = 5.0 -> 0.55.
- **Softening `solref`** — see 2c. On a bore it only scales force; on gear teeth it does
  recruit more contacts (5 -> 32) but accuracy collapses from 13% to 106% error.

That two independent friction implementations both produce noise on the same input is the
strongest evidence that the fault is the contact configuration, not the friction model.
MuJoCo's friction is well validated for grasping and locomotion; a ring of nearly-collinear
tangents with micron penetration is simply a near-singular constraint system, and the
`Linesearch objective is not convex` warnings are the solver saying so.

**What does work, in every single configuration tested:** a clearance fit never transmits.
Shaft 1.195 in a 1.200 bore returned exactly 0.000 across both cone types, three mu values
and four impratio values — 24 for 24. Geometry (does it touch) is reliable; mechanics (does
it drive) is not.



## 3. Gear teeth: mechanism correct, accuracy insufficient

Real meshing pair (15T pinion, tip r 4.00mm; 45T wheel, tip r 12.00mm; centre distance
15.00mm, so the teeth genuinely interleave). Ideal ratio -1/3.

| collision | ratio | vs ideal |
|---|---|---|
| convex decomposition | +205.85 | meaningless |
| SDF | -0.2912 | 12.6% low |

Friction is NOT the cause, which was worth establishing because it is the intuitive
explanation and it is wrong:

| mu | ratio | err | Ft/Fn |
|---|---|---|---|
| 0.0 | -0.2912 | 12.6% | 0.0000 |
| 1.0 | -0.2888 | 13.4% | 0.0907 |
| 5.0 | -0.2007 | 39.8% | 2.9187 |

At mu = 0 the gears still transmit, and transmit *best*. That matches real involute gears,
which drive through tooth normal force; friction is a loss. The residual 12.6% is backlash-
like error — it grows with speed (48.8% at 20 rad/s) and any output load diverges the run.

Good enough to *look* right on video; not good enough to judge a 12:1 ratio, which is
exactly the check the loop depends on.

### 3b. `solref` dominates the gear error — and the repo's value is the bad one

Re-measured on a REAL pair from the watch run `1_12_20260729_174747` (minute_pinion tip r
4.154mm, intermediate_wheel tip r 14.637mm, centre distance 18.00mm; ideal ratio -0.2838).
SDF collision, mu = 0, input commanded at 3 rad/s for 1 s. Only `solref` varies:

| solref | what it is | ratio | err |
|---|---|---|---|
| unset / `0.02 1` | MuJoCo default | -0.2420 | **14.7%** |
| `0.01 1` | | -0.0887 | 68.7% |
| **`0.002 1`** | **the repo's `_GEOM_SOLREF`** | -0.0773 | **72.8%** |
| `0.0004 1` | 2x timestep | +0.2610 | 192% (sign flips) |

The STIFFER the contact, the WORSE gears transmit — the opposite of the intuition the
constant was chosen on. `mjcf_builder._GEOM_SOLREF = "0.002 1"` is commented "stiff,
~critically damped contact" and was tuned for convex-decomposition teeth; under SDF it
costs a factor of ~5 in ratio accuracy. Section 3's 12.6% was measured at the DEFAULT
solref, so it is not comparable to anything the pipeline actually runs.

This also explains a wrong turn worth recording: measuring SDF with the repo's own
constants (solref 0.002, mu 1.0) gives ratio -0.0040 (98.6% off) and reads as "SDF cannot
transmit through gear teeth at all" — a conclusion that is false, and one I reached and
had to retract. **Any comparison of SDF vs convex decomposition must state its solref.**

Two corollaries:
- Convex decomposition at mu = 0 is not merely inaccurate but unusable (ratio +5.79 on the
  same pair, wrong sign). Its ~55% at mu = 1 depends on friction dragging the wheel around
  — i.e. the current pipeline transmits partly by friction covering for tooth geometry.
- The residual error is NOT backlash. See 3c.

### 3c. The gear error is a jittering contact set, and no knob fixes it

Same pair, SDF, mu = 0, solref `0.02 1` (the good one). Four independent sweeps:

**Duration** — backlash would lose a fixed angle once, so its share would FALL with time:

| input | ratio | err | angle lost vs ideal |
|---|---|---|---|
| 1.5 rad (0.5 s) | -0.2561 | 9.8% | +0.042 rad |
| 3 rad (1 s) | -0.2420 | 14.7% | +0.125 |
| 6 rad (2 s) | -0.1979 | 30.3% | +0.515 |
| 12 rad (4 s) | -0.1486 | 47.6% | +1.623 |
| 24 rad (8 s) | -0.1427 | 49.7% | +3.386 |

The lost angle grows monotonically, roughly proportional to total rotation. This is
CONTINUOUS SLIP, not a gap traversed once. Section 3's "backlash-like" reading is wrong.

**Timestep** — a genuine discretisation error must converge as dt -> 0. It diverges:

| dt | err | mean penetration |
|---|---|---|
| 4e-4 | 28.4% | -0.083 mm |
| 2e-4 | 30.3% | -0.084 |
| 1e-4 | 87.8% | -0.114 |
| 5e-5 | 99.7% | -0.104 |
| 2e-5 | **152.8%** | -0.083 |

**solimp** — no monotone relationship either (30.3 / 51.7 / 76.4 / 87.5% across default /
very stiff / extreme / soft), and penetration stays pinned at -0.08..-0.11 mm regardless.

**Contact state** during a clean run: 3.3% of steps have NO contact at all (longest
unbroken gap 7.4 ms = 0.022 rad of input); when in contact, 20.7 points on average
(max 40) at a mean penetration of -0.075 mm, worst -0.245 mm.

So: not friction (3), not backlash, not integration accuracy, not constraint stiffness.
The contact SET itself is unstable — SDF re-solves the tooth-flank intersection every step
and the 20-40 points jump in position and normal between adjacent steps, so the solver
faces a different constraint system each time (`Linesearch objective is not convex`
throughout). A smaller timestep samples that jitter more densely and accumulates more
error, which is why the sweep runs the wrong way. Same disease as section 2's cylindrical
fit, different surface.

**Consequence: there is no setting that makes contact-driven gear ratio trustworthy.**
`<equality>` for the ratio is not a stopgap, it is the only correct option available.

### 3d. SDF speed: it depends entirely on contact density

Both "SDF is 17x faster" and "SDF is 16x slower" are true; they are different scenarios.
Measured on the same watch parts, 2500 steps, identical friction:

| scenario | convex decomposition | SDF | |
|---|---|---|---|
| meshing gear pair (dense contact) | 105 geoms, **0.13 s**, ncon 12 | 2 geoms, **2.31 s**, ncon 28 | SDF **18x slower** |
| 4 parts spread apart (sparse contact) | 173 geoms, **25.26 s**, ncon 768 | 4 geoms, **7.64 s**, ncon 143 | SDF **3.3x faster** |
| model load | 0.42 s | 0.41 s | equal |

Per contact point SDF is dearer; but it generates far fewer points and removes the
N-pieces x M-pieces broadphase entirely (that watch decomposes 13 parts into 255 convex
pieces). Dense sustained contact on few parts favours decomposition; many complex parts
that mostly do NOT touch favour SDF.

### 3e. SDF silently DROPS contacts as the assembly grows — and it is not memory

MuJoCo's SDF collision is not a geometric query. From `mjmodel.h`:

```c
// sdf collision settings
int sdf_initpoints;     // number of starting points for gradient descent   (default 40)
int sdf_iterations;     // max number of iterations for gradient descent    (default 10)
```

It is a **gradient descent** that can fail to converge, and when it does the contact is
simply absent — no warning, no error.

Observed on run `1_12_20260803_195154` iter_1. `rear_bridge_post` sits exactly on the
baseplate (gap +0.0000mm, its underside inside the base solid) and is byte-identical to
`front_bridge_post`, which never falls (252 verts, 500 faces, volume 54.943, both convex,
mirrored positions y = -36.00 / +36.00).

| scene | outcome |
|---|---|
| post + base (2 bodies) | holds, -0.025mm |
| + bridge + front post (4 bodies) | holds, -0.018mm |
| whole assembly (17 bodies) | **falls 52.068mm** |
| whole assembly, convex decomposition | holds, -1.96mm |

Stepping through the full scene: the post starts with **40 contacts** on the base — that
is `sdf_initpoints` exactly — holds them for ~2000 steps, then the count drops to zero and
it free-falls. So the trigger is scene scale, not the part.

What it is NOT:

| suspicion | test | result |
|---|---|---|
| arena memory exhausted | `<size memory="2G">` | **identical** (19.815mm fall, ncon 321) |
| `mjMAXCONPAIR = 50` overrun | count contacts | 40 < 50, never reached |
| the part / its position | 6 positions in a synthetic scene | never reproduces |

Raising `sdf_iterations` 10 -> 50 improves it (19.815mm -> 4.370mm of fall) but does not
fix it: still far past the 1.5mm the support test calls a fall, and every step costs more.

**Consequence.** Anything whose verdict is "did these two touch" must not run on SDF at
assembly scale. `build_support_mjcf` therefore pins itself to convex decomposition —
overfilling a bore only makes a fit read as MORE supported, never less, so the artefact is
harmless there. The real MJCF keeps SDF, where bore clearance decides whether torque
crosses and the precision is the whole point.

The cost of not knowing this was four wasted iterations: the post was reported unsupported
each time, and the agent, told to "move it down until it touches", buried it 0.1mm, then
4mm (through a 4mm-thick base), then 1mm — while that design's gear train was already
correct at 11.820 against a 12:1 target.

## 4. Engine shopping

The session started from a fair objection — `mount=` is a fiction. A part is told which
other part carries it and which it turns with, and the simulation simply believes it. The
question was how much of that could be deleted and left to physics instead.

**Can be handed to physics now:**

| declaration | replace with | evidence |
|---|---|---|
| "these two touch / clear each other" | SDF collision, measured | 0.05mm clearance reads as ncon=0, exactly; convex hulls reported -0.55mm of false contact |
| "this part rests on that one" | gravity settle test | already in use; normal face contact is stable |
| "this fit is press vs running" | measured clearance | the transition is sharp and real: 0.976 -> 0.000 between 0.001 and 0.005 mm |
| tolerances tuned for mesh error (`_FIT_GAP_MM=0.3`; `_PRESS_FIT_CLEARANCE_M=0.10`, since deleted — see §1) | real fit classes | SDF resolves microns, so a fit can be classified by its sign instead of by "mesh slop" |

**Cannot, and the reason is not laziness:**

| declaration | why it must stay | evidence |
|---|---|---|
| "this gear turns 1:1 with its arbor" | contact friction on a cylindrical fit is a near-singular constraint system | 1e15 N normal force, friction cone engaged at 3e-9 of its bound, 40 N.m from a single lopsided contact |
| "these gears mesh at ratio N" | tooth contact is directionally right but 13% off, and diverges under load | mu=0 transmits best (Ft/Fn=0.0000), so it is tooth normal force, not friction; error grows with speed |
| "this shaft sits in two bearings" | not a physics question at all — `mount=` is a tree and cannot express a loop | the trebuchet's right bearing carried nothing; 21 parts in one chain, and MuJoCo skips parent-child collisions, so the counterweight passes through the frame |

Five independent levers were tested against the torque problem — contact stiffness,
`impratio`, friction cone type, SDF vs convex decomposition, and interference depth. None
works. Two of them (elliptic vs pyramidal) are entirely separate code paths in the engine,
and both return noise on the same input, which is what makes this a property of the
configuration rather than of the friction model.

**So the honest split is:**

- `mount=` keeps a *geometric* job (coordinate anchor) that is verifiable.
- The support claim becomes a *hypothesis* that gravity falsifies, not an assertion.
- The kinematic half stays a declaration — but should be *named* as one. URDF calls these
  joints and nobody finds that unreal. `<equality><connect>` / `<weld>` express exactly
  "this shaft rides in this bore", are not restricted to the body tree, and would close the
  loop problem at the same time.

The unreal feeling comes from one field pretending to describe contact physics while
actually declaring kinematics. Splitting it removes the pretence without pretending we can
simulate something we cannot.


Switching engines does not address most of the above. Bullet (Blender) uses maximal
coordinates and would be *worse* on our long joint chains; Blender's soft body is a
mass-spring animation solver, not FEM, so it cannot produce interference-fit clamping
force either. Genuine interference physics needs continuum mechanics (Chrono::FEA, SOFA —
orders of magnitude slower) or an engine with calibrated built-in machine elements
(Algoryx AGX — commercial). Neither is warranted before the mesh-precision and topology
issues above are addressed, since both are ours, not the engine's.
