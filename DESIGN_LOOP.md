# PhysCAD — CAD + Isaac Sim Design Loop

Task-oriented CAD: instead of generating geometry that *looks* plausible, every
design is **simulated in NVIDIA Isaac Sim under the user's actual task** and judged
by a vision model. Failures come back as frames + concrete fix hints, and the
design (or its test scenario) is revised until it physically works.

> **Thesis (shown empirically):** numeric pose metrics gave a *false PASS* on the
> ANYmal stand-still run (tilt read 2.1°), while the VLM watching the frames
> correctly said *FAIL — "tips onto its side by frame 3, ends overturned."*
> The camera/VLM judge is what makes the evaluation trustworthy.

---

## 1. High-level: two nested loops

The system is **two feedback loops in series**, connected by a file contract:

- **Maker subloop (inner, fast, cheap):** a *worker* generates CAD geometry and a
  *6-perspective visual judger* (VLM) checks it on looks alone — proportions, parts
  present, matches the prompt, not malformed. Iterates here until the geometry is
  sane. No physics. Once the geometry passes, **a VLM authors the URDF directly** —
  reasoning from the geometry + its renders to place joints, axes, and densities
  (no separate CAD kernel / FreeCAD step). This is a cheap pre-filter that keeps
  junk geometry out of the expensive sim.
- **Evaluator loop (outer, slow, physical):** only a design that *passes the visual
  gate* gets simulated in Isaac Sim and judged under the actual task. Physics
  failures (`fix_hint`s) feed all the way back to the worker.

```mermaid
flowchart LR
    U([User task<br/>e.g. &quot;car suspension that<br/>clears a curb&quot;]) --> MAKER

    subgraph MAKER["MAKER SUBLOOP  (teammates' cadam — inner, no physics)"]
        direction TB
        W[Worker: prompt → geometry<br/>STL / .scad]
        W --> VJ{6-view visual judger<br/>VLM: looks right?<br/>matches prompt?}
        VJ -- "no: render critique" --> W
    end

    VJ -- "yes:<br/>visual GATE passed" --> JOINT[VLM authors URDF:<br/>reasons from geometry + renders →<br/>joints, axes, density, assembly<br/>→ manifest]

    JOINT -- "design dir:<br/>manifest.json + .scad/.stl" --> EVAL

    subgraph EVAL["EVALUATOR LOOP  (this repo — Isaac Sim, physical)"]
        direction TB
        E1[Simulate in Isaac Sim]
        E1 --> E2[VLM pass/fail judge<br/>under the task]
    end

    EVAL -- "result.json: fail →<br/>per-failure frames + fix_hint" --> W
    EVAL -- "PASS → final design + demo video" --> OUT([Deliverable:<br/>recorded experiment MP4])

    classDef maker fill:#1e3a5f,stroke:#4a90d9,color:#fff
    classDef eval fill:#2d4a2d,stroke:#5cb85c,color:#fff
    classDef gate fill:#4a3d2d,stroke:#d9a04a,color:#fff
    class W,JOINT maker
    class E1,E2 eval
    class VJ gate
```

**The two gates are different questions.** The maker's visual judge asks *"does this
geometry look like a correct suspension?"* (cheap, static, 6 renders). The
evaluator's VLM asks *"does it physically survive the curb?"* (expensive, dynamic,
sim frames). Visual-plausible ≠ physically-working — which is the whole reason the
sim stage exists — but the visual gate stops obviously-broken geometry from ever
burning a ~minutes-long GPU sim.

**The contract** (`evaluator/README.md`): the maker hands over a *design dir* =
`manifest.json` + the `.scad`/`.stl` it references. The evaluator hands back
`out/result.json` = `passed`, a `summary`, and a `failures[]` list where each
failure carries the **frame indices that show it** and a **`fix_hint`** for the
generator. Key constraint: a fused mesh can't articulate — for suspension the
maker emits a `.scad` with **one module per part** (named in
`manifest.parts[].render_module`) so the evaluator can synthesize joints.

---

## 2. The maker subloop (worker + 6-perspective visual judger)

> Owned by teammates (`cadam`); shown here because it is the inner loop that feeds
> the evaluator. No physics runs here — this stage is pure geometry + vision.

The maker iterates **before** anything reaches Isaac Sim. A *worker* turns the
prompt into CAD geometry; a *visual judger* renders that geometry from **6
perspectives** and asks a VLM whether it looks right and matches the prompt. Only
when the geometry clears that gate does a VLM author the URDF (joints, axes,
densities) and emit the design dir.

```mermaid
flowchart TD
    P([User prompt<br/>+ any fix_hint from the<br/>evaluator's last sim]) --> WORK

    WORK["WORKER (cadam)<br/>prompt → CAD geometry<br/>STL / .scad (per-part modules)"] --> REN

    REN["RENDER 6 perspectives<br/>front · back · left · right ·<br/>top · isometric 3-quarter"] --> JUDGE

    JUDGE["VISUAL JUDGER (VLM)<br/>multi-image of the 6 views →<br/>geometry sane? all parts present?<br/>proportions &amp; prompt-match?<br/>not malformed / interpenetrating?"]

    JUDGE --> GATE{looks correct?}

    GATE -- "no:<br/>render critique<br/>(what's wrong, which view)" --> WORK
    GATE -- "yes" --> URDF

    URDF["VLM AUTHORS URDF<br/>reason from geometry + the 6 renders →<br/>place joints (prismatic/revolute),<br/>axes, limits, density, assembly<br/>(NO FreeCAD / CAD kernel)"]

    URDF --> DD[/"design dir:<br/>manifest.json + .scad/.stl"/]

    DD --> HANDOFF([→ EVALUATOR LOOP<br/>Isaac Sim, §3])

    classDef worker fill:#1e3a5f,stroke:#4a90d9,color:#fff
    classDef llm fill:#3d2d4a,stroke:#9b59b6,color:#fff
    classDef gate fill:#4a3d2d,stroke:#d9a04a,color:#fff
    class WORK worker
    class JUDGE,URDF llm
    class GATE gate
```

**Why 6 views and not 1.** A single render hides failure modes behind the geometry
— a missing rear control arm or an interpenetrating part can be invisible head-on.
Six orthogonal-ish viewpoints (front/back/left/right/top + a 3-quarter iso) give
the VLM enough coverage to catch malformed or incomplete geometry that a single
projection would miss.

**Why the VLM authors the URDF directly (no FreeCAD).** `cadam` emits geometry
only (`.scad`/`.stl`, no joints). Rather than round-tripping through a CAD kernel,
a VLM reads the geometry *and its renders* and writes the URDF/manifest — deciding
where joints go, their axes and limits, and per-part density. This keeps the whole
maker in the "AI reasons about pictures + geometry" regime and avoids a lossy
kernel hand-off. (The per-part `.scad` modules matter here: a fused mesh can't
articulate, so the worker must keep parts separable for the URDF author to joint
them.)

**Two feedback signals reach the worker, at different cadence:**
- *fast/inner* — the visual judger's render critique (geometry looks wrong), no sim.
- *slow/outer* — the evaluator's `fix_hint` (geometry is physically wrong: bottoms
  out, parts detach), after a full Isaac Sim pass (§3).

---

## 3. The closed iteration loop (proven end-to-end)

This is the heart of the system: `evaluator/loop.py` runs on the host and drives
Isaac Sim until the design passes or hits `--max-iters`. It is **task-oriented**
— an LLM *scenario designer* invents the physics test from the task + the robot's
real joint/link names, and *revises* it from simulation feedback.

```mermaid
flowchart TD
    START([task + URDF + asset-root]) --> RI

    RI["robot_info_from_urdf()<br/>extract real joint &amp; link names"] --> DESIGN

    DESIGN["scenario_designer.design(task, robot)<br/><i>LLM → structured spec₀</i><br/>orientation · spawn height · joint pose ·<br/>friction · control · pass_criteria"]

    DESIGN --> SPEC[/"spec_N.json"/]

    SPEC --> SIM

    subgraph CONTAINER["isaac-sim:6.0.1 container  (GPU, headless)"]
        SIM["run_scenario.py<br/>URDF → USD articulation →<br/>apply spec → PhysX sim loop →<br/>capture frames + measure metrics"]
    end

    SIM --> SR[/"sim_result.json<br/>metrics + frames_dir"/]

    SR --> HASFRAMES{frames<br/>captured?}
    HASFRAMES -- no --> METRICONLY[metrics-only verdict]
    HASFRAMES -- "yes" --> VLM

    VLM["analyze.py (host)<br/>sample ~12 keyframes →<br/>multi-image msg → VLM judge<br/><i>passed + failures + fix hints</i>"]

    VLM --> GATE
    METRICONLY --> GATE

    GATE{"PASS?<br/>sim verdict == PASS<br/><b>AND</b> VLM passed"}

    GATE -- "yes" --> WIN([✔ SUCCESS<br/>write history.json])
    GATE -- "no" --> MAXED{iter ==<br/>max_iters?}
    MAXED -- "yes" --> STOP([✘ stop:<br/>max iters reached])
    MAXED -- "no" --> FB

    FB["feedback_text()<br/>MEASURED metrics + CAMERA(VLM) failures"] --> REVISE

    REVISE["scenario_designer.revise(task, robot, prev_spec, feedback)<br/><i>LLM diagnoses failure → spec₍N₊₁₎</i>"]

    REVISE --> SPEC

    classDef llm fill:#3d2d4a,stroke:#9b59b6,color:#fff
    classDef sim fill:#2d4a2d,stroke:#5cb85c,color:#fff
    classDef decision fill:#4a3d2d,stroke:#d9a04a,color:#fff
    class DESIGN,VLM,REVISE llm
    class SIM sim
    class HASFRAMES,GATE,MAXED decision
```

**Why two judges, ANDed.** The sim verdict is cheap numeric pass/fail
(`min_base_z`, `max_drift`, `survive_s`). The VLM verdict is what the camera
actually saw. A design only passes if **both** agree — because numbers alone
green-lit a robot that had already toppled. *(Note: the numeric metric code has a
known bug — it reads the articulation-root frame, which stays fixed, not the base
body that moves. Until fixed, the VLM verdict is the one to trust.)*

---

## 4. The strategy gate (how to evaluate at all)

Before holding a static pose, the evaluator can decide a task needs a *learned
controller*. `evaluator/strategy_selector.py` is the planning stage — given the
task + actuated-DOF count, an LLM picks one of three evaluation modes and flags
structural blockers (e.g. a robot with zero actuated joints can't be controlled).

```mermaid
flowchart TD
    T([task + URDF]) --> SEL["strategy_selector.decide()<br/>count actuated DOF ·<br/>assess structural feasibility"]

    SEL --> S1{strategy?}

    S1 -- "static_stability<br/><i>'stand', 'is this pose stable'</i>" --> A["loop.py:<br/>spawn pose, hold,<br/>check it doesn't fall"]
    S1 -- "scripted_motion<br/><i>'squat and stand', 'lift a leg'</i>" --> B["keyframe joint trajectory<br/>(no learning)"]
    S1 -- "rl_training<br/><i>'walk', 'run', 'locomote'</i>" --> C

    subgraph RL["Isaac Lab RL path  (isaaclab/*.sh)"]
        direction TB
        C[convert_urdf_to_usd.sh] --> C2[train_locomotion.sh<br/>shipped velocity task on GPU]
        C2 --> C3[play_policy.sh<br/>render checkpoint → MP4]
    end

    A --> JUDGE[VLM judge<br/>analyze.py]
    B --> JUDGE
    C3 --> JUDGE
    JUDGE --> V([pass/fail + feedback])

    classDef llm fill:#3d2d4a,stroke:#9b59b6,color:#fff
    classDef sim fill:#2d4a2d,stroke:#5cb85c,color:#fff
    class SEL,JUDGE llm
    class C,C2,C3,A,B sim
```

Proven: Cassie trains a walking policy on a single A10 (~0.73 s/iter, ~12–18 min);
the VLM then reads the rendered behavior as "progressing, structurally sound" vs
collapse.

---

## 5. Two entry paths in the code

The repo carries two pipelines that share the same VLM judge and contract shape:

```mermaid
flowchart LR
    subgraph P1["Path A — manifest-driven (original contract)"]
        direction TB
        EV[evaluate.sh] --> RE["run_eval.py (container)<br/>manifest.json + .scad/.stl →<br/>per-part STL → synth URDF →<br/>USD → sim → frames+metrics"]
        RE --> AN1["analyze.py (host)<br/>→ result.json"]
    end

    subgraph P2["Path B — URDF + scenario-spec loop (proven)"]
        direction TB
        LP[loop.py] --> RS["run_scenario.py (container)<br/>URDF + spec → USD →<br/>sim → frames+metrics"]
        RS --> AN2["analyze.py (host)<br/>→ vlm.json"]
        AN2 --> RV[scenario_designer.revise → next spec]
        RV --> RS
    end

    classDef a fill:#1e3a5f,stroke:#4a90d9,color:#fff
    classDef b fill:#2d4a2d,stroke:#5cb85c,color:#fff
    class EV,RE,AN1 a
    class LP,RS,AN2,RV b
```

- **Path A** is the maker→evaluator handoff: takes a real CAD `manifest.json`,
  derives the URDF from geometry, runs a single sim + judge. This is what closes
  the loop *back to the maker* (the maker re-generates geometry from `fix_hint`s).
- **Path B** is the self-contained iteration loop proven end-to-end on ANYmal C
  and Cassie. It iterates the **test scenario** (not the geometry) against a fixed
  URDF — the inner loop that validated the whole approach while the maker was paused.

---

## 6. Where it runs

| Stage | Host | Notes |
|-------|------|-------|
| `loop.py`, `scenario_designer.py`, `strategy_selector.py`, `analyze.py` | host venv | Azure key stays out of the container; LLM/VLM calls go to Azure OpenAI |
| `run_scenario.py` / `run_eval.py` | inside `nvcr.io/nvidia/isaac-sim:6.0.1` | **entrypoint overridden** to `python.sh` (image default just launches the WebRTC streamer); run **detached** so an SSH reset can't kill it |
| RL training | Isaac Lab in `isaac-lab:6.0.1` | GPU, headless |

- **Box:** Aliyun A10 (`ecs.gn7i`, 23 GB, RT cores), Ubuntu 26.04, driver 595.71.05.
  All big artifacts under `/data/physcad`; host `/data/physcad` mounts to container `/work`.
- **VLM:** Azure OpenAI — *no native-video model*, so the evaluator samples ~12
  keyframes into a multi-image message. Live deployment is `gpt-5.4`; needs
  `max_completion_tokens` (not `max_tokens`) and rejects custom `temperature`.
- **Deliverable:** recorded experiment MP4s for the demo (the
  before/after loop videos in `physcad_videos/`).
```
