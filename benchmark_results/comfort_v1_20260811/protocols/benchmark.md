# PhysCAD Researcher Comfort Benchmark v1

## 1. 目的与适用范围

本 benchmark 用于比较 PhysCAD Researcher 与 Claude Code、Codex 等通用 coding harness 在**当前系统已较成熟的机械能力区间**内的端到端表现。它不是完整机械工程能力宣称，也不覆盖汽车、飞机、流体、空气动力学、柔性体、凸轮、棘轮、离合器或缆索等当前尚未稳定支持的领域。

本套件固定为 10 个原则上可由当前 `KinematicModel → MJCF → MuJoCo evaluator` 表达和验证的任务，重点覆盖：

- 普通齿轮传动树；
- 独立同轴坐标；
- 行星 carrier forest；
- 单闭环曲柄连杆；
- 长运动链与旋转—往复输出；
- 数量与参数变化下的泛化。

本结果必须标注为 **Comfort / In-Distribution Suite**，不得单独用于宣称系统支持任意机械拓扑。

---

## 2. 固定实验协议

### 2.1 预算

每个任务统一使用：

- `max-iters = 3`；
- 相同模型、thinking 配置和工具权限；
- 相同 wall-clock timeout；
- 启用 MuJoCo physics；
- 启用 MJCF agent compiler；
- 启用 benchmark telemetry；
- 主结果使用 cold run；
- 每个任务使用全新输出目录；
- 不复用旧 `machine.py`、accepted MJCF compiler、scenario 或任务专属 artifact；
- 不允许人工修改中间产物或只重跑失败阶段后冒充完整 cold run。

推荐命令形式：

```bash
python -m maker2.run "<PROMPT>" --single-agent --physics --engine mujoco --max-iters 3 --benchmark-cold --json --out <FRESH_OUTPUT_DIR>
```

若其他 harness 不能使用相同 CLI，应给予等价的：三次候选/修复上限、相同时间上限、相同 evaluator 输入输出契约和相同文件/终端工具权限。

### 2.2 运行计数

以下均计入端到端成本：

- CAD authoring 与 refinement；
- diagnostician；
- MJCF compiler 的所有 candidate、revision 和 tool rounds；
- scenario/environment designer；
- VLM 视频判定；
- rollback、retry 和 evaluator repair。

必须报告：

- Operational Pass@1；
- Final Success@3；
- wall-clock runtime；
- LLM requests 与 input/output/cache/reasoning tokens；
- tool calls；
- MJCF compiler candidate/submission 数；
- cache hit/miss/rejected-hit；
- fault domain。

历史任务若没有首轮事件，不得从 `iterations == 1` 反推 Pass@1。

---

## 3. 通用成功条件与计分

### 3.1 严格主结果

每个任务只有同时满足以下条件才记为 Final Success：

1. **Execution**：最终机械程序可重新执行，无异常、无超时，并生成非空零件集合；
2. **Assembly**：零件图连通、引用有效、声明关系得到对应类型的机械实现；
3. **Geometry**：实体有效，不存在非几何豁免的刚体穿透；
4. **Physics-Ready**：未应用 support patch 的 accepted MJCF 可加载、初始化并完成固定时长仿真，状态有限且无爆炸；
5. **Functional**：满足本任务规定的输入、传播、输出和机械不变量。

主成功率使用二值判定：

$$
R_{\mathrm{success@3}}=\frac{1}{10}\sum_{n=1}^{10}\mathbb I[e_n a_n g_n p_n f_n=1].
$$

### 3.2 100 分诊断分

百分制只用于解释“离成功多远”，不能替代严格成功率：

| 层级 | 分值 | 判定 |
|---|---:|---|
| Execution | 10 | 最终源程序可重执行并产生零件 |
| Assembly | 15 | 连通、引用有效、声明关系按类型兑现 |
| Geometry | 15 | 有效实体且非豁免 conflict count 为 0 |
| Physics-Ready | 20 | accepted 原模型 load/init/finite/stable；support derivative 不计入 |
| Functional | 40 | 按各任务的四项功能判据评分 |

层级分数采用前置依赖：Execution 失败时后续均为 0；Assembly 失败时 Geometry/Physics/Functional 均为 0；Geometry 失败时 Physics/Functional 均为 0；Physics-Ready 失败时 Functional 为 0。

每个任务的 Functional 40 分统一拆分为：

- 输入达到规定有效行程：5 分；
- 运动传播到全部规定下游件：10 分；
- 目标输出达到规定值/范围：15 分；
- 任务专属机械不变量全部成立：10 分。

任何一项只部分满足时可以给诊断部分分，但 `f_n` 仍为 0。

### 3.3 证据优先级

功能判定按以下证据顺序：

1. body/joint trajectory 与约束残差；
2. 接触、有限状态、力/冲量和 stability 数值；
3. 视频关键帧；
4. VLM 或自然语言摘要。

视频遮挡只能记为 visual inconclusive，不能推翻明确的正确轨迹；数值 ratio 正确也不能覆盖齿轮飞离、轴心公转或连杆脱开等轨迹失败。

`direct_qpos` 只能证明 exact kinematic conformance。任务若使用它，必须标记 `idealized`，不得宣称真实齿面接触、有限力矩或负载能力。凡本任务要求 finite-effort servo 的项目，不得用 direct-qpos 代替。

---

## 4. 十项任务、固定 Prompt 与判据

## 4.1 单级 4:1 直齿轮减速器

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame hand-cranked single-stage spur gear reducer with an exact 4:1 reduction. Use one input shaft with a visible hand crank and one parallel output shaft. Support both shafts in clearly visible running bearings, rigidly attach each gear to its own shaft, expose the complete tooth mesh, and include a stable bench-mounted base. The input must be the hand crank only; the output shaft must not be independently driven. Author complete mechanism semantics for the shaft hinges, press fits, running fits, gear mesh, driver, output, and watched links.
```

### 功能判据（40 分）

- 5：输入轴净转角至少 `6 rad`；
- 10：输入 gear、输出 gear、输出轴全部发生有效运动；
- 15：观测 reduction 满足 `3.8 <= |theta_in/theta_out| <= 4.2`，方向相反；
- 10：两根轴的世界轴心保持固定，齿轮留在各自轴上，gear equality/mesh pair 与 4:1 语义一致。

允许 ideal gear equality；必须披露为 idealized。禁止直接驱动输出轴。

---

## 4.2 二级 9:1 直齿轮减速器

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame hand-cranked two-stage spur gear reducer with an exact overall 9:1 reduction, using two 3:1 stages. Use three parallel shafts: an input shaft with one visible hand crank, a compound intermediate shaft carrying both its driven gear and the second-stage pinion, and one output shaft. Support every shaft in visible running bearings, expose both tooth meshes, and mount the machine on a stable base. Only the input crank is driven. Author complete mechanism semantics, including the compound rigid carrying, both transmissions, driver, output, and watched links.
```

### 功能判据

- 5：输入净转角至少 `9 rad`；
- 10：两级所有传动坐标均运动，输出实际 reached；
- 15：`8.5 <= |theta_in/theta_out| <= 9.5`，且两次外啮合后输入输出同向；
- 10：三根轴心固定，compound gear/pinion 相对姿态恒定，两级均无 dead stage。

---

## 4.3 带惰轮的三轴 1:1 倒向齿轮系

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame three-shaft spur gear reversing train with one input gear, one freely rotating idler gear, and one output gear. Use equal tooth counts for the input and output so the magnitude of the overall ratio is exactly 1:1. All three shaft axes must be parallel and fixed in the world, all two gear meshes must be fully visible, and only the input shaft has a hand crank. The idler must have its own independent hinge and must not be welded to either neighboring gear. Include a stable bench-mounted base and complete mechanism semantics for both meshes, all bearings, driver, output, and watched links.
```

### 功能判据

- 5：输入净转角至少 `6 rad`；
- 10：idler 与 output 均有效运动；
- 15：`0.95 <= |theta_out/theta_in| <= 1.05`；经过两个外啮合，输出与输入同向；
- 10：idler 是独立坐标，三根轴心固定，两条 mesh 均兑现且没有把 idler 错焊成 1:1 rigid body。

---

## 4.4 镂空 12:1 指针钟表

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an openwork mechanical clock display with two clearly visible coaxial hands whose angular speed ratio is exactly 12:1. Use a visible spur gear train, independent coaxial shafts or sleeves with running clearances, rigidly attach each hand to its intended shaft, and expose the gears and both hands from the camera side. Mount the frame rigidly on a stable base. The minute-side input is the only driver; the hour hand is the final output. Coaxial members with different speeds must remain independent and must not be welded or forced to 1:1. Author complete mechanism semantics for every transmission, bearing, press fit, driver, output, and watched link.
```

### 功能判据

- 5：minute input 净转角至少 `12 rad`；
- 10：全部声明的中间传动坐标与 hour output 均运动；
- 15：`11.4 <= |theta_minute/theta_hour| <= 12.6`；
- 10：两指针保持同轴但独立，不出现 1:1 锁死、脱轴或指针飞离。

允许 direct-qpos exact fixture，但必须标记 idealized；该结果不宣称真实齿面负载能力。

---

## 4.5 三行星固定齿圈 4:1 减速器

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame hand-driven planetary reducer with a fixed ring gear, a sun gear input, exactly three equally spaced planet gears on a rigid carrier, and the carrier as output. Choose tooth counts that give an exact 4:1 sun-to-carrier reduction with the ring fixed. Every planet gear must be carried around the sun by the carrier while also spinning on its own dedicated carrier pin hinge. Expose the sun, all three planets, the ring, and the carrier; do not hide them behind a full cover. Include a stable base, a visible input crank, and complete planetary-stage semantics, meshes, bearings, driver, output, and watched links.
```

### 功能判据

- 5：sun input 净转角至少 `12 rad`；
- 10：carrier 与全部 3 个 planet gear/pin 组均运动；
- 15：`0.2375 <= |theta_carrier/theta_sun| <= 0.2625`；
- 10：每个 planet center 随 carrier 公转，gear—pin 中心距离恒定，并存在相对 carrier 的局部自转；固定 ring 不转。

仅“planet body moved”不算通过，必须验证 gear 与对应 pin 同步公转且没有 gear 飞离。

---

## 4.6 四行星固定齿圈 4:1 减速器

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame hand-driven planetary reducer with a fixed ring gear, a sun gear input, exactly four equally spaced planet gears on a rigid carrier, and the carrier as output. Choose tooth counts that give an exact 4:1 sun-to-carrier reduction with the ring fixed. Every planet gear must be carried by the carrier and must also spin on its own dedicated carrier pin hinge. Keep all four planets and their pins visibly exposed, use a stable base and visible input crank, and author complete planetary-stage semantics, meshes, bearings, driver, output, and watched links.
```

### 功能判据

- 5：sun input 净转角至少 `12 rad`；
- 10：carrier 与全部 4 个 planet gear/pin 组均运动；
- 15：`0.2375 <= |theta_carrier/theta_sun| <= 0.2625`；
- 10：四组 planet inventory 全部存在、间隔约 `90 deg`，每组 gear—pin 相对关系恒定并随 carrier 公转，ring 固定。

本项专门检查数量泛化；缺任意一个 planet 或只正确处理三个时均失败。

---

## 4.7 卧式手摇曲柄滑块

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame horizontal hand-cranked slider-crank mechanism. Fix the crankshaft axis rigidly to the base, attach one crank disk or crank web and one dedicated eccentric crank pin, connect it through a single rigid connecting rod to a slider constrained to one horizontal linear guide, and expose the complete crank, both rod ends, and slider. Use only one crankshaft input and do not directly actuate the slider. Author explicit revolute, slide, pin, closure, driver, output, and watched-link semantics. Keep rod collisions against the main shaft, web, frame, and guide physically meaningful; do not broadly exclude the whole rod from the crank body.
```

### 功能判据

- 5：crank 净转角至少 `2*pi rad`；
- 10：rod 与 slider 均有效运动；
- 15：slider 完成双向往复，stroke 至少 `20 mm`，方向反转至少 2 次；
- 10：crankshaft 世界轴心漂移不超过机器尺度的 1%，slider 横向偏离导轨轴不超过 stroke 的 2%，两端 closure 保持闭合。

必须使用 finite-effort servo；不得用 direct-qpos 强推穿过碰撞。

---

## 4.8 立式手摇活塞泵运动机构

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame bench-mounted vertical reciprocating piston-pump mechanism driven by a horizontal hand crank. Use one fixed crankshaft, one eccentric crank pin, one connecting rod, one guided vertical crosshead or piston slider, and one visible pump rod/piston moving only vertically inside an open cylinder frame. Keep the mechanism above the ground plane, expose the crank and rod linkage, and use only the crankshaft as input. This is a mechanical motion benchmark; do not claim or simulate fluid pressure. Author complete revolute, slide, closure, rigid-carrying, driver, output, and watched-link semantics.
```

### 功能判据

- 5：crank 净转角至少 `2*pi rad`；
- 10：connecting rod、crosshead、pump rod 与 piston 全部运动；
- 15：最终 piston 仅沿竖直轴往复，span 至少 `15 mm`，方向反转至少 2 次；
- 10：crankshaft 轴心固定，rod—crosshead closure 保持，pump rod/piston 与 crosshead 的声明刚性相对距离恒定，无 body-ground 假碰撞。

只验证机械运动，不验证流量、压力或阀门性能。

---

## 4.9 开放式手摇抽油机

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame hand-cranked pumpjack mechanism on a stable base. Use one fixed horizontal crankshaft with a hand crank, one rotating crank disk and dedicated crank pin, one pitman connecting rod, one pivoted walking beam on a fixed central support, and one vertical polished-rod output guided to reciprocate. Keep the crank, pitman, beam pivot, and output rod fully visible. Only the crankshaft is driven. Author explicit hinge, pin, closure, slide or guided-output, driver, output, and watched-link semantics. This benchmark tests mechanical motion only, not underground fluid extraction.
```

### 功能判据

- 5：input crank 净转角至少 `2*pi rad`；
- 10：pitman、walking beam 与 polished rod 全部发生有效运动；
- 15：polished rod 竖直往复 span 至少 `15 mm`，方向反转至少 2 次；
- 10：crankshaft 与 beam pivot 世界位置固定，beam 绕其 pivot 摆动，所有 pin/closure 保持连接，输出横向漂移不超过 stroke 的 5%。

必须使用 finite-effort servo。

---

## 4.10 风轮驱动往复泵

### Prompt

```text
Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera.

Build an open-frame wind-rotor-driven reciprocating pump on a stable tower or bench frame. Use one fixed horizontal rotor shaft, a clearly visible wind rotor rigidly attached to it, one crank disk with a dedicated eccentric pin, one connecting rod, one guided vertical crosshead, and a visible pump rod/piston output that moves only vertically. Keep the entire rotor-to-crank-to-piston motion path exposed and use the wind rotor shaft as the only input. Author explicit world-frame shaft hinge, crank, pin, closure, slide, rigid-carrying, driver, output, and watched-link semantics. The benchmark tests imposed rotor-driven mechanical transmission, not aerodynamic power generation or fluid pressure.
```

### 功能判据

- 5：rotor input 净转角至少 `2*pi rad`；
- 10：crank、rod、crosshead、pump rod 与 piston 全部运动；
- 15：piston 输出 span 至少 `15 mm`，方向反转至少 2 次；
- 10：rotor shaft/crank disk 世界轴心固定，crank pin 形成近似圆轨迹，输出只沿竖直轴运动，rod—slider—piston 的 carrying/closure 关系恒定。

不验证真实风力或泵送流体，只验证给定输入下的机械传播。

---

## 5. 自动判定的统一数值规则

### 5.1 旋转量

- 使用连续 unwrap 后的净角位移；
- 不使用逐帧绝对增量之和冒充净转角；
- 传动比优先使用稳定区间回归斜率，endpoint ratio 作为辅助；
- 分母净行程过小时 ratio 判定无效。

### 5.2 往复量

- 输出必须同时满足 span 与方向反转次数；
- 单次掉落、爆炸或接触弹飞造成的大位移不算往复；
- 至少 80% 采样状态必须 finite；正式 Physics-Ready 要求全部关键 `qpos/qvel/qacc/xpos` finite；
- 周期性输出的累计绝对路程不能替代 span 和反转。

### 5.3 固定轴与 carrying

令机器总包围盒对角线为 $D$：

- 声明固定在 base 的轴心世界漂移应满足 `axis_drift <= max(1 mm, 0.01 D)`；
- rigid-carried pair 的相对距离标准差应满足 `sigma_d <= max(0.5 mm, 0.005 D)`；
- planet gear—pin 中心距离的 peak-to-peak 变化应满足 `<= max(0.5 mm, 0.005 D)`；
- closure residual 必须低于对应 pin/port 尺度的 2%。

这些阈值只用于轨迹一致性，不能将真实 CAD 穿透豁免。

### 5.4 Geometry exemption 与 solver exclude

严格沿用 `metrics.md`：

- 只有 authored relation + 几何测量共同证明的 intentional overlap 可进入 `X_geom`；
- solver contact exclude 不自动进入 `X_geom`；
- ideal equality 不自动进入 `X_geom`；
- support patch 不改变正常模型的 geometry/physics/function 成绩。

---

## 6. 汇总报告格式

每个任务至少输出一行：

| 字段 | 说明 |
|---|---|
| Task | 1–10 与短名称 |
| Pass@1 | true / false / unknown |
| Final@3 | PASS / FAIL |
| Score | 0–100 诊断分 |
| Exec/Asm/Geo/Physics/Function | 五层二值结果 |
| Physics mode | finite-effort / exact-kinematic / mixed |
| Idealizations | gear equality、planetary equality 等 |
| Runtime | cold wall-clock 秒数 |
| Iterations | 外循环次数 |
| LLM usage | requests 与 token 分类 |
| Tool/compiler work | tool calls、candidate、submission |
| Fault domain | agent_geometry / agent_ir / builder_compiler / runner_scenario / simulator_numerics / evaluator / none |
| Visual evidence | PASS / FAIL / INCONCLUSIVE |
| Mechanical evidence | 关键 ratio、span、axis drift、closure residual |

核心总表必须报告：

1. Comfort Pass@1；
2. Comfort Final Success@3；
3. 五层成功率；
4. 平均/中位 cold runtime；
5. 平均 iterations；
6. 总/平均 LLM tokens 与 requests；
7. 总 tool calls 与 MJCF compiler candidates；
8. fault-domain 分布；
9. 10 项平均诊断分及每项分数。

若 evaluator bug 导致 runner FAIL、但独立机械证据 PASS，应同时记录：

- `Raw Harness Result = FAIL`；
- `Adjudicated Mechanical Result = PASS`；
- `Fault Domain = evaluator/runner_scenario`。

Raw 结果不得静默改成 PASS，机械能力也不得因此错误记为机构失败。

---

## 7. 不纳入本 Comfort Suite 的任务

完整汽车、完整飞机、真实发动机燃烧、空气动力飞行、液压/流体泵送、cam/Geneva、ratchet/escapement、clutch/brake/backlash、belt/cable/tendon、Stewart/Delta 和柔性减速器不计入本套件成功率。

它们应进入独立的 **Frontier / Capability-Boundary Suite**，评价：

- 是否正确识别当前 ABI 或物理域不支持；
- 是否拒绝伪造不正确的 equality；
- 是否能完成可验证子机构；
- 是否诚实返回 `unsupported_physics`；
- 是否发生数值假 PASS。

正确识别“不支持”在 frontier suite 中可以算 boundary-classification 成功，但不能算本 Comfort Suite 的 Functional PASS。
