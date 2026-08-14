# MJCF Agent 冻结版六项重跑记录

- 代码冻结指纹：`09fc11467689db35c2d8548cd1f6ff9bb766ca4be848d500f050ea9723262fbc`
- MJCF 独立缓存：`output/agent_mjcf_frozen_rerun/cache`
- 规则：不修改 MJCF agent；全新 CAD/IR/MJCF/physics；按顺序运行；数值与视频分别判定。

## 1. 二级齿轮减速器

- Run：`01_two_stage_retry/a_hand_cranked_two_stage_spur_ge_20260811_103553`
- 流程：iteration 0 完成。
- 数值：PASS；input `11.9672 rad`，output `1.3294 rad`，观测减速比 `9.0023:1`，7/7 downstream。
- 视频关键帧：`rgb_0000.png`、`rgb_0020.png`、`rgb_0039.png`。
- 视觉结构：PASS；机架、三根轴和可见齿轮未飞出/脱轴，输入曲柄姿态变化，下层大齿轮留在机架内。
- 视觉啮合：INCONCLUSIVE；上盖板遮挡两级啮合区域，无法从视频直接确认两对齿轮齿面持续啮合。
- 冻结版结论：数值走完全程，但视频证据不足以给出完整机械视觉 PASS。

## 2. 镂空钟表

- Run：`02_watch/an_openwork_mechanical_watch_mov_20260811_104611`
- 流程：iteration 0 完成。
- 数值：PASS；minute `12.0 rad`，hour `1.0 rad`，精确 `12.0:1`，6/6 downstream。
- 视频关键帧：`rgb_0000.png`、`rgb_0020.png`、`rgb_0039.png`；另生成同区域 review crops。
- 视觉结构：PASS；中央指针、齿轮和轴在全过程未飞离或脱轴。
- 视觉输出：PASS；两根指针保持同轴中心且姿态独立变化，无明显 1:1 刚性同步。
- 视觉齿面啮合：PARTIAL；桥板/表盘遮挡部分齿面，能确认齿轮留在各自轴上，但不能从三帧看清全部接触齿面。
- 冻结版结论：数值与可见输出一致，视觉结构通过。

## 3. 三行星轮减速器

- 首次尝试：`03_three_planet/a_hand_driven_fixed_ring_planeta_20260811_110015`。
- 状态：外部中断，不判结果。中断发生在 iteration 2 CAD authoring；iteration 1 尚有 `ring_gear` 对 lower input bearing/lower bridge 的 2 个 interpenetration，未进入 MJCF/physics，不能复用为最终 artifact。
- 处理：保持代码/cache 冻结，从新 retry 目录重新运行完整流程。
- Persistent retry：`03_three_planet_persistent/a_hand_driven_fixed_ring_planeta_20260811_112817` 已完整结束并生成 MJCF、视频和轨迹。
- 数值机械证据：input `20.0 rad`，carrier `5.0 rad`，carrier/input `0.25`（4:1），7/7 downstream，stability PASS。
- 视觉/轨迹：三个 planet gear 与对应 pin 全程 XY 完全同步，gear-pin Z 偏移恒定 `1.5 mm`；无“齿轮飞走、只有轴公转”。例如 planet 1 gear `(21.6,0.005,17.0)→(5.613,-20.858,17.0)`，pin `(21.6,0.005,15.5)→(5.613,-20.858,15.5)`。
- 机械判定：PASS。
- 最终 runner 判定：FAIL（`runner_scenario`）。designer 的 `check()` 正确写出 `ratio >= 0.01 and ratio <= 0.95` 并对 0.25 返回 True；`evaluator/_metrics_runner.py` 二次解析 expected 字符串 `0.01 <= ratio <= 0.95` 时只取首个数字 0.01，并因含 `<=` 误判成单一上限，错误执行 `0.25 <= 0.01`，强制改成 FAIL。该项是 evaluator 区间解析 bug，不是机构失败。

## 4. 四行星轮减速器

- Run：`04_four_planet/a_hand_driven_fixed_ring_planeta_20260811_120530`。
- 流程：iteration 0 完成。
- 数值：PASS；input `12.0 rad`，carrier `3.0 rad`，4.0:1，6/6 downstream，stability PASS。
- 轨迹机械证据：四个 planet gear 到对应 pin 的中心距离全程恒定 `3.0 mm`；每组 gear/pin 的 XY 起终点完全同步公转，无齿轮飞离。
- 示例 planet 1：gear `[30.0,0.006,24.0]→[-29.634,4.673,24.0]`，pin `[30.0,0.006,21.0]→[-29.634,4.673,21.0]`。
- 视频视觉：上盖板严重遮挡行星机构，只能清楚看到输入曲柄姿态变化；视频不足以直接观察四个 planet 的轨道。
- 冻结版结论：机械数值/轨迹 PASS；视频行星可见性 INCONCLUSIVE。

## 5. 曲柄滑块

- Run：`05_slider_crank/a_hand_cranked_slider_crank_mech_20260811_122458`。
- 最终 runner：FAIL；crank 仅 `2.619 rad`，slider span `44.745 mm`，2/2 downstream，未完成一整圈。
- 视觉机械判定：FAIL。首帧曲轴组件在导轨右侧，中帧整体移动到左前方，末帧回到右侧；supported crankshaft 的轴心发生约 `67.64 mm` 世界平移，而非固定轴旋转。slider 往复由错误公转驱动，不可信。
- 根因证据：KinematicModel 将 crankshaft body 放在 `(-35,0,5) mm`，却将 `crankshaft_world_hinge.pos_mm` 写为世界 `(0,0,0)`；emitter 正确转换为 body-local `(35,0,-5) mm`，因此曲轴按 authored IR 围绕世界原点公转。
- Fault domain：`agent_ir`。本次外循环的 physics diagnosis 未生成 verified culprit，反而归为 evaluator/insufficient evidence 并拒绝 CAD regeneration，因此冻结版无法自行修复。
- Dynamic gate 盲区：短 probe 只检查 driver travel/contact A/B，未检查 revolute body 的世界轴心是否保持不动。

## 6. Windpump

- Run：`06_windpump/a_windmill_driven_reciprocating_20260811_123545`；CAD 在 iteration 2 完成，随后从该 artifact 续跑 physics。
- 数值：PASS；rotor input `12.0012 rad`，pump output span `27.422 mm`，3 次方向反转，6/6 downstream，stability PASS。
- Dynamic gate：contact-on/off 均 `1.04954 rad`，无 active contacts，`nq=3`、`nv=3`、finite。
- 轴心轨迹：rotor shaft、wind rotor、crank disk 的世界中心全程固定，axis span 均 `[0,0,0]`；不存在整根轴公转。
- 曲柄轨迹：crank pin 在 XZ 平面形成约 `28 mm × 28 mm` 圆周运动；connecting rod 随之摆动。
- 输出轨迹：piston slider、pump rod、pump piston 的 X/Y 全程固定，只沿 Z 往复，span `27.424 mm`。
- 刚性携带：pump rod–slider 距离恒定 `24 mm`，piston–rod 距离恒定 `1 mm`。
- 视频视觉：首/中/末帧中风轮姿态明显变化，支撑轴位置固定，机构未飞离；泵杆工作区部分被结构遮挡，但轨迹直接确认周期往复。
- 冻结版结论：数值、轨迹和可见结构 PASS。

## 总结

| 项目 | Runner | 机械数值/轨迹 | 视频证据 | 主要问题 |
|---|---|---|---|---|
| 二级减速器 | PASS | PASS | 啮合 INCONCLUSIVE | 上盖板遮挡 |
| 钟表 | PASS | PASS | PASS/PARTIAL | 部分齿面遮挡 |
| 三行星 | FAIL | PASS | 轨迹 PASS，视频遮挡 | `_metrics_runner` 双边区间解析错误 |
| 四行星 | PASS | PASS | 轨迹 PASS，视频遮挡 | 上盖板遮挡 |
| 曲柄滑块 | FAIL | FAIL | FAIL | agent IR 的 world hinge anchor 错误，轴心公转；外循环未正确归因重造 |
| Windpump | PASS | PASS | PASS | support_ground 格式尾项 |

冻结版本能在 6 项中完成 4 项 runner PASS；若按机械轨迹判定则 5 项通过。未能独立走完全程的关键缺口是：

1. evaluator 从自然语言 `expected` 重解释双边区间，制造三行星假失败；
2. dynamic gate 未验证 revolute body 世界轴心固定，漏过曲柄滑块 IR anchor 错误；
3. physics attribution 未将该明确 IR frame fault 路由回 single-agent 重造；
4. 多项视频被盖板遮挡，数值 PASS 不能自动升级为视觉机械 PASS。

## 7. 开放式四自由度串联机械臂

- Run：`07_serial_robot_arm/an_open_frame_four_degree_of_fre_20260811_132222`。
- CAD：iteration 0 因未导入 `Cone` 失败；iteration 1 成功生成 15 parts、0 gear/mesh、0 transmissions。
- MJCF topology：PASS。Agent 生成纯串联树 `base_yaw_revolute → shoulder_pitch_revolute → elbow_pitch_revolute → wrist_pitch_revolute`，没有齿轮、皮带或 transmission equality；四个真实 hinge 均存在，gate candidate 1 accepted。
- Runner：FAIL。IR 将 `shoulder_link` 标为唯一 driver；scenario designer 三次要求驱动根部 `base_yaw_revolute`，但 deterministic role override 每次都强制改回 `shoulder_link`，最终实际驱动 `shoulder_pitch_revolute`。
- 运动结果：shoulder 仅 `0.0001 rad`，0/3 downstream，pointer displacement `0 mm`；首/中/末帧完全不变。
- 重力行为：settle 阶段 elbow pin、pointer、wrist、elbow、shoulder 分别移动约 `82–148 mm`，说明无保持力矩的被动串联臂在驱动前已塌下；现 stability gate 仅以 `>0.5 m` exploded 为硬失败，因此仍误报 stability PASS。
- 冻结版结论：MJCF serial body tree/hinge lowering PASS；完整端到端 benchmark FAIL。主要缺口是 driver role 与 scenario 目标冲突、无多关节保持/控制策略，以及 stability gate 对机械臂塌落过松。
