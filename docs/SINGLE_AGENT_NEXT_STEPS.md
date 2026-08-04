# 单 agent text-to-cad 路径：待办与优先级

承接 `v0.3.0-single-agent`。本文档是**待办清单**，截至撰写时这些都还没动代码。

物理侧的实测依据在 `docs/CONTACT_PHYSICS_FINDINGS.md`（哪些声明能交还给物理引擎、
哪些不能，含六个维度的失败数据）。本文档只讲**接下来做什么**。

优先级：**P0 储能释放类机构（②③）> P1 SDF 碰撞（⑤）> P2 闭环拓扑（④）> P3 前端 SSE（⑥）**

---

## P0 ② `drive:null` 时不记录轨迹也不录像

**③ 的前置，且不会被 ③ 自动解决。**

`evaluator/run_scenario_mujoco.py:341`：

```python
if driver is not None and driver_dofadr is not None:
    for s in range(drive_steps):
        ...
        _sample_traj(s)     # 轨迹记录在门里面
        capture(nf)         # 录像也在门里面
```

轨迹记录和录像都被这个 `if` 包住。投石机这类机构 `control()` 里什么都不做 → 依然没有
driver → 门依然为假 → 循环整体跳过 → 依然没有 `trajectory.json` 也没有 mp4。

**改法**：拆掉这个门，循环无条件执行，每步调用 `control(m, d, t)`（哪怕是空操作），
记录和录像照常。与 ③ 在同一处代码，一次改完。

**实证**：投石机 run `1kg_10m_20260730_125604` 因此完全没有可评测的数据。

---

## P0 ③ designer 写 setup/control/check 三个自由函数

取代现在的 `drive` schema 填空。

### 为什么

现在 `drive.mode` 只能选 `velocity` / `position_sweep`。投石机因此被当成"电机匀速转"
——实测 `input_joint == output_joint == place_throwing_arm`，手臂被 5 rad/s 匀速转了
两圈多，完全没有蓄势/释放/抛出的过程。

**schema 填空题涵盖不了所有机构。**而 `metrics_code` 已经是自由代码且效果好（它自己
想出了"逐级 travel 比值"来定位是哪一级漏了，那不是我们教的）。这是把同一思路往前扩
一步，不是新范式。

### designer 输出

```python
def setup(m, d):
    """开局姿态。想上弦就 d.qpos[...] = -1.0，完全自由。"""

def control(m, d, t):
    """每步做什么。可驱动 / 可什么都不做（投石机纯重力）/
       可分段（前 1 秒驱动后放手）/ 可多关节按相位。"""

def check(traj, result):
    """已存在，不变。"""
```

### runner 固定不变

**这是跨轮次可比性的来源**（用户明确要求）：mj_step 循环、轨迹记录格式、录像、超时、
异常捕获、沙箱隔离（照抄 `evaluator/_metrics_runner.py` 的模式）。

反例：上次那台表 23 轮里 ratio 从 0.567 飘到 106.478，一部分原因就是判据每轮重写。
记录格式必须固定，否则跨轮次没法比。

### 已知代价（已讨论并接受）

- 失败模式从"选错模板"变成"代码跑不通"
- 驱动代码会**写** `d.qpos`，破坏力比只读的 metrics 大
- 多一个嫌疑人：机器不行 vs 驱动代码写错
- 性能（每步跨 Python 边界）——用户明确表示不担心

### 顺带删除

`joint_pose` 这个 spec 字段可以删掉。它从来没被读进 `d.qpos`（grep 零命中），
`setup()` 替代它。实测手动上弦到 -1.0 rad 后纯重力可甩到 1017 rad/s，说明物理本身
没问题，缺的只是设置初始姿态的能力。

---

## P1 ⑤ 用 SDF 碰撞替代凸分解

**④ 的前置**：间隙测准了，配合识别才有意义。

MuJoCo `type="sdf"` 直接吃我们的 STL，不需要凸分解也不需要插件。实测时针/管
0.05mm 间隙：

| 碰撞几何 | ncon | 报告穿透 |
|---|---|---|
| 凸分解 | 4 | **-0.5500mm**（假接触，误差是设计间隙的 11 倍） |
| SDF | 0 | **0.0000mm**（正确） |

齿轮啮合场景还**快 17 倍**（2.9s vs 50.0s）。

**能干掉的假阈值**：`support_test._FIT_GAP_MM = 0.3`（mm）、以及
`mjcf_builder._PRESS_FIT_CLEARANCE_M = 0.0001`（米，即 0.10mm）。实测真实转变点在
**0.001~0.005mm**，比现有阈值小两个数量级——现有阈值主要是为凸分解误差留的余量。

> **`_PRESS_FIT_CLEARANCE_M` 已于 2026-08-04 删除**，而且不是靠调小：改判 interference
> 的**符号**（`shaft_r - bore > 0` 即压配），与 `_is_press_fit_overlap` 统一。任何绝对
> 毫米阈值都是尺度相关的，必然对某类机器是错的。它让一整轮 7 iteration 的手表全废：
> `hour_pipe` 留了 0.050mm 活动间隙被判成压配，焊死在分针轴上，而它自己的啮合要求
> 0.122，两条 equality 打架 → 求解器折中出 1.90:1（设计是 12:1）。齿轮每轮都是对的，
> agent 却被告知"传动比错"，一直在重切本来就正确的齿。

**未验证的风险**：只测过两零件沙箱，几十零件完整装配的性能与稳定性未知。

⚠️ **排查陷阱**：`mjGEOM_PLANE == 0` 与"未设置"无法区分，读 `geom_type` 必须**按名字
读**（`MESH=7`、`SDF=8`）。我因为读了 index 0（地板）两次误判 SDF 不可用。

---

## P2 ④ `mount=` 是树，表达不了闭环框架

### 问题

每个零件只有一个 `mount` 字段 ⇒ 只能表达一棵树。一根轴同时被左右两个轴承支撑是
**闭环**，agent 只能挑一边写，另一边成为纯装饰。

实证：投石机 `right_pivot_bearing` 下面空空如也，支撑测试报它掉了 929mm。

### 次生后果：穿模

21 个零件串成一条深父子链，而 **MuJoCo 默认不检测直接父子刚体间碰撞** ⇒ 配重/篮子/
弹丸可以穿过底座和立柱。

实测排除了其他解释：7 条 exclude 里没有一条涉及 base/upright，碰撞开关
（`contype`/`conaffinity`）也全开——纯粹是父子豁免。

### 方向

`<equality><connect>` / `<weld>` **不受 body 树限制**，仓库里 equality 目前只用于齿轮
比。识别所需的三个几何条件现有代码里全都有（`coaxial_pairs` 共轴、`_radial_extent_m`
孔轴半径匹配、`_really_fits` 轴向重叠），只是产出要从"要不要 exclude"改成"生成什么
约束"。

间隙大小可以决定约束类型，不再是二值 hardcode：

| 间隙 | 物理含义 | 生成 |
|---|---|---|
| 负（过盈） | 压配，锁死 | `weld` |
| 0 ~ 0.05mm | 转动配合 | `connect` + 沿轴 hinge |
| > 0.05mm | 松，会晃 | 不生成，靠接触 |

阈值仍然存在，但它对应**真实工程配合等级**（H7/h6 之类），不是为网格误差留的余量。

⚠️ **向后兼容**：如果把 `mount` 拆成 `rests_on` + `driven_by`，读 json 时必须回落到
旧字段，否则旧 run 的模型会解析成"所有零件互不相连"：

```python
mount = d.get("mount", "")
rests_on = d.get("rests_on", mount)
driven_by = d.get("driven_by", mount)
```

（旧 run 的**视频和 GLB 不受影响** —— 回放走事件流，GLB 只读 URDF + STL，都不碰
`mount`。）

---

## P3 ⑥ physics 异常后前端观感卡住

`single_agent.py:548` 的 except 里 `return result`，**run 其实正常返回了**。所以卡的
不是 Python。

待查 SSE 那一侧：`end` 事件有没有发出去、子进程句柄有没有回收。

（我之前误判成"控制流没收敛"，已修正。）

---

## 已被取代

- **① `joint_pose` 从未生效** —— 被 ③ 取代。designer 直接写 `setup(m, d)` 设
  `d.qpos`，这个字段可以删掉，不需要"接线"。

---

## 更远的：equality 携带物理参数

今天实测发现：软化接触后，法向力对过盈量是**线性**的（胡克定律的形状，比值稳定在
1.1~1.8e14）。所以"过盈产生多大夹紧力"是可以标定的。

设想：用 Lamé 厚壁圆筒公式（课本解析解，不需要 FEM）算出真实保持扭矩，让 equality
不再是二值的"锁/不锁"，而是携带真实物理参数——过盈大就夹得紧、载荷超了就滑脱。

⚠️ **已查证的障碍**：MJCF 的 equality **没有任何 force limit 或断裂阈值**属性（只有
`active` / `solref` / `solimp`）。"超过阈值就滑脱"必须我们自己在仿真循环里做：每步读
`d.efc_force`，超了把 `d.eq_active` 置 0（实测确认这两个都可读写）。

更原生的替代：`joint frictionloss`（关节级静摩擦阈值，天然有 stick-slip 语义）或
actuator `forcerange`。**`frictionloss` 在压配场景下的表现还没测过**，可能是更好的路。

依赖 ⑤ 先把间隙测准，否则标定的输入就是错的。
