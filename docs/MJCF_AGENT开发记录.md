# MJCF Agent 化开发记录

> 状态：持续维护中
> 起始日期：2026-08-10
> 最后更新：2026-08-10
> 适用范围：`maker2` 的 KinematicModel → MuJoCo MJCF → support probe → scenario/evaluator 路径

## 1. 文档目的

本文记录从确定“让专用 agent 针对每个具体机构编译 MJCF”以来的全部架构决策、代码修改、修改理由、验证结果、失败教训和未完成项。

以后凡是修改以下内容，都必须同步更新本文：

- MJCF facts、agent prompt、agent tool loop、emitter、validation gate；
- MuJoCo support derivative、scenario runner、传动测量和 fault attribution；
- agent MJCF 的缓存、失败路由、benchmark 和 golden；
- 任何会改变 KinematicModel 到 MJCF 语义的代码。

本文只记录 MJCF agent 化及其直接下游，不替代 `maker2/PIPELINE.md`。

---

## 2. 为什么改成 MJCF agent

### 2.1 根因不是 XML 难写，而是 body tree 没有固定通用解

旧 `mjcf_builder.py` 用确定性启发式逐步扩充结构支持：

1. 钟表要求多根同轴但不同速的轴，不能把“同轴”当成 weld 或 1:1；
2. 曲柄滑块和四连杆是闭环，不能把所有机械连接都塞入一棵 body tree；
3. 行星机构要求 carrier 携带 planet center 公转，同时 planet 在 carrier 局部 hinge 上自转；
4. 机械臂、云台等串联机构又确实应使用普通父子运动树；
5. 压配齿轮、指针、手柄等从属件应刚性随 carrier 运动，不应凭名字虚构 `<link>_spin` joint。

此前每支持一种机构，就向确定性 builder 增加一组规则。规则之间互相冲突：修曲柄滑块会影响树继承，修行星会影响同轴钟表，修钟表又可能破坏普通齿轮传动。

因此本次改造的中心原则是：

> 对每个完整 `KinematicModel` 做一次案例级机构拓扑分析，同时决定 body tree、非树闭环、刚性携带、独立同轴坐标、传动 equality 和 contact exclude；不再由多个互不知情的确定性函数分段猜测。

### 2.2 权限边界

**MJCF agent 负责决策：**

- body parent graph；
- 实际 generalized coordinates；
- tree edges 与 closure edges；
- 刚性携带关系；
- 独立同轴坐标；
- transmission lowering；
- contact 保留或 exclude；
- support probe 应移除的约束或应释放的 body；
- 每个 authored entity 如何被生成节点兑现。

**确定性代码只负责：**

- 提供不可变的 IR 和测量事实；
- 注入真实 mesh、质量、COM、惯量和 frame；
- 限制 agent 编译脚本能力；
- 执行脚本；
- 验证 XML、manifest、MuJoCo load 和有限状态；
- 接受或拒绝，绝不静默补 constraint/exclude。

### 2.3 失败路由原则

- agent 编译脚本、XML、manifest 或 load gate 失败：`builder_compiler`；
- runner 的 scenario、joint 命名或 metrics 失败：`runner_scenario`；
- 数值 NaN/Inf：`simulator_numerics`；
- evaluator 自相矛盾或误判：`evaluator`；
- 只有被事实验证的 `agent_geometry` / `agent_ir` 且有明确 culprit，才允许 CAD agent 重造。

禁止 agent MJCF 编译失败后静默 fallback 到 legacy builder，也禁止因此触发 CAD 全量再生成。

---

## 3. 新的总体数据流

```text
machine.py / MECHANISM
        ↓
KinematicModel（机械语义权威输入）
        ↓
extract_mjcf_facts()
        ↓
mjcf_facts.json（不可变事实）
        ↓
MJCF compiler agent
  ├─ read_mjcf_facts
  ├─ query_port_fit
  ├─ query_pair_geometry
  ├─ query_motion_path
  ├─ submit_compiler_source
  └─ run_mjcf_gate
        ↓
mjcf_compiler.py
        ↓
受限 MJCFEmitter
        ↓
model.agent.candidate.mjcf + builder_manifest.json
        ↓
确定性 validation gates
        ↓
model.mjcf（仅通过后原子提升）
        ├─ normal scenario / physics
        └─ manifest-declared support derivative
```

Agent 输出紧凑 Python：

```python
def compile_mjcf(facts, out):
    ...
```

而不是直接生成长 XML。这样可复用 loops、保持输出短、便于 gate 和缓存。

---

## 4. 逐文件修改记录

## 4.1 `maker2/mjcf_facts.py`（新增）

### 修改

新增不可变 MJCF facts 提取器：

- `extract_mjcf_facts(model, ctx, settings=None)`；
- `facts_hash(...)`；
- `query_port_fit(...)`；
- `query_pair_geometry(...)`。

事实包含：

- canonical KinematicModel；
- 稳定 entity IDs；
- link 世界 frame；
- STL 相对路径与 hash；
- bounds、volume、watertight；
- density、friction、mass、COM、完整 inertia；
- local/world port frames；
- simulation settings。

`query_pair_geometry()` 后续改为加载实际 STL 并应用世界位姿，报告 surface distance、AABB overlap、可选实体交叠和相对变换，而不是只比较 link 原点。

### 理由

Agent 需要的是可核验事实，不能让旧 builder 先给出“应该 weld/exclude”的结论，否则只是把旧启发式包装成 agent。

### 已知未完成

- `query_nearby_parts()` 目前仍按世界原点距离，不是表面距离；
- pair query 尚需按 mesh hash + transform 做显式缓存；
- 部分事实可继续压缩，避免 agent 对话携带不需要的矩阵。

---

## 4.2 `maker2/mjcf_emitter.py`（新增）

### 修改

新增受限 emitter API：

- `out.topology_plan(plan)`；
- `out.body(name, parent='')`；
- `out.joint(...)` / `out.freejoint(...)`；
- `out.weld(...)` / `out.connect(...)`；
- `out.joint_equality(...)`；
- `out.exclude(...)`；
- `out.decision(...)`；
- `out.support_patch(...)`。

Emitter 自动从 facts 注入：

- body 世界/局部 frame；
- mesh 路径及毫米到米 scale；
- mass、COM、full inertia；
- material friction。

Agent 不能伪造这些测量数据。

### equality ABI 修正

最初接口只写 `joint1/joint2/ratio`，但没有明确机械方向。真实 reducer 暴露出 MuJoCo XML equality 的参数方向容易被误解。

当前接口语义明确为：

```text
out.joint_equality(name, driving_joint, driven_joint, ratio, offset)
=> driven = offset + ratio × driving
```

由于 MuJoCo XML 的 polynomial 由 `joint1` 对 `joint2` 定义，emitter 会按该机械 ABI 排列 XML references。

### 理由

调用者必须使用符合 `TransmissionSpec` 的“驱动 → 从动”语义，不能要求每个 agent 记忆 MuJoCo XML 的反直觉参数方向。

---

## 4.3 `maker2/mjcf_validation.py`（新增）

### 修改

新增 acceptance-only gate：

1. Python AST allowlist；
2. disposable subprocess + timeout；
3. allowed MJCF tags；
4. link/body exact coverage；
5. body/joint/mesh 唯一性；
6. mesh path allowlist；
7. constraint references；
8. XML/manifest exclude 一致性；
9. 每个 entity 恰好一个 decision；
10. decision reason、fact IDs、generated nodes；
11. topology plan 与 coordinate map 基础检查；
12. exclude provenance；
13. support patch reference；
14. `MjModel.from_xml_path`；
15. `MjData + mj_forward` finite；
16. bounded smoke 的 qpos/qvel/qacc/xpos finite。

脚本 allowlist 后续允许了安全容器/字符串方法：

- `append`、`extend`、`update`、`setdefault`；
- `startswith`、`endswith`。

### 理由

首次真实 reducer agent 使用普通 list/dict 构造脚本时，被过窄 AST gate 拒绝；这是 compiler infrastructure 错误，不应消耗 agent 轮次或被误判为 mechanism 问题。

### 已知未完成

仍需增加更强的语义 gate：

- XML parent graph 与 `topology_plan.tree_edges` 完全一致；
- tree/closure 不重复；
- 每个 closure 对应 authored relation；
- 每个 transmission 对应实际 equality；
- `represented_by` 不得任意指向无关节点；
- independent coaxial 不得被 weld/equality 锁死；
- carried center 随 parent 运动；
- closure residual bounded。

---

## 4.4 `maker2/prompts/mjcf_compiler_prompt.py`（新增）

### 修改

新增专用 MJCF compiler system prompt，当前版本为 3。

Prompt 强制 agent：

- 先分析整个机构，再写脚本；
- 区分串联树、闭环、行星 carrier、钟表独立同轴；
- 生成完整 `topology_plan`；
- 每个 authored entity 生成 decision；
- 每个 constraint/exclude 带 reason、source entity IDs、fact IDs；
- 禁止虚构 joint；
- 禁止把“同轴”自动理解为 weld/1:1；
- 只允许 support patch：`remove_constraint` / `free_body`；
- 使用精确 emitter ABI，不直接传 MuJoCo `polycoef`。

### 理由

首个 reducer candidate 曾错误调用：

```python
out.joint_equality(..., polycoef=(...))
```

并试图使用不支持的 support action `weld`。这不是拓扑推理问题，而是 prompt 与 emitter ABI 没同步，因此补充了精确签名和含义。

---

## 4.5 `maker2/mjcf_agent_compiler.py`（新增）

### 修改

新增单 artifact、有界、持续 conversation 的工具循环：

- `compile_agent_mjcf(...)`；
- 最多 20 轮；
- 最多 5 个 candidate；
- 失败报告回给同一个 agent 修脚本；
- 不调用 CAD agent；
- 通过后写入：
  - `mjcf_facts.json`；
  - `mjcf_compiler.py`；
  - `mjcf_gate_report.json`；
  - `mjcf_agent_trace.json`；
  - `model.mjcf`；
  - `builder_manifest.json`。

新增缓存：

```text
~/.cache/physcad/mjcf_agent/<artifact-hash>
```

或 `settings.mjcf_compiler_cache_dir`。缓存保存 agent 编译脚本，不盲信旧 XML；命中后重新执行脚本并重跑 gate。

### facts 分页问题及修复

第一版 `read_mjcf_facts(offset, limit)` 按 JSON 字符切片，agent 却按条目分页理解。已改为按顶层 list/dict entry 分页，并返回：

- `section`；
- `offset`；
- `returned`；
- `total`；
- `next_offset`；
- `items`。

单页最多 25 项，单结果约束在约 16k 字符。

### facts 输入压缩

真实 reducer 暴露出 `links`、`ports` 页面过大。最初 agent 会反复读入：

- 完整 $4×4$ matrix；
- local/world quaternion；
- mesh hash；
- bounds、volume；
- 完整 inertia。

这些数据多数不参与拓扑判断。现改为 agent 默认页面只返回：

- link：entity ID、DOF、轴、driver、mount、mass、world xyz/quaternion；
- port：entity ID、type、axis、直径、深度、pitch radius、world xyz；
- 完整几何仍保留在 facts 中，供 emitter 自动注入或 query tool 按需访问。

### 理由

第二轮 API HTTP 400 的真实原因不是 dotenv，也不是模型 context 不够，而是本地 `Conversation` 的 100k 字符历史截断后产生空 messages。压缩 facts 可以降低成本并减少截断概率。

---

## 4.6 `maker2/agent_tool_runtime.py`

### 修改

- `run_agent_tool_loop()` 新增 `history_max_chars`；
- 每轮显式传给 conversation；
- 若截断后 messages 为空，立即抛出明确错误，而不是发送空 API request。

MJCF compiler 当前使用 300k 字符受控窗口，同时依靠 facts 压缩避免实际接近该上限。

### 理由

原行为会把空 messages 发送给 gateway，得到难以理解的：

```text
One of "input" or "previous_response_id" or "prompt" or "conversation"
must be provided.
```

应在本地 fail fast。

---

## 4.7 `maker2/llm/conversation.py`

### 修改

修复 tool-call/result 成组截断：

- 仍禁止拆开 assistant tool call 与 tool results；
- 若保留后缀以 assistant tool group 开头，则插入最初 user instruction 作为 anchor；
- 不再为了“必须以 user 开头”把整个完整 tool group 全部删掉，导致空请求。

### 理由

真实 reducer 首轮/多轮读取 facts 后累计超过 100k 字符。旧截断逻辑保留最新 tool group 后，又因第一条不是 user 而全部 `pop(0)`，最终 gateway 收到空 conversation。

这是通用 tool-loop infrastructure bug，不是 MJCF agent 推理失败。

---

## 4.8 `maker2/mjcf_support_derivative.py`（新增）

### 修改

新增 `derive_support_mjcf(...)`：

- 复制 accepted MJCF；
- 只应用 accepted manifest 声明的 support patches；
- selector 必须命中；
- 不重新解释 mount、coaxiality 或 part name；
- 不进行第二次 LLM 调用。

### 理由

旧 support path 会自行拆 weld、增加 coaxial/closure/touching carrier credits，可能把正常闭环拆坏，或者替错误 CAD“演出”支撑。

### 当前问题

真实 reducer manifest 声明 `free_body(baseplate)`，但 support derivative 报：

```text
support_ground cannot be freed
```

目前 support 被跳过。该问题尚未修完，不能把“support unavailable”当作 support PASS。

---

## 4.9 `maker2/mjcf_builder.py`

### 修改

- 旧 builder 重命名为 `build_mjcf_legacy(...)`；
- 新 `build_mjcf(...)` 成为 facade；
- 有 `Settings` 时默认 `mjcf_compiler_mode='agent'`；
- 显式 `legacy` 才进入旧 path；
- agent 失败不 fallback legacy；
- 无 Settings 的旧库级 tests 暂时仍走 legacy，避免一次性破坏全部历史 fixtures。

此前为 legacy path 增加：

- `_existing_scalar_joint(...)`；
- `_ensure_compound_follower_joint(...)`。

它们用于阻止旧 builder 引用虚构 `<link>_spin`，但不作为新生产拓扑编译器继续扩张。

### 理由

保持外部调用边界和 artifact contract，同时停止在旧 builder 中继续堆结构特例。

---

## 4.10 `maker2/config.py`

### 修改

新增/调整：

```python
mjcf_compiler_mode = "agent"
mjcf_compiler_max_tokens = 32000
mjcf_compiler_cache_dir = ""
```

MuJoCo 仍是默认 physics backend；Chrono 保留为可选 sidecar，不是本改造默认。

---

## 4.11 `maker2/physics.py`

### 修改

- MuJoCo path 捕获 MJCF compiler exception；
- 返回 verified `builder_compiler` diagnosis；
- compiler failure 不进入 CAD refinement；
- timed-out MuJoCo subprocess 不再在当前进程重跑；
- 非有限状态归 numerical health；
- metrics broken-check 与 machine functional failure 分流；
- accepted manifest 路径写入 metrics。

### 理由

旧行为可能在 subprocess timeout 后再 in-process 运行同一不稳定模型，导致前端继续等待甚至被 NaN/Inf 循环拖死。

### 当前暴露问题

真实 reducer 的 scenario designer 第一次仍生成旧 joint 名：

```text
input_shaft_spin
output_shaft_spin
```

retry 后才使用 manifest 中的真实 joint。说明 metrics/scenario prompt 的一条路径尚未完全使用 `coordinate_map`。

---

## 4.12 `evaluator/run_scenario_mujoco.py`

### 修改

- 每步检查 qpos、qvel、qacc、xpos finite；
- 首次 NaN/Inf 立即失败；
- 读取 manifest v3 的 `topology_plan.coordinate_map`；
- `joint_info()` 优先使用 agent 实际 emitted joint name；
- trajectory 记录真实 joint name；
- 区分有限力矩 servo 与精确 `direct_qpos` fixture；
- `direct_qpos` equality graph 做硬投影。

### `direct_qpos` 修正

真实 reducer 调查中发现：旧 direct fixture 会：

1. 设置 input qpos；
2. 投影 followers；
3. `mj_forward`；
4. 随后仍 `mj_step`；
5. 接触冲量在采样前把 follower 推离投影值。

现改为 direct fixture 不执行 dynamics step，只做：

```text
设置 input → equality 投影 → qvel/qacc 清零 → mj_forward
```

servo 模式仍执行 `mj_step()`，保留有限力矩、接触阻塞和负载真实性。

### 当前调查结论

Reducer 的 MJCF 单独 MuJoCo 探针已证明 equality 正确：

```text
compound / input ≈ -1/3
output / input ≈ 1/12
```

但完整 scenario 使用的是 servo，不是 direct_qpos。servo 下实际轨迹为：

```text
input delta       ≈ 1.291 rad
compound delta    ≈ 19.326 rad
output delta      ≈ -4.829 rad
```

这说明有限力矩+contact 下 equality 被显著拉离。已完成“保留接触 vs 全关闭接触”的相同 servo A/B：

```text
保留接触：
input delta       = 1.29045
compound delta    = 19.34183
output delta      = -4.83592
compound/input    = 14.98848
output/input      = -3.74748

关闭全部接触：
input delta       = 25.48721
compound delta    = -8.49475
output delta      = 2.12271
compound/input    = -0.33329
output/input      = 0.08329
```

关闭接触时与声明的 $-1/3$、$+1/12$ 完全一致，证明：

- agent 的 transmission 拓扑正确；
- emitter 的 equality 方向正确；
- MuJoCo equality 本身足以维持理想传动；
- 失败由动态接触触发，而不是 KinematicModel 信息不足。

下一步必须记录 drive 期间的具体 contact body pair、penetration 和 impulse，定位 agent 漏掉的 exclude、SDF collision artifact 或真实 CAD 干涉。尚不能在没有具体 contact 证据时归因给 CAD。

---

## 4.13 `maker2/support_test.py`

### 修改

- agent mode 使用 accepted manifest 派生 support MJCF；
- 禁用 legacy 的 coaxial fit、closure neighbor、touching carrier 启发式 credits；
- legacy mode 保留旧行为用于 debug。

### 理由

support probe 必须测试 agent 已接受拓扑在明确 patch 下的支撑，不能由第二套确定性规则重新发明机构语义。

---

## 4.14 `maker2/assembler.py`

### 修改

agent mode 不再在子装配/assembly 中间阶段调用 MJCF agent；只在最终完整 KinematicModel 上编译一次。

### 理由

body tree、跨子装配 closure 和 transmission 需要看到完整机器。对子装配单独编译既贵，也可能得到无法组合的局部树。

---

## 4.15 `maker2/benchmarks/compile_gate.py`

### 修改

agent mode 跳过 per-sub MJCF compilation，只保留 deterministic KinematicModel/mesh/facts gate，最终 assembly 才调用 agent。

### 理由

与 assembler 相同：结构拓扑必须全局分析。

---

## 4.16 `evaluator/attribution.py`

### 修改

- 引入六域 fault schema；
- `compare_authored_ir_compiled()` 读取 manifest v3 decisions；
- compiler、runner、numerics、evaluator fault 不再伪装成 CAD fault；
- 未经验证且没有 culprit 的 structure 猜测转换为 evaluator/harness halt。

### 理由

过去下游 builder/evaluator 错误会广播给 boss/manager，导致 CAD 重新设计，既慢又掩盖真实 harness bug。

---

## 4.17 测试文件

### `maker2/tests/golden_mjcf_agent_compiler.py`（新增）

验证：

- fake tool client；
- facts read；
- source submission；
- gate；
- manifest coordinate map；
- cache hit 不再次调用 LLM。

后续随结构化 facts API 更新，把旧 `section='all'` 改为 `section='index'`。

### `maker2/tests/golden_mjcf_agent_validation.py`（新增）

验证：

- accepted minimal compiler；
- validator 不自动添加 equality/exclude；
- unknown body rejection；
- unsafe import rejection；
- equality driving→driven ABI 的 XML reference 顺序。

### `maker2/tests/golden_mjcf_support_derivative.py`（新增）

验证 support derivative 只应用 manifest patch，不自行推断。

### `maker2/tests/golden_agent_team_protocol.py`

新增 conversation compaction 回归：即使最新完整 tool group 本身超过窗口，也必须保留初始 user anchor 和完整 assistant/tool pair，不能生成空 API request。

---

## 5. Builder manifest v3

当前 manifest 记录：

- `manifest_version: 3`；
- `engine: mujoco-agent`；
- `topology_plan`；
- `decisions`；
- `excludes`；
- `support_patches`；
- bodies、joints、constraints inventory。

稳定 entity IDs 包括：

- `link/<name>`；
- `pose/<name>`；
- `port/<link>/<name>`；
- `relation/<name>`；
- `motion_joint/<name>`；
- `transmission/<name>`；
- `planetary_stage/<name>`；
- driver/output/watch roles。

每个实体必须恰有一个 decision。每个 constraint/exclude 必须有 reason、source entity IDs 和 fact IDs。

`coordinate_map` 是 runner 的权威 joint 映射，禁止再猜 `<link>_spin`。

---

## 6. 首个真实 benchmark：二级齿轮减速器

Artifact：

```text
output/agent_mjcf_benchmarks/two_stage/
  a_hand_cranked_two_stage_spur_ge_20260810_172810/
```

### 6.1 CAD/IR 状态

- 25 parts；
- 2 mesh pairs；
- 14 relations；
- 3 motion joints；
- 5 transmissions；
- 7 watch links；
- STEP/STL/URDF/KinematicModel 已生成。

本轮所有 MJCF 调查均复用该 artifact，没有重造 CAD。

### 6.2 第一次 compiler agent 失败

Agent 已正确分析出：

- 三根独立 shaft hinges；
- press-fit accessories 刚性随轴；
- 两级齿轮 equality；
- journal contact 保留；
- press-fit 与理想 gear mesh contact exclude。

失败原因是 protocol/infra：

- AST 禁止 `append/update`；
- emitter 不接受 agent 猜测的 `polycoef=`；
- agent 猜测 support action `weld`；
- decision 有空 `generated_nodes`；
- repair 轮次不足。

对应修复：allowlist、明确 ABI、明确 support actions、加强 decision gate、轮次 12→20、candidate 4→5。

### 6.3 第二次失败：HTTP 400

首轮多工具读取 facts 后，Conversation 截断产生空 messages，gateway 返回：

```text
One of "input" or "previous_response_id" or "prompt" or "conversation"
must be provided.
```

修复：结构化分页、facts 压缩、user anchor、history window、empty-request fail fast。

### 6.4 编译成功

真实 agent 最终：

- 第一个 candidate 被 manifest gate 拒绝；
- agent 读取 report 后提交第二版；
- revision 2、candidate 2 通过；
- `model.mjcf` 成功生成；
- cache key `5044754f763c...` 命中后可无 LLM 重编译并重新 gate。

Agent 声明的理想传动：

```text
compound = -1/3 × input
output   = -1/4 × compound
总比值 output/input = +1/12
```

### 6.5 Physics 暂未通过

已区分三个问题：

1. **MJCF ideal equality 本身正确**：直接 MuJoCo 探针证实 $-1/3$ 和 $+1/12$；
2. **finite-effort servo 下轨迹严重偏离 equality**：正在用 contact A/B 探针定位；
3. **metrics_code 用逐帧绝对角增量求和**：振荡会把 travel 膨胀，导致 metrics 与端点位移不一致。

因此 benchmark 1 当前状态是：

```text
CAD/IR：成功
MJCF agent compile：成功
MJCF deterministic gate：成功
ideal kinematic transmission：成功
finite-effort/contact physics：PASS
support probe：不可用，仍属 harness 尾项
最终 benchmark：正常 physics/功能 PASS；support derivative 未覆盖
```

不能将当前失败归为 CAD structure fault。

---

## 7. 已通过验证

截至 2026-08-10：

```text
py -3.14 -m maker2.tests.golden_mjcf_agent_compiler          PASS
py -3.14 -m maker2.tests.golden_mjcf_agent_validation        PASS
py -3.14 -m maker2.tests.golden_mjcf_support_derivative      PASS
py -3.14 -m maker2.tests.golden_agent_team_protocol          PASS
```

真实 reducer：

- compiler agent accepted；
- MuJoCo load/finite gate accepted；
- equality 单独探针为正确 $12:1$ reduction；
- 完整 finite-effort scenario 尚未通过。

---

## 7.1 六项顺序 benchmark 最终结果

| 顺序 | Benchmark | 最终结果 | 核心证据 |
|---|---|---|---|
| 1 | 二级齿轮减速器 | PASS | input 12.7905 rad，output 1.0647 rad，12.0138:1，6/6 downstream |
| 2 | 镂空钟表 | PASS | minute 15.0 rad，hour 1.25 rad，精确 12.0:1，独立同轴与压配指针正确 |
| 3 | 三行星轮减速器 | PASS | sun 12.0 rad，carrier 3.0 rad，4.0:1，三个 planet 公转+局部自转 |
| 4 | 四行星轮减速器 | PASS | input 12.0 rad，carrier 3.0 rad，4.0:1，四个 planet 公转+局部自转 |
| 5 | 曲柄滑块 | PASS | crank 11.9999 rad，双向 slider stroke 44.018 mm，spanning tree + closure |
| 6 | windpump | PASS | rotor 12.0015 rad，pump stroke span 22.746 mm，4 次反转，5/5 downstream |

所有最终 PASS 均复用了已生成 CAD/IR；MJCF compiler、contact policy、runner 或 evaluator fault 未触发 CAD 重造。各 run 均生成视频。部分 run 仍有 support derivative `support_ground` 格式尾项，详见未完成工作。

## 8. 尚未完成的工作

### P0：完成 reducer benchmark 1

- 已将 legacy builder 的逐 pair exclude 规则迁移到 MJCF agent prompt v4；
- 已将 nearby 查询从 link 原点距离修为放置后 world-AABB 表面距离，并升级 prompt/cache 到 v5；
- 已增加 acceptance gate：所有接触相关 relation 必须有 keep/exclude 决策，decision exclude 集合必须与 XML/manifest 完全一致；
- 已完成 contact-on/contact-off servo A/B，确认动态接触是触发条件；
- 记录 drive 期间具体 contact pair、penetration 和 impulse；
- 根据证据修漏掉的 exclude、SDF artifact 或真实 CAD attribution 中实际成立的一项；
- 修 metrics travel 算法，不用振荡累计量冒充净传动；
- 修首次 metrics_code 仍猜 `_spin`；
- 修 support_ground/free_body contract；
- 重新运行 physics，必须同时报告 ideal fixture 与 finite-effort test 的不同含义。

### P1：加强 topology validation

- 已新增 policy v2 短时 finite-effort dynamic transmission A/B gate：相同 servo 分别 contact-on/off，只有 contact-on 相对 no-contact 出现 stall/失真时才拒绝；报告真实 driver travel、每条 transmission normalized residual、active pair、最大穿透/力/累计冲量；短 probe 的 no-contact soft-equality 过渡 residual 只记 warning，不作为 contact attribution；gate 只拒绝并给证据，不自动 exclude，修复仍由同一个 MJCF agent 完成；
- tree_edges 与 XML parent graph；
- closure 唯一兑现；
- transmission equality 完整兑现；
- generated node 语义；
- independent coaxial；
- carried center；
- closure residual；
- support patch completeness。

### P2：剩余顺序 benchmark

严格按用户指定顺序、`max-iters=3`：

钟表 benchmark 已完成并 PASS：iteration 0 因生成代码含 U+2019 弯引号发生 SyntaxError；iteration 1 成功生成 31 parts、2 mesh pairs、13 relations、5 watch links。MJCF agent 生成 `minute_rotation / intermediate_rotation / hour_rotation` 三个独立坐标；minute/hour pipes 被列入 `independent_coaxial`；两根指针分别 rigid-carried；`nq=3`、`nv=3`、finite。首次 finite-effort physics 被 5 个 SDF 假接触 pair 锁死；contact-off 证明 `intermediate/minute=-0.25009`、`hour/minute=0.083314≈1/12`。policy v2 动态 gate 拒绝旧 candidate并返回 active pairs；prompt v7 的同一 MJCF agent 经过 3 个 candidate 自修后 accepted，最终 gate contact-on active contacts 为 0。servo 的 soft-equality 长时振荡曾造成 endpoint ratio 12.9123 与 metrics ratio 7.4956 矛盾；按 IR 语义改用 exact fixture（有 transmission、无 pin/revolute、且 running journal 两端均为独立 spin members）。最终结果：minute input 15.0 rad、hour output 1.25 rad、精确 12.0:1、4/4 downstream、support/stability/functional 全 PASS、视频生成。全程未按“watch”名称硬编码，未重造 CAD。仍有 evaluator 尾项：第一次 metrics 猜 `_spin`，retry 后才使用真实 coordinate map。

1. 二级齿轮减速器（PASS：12.0138:1）；
2. 钟表（PASS：精确 12.0:1，support/stability/functional PASS）；
3. 三行星轮减速器（PASS：sun input 12.0 rad、carrier output 3.0 rad、精确 4.0:1；3 个 planet 均有 carrier-local spin 且随 carrier 公转；6/6 downstream；policy v2 dynamic gate、support、stability、functional 全 PASS；视频生成。首次 metrics 仍猜 `_spin`，retry 后使用真实 coordinate map）；
4. 四行星轮减速器（PASS：input 12.0 rad、carrier output 3.0 rad、精确 4.0:1、7/7 downstream；四个 planet 均有独立 carrier-local hinge 与公转；dynamic gate/support/stability/functional 全 PASS；视频生成。首次运行的 runner_scenario `_spin` 假名根因已修：`_robot_info(model, run_dir)` 读取 accepted manifest v3 coordinate_map 与 manifest joints，initial/revise 均使用真实 trajectory keys；重跑第一次 scenario 即通过，无 retry）；
5. 曲柄滑块（PASS，iteration 0：crank input 11.9999 rad；slider 往复 stroke 0.044018 m 且双向运动；4/4 downstream；agent 选择 spanning tree + 非树 `rod_small_end_connect` closure；big-end/small-end pin-revolute pairs 明确保留 contact，无 whole-body exclude；finite-effort servo、stability、functional PASS；视频生成。尾项：support derivative 因 `support_ground` 写成描述文本而非 body name 跳过；single-agent score 打印 `-inf`，但最终 `ok=true/PASS`）；
6. windpump（PASS：iteration 0 CAD/MJCF 生成后首次 physics input 仅 0.2386 rad、1/5 downstream、output dead。先后定位并修复三层 harness 问题：① world joint anchor/axis 被误当 body-local，新增 `frame='world'|'local'` 并升级 prompt v8；② closure-only 未被 dynamic gate 覆盖，policy v3 增加 no-contact commanded-travel gate；③ below-grade pump rod 被 global ground 阻挡，新增 `out.exclude_ground` 与 policy v4；④ dedicated wrist pin 的局部轴承接触与 hinge 重复，prompt v10 区分共享 crank/web body 与 dedicated pin body。v10 agent 经 6 candidate accepted：dedicated `wrist_pin-connecting_rod` exact exclude、`pump_rod` ground exclude；gate contact-on 1.0316 rad、off 1.0503 rad，98.2% 行程，`nq=3`、`nv=3`、finite。最终 physics：wind input 12.0015 rad、pump output span 22.746 mm、4 次方向反转、5/5 downstream、functional/stability PASS、视频生成。support derivative 仍因 support_ground 格式尾项跳过，但正常 stability PASS）；

前一个未形成可信结论前，不启动后一个。

### P3：最终工程清理

- hierarchy 中 compiler fault 不广播重造；
- runner 所有路径统一读取 manifest coordinate map；
- frontend typecheck；
- 完整 dirty diff review；
- 旧 builder 仅保留显式 debug path；
- 不提交、不 push，除非用户明确要求。

---

## 9. 重要教训

1. **Single-agent 的 `MECHANISM/KinematicModel` 已足够表达机构语义。** 当前多次失败来自 compiler protocol、emitter、runner 和 evaluator，而不是必须让 CAD agent 再写一份机构。
2. **不能用“XML 能 load”代替语义验证。** Equality 方向、坐标映射和真实 qpos 比值必须做动态/投影探针。
3. **不能把 ideal fixture 与 finite-effort dynamics 混为一谈。** `direct_qpos` 回答“声明的运动学是否兑现”；servo 回答“有限力矩和接触下是否能运动”。
4. **不能让 metrics 自由累计振荡后冒充传动比。** 净位移、连续角、速度比和任务指标必须按测试目的选择。
5. **不能让 support probe 自己再发明一套机械结构。** 它只能应用 accepted manifest 的 patch。
6. **不能把 harness bug 路由回 CAD。** 未经验证、无 culprit 的 structure 猜测必须停止在 evaluator/builder domain。
7. **Agent 工具输入应按需、结构化、分页。** 完整惯量矩阵和 mesh metadata 不应在每轮对话重复出现。
8. **缓存应缓存编译决策脚本，不应盲信旧 XML。** Emitter/validation 修复后必须重新执行和 gate。

---

## 10. 后续更新格式

每次 MJCF agent 相关修改后，在本文末尾追加：

```text
日期 / 变更标题
- 现象：
- 根因：
- 修改文件：
- 修改内容：
- 为什么这样改：
- 验证：
- 尚未解决：
```

并同步更新相关章节的“当前状态”和“尚未完成”。

---

## 11. 变更日志

### 2026-08-10：建立 agent-authored MJCF 主路径

- 新增 facts、compiler agent、emitter、validation、support derivative；
- `build_mjcf` 改为 agent 默认、legacy 显式 fallback；
- builder/compiler fault 与 CAD fault 分流；
- 新增 manifest v3 与 coordinate map。

### 2026-08-10：修复真实 reducer compiler protocol

- facts 改为按 entry 分页；
- 精简 link/port 页面；
- 修 Conversation tool-group 截断空请求；
- 扩充安全 AST 方法；
- prompt 与 emitter ABI 同步；
- compiler rounds/candidate budget 提高；
- 真实 reducer revision 2 accepted。

### 2026-08-10：修复 equality ABI 与 direct fixture

- 明确 driving→driven ratio 语义；
- 新增 equality XML 方向回归；
- `direct_qpos` 不再在 hard projection 后执行 dynamics step；
- 单独 MuJoCo 探针证明 reducer 理想传动为 $12:1$。

### 2026-08-10：迁移 legacy builder exclude 规则到 agent v4

- 不把旧规则重新接回确定性 auto-repair，而是作为 agent 的强制逐 pair 分类契约；
- press fit：刚性携带/1:1 + exact pair exclude；
- running fit / journal / ball bearing：保持独立坐标或 hinge + exact pair exclude，不靠摩擦传扭矩；
- equality gear/planet mesh：exact tooth pair exclude；
- zero-solid-overlap 的 SDF/proxy 假接触：exact pair exclude；
- 正实体重叠：除声明 press fit 或理想 gear mesh 外保留 contact 并暴露 CAD 干涉；
- pin/revolute closure 禁止 whole-body exclude，避免连杆穿主轴/web；
- 不确定时保留 contact；禁止 all-coaxial、all-parent-child 等宽泛 exclude；
- validation 要求 contact_decisions 为逐 pair list，覆盖所有接触相关 relation，且 exclude 集合与 XML/manifest 完全一致。
- reducer v4 真实重编译已通过：agent 查询 6 个 press fit、6 个 journal bearing、2 个 gear mesh，并检查附近运动件；revision 2 / candidate 2 accepted。
- accepted candidate 生成 14 个精确 excludes：6 个 press-fit pair、6 个 shaft-bearing journal pair、2 个 ideal gear-mesh pair；没有使用 all-coaxial 或 parent-child 宽泛排除。

### 2026-08-10：动态接触唯一定位与 nearby v5 修复

- drive 全程唯一活跃 pair 为 `compound_shaft ↔ hand_crank`；
- 累计 contact point 计数 782,202，最大穿透 0.322 mm，最大力 270 N，累计冲量 19,211 N·s；
- 放置 STL 的真实表面距离为 10.21 mm，AABB 无正体积交叠，证明是 collision-proxy/SDF 假接触，不是 CAD 干涉；
- v4 agent 没检查该 pair，因为旧 `query_nearby_parts(30mm)` 按 origin 距离筛选，而两零件 origin 相距 39.48 mm；
- facts version 1→2；nearby 改为放置后 world-AABB 表面距离；prompt/cache 版本 4→5；
- v5 真实运行仍漏掉目标 pair：agent 自行请求 `radius_mm=1`，小于 10.2 mm AABB surface gap；工具虽不再按 origin 筛选，但仍允许 agent 用过小半径漏检；
- v6 将有效 nearby 半径下限设为目标零件包围盒半对角线，agent 的 radius 只能扩大不能缩小该尺度相关范围；
- prompt v6 强制逐对查询 nearby 返回项；不能因真实 mesh surface distance 为正就自动 keep，因为 SDF proxy 可跨大间距产生假接触；零实体 overlap 且无明确接触功能时应 exact exclude；
- 全程禁止按 `compound_shaft/hand_crank` 名称硬编码。
- v6 真实重编译已验证：agent 实际调用 `query_pair_geometry(compound_shaft, hand_crank)`，accepted manifest 记录该 pair 及几何 fact，XML 生成 exact exclude；gate 通过，`nq=3`、`nv=3`、finite。
- v6 finite-effort physics 最终 PASS：input 12.7905 rad、output 1.0647 rad、观测 reduction 12.0138:1（目标 12:1），6/6 下游件运动，functional/stability 均 PASS，视频生成。
- 本 benchmark 仍暴露两个非功能尾项：首次 scenario metrics 仍猜 `_spin` 后靠 retry 修正；support derivative 报 topology_plan 的 support_ground 不可用。二者不改变正常 physics PASS，但必须后续修复。

### 2026-08-10：完成 finite-effort/contact A/B 调查

- 完整 servo 轨迹与 ideal equality 矛盾；
- metrics 的逐帧绝对 travel 又与端点位移矛盾；
- contact-on 时 equality residual 巨大、传动偏离；
- contact-off 时恢复 `compound/input=-0.33329`、`output/input=0.08329`；
- 结论：动态接触是触发条件，下一步查具体 contact pair，不能归因给 transmission 或 CAD。

### 2026-08-10：新增 dynamic transmission/closure acceptance gate

- validation policy v2 加入相同 servo 的 contact-on/off A/B，报告 driver travel、transmission residual、active pair、最大穿透/力/累计冲量；
- policy v3 覆盖无 scalar equality 的 closure-only 机构，并检查 no-contact commanded travel；
- policy v4 对闭环 contact-on 低于 contact-off 80% 的 candidate 拒绝；
- gate 只提供证据和拒绝，不自动添加 exclude；agent 根据反馈修 compiler。

### 2026-08-10：scenario designer 改用 manifest 真实 joint names

- `_robot_info(model, run_dir)` 优先读取 accepted manifest v3 `coordinate_map` 和 `joints`；
- initial design 与 revise 均接收真实 `motion_key_by_part` 和 `allowed_joint_keys`；
- manifest 缺失时才回退 `<link>_spin` legacy 推导；
- 四行星 benchmark 由连续 `_spin` 假名失败变为第一次 scenario 即 PASS。

### 2026-08-10：新增 world-frame joint ABI

- `out.joint(..., frame='world'|'local')`；
- world 模式同时把世界 anchor 和 axis 转为 emitted body-local frame；
- prompt v8 要求 MotionJointSpec/world port facts 使用 `frame='world'`；
- 修复 windpump 嵌套 body 下 joint anchor 重复平移导致 no-contact 闭环 stalled。

### 2026-08-10：新增 ground-exclude 与 dedicated-pin policy

- `out.exclude_ground(body,...)` 使用 collision masks 只关闭指定 body-ground contact，保留所有 body-body 碰撞；
- manifest/validation 核对 ground mask 与 provenance；
- prompt v9 禁止删除全局 ground 或排除结构 base/support；
- prompt v10 区分共享 crankshaft/web body 与 dedicated pin body：前者必须保持 rod collision，后者在 hinge 已表达且动态证据 stall 时可 exact exclude；
- windpump 最终从 input 0.2386 rad/output dead 修复为 input 12.0015 rad、22.746 mm pump stroke、4 次反转、5/5 downstream PASS。

### 2026-08-10：六项顺序 benchmark 全部通过

- 二级 reducer、钟表、三行星、四行星、曲柄滑块、windpump 全部完成正常 physics/functional PASS；
- 验证了普通 transmission tree、独立同轴、planet carrier forest、三/四 planet inventory、闭环 spanning tree+closure、长链 windpump；
- 所有 harness-domain 修复均复用现有 CAD/IR，没有因 builder/runner/evaluator fault 触发 CAD 重造。
