# NEXT_STEPS — 方案B 迭代收敛 / 约束优先级 / 加速 / skill 引入

本文档是**设计计划**，含实现步骤但截至撰写时代码未动（precheck 非配合碰撞检测已单独实现，见 commit 5ced916）。

按优先级：A 迭代收敛（含约束优先级）> D 加速 > C params 漂移（并入 A）> B text-to-cad。

---

## 议题 A：迭代为什么不能正向变好

### 现象（多轮 run 证据）
- **故障对 gate 隐形**：run 114909 两个 iteration precheck 都 `ok=True`，但肉眼可见两个大齿轮咬进 top_cover。原因：precheck 把整个 insert seam 用一个宽松 0.60 floor，齿轮-壳盖 8% 实体重叠被当"插入配合噪声"drop。→ **已修**（commit 5ced916：宽容度只给 seam 声明的配合对，非配合对用 0.05 严格 floor）。
- **judge 只看运动学**：两轮 judge 反馈全是"加 fixed/revolute 关节"，从没提齿轮碰撞。几何问题没有任何 gate 呈现。
- **重规划不改几何**：`re-plan: reusing 4 unchanged, rebuilding 0` —— boss 只改关节，齿轮大小/壳体大小全程冻结，两轮完全一样。
- **广播式重跑**：run 104424 iteration 0 四个 sub 全 `skip-check: REBUILD`，即使故障只在一处也整机重来，正确的 sub 和错误的 sub 一起被推翻。

### 根因
1. **故障未归因到单一 agent**。precheck violation 已带 `involved_sub_ids`/`parent_local_link`/`child_local_link`/`is_mating_pair`，但系统把 interface fault 一律上抛 boss，boss 重规划一切。
2. **重规划 = 重新生成 params**（→ 议题 C 的 params 漂移）。
3. **没有约束优先级**：系统不知道"齿轮直径由减速比定死、不可缩小 → 该让壳体让步"。judge/precheck 报了碰撞也不知道谁该改。

### 设计方向（实现步骤）——analyzer 作为"诊断官"（用户拍板方向）

核心思想：precheck 只报**症状**（谁和谁重叠、差多少），它不懂"该改谁、怎么改"。把现有 `assembly_analyzer.py` 从"修理工"（选几何修复候选并 apply/simulate）改造成**纯诊断官**：precheck 之后调用，用它已有的只读调查工具深入看（read run.log / model / precheck report / manager 源码），输出一份结构化诊断 JSON——**责任 sub + 根因 + 具体改法**，然后 orchestrator 据此**定向重跑那一个 manager**。analyzer 本身**不碰任何几何/代码**。

PM 诉求对齐：analyzer 的诊断 JSON 既是**路由依据**，又是给 PM 看的**"agent 吸取教训"的证据**——它明确写出"leg manager 只建了 1 条腿，params.leg_poses() 有 4 个位置，应在每个位置建一条"，这条 fix_instruction 比 raw violation.detail 精确得多，且可打印成学习轨迹。

**A1. analyzer 产物契约改造（`assembly_analyzer.py`）**
- 保留：`build_read_tools` / `analyze_failure` 的只读调查工具（list_artifacts / read_text / read_json / search_log / search_files-source / kb_search）。
- 保留并强化诊断字段：`culprits`（责任 sub id 列表）、`root_cause`、`evidence`、`layer`、`confidence`、`explanation`（这些现在就有）。
- **去掉动作出口**：不再 `selected_candidate_id` + `simulate_candidate` + apply-candidate + rollback。删掉 candidates 依赖（`list_repair_candidates` / `simulate_candidate` 工具，`gear_pose_axial_alignment` / `rear_mount_axial_alignment` / `housing_*` 那些确定性修复候选模拟）。
- **新增字段** `fix_instruction`（给责任 manager 的一句可执行指令，如上例）+ `blamed_sub`（路由目标 sub id）+ `route`（`manager` | `boss`）。`route=boss` 仅当真拓扑/接口契约错（seam 连错、frame 缺失、减速比矛盾）。
- analyzer 现在只在 solver-failure 分支跑（`enable_solver_failure_analyzer`）；扩展成 **precheck 失败即调用**，覆盖所有 violation 类型（part_overlap sub/interface、aabb_overlap、frame_misalign、weld_frame_coincidence 等）。

**A2. orchestrator 定向路由（`orchestrator_boss.py`）**
- precheck 失败 → 调 analyzer → 拿诊断 JSON。
- `route=manager`：把 `blamed_sub` 加入 `replan_blamed`，把 `fix_instruction` 写进 `feedback_by_sub[blamed_sub]`，只重跑那一个 sub（走已有的 per-sub 定向模板 1640-1646 + `_build_all_subs`）。**boss 不介入，params 冻结。**
- `route=boss`：才回 boss 广播（现有 1631-1639 逻辑，但现在只有真拓扑错走这里）。
- judge 的几何 FAIL 同样先过 analyzer 定向，不直接广播（1701-1706）。
- **params 冻结**：`route=manager` 时复用上一轮 params，禁止 boss 重新生成（消灭议题 C 的漂移）。

**A3. 收敛账本（PM 的关键，`orchestrator_boss.py`）**
- 加一个 per-sub 账本：每轮记录 `{sub, iteration, culprit?, fix_instruction, 结果 ok/still-failing}`。
- 用途 1（收敛判断）：某 sub 连续 N 轮同样 root_cause = 没吸取教训 → 升级（换更强 analyzer / 回 boss / 停）。
- 用途 2（PM 展示）：打印学习轨迹——"iter0: leg 少建 3 腿 → analyzer: '建全部4条' → iter1: 4 条齐, precheck 过"。这是 demo 里"迭代明显进步"的可视化。
- `feedback_by_sub` 不再每轮无脑清空；账本跨迭代保留。

**A4. 约束优先级（写进 params）** 见下方独立小节——analyzer 判 insert 配合/碰撞谁让步时读 params 的 hard/soft 标记（齿轮 hard 不缩、壳体 soft 增大）。

---

## 约束优先级：写进 params（用户拍板）

### 目标
让系统知道"齿轮不能减小（减速比硬约束）→ 增大壳体让开 → 孔位不变"这条人类推理链。

### 设计
params 的每个功能尺寸不再只是一个数值，而是带**优先级标记**的量：

- **hard（硬约束）**：由规格/物理定死，不可为避碰而改。例：齿轮 pitch 直径 = f(减速比)、中心距 = f(模数,齿数)、接口 frame 坐标。
- **soft（软约束）**：为满足硬约束和避碰可自由调整。例：壳体壁厚、内腔尺寸、盖高度、脚位置。

实现层面（params 模块约定）：
```python
# params.py 里每个功能尺寸配一个优先级注册
PRIORITY = {
    "gear_pitch_dia":   "hard",   # 减速比定死
    "center_distance":  "hard",
    "seat_frame_xyz":   "hard",   # 接口位置不可动
    "housing_wall_thk": "soft",   # 可增厚让开
    "housing_cavity":   "soft",
}
def priority(name): return PRIORITY.get(name, "soft")
```

boss prompt 增加要求：生成 params 时，对每个功能尺寸声明 hard/soft，规则——
- 任何由减速比/模数/齿数/接口位置推导的量 = hard
- 任何纯包容/结构/外形的量 = soft

### gate 如何使用
precheck 报非配合碰撞时（已实现的 detail 已含文字提示），A1 路由据 PRIORITY 决定谁让步：
- 碰撞双方查 PRIORITY：hard 的不动，soft 的那个（壳体）的 manager 被重跑，指令="增大到包住对方，接口 frame 不变"。
- 若双方都 hard → 真冲突，上抛 boss（说明拓扑/减速比本身矛盾）。

### 与现有骨牌契约的关系
这是 `plan_b_params_granularity_contract`（params=功能连接件层、全推导变量）的扩展：不仅是"值由根值+函数推导"，还要"每个值标 hard/soft"。managers 读 params 时也能据此知道自己哪些尺寸能调、哪些锁死。

---

## 议题 D：加速迭代（新增 —— 用户核心痛点）

### 时间画像（run 114909，73.6 分钟 / 3 iteration）
| 阶段 | 时长 | 大头 |
|---|---|---|
| iter0 | ~39 min | boss 规划 + 4 sub LLM 建模 + **coacd 凸分解(~16min)** + 装配 |
| iter1 | ~22 min | 同上，重来 |
| iter2 | ~13 min | 同上 |

- **coacd 凸分解是计算大头**：每次装配分解出 248 个 cvx 块，45 次 decompose 调用，且**每 iteration 对所有零件重新分解**（含未变的 reused sub）。
- **人在环是感知大头**：用户要肉眼看渲染 → 描述 → Claude 分析。这条往返比机器还慢。

### 加速方向

**D1. coacd 结果缓存（最大机器加速）**
- coacd 输入=单个零件 STL，输出=cvx 块。对同一 STL（内容 hash 不变）缓存分解结果，跨 iteration/跨 sub 复用。
- reused sub 的零件 STL 没变 → 直接取缓存，跳过分解。预计 iter1/iter2 的 coacd 时间趋近 0。
- 实现：以 STL 内容 hash 为 key，缓存 `<hash>_cvx_*.stl` 到一个 run 级或全局缓存目录。

**D2. 物理/渲染只在需要时做**
- 当前每 iteration 都跑完整 coacd + mjcf + physics。但如果这一轮只改了几何、还没到要判物理行为的阶段，可以**先只跑 precheck（几何 gate，秒级）**，precheck 过了再跑昂贵的物理。
- precheck 失败直接 re-plan，省掉这一轮的 coacd+physics。

**D3. 确定性几何自检报告（发现问题的主手段）**
- **教训**：本会话多次证明，纯看渲染图（人看或 VLM 看）对装配问题不可靠——Claude 看六视图没看出"轴没穿孔"，是量 MJCF 坐标才发现的；而且结构在壳体内部，外部视图被遮挡看不到。发现问题靠的从来是**确定性数值检查**，不是眼睛。
- 固化一个"装配自检报告"，每次装配后自动产出文字+数字，Claude 直接读它判对错，不漏内部/遮挡：
  - 每根轴：轴向 span，其上每个件是否落在 span 内（**穿孔检查**——轴向 range 不重叠 = 轴没穿过件）
  - 每对啮合齿轮：实际中心距 vs 理论值（`m*(z1+z2)/2`），是否啮合
  - 每个功能件 vs 每个结构件：实体相交 %（**碰撞检查**——即 precheck 已加的非配合碰撞，扩展到全部对）
  - 每个 sub 的 params_hash：是否一致（**漂移检查**）
  - 每个件的实现回转轴 vs `params.<frame>_axis()`：朝向对不对（含符号，不用 abs）
- precheck 就是这个思路的雏形，本会话加的非配合碰撞检测（commit 5ced916）是往这个方向扩。自检报告 = 把这些散落的确定性检查汇总成一份可读报告。

**D3b. 选择性分层渲染（辅助，补数值盲区）**
- 渲染留作**辅助**（人/Claude 快速扫整体形态），不作发现问题主手段。
- **分层出图解决遮挡**：除完整六视图外，再出**排除结构件（壳体/壁/盖/座/脚）、只留功能件（轴/齿轮/轴承）**的六视图，让内部关系裸露一目了然。
- 实现：本会话的 trimesh+matplotlib 出图代码里按 link 名分组（`housing/wall/cover/seat/foot/base` = 结构，其余 = 功能），渲不同子集。很轻。可进一步按 sub 分组、任意组合。
- 数值自检（D3）给真相，分层渲染（D3b）给直观，两者互补。


**D4. 快速验证模式（--fast-geo）**
- 给 run 一个开关：跳过 physics/judge，只做 boss→manager→assemble→precheck→**自动六视图**，产出几何结果给人/Claude 看。用于快速几何迭代，省掉每轮 physics 的大头。
- 全流程（含 physics）只在几何稳定后跑。

### 建议优先级
- **D3（确定性几何自检报告）**：发现问题的主手段，比看图可靠（不漏内部/遮挡），Claude 直接读数值判对错，消除用户人肉看图往返。**先做。**
- **D3b（选择性分层渲染）**：辅助直观，解决壳体遮挡，随 D3 一起做（很轻）。
- **D1（coacd 缓存）**：机器提速最大，iter1+ 几乎免 coacd。
- **D4（fast-geo 模式）**：配合 D3，几何迭代不跑 physics。
- **D2**：precheck 前置早停。

---

## 议题 C：params 漂移（已查证，并入 A）

md5 查证证实真实但只在 re-plan 出现：boss 重规划重新生成整套 params，不同步分发（housing 旧版 `I_TOTAL` vs 轴新版 `RATIO_TOTAL`）。

**A1 的"params 冻结"直接消灭它。** 若不做 A1，独立 backstop：
- C1. params 升为 run 级单一文件 `<session>/params.py`，所有 sub import 同一份。
- C2. 每 sub 记 params_hash，assembler 组装前校验一致，不一致拒绝组装。

---

## 议题 B：引入 text-to-cad skill（几何/装配层）

### 调研结论（已实锤，读了 skills/cad/requirements.txt + SKILL.md）
- **底层 = build123d**（不是 CadQuery），封装在自研 **`cadpy`** 包里。requirements: `--editable ./scripts/packages/cadpy` + `playwright`。
- 关键能力：
  - **`cadpy.assembly.AssemblyHelper`** —— 装配辅助，带 **source-level build123d joints、命名配合基准（named mating datums）、native labels**。这正是本 harness 手搓的东西（seat↔bearing 配合、place_axial 基准、bearing_od 配合链）的工具化版本。
  - **STEP-first**（B-rep 主输出，STL/3MF/GLB 二次导出），比你现在 mesh 路线几何更干净。
  - **`scripts/inspect`** —— measure / align / frame / diff，即你手写的坐标探针/实体相交检查的工具化版本。
  - **`scripts/snapshot`** —— PNG/GIF 视觉复查（同样有看不到内部的遮挡问题，不是银弹）。
- 它**没有**：物理闭环、失败驱动重规划、约束优先级、多 agent 分布式一致性。这些是 harness 核心。

### 它能解决什么、不能解决什么（重要，别乐观）
- **能消灭**：单个零件/单次装配的几何表达错误——你这几天 debug 的轴向基准、轴承配合直径、叉积朝向，它有工具化配合基准，大概率从源头不犯。
- **不能消灭**：
  - **约束优先级**（齿轮不能缩、该让壳体）——领域知识，通用 CAD skill 没有。
  - **多 manager 分布式装配的一致性**（params 单一真值、跨 sub frame 对齐）——benchmark 的行星齿轮是**一个 agent 装整机**，没有你多 manager 并行各建各的再拼这种分布式问题。
  - **失败驱动的迭代重规划**（几何错→定位到哪个 manager→定向重建）。
- 所以"用了 skill 就什么装配问题都解决、直接进物理" = **乐观**。它把"单点几何表达"做对，但"分布式一致性 + 约束驱动迭代"仍是你的。

### 用户设想的方案（值得认真试）
保留 boss 分工 + manager 并行，把 **worker/装配层换成 skill 的 build123d + AssemblyHelper**。合理，但有个关键前提未验证：
- `AssemblyHelper` 是为**一个 agent 装整机**设计的。你要每个 manager 用它装自己那个 sub，再由你的 assembler 拼 subs。
- **未知**：skill 的命名配合基准能否**跨 sub 暴露**给你的 assembler 对齐？还是它假设所有配合在一个 build123d 上下文里？这决定"完美契合"还是"又要手搓跨 sub 胶水"。

### Spike 设计（提到 B 的最前，先验证再决定）
半天工作量，验证跨 sub 配合基准是否可用：
1. 用 skill 的 CAD 生成**一个轴 sub**（轴+齿轮+轴承，带命名配合基准）→ STEP。
2. 生成一个 **housing sub**（带座孔配合基准）→ STEP。
3. **验证**：两个独立生成的 sub，其配合基准能否被 assembler 读出、对齐、拼成整机——不重叠、轴穿孔、齿轮啮合。
4. 判据：
   - 若跨 sub 配合基准可读可对齐 → 用户判断成立，worker+装配层换 skill，省掉大量手搓，直逼物理仿真。
   - 若配合基准跨不出单上下文 → 只**借鉴概念**（命名配合基准、inspect 式确定性检查）用在现有 CadQuery 管线，不整体迁移。

### 迁移成本提醒
build123d ≠ CadQuery，API 不同。现有 place_axial 契约、make_gear、所有 manager prompt 都是 CadQuery 写的。整体换 = 重写 worker 层 + 全部 prompt。不是小工程——**先 spike，再定全换还是借鉴**。


---

## 建议执行顺序
1. **D3 确定性自检报告 + D3b 分层渲染** —— 立刻解决用户看图往返，比看图可靠（最痛、最轻）
2. **B spike（跨 sub 配合基准验证）** —— 半天，决定 worker 层是否换 skill；早验证避免后续白搓/白迁
3. **A3 约束优先级写进 params** + **A1 定向路由 + params 冻结** —— 迭代真正收敛，顺带消灭 C
4. **D1 coacd 缓存** —— 机器提速
5. **A2 反馈精准化 / A4 记账** —— 收敛质量与度量

> 注：B spike 提前，因为若结论是"worker 层换 build123d"，A3/A1 的实现就该在新 worker 上做，避免在旧 CadQuery 管线上白做。若 spike 结论是"只借鉴概念"，则继续在现有管线推进 A。

## 待用户决策
1. **D3**：先做数值自检报告，还是自检 + 分层渲染一起做？（推荐一起，渲染很轻）
2. **B spike**：现在做（半天验证跨 sub 配合基准），还是先把 A 迭代收敛做完再评估几何层？
3. **A3 约束优先级**：params 里显式 `PRIORITY` 字典，还是让 boss 在每个尺寸的注释/命名里标 hard/soft？
4. **A1 定向路由**：先只做"sub 内部故障不上抛 boss"，还是一步到位含约束优先级让步路由？
