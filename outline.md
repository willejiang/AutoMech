# AutoMech GitHub Pages Outline

## 🎯 页面目标

- 建立一个面向研究者、工程师和项目评审者的公开项目主页。
- 用一页讲清楚 AutoMech 解决的问题、核心方法、闭环流程和实际能力。
- 优先展示可验证的机械行为，而不是只展示外观合理的 CAD 模型。
- 引导访客观看演示、阅读技术文档、访问 GitHub 仓库并在本地运行项目。
- GitHub Pages 仅部署静态展示站；完整 AutoMech 应用仍需本地 Node、Python、CAD/物理依赖和 LLM 网关。

## 👥 目标受众

| 受众 | 最关心的信息 | 页面对应内容 |
| --- | --- | --- |
| 研究人员 | 方法创新、验证闭环、技术结构 | Problem、Method、Architecture、Evaluation |
| CAD/机械工程师 | 能生成什么、是否具有关节和物理行为 | Capabilities、Demo、Examples |
| 开发者 | 如何运行、依赖什么、代码在哪里 | Quick Start、Repository、Documentation |
| 评审者与潜在合作者 | 项目价值、完成度、成果 | Hero、Demo、Highlights、Recognition |

## 🧭 页面信息结构

```mermaid
flowchart LR
    accTitle: AutoMech Landing Page Structure
    accDescr: The page introduces the project, demonstrates the verification loop, presents evidence, and directs visitors to the repository and local setup instructions.

    hero["Hero"] --> problem["Problem"]
    problem --> method["Verification Loop"]
    method --> demo["Interactive Demo"]
    demo --> capabilities["Capabilities"]
    capabilities --> architecture["Architecture"]
    architecture --> evidence["Results and Cases"]
    evidence --> quick_start["Quick Start"]
    quick_start --> footer["Repository and Links"]

    classDef primary fill:#dbeafe,stroke:#2563eb,stroke-width:2px,color:#1e3a5f
    classDef evidence fill:#dcfce7,stroke:#16a34a,stroke-width:2px,color:#14532d
    classDef action fill:#fef3c7,stroke:#d97706,stroke-width:2px,color:#78350f

    class hero,problem,method,capabilities,architecture primary
    class demo,evidence evidence
    class quick_start,footer action
```

## 🏠 1. Hero

**目的：** 在首屏内说明 AutoMech 是什么，以及它与普通文本到 CAD 系统的区别。

- 项目名：`AutoMech`。
- 主标题建议：`Design Machines That Are Verified to Work`。
- 一句话说明：从自然语言生成机械 CAD，并通过视觉检查与物理仿真持续修正。
- 成果标识：Microsoft Global Intern Hackathon 2026 Hardware AI Innovation Track 二等奖。
- 主按钮：`Watch Demo`。
- 次按钮：`View on GitHub`、`Read the Method`。
- 首屏视觉：AutoMech Logo、机械模型渲染或演示视频静态封面。

## ⚠️ 2. Problem

**目的：** 解释“看起来像机器”不等于“能够工作”。

- 传统生成式 CAD 容易只优化几何外观。
- 机械设计还需要满足装配、支撑、间隙、关节和传动关系。
- 单一数值指标可能漏掉倾倒、卡死或错误运动等真实失败。
- 本节采用一个简短对比：`Geometry-only Generation` 与 `Task-verified Generation`。

## 🔁 3. Verification-Guided Refinement Loop

**目的：** 作为页面核心方法图，展示 AutoMech 如何从提示词走到可验证结果。

- 输入：自然语言机械任务，可选参考图像。
- 设计：LLM 生成完整参数化机械设计。
- 构建：生成零件、装配信息和可执行机械表示。
- 几何检查：检测干涉、悬空、错误配合和装配冲突。
- 视觉检查：从多视角判断结构与任务是否匹配。
- 物理评估：在 MuJoCo/PyBullet 中驱动输入并测量输出行为。
- 修正：将失败指标、图像和具体修复建议反馈至下一轮。
- 终止：通过任务测试或达到预设迭代条件。

## 🎬 4. Demo

**目的：** 让访客无需安装即可看到系统实际运行过程。

- 使用 `assets/AutoMech1.mp4` 作为主要演示素材。
- 视频默认显示封面，不自动播放，用户点击后加载。
- 视频下方用 3–4 个步骤解释画面：Prompt、Build、Inspect、Simulate/Refine。
- 后续补充短 GIF/WebM 片段，分别展示模型生成、三维查看和物理测试。
- 当前视频约 45 MB，正式实现前评估压缩版本，避免首屏直接加载完整文件。

## 🧩 5. Core Capabilities

使用 4–6 张能力卡片，每张卡片只表达一个核心能力：

1. `Natural Language to Mechanical CAD`：从任务描述生成多零件设计。
2. `Assembly-Aware Construction`：显式保留零件、关节、连接和机械意图。
3. `Geometric Self-Checks`：检查干涉、支撑、配合和碰撞问题。
4. `Visual Verification`：利用多视角渲染发现外观与结构错误。
5. `Physics-Based Task Evaluation`：验证输入、输出与预期机械行为。
6. `Automatic Refinement`：基于失败证据重新设计，而不是一次性生成。

## 🏗️ 6. System Architecture

**目的：** 用一张简洁结构图说明浏览器、Node 服务、Python 流水线、LLM 网关和输出文件之间的关系。

- Browser：提示输入、实时进度、3D 模型和物理视频。
- Node/TanStack Start：API、SSE 流和 Python 子进程管理。
- Maker：参数化 CAD、零件网格、装配和几何检查。
- Evaluator：场景选择、物理仿真、指标和视频输出。
- LLM Gateway：设计、视觉判断和修正推理。
- Artifacts：GLB、MJCF/URDF、指标、事件日志和 MP4。
- 页面仅展示精简架构；详细流程链接至 `DESIGN_LOOP.md` 和 `maker2/PIPELINE.md`。

## 📊 7. Results and Case Studies

**目的：** 用真实证据回答“AutoMech 是否比只生成几何更可靠”。

- 主案例：数值姿态指标误判通过，但视觉观察识别出机器人倾倒。
- 机械传动案例：展示齿轮输入与输出方向、速比或持续传动结果。
- 每个案例采用统一结构：`Task → Initial Failure → Feedback → Refined Result`。
- 每个案例预留一张模型图、一段短视频和一组关键指标。
- 若正式实验结果尚未整理，先使用明确的占位符，不虚构数值。

## 🖥️ 8. Product Interface

**目的：** 展示当前可运行系统，而不是让 GitHub Pages 模拟后端功能。

- Launcher：输入任务与运行选项。
- Pipeline Timeline：实时展示设计、构建、检查和仿真阶段。
- 3D Workbench：旋转查看生成模型和零件结构。
- Physics Panel：播放物理测试结果并查看评估信息。
- Past Runs：回放历史事件和生成产物。
- 使用真实界面截图；可做轻量图片轮播，但不在 Pages 中连接 Python 后端。

## 🚀 9. Quick Start

**目的：** 给开发者一条最短的本地运行路径。

- 前置要求：Node.js、npm、Python 3.10+、`uv` 和 OpenAI-compatible LLM gateway。
- 推荐创建独立 `uv` 环境。
- 安装前端和 Python 依赖。
- 设置 `PYTHON_BIN` 指向项目虚拟环境。
- 启动 `npm run dev` 并访问本地端口。
- 页面仅展示最小命令，完整参数与故障排查链接回仓库 `README.md`。

## 📚 10. Documentation and Repository

- `View Source`：GitHub 仓库主页。
- `Pipeline Internals`：`maker2/PIPELINE.md`。
- `Design Loop`：`DESIGN_LOOP.md`。
- `Physics Findings`：`docs/CONTACT_PHYSICS_FINDINGS.md`。
- `Run Locally`：`README.md#setup`。
- 可选链接：Issue、Discussion、论文或技术报告。

## 🧾 11. Footer

- AutoMech 项目名与简短定位。
- GitHub 仓库链接。
- License 链接；在实现前确认仓库正式许可证。
- Team/author 信息；在公开前确认展示名称和联系方式。
- 项目状态说明：Research prototype。

## 🎨 视觉方向

- 风格：深色工程界面，强调机械结构、仿真轨迹和验证状态。
- 主色：深蓝/石墨色；成功使用绿色，失败与反馈使用橙色或红色。
- 字体：现代无衬线字体；标题简洁，正文保持较高可读性。
- 背景元素：低对比度网格、工程图线条或机械零件轮廓。
- 动效：仅用于闭环箭头、阶段进入和指标变化，避免无意义的装饰动画。
- 响应式：桌面优先，同时保证移动端视频、架构图和能力卡片可读。

## 🗂️ 建议目录结构

```text
site/
├── index.html
├── src/
│   ├── main.tsx
│   ├── styles.css
│   ├── components/
│   └── data/
├── public/
│   ├── images/
│   └── videos/
└── vite.config.ts

.github/
└── workflows/
    └── deploy-pages.yml
```

- 推荐将展示站放在独立 `site/` 目录，避免与当前需要服务器能力的主应用混合。
- 使用无运行时依赖的 HTML、CSS 与 JavaScript，避免与主应用的服务端能力耦合。
- GitHub Actions 直接将 `site/` 作为静态产物部署至 GitHub Pages。
- 配置仓库子路径 base URL，确保静态资源在 `/AutoMech/` 下正常加载。

## 🧱 实施阶段

| 阶段 | 工作内容 | 输出 |
| --- | --- | --- |
| 1 | 确认文案、页面结构和公开信息 | 本大纲定稿 |
| 2 | 整理截图、视频封面和案例素材 | 页面素材清单 |
| 3 | 实现静态响应式页面 | `site/` 源码 |
| 4 | 压缩视频与图片、检查移动端 | 优化后的静态资源 |
| 5 | 添加 GitHub Actions Pages 部署 | 自动部署工作流 |
| 6 | 检查链接、性能与公开内容 | 可发布版本 |

## ❓ 实现前待确认

- 页面提供中英文切换，并记住访客上一次使用的语言。
- 是否公开团队成员、联系方式和 Hackathon 相关标识。
- 是否已有论文、技术报告或实验表格可以链接。
- 首批公开展示哪些机械案例及其真实评估结果。
- 页面使用压缩后的 WebM/MP4，仓库中的 45 MB 原始视频继续作为高质量源文件保留。
- GitHub Pages 使用默认地址，还是绑定自定义域名。
