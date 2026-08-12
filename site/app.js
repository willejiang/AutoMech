const translations = {
  en: {
    skip: "Skip to content", navMethod: "Method", navDemo: "Demo", navArchitecture: "Architecture", navStart: "Quick start",
    award: "Microsoft Global Intern Hackathon 2026 · 2nd Place", heroEyebrow: "TASK-VERIFIED GENERATIVE CAD",
    heroTitle: "Design machines that are<br /><span>verified to work.</span>",
    heroLead: "AutoMech turns natural-language intent into mechanical CAD, then closes the loop with geometry checks, visual inspection, and physics simulation.",
    watchDemo: "Watch the demo", viewSource: "View source", factLocal: "Local execution", factVerify: "Verification layers", factLoop: "Refinement loop",
    running: "RUNNING", inputTorque: "Input torque", gearRatio: "Gear ratio", taskStatus: "Task status", stageDesign: "Design", stageBuild: "Build", stageInspect: "Inspect", stageSimulate: "Simulate",
    problemEyebrow: "THE MISSING FEEDBACK LOOP", problemTitle: "A plausible shape is not<br />a working machine.", problemLead: "Mechanical designs must assemble, move, transmit force, and survive the task they were created for.",
    geometryOnly: "Geometry-only generation", taskVerified: "Task-verified generation", looksRight: "Looks right", worksRight: "Works right",
    problemOne: "Independent coordinates can disagree", problemTwo: "Parts may collide, float, or lock", problemThree: "No evidence that the task succeeds",
    solutionOne: "Mechanical intent stays explicit", solutionTwo: "Geometry and motion are inspected", solutionThree: "Failure evidence drives the next design",
    methodEyebrow: "VERIFICATION-GUIDED REFINEMENT", methodTitle: "Generate. Test.<br /><span>Learn from failure.</span>", methodLead: "Every failed check becomes structured evidence for the next iteration—not a dead end.",
    nextDesign: "Next design", evidenceDriven: "Evidence-driven", loopDesign: "Design", loopDesignText: "Natural language → parametric machine", loopBuild: "Build", loopBuildText: "Parts, poses, joints, and meshes", loopCheck: "Check", loopCheckText: "Interference, support, and assembly", loopJudge: "Judge", loopJudgeText: "Six rendered views expose visual faults", loopPhysics: "Simulate", loopPhysicsText: "Drive inputs and measure task behavior", loopRefine: "Refine", loopRefineText: "Metrics, frames, and repair guidance",
    demoEyebrow: "PROJECT DEMO", demoTitle: "See the complete<br />research loop.", demoLead: "Explore the generated result in 3D, then watch a concise walkthrough of the motivation, architecture, simulation, and evaluation workflow.", modelTitle: "Generated mechanical watch", modelSubtitle: "Final verified assembly · Interactive GLB", mechanismMotion: "Mechanism motion", autoRotate: "Auto rotate", resetView: "Reset view", loadingModel: "Loading verified assembly", modelHint: "Drag to rotate · Scroll or pinch to zoom", modelParts: "Components", modelFaces: "Triangles", modelFormat: "Web asset", modelContent: "Mechanism", modelContentValue: "Gear train + hands", fullWalkthrough: "Full project walkthrough", videoFallback: "Your browser does not support embedded video.", videoProblem: "Problem & vision", videoArchitecture: "System architecture", videoEvidence: "Evaluation evidence",
    capEyebrow: "ONE SYSTEM, COMPLETE LOOP", capTitle: "Built for mechanical reasoning.", capOneTitle: "Language to CAD", capOneText: "Turn a task description into a complete multi-part parametric design.", capTwoTitle: "Assembly-aware", capTwoText: "Preserve parts, joints, connections, fits, and mechanical intent.", capThreeTitle: "Geometry checks", capThreeText: "Detect rigid conflicts, unsupported parts, and invalid clearances.", capFourTitle: "Visual inspection", capFourText: "Judge six rendered views and return concrete structural fixes.", capFiveTitle: "Physics evaluation", capFiveText: "Drive the mechanism and measure whether its intended task succeeds.", capSixTitle: "Automatic refinement", capSixText: "Route failure evidence back into the design until behavior improves.",
    architectureEyebrow: "SYSTEM ARCHITECTURE", architectureTitle: "One interface.<br />Three coordinated processes.", architectureLead: "The browser streams live stages from Node, while Python builds and evaluates each machine against an OpenAI-compatible model gateway.", archBrowser: "Browser", archBrowserText: "Prompt · Timeline · 3D · Video", archNodeText: "Routes · Events · Process control", archGateway: "LLM Gateway", archGatewayText: "Design · Vision · Repair", artifacts: "Artifacts", readLoop: "Read design loop", readPipeline: "Explore pipeline internals",
    evidenceEyebrow: "WHY THE JUDGE MUST WATCH", evidenceTitle: "Metrics can pass.<br /><span>The machine can still fail.</span>", evidenceLead: "In an ANYmal stand-still test, a numeric pose metric reported only 2.1° of tilt—yet the rendered frames clearly showed the robot tipping onto its side.", evidenceQuote: "“A design is only as good as the judge, and the judge has to watch the machine work.”", numericMetric: "Numeric pose metric", visualJudge: "Visual behavior judge", overturned: "FAIL · OVERTURNED",
    startEyebrow: "RUN IT LOCALLY", startTitle: "One app.<br />One command.", startLead: "AutoMech runs on your machine. The web interface launches the Python pipeline directly and streams every stage back to the browser.", modelGateway: "Model gateway", terminalEnv: "Create an isolated environment", terminalRun: "Install and launch the interface", readyAt: "Ready at",
    ctaEyebrow: "BUILD BEYOND APPEARANCE", ctaTitle: "Make the machine.<br /><span>Then make it work.</span>", openGithub: "Open on GitHub", readDocs: "Read setup docs",
    footerText: "A research prototype for task-verified generative mechanical design.", backTop: "Back to top ↑"
  },
  zh: {
    skip: "跳到主要内容", navMethod: "方法", navDemo: "演示", navArchitecture: "架构", navStart: "快速开始",
    award: "Microsoft Global Intern Hackathon 2026 · 硬件 AI 创新赛道二等奖", heroEyebrow: "面向任务验证的生成式 CAD",
    heroTitle: "设计不仅看起来合理，<br /><span>还要真正能够工作。</span>",
    heroLead: "AutoMech 将自然语言需求转化为机械 CAD，并通过几何检查、视觉检验与物理仿真形成闭环。",
    watchDemo: "观看演示", viewSource: "查看源码", factLocal: "本地运行", factVerify: "验证层级", factLoop: "迭代修正",
    running: "运行中", inputTorque: "输入扭矩", gearRatio: "传动比", taskStatus: "任务状态", stageDesign: "设计", stageBuild: "构建", stageInspect: "检查", stageSimulate: "仿真",
    problemEyebrow: "缺失的反馈闭环", problemTitle: "形状合理，<br />不代表机器能够工作。", problemLead: "机械设计必须能够装配、运动、传递作用力，并完成它所面向的真实任务。",
    geometryOnly: "仅生成几何", taskVerified: "面向任务验证", looksRight: "看起来正确", worksRight: "实际能够工作",
    problemOne: "独立预测的尺寸与坐标可能冲突", problemTwo: "零件可能干涉、悬空或锁死", problemThree: "无法证明机械任务是否成功",
    solutionOne: "显式保留机械意图与依赖关系", solutionTwo: "同时检查几何结构与运动行为", solutionThree: "失败证据直接驱动下一轮设计",
    methodEyebrow: "验证引导的迭代修正", methodTitle: "生成、测试，<br /><span>再从失败中学习。</span>", methodLead: "每一次检查失败都会转化为下一轮迭代的结构化证据，而不是流程终点。",
    nextDesign: "下一版设计", evidenceDriven: "证据驱动", loopDesign: "设计", loopDesignText: "自然语言 → 参数化机械结构", loopBuild: "构建", loopBuildText: "零件、位姿、关节与网格", loopCheck: "检查", loopCheckText: "干涉、支撑关系与装配", loopJudge: "判断", loopJudgeText: "六视角渲染发现视觉错误", loopPhysics: "仿真", loopPhysicsText: "驱动输入并测量任务行为", loopRefine: "修正", loopRefineText: "指标、画面与具体修复建议",
    demoEyebrow: "项目演示", demoTitle: "查看完整的<br />研究闭环。", demoLead: "先在网页中交互查看真实生成结果，再观看项目动机、系统架构、仿真与评估流程。", modelTitle: "生成的镂空机械表", modelSubtitle: "最终验证装配 · 交互式 GLB", mechanismMotion: "机芯运动", autoRotate: "自动旋转", resetView: "复位视角", loadingModel: "正在加载验证后的装配体", modelHint: "拖动旋转 · 滚轮或双指缩放", modelParts: "组件数量", modelFaces: "三角面", modelFormat: "网页模型", modelContent: "机械结构", modelContentValue: "齿轮传动 + 指针", fullWalkthrough: "完整项目演示", videoFallback: "当前浏览器不支持内嵌视频。", videoProblem: "问题与愿景", videoArchitecture: "系统架构", videoEvidence: "评估证据",
    capEyebrow: "一个系统，完整闭环", capTitle: "为机械推理而构建。", capOneTitle: "语言生成 CAD", capOneText: "将任务描述转化为完整的多零件参数化设计。", capTwoTitle: "装配感知", capTwoText: "保留零件、关节、连接、配合与机械意图。", capThreeTitle: "几何检查", capThreeText: "发现刚体冲突、悬空零件与错误间隙。", capFourTitle: "视觉检验", capFourText: "从六个视角判断设计并给出具体结构修复建议。", capFiveTitle: "物理评估", capFiveText: "驱动机械结构并测量预期任务是否完成。", capSixTitle: "自动迭代", capSixText: "将失败证据反馈至设计端，持续改善机械行为。",
    architectureEyebrow: "系统架构", architectureTitle: "一个界面，<br />三个协同进程。", architectureLead: "浏览器通过 Node 实时接收运行阶段，Python 负责构建与评估机械设计，并连接兼容 OpenAI 协议的模型网关。", archBrowser: "浏览器", archBrowserText: "提示词 · 时间线 · 三维模型 · 视频", archNodeText: "路由 · 事件流 · 进程控制", archGateway: "模型网关", archGatewayText: "设计 · 视觉判断 · 修正", artifacts: "生成产物", readLoop: "阅读设计闭环", readPipeline: "查看流水线细节",
    evidenceEyebrow: "为什么评估者必须观察过程", evidenceTitle: "指标可以通过，<br /><span>机器仍然可能失败。</span>", evidenceLead: "在 ANYmal 静止测试中，数值姿态指标仅报告 2.1° 倾角，但渲染画面清楚显示机器人已经侧翻。", evidenceQuote: "“一个设计的可靠性取决于它的评估者，而评估者必须真正观察机器如何工作。”", numericMetric: "数值姿态指标", visualJudge: "视觉行为评估", overturned: "失败 · 已侧翻",
    startEyebrow: "本地运行", startTitle: "一个应用，<br />一条命令。", startLead: "AutoMech 在本机运行。Web 界面直接启动 Python 流水线，并将每个阶段实时传回浏览器。", modelGateway: "模型网关", terminalEnv: "创建独立环境", terminalRun: "安装并启动界面", readyAt: "服务地址",
    ctaEyebrow: "不止生成外观", ctaTitle: "先造出机器，<br /><span>再让它真正工作。</span>", openGithub: "前往 GitHub", readDocs: "阅读安装文档",
    footerText: "面向任务验证的生成式机械设计研究原型。", backTop: "返回顶部 ↑"
  }
};

const languageToggle = document.querySelector("#language-toggle");

function applyLanguage(language) {
  const dictionary = translations[language];
  document.documentElement.lang = language === "zh" ? "zh-CN" : "en";
  document.body.dataset.language = language;

  document.querySelectorAll("[data-i18n]").forEach((element) => {
    const value = dictionary[element.dataset.i18n];
    if (value) element.textContent = value;
  });

  document.querySelectorAll("[data-i18n-html]").forEach((element) => {
    const value = dictionary[element.dataset.i18nHtml];
    if (value) element.innerHTML = value;
  });

  languageToggle.innerHTML = language === "en"
    ? '<span class="language-active">EN</span><span class="language-divider">/</span><span>中文</span>'
    : '<span>EN</span><span class="language-divider">/</span><span class="language-active">中文</span>';
  languageToggle.setAttribute("aria-label", language === "en" ? "切换为中文" : "Switch to English");
  localStorage.setItem("automech-language", language);
  const url = new URL(window.location.href);
  url.searchParams.set("lang", language);
  history.replaceState({}, "", url);
}

languageToggle.addEventListener("click", () => {
  applyLanguage(document.body.dataset.language === "zh" ? "en" : "zh");
});

const queryLanguage = new URLSearchParams(window.location.search).get("lang");
const preferredLanguage = (queryLanguage === "zh" || queryLanguage === "en" ? queryLanguage : null)
  || localStorage.getItem("automech-language")
  || (navigator.language.toLowerCase().startsWith("zh") ? "zh" : "en");
applyLanguage(preferredLanguage);

const revealObserver = new IntersectionObserver((entries) => {
  entries.forEach((entry) => {
    if (entry.isIntersecting) {
      entry.target.classList.add("visible");
      revealObserver.unobserve(entry.target);
    }
  });
}, { threshold: 0.12 });

document.querySelectorAll(".reveal").forEach((element) => revealObserver.observe(element));

const header = document.querySelector(".site-header");
window.addEventListener("scroll", () => header.classList.toggle("scrolled", window.scrollY > 20), { passive: true });
