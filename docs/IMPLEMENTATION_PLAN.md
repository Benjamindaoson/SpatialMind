# SpatialMind — 下一阶段执行计划（岗位导向）

> 制定日期：2026-10-08  
> 目标职位：物理 AI Agent 算法工程师（J11426）  
> 状态：**计划，不代表下列功能已实现**  
> 代码仓库：https://github.com/Benjamindaoson/SpatialMind  
> 运行基线：主分支 GitHub Actions [37757260097](https://github.com/Benjamindaoson/SpatialMind/actions/runs/37757260097) 已通过 22 项测试，并运行 3 seeds × 6 scenes × 3 policies = 54 个网格任务。

## 1. 北极星与范围

**核心演示：** 一个移动机器人在室内多房间环境中接到自然语言指令，利用 RGB-D 感知与真实导航反馈建立空间记忆，自主搜索目标；在物体移动、视觉遮挡、路径阻断、任务改口后，能够更新记忆、重规划并以传感器证据完成验证。

**必须证明的 4 项能力：**
1. Physical grounding：像素/深度/里程计/地图坐标与语言语义一致。
2. Memory consistency：时空记忆带时间、来源、置信度、不确定性与负证据，仅在覆盖条件满足时纠错。
3. Long-horizon execution：异步工具执行、导航反馈、取消、失败分类、有限重试、checkpoint/resume。
4. Rigorous evaluation：与合理 baseline 配对对比，有真实运行日志、版本、随机种子和成本/延迟指标。

**暂不做：** 训练大规模 VLA、重新实现 SLAM/控制器、机械臂抓取、重型 3D 神经重建、复杂多机器人协同、凭空声称真机落地。这些不属于本岗位最高优先级。

## 2. 真实现状与关键缺口

已存在：`GridWorld / SimRobot / SpatialMemory / ActiveSearchPlanner / AgentRuntime / ContextEngine / EventStore`、小规模消融、CLI、浏览器工作台、ROS2/Nav2 transport adapter、GitHub CI。

尚缺：
- 无 Gazebo/Nav2 实际联调、无可一键启动的 ROS2 launch/world/config。
- 传感器模拟输出完美对象 ID、标签、坐标；缺 RGB-D 检测、TF2 投影、关联与过期处理。
- `AgentRuntime`、`ActiveSearchPlanner` 直接依赖 `GridWorld`，未抽象真实 map/perception 服务。
- `Nav2RobotAdapter` 是同步阻塞，缺完整 feedback、取消确认、执行状态与稳健 TF2。
- Memory 单点 + 简单乘法更新不充分，无法处理重复物体、定位漂移、误检和先前观测老化。
- `check_same_thread=False` 仅取消线程检查；HTTP 多并发任务仍缺明确串行策略/事务边界。
- 当前 54 次测试来自 2D toy-grid；三组策略全部 100% 成功，Uniform Search 总路径成本有时更低，不能推广当前优势到物理世界。

## 3. 里程碑（按依赖推进；不按写了多少代码验收）

### G0 — 真实性与架构硬化（P0，最先完成）

**任务**
- 增加机器人适配协议、地图查询接口（`MapProvider`）、语义环境查询（`SemanticMap`）、感知/目标证据协议（`ObservationProvider`）；核心 Agent 脱离 `GridWorld` 类型。
- 统一地图坐标系：浮点米制位置、方向、协方差/定位质量、TF frame、时间戳；不要把网格 Point 当真实 ROS Pose。
- 将 runtime 重构为可暂停的任务状态机，支持异步 `navigate(goal)`、`observe()`、cancel/timeout/feedback，防止 ROS 回调线程与 Agent 同步执行互锁。
- 让同一机器人只运行一个受控主动任务；SQLite 写入队列或带锁数据库仓储，避免 concurrent SQLite 使用风险。
- 锁定 v0.1 报告为 `grid-v0.1` 的历史对照，不再混同 Gazebo 指标。

**验收（Gate）**
- 原有自动化测试保留；新增并发/取消/超时/坐标/状态迁移测试。
- Grid backend 在新接口之下仍能跑原基准；无显式类型依赖；所有 action 有 task/attempt/goal/observation ID。
- `Stop` 或取消确实中断未完成动作，不只返回文字“停止”。

### G1 — ROS2 + Gazebo + Nav2 真正运动闭环（P0）

**技术路线**
- Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic + ros_gz_bridge + Nav2；从官方 Nav2 TurtleBot3 simulation quickstart 起步，确认环境后再选机器人型号。
- 优先复用官方导航模型/传感器，不从头写 SLAM 或底层控制器。
- 设计一个多房间室内场景：通道、门洞、可移动物体、可变障碍；可固定随机种子还原。
- 建立 bringup、map/localization、Nav2 action、controller/odometry/TF 与日志采集的 ROS2 package/launch。
- 以 headless Gazebo 默认运行；RViz/Gazebo GUI 用于本地开发与录制。

**验收**
- Agent 从自然语言目标发起 `NavigateToPose`，真实 Gazebo 机器人移动，Nav2 返回结果；录制 rosbag2 与关键事件。
- 至少 20 个不同起终点场景能自动执行并输出成功/超时/取消/不可达分类；基本场景目标成功率 >=90%（**目标，不是当前成绩**）。
- 人为堵塞路线时可展示 Nav2 局部恢复与 Agent 任务级重规划分工；不可安全绕行时正确 abstain/上报。
- 一条命令启动最小系统，另一条命令跑 smoke test。

### G2 — 真实 RGB-D → 对象空间记忆（P0）

**实现任务**
- 订阅 RGB 图、深度、CameraInfo、TF2；将目标检测框/掩膜中的有效深度反投影成相机坐标，再转换至 `map` frame；时间同步与 TF 可用性校验必须显式。
- 先实现三类目标（工具箱、急救箱、常见障碍）的小范围演示；检测器优先 CPU/小 GPU 可用，模型作为可替换组件。
- 真实或仿真渲染出的视觉信息必须经过检测器；不能从 Gazebo 的对象真值接口直接填入 Agent Memory。
- 为对象建立 track/data association、位置协方差、观测质量和多视角融合；将 missing 与 occluded/unknown 区分。
- 负证据更新需满足有效视场、深度覆盖、非遮挡、探测概率门槛；移动目标需要旧位置失效、新位置再关联。
- 地图和物体知识分层管理：Occupancy/Navigation map、Semantic room map、Object memory、Episodic observation。

**验收**
- 录像/rosbag2 重放可复现 object map 的版本更新。
- 同一类两个物体不会仅因同标签被错误合并；遮挡时不能误删；目标移动后能纠正位置。
- 用独立标注集测检测 Precision/Recall、位置误差（m）、identity switch、negative-update error，不能用仿真真值直接作为运行时观测。

### G3 — Physical Agent 策略与 Context（P1）

**实现任务**
- Hierarchical Task Planner：意图澄清 -> 语义目标 -> 子任务 -> Nav2 Goal -> 观测与验证 -> 重规划。
- Active Perception：根据对象存在概率、信息增益、视场可见性、可达性、路径代价、危险区域进行 viewpoint scoring；至少与 Uniform Search、Greedy Nearest 做对照。
- Context Manager：任务状态/约束是 authoritative；长期环境记忆动态检索，历史摘要不是事实来源；检查地图版本/数据时效。
- HRI：执行中用户改口、主动澄清、进度反馈、超时解释；分开“对话理解”和“运动安全”。
- Recovery：障碍动态出现、导航超时、定位丢失、记忆冲突、模型/感知不可用分别处理；限制重试与 tool budget；可取消和恢复。
- 采用可替换 LLM/VLM，不依赖某个模型品牌；预设低成本规则/本地/在线策略模式。

**验收**
- 在一个完整的演示任务中顺序覆盖：指令 -> 旧目标记忆 -> 物体移位 -> 观测证伪 -> 主动搜索 -> 路径阻挡 -> 失败恢复 -> 目标验证 -> 结果证据；任务中途改口不引发失控运动。
- 每一步提供 causal trace：输入/当前状态/目标/工具动作/动作反馈/观测/记忆版本/代价/结果。

### G4 — 强 Baseline、反事实评测与数据证据（P1，MVP 必须做）

**实验原则**
- 不能让任何决策模块读仿真 ground-truth 物体坐标；同一 perception/nav/config 对比所有策略。
- 实验因子：物体移动、遮挡、可达性/临时障碍、定位噪声、同类多目标、用户指令歧义与任务改口、长任务状态退化。
- 干预组和对照组使用相同 world layout、initial pose、seed、目标、传感器噪声与时间预算（paired trials）。
- Baselines: (B0) LLM/规则 + Nav2 + 当前观测；(B1) Static Memory + Nav2；(B2) Dynamic Memory + Uniform/Nearest Search；(Full) 时空记忆 + 主动搜索 + 恢复。要分别消融记忆更新、信息增益和恢复模块。
- 指标：end-to-end success with external evidence、success given navigation reachable、distance (m)、SPL/路径效率（如适用）、replan count、false memory invalidation、false positive completion、latency P50/P95、total token/cost、peak RSS/VRAM、safety stop/collision、checkpoint recovery rate。
- 首轮 30 个 smoke/e2e cases；正式门槛至少 200 个**不同、独立可复现任务实例**，横跨多地图/物体配置与随机种子；严禁把同一个固定地图换多个 seed 就包装成不同场景。
- 统计 paired delta + bootstrap 95% CI，按干预类型分组；保留失败案例。若 Uniform 或 Static Memory 更强，报告它并针对算法改进，不改数据。
- 记录 `git_sha`、`world_id`、`seed`、`task_id`、`model_id`、`map_version`、`observation_id`、`plan_id`、`nav_goal_id`、`checkpoint_id` 与配置 hash。所有指标由执行日志计算，禁止手写数字。

**验收**
- 可复现实验命令；生成 `results.csv/parquet`、`metrics.json`、对照表、CI 图、典型轨迹/视频及负面案例。
- 结果写简历前要可对照原始日志，注明环境、样本、比较对象、是否显著；不使用“100% 物理成功率”等误导表述。

### G5 — MLOps、性能优化、旗舰交付（P1）

**任务**
- Headless-first，可单独运行 Gazebo/ROS2+Nav2 而无需每次启动模型；模型按事件触发，避免逐帧 LLM。
- 比较小模型/云端模型/缓存策略/量化（如硬件支持），记录成功率、token、P95 决策时延、峰值内存/显存、模型加载时延。
- 对相机感知采用帧率预算/裁剪/批处理或模型优化；优化结果以指标为准，不能只写“做了推理加速”。
- 统一 ROS bag2 录制、压缩/保留策略；采用 300GB 数据盘的存储配额：环境与容器 <=50GB、rosbag2/raw <=140GB、结果与压缩轨迹 <=60GB、预留 >=50GB（**初始预算，可按实际调整**）。
- CI 拆快慢两层：PR 单元/接口/网格 mock；周期性/手动 headless Gazebo 集成与小样本评测；正式完整评测独立留存。
- README：一键运行、截图/视频、架构、复现实验、已知限制、资源账本；提供 3–5 分钟 demo 和 3 个典型失败恢复案例。

**验收**
- 新环境可依 README 复现 MVP；不提供密钥也能跑仿真与评测。
- 展示所有候选算法的实际 P50/P95/cost/performance 数据；推理优化不得显著恶化关键安全/质量指标。
- 所有“completed”在项目和简历中的表述均有 artifact、日志或视频对应。

## 4. 工作顺序与停止线

优先链：**G0 → G1 → G2 → G3 → G4 → G5**。这不是“功能越多越好”的清单；每个 Gate 必须有可执行测试/录像/报告后才进入下一个。

**最早可投递 MVP：** G0+G1+G2（真实 Gazebo 导航 + RGB-D 空间对象闭环）达到验收，再从 G4 先做少量干预对照。HRI 与高级主动搜索可以在此后深化。

**完整旗舰版：** G0–G5，数据足够支持部分量化简历成绩。若时间/硬件受限，停止在可验证版本，绝不把规划中的模块标记为已完成。

## 5. 开发与提交规范（供 Codex/Cursor/GitHub 使用）

每个小交付必须具备：代码 + 类型/接口契约 + 至少 1 个失败路径测试 + 可运行命令 + 指标埋点 + README/变更记录。先保证集成证据，再做 UI 美化。

推荐目录扩展：
```text
spatialmind_ros/             # ROS2 package(s): launch, config, TF/bridge
simulation/worlds/           # Gazebo world and scene variants
simulation/models/           # Sensor-equipped mobile robot and objects
src/spatialmind/robot/       # Robot interface, AsyncNav2Adapter
src/spatialmind/perception/  # RGB-D projection, detection, tracking, visibility
src/spatialmind/maps/        # Navigation/semantic map interface
src/spatialmind/agent/       # Task state machine, context, planning, recovery
src/spatialmind/evaluation/  # Paired seeds, ablation, evaluator, report
configs/                     # Versioned agent/ROS2/model configs
scripts/                     # One-command bringup, smoke, benchmark, replay
docs/                        # Architecture, experiments, known limitations
```

**注意：** 不要求强行拆迁已有模块，允许按依赖渐进式重构；避免首次提交就全面重写仓库。

## 6. 明确的“不做”

- 不重造 Nav2、SLAM、视觉基础模型。
- 不把 GridWorld 理想物体 ID 当成真实视觉定位能力。
- 不仅靠提示词去解决定位、物理碰撞与安全问题。
- 不为了增加一个技术关键词而引入新数据库/新框架。
- 不虚构实验数字、不混用网格长度和米制路径。
- 不在未经安全评估的真机上直接运行自主运动算法。

## 7. 上游技术依据

- Nav2 Jazzy 官方 Gazebo setup: https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/
- Nav2 Jazzy quickstart: https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/
- Gazebo Harmonic ROS2 integration: https://gazebosim.org/docs/harmonic/ros2_integration/
- ros_gz Jazzy/Harmonic compatibility: https://github.com/gazebosim/ros_gz/

## 8. 下一项应开工的任务

**`G0-01`：真实执行接口重构（只改最关键依赖链）**

1. 为真实米制 pose/frame/timestamp 定义 typed contract，不破坏 GridWorld adapter。
2. `AgentRuntime` 与 `ActiveSearchPlanner` 移除对 `GridWorld` 的具体类型依赖，改为接口注入。
3. 将 `NavigationResult` 分解为状态/原因/实测成本；为 async + cancellation 定义协议。
4. 增加接口替身测试、运行轨迹字段与旧任务基准回归检查。
5. 所有核心与 API CI 保持通过，数据指标口径不变。

**进入下一阶段的唯一条件：** 现有软件示例无功能回退，且 ROS2 adapter 能被 Agent Runtime 以标准接口替换，而不是再次复制一套 agent。