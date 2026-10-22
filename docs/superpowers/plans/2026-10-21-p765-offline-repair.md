# P7.6.5 离线修复实施计划

> **For agentic workers:** 使用 systematic-debugging、test-driven-development、dispatching-parallel-agents 和 verification-before-completion；按独立问题分工，最后统一审查。

**Goal:** 修复可证明的问题，建立可信证据和放行条件；不强迫 E0/E1/E2 通过。

**Architecture:** 保留既有双 DKF、X MINCO / Y Yan Body-rate 架构。几何诊断、实时性记录、动力辨识和资格审计独立推进；重型回放只在离线 admission 执行。

**Tech Stack:** Python / NumPy / ROS 2 Humble / PX4 SITL 离线证据。

**Spec:** 用户粘贴任务 `/home/qin/.codex/attachments/ef696651-8888-439f-9c40-116dcc9bda82/已粘贴的文本.txt`。

## 约束与审查重点

- 基线 33a267f，当前分支内修改和本地提交；不 push、不合并。
- 不启动 Gazebo/PX4、不解锁、不运行 A1–D；原预算计数保持。
- 125 ms、7 m/s、TWR 2.85、捕获 0.25 m、Y 公式/增益、物理模型不变。
- 旧证据不改写；新输出仅在独立 P765 目录。
- 缺失深度不能推断具体拟合失败原因；合成通过不等于真实 3D。
- 离线性能不等于同条件实测提升；240 ms 模式退出不等于同接口推力响应。
- 原始时间身份、NaN / Inf 深度、mask 坐标、重置/重复帧、伪造 PASS 与跨模型资格均需拒绝测试。

## 任务与验证

- [x] 1. 几何：逐帧复算 362 拒绝；为球拟合增加明确诊断；已知球/退化数据先写失败测试后修复；实现并验证有界 lossless float32 ROI 采集。
- [x] 2. 实时性：先跑 EOF 子进程回归；从旧数据分解 inference/source age/callback/scheduling。对推理阻塞接收、记录器阻塞建立定向测试后最小修复，保留原始157丢周期。
- [x] 3. 动力与 E2：离线比较五种辨识协议，明确上界及未知；复现固定500 ms压力测试，只做预先声明的少数对照，不调论文增益。
- [x] 4. 资格：先构造伪造/陈旧身份测试，修复 E1/E2 独立复算、C→D物理/原始评价绑定、新预算门禁，B阈值标记未资格化。
- [x] 5. 集成：独立审查定向变更，冻结源码，执行一次完整可用回归、flake8、shell语法、构建与ROS prefix、旧P7入口检查。
- [x] 6. 交付：中文报告、新基线证据及SHA256、唯一推荐未解锁采集方案和预检；明确E0/E1/E2/A1结果，逐路径暂存、本地提交。

## 执行记录

初始工作区干净，HEAD和四个远程URL核对一致。App无附加终端。任务已明确自主执行上述离线工作，不重复申请设计批准。相互独立的几何、动力、资格按并行技能分工；主代理负责实时性和集成。


独立审查已关闭重要问题：原子 generation/publish、detector 自己的代码身份、ROI one-shot/rollback/epoch/receipt、末尾 SESSION_END、postflight 数据核验/失败退出码、E1 同 run/hash 绑定。各问题先重现后定向通过。

完整可用回归只执行一次：2568 passed / 2 failed(style only) / 2 skipped，47.05s。只修改两处测试格式/文档字符串，33项受影响与lint测试复测通过；运行源码与完整回归时相同，不为这两处重复全量测试。历史R11缺失输入的两个模块显式排除。最终证据保留原失败，不伪造全绿。

构建2包通过（2min1s），prefix指向新工作区；最终只读预检无仿真/串口、UDP8888空闲，检测器weight SHA匹配。新会话未获授权、未启动。

源码本地提交 `3cc13bb`；报告/证据包完成并纳入单独提交。新包 `data/baselines/20261022_p765_validation_01/`；无push/merge，无新仿真或预算。
