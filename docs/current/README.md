# 当前文档入口

本目录描述 `85e13461` 保留的成功控制基线及后续开发状态。当前默认前视 RGB-D、0.15 m 升沉与末端修复另见相机文档和 RGB-D 验证报告。原飞行链仍采用 BCTRA/MINCO/Tracker；P2 MPC 仍作研究输出；P4.5已有隔离名义MINCO接口，P5 H1.6实际接管尚未验证（最新复验已加载Planner但触发输入失效，见P5复验报告）。历史报告与代码细节必须结合当前源码核实。

| 文档 | 用途 |
|---|---|
| [P5.1 接管与恢复](p51_follow_minco_switching.md) | 接收器单一准入、5s稳态、2s冷却、影子探测与真实owner |
| [P5.1 时效与lease](p51_online_timing_and_lease.md) | 旧131次回退、原请求profile、接替调度与134.6ms分解 |
| [P5.1 验收与下一步](p51_closed_loop_validation.md) | 代码/离线验证、新有界实跑入口；F2/F3尚未执行 |
| [架构](architecture.md) | 节点、消息、速度控制和真值边界 |
| [前视 RGB-D、ToF 与升沉](camera_simulation.md) | 当前 RGB-D 输入、保留的 ToF、0.15 m 场景及切换 |
| [RGB-D 末端重新验收](terminal_replan_rgbd_20261007.md) | 间歇减速归因、新鲜轨迹接替与实际验证 |
| [带噪定位闭环复验](tof_fit_validation_20261007.md) | 定位适配、实际精度、减速证据与失败边界 |
| [ToF 初始验证](tof_validation_20261007.md) | 两轮早期失败归因与理想深度成功对照 |
| [基线](baseline.md) | 四轮实验、完整配置、速度与冻结证据 |
| [环境](environment.md) | 已核实依赖、构建、启动与检查命令 |
| [分析与恢复](analysis_and_recovery.md) | 当前分析入口和历史输入恢复 |
| [MPC 范围](mpc_scope.md) | 后续实现接口和验收边界 |
| [P1 相机可见性与 MPC–MINCO 接口](mpc_minco_visibility_design.md) | 纯几何、完整姿态、目标整体视场与共同执行起点 |
| [P2 MPC 影子验证](mpc_seed_validation.md) | 保留的 MPC 基线、原始时间戳和研究准入结果 |
| [P3 架构审计](p3_architecture_audit.md) | 节点职责、公共研究入口、轻量图与测试分层 |
| [P3 贪心与可见性 MINCO 验证](greedy_minco_validation.md) | Q/T/yaw 优化、消融、实际影子结果和在线接入缺口 |
| [P3.1 快速可见性 MINCO](p31_fast_visibility_minco_validation.md) | 伴随/混合梯度、逐轨迹归因、实际影子时效与 P4 阻碍 |
| [P3.2 实时观测点与交接](p32_realtime_follow_validation.md) | 批量几何、时效分级、起点 jerk、冻结消融与实际 F2/F3 |
| [P3.2 未来交接协议](p32_follow_handover_protocol.md) | 三类时间契约、失败关闭 dry-run 与未授权执行边界 |
| [P4 滚动 FOLLOW 研究验证](p4_rolling_follow_validation.md) | 进展目标、滚动消融、验收与未获准接管原因 |
| [P4 FOLLOW 专用接口](p4_follow_interface_contract.md) | 完整系数、真实拒绝 ACK、TTL 与持有/桥接门禁 |
| [P4 原 FOLLOW 与影子比较](p4_closed_loop_comparison.md) | 本轮实际运动、FOV、负载、故障与证据限制 |
| [P4.1 跟随优化](p41_follow_tracking_optimization.md) | 动态参考加速度、时域和因果滚动消融 |
| [P4.2 安全资格](p42_follow_safety_qualification.md) | 共享时间代、新预测重验和仍关闭的接管门禁 |
| [P4.2 本轮仿真](p42_closed_loop_validation.md) | 独立重复、真实 ACK、故障与未执行项 |
| [P4.3 动力学与时域](p43_dynamic_limits_study.md) | 12组限值扫描、共同初始化与四时域消融 |
| [P4.3 高效可见性 MINCO](p43_fast_visibility_minco.md) | 互斥计时、缓存与Q/T/yaw及负结果 |
| [P4.3 在线验证](p43_online_follow_validation.md) | 本轮独立SITL、真实拒绝ACK、失败轮与验证清单 |
| [P4.3 安全资格](p43_safety_qualification.md) | 本地约束快照、PX4响应及未获准接管的原因 |
| [P4.4 候选计算审计](p44_candidate_profiling.md) | 五候选贡献、互斥计时及共享/批量/按需构造收益 |
| [P4.4 自适应候选验证](p44_adaptive_candidate_validation.md) | A–F消融、逐请求反例、共同初始化与时域质量 |
| [P4.4 在线影子验证](p44_online_follow_validation.md) | 本轮SITL、真实Tracker到达年龄及未获接管资格 |
| [P4.6 原 FOLLOW 引导 MINCO](p46_follow_guided_minco.md) | 完整FOLLOW滚动、Q/T/yaw消融、离线退化及未通过替代验收的原因 |
| [P4.7 实际执行归因](p47_execution_root_cause.md) | 同 epoch 误差分解、参考高度漂移机制及 lease 到期时间线 |
| [P4.7 闭环与导数归因](p47_minco_closed_loop_diagnosis.md) | 历史 jerk 人口、终端/Q 反事实及旧能量报告问题 |
| [P4.7 FOLLOW 可行性研究](p47_follow_guided_feasibility.md) | 独立 engine、冻结消融、预算和同窗长期质量验收 |
| [P4.7 H1.6 封版](p47_release_decision.md) | 限定离线候选、旧 H1.2 反例及轻量来源索引 |
| [P5 架构与基线](p5_architecture_and_baseline.md) | 独立研究配置、P44 切回及真实执行链 |
| [P5 实际闭环验证](p5_minco_closed_loop_validation.md) | 首次 F2、条件 F3、停止判据与实际结果 |
| [P5 修复后复验](p5_revalidation_20261012.md) | Planner已加载但TTL停止；工具修复、稳态门禁及F3未准入 |
| [P5 入口与 Git 清理](p5_code_cleanup_report.md) | 引用图、保留依据、标签和历史体积 |
| [整理交付](repository_cleanup.md) | 清点、规模、归档和实际检查结果 |
| [源码冗余清理](redundancy_cleanup.md) | 确认移出项、保留理由、恢复与回归 |

历史文件入口：[archive_index.md](../archive_index.md)。仓库操作约束：[AGENTS.md](../../AGENTS.md)。
