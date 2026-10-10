# 当前文档入口

本目录描述 `85e13461` 保留的成功控制基线及后续开发状态。当前默认前视 RGB-D、0.15 m 升沉与末端修复另见相机文档和 RGB-D 验证报告。原飞行链仍采用 BCTRA/MINCO/Tracker；P2 MPC 和 P3 FOLLOW MINCO 仅作研究输出，尚未在线接入。历史报告与代码细节必须结合当前源码核实。

| 文档 | 用途 |
|---|---|
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
| [整理交付](repository_cleanup.md) | 清点、规模、归档和实际检查结果 |
| [源码冗余清理](redundancy_cleanup.md) | 确认移出项、保留理由、恢复与回归 |

历史文件入口：[archive_index.md](../archive_index.md)。仓库操作约束：[AGENTS.md](../../AGENTS.md)。
