# 当前文档入口

本目录描述 `85e13461` 保留的成功控制基线及后续开发状态。当前默认前视 RGB-D、0.15 m 升沉与末端修复另见相机文档和 RGB-D 验证报告。MPC 尚未实现；历史报告与代码细节必须结合当前源码核实。

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
| [整理交付](repository_cleanup.md) | 清点、规模、归档和实际检查结果 |
| [源码冗余清理](redundancy_cleanup.md) | 确认移出项、保留理由、恢复与回归 |

历史文件入口：[archive_index.md](../archive_index.md)。仓库操作约束：[AGENTS.md](../../AGENTS.md)。
