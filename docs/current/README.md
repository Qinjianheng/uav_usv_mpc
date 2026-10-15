# 当前开发入口

当前运行链为视觉/KF → BCTRA → MINCO → 唯一Tracker速度控制；MPC尚未作为在线控制器接管。P53 R11是保留的成功对照；2026-10-14目录精简后的新仿真与首轮预测时效回退，见[系统精简与复验](system_simplification.md)。历史成功不替代新实测。

| 文档 | 用途 |
|---|---|
| [P6 开发状态与研究边界](p6_design.md) | 五模式、IMM、校准、鲁棒几何、离线消融及实际闭环的当前进度与负结果 |
| [P6 单轮闭环入口](p6_sitl_driver.md) | 完整参数、独立资格、固定60s控制权及逐轮停止条件 |
| [系统精简、复验与恢复](system_simplification.md) | 最新直接删除旧日志、近期场景与运行依赖保留、前两轮归档及实际F2/F3边界 |
| [P5.3 全面修复与最终验收](p53_comprehensive_validation.md) | 最新R11同配置F2/F3均100%MINCO、零回退；全窗口飞行误差、阶段回归与证据 |
| [P5.3 全链路代码审查](p53_comprehensive_diagnosis.md) | F2成本低估、F3时效误标、文献对照与确认/未确认边界 |
| [架构](architecture.md) | 节点、消息、速度控制和真值边界 |
| [前视 RGB-D、ToF 与升沉](camera_simulation.md) | 当前 RGB-D 输入、保留的 ToF、0.15 m 场景及切换 |
| [RGB-D 末端重新验收](terminal_replan_rgbd_20261007.md) | 间歇减速归因、新鲜轨迹接替与实际验证 |
| [基线](baseline.md) | 四轮实验、完整配置、速度与冻结证据 |
| [环境](environment.md) | 已核实依赖、构建、启动与检查命令 |
| [分析与恢复](analysis_and_recovery.md) | 当前分析入口和历史输入恢复 |
| [MPC 范围](mpc_scope.md) | 后续实现接口和验收边界 |

按阶段查询研究设计、失败归因和旧验收，使用[研究历史索引](../history.md)；更早材料见[归档索引](../archive_index.md)。
仓库协作边界见[AGENTS.md](../../AGENTS.md)。历史报告不作为当前配置事实。
