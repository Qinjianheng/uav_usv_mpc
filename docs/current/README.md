# 当前开发入口

P7 配置已实现 X 后 KF/BCTRA/MINCO FOLLOW、Y 后 IBVS/PNG 安全接近，由唯一 Tracker 输出速度控制，见 [P7 使用与验证](p7_progress.md)。原 baseline/IMM 研究入口保留；MPC 尚未在线接管。P53 R11 是历史成功对照，不替代当前实测。

| 文档 | 用途 |
|---|---|
| [P7 使用与实际验证](p7_progress.md) | 专用启动、X/Y 控制权、FOLLOW/制导曲线、当前边界与阶段提交 |
| [P7 半径捕获与变速验证](p7_capture.md) | 显式终端下降、0.50 m 三维捕获、暂停冻结与三种变速场景 |
| [P7 FOLLOW 修复与八字捕获](p7_follow_repair_20261018.md) | 原 4 m/s 八字捕获并暂停；变速八字/S/Z 各约 25 s 实际 MINCO 100%，根因、修复与保留边界 |
| [P7 三种变速捕获](p7_variable_capture_20261018.md) | 最终同源码八字/S/Z 均捕获并暂停；S 恢复接口修复、完整测试及 F2 连续性边界 |
| [P7 直线 FOLLOW 连续性](p7_follow_continuity_20261018.md) | 后继完成验证误拒绝修复；同配置两次 25 s 回归均实际 MINCO 100%、零回退；保留偶发输入陈旧归因边界 |
| [当前版本变速 S 全流程](p7_variable_s_full_validation_20261018.md) | 单轮捕获并暂停，Y 前 MINCO 100%；核查末端 0.13 s 失锁后的纯速度保护，保留近场视觉边界 |
| [近场制动修复与单次 S 捕获](p7_nearfield_repair_20261018.md) | 实测速度与图像趋势制动、升沉参考导数修复；单轮 14.51 s 捕获，290 帧有效，保留部分裁切边界 |
| [变速 S 视觉丢失归因](p7_s_visual_loss_diagnosis_20261018.md) | 同图像时刻矩形视锥与离散目标位姿复算；确认目标从下边缘出画，近场实测闭合速度反馈存在缺口 |
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
