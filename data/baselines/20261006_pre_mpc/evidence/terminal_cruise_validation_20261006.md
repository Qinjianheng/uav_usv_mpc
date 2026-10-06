# 末端速度修复与两次重复实验（2026-10-06）

用户要求在已成功的当前版本上再做两次重复实验，两次均成功后收尾。结果为 **2/2 SUCCESS**；加上修复后的首次实验，该版本连续三次成功，均在第一次末端进近中达到原有捕获判据，捕获前恢复次数为零。三次之间没有修改控制代码或配置。基线为 master/6818e5a，修订保留在工作区，未自动提交或推送。

## 修改与原因

最近一次用户失败记录为 `modular_intercept_20261006_093918_367847_mission_1`：TIMEOUT，评价最小三维距离 0.557574 m，首次末端进近进入恢复。该次失败未触发成功暂停，因此不能归因于捕获后的共同停止功能。

1. **修正反馈比较的时间。** 原 tracker 把旧导航测量的 stamp 改成发布时刻，再将旧位置与发布时刻的 MINCO 位置比较。飞机以 6 m/s 飞行时，40 ms 的测量年龄会形成约 0.24 m 的虚假跟踪误差。现在保留物理测量 epoch，将位置反馈与该 epoch 的 MINCO 参考比较；前馈、限速、指令加速度及轨迹到期判断分别使用当前发布 epoch。测试覆盖 20/40/80 ms 延迟、加速轨迹以及旧测量不能延长过期轨迹的执行。
2. **增加末端新鲜目标方向修正，并保持水平速度。** 当有效轨迹剩余接触时间在 0～0.7 s、预测水平距离不超过 2.5 m，而且 KF 与 BCTRA 输入仍满足原有 freshness 时，使用 KF 目标速度估计短时提前量，再从 BCTRA 取未来目标点，更新水平追击方向。水平速度逐步升至 tracker 上限 6.5 m/s，转向优先分配水平加速度预算，以保持速度模长的方式转向，避免速度向量插值带来的制动。这里只修正水平执行指令，竖直 MINCO、海面安全保护及过期拒绝保留。

MINCO 的数学终点速度仍为预测目标速度加沿接近方向的闭合速度，闭合候选为 1.2～1.5 m/s；没有将整个规划的终点强制设为 6.5 m/s。此前这种强制终点试验导致候选动力学不可行，失败记录保留。最新修复位于执行反馈层：原始 MINCO 参考仍可能末段下降，但最后 0.7 s 的实际水平指令和 PX4 测量速度在这三次记录中没有下降。

主要实现：

- [trajectory_tracking.py](../../src/uav_control/uav_control/control/trajectory_tracking.py)：分离测量与发布时刻，末端提前瞄准和限加速度转向。
- [trajectory_tracker_node.py](../../src/uav_control/uav_control/control/trajectory_tracker_node.py)：传入原始导航测量、新鲜 KF 速度及 BCTRA 预测查询。
- [baseline.yaml](../../src/uav_usv_bringup/config/baseline.yaml)、[visual_geometry_diagnostics.yaml](../../src/uav_usv_bringup/config/visual_geometry_diagnostics.yaml)：启用 `terminal_cruise_enabled`。
- [test_trajectory_tracking.py](../../src/uav_control/test/test_trajectory_tracking.py)：新增七个回归测试用例，覆盖反馈时间、末端转向不制动及早期轨迹保持原行为。

## 实验结果

三次均独立启动完整 Gazebo/PX4/ROS 任务，自动 X 起飞，在 X 后约 31.18 s 且满足新鲜锁定 FOLLOW 时发送 Y。目标参数和任务入口相同，实际导航状态与调度存在正常运行差异；不是对同一条录制轨迹重复计算。重复实验期间不调参数、不注入真值、不手动暂停。

| 指标 | 修复后首次 | 重复 1 | 重复 2 |
|---|---:|---:|---:|
| 结果 | SUCCESS | SUCCESS | SUCCESS |
| Y → 捕获 s | 4.413230 | 4.622916 | 4.484602 |
| 评价最小三维距离 m | 0.464702 | 0.482564 | 0.461706 |
| 捕获水平距离 m | 0.241781 | 0.265424 | 0.247500 |
| 捕获垂向误差 m | 0.396850 | 0.403011 | 0.389764 |
| 捕获相对速度 m/s | 2.826063 | 2.801147 | 2.870920 |
| 径向闭合速度 m/s | 1.412620 | 1.378342 | 1.328066 |
| 最后记录水平指令 m/s | 6.500000 | 6.500000 | 6.500000 |
| 捕获前最后记录实测水平速度 m/s | 6.803260 | 6.783259 | 6.828834 |
| 最后 0.7 s 实测峰值减最后值 m/s | 0 | 0 | 0 |
| 捕获前恢复次数 | 0 | 0 | 0 |
| 自动暂停及 UAV/目标共同冻结 | 通过 | 通过 | 通过 |

最后 0.7 s 每次有 14 个控制诊断样本，实测水平速度逐样本增加；指令变化中仅有小于 2×10⁻¹⁵ m/s 的浮点舍入下降。速度图保留导航消息的物理 sample epoch，不重写时间戳。表中最后实测值是捕获前最后收到的导航样本，不将它描述为精确碰撞瞬间测量。

指令上限 6.5 m/s 与实测速度有区别：三次任务实测水平峰值分别为 6.812254、6.789335、6.857137 m/s，存在 PX4 跟踪超调，均低于规划上限 7.0 m/s。没有宣称实测速度严格受 6.5 m/s 约束。

捕获后约 7 s 的额外只读核对同时检查 `/world/default/pose/info`、world stats 和 `/target/state`。三次均检测到自动 paused；世界时间不再增长，原生 UAV 与目标姿态固定，目标发布位置不变且速度为零。成功之后墙钟仍运行所产生的 PLAN_RECOVERY/SAFE_RECOVERY 诊断不计为捕获前恢复。第二次记录器退出后的 `/target/state` 仅有评价器订阅，没有在线 tracker/planner 订阅。

## 保留边界与验证范围

- Y 前 FOLLOW 保持 5 m 高度和 5 m 水平距离，起飞后直接 FOLLOW 的流程保持。
- 图像/KF/规划输入 freshness 为 125 ms；采集戳、因果等待、超时拒绝和预测插值边界保持。末端中的导航短时模型用于发布边界，未伪造测量 epoch。
- KF Q/R、捕获半径 0.50 m、planner 最大时域 1.5 s、predictor 时域 4.0 s 保持；真值限于评价与离线审计。
- 共同停止仍由评价器成功后暂停 Gazebo，目标节点根据原生 world stats 冻结积分与 set_pose；没有让真值评价结果参与在线制导。
- 这是同一目标参数和任务入口下的重复验证；不据此声明任意运动、不同入口条件或无标记真实 USV 的总体捕获率。红球仍属于感知接口测试。

当前完整测试为 **836 passed / 1 skipped / 2 warnings**，两条 warning 为已有 ament/flake8 的 SelectableGroups 弃用提示；项目 flake8、uav_control 与 uav_usv_bringup 构建通过。控制文件和配置 SHA256 与重复实验前保存的指纹一致。实验完成后关闭本次启动的仿真进程，保留全部成功及失败数据。

## 可复核证据

汇总：[terminal_cruise_repeat_validation.json](y_intercept_evidence_20261005/terminal_cruise_repeat_validation.json)。

| 运行 | 原始实验前缀 | 速度曲线 | 停止核对 |
|---|---|---|---|
| 修复后首次 | `modular_intercept_20261006_095151_101120_mission_1` | [曲线](y_intercept_evidence_20261005/terminal_cruise_user_phase_1_speed_audit.png) | [JSON](y_intercept_evidence_20261005/terminal_cruise_user_phase_1_freeze.json) |
| 重复 1 | `modular_intercept_20261006_095428_701507_mission_1` | [曲线](y_intercept_evidence_20261005/terminal_cruise_repeat_1_speed_audit.png) | [JSON](y_intercept_evidence_20261005/terminal_cruise_repeat_1_freeze.json) |
| 重复 2 | `modular_intercept_20261006_095732_501270_mission_1` | [曲线](y_intercept_evidence_20261005/terminal_cruise_repeat_2_speed_audit.png) | [JSON](y_intercept_evidence_20261005/terminal_cruise_repeat_2_freeze.json) |

原始 JSONL 在 `data/experiments/current/y_intercept_20261006_terminal_cruise_{user_phase_1,repeat_1,repeat_2}.jsonl`，同期 CSV、vision CSV、summary JSON 与配置快照按上表前缀保留。记录工具 [y_user_phase_record.py](y_intercept_evidence_20261005/y_user_phase_record.py)、离线速度工具 [terminal_speed_audit.py](y_intercept_evidence_20261005/terminal_speed_audit.py) 与 [pause_freeze_record.py](y_intercept_evidence_20261005/pause_freeze_record.py) 已保留。

测试/构建日志、指纹和真值订阅核对均在 `y_intercept_evidence_20261005/terminal_cruise_*`。旧的停止修复与失败试验历史见 [terminal_sphere_validation_20261005.md](terminal_sphere_validation_20261005.md)，本文件是最新收尾依据。
