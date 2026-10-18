# P7 半径捕获与变速场景

本轮按用户的新要求扩展到仿真捕获。原 P7 安全接近入口仍可用；显式增加 `--capture` 才允许终端下降。评价使用既有 **0.50 m 三维半径**，不需要碰撞事件。

后续修复已完成原 **4 m/s 八字捕获并暂停**，以及三种变速模式各约 25 s 的 100% 实际 MINCO FOLLOW，详见[失效归因、修复与当前结果](p7_follow_repair_20261018.md)。下方保留首轮直线捕获与三种失败的历史记录；变速模式的捕获不在后续 FOLLOW 验证结论内。

## 使用

从工作区启动隔离、限时的交互会话：

```bash
cd /home/qin/data/uav_usv_mpc
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
python3 scripts/p7_follow_sitl.py \
  data/experiments/p7_capture_manual_NEW --manual --capture
```

输出目录必须不存在。起飞前用 1–5 选择模式，X 起飞并进入 BCTRA/MINCO FOLLOW，锁定稳定后 Y 切换到 IBVS/PNG 捕获。到达半径后 Gazebo 暂停。Q 中止，R 重启。交互会话总上限 280 s；退出后由运行器清理本会话进程。

自动完整 X/Y 单轮：

```bash
python3 scripts/p7_follow_sitl.py \
  data/experiments/p7_capture_variable_s_NEW \
  --guidance d --capture --motion variable_speed_s
```

三种变速模式使用现有 `variable_speed_eight`、`variable_speed_s`、`variable_speed_z`，保留原默认 2–4 m/s 速度规律与运动参数。完整配置、源码快照、原始记录、ULog 和 SHA256 均在各轮目录保存。

## 控制与终态边界

`p7_capture_enabled` 默认为 false。捕获时最大下降请求 0.60 m/s，最低高度请求为海面以上 0.33 m；目标在海面上方时相对目标再保留 0.33 m。水平参考为 `min(4.5, 0.30 + 2 * remaining_descent)`，避免下降结束前进入相机近场。垂直图像误差超过 0.70 时请求减小闭合/后退；该值经过当前相机安装位置、28°俯角和帧边界的独立端点检查。1 m 内使用有界目标速度匹配与相对位置控制，停止使用近零范围 PN 除法。原最终海面制动、失视线恢复、125 ms 输入有效期及单 Tracker setpoint 所有权保留。

在线输入来自图像、KF 与 PX4 导航。`/target/state` 仍仅用于评价和离线曲线；评价器成功后暂停世界，运行器只记录终态及退出，不把真值用于纠正控制。

捕获前检查实际 PX4 状态、单控制者与视觉；捕获后检查暂停标志、冻结的世界时钟、目标位置和零目标速度。暂停后的导航停止更新不作为飞行失效。重复原生位姿是单独的补充审计：`freeze.world_verified` 表示上述世界/目标暂停证据，`freeze.native_pose_verified` 表示重复当前位姿查询，`freeze.verified` 要求两者都通过。任务验收使用前者，不把后者未取得的结果写成已验证。

底层 FOLLOW 实验运行器原有 0.50 m **飞行高度停止线**并非捕获半径。显式捕获且实际进入 P7 后，运行器改为 0.25 m 停止线（同时取机体/控制净空要求的较大值），仍高于 0.23 m 机体下缘和 0.20 m 控制预留；普通 FOLLOW 保留原停止线。控制海面制动参数没有放宽。

## 首轮实测记录（后续修复前）

| 场景 | 本轮结果 | 证据 |
|---|---|---|
| 直线，完整 X→MINCO→Y | **捕获并暂停**；Y 后 12.3056 s，三维判定距离 0.498403 m | [原始轮次 08](../../data/experiments/20261018_p7_capture_linear_08/)、[独立分析](../../data/experiments/20261018_p7_capture_linear_analysis_08/metrics.json)、[曲线](../../data/experiments/20261018_p7_capture_linear_analysis_08/capture.png) |
| 变速八字 | **未通过**；Y 前 `FOLLOW_LOCK_RECOVERY_TIMEOUT`，实际 MINCO 约 3.01 s；`INPUT_WAIT→REACQUIRE` 时目标仍可见，末 4 s 无无效位置观测 | [原始轮次 02](../../data/experiments/20261018_p7_capture_variable_eight_02/)、[分析](../../data/experiments/20261018_p7_capture_variable_eight_analysis_02/metrics.json) |
| 变速 S | **未通过**；Y 前锁定恢复超时；末 4 s 记录 41 次 `DEPTH_RATIO_LOW`，未形成 MINCO 接管 | [原始轮次 01](../../data/experiments/20261018_p7_capture_variable_s_01/)、[分析](../../data/experiments/20261018_p7_capture_variable_s_analysis_01/metrics.json) |
| 变速 Z | **未通过**；初始追赶中 KF 不新鲜，末 4 s 有 69 次 `DEPTH_RATIO_LOW`；内层因失锁 2 s 结束，外层记录 `HEALTH_INPUT_LOST` | [原始轮次 01](../../data/experiments/20261018_p7_capture_variable_z_01/)、[分析](../../data/experiments/20261018_p7_capture_variable_z_analysis_01/metrics.json) |

直线捕获前实际 MINCO 输出跨度 6.31 s，P7 制导及目标可见比例均为 100%，速度峰值 2.772 m/s、水平加速度峰值 0.447 m/s²、最低高度 0.360 m，Y 指令速度步长 0.0140 m/s。只有一个 BCTRA 主预测器和一个最终 Tracker，P7 使用纯速度接口，Y 后无旧 FOLLOW ACTIVE/旧截击 proposal。末端垂直图像误差峰值 1.349：可见性是检测仍有效，不等于目标中心或整球始终位于帧内。

暂停后有 49 个世界统计样本，覆盖 4.921 s；`paused=true`、仿真时钟跨度 0、目标位置跨度 0、目标速度为零。**暂停期间未取得重复 ECS 位姿**，补充原生位姿冻结审计仍未通过。`scene/info` 已证实返回初始模型姿态，不能用于此证明；解码器已通过运行中 ECS 当前位置和有符号 protobuf 键检查，但不能据此外推为已验证暂停查询。

原始 08 的 `healthy=false` 采用了较早的“必须重复原生位姿”总门槛，原文件与指纹保留。独立分析明确分列任务所需世界暂停证据和补充位姿证据，并保存原判定、源码/原始记录 SHA256；其任务判定为通过，完整补充审计仍为 false。没有补做或伪造位姿。

三种变速模块未修改。八字轮次实际观察到约 2.00–4.00 m/s；S、Z 因早停只覆盖约 3.46–4.00 和 3.49–4.00 m/s，不能据此宣称完整速度周期或捕获通过。此前的参数选择/ACK 测试同样不能替代这些负结果。

早期直线 01–07 均保留：01 已捕获但终态记录竞态；02 暴露过早水平闭合；03 被原 FOLLOW 高度停止线截断；04 捕获后补充审计不足且曾恢复；05 图像保护过于保守，60 s 最近 0.5138 m；06 的 Y 在瞬时失锁中被正确拒绝；07 12.175 s 捕获，但旧 scene 位姿审计资格已在[独立复核](../../data/experiments/20261018_p7_capture_linear_reaudit_07/)撤销。变速八字 01 为核对 ECS 解码主动结束，未宣称捕获。这些调试轮次不作为额外通过样本。

## 验证与下一步

交付有 277 项定向测试、项目范围 flake8、shell 语法、两包构建和 diff 检查；包前缀为当前工作区。最终进程检查无 PX4/Gazebo/DDS 残留、无串口设备、UDP 8888 空闲。证据与 SHA256 见[本轮交付](../../data/experiments/20261018_p7_capture_delivery_01/)。按用户要求单智能体执行并自审，阶段提交不推送。

首轮留下的下一问题是稳定 FOLLOW 资格：对齐八字 `INPUT_WAIT` 起点和三帧重锁，以及 S/Z 同刻相机深度与起飞追赶时序。该归因和后续有界验证现已完成，结果见上方修复报告；首轮失败原件未改写。
