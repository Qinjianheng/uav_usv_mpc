# RGB-D 末端重新验收与间歇减速

当前默认前视 RGB-D、下视关闭、目标升沉 0.15 m / 0.25 Hz，ToF 代码和配置保留。MPC 尚未实现。修复后两轮独立全仿真均在首次末端接近捕获，MINCO 执行至捕获前水平指令没有下降，也没有进入 SAFE_RECOVERY。按用户要求，首轮成功后只追加一轮仿真。

原始记录、配置、修改前后代码指纹和可复算工具保存在 [本轮证据包](../../data/baselines/20261007_rgbd_terminal_replan_audit/README.md)。证据包不改写旧基线，包含用户提供的成功及失败记录。

## 实际触发条件

用户记录 `20261007_114314_183841` 为 FAILURE / TIMEOUT，最近距离 0.589769 m。第一次原接触时刻前的采样为水平距离 0.325 m、竖直间距 0.560 m，随后原轨迹到期进入 SAFE_RECOVERY 并刹车。原 0.50 m 捕获球没有触发。当时末端水平速度已约 6.8 m/s，不能将失败归结为保速参数关闭。

升沉低位时，海面保护限制下降，垂向误差占据捕获球的大部分余量。planner 已有 TERMINAL_CONTACT_RECOVERY，用新鲜预测寻找新的可达接触时刻；此前 tracker 的 TERMINAL_COMMITTED 无条件拒绝新轨迹，阻断了这条路径。新鲜度失效也会触发保护，需要单独统计，不能宣称所有减速均是同一原因。

用户成功记录 `20261007_114222_380966`、`20261007_114512_706892` 与上述失败原始 CSV/YAML/JSON 均已按字节保留。改动前另补录两轮前视 RGB-D 对照，分别为 FOLLOW 3 s、5 s 后 Y；均成功。这些对照不能作为重新验收修复的验证结果。

## 本轮修改与约束

- 保留用户对启动脚本默认 ideal 的修改，并对齐独立 modular launch 的默认值。
- TERMINAL_COMMITTED 中只有新鲜 KF、prediction、导航和视觉锁定均有效，才允许新候选继续经过原完整验收。
- 成功且实际接替的轨迹撤销旧 execution context；失败、失效或冗余候选不能修改旧期限或多项式。
- 轨迹剩余执行时间在当前 control epoch 再检查，几何验收仍使用原始导航 sample epoch。导航稍有延迟时，不会接纳已低于原 0.20 s 剩余期限的候选。
- MissionManager 仅在新轨迹的 PLAN_ACCEPTED 通过原门控时撤销旧期限，后续诊断为新的已验收轨迹建立执行窗口。
- 水平位置重合不等于三维捕获。水平转向目标暂时无法定义时保留上一实际水平指令，避免退回较慢的多项式速度；垂向保护和原期限仍有效。
- 失效后的保速仍只限最后 0.7 s，且不能超出相应已验收轨迹的原有效期限。125 ms、0.50 m 捕获球、海面保护、速度/加速度限制、KF Q/R 和真值隔离保持。
- BCTRA、MINCO 求解器、目标运动、模型和 PX4 补丁未修改。红球仍为感知接口测试，不能外推无标记真实 USV 能力。

## 验证记录

| 轮次 | 记录 ID | 结果 / 截击耗时 | evaluator 最近距离 | 接触前水平指令 |
|---|---|---|---|---|
| 修改后首轮 | `20261007_151119_585943` | SUCCESS / 10.448 s | 0.460 m | 3.976 → 6.500 m/s，无下降 |
| 独立重复 | `20261007_151710_496882` | SUCCESS / 10.876 s | 0.404 m | 3.905 → 6.500 m/s，无下降 |

两轮均为完整重新启动，baseline.yaml、RGB-D ideal、下视关闭；flight_ready 后 10 s 发 X，FOLLOW 连续 3 s 后发 Y。命令触发只依赖就绪和任务阶段，不读取真值。采集器订阅真值仅供离线评价。

首轮在 TERMINAL_COMMITTED 后实际接纳 plan 48，将接触时刻后移 **0.192974 s**；接替时视觉锁定有效，观测和预测 age 均为 **46.6 ms**，剩余执行时间 **0.826 s**，连续性误差经过原验收。下一候选因 STATE_VELOCITY_MISMATCH 被拒绝，已接纳轨迹保持。独立重复没有在 committed 状态后接替轨迹，同样成功。因此首轮覆盖了新接替路径，重复轮覆盖了继续原轨迹路径。

两轮捕获后约 6 s 的评价记录：暂停已见、仿真时钟跨度 0、目标位置跨度 0、目标速度 0；分别读取两次 Gazebo 原生 world state，暂停状态、时钟及机艇模型姿态一致。捕获后的 SAFE_RECOVERY 诊断不计入捕获前减速。距离采用一次性 InterceptResult；任务 CSV 以较低频率记录缓存样本，最后一行距离不能替代捕获事件的距离。

附带记录的末端有效视觉样本误差：首轮 RMSE 0.132 m / P95 0.184 m（22 样本），重复轮 RMSE 0.050 m / P95 0.103 m（25 样本）。这些是 evaluator 同图像时刻匹配真值的离线数据，本轮未修改感知算法，不应解读为定位精度改进。

实际静态检查：pytest **908 passed、1 skipped**（两条既有依赖 warning）；项目范围 flake8 通过；8 个已跟踪 shell 脚本 bash -n 通过；4 个 ROS 包构建通过；git diff --check 和 git diff --cached --check 通过。ros2 pkg prefix uav_control 指向本工作区 install/uav_control。独立代码审查的剩余期限和零水平距离问题已修复并回归。

旧基线 SHA256 全部通过（46 / 124 / 63 个文件）；9 个模型、world 和 PX4 补丁文件与 HEAD 相同。完整检查输出见证据包 checks/。生产源码在两轮仿真间及最终交付检查中保持相同指纹。

两轮成功不能证明所有升沉相位、失效时长及任意目标均必然捕获。超过 125 ms 的感知失效、导航失效、轨迹到期或安全退出仍会按原规则保护；此修复避免新鲜且验收合格时被旧 commitment 阻断，不能取消保护来保证速度。

## 后续启动

```bash
cd /home/qin/data/uav_usv_mpc
export UAV_USV_WS=/home/qin/data/uav_usv_mpc
export UAV_USV_EXPERIMENT_CONFIG_FILE=/home/qin/data/uav_usv_mpc/src/uav_usv_bringup/config/baseline.yaml
export UAV_USV_ENABLE_DOWN_CAMERA=false
export UAV_USV_FRONT_DEPTH_MODEL=ideal
export ROS_LOG_DIR=/home/qin/data/uav_usv_mpc/data/experiments/ros_logs
./scripts/uav_lab.sh --no-build
```

就绪后按 X，进入稳定 FOLLOW 后按 Y。更改源代码后先 ./scripts/build_workspace.sh。恢复 ToF 时显式设置 UAV_USV_FRONT_DEPTH_MODEL=tof 并重启完整仿真；ToF 的带噪失败边界仍以对应历史报告为准。
