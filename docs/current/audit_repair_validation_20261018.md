# 六项缺陷修复与单次变速 S 捕获验证

已完成[首次审查](code_audit_20261018.md)中的六项修复，随后只启动了一轮变速 S 型完整捕获实验。结果为成功进入原 **0.50 m** 接触半径并暂停世界，未自动复飞。当前控制仍是视觉/KF、BCTRA、MINCO FOLLOW 与 Y 后 IBVS/PNG，未实现或切换为 MPC。

本次以 `master` 的 `b3ee89ec4f66ca1acb90ae966d678e8ce02a2228` 加工作树修改运行，未提交或推送。飞行运行指纹为 `0ad1ff84f9025f3af43c182bd8a5a3021aeaeb230dbd136ad39a8d3e16c599e4`；运行快照、飞前源码和飞后当前源码逐项一致。原始失败材料、冻结基线、视频与原仓库均保持原样。

## 修复行为与回归

| 问题 | 修复后的行为 | 验证入口 |
|---|---|---|
| 普通基线终态重入 | core 完成后不再接受 tracker/tick 转移；清除截击请求与 active plan。Tracker 同任务终态锁存，并独立处理 `completed=true`；旧任务消息被拒绝，新任务 ID 才能重新开始 | `test_audit_terminal.py`，Q/capture/failure 与迟到状态 |
| 启动器误停外部 PX4 | 无归属证明的现存 PX4 只报告 PID 并阻止新启动，不再依据本 partition 无时钟而发送 SIGINT | `test_audit_startup.py`，真实 shell 分支、系统信号桩 |
| KF 未来时间戳阻塞 | 修改滤波状态前拒绝未来采集戳，后续正常图像继续接纳；不改写原戳 | `test_audit_kf.py`，初始化前后两种情况 |
| KF 拒绝后仍改变状态 | 在独立候选滤波状态上做 prediction/update，成功后一起提交；协方差拒绝或数值异常不改变状态、协方差与 epoch | `test_audit_kf.py`，负协方差、奇异更新与随后正常观测 |
| 跨坐标重置插值 | 位置/速度/heading 与 quaternion reset counter 发生变化时切断位置和姿态 history，锁存导航故障；感知/KF/预测/规划/Tracker 关闭旧坐标链路，任务进入失败态。首个有效 counter 只播种，不视为重置 | `test_navigation_frame_fault.py`，跨 reset、乱序、时间同步变化、队列输入、发布阻断、真实本机 DDS late join |
| 捕获统计越过结束时刻 | guidance 样本数与全部极值统一按物理结束时刻裁剪；末样本距离及其戳、末两秒中位距离分开记录；空窗口返回无效结果 | `test_audit_analysis.py`；旧 S 原始数据在新目录重分析，恢复为 290 帧与垂直误差峰值 1.591220 |

坐标故障通过 `/navigation/frame_fault` 的可靠、transient-local 消息传播；空消息、随后图像或重复 X/Y/R ROS 命令不能解除。故障时 Tracker 撤销旧轨迹并停止 PX4 setpoint/Offboard 发送，由 PX4 进入已有失联保护；需要命令台 R **完整重启**。此次没有尝试自动转换跨 epoch 轨迹，也没有注入真值校正。位置 counter 按因果恢复的原始 native sample epoch 排序，避免 DDS 线戳的正常 timesync 变化导致误拒绝。

另补齐了自动 P7 驱动的 `UAV_USV_P7_MODE` / `UAV_USV_P7_CAPTURE` 传递：上轮只验证了 launch 的条件图，自动驱动此前遗漏该标志，仍可能启动闲置旧规划器；本次实际启动日志确认 9 个主图进程，加按需启动的独立 FOLLOW 规划器，共 **10 个在线节点**，没有 `intercept_planner_node`。QGC 未自动打开，下视/额外诊断保持关闭。

## 唯一一轮实际结果

命令：

```bash
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
python3 /home/qin/data/uav_usv_mpc/scripts/p7_follow_sitl.py \
  /home/qin/data/uav_usv_mpc/data/experiments/20261018_audit_repair_s_01 \
  --scene f2 --guidance d --capture --motion variable_speed_s
```

| 指标 | 本轮结果 |
|---|---:|
| 运动模式 / 观察到的目标速度范围 | 变速 S / 2.0000–4.0000 m/s |
| 评价器结果 | `SUCCESS / CAPTURE_RADIUS_REACHED` |
| Y 接近阶段至捕获 | 11.858797 s |
| 评价器接触统计距离 | 0.498350 m |
| 独立导航/目标同刻点距离 | 0.478993 m |
| 独立 Gazebo 原生模型参考点距离 | 0.480021 m |
| 接管后、Y 前实际 MINCO 样本 | 153/153，观察跨度 6.260149 s |
| 异常撤权 / 到期 / 导航坐标故障 | 0 / 0 / 0 |
| 捕获前有效制导周期 | 238，均 `P7_GUIDANCE`、可见且锁定 |
| 捕获窗口原始采集图像 | 237 帧，0 无效 |
| 结束后的制导输出 | 2 帧，均排除出捕获前指标 |
| 实测水平速度 / 水平加速度峰值 | 4.899819 m/s / 1.426319 m/s² |
| 最低高度 / 竖直速度绝对峰值 | 0.355720 m / 0.634851 m/s |
| 水平 / 竖直图像误差绝对峰值 | 0.199573 / 0.779020 |
| 单一 Tracker / BCTRA、纯速度接口 | 均通过 |
| 世界暂停 / 时钟跨度 / 目标位置跨度 | 48 样本、4.574514 s / 0 / 0 |
| 清理错误 / 本任务残留进程 | 0 / 0 |

Y 切换时唯一的 `P7_GUIDANCE_ENTERED` 撤权是正常权限交接，没有计入异常回退。MINCO 的 100% 是接管后的局部样本占比，不代表整轮起飞期，也不替代固定 25 s FOLLOW 回归或 F3 资格。

严格窗口为 `[1792320852.1254041, 1792320863.9826999]`。独立导航 bracket 为 `[1792320863.976464, 1792320864.0004451]`，目标有事件同戳样本。Gazebo 事件时刻通过同条 clock 的 system/sim 原始对应进行 28.017 ms bracket 插值，模型位姿 bracket 为 `[53.784, 53.824]`，40 ms；使用配置中的 0.42 m 红球安装偏移还原 USV 参考点。没有外推、拟合延迟或调整坐标偏置。三种距离的输入与统计定义不同，数值没有声称逐毫米一致，但均在原接触半径内。

![本轮捕获距离、高度、轨迹与图像误差](../../data/experiments/20261018_audit_repair_s_analysis_01/capture.png)

## 已验证范围与保留边界

世界暂停、目标静止与捕获前视觉连续性已经核对。暂停后出现 `STATE_STALE` 和图像超时属于世界停止后的状态，没有计入捕获前故障。未取得暂停后重复的 ECS UAV 机体位姿，所以 `native_pose_verified=false`；不把世界时钟/目标冻结冒充为独立的重复机体冻结验证。

本轮存在 Gazebo 目标创建/位姿服务的短时重试警告，捕获前未造成观测无效或失锁；原日志保留。没有进行坐标重置、未来时间戳或异常协方差的在飞故障注入，这些边界由离线回归与本机 DDS 测试验证。近场整目标可见性仍无形式化三维约束，不宣称任意模式、任意姿态或无标记真实 USV 已被验证。

飞行中的只读 CLI 节点图探针在清理完成后才返回，未取得节点列表；10 节点结论依据实际 launch 的 process-start 日志、独立 FOLLOW 规划器日志及发布者记录。没有为获取节点图追加飞行。

完整 pytest 为 **2212 passed、2 skipped、3 warnings**，38.96 s；四包构建、指定范围 flake8、shell 语法、diff 检查通过。包前缀为 `/home/qin/data/uav_usv_mpc/install/uav_control`。飞后检查没有现存仿真/串口，UDP 8888 空闲。

## 证据与恢复入口

- [修复与复核包](../../data/experiments/20261018_audit_repair_01/)：修改前逐文件副本、计划、完整回归与构建、单轮授权、飞前指纹、严格分窗与两套同刻距离复算、SHA256。
- [唯一原始轮次](../../data/experiments/20261018_audit_repair_s_01/)：完整实际参数、源码快照、JSONL、ULog、终态、启动/规划/清理日志与 SHA256。
- [本轮曲线与离线数据](../../data/experiments/20261018_audit_repair_s_analysis_01/)：捕获曲线、距离 CSV、复算指标与原始数据指纹。
- [原 S 数据的新分析目录](../../data/experiments/20261018_audit_repair_01/previous_s_reanalysis/)：验证分析器修复，未覆盖旧材料。

新增实验仍处于默认忽略目录；本地存在不等于远程已备份。此次没有提交或推送。
