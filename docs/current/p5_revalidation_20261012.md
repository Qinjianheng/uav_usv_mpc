# P5 修复后复验与启动阶段归因

**当前结论：F2_INPUT_STALE_BEFORE_TAKEOVER；P5 实际持续闭环未通过，F3 未执行。**

本次续做起点为用户已提交的 `877ec707d88213b1c4b3527a4e5ebf316e3358da`。
用户先回复“续续”，随后明确“继续，可以进入F3，确保完整修改解决问题，方便开展下一步工作”。
F3 许可保留原 F2 健康门禁；没有将许可解释为取消输入时效或允许失败后直接跳入 F3。
每次新授权对应新的固定根目录和排他次数记录，旧次数、原始日志及源码快照均保留。

## 实际尝试与边界

| 目录 | 实际结果 | X / Planner / MINCO ACTIVE |
|---|---|---|
| `20261012_p5_closed_loop/f2_h16_01` | 上轮 float32 健康记录器错误 | 是 / 否 / 0 |
| `20261012_p5_revalidation_01/f2_h16_01` | 起飞前 HEALTH_MONITOR_STALLED | 否 / 否 / 0 |
| `20261012_p5_revalidation_02/f2_h16_01` | PREDICTION_INPUT_STALE | 是 / 是 / 0 |

目录均位于本工作区 `data/experiments/`，完整日志继续忽略。
本次续做实际启动两次隔离会话，各自来自相邻的用户续做授权；没有更名自动重试。
F3 的尝试文件未创建。当前固定根目录 `_02` 的 F2 次数已消耗，不可直接重跑脚本。

`revalidation_01` 没有保存触发瞬间的健康快照和比较时间，因此不能断言本次实际触发
来自负年龄还是观察器停顿。确定发现并复现的代码缺陷是：先取 `now`，随后读到另一个
进程新写的快照，就可能出现 `written > now`，误报停顿。新版本已读后取时，且保留停止快照。
见 [第一次复验摘要](audit/p5_revalidation_20261012/revalidation_01_outcome.json)。

## 第二次复验：已实际加载研究 Planner

配置仍为 P47/C、qva、refinement=false、adaptive、H1.6、P44/D 后备。
真实隔离授权已签发，`condition.minco_authorized=true`；Planner 原始 JSONL 的
`research_mode=p47_follow_guided`、请求和完整约束指纹均记录。没有强制 ACTIVE。

| 必需指标 | 实际结果 |
|---|---|
| FollowTrajectory proposal / ACCEPTED / ACTIVE | 0 / 0 / 0 |
| MINCO 实际占比 / 连续控制 | 0% / 0s；25s 观察窗口未开始 |
| 停止原因 | PREDICTION_INPUT_STALE |
| 停止快照观测年龄 | 134.604ms，原门槛 125ms |
| 停止快照预测 source 年龄 | 107.439ms |
| 快照文件年龄 | 45.796ms，未触发 250ms 观察器停顿门槛 |
| 同期独立证据 | predictor 发布 STATE_STALE；Tracker 短暂 locked=false / REACQUIRE |
| PX4 指令唯一发布者 | 21 个完整发现样本均为原 Tracker；冲突 0 |
| lease 到期次数 / 接收余量 / MINCO 高度误差 | 没有计划，无法评价回退改善或接收性能 |
| 清理 | 原 harness、P5 及晚启动组件无存活；宿主预检无仿真/串口、UDP8888 空闲 |
| F3 | 不满足健康 F2 门禁，未执行 |

停止时快照确实已超过原始期限，独立记录证实发生过短暂失效；不能把本次中止归咎于
求解器高度漂移或 Tracker 接管失败，也不能将稍后的输入恢复反推为当时没有失效。
健康快照有采样延迟，因此不宣称外层停止时刻等于输入首次失效时刻。

原 FOLLOW 的短暂起飞后过渡窗口水平 RMSE 15.852m、高度 RMSE 0.0718m，整球安全比例
89.35%，最小垂直裕度 -0.01887rad；没有 20–60s 稳态人口。这些是**原 FOLLOW 过渡**，
不能用来比较 P47 收益或证明 P47 高度控制。红球仍仅为仿真感知接口测试。

完整轻量结果与逐文件来源 SHA：
[第二次复验摘要](audit/p5_revalidation_20261012/revalidation_02_outcome.json)。
完整本机目录各有 `P5_DELIVERY_SHA256SUMS`；原始 summary/JSONL 不覆写。

## 同输入离线归因

只回放本轮三个已记录请求，没有重跑历史矩阵或新仿真；原始 stamp、状态和约束不变。
回放时钟为原请求时刻加真实测得的求解耗时，保留原到期时间和 20ms reserve。

| 周期 | 在停止前完成 | 原剩余计算预算 ms | 原求解 ms | 同输入离线回放 |
|---|---|---:|---:|---|
| 1 | 是 | 32.576 | 38.484 | 31.204ms，VISIBILITY_INFEASIBLE |
| 2 | 否，收尾期 | 48.519 | 89.011 | 49.205ms，DEADLINE_EXCEEDED |
| 3 | 否，收尾期 | 39.293 | 43.728 | 22.962ms，SAMPLED_FEASIBLE |

原周期 1 在剩余计算预算内未完成，正常拒绝；另外两个周期在外层 STOP 后的收尾期，
不能混入有效控制观察人口。只有三个启动请求，不据此宣称在线 P95/P99 或收益。
回放证明启动状态和实时预算均须处理；“Planner 启动负载导致图像延迟”仍是待隔离的因果假设。

在实际 Planner 启动前已经收到的 KF/native navigation 数据上做共同 epoch 核验：
UAV 水平速度约 6.474m/s、目标约 1.950m/s，相对速度 4.525m/s、跟随位置误差 8.255m。
KF 位置是其原始 stamp 上的投影状态；导航仅在已收到的原始物理样本之间插值，不外推，
没有重写图像或 native stamp，没有真值进入实验启动决策。

因此旧“FOLLOW 健康持续 5s”只是健康门槛，没有保证跟随已收敛。
新只读启动门禁明确要求同刻水平误差 ≤1m、相对水平速度 ≤0.5m/s、高度误差 ≤0.1m
连续 5s；缺失/陈旧/无法找到导航括区间即不计时，60s 未收敛就停止。
这些是实验启动条件，控制限值、125ms 时效和 450ms lease 不变。
[新门禁对本次记录的核验](audit/p5_revalidation_20261012/warmup_repair_check.json)
确认它会拒绝该启动时刻。**此最后一项修复没有再次 SITL，不能宣称实际问题全部解决。**

可复算本轮三个请求：

```bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
cd /home/qin/data/uav_usv_mpc
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 \
  data/experiments/20261012_p5_revalidation_02/diagnosis_02/replay_requests.py \
  data/experiments/20261012_p5_revalidation_02/diagnosis_replay_03
```

该命令只接受新的输出目录，已存在则拒绝，避免覆写原记录；仅离线求解，不授予执行权限。
计时随本机负载变化，原始实跑数据始终以 SHA 索引为准。

## 修复和验证

本次修改仅涉及新增 P5 工具与其测试：

- 原子快照读取后取时间，保留停止快照和真实比较时间，继续拒绝未来/陈旧值。
- 等待完整 environment 发布，拒绝其他实验 scope；清理 scope 不明时不报告成功。
- 原 harness 收尾后，按精确 root/domain/partition、PID/start 双检清理晚启动组件。
  `_01` 漏出的 PX4 组经宿主证实本轮所有权后精确停止；没有按进程名杀其他任务。
- 只读 KF/native navigation 共同 epoch 稳态启动门禁，替代单纯健康计时。

最新 pytest：**1620 passed、1 skipped、3 warnings（16.75s）**；P5 专项 52 项通过。
项目 flake8、diff 检查通过。实际 ROS 只读记录器集成检查 185 个快照，年龄 0.2918–50.154ms，
无输入时从不认作稳态，退出码 0；该检查无飞行、无发布者。
算法、Tracker、预测器、模型、PX4 补丁及研究配置保持原字节，173 个封版源码 SHA 再核通过。
四包构建沿用上一轮成功证据；本次未改变 ROS 生产包文件，不冒称本次重新构建。
独立审查发现的 environment 竞态已用 RED→GREEN 回归修复；后续稳态门禁增加对应单测及原记录核验。

本轮实跑源码快照早于最后稳态门禁修改，当前版本另存
[修复版本指纹](audit/p5_revalidation_20261012/repair_sources.json)，不混称为同一运行版本。

## 下一步主问题

下一阶段仍是实际接管链：先以新稳态门禁启动，针对输入原始年龄、消息到达及完整求解/重验/
发布预算作一次有明确目的的验证；若仍失败，继续离线剖析阶段耗时。不得延长 TTL/lease、
放宽 FOV、用真值校正或先调整 PX4。健康 F2 完整通过后才开展 F3，随后才能进入预测/FOV 创新。
本次不再新增仿真，不自动提交或推送，也不重写 Git 历史。
