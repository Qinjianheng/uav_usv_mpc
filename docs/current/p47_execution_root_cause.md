# P4.7：P4.5 实际执行归因

审计起点 HEAD：`353c3e1a9dba15277b156896e1270bd0abe74f97`。本报告只使用
P4.5 已保存的四轮 F2 原始记录；没有新增飞行，也没有改动 Tracker、PX4、lease
或安全约束。新分析位于 `data/experiments/20261010_p47_follow_guided/`。

## 结论和适用范围

H1.6 的米级高度误差首先是**已执行参考曲线偏离 -5 m**。在共同导航采样时刻，
UAV 对 MINCO 参考的垂向 RMSE 只有 0.053/0.059 m。不能据此把米级误差归为
PX4 跟踪不足。原 FOLLOW 回退会改变后续参考和命令，但不是所有高度偏差的唯一原因。

四轮共 131 次 `MINCO_FOLLOW → FOLLOW` 的直接原因均为 0.45 s 名义 lease
到期。129 次有 EXPIRED ACK；另 2 次在同一 receiver tick 中被 pending plan 的
START_MISSED ACK 覆盖，原 active deadline 与 owner 转换仍可重建。到期前没有
及时可激活的接替曲线，不能把这些转换统称为 jerk 规划失败。

## 同一导航 epoch 下的误差分解

期望观测点为真值 USV 水平位置减去沿速度方向的 5 m，NED z 固定 -5 m；真值
仅用于离线评价。配对使用原始 `execution_debug.state_stamp` 与对应 native
navigation sample；排除早于 active start 而被 sample clamp 的记录。USV 真值只在
150 ms 内包围插值，无外推、无经验时延。向量恒等式为：

`实际−期望 = (参考−期望) + (实际−参考)`。

下表统一为 FOLLOW 开始后 [20,60] s 内的 MINCO_ACTIVE 有效配对样本，单位 m。
RMSE 不是可直接相加的量；CSV/JSON 同时保留交叉项和恒等式闭合残差。

| 轮次 | H | N | XY 参考−期望 | XY 实际−参考 | XY 实际−期望 | Z 参考−期望 | Z 实际−参考 | Z 实际−期望 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| b_f2_1 | 1.2 | 447 | 2.6795 | 0.1006 | 2.6961 | 0.0773 | 0.0111 | 0.0796 |
| b_f2_2 | 1.2 | 485 | 0.9272 | 0.1071 | 0.9192 | 0.0528 | 0.0086 | 0.0519 |
| c_f2_1 | 1.6 | 515 | 0.6105 | 0.0768 | 0.5639 | 1.5163 | 0.0532 | 1.5079 |
| c_f2_2 | 1.6 | 457 | 0.3850 | 0.0510 | 0.3539 | 0.5227 | 0.0589 | 0.5212 |

这个 ACTIVE 条件窗口与 evaluator 的全 owner 窗口不同。后者同一 [20,60] s 的
高度 RMSE 分别为 0.0712、0.0522、2.2456、1.0909 m，包含回退和重新接管。
不能将两个口径混用，也不能从两轮重复外推任意场景的 H 优劣。

## 垂向信号与已证实的生产者机制

| 轮次 | ACTIVE z_ref 范围 m | ref vz RMSE m/s | 最终 ROS vz RMSE | 实测 vz RMSE | 垂向 shaping 改变样本 | sea guard 改变样本 |
|---|---|---:|---:|---:|---:|---:|
| b_f2_1 | [-5.044,-4.729] | .0362 | .0364 | .0418 | 0 | 0 |
| b_f2_2 | [-5.143,-4.850] | .0353 | .0353 | .0371 | 0 | 0 |
| c_f2_1 | [-9.491,-.786] | 1.2134 | 1.2271 | 1.2313 | 29 | 0 |
| c_f2_2 | [-7.005,-3.335] | .7458 | .7765 | .7210 | 24 | 0 |

shaping/guard 是由日志输入与字节匹配的运行源码复算的派生信号，不是当时直接记录。
复算最终 ROS 三轴速度最大残差约 1.2e-7 m/s；稳定窗 guard 均为 SAFE。ULog
native position 与原始 navigation 的 4203–4287 个精确 sample 逐轮匹配，最大位置
差为 0。ULog `trajectory_setpoint` 只有约 5 Hz，不能宣称每个控制 tick 的 PX4
接收瞬间都已直接观测。

现有 P44 candidate rank 的位置/速度误差只含 XY；候选 scale 同时缩放 XYZ jerk，
而局部多项式端点随起始 PVA 和 jerk 积分，并不强制 z=-5。Accepted parent 保持
PVA 连续，也会保留非零 vz/az。因而参考高度可以持续漂移，Tracker 仍能准确跟随。
c_f2_1 的 167 个 |z_ref+5|≥1 m 样本中，scale=0/.5/1 分别为 86/59/22，74 个
scale=0 样本有 parent。这些已绑定已发布曲线和实际 ACTIVE debug。c_f2_2 的
43 个高误差样本没有 scale=0，所以不能把所有高度偏差简化为零 jerk 单一原因。

P4.7 应在参考生成/固定时域边界处保留原 FOLLOW 的固定高度目标；当前证据不支持
盲目修改 Tracker 增益或 PX4 参数。P44/D 作为冻结基线保持原行为。尚未验证新的
参考生产者在实际植物中的闭环收益。

## 控制权时间线与回退

| 轮次 | ACCEPTED / ACTIVE | 到期回退 | active 租约内无接替 proposal | 接替被拒 | accepted 接替错过 start | proposal 间隔 P95 s |
|---|---|---:|---:|---:|---:|---:|
| b_f2_1 | 140 / 137 | 35 | 32 | 3 | 0 | 1.417 |
| b_f2_2 | 145 / 140 | 31 | 28 | 1 | 2 | 1.498 |
| c_f2_1 | 166 / 165 | 28 | 25 | 2 | 1 | 1.029 |
| c_f2_2 | 152 / 148 | 37 | 31 | 3 | 3 | 1.064 |

四轮接替提案间隔中位数约 .216–.223 s，但超过 lease 的间隔分别 38/29/29/33。
publish→start 中位数只有 .095–.101 s，配置的 .15 s lead 已被前置求解和发布开销
消耗。上游 nonpublication/rejection 原因逐事件保存在 switches 与 planner 时间线中，
不能把接替空档唯一归到某个物理状态或某种求解失败。

回退前最后 prefix 均为 VALID。没有 REVOKED；没有证据表明 PREDICTION_CHANGED
直接撤销 active。代码在 tick 的 context 失配中显式排除该项，使用最新预测重验
prefix。PARENT_REFERENCE_INVALID 是接替候选拒绝，和旧 active 到期是两种事件。
receiver 没有以实际 PVA 偏离参考直接撤销曲线的门槛。

## 切换与连续性

| 轮次 | 到期前实际−参考位置误差中位 / P95 m | 连续控制 tick 最终命令变化中位 / P95 m/s |
|---|---|---|
| b_f2_1 | .0791 / .2220 | .1553 / .2124 |
| b_f2_2 | .0693 / .2474 | .1552 / .2126 |
| c_f2_1 | .0599 / .1758 | .2121 / .2131 |
| c_f2_2 | .0536 / .1867 | .1608 / .2127 |

逐次切换 CSV 保留旧参考 PVA、实际 PVA、旧/新命令和高度误差，均为其真实时戳。
原 FOLLOW 几何 PVA 没有直接记录，PX4 速度模式 position/acceleration 为 NaN；
不能捏造一个切换瞬间的 FOLLOW 参考 PVA。Accepted 曲线交接检查 11 维参考
PVA/yaw/rate；这个参考间连续性既不等于实际 UAV 重合，也不保证后续不会因 lease
空档进入不同的 FOLLOW 命令。命令整形限制实际速度 setpoint 的步变。

## 证据、复算与未验证项

输入：`data/experiments/20261010_p45_minco_execution/{b_f2_1,b_f2_2,c_f2_1,c_f2_2}/`
中的 `offline_evidence.jsonl`、shadow ACK/solver 日志、condition/config、source_snapshot
和 ULog。`config/flight.yaml` 的禁用项被 launch override 覆盖，最终 owner/ACK 是
实际执行的证据；不能把该 YAML 当完整有效运行配置。

新输出和原始 SHA256 索引：

- `data/experiments/20261010_p47_follow_guided/execution_physical/revision_02/`：最终每轮
  `diagnosis.json`、`active_same_epoch.csv`、原生信号、派生整形复算、PNG、来源 manifest。
- `data/experiments/20261010_p47_follow_guided/execution_timeline/`：每轮
  `summary.json`、`switches.json/csv`、`callbacks.json`、`original_clock_timeline.jsonl`、
  `timeline.png`、来源 SHA256。独立 shadow ACK 与 monitor ACK 四轮全部一致。
  40 个原始输入前后 hash 相同，35 个新产物 SHA256 校验通过；physical revision_02
  的 48 个原始输入前后 hash 相同，42 个新产物 SHA256 校验通过。
- 复算工具：`scripts/p47_physical_diagnosis.py` 和
  `scripts/p47_execution_timeline.py`；分别 8/9 项针对性测试通过，核心接口经历 RED→GREEN。

尚未直接观测完整率的 PX4 接收时刻、shaping 中间变量与 safety_state；已清楚区分
源码匹配复算与直接记录。没有进行植物反事实试验。后续最多一次 F2 的受控验证只有在
离线可行率、长期质量、时效余量和隔离门禁全部通过后才可启动；本阶段不授权绕过门禁。
