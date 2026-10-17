# P3.2 实时观测点规划、起点动态可行性与交接验证

本轮结论：已完成可安全执行的离线、影子算法和 PX4/Gazebo 验证。原 FOLLOW、MissionManager、Tracker、偏航仲裁、PX4 setpoint 发布及所有安全/新鲜度门限保持原配置。可以继续研究短时域规划和交接实现；尚不具备自动接管 Tracker 的条件。

证据目录：`/home/qin/data/uav_usv_mpc/data/experiments/20261009_p32_realtime_follow/`（以下简称 R）。旧仓库只读，未 fetch/pull、暂存、提交、推送或清理实验。起始分支 master，HEAD `c1e57663634dad71824539fd8c93a665e61bed6b`，工作区干净；本轮以实际文件为准。

## 1. P3.1 问题归因

对冻结 F3 全部 146 个 request 逐函数 cProfile，初始化累计 3.303 s：`assess` 5708 次/2.276 s、P1 可见性 1.253 s、输入验证 0.991 s、旋转验证 11082 次/0.698 s、观测点运动学 6146 次/0.481 s、目标插值 6730 次/0.389 s、运动学拟合 438 次/0.222 s、规划姿态 5708 次/0.203 s。各累计时间有调用嵌套，不能相加。实际几何计算约 0.193 s；重复静态参数/SO(3) 检查与小数组调用是主要热点。完整候选 JSON 序列化约 3–4 ms/周期，单独测量，不能当成 MINCO 核心计算。

正常回放的原初始化 P95 14.47 ms，批量 5.89 ms，有界候选 4.68 ms。带 cProfile 的对应 P95 为 25.86/8.92/7.34 ms，不能混用两组时间。完整初始化加 Q/T 调整有额外开销。`profile_v2/functions.json`、`baseline_profile.pstats` 保留原始函数级证据。

从真实 P3/P3.1 F3 原记录重算 61 条动态失败曲线，60 条水平 jerk 全局峰值位于起点。不能仅归因于 Q1：`j(0)=6*c3` 同时依赖 P0/V0/A0、Q1、Q2、各段时间及终点边界。`start_jerk_forensic.jsonl` 的 213 条记录是重新构造的初值，不冒充历史实际候选；`actual_dynamic_failure_peaks.json` 才是实际失败曲线归因。未发现这批初值的 A0 本身超限；主要问题是短时间内从当前 P/V/A 强行接近观测点所需 jerk。

时间问题独立于可行性：原始图像/导航 125 ms TTL，未来执行起点 now+150 ms。预测覆盖至 4 s 不代表测量新鲜到那个时刻，也不是轨迹持有授权。研究发布合法与未来执行合法必须分开。

## 2. 实现与约束

- `camera_visibility_batch.py` 批量规划姿态和旋转校验，静态相机标定只验证一次；几何仍逐行调用原 P1 `_evaluate_geometry`，没有另造宽松代理。非法行回落原 P1 原因码。
- `follow_reference.py` 新增批量观测点运动学；旧标量接口不变。`FollowProblem.assess(batch=True)` 为显式研究选项，旧默认路径不变。
- 初始化 opt-in 两级评价：固定 Q/时间下必要条件 `||p-p0-v0*t-a0*t²/2|| <= Jmax*t³/6`，仅筛选候选，不能证明最终曲线安全。无可达候选时保留显式不安全初值供诊断，不将其发布为有效。
- 可行性优先粗排序和有限候选改变策略；单纯批量计算不改变候选与排序。L0 固定后方、L1 三方向、L2 最多 6 候选/节点（18 总候选）、L3 只有剩余预算足够才进行原快速 Q/QT 优化。
- 初始化前计算 `min(原 solve budget, 原始 TTL-now-publication reserve)`，预留发布 20 ms、完整验收 25 ms；每次有限候选初始化合作式预算 12 ms。边界检查不是抢占式实时保证；单次 NumPy/P1 调用、GIL、ROS 回调排队会超出 12 ms，在线尾部数据如实报告。
- `start_jerk.py` 调整 Q1/T1 初值，固定总时域、起终点 P/V/A，半径 0.75 m，比较解析全局水平 jerk 峰值和 jerk² 积分；拒绝增加起点 jerk 的调整。精调 L1 只改 Q，L2 改 Q/T。最后仍需完整验收。
- `polynomial_extrema.py` 避免重复多项式乘积和补零；所有解析导数峰值门限不变。相机距离、FOV、海面余量、速度/加速度/jerk、数据年龄均未放宽。

## 3. 数值与同输入验证

268 个 P3.1 request 的原默认初始化逐字段完全一致。四组回放合计 43,364 对候选，标量/批量最大差异 `5.684e-14`，无排序、选择、粗可行性或结果差异。覆盖实际下倾安装、平移外参、FRD/NED 与 FLU/光学符号、无效旋转、后方目标、边缘和完整目标安全视场。证明的是已有相机近似球模型一致，不是任意真实艇体或姿态不确定性的连续时间保证。

冻结样本：P3.1 F2=122、P3.1 F3=146、P3 F2=208、P3 F3=67，共 543；12 方法×2 边界组=13,032 条，保留每个输入哈希、候选、Q/T/yaw、边界、时间、验收及约束指标。`replay_*/provenance.json` 指向未修改的原 JSONL。分别报告固定同一远端终点与自由观测终点，不能把两种实验的成功数混为一项收益。

### F3 同输入消融（146 个 request）

| 方法 | 同边界可行数 | 自由终端可行数 | 自由终端初始化 P95 ms |
|---|---:|---:|---:|
| p31 | 48 | 55 | 14.47 |
| batch | 48 | 55 | 5.89 |
| two_level | 48 | 57 | 5.46 |
| feasible_first | 48 | 55 | 5.82 |
| bounded | 48 | 48 | 4.68 |
| start_score | 48 | 55 | 5.54 |
| time_allocation | 48 | 55 | 6.52 |
| q1 | 68 | 93 | 6.91 |
| q1_t1 | 68 | 93 | 8.72 |
| full_initializer | 59 | 59 | 7.74 |
| early_budget | 13 | 15 | 13.90 |
| full_p32 | 50 | 63 | 14.33 |

批量化本身保留 55/146；删候选的 bounded 自由终端降为 48/146，不能声称加速无代价。Q1 调整改善到 93/146，单独改时间没有改善；同边界对应 48→68，说明收益部分来自早期平滑，部分来自自由终端。完整实时调度可行数低于不受在线时效限制的消融，因为它保留原始 TTL 并优先便宜固定候选。精调后干净回放四组结果为 94/122、63/146、184/208、26/67。另一次与回归负载重叠的回放保存在 `refined_replay_with_regression_load.*`，不作为干净耗时基准。

所有方法的 FOV、跟随 RMSE、jerk 积分及具体失败原因见各 `summary.json/records.jsonl`；RMSE 是预测轨迹质量。B/C/D/E 四组规划对照，F2 可行数 93/93/95/106，F3 为 19/19/22/55，详见 `follow_planning_groups.jsonl`。

## 4. 原 FOLLOW 与影子轨迹的边界

原 FOLLOW 已执行，研究轨迹没有执行。原 FOLLOW 固定 20–80 s 稳定窗口位置 RMSE：历史 P3.1 F2 0.855 m，F3 0.684 m；本轮 F2_retry 0.798 m，首次 F3 0.685 m。全程指标包含起飞/追赶，另存 `original_follow_comparison.json`。不同轮次 UAV 初始运动与实际艇体更新并不完全一致，因此不能宣称新规划器改善实际 FOLLOW。KF 测量误差、规划参考误差、控制跟随误差各自独立，evaluator 快照不是图像同刻测量。

## 5. 真实 PX4/Gazebo 影子实验

所有实验沿用对应 F2/F3 原飞行配置，X 进入 FOLLOW，未发送 Y；12 个轻量节点含只读 recorder。冻结研究源码、配置、参数快照、raw CSV/JSONL、launcher/monitor/ROS 日志和 owned-session 身份记录。仅停止本次 PID/start-time/进程组匹配的会话。研究节点只发布 research trajectory/diagnostic；实际三类 PX4 发布者仍各为原 Tracker 一个。

| 实验 | 完成周期 | 核心采样可行 | 时效合法研究发布 | 初始化 P50/P95/P99 ms | 整周期 P50/P95/P99 ms |
|---|---:|---:|---:|---|---|
| f2_retry | 118 | 46 | 42 | 1.19/18.81/22.86 | 46.58/72.47/77.71 |
| f3 | 139 | 7 | 7 | 1.51/28.08/32.40 | 27.65/63.16/81.40 |
| f3_refined | 146 | 19 | 18 | 1.48/28.45/32.22 | 29.40/62.58/69.03 |

- `f2_retry`：失败/成功状态 `{'DYNAMIC_INFEASIBLE': 6, 'INITIALIZATION_DEADLINE': 9, 'NO_FRESHNESS_BUDGET': 27, 'DEADLINE_EXCEEDED': 27, 'SAMPLED_FEASIBLE': 46, 'NO_VALIDATION_BUDGET': 3}`；初始化前剩余新鲜度中位数 55.3 ms，规划后 33.6 ms。
- `f3`：失败/成功状态 `{'INITIALIZATION_DEADLINE': 26, 'DYNAMIC_INFEASIBLE': 31, 'NO_FRESHNESS_BUDGET': 46, 'NO_VALIDATION_BUDGET': 22, 'SAMPLED_FEASIBLE': 7, 'DEADLINE_EXCEEDED': 7}`；初始化前剩余新鲜度中位数 53.9 ms，规划后 39.2 ms。
- `f3_refined`：失败/成功状态 `{'DYNAMIC_INFEASIBLE': 36, 'INITIALIZATION_DEADLINE': 44, 'NO_FRESHNESS_BUDGET': 38, 'SAMPLED_FEASIBLE': 19, 'DEADLINE_EXCEEDED': 4, 'NO_VALIDATION_BUDGET': 5}`；初始化前剩余新鲜度中位数 56.2 ms，规划后 39.5 ms。

首次 F3 有效 7/139，精调 18/146，较 P3.1 原记录 0/146 恢复合法研究发布；F2 42/118，原 P3.1 为 16/122。不同仿真不能要求相同分母，因果收益需结合冻结回放。精调可行层 L0=15、L1=4，L1 单层初始化 P95=22.78 ms，仍不是硬 12 ms；多层总初始化 P95=28.45 ms。L2/L3 难获足够预算。核心 19 个中 1 个未通过最终发布准入，保留拒绝，不补造成功。

20 s 采样的本次所有进程组 CPU 均值 F2/F3/精调 F3 为 310.99/297.19/289.73%，RSS 总和 1242.34/1242.47/1245.59 MiB；一个 CPU 核为 100%，共享内存重复计入 RSS，未包括 Codex/QGC/额外 CLI。实际艇体位置与理想场景的 native 双时钟对照 RMSE 为 0.113/0.295/0.242 m，P99 0.202/0.678/0.431 m，未拟合经验延迟。F3 存在实际运动/更新偏差，不能当成完全相同的理想八字对照。

初次 `f2` 的只读 monitor 在 NumPy float32 序列化失败，已修复并单测，停止自有会话后重跑 `f2_retry`；保留失败证据。F2 node-info 查询曾受 daemon 缓存影响失败，F3 用独立无 daemon 证据确认发布者，未把失败查询当成功。

早期 P3.2 raw record 的顶层 `research_mode` 因缺失 solver.mode 错误回落为 mpc_seed，实际 engine/config/candidate_kind 是 follow MINCO。现已显式设为 greedy_minco；历史 raw 不改写，其数值不受此诊断标签影响。

## 6. 追加需求：观测到 BCTRA 的延迟与短时域滚动规划

用户在本轮中进一步明确授权检查并改善 CTRA 输出延迟，以及缩短贪心/MINCO 预测时域。因此额外修改了三个 predictor 文件，其他原控制链文件仍保持原指纹。未修改 BCTRA 参数、滤波 Q/R、预测模型、积分步长、限幅、采集戳、发布去重、125 ms TTL、KF/定位/桥或主飞行配置。

代码检查确认：KF measurement callback 已立即发布，BCTRA state callback 已立即生成；同一图像的定时 KF 投影不会当作新观测。20 Hz 定时器负责过期状态报告，不能凭频率推断正常观测必然等待 50 ms。

BCTRA 原 `PredictionEngine.generate` 为每个未来时刻从零积分，默认 4 s/0.1 s=41 点，重复执行相同积分前缀。新增 `predict_many` 对积分网格对齐的时刻共享前缀，非对齐时刻使用未改动的标量 `predict`；CV/恒转率/低速/非法值保持原处理。限速/限转率及纵向衰减均保留。每轮前缀只在显式同一输入中共享，绝不跨新观测缓存。默认 4 s 主预测覆盖不缩短，确保原控制消费者不受影响。

400 次、无 profiler 单线程测量：41 点标量序列 P50/P95/P99=1.884/2.005/2.168 ms，共享前缀 0.136/0.147/0.159 ms；ROS 消息构造另需 0.562/0.592/0.633 ms。位置/速度最大差异 `7.105e-15`。更新 `compute_time` 为从进入生成到 Python ROS 消息构造完毕，不含 DDS publish/接收排队；不能把历史只含算法的 compute_time 与新定义直接当同一指标。`prediction_compute_benchmark.json` 保留基准。

原精调 F3 去重 238 个有效预测：采集→KF 求值 epoch P95=47.43 ms，KF epoch→生成 P95=1.39 ms，生成→影子接收 P95=11.74 ms，采集→影子接收 P95=56.15 ms。阶段百分位不可相加；KF epoch 是状态求值时刻，不是 callback 收到时刻。旧证据没有全链路 observation/prediction recorder，不能把其 47 ms 全归为定位计算。因此新增只读 recorder 接收定位时间戳和 prediction compute_time，单独报告采集/接收/处理/发布/预测及传输。

### 短时域局部规划

新增 opt-in `ShortHorizonFollowSolver`，默认 1.2 s，允许 0.8–1.6 s；配置 `p32_short_follow_research.yaml` 使用 5 Hz 单 worker 影子请求，不改变原默认配置。贪心排序目标进展、速度匹配，只试至多 3 个有限 jerk（加可用历史 hint 最多 4 个）。原 MINCO S3 映射采样和原完整安全验收均保留，不新增优化器。

局部初值 `p(t)=p0+v0*t+a0*t²/2+j*t³/6`，使用该曲线的 Q、终点 P/V/A、同一 execution epoch/总时域映射到原 MINCO。独立已知多项式测试确认映射还原原三次曲线及常 jerk，初值并不意味着安全。yaw 独立参考且受原角速度/可见性验收。局部进展终点未承诺在短时间到达 5 m 观测参考，不宣称终端收敛或稳定性。

四组全部 543 request、3 时域、cold/rolling 和固定远端终点负对照，共 4887 条记录，保存于 `short_horizon_replay.jsonl`，汇总 `short_horizon_summary.json`。F3/146：

| 时域 | 局部 cold 可行数 | rolling 可行数 | 整计算 P95 ms（cold） | 强制原远端终点可行数 |
|---|---:|---:|---:|---:|
| 0.8 s | 89 | 89 | 20.33 | 0 |
| 1.2 s | 97 | 97 | 21.25 | 0 |
| 1.6 s | 104 | 104 | 21.00 | 0 |

局部短终点与原 2.4 s 远端终点不是同一个问题，只能说明有限动态可行的局部进展有研究价值。1.2 s 97/146 不能冒充同边界击败原 2.4 s 的优化器。完整验收仍占大部分计算；较少视场样本并不自动成比例降低整周期时间。

同一批 148 条预测与后续 Gazebo 实际艇体位置的水平离线误差 RMSE：0.8/1.2/1.6/2.4/4 s 分别 0.819/1.061/1.362/2.031/4.389 m，P95 0.850/1.155/1.647/2.664/7.463 m。只取共同具有后续覆盖、保留完整快照的样本，使用原 native 仿真时钟映射，无经验时间平移/真值纠偏/外推；该误差包含 KF 初始状态误差、BCTRA 预测误差和实际场景更新偏差，并非 BCTRA 增量误差。真值仅进入只读评价文件。

### 滚动方式能否降低延迟和提高效果

短时域频繁重规划能减少远期模型误差，并使每次候选规模可控；复用上一轮有效初始化可能节省求解，但不缩短采集、定位和 KF 的上游延迟，也不保证稳定性、可行性或 CPU 尾延迟改善。5 Hz 会增加 CPU 使用率，需要单 worker/busy 跳过、latest-only 输入和失败拒绝，不能通过排队完成陈旧规划。

本轮 rolling hint 只复用 jerk 初值，以当前明确 P/V/A 重新构造曲线，在当前最新预测下重新完整验收。旧 hint 必须未过原 TTL、同 mission/clock generation/frame/source，且新 prediction source epoch 严格更新；拒绝即清空，不延长旧 deadline。它不是上一条已接纳轨迹，不替代实际交接 P/V/A。冻结回放每轮间隔超出 125 ms，warm 命中为 0，cold/rolling 数值相同；据此不能声称暖启动已降低延迟。

真正的滚动执行需要未来 Tracker 在实际交接时刻采样上一条已接纳轨迹的期望 P/V/A/yaw/yaw-rate，使用同一 epoch 新解，并只执行前段。需要短时控制保持安全模型、失效回退、预测不确定性/实际姿态裕度和明确 ACK；不能把旧轨迹未来终点当所有新解起点。当前仅进行影子滚动重规划，尚未实现或验证闭环滚动优化。

## 7. 交接协议和 P4 条件

详见 [p32_follow_handover_protocol.md](p32_follow_handover_protocol.md)。纯算法 contract/dry-run 已覆盖原始输入过期、预测覆盖、mission/clock generation、候选状态、采样 P/V/A、yaw wrapping/角速连续、旧轨迹状态、最新预测重验和 receiver ACK。180 个合成 lead/runtime/jitter case 区分无旧轨迹、无 holding 授权、测试专用 holding；所有实际 accepted_by_tracker 均为 false。

当前实际 Tracker 不提供 FOLLOW 已接纳期望轨迹与持有安全依据，因此在线 dry-run 为 `OLD_TRAJECTORY_UNAVAILABLE`。150 ms lead 与 125 ms 输入 TTL 的矛盾没有通过扩龄、重写 stamp 或伪造 accepted 解决。预测覆盖和采样安全都不能替代持有授权；采样 FOV 不等于连续时间证明，规划姿态不等于 PX4 实测姿态。

是否进入 P4：

1. 纯批量初始化显著加速，但现场合作式 deadline/GIL 尾部仍不满足硬实时保证，不能称所有自适应路径足够快。
2. F3 能在原 125 ms 门限下合法研究发布，精调 18/146；成功率仍有限。
3. Q1 初值调整在同输入消融改善 jerk 可行性；受整体边界影响，未证明消除所有起点违约。
4. 125 ms TTL、上游观测到达年龄、paired navigation 年龄与回调排队仍是瓶颈；纯 BCTRA 计算仅约几毫秒。
5. 有具体可实施的交接接口设计，但真实 Tracker 的期望状态/ACK/holding 安全模型尚缺失。
6. 原 FOLLOW 已执行指标与影子轨迹质量仍不可作为公平闭环优劣比较。
7. 可以进入 P4 的接口/安全保持与短时域研究准备；不能现在自动接管 FOLLOW。需要先补受控交接、姿态与不确定性误差、失效回退和闭环稳定性，再另行授权执行验证。

## 8. 仓库和证据管理

起始发现 `data/experiments` 已跟踪约 2400 文件，最近提交含约 2397 实验文件、140 万新增行，Git pack 约 600.79 MiB；若需清理应另立任务：完整归档并校验、逐文件恢复清单、确认保留入口、选择性移出索引和忽略配置。未执行 git rm/reset/clean、历史重写或全量暂存。R 内自有 `.gitignore` 和 COLCON_IGNORE 只隔离本轮证据，避免源码快照被当作重复 colcon package。

新 PX4 参数备份仍在 `data/experiments/px4_parameter_backups/`，保留未跟踪状态；不会自动提交或删除。ULog、配置、源码版本、失败日志和所有回放均纳入本轮 SHA256 清单，具体路径以 R 的 evidence manifest 为准。P3.1 冻结 445 项校验已核实未改变。

## 9. 本轮验证

新增测试：批量 P1/观测点一致性、旋转/非法输入、可行优先/候选上限/提前停止、raw freshness/时钟、起点 PVA/解析 jerk/Q/T、影子完整准入、交接状态机/ACK/holding、float32 recorder、BCTRA 批量/标量/已知 CV/非法时域、短曲线已知 P/V/A/jerk、短时域、错误 yaw 和 rolling 缓存失效。保留全部既有测试。

追加功能后的本轮结果：algorithm **253 passed、3 deselected**；related **406 passed**；完整包级 pytest **1281 passed、1 skipped**（现有 2 个 SelectableGroups 警告）。新增预测相关 54 passed。项目范围 flake8 在 ROS 环境及根配置下通过。构建用 `scripts/build_workspace.sh --packages-select uav_control uav_usv_bringup`，2 包成功；`ros2 pkg prefix uav_control` 为新工作区。shell、Python、launch 参数、diff 与受保护范围检查见 R 的最终记录。

过程失败均保留：初始预期红测试；一次未 source ROS 导致接口导入失败；新增 docstring D401/D213 与缩进/99 字符 style 失败修复后完整复跑；裸 flake8 命令未在 PATH 后改用 ROS 环境 `python3 -m flake8`，未格式化历史工具。没有引用历史 1230 测试结果冒充本轮结果。未执行 Y 截获、Tracker 接管、全真实艇体识别、连续时间安全证明或真实硬件实验。

## 10. 追加现场结果与证据限制

`f3_detailed`（120 s 加 owned cleanup）：完成 85 周期、核心可行 11、合法研究发布 11；初始化 P50/P95/P99=5.12/31.67/34.43 ms，整周期 42.97/68.02/76.86 ms。

详细诊断和每 10 周期 cProfile 会增加开销；与精调轻日志相比初始化尾部 31.67 vs 28.45 ms、整周期 68.02 vs 62.58 ms，但不同实际轮次也有调度/场景差异，不能把全部差值归因于日志。逐函数现场 profile 保留在 p32_analysis.json。20 s CPU/RSS 采样窗口与会话退出重叠，含停止过程，标为不能用于稳态 CPU 对照；不以其较低均值声称节省算力。该轮 native 实际/理想场景 RMSE 0.468 m、P99 2.937 m，也不适合严格同场景控制效果比较。CLI node-info 查不到节点，失败证据保留，不能当发布者验证成功。

1929 对原图像戳的 observation/prediction 匹配，未经验移动时间：

| 阶段 | P50 ms | P95 ms | P99 ms |
|---|---:|---:|---:|
| image_to_received | 23.35 | 34.71 | 40.41 |
| received_to_processed | 7.28 | 17.15 | 19.57 |
| processed_to_publish | 0.04 | 0.08 | 0.11 |
| publish_to_generation | 1.87 | 3.18 | 3.98 |
| prediction_python_compute | 2.01 | 3.91 | 4.76 |
| image_to_prediction_receipt | 40.05 | 49.46 | 56.82 |

主要延迟来自采集→接收与因果定位处理/等待；该分段没有继续细分图像传输、pose-history 等待和视觉纯计算，因此不能断言其中全部是某一函数计算。下一步应加轻量因果等待/处理计时并检查源 topic/callback 到达，不能扩大等待门限、重写 epoch 或放宽 125 ms。新 BCTRA 计算定义已含消息构造，现场 total P95 3.91 ms。与旧整链路 P95 56.15 ms、新 51.90 ms 的不同轮次差值仅作观察，不作为严格同输入实时加速率。

只读 monitor 在 f3_detailed 退出时遭已 shutdown 的 ROS context，退出 traceback 保留；已用 rclpy.ok guard 修复，后续短时域会话使用新源码快照。raw recorder 采集结果仍完整保存。

### 1.2 s、5 Hz 短时域 F3 现场实验

`f3_short`（140 s 加 owned cleanup）：489 个完成周期，核心采样可行 328、时效合法发布 325。初始化 P50/P95/P99 1.47/3.50/5.56 ms，整周期 28.07/64.62/70.24 ms。状态分布 `{'DYNAMIC_INFEASIBLE': 72, 'NO_FRESHNESS_BUDGET': 74, 'SAMPLED_FEASIBLE': 328, 'DEADLINE_EXCEEDED': 15}`；规划后剩余 TTL 中位数 43.48 ms。328 个核心可行中 3 个被最终发布时效拒绝，均保留。

局部终点、时域和 5 Hz 请求率均不同于此前 2.4 s/1 Hz，不能宣称其成功比例可直接代表控制改善。初始化尾部显著缩短，但整周期 P95 没有较精调 2.4 s 的 62.58 ms 降低；完整验收、进程调度、原始导航新鲜度和 ROS 发布仍限制周期。warm hint 命中 0；未复用过期输入。

有效稳态 20 s 本次进程组 CPU 均值 286.87%，RSS 总和 1471.51 MiB；research 节点 CPU 51.90%、RSS 119.58 MiB。不能据此声称滚动规划节省 CPU，5 Hz 增加研究调用频率。实际艇体与理想轨迹 RMSE 0.106 m，P95/P99 0.179/0.190 m。只读 recorder 退出无前轮 context-shutdown 错误。

2275 对 observation/prediction，采集到只读预测接收 P50/P95/P99 33.97/44.82/88.64 ms；新 BCTRA+Python 消息构造 1.79/3.44/4.42 ms。research 看到的 603 条去重有效预测的采集→影子接收 P95 44.96 ms。实际 upstream 延迟和初始状态不完全相同，因此不用两轮百分位差声称严格端到端提速。

本轮短时域 CLI 第一次 DDS discovery 漏查了研究节点；之后加长 spin-time 的重查返回时会话已停止，节点/三个 PX4 topic 查询仍失败，原始失败日志保留。实际 P3.2 首次 F3 的三类发布者证据和本轮受保护 Tracker/PX4 文件哈希、研究节点相同订阅/发布实现仍支持控制隔离；不能把短时域轮的 CLI 失败当成实时发布者复核成功。

1391 项起始指纹最终审计：只改变明确授权的研究算法/测试脚本及追加授权的 3 个 predictor 文件，无非授权变更。6 份 ULog、12 份参数备份文件已复制到对应新 run 子目录并逐份校验，原文件不删除；最后所有本轮 owned 进程组非 zombie 存活数为 0。来源见 `raw_evidence_sources.json`，完整内容清单与 SHA256 见 R 根目录。

### 文件清单

授权源码/文档变化如下；PX4 自动生成的未跟踪备份另保留，不包含在功能修改清单。

- `docs/current/README.md`（修改）。
- `scripts/test_research_planners.sh`（修改）。
- `src/uav_control/uav_control/controllers/follow_research_shadow_node.py`（修改）。
- `src/uav_control/uav_control/guidance/adaptive_follow_initializer.py`（修改）。
- `src/uav_control/uav_control/guidance/follow_minco_optimizer.py`（修改）。
- `src/uav_control/uav_control/guidance/follow_problem.py`（修改）。
- `src/uav_control/uav_control/guidance/follow_reference.py`（修改）。
- `src/uav_control/uav_control/guidance/polynomial_extrema.py`（修改）。
- `src/uav_control/uav_control/tracking/maneuvering_target_predictor.py`（修改）。
- `src/uav_control/uav_control/tracking/target_prediction.py`（修改）。
- `src/uav_control/uav_control/tracking/target_predictor_node.py`（修改）。
- `docs/current/p32_follow_handover_protocol.md`（新增）。
- `docs/current/p32_realtime_follow_validation.md`（新增）。
- `scripts/p32_follow_analysis.py`（新增）。
- `scripts/p32_follow_experiment.py`（新增）。
- `scripts/p32_latency_analysis.py`（新增）。
- `scripts/p32_shadow_monitor.py`（新增）。
- `src/uav_control/test/test_prediction_batch.py`（新增）。
- `src/uav_control/test/test_realtime_follow.py`（新增）。
- `src/uav_control/test/test_short_follow.py`（新增）。
- `src/uav_control/uav_control/controllers/realtime_follow_solver.py`（新增）。
- `src/uav_control/uav_control/controllers/short_follow_solver.py`（新增）。
- `src/uav_control/uav_control/guidance/camera_visibility_batch.py`（新增）。
- `src/uav_control/uav_control/guidance/follow_handover.py`（新增）。
- `src/uav_control/uav_control/guidance/start_jerk.py`（新增）。
- `src/uav_usv_bringup/config/p32_follow_research.yaml`（新增）。
- `src/uav_usv_bringup/config/p32_short_follow_research.yaml`（新增）。

Launch 参数检查初次未显式 ROS_LOG_DIR，尝试写只读 ~/.ros/log 失败；修正到 R/final_check_ros_logs 后重跑，未修改 launch 配置。
