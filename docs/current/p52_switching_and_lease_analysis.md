# P5.2 切换、输入时效和 lease 分析

## 历史实际人口

沿用并重新核验 `audit/p51_follow_minco/historical_timing.json` 的原日志哈希。
四轮旧 P4.5 共 131 次 MINCO→FOLLOW：116 次 active lease 内没有后继 proposal，
9 次后继被拒，6 次 ACCEPTED 后错过 start；直接触发均关联原 lease 到期。
proposal 间隔 P95 约 1.03–1.50s；publish→ACTIVE P95 153–158ms，最大169.609ms。
这些是旧运行数据，不能代表 P5.2 已改善。

“未发布”只是下游事实。旧日志不能把全部116次进一步唯一分摊为缺预测、过期、未调度、
worker忙、求解超时/不可行、后备失败、重验失败；缺字段保留 unknown，不能用计数猜根因。
同输入本轮确认了一种上游供应缺陷：成功成本估计被失败搜索污染，造成预算饥饿。

## 134.604ms 快照分解

| 物理时间段 | ms |
|---|---:|
| 图像采集→Localizer收到 | 21.151 |
| Localizer处理 | 5.142 |
| 处理后→KF | 0.872 |
| KF→BCTRA生成 | 0.604 |
| 生成→监视器收到 | 3.049 |
| 收到→健康快照读取 | 103.786 |

快照年龄包含监视器持有间隔；下一帧采集→收到本身88.100ms，说明也存在上游交付空隙。
不能仅凭该快照断言 Planner 阻塞 executor 或 CPU 不足。
本轮未用重写 epoch、外推、等待阈值放宽或旧目标缓存规避时效。

## 三条原请求，同输入修复前后

实际 perf_counter 计时映射到请求的原 ROS now；不混减时钟域。
预热只含固定H1.6/qva线性映射，不计入每次请求且单独记录构造耗时。
原 TTL 余量约52.576/68.519/59.293ms；扣原20ms发布保留后，
计算预算约32.576/48.519/39.293ms，入口记账另有微秒级开销。

| 请求 | 修复前 guard | 修复后 guard | 修复后墙钟 ms |
|---|---|---|---:|
| 1 | DEADLINE_EXCEEDED | DEADLINE_EXCEEDED | 34.403 |
| 2 | VISIBILITY_INFEASIBLE | VISIBILITY_INFEASIBLE | 47.638 |
| 3 | NO_FRESHNESS_BUDGET | SAMPLED_FEASIBLE（P44） | 22.583 |

第三条同轮不带 guard 的结果也合法，证明原预算调度抑制了实际可求出的候选。
修复把预热后 FOLLOW 初始估计从45ms改为15ms；失败 P44 全搜索成本另记，
不永久抬高成功候选估计；不可能装入的 P44 预算不阻止可能装入的 FOLLOW。
实际截止检查、完整轨迹验收及接收端独立验收始终保留。
15ms是工作选择估计，不是实时上界；前两条启动几何仍不适合接管，新的5s稳态门禁会抑制执行。
三条样本不能给P50/P95/P99或在线超时率改善结论。

独立审查后的最终串行复算（request_replay_final_serial）仍保持同样三个结论，
guard墙钟分别34.953/48.447/22.975ms。另一次与构建/测试并行的复算
（request_replay_final）分别35.198/57.854/34.067ms，第二条由不可行变为截止拒绝。
该并行轮完整保留，不作为性能改进证据；它也说明尾延迟仍未知，不能把15ms先验当保证。
审查还修复了慢FOLLOW成本估计无法恢复的问题：跳过的成本逐请求衰减至预热先验，
不新增请求、提高频率或延长实际截止；两项先失败回归及最终全量测试通过。

## 接替与恢复

正常保持5Hz、单worker无队列；失败时仅对真实ACTIVE parent有一次有界提前重试，
每次重新取最新有效预测。pending等待真实ACK；真实ACTIVE才成为继承parent。
继承检查P/V/A/yaw，启动前独立前缀检查；不能延长旧450ms授权。
当前测试保留一次失败继续执行旧轨迹、无后继到期回退、非法parent/START_MISSED等原P5.1覆盖。
新增原parent前缀失效时立刻取消尚未启动的子轨迹，避免孤儿pending推迟恢复。

| P5.2离线故障 | 后继ACTIVE | 回退 | 直接原因 | 首次重入延迟 |
|---|---:|---:|---|---:|
| 无故障 | 71 | 0 | 无 | 无 |
| 连续两次求解失败 | 70 | 0 | 旧lease/已排队后继合法 | 无 |
| 150ms输入中断 | 31 | 1 | EXECUTION_CONTEXT_INVALID | 7.75s |
| 持续性能偏差 | 32 | 1 | PERFORMANCE_DEVIATION | 7.75s |
| 前缀失效 | 31 | 1 | LATEST_PREFIX_INVALID | 7.75s |
| 每tick新预测版本 | 71 | 0 | 556次当前几何前缀重验 | 无 |

以上为解析共速、虚拟时钟夹具，时序事件保存在新 events.jsonl；实际耗时另记。
没有真实发布、机体动力学或CPU调度，不能包装为本轮实际控制占比/供应率。

## 新实际轮的归因入口

新轮监视器记录 dispatch、worker timing、full revalidation、proposal、receive/ACK、
execution_debug和最终Tracker command，按receiver/planner boot、generation、mission、plan关联。
缺预测/输入过期/未调度/worker忙/超时/不可行/P44失败/重验失败/未发布/拒绝/parent/
START_MISSED/先到期须分别保留；没有该轮数据前，各原因次数、持续时间和实际影响为未知。
FOLLOW起点/稳定/首ACTIVE、各fallback至重入的持续时间和未完成pending应分别报告。
