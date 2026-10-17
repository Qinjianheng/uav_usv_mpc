# P5.2 故障修复与复验

> 授权续做更新：P52第1轮无ACTIVE；第2轮35.47%/最长5.35s；第3轮27.41%/最长6.35s，均未达到80%，F3未准入。三轮已封存。本文以下保留原离线交付的当时状态；最新完整窗口、修复与待授权复验见 [P5.3验收](p53_f2_validation.md)。


本轮仅处理已证实的执行资格、身份和供应缺陷。原始RED和GREEN输出均在新offline目录，
最终测试/构建及独立审查结果见 `audit/p52_follow_minco/verification.json`。

| 原症状与根因 | 最小修复 | RED→GREEN与范围 | 剩余限制 |
|---|---|---|---|
| 配置门槛与固定settled门槛不一致 | follow_settling显式接收AdmissionConfig，Tracker和health共用配置 | tight/default阈值及实际config读取 | 不改变默认阈值 |
| 最新KF失效却选旧healthy行 | 最新无效/非有限/不可能source顺序直接拒绝；callback立即更新资格 | 无效KF在control tick间重置或撤销 | 不能证明上游供应率 |
| boot/mission/generation改变仍短暂保留旧owner | Tracker观察到身份变化即撤销；Planner保留身份高水位和退休boot | 身份变化/旧心跳/未来心跳覆盖 | 实际DDS延迟仍待F2 |
| 最大进入次数后仍无意义探测 | Receiver心跳明确FOLLOW_ENTRY_LIMIT_REACHED，Planner遵循 | max-entry原因与probe抑制 | 保留最大3次 |
| old active失效但future child滞留 | 同tick取消其子pending，按原active记录loss | orphan_red复现→定向通过 | 不能保留过期parent |
| 45ms初估和失败P44搜索抬高估计导致合法请求被跳过 | warm estimate15ms；失败搜索另记；不可行reserve不饿死FOLLOW | budget_red 3fail；同原第三请求跳过→22.583ms合法 | 前两条仍拒绝，非在线尾延迟保证 |
| 本轮尚无长序列恢复证据 | 独立P52 production replay，保留旧P51夹具 | >20接替、2次失败、输入中断、性能/前缀loss、重入 | 非SITL、无ROS/PX4命令发布 |
| 旧资格不能覆盖本轮源码 | 独立P52入口/root/hash/当前验证与独立review；F2/F3分开授权 | entry_red缺模块→entry_green通过；旧P51资格拒绝 | 实际试验须另授权 |

修改生产文件：`guidance/follow_admission.py`、`guidance/follow_minco_execution.py`、
`control/trajectory_tracker_node.py`、`controllers/p4_follow_planner_node.py`、
`controllers/follow_guided_feasible.py`、`scripts/p5_live_health.py`、
`scripts/p4_shadow_monitor.py`、`FollowReceiverState.msg`。
新增P52 offline analysis/replay/SITL入口及对应4组测试；更新旧预算回归的“小预算”用例
从32ms到20ms，以继续验证新的15+12ms成本策略的P44优先分支。
没有改优化目标、FOV计算、动力学验收、原FOLLOW控制律、导航epoch或真值边界。
消息新增字符串字段需重新构建interfaces和两端，不支持混用旧生成消息运行。

## 审计决定

- 保持master，不commit/push；保留ledger和原始证据便于继续。新资格只标离线准备。
- 15ms仅是预热后工作选择估计；不宣称P95，代价是负载高时仍可能浪费工作并截止拒绝。
- 失败搜索成本与成功候选分开；实际时钟仍可拒绝整轮超时，不能伪造合法输出。
- 独立P52夹具避免改写P51夹具/结果；少量测试脚本重复换取历史证据稳定。
- 连续两次失败不强制假设必然回退；若已有合法pending可接替就继续，否则按期回退。

## 未解决的实测问题

实际FOLLOW稳定时间、接管后25s占比/最长连续段、合法后继尾延迟、高度漂移、参考跟踪RMSE、
完整目标FOV、安全回退与最终PX4命令尚未有本轮数据。CPU/executor因果未确证。
最小下一步是用户授权一次新F2；失败后读取同轮日志继续离线修复，禁止未经授权重复或转F3。

## 独立审查后的修复

单次只读独立审查发现1项Important、无Critical/Minor：FOLLOW一次慢求解抬高成本后，
后续被跳过的路径永远无法更新估计。新增“实际受控时钟超时→恢复正常成本”及后备
成功成本离群值测试，两项先失败；修复后相关52项通过。被预算跳过的FOLLOW/P44估计
逐请求乘0.8衰减至预热先验15/12ms；不增加请求数、不复用过期预测、不改变频率或截止。
这是工作调度恢复，不是新的性能上界。负载持续高时仍可能截止失败，真实尾延迟待F2。
修复后重新跑全量测试和四包构建；资格中的独立审查标记表示已审查且重要问题均被
RED→GREEN覆盖，不声称独立审查员再次复审了最终代码。

审查中暂未判断的最终哈希/构建由执行者核验；在线性能和方向反转实际表现保留未知；
116次未发布的上游唯一根因因缺字段保留unknown。这些是明确裁定，不是静默忽略。
