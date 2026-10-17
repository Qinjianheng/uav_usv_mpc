# P4.7 H1.6 候选封版决定

封版时状态：**OFFLINE_CANDIDATE_PASS / SITL_NOT_YET_VALIDATED / PRODUCTION_NOT_QUALIFIED**。
唯一研究候选为 `p47_follow_guided + qva + refinement=false + adaptive + H1.6s`，后备为 P4.4/D。
此结论只限定 H1.6 离线参考；旧全时域 qualification 仍为 false，不能授权生产接管。

2026-10-12 对本机 `data/experiments/20261010_p47_follow_guided/comparison/formal_20261012_2/`
及 `final_evidence_20261012/` 逐项校验 227 / 190 项 SHA。173 项历史源码与当前源码逐项比较；
其中 3 项只有 docstring 差异，执行 AST 一致。没有重新求解矩阵。

| 指标 | F2 | F3 |
|---|---:|---:|
| P44/D 单请求总成功 | 109/118 | 450/489 |
| C 独立成功 | 87 | 323 |
| C P44 后备成功 | 22（触发31） | 127（触发163） |
| C 最终总成功 | 109/118 | 450/489 |
| 同窗稳态样本数 | 86 | 376 |
| A → C 水平 RMSE，m | 0.088567 → 0.065157 | 0.504651 → 0.431856 |
| 理想滚动覆盖 | 100%，106.244s | 100%，100.899s |
| C 单请求/滚动有效曲线复验数 | 109+106 | 450+464 |
| 原完整动力学、姿态、整球 FOV 复验失败 | 0 | 0 |
| 稳态参考高度 RMSE，m | 5.06e-12 | 1.66e-7 |
| 真实求解 wall P95 / P99，ms（两人口取较差者） | 9.771 / 10.391 | 12.633 / 14.609 |
| 最小余量，ms（原输入 TTL −真实求解 wall−20ms） | 1.780 | 0.835 |

同窗按原始输入行 SHA、行号和执行 epoch 对齐，连续 episode 年龄≥20s，A/C/D 共用人口。
计时包含实际求解与 fallback，不含事后拟合分析；研究求解预算 25ms。
余量是求解端推算，**尚无 P5 Tracker 实测接收余量**。新门禁只允许开展有条件研究实验；
消息时效、完整接收验收、真实 ACK、执行授权及 450ms lease 仍须在线逐次成立。

负结果：H1.2/F2 稳态水平 RMSE 为 0.089260→0.187594m，旧门禁第二项继续失败。
即使 H1.6，F2 相对速度 RMSE 也由 0.063937 增至 0.085164m/s；FOV 裕度较 A 缩小。
H1.6 单请求+滚动有效曲线中的最小整球 FOV 裕度为 F2 0.122666rad、F3 0.097532rad。
不得把水平误差改善推广成全指标改善或 PX4 实测收益。

复核工具：`scripts/p5_release.py`。轻量证据入口：[release.json](audit/p5_20261012/release.json)、
[matched_samples.jsonl](audit/p5_20261012/matched_samples.jsonl)、[SHA256SUMS](audit/p5_20261012/SHA256SUMS)。
本机完整复核输出：`data/experiments/20261012_p5_release_03/`；首次适配失败 `_01` 和中间复核 `_02` 保留。
完整来源目录与逐文件 SHA、原始输入路径/行指纹均在 release.json，GB 级 JSONL 继续保留本机且忽略。

P5后续实际一次F2因新增健康记录器错误在研究Planner启动前中止，没有MINCO接管证据；未重试、未执行F3。实际结论见 [P5闭环报告](p5_minco_closed_loop_validation.md)，离线封版不随工具中止变成真实执行通过。

后续用户授权的 [修复后复验](p5_revalidation_20261012.md) 已实际加载研究 Planner，
但原始输入超过125ms触发停止，ACTIVE仍为0；F3未准入，生产资格及持续闭环资格仍未通过。
