# P4.7 新分析与冻结输入研究

本目录是 P4.7 的唯一新增根目录；后续重跑只写入新子目录。没有新增 SITL，
原始 P4.5/P4.6 输入与 `data/baselines/20261006_pre_mpc/` 保持只读。

正式结论：F3 可行率和长期误差改善，但 F2/H1.2 同窗长期误差退化；
第五阶段因第二门未通过而跳过。`final_evidence_20261012/qualification.json`
保留否决原因；当前没有实际 SITL attempt claim。

| 路径 | 角色与使用范围 |
|---|---|
| `execution_physical/revision_02/` | P4.5 四轮 F2 的正式同 native epoch 误差分解、ULog 对照与参考高度机制；同级首版保留 |
| `execution_timeline/` | P4.5 owner/ACK/lease/prefix/parent 逐事件回退时间线 |
| `derivative_diagnosis/` | 历史 162/112 hJ 人口与终端 A/VA/QVA/Q/T/yaw 反事实；顶层为修正积分后的正式分析 |
| `derivative_diagnosis/legacy_reported_attempt/` | 发现旧 `control_effort` 积分报告问题前的旧分析，保留且不用于正式能量结论 |
| `rollout_performance/full_matrix_01/` | 1214 个请求×H、原 NumPy10/scalar5/10/20 ms 共 4856 行，数值和耗时对照 |
| `comparison/smoke_20261012_1/` | 首版快速冒烟，非正式全量结论 |
| `comparison/formal_20261012_1/` | 首版全目标加权初始化失败消融，完整 104 配置；不可用新版源码直接冒充复现 |
| `comparison/formal_20261012_2/` | 改用有界精确 jerk 能量初始化后的完整矩阵；每次修正分别保留边界、优化及最终选中候选 |
| `final_evidence_20261012/` | 历史人口见证、BVLS/KKT 核验、同窗摘要、五门判定、173 源码快照及逐文件变更清单 |
| `verification_20261012/` | 实跑全量测试、四包构建、消息/launch/shell、来源与 SHA 校验日志 |

主报告：`docs/current/p47_follow_guided_feasibility.md`。
实际执行归因：`docs/current/p47_execution_root_cause.md`。
导数归因：`docs/current/p47_minco_closed_loop_diagnosis.md`。

各正式包有独立 `SHA256SUMS`，必须从所属目录执行 `sha256sum -c SHA256SUMS`。
来源文件及行号/行 hash 位于各 manifest；分析不重写原始时间戳、不外推、不补造请求。
每个 `*_analysis.jsonl` 与同名原始结果 JSONL 逐行对应，诊断不计入求解器时间。

当前 HEAD 在执行期间由工作区外部操作从起始 `353c3e1` 变为 `741000c`；
第二次正式矩阵以 `741000c` 加工作树 SHA256 记录实际源码，不把 HEAD 当作完整源码证明。
本任务没有自动 commit/push。
