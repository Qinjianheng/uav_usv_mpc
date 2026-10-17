# P6 实际 IMM 状态与协方差诊断

`scripts/p6_imm_actual_validation.py` 是只读日志分析工具，复用
`p6_actual_analysis.py` 的原始观测关联、IMM 身份与混合协方差核对、固定窗口和
`BoundedTruth`。它不拟合参数、不运行 ROS、不修改原始日志，也不生成主预测器资格。
原 KF、IMM、控制与 launch/config 未因本工具改变。

```bash
cd /home/qin/data/uav_usv_mpc
python3 scripts/p6_imm_actual_validation.py \
  data/experiments/20261015_p6_stabilization_01/pilot_A_linear \
  data/experiments/<new_analysis>/stabilization_diagnostic \
  --data-role diagnostic
```

输出必须使用全新、位于原 case 之外的目录。保存 `analysis.json`、
`forecast_validation.jsonl`、`source_validation.jsonl`、两份实际执行的 analyzer 源码快照及
递归 `SHA256SUMS`；报告同时绑定原 JSONL、case、原 summary、execution audit 与完整配置 SHA。
逐样本记录保留原 JSONL 行号。非有限的拒绝输入字段输出为 null，原始字节仍在源文件，
没有用有限替代值参与评分。畸形 JSON 行号、无效消息、严重误差、缺失诊断和截尾样本均保留。

## 时间和统计定义

cohort 固定为原 `first_minco_receipt` 到其后 60 秒；缺少该原始锚点直接拒绝分析。
receipt 只用于 cohort，误差评价使用未改写的 `source_stamp + horizon` 与真值 `stamp`。
真值接收时间截止于窗口终点，最大插值间隔 0.2 秒，禁止外推或借用停止后冻结。
时域固定为 0.2、0.5、0.8、1.2、1.6、2.4、4.0 秒。

对位置误差 `e=p_hat-p_truth` 分别计算：

| 诊断 | 协方差 | 95% 阈值 |
|---|---|---:|
| 水平 2D 椭圆 | `P[:2,:2]`，包含 xy 相关项 | χ²(2)=5.9914645471 |
| 空间 3D 椭球 | 完整 xyz 3×3 | χ²(3)=7.8147279033 |

以 Cholesky `P=L Lᵀ` 计算白化残差 `z=L⁻¹e`、`q=zᵀz` 和 `sqrt(q)`。
还保存各坐标边缘标准化残差 `e_i/sqrt(P_ii)`；它与白化残差不同。
奇异、非正定、非对称或非有限协方差明确拒绝，不加 jitter、不用伪逆。
协方差必须在已发布数组中有对应时域的原始样点，不对协方差作新插值。
IMM 的统计评分另要求原 lease、唯一预测身份、raw acquisition 和完整条件诊断匹配；
协议失败时仍保留可计算的位置误差，覆盖率项记录拒绝原因。
raw observation bounded join 仍沿用原 1 μs 边界。消息与 IMM diagnostics 的四个 epoch 和
完整相对时间数组则另按在线 `attach_model_diagnostics` 的 ROS 1 ns 网格严格核对。
这不会恢复 float 日志已经丢失的原始纳秒，也不会把两种容差混称“严格匹配”。

这是位置残差一致性诊断，不是 innovation NIS；没有把真值插值误差或参考不确定度加入 P。
BCTRA 的工程协方差也可得到数值覆盖率，但不会因此成为经独立数据校准的概率模型。
IMM 混合分布不一定 Gaussian。重复帧和重叠时域相关，因此不报告 IID 置信区间、整条
轨迹联合概率或连续时间安全保证。水平覆盖率不能替代垂向/3D 覆盖率。

## 初态、增量与 KF 比较

每条评分保存完整 xyz 向量，并核对以下代数分解：

```text
e_total = p_hat(h) - p_truth(h)
e_source = p_hat(0) - p_truth(0)
e_increment = [p_hat(h)-p_hat(0)] - [p_truth(h)-p_truth(0)]
e_total = e_source + e_increment
e_initial_cv = e_source + h * [v_hat(0)-v_truth(0)]
e_beyond_initial_cv = e_total - e_initial_cv
```

分量范数不能相加；increment 与 beyond-CV 仍包含潜在初态误差，不能解释成纯模型因果收益。
raw observation、原 KF 和 IMM sample0 在各自原始状态历元评价，报告 signed xyz bias、
各坐标 RMSE、水平/3D 位置与速度误差，避免只看误差模长掩盖持续轴偏差。

IMM 对 TrackerKF 的同刻比较将已记录 KF P/V 离线插值至 IMM 的原 source epoch，
最大 bracket 间隔仍为 0.2 秒，不外推、不改采集戳。这是 retrospective 状态比较：
不代表该 KF 样本在当时已可供在线使用，也不保证使用同一采集帧，更不是同初态纯模型对照。
阶段沿用既有 motion-truth 的因果标签；缺失或过旧为 unknown，模型概率按阶段单独汇总。

## 两轮 linear 实际结果

最终源码绑定的新报告位于
`data/experiments/20261015_p61_imm_validation_01/stabilization_diagnostic_02/` 与
`prior_diagnostic_02/`。两轮均为原 first-MINCO 起点后的固定 60 秒，全部阶段标签为 straight。
旧 v1 输出保留，只能作过程证据；最终资格相关统计引用 `_02` 的 exact-wire 结果。

| 项目 | 2026-10-15 stabilization | 2026-10-14 prior |
|---|---:|---:|
| 完整 IMM 预测/独立 acquisition | 1200 / 1200 | 1200 / 1200 |
| 原 1 μs analyzer 身份匹配 | 1200 | 1200 |
| 精确 ROS 1 ns 消息/诊断身份匹配 | 1200 | 946 |
| 精确身份拒绝 | 0 | 254 |
| IMM update+forecast+message P95 / P99 ms | 17.219 / 19.731 | 15.072 / 18.361 |
| 实际 MINCO 占比 | 15.473% | 86.882% |
| 最长连续 MINCO s | 4.367 | 49.456 |
| fallback / lease expiry | 3 / 3 | 1 / 0 |
| 原记录 f3_eligible | false | true |

prior 的 254 条精确身份失败与已知旧时间序列化问题一致；失败字段分别为 source 89、
generated 89、observation 96、valid_until 84，字段数量有重叠。它们的物理误差仍全部评分，
协方差覆盖项明确拒绝，不借原 1 μs 匹配掩盖当前在线 1 ns 契约失败。
prior 的旧 `f3_eligible=true` 是原记录事实，不能复用为当前版本或 IMM 主链资格。

stabilization 每个时域 1200 次 IMM 尝试；下表 N 为未来成熟且严格协议合格的样本数，
其余分别是 4/10/16/24/32/48/81 条 right-censored，没有补用停止后冻结。
原 BCTRA 与 IMM 的 source 时刻略有差别，以下为同固定窗口的完整链分布对照，
不能称为同采集帧、同初态的纯模型实验。

| h s | N IMM | BCTRA 水平 RMSE m | IMM 水平 RMSE m | IMM 2D / 3D 覆盖率 | 平均 q₂ |
|---:|---:|---:|---:|---:|---:|
| 0.2 | 1196 | 0.03633 | 0.03649 | 100% / 100% | 0.060424 |
| 0.5 | 1190 | 0.03869 | 0.03927 | 100% / 100% | 0.019154 |
| 0.8 | 1184 | 0.04287 | 0.04331 | 100% / 100% | 0.008868 |
| 1.2 | 1176 | 0.05256 | 0.05052 | 100% / 100% | 0.004447 |
| 1.6 | 1168 | 0.06818 | 0.05978 | 100% / 100% | 0.002784 |
| 2.4 | 1152 | 0.11798 | 0.08424 | 100% / 100% | 0.001517 |
| 4.0 | 1119 | 0.23687 | 0.15365 | 100% / 100% | 0.000711 |

在名义零均值 Gaussian 参考下 q₂ 的均值为 2。实际远小于该尺度，且随时域增长进一步下降，
支持“稳态直线 covariance 明显保守”的判断；100% 覆盖不是独立真实 95% 校准证明。
这也为后续排查 robust 过保守提供证据，但没有据此缩放 covariance、修改 Q/R 或调节风险。
prior 在严格协议合格的 942/936/931/925/918/907/883 个样本上，2D/3D 覆盖也均为 100%；
其 254 条协议失败完整保留，不能将这个条件子集扩展为全部 1200 条统计合格。

stabilization 的 h=1.6 s IMM 水平 source/increment/initial-CV/beyond-CV RMSE 分别为
0.03555/0.04312/0.05469/0.00981 m；h=4 s 为 0.03567/0.14545/0.10189/0.06926 m。
长时域增量逐渐超过 source error，仍不能把增量解释成独立纯运动模型误差。

| 当前状态，stabilization | signed xyz bias m | 水平位置 RMSE m | 3D 速度 RMSE m/s |
|---|---|---:|---:|
| raw observation，同采集历元 | +0.00976 / −0.02612 / −0.06560 | 0.03635 | 不记录 raw velocity |
| 原 KF，同原状态历元 | +0.00967 / −0.02612 / −0.06551 | 0.03540 | 0.12581 |
| IMM sample0，同 source 历元 | +0.00968 / −0.02612 / −0.06552 | 0.03534 | 0.10328 |

1200 个 retrospective 同刻 IMM−KF 差值的水平/3D 位置 RMSE 为 0.00186/0.00878 m，
水平/3D 速度差 RMSE 为 0.00721/0.02439 m/s。共享 signed bias 说明完整链当前位置误差
尚未被新滤波器消除，不能据此归因到某个外参或轴修正。没有在线真值纠偏或固定偏置补偿。
BCTRA 在 0.2 s 的水平覆盖为 100%，但 3D 只有 69.15%；二维指标会掩盖垂向不一致。

stabilization 的 CV/CTRV/CTRA 平均概率为 0.26801/0.40913/0.32286，变化范围仅
0.00282/0.00156/0.00206；1200/1200 条均由 CTRV 占优。prior 的均值和现象相近。
这不足以支持模型明确识别 straight，更没有变速、转弯增长/下降或反转阶段的实际证据。

## 资格与下一步

报告保留原 `existing_shadow_diagnostic` 与实际控制统计。现有 shadow diagnostic 检查
完成/健康/时长/P5.1/身份/owner/native 等，但不直接检查 MINCO fraction 或 lease expiry。
所以其 pass 不能替代供应阶段验收，也不授予 IMM 主链资格；本工具的
`qualification_generated=false` 与 `main_chain_eligible=false` 始终成立。

data role 只标记用途，不能凭 `calibration_train` 或 `heldout_validation` 字符串证明数据独立。
当前两轮 linear pilot 仅用于 diagnostic，不拟合 covariance scale、Q/R、偏置或时间补偿。
进入真实校准须先明确独立训练 run/seed、冻结方法与源码，再用另一次未参与选择的实际
验证 run；同轮真值不能反向调整该轮在线预测。

在供应/F2 门禁恢复前仅准备离线证据。后续最小独立步骤是健康 linear+BCTRA-primary+
IMM-shadow 固定 60 秒资格，随后按独立阶段推进 figure_eight 及三种变速模式。
当前 linear 数据不支持五模式实际结论、反转阶段模型识别结论、IMM 主链或 robust 实飞资格。

## 验证入口

```bash
cd /home/qin/data/uav_usv_mpc/src/uav_control
python3 -m pytest -q test/test_p6_imm_actual_validation.py
```

18 项纯测试通过，当前两文件 flake8 通过；没有在该独立诊断任务中运行构建或全套测试。
RED/GREEN、源码快照、逐行记录与 SHA 位于上述独立目录。
纯测试覆盖相关协方差白化、2D/3D 不同判定、奇异/负/非对称/NaN 拒绝、精确向量分解、
同 source epoch KF 插值、身份失败、无效/截尾分母、signed bias、阶段 unknown、
只读 case/新目录、数据用途、非有限拒绝字段序列化、239 ns 严格身份拒绝与畸形时间数组。
该测试通过仅证明离线分析行为。
