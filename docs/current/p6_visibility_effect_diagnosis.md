# P6 可见性与观察点效果归因（2026-10-15）

当前证据支持 **IMM 的 0.45 s 离散执行前缀有可行的理想 MINCO 曲线**，但不支持全 1.6 s 鲁棒、真实相机安全或自适应观察点收益。固定 5 m 参考并非没有其他候选可用；在正常 IMM 样本中，它同时满足短窗包络且软代价最低。BCTRA 增强的主要问题是短窗包络已经溢出，而慢速观察点过渡无法在该租约内移到静态评分所假设的远处。

本页只新增离线研究；没有更改运行时、相机数学、风险参数、控制权或原硬门禁。实际基础 F2/F3、IMM 主链和后续 A/B/C/D 资格仍由根任务独立处理。

## 输入、输出与复算

- 冻结输入：[ablation_02](../../data/experiments/20261014_p6_development_01/visibility/ablation_02/)，475 个原尝试全部保留；`attempts.jsonl` SHA256 为 `191e51d17e23f56a14b2b21cb24d9e2c43eff08bfe5321657a2aa320b9a4d191`。
- 校准：[prediction_04](../../data/experiments/20261014_p6_development_01/prediction_04/)，训练 seed 127、heldout 3731；原消融 seed 1913。使用现有训练所得数值，未在 heldout 上拟合。
- 原曲线归因：[visibility_replay_04/analysis.json](../../data/experiments/20261015_p6_stabilization_01/visibility_replay_04/analysis.json)。它没有重跑 solver，也没有改协方差。
- 新探针：[reference_probe_03](../../data/experiments/20261015_p61_visibility_study_01/reference_probe_03/)，`protocol.json` 在计算候选前写出；12 个固定场景键，每键 9 个 prepare 时刻瞬时参考、9 个执行时刻瞬时参考、9 个有界过渡参考，共 324 个纯参考窗口。
- [研究脚本](../../scripts/p6_visibility_effect_study.py)及[边界测试](../../src/uav_control/test/test_p6_visibility_effect_study.py)。每个条件模型的概率、逐点裕度及失败原因保留在 `candidate_probes.json`，不丢弃低概率分支。

12 个键预先固定为五模式 24 s 的 C/D 正常条件，以及 figure_eight/24 s/D 的原侧移 2 m、原 yaw +0.25 rad 条件。探针从静止 `d=5 m,beta=0` 的观察点状态开始，不重跑消融的 1.6 s 选择历史。它刻画候选机制，并非复现 C 既有运动中的 selector 状态。所有新参考仅评价理想几何，`executable_minco_checked=false`、`actual_attitude_checked=false`。

`reference_probe_01` 保存离线 wrapper 的错误：误把 `FollowProblem.reference()` 第三返回 yaw 当成 acceleration，触发 `INVALID_BATCH_SHAPE`；修正后新增回归测试，失败目录与源码快照保留。`reference_probe_02` 仅在保存的未来执行 epoch 上比较瞬时参考；`reference_probe_03` 显式分开 prepare 与未来执行 epoch，作为本文主证据。两次有效输出均确认所记录源码在运行前后 SHA 未变；不根据并发执行耗时形成性能结论。

复算到新的非空目录之外：

```bash
cd /home/qin/data/uav_usv_mpc
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/p6_visibility_effect_study.py \
  --ablation data/experiments/20261014_p6_development_01/visibility/ablation_02 \
  --calibration data/experiments/20261014_p6_development_01/prediction_04 \
  --replay data/experiments/20261015_p6_stabilization_01/visibility_replay_04 \
  --output data/experiments/NEW_VISIBILITY_DIAGNOSIS
```

新目录保存输入 SHA、全部 guidance/controller 源码字节快照和 `SHA256SUMS`。最终研究脚本的纯格式整理不回写旧快照；精确依赖以其 `source_snapshot` 及 `protocol.json` 的指纹为准，复算时应先核对或在隔离包中恢复对应版本。原 `ablation_02` 的 adaptive/robust 数学源码 SHA 与现有文件一致；P6 runtime 与 batch 相机模块随根任务工程修复已有独立版本，本文不将新探针指纹冒充旧消融指纹。

## 原曲线是否存在可执行候选

原 475 个窗口均通过原名义门禁；这里的“可执行”仅指离线 solver 及完整名义几何/动力学检查，全部 `accepted_by_tracker=false`。并没有本轮真实 Receiver 或 PX4 资格。

| 正常条件，每组 25 个 | 全 1.6 s/配置窗口包络通过 | 0.45 s 租约前缀包络通过 | 原名义门禁通过 |
|---|---:|---:|---:|
| C：BCTRA + robust/adaptive | 0/25 | 0/25 | 25/25 |
| D：IMM + robust/adaptive | 0/25 | 25/25 | 25/25 |

D 全窗第一次失败在执行后 0.50–0.85 s，所有条件模型中心名义几何仍通过；最大条件位置 sigma 在 1.6 s 末端可达 2.883 m。D 前缀全部模型的最小剩余垂直裕度仍有 0.02246 rad，水平裕度最小 0.32070 rad。C 前缀最小垂直裕度为 −0.18250 rad，不能作为鲁棒候选通过。

完整条件模型核对揭示的是**这个冻结曲线和该经验包络**的失败，不是“所有轨迹都无解”的证明。全窗失败可由增长协方差、分支差异及姿态工程包络共同产生；没有真实残差校准，不能单凭 nominal 中心可见就判定包络错误。

D 前缀的 25 条曲线与 D_fixed_profile 的 xyz/yaw/duration 系数逐项完全相同。因此这些样本已有合格的短窗理想曲线，但没有独立自适应收益。B→固定解析参考的约 7.14% 理想 5 m RMSE 下降来自参考分支及解析运动学前馈；不能归因于距离/侧角变化。C 相对 A 的原负结果保留，不能用“候选存在”替代效果评价。

## 为什么固定点会赢

当前 `P6Runtime.prepare()` 的 3×3 网格为距离 4.5/5/6 m、侧后角 −20/0/+20°。先在理想、已经位于该 d/beta 的参考上生成 P/V/A，并假设 yaw 正好朝目标；再将风险、参考偏差、加速度及 jerk 交给 selector。原默认软项为：

```text
risk = 100 max(0, 0.12 − minimum_margin)^2
preferred = 0.2 [(d−5)^2 + 10d(1−cos(beta))]
extra_error = 0.2 [(d−5)^2 + beta^2]
smooth = 0.2 [(d−current_d)^2 + 25(beta−current_beta)^2]
switch = 0.05（更换候选）
acceleration = 0.02 mean(|a|²)，jerk = 0.005 mean(|j|²)
另有 hysteresis=0.1，最小驻留0.5s；正在运动时保留当前动作。
```

距离偏差在 preferred 和额外 error 中重复计入，并非几何硬约束。从静止 5 m 出发，4.5 m/0 的非动力学代价为 0.20，6 m/0 为 0.65，5 m/±20° 为 1.28669；挑战者还必须抵消 0.1 的滞回。它们是保守任务偏好，不能仅凭没有切换就称为错误。

| 冻结runner的24 s正常IMM场景 | 5 m/0 prepare 最小裕度 rad | 5 m/0 总评分 | 6 m/0 总评分 | 选择 |
|---|---:|---:|---:|---|
| linear | 0.10038 | 0.03849 | 0.65002 | 5 m/0 |
| figure_eight | 0.10399 | 0.02654 | 0.65088 | 5 m/0 |
| variable_speed_eight | 0.10600 | 0.01961 | 0.65002 | 5 m/0 |
| variable_speed_s | 0.10723 | 0.01739 | 0.65105 | 5 m/0 |
| variable_speed_z | 0.11130 | 0.00970 | 0.65211 | 5 m/0 |

这五个 prepare 时刻静态网格均有 9/9 前缀包络通过。探针逐候选调用 kernel，原 runtime 则批量拼接候选；此处经验包络的 confidence scale 固定，采样数不改变几何结果，但风险配置指纹仍分别保留，不互称同一概率窗口。远处裕度虽提高，已有 5 m 的风险代价非常低，收益不足抵消偏离任务的代价；没有调整权重的必要证据。已有 figure_eight 侧移/yaw 压力的选择分数与正常条件相同，因为这里评价的是理想参考及朝向，不使用 UAV 实际侧移/初始 yaw 构造瞬时评分轨迹。最终 MINCO/Receiver 仍会检查实际边界；初选压力响应不应被误称为实际闭环抗扰动。

## 静态候选与真正过渡不是同一条轨迹

候选打分后，selector 才创建限距离变化率 0.15 m/s、限角速度 0.05 rad/s 的 C2 五次动作。现有动作数学与限幅没有修改。

| 从静止 5 m/0 到目标 | 原 profile 持续时间 | 动作后 0.45 s | 动作后 0.60 s（约包含原0.15s执行lead） |
|---|---:|---|---|
| 4.5 m/0 | 6.333 s | d=4.998392 m | d=4.996330 m |
| 6 m/0 | 12.667 s | d=5.000425 m | d=5.000989 m |
| 5 m/+20° | 13.265 s | beta=0.000129 rad | beta=0.000302 rad |

在五个 24 s BCTRA 正常键中，静态 prepare 网格每键都有 3/9 通过；当前软评分均希望去 6 m/0。若把各个目标变为从同一 5 m 初值出发的原限幅 profile，并在保存的未来执行 epoch 查询 33 点，**每键 0/9 前缀通过**。静态“远处安全”不能承诺当前租约能安全移过去。增加风险权重也不能突破此可达性限制。

原 frozen ablation runner 在 prepare 之后才做 `.15 s` future boundary 投影；该离线初选与最终检查的预测时域相差约 0.15 s。五个 IMM 键的 5 m 静态裕度由 prepare 时刻的 0.100–0.111 rad 降至未来执行时刻的 0.065–0.075 rad，总评分变为0.208–0.304，而6m仍约0.650–0.652，固定点结论不变。**这不是已确认的在线epoch错误**：`FollowResearchShadowNode` 的 factory 通过 `partial(make_shadow_request, execution_lead=.15)` 先生成 future boundary，P4包装器再调用 P6 prepare。在线应继续保持该顺序；后续离线对照必须与它一致。最终 MINCO 的完整验证始终有权拒绝，不能宣称硬门禁被绕过。

这些过渡参考也使用理想姿态/yaw，不模拟 UAV 从侧移边界进入参考。IMM 九条过渡参考通过并不证明九条 MINCO 都能满足动力学或真实视觉条件。实际可行的证据仍限于上一节冻结曲线，不能把参考层通过补写成执行通过。

## 工程风险包络能否缩小

BCTRA 的 `covariance_scale=78.3861` 来自所有离散校准时域的最大值，最大项在 4.0 s。`empirical_scale=2.79548` 保持。因此原始 sigma 要乘约 24.75 才成为经验包络半径；C 前缀原始 sigma 最大约 0.08045 m，经验主轴半径可达约 1.99 m，另有目标半径和姿态误差支持界。原始 sigma 小不能解释成实际包络小。

已存在逐时域训练倍率较小，但校准 JSON 中 per_horizon 的 coverage 使用的是全局最大倍率。新脚本对**未修改的 heldout 残差**独立重算逐时域倍率覆盖：

| 源预测时域 s | 训练所得逐时域倍率 | 原全局倍率 heldout 覆盖 | 逐时域倍率 heldout 覆盖 |
|---|---:|---:|---:|
| 0.2 | 4.1379 | 100% | 93.36% |
| 0.5 | 4.7708 | 100% | 93.71% |
| 0.8 | 6.2879 | 100% | 93.15% |
| 1.2 | 10.4191 | 100% | 94.06% |
| 1.6 | 17.1817 | 99.93% | 94.76% |
| 2.4 | 39.8726 | 99.58% | 95.52% |
| 4.0 | 78.3861 | 95.10% | 95.10% |

每个 heldout 时域 1430 点，窗口重叠且共享运动路线，不能当作独立同分布置信区间。这个负结果禁止直接把较小倍率放进 C 以获得通过。它只支持另立**时间条件工程包络校准**：用新训练数据和足够密的真实源相对时域拟合，留独立测试，包含 RGB-D 深度/姿态偏差、丢帧簇和导航误差。不能用执行时间 t=.45 误索引源预测时域，须加入 execution_start−prediction_source。

IMM 的既有逐时域倍率均为 1；重算与全局覆盖相同。条件模型校准仅覆盖概率≥0.05 的分支，而几何检查全部分支；全模型几何检查并不补足低概率分支的统计证据。两次实际直线 pilot 可提供新的诊断输入，但直线覆盖率高或 Mahalanobis 值小仍不能用来校准所有转弯模式，更不能直接删除条件分支或缩矩阵。

## 建议的单一下一步与预注册资格

基础连续供应恢复后，建议首先修改**初选评分的轨迹来源**，保留现有 d/beta 网格、软权重、滞回、slew bounds 和包络数值：

1. 在真实选择 epoch 推进 incumbent 一次；从此时的同一 P/V/A 生成每个允许动作的原有 C2 profile。正在运动/驻留条件继续限制可切换动作。
2. 继续按与最终候选相同的执行 lead 查询 profile，并用目标源时域取完整 covariance/全部条件模型；在线factory的既有future顺序保留，离线runner与它对齐。不能用瞬时到达目标 d/beta 的姿态取代过渡。
3. 初选只排序、不授予安全资格；进入 MINCO 的候选仍通过原全1.6s名义 FOV/动力学和0.45s包络，Receiver独立检查、125ms时效和原非续期租约不变。记录每候选风险/成本、所选profile与最终MINCO风险差异。

这是有证据的模型一致性改进机会，**尚未实现，也没有改善真实可见性的结论**。不能保证慢速 profile 会在当前前缀获得新的适应收益。若在匹配压力下仍无可达的收益，应保留固定解析参考与 nominal fallback，而不是为了表现变化调大距离范围或压低任务代价。

后续协议见独立 [qualification_protocol.json](../../data/experiments/20261015_p61_visibility_study_01/qualification_protocol.json)，尚未执行：

- 先做原五模式24s正常 + figure_eight24s侧移2m/yaw0.25的7个固定键，比较 D0 固定解析参考、D1 当前自适应、D2 执行对齐C2评分，共21次离线solver；预测/cov/校准/初始导航、实际CPU freshness匹配，保留所有失败。D0与D2必须同解析前馈，隔离真正的 d/beta 效果。
- 当前冻结1913输入仅作开发归因。独立资格采用预注册 seed8753，不得用结果选seed/窗口；训练127/heldout3731不参与重新拟合。同类新运动、噪声及压力输入在策略间完全共享。
- 压力保持导航侧移2m和yaw+0.25rad的既有定义；后续实际图像/姿态压力应由真实误差分布支持，不能把理想camera瞬时旋转当执行yaw控制器。
- 通过名义基础F2/F3、真实RGB-D校准和同源shadow后，才进入相应独立P6.7固定60s闭环。实际失锁、P5/最低FOV裕度、图像中心偏差、观察点误差、加速度/jerk、MINCO占比/最长连续、fallback和P95/P99开销均匹配报告。P6-S单轮上限不因此重置。
- 原完整名义FOV、海面/物理界、125ms和450ms、唯一Tracker所有权保持。包络失败按显式配置保留nominal fallback并标无新增鲁棒保证；不能将前缀结论扩成全轨迹或全任务概率保证。
