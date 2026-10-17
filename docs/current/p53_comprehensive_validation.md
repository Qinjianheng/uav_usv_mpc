# P5.3 全闭环修复与同配置验收

2026-10-13。用户授权全面检查、必要结构调整与自主仿真。诊断基准5a783ca，续做HEAD2467c8a；不自动提交或推送。
当前仍为BCTRA/MINCO/唯一Tracker速度控制，MPC尚未实现。

2026-10-14维护备注：本报告数值和原资格是历史证据。R1～R10共17个场景目录已逐文件校验后外置归档，R11完整成功对照仍在原路径；查看旧原始证据须先按[系统精简与复验](system_simplification.md)恢复。该入口也记录清理后的新实测，不把R11成功外推为时序稳定性保证。

## 修复内容

完整归因见[p53_comprehensive_diagnosis.md](p53_comprehensive_diagnosis.md)，涉及两篇用户提供文献的设计对照及证据边界。

1. F2成本估计：等待新预测时不能抹去近期真实求解耗时。最近8条有效候选的中位耗时及20%余量保护hint，
   按新图像epoch进度有界衰减；时钟回退清空，几何warm start清理保留纯CPU成本知识。
2. F3状态/资格：视觉锁、native/KF/预测时效、直接三维观测与物理恢复分别检查和报告。
   预测过期报告`PREDICTION_STALE`，不再误标成视觉丢失。v2审计缺少任一时效字段即拒绝。
3. 输入等待：仅已完成FOLLOW阶段锁定且最近就绪时，RGB仍锁但三维输入暂缺进入`INPUT_WAIT`；
   保持视觉yaw、有界刹车。最多沿用原可见性失联窗`target_lock_max_age*target_loss_frames=0.45s`，
   持续无三维输入恢复原RGB引导接近。该时限不延长任何MINCO输入或轨迹期限。
   起飞阶段的短暂锁定不建立FOLLOW历史；非FOLLOW在任何提前返回前清理历史。

本轮没有缩短5s稳定/2s冷却，也没有改变KF Q/R、BCTRA、MINCO数学、PX4增益、捕获球或真值隔离。
在线native/KF/预测125ms、lease450ms、激活50ms、直接3D原150ms与完整轨迹检查保留。
实际研究配置沿用R9：P47/C H1.6、qva、无refinement、adaptive、P44/D后备、水平加速度前馈开启，
相对速度准入1.8m/s/偏差退出2.5m/s；通用前馈默认仍关闭。

## R10已完成的验证及失败保留

| 轮次/场景 | 实际MINCO占比 | 最长连续 | 水平观察点RMSE | 高度RMSE | 回退 |
|---|---:|---:|---:|---:|---|
| R4 F2 | 100% | 24.950s | 0.0648m | 0.0088m | 0 |
| R9 F2 | 84.57% | 21.100s | 0.0954m | 0.0096m | 1 lease |
| R10 F2 | 100% | 25.000s | 0.0381m | 0.0092m | 0 |
| R4 F3 | 28.44% | 11.416s | 0.7405m | 0.0114m | 1 lease、2偏差 |
| R9 F3 | 65.53% | 29.335s | 0.7235m | 0.0112m | 1上下文失效、1 lease |
| R10 F3 | 未接管 | — | 不构造接管后指标 | — | Planner启动前停止 |

R10 F2完整25s包含全部owner，实际观测24.999937s，623个MINCO诊断/参考样本，121条活动计划，
参考跟踪3D RMSE0.02499m；完整目标FOV626帧全部通过，最小水平/竖直裕度0.7525/0.3092rad。
唯一Tracker发布者发现样本全部通过，违规0；ULog指定物理窗口98条PX4 setpoint均记录有限水平前馈。

R10 F3的直接停止原因是`HEALTH_INPUT_LOST`：快照采集时control/nav/attitude receipt年龄264/271/254ms，
超过观察器250ms。与此同时，FOLLOW开始后的64条诊断全部RGB可见、全部`INPUT_WAIT`，64条3D观测均无效，
最后有效KF source在停止时已旧4.939s。起飞锁定历史被错误继承，使初始接近停住，是本轮新增状态的确认回归；
不能把观察器停顿全部归因于该回归。新阶段判定和有界等待已补反例修复。

原始记录：`data/experiments/20261013_p53_closed_loop_10/`；F2分析：
`data/experiments/20261013_p53_comprehensive_validation_01/f2/analysis.json`；
F3失败诊断：`data/experiments/20261013_p53_acquisition_phase_01/diagnosis.json`。
原始失败均保留并校验，不覆盖或追认通过。

## 最新验证入口

阶段修复有起飞锁定/首FOLLOW接近、跨SAFE_RECOVERY/非法update返回清理、输入等待超时等RED→GREEN反例。
定向97 passed；全量1848 passed、1 skipped、3 warnings；四包构建、flake8、shell语法、Git空白检查通过。
所有物理误差包含固定窗口内全部owner，真值仅离线评价；不同实际运行不能视为固定输入的单因果消融。

## 最终R11同配置实际验收：F2、F3均通过

| 固定窗口指标 | F2：首接管后25s | F3：首接管后60s |
|---|---:|---:|
| 实际MINCO占比（相邻诊断积分，含全部回退） | 100% | 100% |
| 最长连续MINCO | 24.950s | 59.950s |
| 回退 / 审计违规 | 0 / 0 | 0 / 0 |
| 诊断覆盖率 | 100% | 100% |
| 水平后方5m观察点RMSE | 0.0546m | 0.6977m |
| 高度RMSE | 0.0090m | 0.0116m |
| 实际对执行参考3D RMSE | 0.0283m | 0.2310m |
| 完整目标FOV | 100% / 626帧 | 100% / 1502帧 |
| 最小水平 / 竖直FOV裕度 | 0.7473 / 0.2998rad | 0.2214 / 0.3106rad |
| 活动计划数 / 参考诊断数 | 120 / 619 | 286 / 1457 |
| 实测native水平加速度最大值 | 0.105m/s² | 2.569m/s² |
| 实测水平速度最大值 | 2.044m/s | 4.464m/s |

固定25/60s窗口全部保留，约50ms诊断边缘未积分，不声称每一瞬间均有独立诊断。
物理误差分别使用1250/3000条native UAV状态，参考误差使用436/999条同epoch可用记录；
不是只取成功片段。F3已正常完成远距离初始接近，原FOLLOW开始到首MINCO约18.14s。
F2原FOLLOW开始到首MINCO的精确时间见`bounded_stop.json`；未以影子可行率或ACK数替代实际控制权。

PX4 ULog指定物理窗口F2 101/101、F3 200/200条setpoint记录有限水平前馈；最终前馈幅值不超过3m/s²。
发现唯一Tracker发布者的58/95个图样本均通过，无冲突；两轮清理errors/survivors均为空。
原`execution_analysis.json`保留setpoint时间差计算的瞬时速度导数：密集回调时可超过3m/s²，
不能将其当作实测物理加速度或隐去。上述实测加速度来自原始native导航sample。

F3在转弯处仍有约0.70m的后方观察点误差，不宣称厘米级八字跟踪。
独立同epoch分解窗口中，规划观察点误差0.655m、实际参考跟踪误差0.240m；该分解窗口与完整60s不同。
后续若需更紧密的观察点跟随，应针对参考质量另立问题；当前确认的持续接管、初始接近卡住、
错误失锁归因及F2后继供应回退，已由新实跑覆盖。红球结果仍限于本仿真感知接口，不外推真实无标记USV。

### 当前证据与继续入口

- 原始场景与完整运行时配置：`data/experiments/20261013_p53_closed_loop_11/{f2_h16_01,f3_h16_01}/`。
- 最新源码资格、独立复核、1848项回归和构建：`docs/current/audit/p53_follow_acquisition_phase/`。
  前一项修复资格保留于`docs/current/audit/p53_comprehensive_flight_repair/`，两者均为运行时快照。
- 全窗口指标、限制检查与轨迹图：`data/experiments/20261013_p53_comprehensive_validation_02/`；
  F2 `f2/analysis.json`、F3 `f3_physical.json`、执行`execution_limits.json`、图`f3_plot/flight.png`。
- 普通启动入口仍为`./scripts/uav_lab.sh --no-build`，不能将通用默认前馈关闭的配置误称为本次P53配置。
  本次有界driver为`scripts/p53_minco_sitl.py`，round11的一次F2/F3已消费，勿复用或覆盖；新实验需独立目录和新资格。
- 无自动commit/push、无视频暂存、无冻结基线改写；原仓库保持只读。
