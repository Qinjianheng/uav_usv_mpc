# P5.3 以实际飞行效果为主的准入标定

最新同配置R11 F2/F3均完成，实际MINCO占比100%、零回退，见[全面修复与最终验收](p53_comprehensive_validation.md)。
以下保留R4至R8的阶段记录；不可用末尾旧“待启动”文字替代当前实测结论。

2026-10-13用户再次授权继续修改，明确允许适度放宽检查与指标、优先实际飞行效果。
本轮基础HEAD为5a783ca；F3先完成准入标定，再基于实飞结果修复执行前馈。原始失败和成功记录均保留。

## 已证实的阻断

F3原FOLLOW的212个本地同刻样本，水平误差最大0.5375m、高度误差最大0.0171m、目标持续可见，
相对目标中心速度差最大1.5154m/s。原准入要求速度差始终≤0.5m/s，故反复重置5s稳定计时；
19次SAMPLED_FEASIBLE没有发布到wire，原因均为FOLLOW_ENTRY_INHIBITED。

该速度差包含转弯保持后方观察点的必要运动：

`p_ref = p_target - 5 * heading`

`v_ref = v_target - 5 * heading_rate`

因此0.3rad/s转弯时，即使偏置点跟踪无误，UAV与目标中心也有1.5m/s的速度差。
现有指标不能把这部分全部解释为飞行跟踪误差。

## 本轮修改

仅P53研究round4采用相对速度准入1.8m/s、持续偏差退出2.5m/s。准入比实测正常转弯峰值
留约19%余量，退出保留滞环。5s稳定、3次合法探测、位置1m/退出1.5m、高度0.1m和所有
实际轨迹执行检查保持；KF、FOLLOW增益、MINCO数学与动力学限值不改。

- `scripts/p53_minco_sitl.py`：新增独立round4与`flight_admission()`，旧round1～3保留旧策略。
- `scripts/p5_minco_sitl.py`：condition记录实际加载的相对速度准入值。
- `scripts/p51_execution_audit.py`：从该轮真实flight.yaml读取资格配置，避免用默认值误报。
- `src/uav_control/test/test_p53_practical_admission.py`：独立转弯几何用例、5s/探测/失效及实际配置审计。

原始本地指标回放：旧策略在该10.6s片段收敛样本0，新策略111；这只是策略回放，不能代替
实际ACTIVE或飞行验证。诊断、RED→GREEN和全套1787 passed/1 skipped，以及新资格见
`data/experiments/20261013_p53_practical_admission_01/`和`docs/current/audit/p53_practical_admission/`。
生产src共441项SHA与此前构建版本相同，本轮仅研究driver、审计和测试改变。

## 实际验证

round4已完成，原始记录在`data/experiments/20261013_p53_closed_loop_04/`，分析在
`data/experiments/20261013_p53_practical_validation_01/`。

| 场景 | 实际MINCO占比 | 最长连续 | 水平RMSE | 高度RMSE | 完整目标FOV | 回退 |
|---|---:|---:|---:|---:|---:|---|
| F2，首接管后25s | 100%观测区间 | 24.950s | 0.0648m | 0.0088m | 100% / 626帧 | 0 |
| F3，首接管后60s | 28.44% | 11.416s | 0.7405m | 0.0114m | 100% / 1505帧 | 1次lease、2次偏差 |

所有物理指标包含窗口内原FOLLOW和MINCO两种owner；不能把整体误差只归功于MINCO。
F2固定25s的诊断边缘未积分约50ms，观测区间占比100%不等于每一瞬间都有诊断样本。
F3完成整个60s健康窗口，但连续接管不达预期。两次偏差退出前，实际参考跟踪误差约1.1m，
速度命令已有位置纠偏，PX4实际速度仍落后；加速度前馈始终NaN。FOV与高度未成为阻断。

## 水平前馈修复与round5

增加`follow_minco_acceleration_feedforward`显式开关（通用默认false，P53 round5开启）。
在唯一Tracker的名义MINCO ACTIVE分支，水平前馈采用
`a_ref(now) + K * (v_ref(native_epoch) - v_actual(native_epoch))`，与已有速度反馈的导数对应。
速度/加速度整形生效时改用最终输出速度的差分；速度达上限时去除朝外加速分量。
水平前馈幅值取模型和执行层加速度上限较小值；竖直为NaN，保留原高度/海面保护。
PX4仍处于velocity Offboard模式，现有PX4代码会把该前馈叠加到速度PID加速度输出。
不改FOLLOW增益、KF、优化器、捕获半径、原始时间戳或真值隔离。

独立复核发现的“前馈可能绕过速度整形”已用限速/限加速度/切向保留反例修复。
全套1798 passed、1 skipped、3 warnings，四包构建、flake8和shell语法通过。
本轮源码资格及原始测试见`docs/current/audit/p53_acceleration_feedforward/`和
`data/experiments/20261013_p53_feedforward_01/`。

round5的F2在Planner启动前被`HEALTH_INPUT_LOST`中止，未取得任何MINCO控制权。
快照交付延迟213ms，采集时control/nav/prediction/attitude年龄分别45/10/1/11ms；
控制仍locked FOLLOW，健康数据有效。保留整轮和校验清单。

实验监视器现将采集时receipt年龄≤250ms、快照交付≤250ms分设预算，读取时总年龄
可到500ms（vehicle_status为1s）。这是实验观察容忍度的放宽，不能报告为实时250ms
界限未变；控制器125ms freshness和lease保持。独立回放与8项故障拒绝复核通过。
新增6例反例；全套1804 passed/1 skipped。资格见`docs/current/audit/p53_observer_epoch/`。
round6在GUI world启动前失败，无X、无MINCO；原始记录保留。独立15s服务端诊断可建立世界，
增加显式`UAV_USV_GZ_SERVER_ONLY=1`（默认0）。round7已正常生成RGB/depth和PX4模型，
但X CLI文件创建时健康记录器仍在INIT队列，地面宽限先结束；停止时刻比X发布完成早3.695s。
这两轮均不能用来判断前馈飞行效果。

当前增加P53专用起飞同步：唯一会话发令器在独立健康快照持续500ms满足新鲜、GROUND_HOLD、
Offboard且未解锁前，不创建X命令文件。旧入口默认行为保留，不增加发布者，不放宽飞行门限。
新8例与全套1812 passed/1 skipped通过；round8将据此重启F2，通过后才进入F3。
