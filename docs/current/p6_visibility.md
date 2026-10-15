# P6.5 鲁棒完整目标几何与观察点参考

本页对应纯数学实现、独立离线测试和已接通的研究规划参考。它不代表已经完成真飞或 SITL 闭环验收。原 `camera_visibility.py` 与 `camera_visibility_batch.py` 保持不变；完整目标名义 FOV、前平面、near/far、动力学和 Tracker 独立验收仍有原权威。

## 输入与接口

实现位于 `src/uav_control/uav_control/guidance/robust_visibility.py`。

```python
report = evaluate_robust_visibility_batch(
    positions, rotations, targets, target_covariances,
    intrinsics, extrinsics, target_sphere, original_constraints, risk_config,
    model_positions=model_positions,
    model_covariances=model_covariances,
    model_probabilities=model_probabilities,
)
```

`positions`/`targets` 为 N×3 NED，`rotations` 为 N×3×3 完整 FRD→NED，世界位置协方差为 N×3×3 或 N×9。相机 FLU 平移、完整安装旋转及光学轴约定全部复用原几何。所有数据必须具有调用者已经核对的同一物理 epoch；模块没有 ROS、订阅、真值查询或时间戳修补。

IMM 条件位置为 M×N×3、条件协方差为 M×N×3×3，概率为 M 或 N×M。全部模型均检查，包括小于 `model_mass_floor` 的次要分支和零概率分支。阈值只设置 `major` 诊断标识，`unchecked_model_mass` 恒为零；概率和不为一时失败，不丢质量或暗中重新归一化。融合均值的名义检查与协方差检查仍然保留，不能用融合均值隐藏分叉。

`RobustVisibilityReport` 分别给出 `nominal_ok` 与 `robust_ok`、逐时刻剩余和动态水平/垂直角裕度、条件模型结果、风险配置 SHA256 与版本、模型假设类型、姿态来源和具体失败原因。非法 PSD、非对称、非有限协方差、缺少条件模型数据、未经校准、风险采样数不够均不能产生鲁棒通过。

## 几何与概率适用范围

世界协方差通过完整光学旋转映射：`C_optical = R_world_to_optical C_world R_world_to_optical.T`。水平/垂直中心角为 `atan2(x,z)`、`atan2(y,z)`，解析 Jacobian 为：

```text
J_optical = [[z/(x²+z²), 0, -x/(x²+z²)],
             [0, z/(y²+z²), -y/(y²+z²)]]
J_world = J_optical R_world_to_optical
```

`world_target_angle_jacobian()` 单独暴露该计算，并用已知光轴和独立中心差分核对。角协方差仅用于诊断。准入使用完整球与置信椭球的 Minkowski 和，对任意单位平面法向 `n` 的支持函数为：

```text
n·mu + target_radius + k sqrt(n.T C_optical n)
```

所有水平/垂直切平面角通过批量二分求根获得。它不会依赖一阶线性化在大不确定度下仍成立的假设；独立椭球边界采样验证所得角包络包含实际球投影。前平面和轴向距离使用同一集合支持函数；径向距离使用最大特征值给出的保守外球。名义完整球不通过时，鲁棒结果必不通过。

`risk_budget=alpha` 是整个**配置离散窗口**的风险预算。`sample_count=K` 必须覆盖整窗所有检查时刻，单点风险为 `alpha/K`，不需要时间独立性：

```text
P(任一配置时刻失效) <= sum_t P(该时刻失效) <= alpha
```

Gaussian 模式取 `k=sqrt(chi2_3.isf(alpha/K))`。奇异协方差仍用三自由度，是保守处理。`moment_bound` 取 `sqrt(3K/alpha)`，要求所声明条件均值与协方差确为分布的真实二阶矩；估计器输出一个矩阵本身不建立这个条件。各条件模型使用同一单点预算，按归一化权重混合后仍不超过该预算，不要求模型身份整窗固定。

单时刻 95% 不等于 1.6 s 全轨迹 95%。离散 union bound 也不涵盖采样之间的连续时间；报告固定 `continuous_time_guarantee=False`。Gaussian、二阶矩和 IMM 概率保证均依赖所声明且经过校准的条件预测分布，不保证任意真实机动。`calibrated=True` 与 `calibration_id` 是调用者绑定证据的声明，本纯模块不能验证其真实性；在线加载和独立接收器必须进一步绑定真实校准文件 SHA256、预测身份与本地约束快照。

`empirical_envelope` 使用指定 `empirical_scale` 的校准经验椭球，仅证明该包络内的几何包含关系，报告 `probability_bound_available=False`，不能将经验测试覆盖率当作解析全窗概率保证。

旧 `prediction/*_calibration.json` 只校准了合成 xy 残差，其早期半径语义也已被修正，不能作为最终资格。后续 `prediction_02` 使用完整 xyz 残差与三自由度经验半径（`empirical_scale=2.7954834829151074`），其后独立 CTRA 极低速 Jacobian 和停止过程可观测性修复又改变了预测器。最终研究绑定 `prediction_04` 校准和完整源码字节快照；旧数据保持，不覆盖。无论哪一版，合成工程包络均不建立真实 RGB-D 的统计覆盖资格；低概率条件模型不在当前统计校准范围内。全模型几何检查可以更保守，不能弥补统计适用范围缺口。

## 规划姿态与工程误差界

配置明确区分 `attitude_source='planned'` 与 `'measured'`。yaw、完整姿态和外参旋转误差是工程上界，必须给出 `error_bound_source`；它们不会把理想加速度姿态变成实测姿态。

总旋转上界 `delta` 转为额外空间包络，使用 `2 R sin(delta/2)` 的 SO(3) 位移界，并计入完整目标、协方差半径及安装杠杆臂。平移外参上界另行累加。这样不会把三维旋转误差直接等同于两个光学角各加同一常数；后者在非平面目标方向上未必保守。真实误差是否落在这些工程界内，仍需独立数据验证。视觉 yaw 执行者和 PX4 发布所有权没有改变。

## 自适应观察点

实现位于 `guidance/adaptive_viewpoint.py`，继续调用 `follow_reference.viewpoint_state()` 的解析运动学。默认候选距离 4.5–6 m、侧后角 ±20°，偏好原 5 m 后方；默认距离变化率 0.15 m/s、角度变化率 0.05 rad/s，另有限加速度、平滑代价、切换代价、驻留及滞回。距离域有硬上界，不能通过不断后退改善指标。

`AdaptiveViewpointSelector.select(dt, risks, ...)` 使用真实 dt，先推进此前已经选择的 profile 到当前时刻，再从当前 P/V/A 开始新动作；新观测不会回填上一采样区间。每个动作是经过全部内部极值检查的有界五次曲线，到达目标后 P/V/A 连续接入零导数保持。动作中不因小幅评分变化切换另一侧；正在执行的观察点被判为不可用时，显式报告失败，由原安全回退处理。失败不会通过冻结 profile 时间伪造连续性。

```python
state = selector.forecast(t)
p, v, a = viewpoint_state(..., **state.reference_kwargs())
metadata = selector.reference_profile()
state_again = profile_state(metadata, t)
```

metadata 为 `duration, elapsed, distance_coefficients[6], beta_coefficients[6]`，升幂且自变量为归一化时间 `s=t/duration`。持续曲线可独立重建；`duration=0` 只允许常值。六个瞬时状态为 `d,beta,d_dot,d_ddot,beta_dot,beta_ddot`，单靠它们不能重建整个五次动作，不能改用无界常加速度抛物线替代。`ViewpointState.forecast()` 仍保留为严格常加速探针，包含区间内部极值的域检查，越界直接拒绝。

研究链 `forecast_viewpoint`、`FollowProblem.reference` 及两个 FOLLOW rollout 已使用该 profile；无 profile 保留原默认分支。请求保存真实 `now_stamp` 时刻的 elapsed，未来执行参考统一查询 `execution_start_stamp-now_stamp+t`。候选 beta 是相对 profile 的偏移，显式 distance 是相对原 `follow_distance` 的偏移，导数继续取 profile；最终越界直接拒绝，不做 clip。增强 rollout 以明确参考 P/V 生成期望速度，再走原 `FlightGuidanceCore` 的限速、限加速度和海面 guard；目标预测数据不被伪装为参考点。

## 验证与未完成边界

新增测试覆盖独立光轴、完整外参、球切平面边界、协方差朝向/相关性、Jacobian、低质量分支、非法输入、全窗风险配置、yaw 工程界、抗振荡、解析导数、profile 序列化、时间因果、future lead、main/fast rollout 与海面失败状态保留。

本机 17 时刻纯几何微基准中，融合分布约 1.54 ms 中位数、1.63 ms P95；融合加三个条件模型约 3.43 ms 中位数、3.54 ms P95。模型与角边维度一起批量求根，优化前后三模型中位数约 10.0→3.43 ms。该结果只代表本机合成几何输入，不包括预测器、ROS、候选搜索或接收器全部在线耗时。

日志与来源指纹位于 `data/experiments/20261014_p6_development_01/visibility/`。校准有效性、图像实际失锁、规划/真实姿态差异、MINCO 实际控制权和五模式真实闭环效果均须以新实验独立验收，不能从这些纯数学测试推断。

## 与真实短租约一致的风险范围

后续root集成新增显式 `P6Policy.risk_horizon_seconds`：`0` 表示完整MINCO离散窗，`.45` 表示实际不可续期执行租约前缀。只允许这两个值，不能以`.2`的风险检查覆盖`.45`的执行权。普通启动仍nominal；增强策略默认`.45`，原完整1.6 s名义球/FOV/range/动力学检查保持。

接收器原租约终点为 `min(curve.end, curve.start+.45)`，每个新child各自重验，tick不会把它改为`now+.45`。风险查询只检查实际当前剩余区间，并明确输出 `checked_indices`、`checked_relative_times`、`risk_scope` 和 `entire_minco_curve_robust=False`。未经检查的后段没有被标为鲁棒安全，仍受整条原名义硬门禁。`.45`端点必须显式采样，否则报告`P6_RISK_ENDPOINT_NOT_SAMPLED`；风险scope进入本地配置与约束指纹。

kernel仍按原完整query数量分摊风险预算；取子集不会偷偷增大单点预算。独立scope审查覆盖精确端点、原deadline、完整后段名义拒绝、批量索引及指纹，见`test_p6_risk_prefix_review.py`。这种工程策略对齐实际执行权限，不把多个tick/多个lease合并成整任务95%保证，也不扩大原执行寿命。
