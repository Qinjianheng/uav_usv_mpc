# P6 原始观测 IMM 实现与边界

新增代码是独立的 CV/CTRV/CTRA 三模型 IMM EKF。它直接读取
`TargetObservation`，原 KF/BCTRA 链和默认预测器保持原状。默认输出
`/planning/imm_target_prediction`，可由 P6 launch 显式选择主预测器。
该文档确认数学、时间与消息边界及离线 CPU 测试。五模式合成数据研究已经完成，
见 [预测研究报告](p6_prediction_report.md)；真实影子飞行与联合闭环另行验收。

## 状态与交互数学

纯模块为
[`imm_predictor.py`](../../src/uav_control/uav_control/tracking/imm_predictor.py)。
共同水平状态为 `[x, y, vx, vy, omega, a_long, alpha]`，单位依次为
`m, m, m/s, m/s, rad/s, m/s², rad/s²`。不存储或混合航向角，避免 ±π 边界。

- CV 保持 Cartesian 速度，将机动状态投影为零。
- CTRV 保持速度大小与转弯率，将纵向加速度与角加速度投影为零；
  位置积分使用解析 sinc 形式，小转弯率导数使用级数。
- CTRA 保持纵向加速度、角加速度，令
  `theta(t)=omega*t+alpha*t²/2`，速度沿初始速度方向加速后旋转；
  位置使用 12 节点 Gauss 积分，使用解析 EKF Jacobian。
  零速时纵向方向不可观测，因此不会制造任意朝向。

负纵向加速度只积分到 `t_stop=-speed/a`，随后位置固定、速度与向量加速度为零，
不允许用负 scalar speed 反转 Cartesian 航向。位置截停边界的速度为零，因此
Jacobian 的积分上限导数边界项为零；速度映射在恰好停止时非光滑，使用停止侧
Jacobian。正加速在速度 <=1e-3 m/s 时不制造不可观测的朝向；负加速即使低于
该门槛仍正确截停，保留分步传播一致性。

停止分支不能同时投掉过程噪声：Cartesian acceleration 驱动在该边界采用 CV 型
P/V covariance，保留 `Qpp=q*t³/3`、`Qpv=q*t²/2`、`Qvv=q*t`，使随后真实位置
观测可以重新识别速度。20,000 粒子独立过程噪声 Monte Carlo 已核对其 covariance。
其余曲线的 frozen-posterior Jacobian 积分及 latent 噪声幅值不变。

当前 posterior latent 工程均值边界复用既有 BCTRA 范围：omega ±0.7 rad/s、
a_long ±2 m/s²、alpha ±1.5 rad/s²。所有模型还采用相同的当前水平速度先验
5 m/s，来源为声明的五模式最大 4 m/s 加 25% 估计余量；vx/vy 按向量范数投影。
未来更快目标需要显式调整 `maximum_horizontal_speed`，不能沿用该先验。
均值投影后增加 `delta*delta.T` 保留围绕
新均值的二阶不确定度，不缩协方差、不重置模型概率、不读取真值。
这是当前状态估计的工程先验，未来 constant-alpha/a 路径不会被强制改成统一终端速度，
也不能声称未来速度、转弯率处处满足当前 posterior 边界。

矩阵 `M[i,j]` 明确表示从当前模型 i 转移至下一模型 j，行和为一。
每个新采集戳进行：

1. 计算 `c[j]=sum_i mu[i]*M[i,j]` 与 `w[i,j]=mu[i]*M[i,j]/c[j]`。
2. 混合各目标模型的初始状态，以及包含模型均值分歧项的协方差。
3. 独立执行模型预测、位置观测 EKF 更新、Joseph 协方差更新。
4. 用 innovation Gaussian log likelihood 与 Markov prior 做 log 域归一化。
5. 融合模型后验均值与协方差；融合协方差包含模型内部协方差和模型间分歧。

交互定义参考 [FilterPy 官方实现 IMM.py](https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/IMM.py)，
本项目自行实现且不添加 FilterPy 依赖。精确零概率支持不被概率下限或均匀重置复活；
极大共同 residual 先减去最大 log likelihood，再加入 log prior，避免浮点间距吞掉 prior。
数值失败显式拒绝更新或发布 invalid，不把异常重置为均匀概率。

过程协方差使用 `integral F(lag,x_post)*Qc*F(lag,x_post)' dlag` 的 4 节点积分，
其中 `F` 为冻结 posterior 处的有限时间 Jacobian。这对 CV、直线名义 CTRA
积分链精确；弯曲 CTRA 是局部 EKF 近似。纵向/角加速度噪声会在本预测区间
贡献未来位置方差。它不是精确非线性随机过程分布，仍须独立残差校准。

垂向使用独立 z/vz CV 更新及指数衰减、有界位移预测，默认速度上限 1 m/s、
衰减时间 0.75 s、最大位移 0.5 m。它不订阅原 KF 或真值。拆分测量 xy/z
交叉协方差时，以被删除交叉项绝对值的对角膨胀保持保守的块独立测量协方差。
垂向均值限幅后仍保留未限幅传播的不确定度，不宣称是实际升沉的确定误差界。

## 纯数学 API

```python
import numpy as np
from uav_control.tracking.imm_predictor import ImmConfig, ImmPredictor

predictor = ImmPredictor(ImmConfig())
status = predictor.update(10.0, (1.0, 2.0, -0.15), np.eye(3) * 0.01)
assert status.accepted
forecast = predictor.forecast_at(10.02, (0.0, 0.2, 0.5, 0.8, 1.2, 1.6, 4.0))
```

`update(stamp, position, covariance)` 接受 xyz 与 3×3 或平铺 9 元协方差，
返回 `UpdateStatus(accepted, reason)`。重复/乱序/不合法数据不改变 posterior。
大于 `max_update_gap` 的真实新观测重新初始化为高速度不确定度轨迹，
返回 `REINITIALIZED_AFTER_GAP`，不会从旧轨迹延长执行资格。

`predict(horizons)` 等价于 `forecast_at(predictor.stamp, horizons)`。
`forecast_at(source_stamp, horizons)` 从最新原始观测 posterior 显式传播到
source 历元，返回未来点相对于 source 历元的预测，且不突变滤波器。

`ImmForecast` 提供：

| 字段 | 形状/意义 |
|---|---|
| `source_stamp`, `observation_stamp` | 显式状态历元、原采集历元 |
| `horizons` | N，相对 source 历元的秒 |
| `position`, `velocity`, `acceleration` | N×3，融合 xyz |
| `position_covariance`, `model_disagreement` | N×3×3，总位置协方差、模型间分歧 |
| `model_probabilities` | 3，顺序 CV、CTRV、CTRA |
| `state`, `state_covariance` | 7、7×7，source 历元的水平融合状态 |
| `conditional_position/velocity/acceleration` | 3×N×3 |
| `conditional_position_covariance` | 3×N×3×3 |

未来模型概率固定为当前后验，每条条件轨迹表示保持当前模型的未来，
不宣称已经枚举未来 Markov 切换树。模型明显分歧时必须检查条件分布，
不能将一个融合 Gaussian 当作精确多峰描述。

独立单模型滤波对照可以使用 `transition_matrix=np.eye(3)`，并设置
`initial_probabilities` 为对应 one-hot。公开函数
`propagate_model(state7, covariance7x7, dt, model, config)` 可用于相同初始状态下
的纯运动模型对照。两类对照不可混称完整感知预测链对照。

## ROS 与时间契约

包装器为
[`imm_target_predictor_node.py`](../../src/uav_control/uav_control/tracking/imm_target_predictor_node.py)，
节点名 `imm_target_predictor_node`。它只订阅原始 `TargetObservation` 与
`MissionState`。输入 topic 及 resolved remap 均拒绝真值 topic 与
`/tracking/target_state`，消息还拒绝 truth source、frame 错误与采集戳不一致。

每个 acquisition stamp 只触发一次滤波更新与一次有效预测发布。重复/乱序
消息只记拒绝原因，保留已接受的新观测；定时器仍发布过期 invalid 心跳。
无效观测、错误 frame、future/stale 输入会撤销当前有效输出。ROS 时钟回退或
image/PX4 clock reset count 变化清除 posterior；reset count 变化的触发观测
拒绝，随后新观测才能重建轨迹。mission_id 改变清除前一任务轨迹。

有效 `TargetPrediction` 复用现有消息：

- `source='tracking'`，`model='IMM_CV_CTRV_CTRA'`。
- `source_stamp=generated_stamp=now` 是由最后 raw posterior 显式传播的状态历元。
- `observation_stamp` 始终为最后接受的图像采集戳；不会用 receipt/generated 代替。
- `valid_until=observation_stamp+input_timeout`，`input_timeout<=0.125 s` 为硬上限。
- 每个 `position_covariance[9]` 由实际模型传播产生，不使用 BCTRA 经验标准差。
- invalid 时 samples 为空；不会延长旧轨迹、伪造新鲜观测或发布控制命令。

默认 topic 参数：

| 参数 | 默认值 |
|---|---|
| `observation_topic` | `/perception/front/target_observation` |
| `prediction_topic` | `/planning/imm_target_prediction` |
| `diagnostics_topic` | `/planning/imm_diagnostics` |
| `mission_topic` | `/mission/state` |
| `frame_id` | `local_ned` |
| `update_rate_hz` | 20 |
| `prediction_horizon`, `prediction_sample_period` | 4.0 s、0.1 s，含 0 共 41 点 |
| `input_timeout` | 0.125 s，可缩短不可延长 |

`ImmConfig` 的所有字段均使用 `imm_` 前缀声明为 ROS 参数，包括垂向参数；
transition matrix 在 ROS 中为按行平铺 9 元。默认转移矩阵按每个唯一观测
执行一次，未按可变 dt 转为连续时间 hazard，改变观测率需要重新检查该假设。
噪声与初始不确定度默认值在 dataclass 中集中定义，非法参数启动时拒绝。
公开 `node_parameter_defaults()` 返回原生 Python 类型的全量 ROS 参数副本，
供节点与每轮完整 YAML 同源物化；已测试 `yaml.safe_dump/safe_load` 逐值一致。

## 只读诊断

JSON schema 为 `p6_imm_v1`，按 `(mission_id, sequence_id)` 与预测绑定。
含 source/observation/generated/valid_until、frame/source/model、原输入年龄、
原 received/image stamp、clock identity、模型概率、7×7 状态协方差、模型分歧。
`relative_times` 全部相对 `source_stamp`。诊断还记录 `posterior_projection_count`、
`posterior_projection_covariance='P_plus_outer_mean_shift'` 与当前速度先验。

`model_predictions` 为三个对象，每项含 `model`、`probability`、`positions`、
`velocities`、`accelerations`、`position_covariances`，供后续鲁棒 FOV 检查条件轨迹。
失效时 `valid=false`、明确 `invalid_reason`，不保留合法条件预测数组。
异常原输入的 NaN/Inf 仅在 invalid 诊断中映射为 null，不能成为有效预测。

消息 `compute_time` 及诊断的同名字段计入本次 raw update、forecast 与消息转换；
JSON/DDS 开销不在该字段内，并通过 `compute_time_scope` 明确标注。
完整 JSON 诊断约 43 KB/41 点，在线 DDS 带宽与丢包仍需 shadow 实测。

## 当前验证与未验证项

测试文件为 `test/test_p6_imm_predictor.py` 与 `test/test_p6_imm_node.py`，覆盖
手算 Markov/协方差混合、CV/CTRV/CTRA 解析边界、独立数值 Jacobian、连续
jerk 对位置/速度/交叉协方差的贡献、零速/小转弯率、±π 经过、PSD/奇异测量、
log 下溢保护、模型选择、条件分布、原采集期限、坏输入、重复乱序、clock reset、
mission reset、消息复用、输入真值隔离与 remap 拒绝。

TDD RED/GREEN 与复跑 CPU 脚本位于忽略目录
`data/experiments/20261014_p6_development_01/imm/`。冻结数学版本的 targeted
数学/包装/研究/契约测试 93 项通过，日志为 `37_final_frozen_model_regression.log`；
另有当前 root 集成风险窗口/身份契约 47 项独立复核通过，日志为 `36_current_root_integration_review.log`。

最终 `imm/final_04/benchmark.json` 绑定数学与包装源码 SHA，30 帧 warmup 后
300 帧测量，41 点×3 模型：raw update+forecast+消息转换 P95 4.20 ms，
再含完整 JSON P50/P95/P99 为 4.90/5.18/8.14 ms；最大 10.03 ms。
这是当前机器离线 CPU 测量，不含 ROS DDS/Gazebo/PX4 运行负载，也不是硬实时上界。

新增离线研究与已保留失败见 [预测研究报告](p6_prediction_report.md)。
冻结线性化 Q 曾导致 locked CTRA 停止后失去速度恢复，已用独立 covariance 和
连续观测回归修复。仅修 Q、仅加 latent 先验的中间短回放仍失败，均保留；最终
加入声明的 5 m/s 当前速度先验后，同一五模式×两 seed 的 10 条 locked CTRA
短回放全部通过预注册恢复检查：最大当前位置误差 1.587 m、当前速度 5 m/s、
4 s 相对位移 25.405 m，均保持 PSD。10 m/s 只是此前固定的数值恢复检查门槛，
当前物理先验为可配置的 5 m/s，二者不混用。
修复中间版本 `prediction/`、`prediction_02/`、`prediction_03/` 均保留，不能混用
不同源版本的结果、校准与资格。最终 `prediction_04/` 使用新 heldout seed3731，
保存七份实际源码/配置字节与完整 SHA 清单。root 集成曾发生的 source drift
已在当前文件重读与测试后复审，证据为 `imm/root_integration_review_04.json`；
该审查支持 BCTRA 主链、IMM shadow 的首轮 pilot，未授予 IMM 主链飞行资格。
真实 shadow 负载、真实误差校准与联合闭环仍需实际证据，不得据此声称概率安全保证。
