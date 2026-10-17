# P6 真实日志离线分析

`scripts/p6_actual_analysis.py` 只读取已保存的真实 case，输出评分和 shadow 证据诊断。
它不运行仿真、不修改原始 case、不发布消息，也不生成或批准 qualification。
合成测试通过只证明分析协议的实现行为；真实 shadow 门禁仍须实际健康 pilot 数据。

```bash
cd /home/qin/data/uav_usv_mpc
python3 scripts/p6_actual_analysis.py \
  data/experiments/<run>/<case> \
  data/experiments/<run>/<case>_actual_analysis_01
```

输出目录必须尚不存在，且必须位于 case 之外。分析器保存 `analysis.json`、逐预测逐时域
`forecast_scores.jsonl` 和 `SHA256SUMS`，记录原始输入及分析器 SHA256。
CLI 返回成功表示分析完成；`shadow_gate.passed` 是报告内的诊断字段，不授权主预测器切换。

## 输入和时间边界

固定输入为 `case.json`、`flight/offline_evidence.jsonl`、
`flight/p5_run_summary.json` 和 `flight/execution_analysis.json`。
后三者使用现有真实运行记录，缺失 summary 或 execution audit 会使 shadow 诊断不通过。
JSONL 畸形行保留行号并阻止 raw 完整性门禁通过。

预测评分 cohort 事先固定为原 summary 的 `first_minco_receipt` 至其后 60 秒。
该 `receipt` 是监视器接收的墙钟秒，不能与消息 ROS epoch 直接相减。
预测样本的 `source_stamp + horizon` 只与 `ideal.stamp` 比较；评分时域固定为
0.2、0.5、0.8、1.2、1.6、2.4、4.0 秒。

未来真值只在离线分析中按保存的时间戳做线性插值，最大相邻间隔 0.2 秒，禁止外推。
真值记录接收时间严格截止于上述 60 秒终点，不使用停止后的目标冻结补足未来。
区间左端、右端、长间隙和时钟回退分别记录为
`left_censored`、`right_censored`、`interpolation_gap`、`truth_clock_ambiguous`。
1 微秒仅用于浮点 epoch 的比较容差，不是延迟补偿。
相同 epoch 且 p/v 一致的重复真值可去重；冲突重复或回退使整段插值拒绝。

预测 cohort 引用的原始观测可在起点前到达，相同 mission/sequence 的 IMM JSON 可在终点后
数毫秒到达；辅助记录按原身份关联，不因此延后真值截止时间。

原观测关联保留同时间桶内的全部记录，固定检查预测 acquisition 桶及两侧各两个相邻桶。
候选必须同时满足原 1 微秒时间差、诊断 raw acquisition 时间差以及明确的
`observation_source/frame/clock reset identity`；这只是大历元浮点表示的索引修复，
没有扩大容差，也不从全体记录选择最近项。只有一个候选才能关联；多个合法候选（包括
同桶别名和一个精确加一个邻近 epoch）显式拒绝为 `RAW_OBSERVATION_AMBIGUOUS`。
报告 `imm.observation_join_reasons` 保留成功、缺失和歧义数量。

## Raw 记录字段

所有 JSONL 行包含 `kind` 和 `receipt`。现有 monitor 在 `UAV_USV_P6_RECORD=1` 时保存完整数据：

| kind | 必需内容 |
| --- | --- |
| `p6_observation` | `stamp/source/frame/valid/reason/p/covariance`，`received/processed/published/image_measurement_stamp`，`image_clock_reset_count/px4_clock_reset_count` |
| `p6_prediction` | `topic/mission/sequence/source_stamp/observation_stamp/generated_stamp/valid_until/model/source/frame/valid/reason/compute`，完整 `times/p/v/a/covariance` |
| `p6_imm` | `schema_version=p6_imm_v1`，`mission_id/sequence_id` 和同一组 epoch，`source/model/frame_id/observation_source/valid/raw_input_stamp/clock_identity`，`relative_times/model_probabilities/state/state_covariance/compute_time_scope`，CV/CTRV/CTRA 三组完整 `model_predictions` |
| `ideal`、`kf` | `stamp/valid/p/v`；monitor 同时保存 `source/a` |
| `graph` | `publishers[topic]` 列表中的 `node/namespace/type/gid` |
| `reference` | 实际 `/control/reference` setpoint 的 `stamp/v/a`；原执行分析保留其 PX4 setpoint 时间增量审计 |
| `p6_motion_truth`（可选） | `stamp_ns/stage/mode`；monitor 同时保存 `position/velocity/acceleration/turn_rate` 等诊断量 |

每个 `model_predictions` 包含 `model/probability/positions/velocities/accelerations/position_covariances`。
位置、速度、加速度为 Nx3，位置协方差为 Nx3x3 或 Nx9；状态协方差为 7x7。
原始观测必须是有效非真值来源、`local_ned`、有限 p、PSD 协方差，图像原始 epoch 与 `stamp`
一致，且保留两个时钟 reset identity。运动真值只作离线阶段标签，不能回灌预测或控制。
没有 motion truth、记录过旧或未能因果匹配阶段时，阶段为 `unknown`。

## 评分与分解

报告分别保存 observation/KF 的同刻真值误差、主预测和 shadow 在每个时域及运动阶段的评分。
指标包括水平位置、3D 速度、航向误差，以及 count/RMSE/P50/P95/P99/max。
速度低于 0.2 m/s 的样本不计算航向误差。无效消息、缺少预测样本、未成熟未来和严重误差均保留，
不按误差大小裁剪。每个时域显式保存 attempted/scored/invalid 数量和未评分原因。

令预测初态为 `p̂0/v̂0`，源时刻真值为 `p0/v0`，未来真值为 `ph`：

```text
e_total     = p̂h - ph
e_initial   = p̂0 - p0
e_increment = (p̂h - p̂0) - (ph - p0)
e_total     = e_initial + e_increment
e_initial_cv = e_initial + h * (v̂0 - v0)
e_beyond_cv  = e_total - e_initial_cv
```

该等式针对水平向量严格成立，各向量范数不能直接相加。保存初始位置/速度误差、位移增量误差、
初始 CV 效应和剩余误差及其逐条向量。增量与剩余项仍含潜在初态误差，不能解释成匹配初态的纯模型因果误差。

## Actual shadow 证据诊断

`shadow_gate.passed` 要求同时满足以下预定义条件：

- 原始 case 为 `role=pilot/motion_mode=linear/prediction_model=bctra`，原 summary 确认完整健康
  60 秒（记录时长至少 59.5 秒、diagnostic coverage 至少 95%），P5.1 六项审计均通过；
  原 execution audit 确认实际单 tracker owner、ACCEPTED/ACTIVE 和 native setpoint 记录。
- 原始 raw 数据完整，obs/KF 的有效记录最大接收间隙及两端容差为 0.5 秒，ideal 为 0.2 秒，
  主预测及 shadow 的所有消息记录为 0.5 秒，`reference` 为 0.15 秒；没有畸形 JSON、真值时钟冲突。
- 超过 200 个 full IMM shadow forecasts 以及超过 200 个独立 acquisition；full 采用当前默认
  至少 41 个样本、从 0 开始且覆盖 4 秒的协议。每个有效 shadow 对应唯一完整 IMM diagnostics，
  没有重复 mission/sequence。
- 原始 observation/source/generated/expiry 顺序合法，`valid_until - observation_stamp` 为正且
  不超过原 125 ms lease；消息、诊断、raw acquisition 及 reset identity 一致。
- 三个条件模型和输出协方差对称 PSD；融合位置均值及全协方差（模型内项＋模型间项）与消息一致。
- 整个 60 秒的 graph 记录中主 `/planning/target_prediction` 只有预期 predictor 一个发布者；
  graph 最大间隔 2 秒、两端容差 1.5 秒。
- IMM 模型概率有限、归一化，至少一个概率实际变化范围超过 1e-6。
- 有效 shadow 的原消息 `compute` 全部有限非负，P95 ≤20 ms。计时作用域为
  observation update＋forecast＋message，排除 JSON/DDS；这是 20 Hz/50 ms 周期内的工程预算，
  不替代 acquisition 的原始 125 ms TTL。

报告原样引用 run summary、owner/lifecycle/native 审计并单独列出 60 秒 reference 覆盖。
只有真实完整健康 pilot 可形成 actual shadow 诊断；离线数学、合成 fixtures 或 source SHA
都不能替代它。即使该诊断通过，qualification 和 IMM 主链启用仍由独立运行门禁处理。

## 验证入口

```bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
cd /home/qin/data/uav_usv_mpc/src/uav_control
python3 -m pytest -q test/test_p6_actual_analysis.py
```

RED/GREEN 和最终验证清单保存在
`data/experiments/20261014_p6_development_01/sitl_analysis/`。
当前合成测试覆盖有界插值、分解、严重误差保留、右端删失、阶段缺失、完整性/身份/lease/PSD/
概率/算时/发布者门禁，以及辅助记录跨窗口到达和原 case 不可写边界。

首轮实际 `pilot_A_linear` 的旧分析报告保留不变。其 1,200 个有效 shadow 预测中，
22 条因大 ROS epoch 浮点往返相差一个 ULP（0.2384 微秒）跨入相邻索引桶，旧单桶
查找漏掉原观测，导致身份和 covariance 检查同时提前失败。独立逐矩阵检查全部
state、条件模型和消息协方差均有限、对称、PSD；新分析仅修复 bounded join 后另存
`pilot_A_linear_analysis_02/`，不会回填旧报告或赋予 IMM 主链资格。
本次独立诊断、真实行号示例及 RED/GREEN 位于
`data/experiments/20261014_p6_sitl_01/pilot_A_linear_imm_diagnosis_01/`。
