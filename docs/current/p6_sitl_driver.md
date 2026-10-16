# P6 独立 SITL 驱动与资格边界

`scripts/p6_minco_sitl.py` 当前使用唯一 IMM 主预测器，提供 B/D 条件清单、案例准备、资格绑定和单次执行入口。它没有自动批量启动或重试循环。案例准备、离线资格和真实飞行完成分别记录，不互相代替；历史 A/C 案例保留为冻结证据。

## 执行顺序与配置

每个案例是 `data/experiments/` 下从未使用的新目录，保存完整 `config/flight.yaml`、`config/research.yaml`、可选的校准原文件、`case.json` 和 `SHA256SUMS`；真实运行另写该目录的 `flight/`。输入文件保持原字节。完整 flight/research 可显式指定；pilot 默认只读复制 R11 F2 flight（沿 +X、2 m/s），其余角色默认使用 R11 F3 flight，research 默认使用 R11 F3 快照。准备阶段以 MotionParameters 以及 IMM 节点同源 `node_parameter_defaults()` 补齐未显式给出的声明参数；已有用户值保留，数组按 ROS 兼容列表保存，主预测 topic 与 launch 的最终覆盖一致。

角色有三种：

1. `pilot`：F2、`linear`、B 组、IMM 主预测，固定 60 秒实际接管观察窗口。
2. `baseline`：F3、原 `figure_eight`、B 组。必须引用新的真实健康 pilot，固定 60 秒。
3. `experiment`：五种运动与 B/D 的显式单个案例。必须引用新的真实健康 baseline。矩阵命令只打印 10 个比较条件。

| 组别 | 主预测器 | 可见性 | 自适应观察点 |
|---|---|---|---|
| B | IMM | nominal | 关闭 |
| D | IMM | robust | 开启 |

新运动通过完整 flight 的 `moving_target.ros__parameters.trajectory_type` 设置。案例记录 `UAV_USV_P6_PREDICTOR=imm`、`UAV_USV_P6_IMM_SHADOW=false` 和 `UAV_USV_P6_RECORD=1`，保留原始观测、预测诊断及唯一主预测图证据。驱动拒绝旧 `--imm-shadow` 开关，不恢复独立 BCTRA。

Planner/Receiver 的 `p6_visibility_mode`、`p6_adaptive_viewpoint`、`p6_calibration_file` 及其他共享 P6Policy 配置逐项一致。输入两侧已经配置却不一致时拒绝准备。robust 必须提供真实校准文件，复制并绑定 SHA256；驱动不会生成校准分布、伪造校准通过或修改风险阈值。该文件是否满足 P6 schema、对应正确预测器家族及独立数据资格，由资格阶段和运行时 P6Policy 再验证。

seed 记录作用域为“元数据；运动确定性；驱动不注入噪声”，不会暗示未实施的随机控制。默认配置的原硬限制、输入时效、唯一 Tracker 和执行租约不变。

## 准备命令

以下命令仅准备文件或展示条件，不启动仿真。先 source 新工作区：

```bash
cd /home/qin/data/uav_usv_mpc
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
python3 scripts/p6_minco_sitl.py matrix
python3 scripts/p6_minco_sitl.py prepare \
  /home/qin/data/uav_usv_mpc/data/experiments/20261014_p6_sitl_01/pilot \
  --role pilot --motion linear --group B \
  --flight-config /absolute/path/full_flight.yaml \
  --research-config /absolute/path/full_research.yaml
```

移除两个 `--*-config` 参数会使用上述按角色选择的 R11 完整快照。每个 case 都须独立完成新完整配置资格；D 组另传 `--calibration /absolute/path/calibration.json`。

## 资格 JSON 与 readiness 生成

驱动只绑定已有验证与独立审查，不自行创建通过结论。`verification.json` 必须实际记录 `passed=true` 与 `independent_review_pass=true`。`qualification.json` 必须包含：

- `kind: P6_TESTED_PROTOCOL_CANDIDATE`，不能标成新系统真飞成功。
- `candidate` 为现有 H1.6 P47/C：`engine=p47_follow_guided`、`horizon=1.6`、`terminal_mode=qva`、`refinement=false`、`scheduling=adaptive`。
- `offline_gate_pass=true`，`qualified_cases` 明确列出该 `case.json` 的 `case_id`；`qualified_case_manifests` 为 `{case_id: SHA256(case/SHA256SUMS)}`，资格必须绑定真实已检查的完整配置清单。同名案例的不同配置不能沿用旧资格。
- `source_checks` 为实际已审核源码的 `{path, current_sha256}` 列表，必须覆盖 `critical_sources()` 返回的完整运行源码清单；运行前重新检查。可从该公开 helper 得到清单，review/test 通过后再写入对应当前哈希。
- `formal_directory` 指向带有效 `SHA256SUMS` 的原 H1.6 完整约束证据，包含原 `f2_h1.6_main_C_single.jsonl` / `f3_h1.6_main_C_single.jsonl`。这是原 MINCO 约束参考；不赋予新 IMM/robust 飞行成功结论。
- IMM 案例额外要求 `imm_primary_evidence: {path, sha256}`，引用通过的 `P61R_IMM_OFFLINE_BOOTSTRAP` 报告；`shadow_gate_pass=true` 不能替代该证据。报告必须保留 `scope=linear_nominal_pilot`、通过的 replay/review/verification 文件及其 SHA256，以及 `p61r_imm_qualification.sources()` 覆盖的当前源码指纹。
- 合格的 linear/B/pilot bootstrap 可作为 baseline（figure_eight/B）及五模式 B/D experiment 的共同 IMM 基础证据。该复用不扩展原报告的实测范围，也不赋予后续 case 飞行成功结论。B 必须为 nominal 且关闭自适应观察点，D 必须为 robust 且开启自适应观察点。
- 每个后续 case 仍须独立列入 `qualified_cases`，绑定自己的完整配置清单、当前运行源码及 verification/独立审查；D 组还须真实 `robust_gate_pass=true` 和独立校准。缺任一条件保持拒绝。

qualification 所在目录还必须有覆盖实际资格文件的 `SHA256SUMS`，原 p5 claim 在消耗真实尝试之前核验该清单与 formal 清单。readiness 可放在独立文件，再由根任务完整归档。

```bash
python3 scripts/p6_minco_sitl.py readiness \
  /absolute/path/pilot_readiness.json \
  --case /home/qin/data/uav_usv_mpc/data/experiments/20261014_p6_sitl_01/pilot \
  --verification /absolute/path/verification.json \
  --qualification /absolute/path/qualification.json
```

readiness 绑定案例与配置清单、verification、qualification 和完整运行源码清单，覆盖 IMM、robust、adaptive、p6_follow、Planner/Tracker core、launch、脚本及消息。任何后续修改或源码清单增减都会使 readiness 失效，必须先补对应测试/审查，再生成新 readiness；不能仅重写哈希追认旧资格。

运行前会重新验证共享 IMM 报告及其底层证据，readiness 创建后修改报告或审查文件仍会被拒绝。本轮资格校验源码及其依赖清单已变化，旧源码指纹的资格包需重新验证和审查，保存到新目录；历史证据不改写。bootstrap 只验证离线兼容性，实际执行仍遵守 pilot → baseline → experiment 的真实健康先行报告门禁。

## 输入对齐的后继规划调度

启用实际执行的 `deadline_scheduling` 时，P4ResearchRunner 的常规规划周期从所选预测回调的 `receipt_monotonic` 起算。周期到达后，只有收到该周期的新预测才准备请求；原始图像、导航及预测时间戳仍用于全部新鲜度、身份和接管检查。接收时刻只决定何时尝试求解，不能更新原始125ms有效期或450ms租约。单次有界 parent retry 保留原预算检查和重试冷却；已完成 worker 始终优先消费，不积压请求。未启用实际执行的 shadow 路径继续使用原定时冷却。

输入抖动可能使常规实际派发低于配置的5Hz；两轮真实 monitor 到达记录的离线时序模型约为4.4Hz。`dispatch_timing` 固定保存本次实际所选预测的 sequence、采集时间、ROS/monotonic接收时间及派发时间，完成时的新预测不会覆盖这些字段。该记录用于归因，与现有完整 request/provenance 审计并存。

定向行为回归在 `test_p6_input_aligned_dispatch.py`。阶段证据位于 `data/experiments/20261016_p61r3_input_aligned_03/`；其中 monitor 接收时间仅作为 planner callback 到达的代理，时序模型使用明确标注的中性导航和固定 worker 服务时间，没有验证 MINCO 可行性、所有权或飞行效果。源码变化使既有 Readiness 失效；该改动不授予新的 SITL、F3 或 D 组资格。

`successor_scheduling: delivery` 显式启用后继交付调度；缺省 `periodic` 保持既有 case 的周期策略。仅在新鲜、真实 ACTIVE ACK 对应的父轨迹下提前派发一次后继；交付失败或丢失 ACK 时，仅对新采集输入允许一次有界重试。pending 后继阻止再提交，单 worker 不积压任务。ACTIVE ACK 可提前更新调度状态，较旧 heartbeat 不得覆盖；published/ACCEPTED 不代表实际控制接管。

delivery 的预算使用完整 worker 成本提示、准备后实际时钟、排队/调度抖动提示及原有 20 ms 发布预留。排队/调度提示为近期排队中位数的 1.2 倍且保留既有 20 ms 下限；带载回放否决了仅使用排队中位数的较小预留。提示不保证完成，原 125 ms TTL、150 ms execution lead、450 ms 租约、50 ms 激活余量、预测复核及独立 Tracker 验收均保留。完成复核检查原始候选的接管时刻、最新输入和真实父轨迹，不再构造并丢弃一条更晚的新请求。P47 与 fallback 共享严格系数/约束/任务/时钟 key 的纯数学缓存，当前目标、视场和海面约束仍独立复核。

`test_p6_successor_delivery.py` 覆盖提前派发、新观测有界重试、pending 阻塞、乱序 ACK/heartbeat、过期状态和成本证据老化。阶段证据位于 `data/experiments/20261016_p61r3_successor_delivery_05/`；真实已配对输入的多租约回放仅为 recorded-state 时序模型，不是 ROS/PX4 闭环或飞行资格。新策略必须建立新 case 配置/源码指纹、当前测试、独立审查、资格及 Readiness 后才能启动有界 F2。

R4 的 `takeoff_visual_tracking: true` 为显式试验开关，缺省关闭。它将新鲜且锁定的 `/tracking/target_state` 接入已有 FlightGuidance 起飞水平混合：相对起飞高度 0.5 m 以下保持零水平期望，0.5–1.5 m 渐进开启，沿用起飞速度/加速度与海面保护。首次速度命令从实测速度初始化，避免冷启动跳变。仅当真实可见 bearing 在 125 ms 内、PX4 状态新鲜且已 armed/OFFBOARD 时，可在 TAKEOFF 提前使用受限视觉 yaw。起飞前 PX4 会清空未进入 rampup 的 setpoint，进入 rampup 后保留其内部推力/倾角限制；本改动只解除新鲜视觉 yaw 的高度抑制，不改变 0.5 m 平移门限。失去可见性仍保留原 1.5 m 盲搜索高度。INIT/GROUND_HOLD、MINCO 所有权和原始时间戳门禁不变。

该开关针对八字首轮起飞期间的视场/深度失锁，不能把单位测试或记录状态下的命令回放称为成功起飞。启用它必须建立新 case、新源码资格和独立审查；旧 Readiness 作废。R4 首轮失败原始证据保存在 `data/experiments/20261016_p61r3_successor_delivery_05/baseline_figure_eight_B_01/flight/`，阶段分析与复测材料在 `data/experiments/20261016_p61r4_figure_eight_01/`。

弯道修复的 `follow_reference_mode` 缺省为 `target`，新 case 可在 planner/Tracker 两端显式设为 `rear_point`。P47 使用既有后方观察点 P/V 参考，其常量偏置批处理与标量拟合逐点等价；不改变控制速度/加速度/jerk 整形或海面保护。接收端使用最多五个不同图像采集戳、0.25 s 因果历史拟合速度方向转率；KF 发布戳抖动不能当转率。最新 KF/导航仍须在 125 ms 内且 native bracket ≤50 ms。后方点速度误差与原始 target-relative 速度分别记录，位置/速度误差阈值保持；历史不足失败关闭，不授予入场。真实 MINCO 期间才可使用同刻 KF/native LOS yaw-rate 前馈，过期或未配对的状态不提供前馈。

接收端 `execution_prediction` 按接收端可见性策略选择负载：nominal 仅使用已通过 ROS 转换验证的预测均值，不在控制回调解码未使用的 IMM 条件模型 JSON；robust 仍要求完整模型诊断及严格身份匹配。原始诊断队列继续有界保存，125 ms 过期、独立全前缀几何和最终时钟复查保留。该优化不授予 D 组资格，也不把历史 pilot 变为当前版本的飞行结果；每个新 case 仍须绑定当前源码和独立审查。

显式研究 yaw 上限可到 2.5 rad/s，标准 MpcConfig 上限不变。该值需逐 case 绑定新约束指纹与当前真实输入离线结果、PX4 物理依据、独立审查和新 Readiness，不能复用原 1 rad/s formal 包。P6 准备拒绝 planner/receiver yaw 或 reference mode 不一致；实际 harness 从已审研究配置派生加载值，不再强制覆盖为 1 rad/s。当前离线证据在 `data/experiments/20261017_p61r4_turn_reference_03/`，其中新生成的 `f*_h1.6_main_C_single.jsonl` 仅保留旧 parser 文件名，内容明确标为当前真实 IMM/B 输入的离线计算，不能称为历史 C 组飞行或新闭环成功。

本次仍保留 125 ms 原始观测租期。接收端新增 `validation_cpu_seconds` 和 `validation_stages`，与壁钟耗时分别记录，区分 admission、预测转换、prefix geometry、最终上下文/tick；不由 monitor receipt 推断 Tracker callback 到达时刻。修改 freshness 必须另建跨预测/规划/接收端一致的时间契约与资格，不能只改 YAML 或延长已有消息的 valid_until。

## 单次真实执行入口

下面是根任务完成资格后才使用的入口，本开发没有执行：

```bash
python3 scripts/p6_minco_sitl.py run \
  /home/qin/data/uav_usv_mpc/data/experiments/20261014_p6_sitl_01/pilot \
  --qualification /absolute/path/qualification.json \
  --readiness /absolute/path/pilot_readiness.json --execute
```

baseline / experiment 另传 `--prior-summary /absolute/path/prior_case/p6_run_summary.json`。引用必须同时具备未变的案例清单、真实 `flight/p5_run_summary.json`、对应 SHA256、原始 `offline_evidence.jsonl` 与 P53 健康接管门槛，并覆盖至少 59.5 秒。仅写一个带 `completed/healthy=true` 的 JSON 不满足门禁。

执行复用原 p5 有界启动、隔离授权、健康监控、原 FOLLOW 稳定等待、真实接管判断以及精确任务进程清理。`observation_seconds=60` 同时作用于停止条件和 owner 统计，p51 独立执行审计选用原 60 秒分支。窗口从首次真实 `MINCO_FOLLOW` 控制权开始，统计所有后续 owner；后半段回退保留。真实 owner/native PX4/ACK 链仍由原审计判定，轨迹、proposal 数量与计划发布不会冒充实际控制权。

`p6_run_summary.json` 保留原审计结果及身份/指纹，并单独记录 `window_evidence_complete`：实际 owner 记录至少 59.5 秒且诊断覆盖至少 95%。缺失/不完整记录不能返回成功。源 p5 的结果字段不会被伪改成完整窗口。性能研究目标另行比较，不以修改安全门槛实现通过。

## 对旧驱动的最小扩展与测试

`p5_minco_sitl.prepare(..., overlay_config=None)` 默认仍委托原 prepare；显式 overlay 使用完整新 flight，不依赖历史 P41 flight。`run(..., overlay_config=None, observation_seconds=None)` / `assess(..., observation_seconds=None)` 保持默认 F2 25 秒、F3 60 秒，P6 显式传入 60 秒。

P6 condition 中的历史合同明确记录为 `baseline_reference_constraint_fingerprint`；`constraint_fingerprint` 与 `actual_constraint_snapshot` 由保存的完整 research 参数、本地 ResearchConfig、原相机常量以及 P6Policy/校准构造，不能用历史 raw 的 fingerprint 冒充新 robust/adaptive 合同。B 组关闭增强时，实际 fingerprint 必须逐值等于原 raw；不相等则在消耗尝试之前拒绝。已对真实 R11 research/condition 做该一致性测试。无 P6 overlay 的旧路径继续保持原字段与行为。

历史开发证据在 `data/experiments/20261014_p6_development_01/sitl_driver/`：首轮 RED，以及部分窗口、record 开关、配置资格绑定、完整参数和实际合同的 RED→GREEN。`green_full_parameters.log` 为 83 passed（P6 driver、P5/P53 和 pep257）；联合 IMM 数学/Node 的 `green_final_binding_parameters.log` 为 147 passed。这些历史结果包含当时的 A/B/C/D 条件，不能代替当前 IMM-only 版本验证。

当前作用域回归位于 `test_p61r_imm_qualification.py` 与 `test_p6_sitl_driver.py`，覆盖三种角色、五模式 B/D、非法配置组合、证据完整性及审查失败、共享 bootstrap 与独立 case 绑定、源码变更、robust 门禁和真实先行报告拒绝。

测试中的合成外部进程返回值仅用于验证驱动拒绝错误，不能用作任何真实 pilot、baseline、IMM shadow 或飞行资格。该驱动为本人实现，测试与代码核对属于自检；独立审查由根任务另行完成。
