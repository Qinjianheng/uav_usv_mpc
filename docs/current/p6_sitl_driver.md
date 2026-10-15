# P6 独立 SITL 驱动与资格边界

`scripts/p6_minco_sitl.py` 已实现案例准备、A/B/C/D 条件清单、资格绑定和单次执行入口。它没有自动批量启动或重试循环；本次开发未启动 PX4/Gazebo。案例准备、单元测试、已有影子资格和真实飞行完成分别记录，不互相代替。

## 执行顺序与配置

每个案例是 `data/experiments/` 下从未使用的新目录，保存完整 `config/flight.yaml`、`config/research.yaml`、可选的校准原文件、`case.json` 和 `SHA256SUMS`；真实运行另写该目录的 `flight/`。输入文件保持原字节。完整 flight/research 可显式指定；默认只读复制 R11 F3 实际快照，不把普通 baseline 配置当作 R11。准备阶段以 MotionParameters 以及 IMM 节点同源 `node_parameter_defaults()` 补齐所有未显式给出的声明参数；已有用户值保留，数组按 ROS 兼容列表保存，主/影子预测 topic 与 launch 的实际最终覆盖一致。

角色有三种：

1. `pilot`：F2、`linear`、A 组、显式 IMM shadow。保留 BCTRA 真正主预测，IMM 只观察，固定 60 秒实际接管观察窗口。
2. `baseline`：F3、原 `figure_eight`、A 组。必须引用新的真实健康 pilot，固定 60 秒。
3. `experiment`：五种运动与 A/B/C/D 的显式单个案例。必须引用新的真实健康 baseline。矩阵命令只打印 20 个比较条件。

| 组别 | 主预测器 | 可见性 | 自适应观察点 |
|---|---|---|---|
| A | BCTRA | nominal | 关闭 |
| B | IMM | nominal | 关闭 |
| C | BCTRA | robust | 开启 |
| D | IMM | robust | 开启 |

新运动通过完整 flight 的 `moving_target.ros__parameters.trajectory_type` 设置。启动器选择通过 `UAV_USV_P6_PREDICTOR=bctra/imm` 与 `UAV_USV_P6_IMM_SHADOW=true/false` 传递，记录在案例与实际环境快照中。所有案例显式设置 `UAV_USV_P6_RECORD=1`，保留原始观测、shadow 预测/条件协方差及唯一主预测图证据；此项来自 `p6_imm` 代理对驱动的独立审查，已添加缺失开关 RED→GREEN 和实际环境传播测试。当前 launch 只在 BCTRA 为主时运行 IMM shadow，因此 IMM 主模式请求 shadow 时，案例也明确记录 `imm_shadow_effective=false`。

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
  --role pilot --motion linear --group A --imm-shadow \
  --flight-config /absolute/path/full_flight.yaml \
  --research-config /absolute/path/full_research.yaml
```

移除两个 `--*-config` 参数会使用上述 R11 完整快照。B/C/D 以及新运动应使用根任务生成并完成资格的新完整配置。C/D 另传 `--calibration /absolute/path/calibration.json`。

## 资格 JSON 与 readiness 生成

驱动只绑定已有验证与独立审查，不自行创建通过结论。`verification.json` 必须实际记录 `passed=true` 与 `independent_review_pass=true`。`qualification.json` 必须包含：

- `kind: P6_TESTED_PROTOCOL_CANDIDATE`，不能标成新系统真飞成功。
- `candidate` 为现有 H1.6 P47/C：`engine=p47_follow_guided`、`horizon=1.6`、`terminal_mode=qva`、`refinement=false`、`scheduling=adaptive`。
- `offline_gate_pass=true`，`qualified_cases` 明确列出该 `case.json` 的 `case_id`；`qualified_case_manifests` 为 `{case_id: SHA256(case/SHA256SUMS)}`，资格必须绑定真实已检查的完整配置清单。同名案例的不同配置不能沿用旧资格。
- `source_checks` 为实际已审核源码的 `{path, current_sha256}` 列表，必须覆盖 `critical_sources()` 返回的完整运行源码清单；运行前重新检查。可从该公开 helper 得到清单，review/test 通过后再写入对应当前哈希。
- `formal_directory` 指向带有效 `SHA256SUMS` 的原 H1.6 完整约束证据，包含原 `f2_h1.6_main_C_single.jsonl` / `f3_h1.6_main_C_single.jsonl`。这是原 MINCO 约束参考；不赋予新 IMM/robust 飞行成功结论。
- IMM 主预测案例额外要求真实 `shadow_gate_pass=true`；robust 案例额外要求真实 `robust_gate_pass=true`。未通过时保持拒绝。

qualification 所在目录还必须有覆盖实际资格文件的 `SHA256SUMS`，原 p5 claim 在消耗真实尝试之前核验该清单与 formal 清单。readiness 可放在独立文件，再由根任务完整归档。

```bash
python3 scripts/p6_minco_sitl.py readiness \
  /absolute/path/pilot_readiness.json \
  --case /home/qin/data/uav_usv_mpc/data/experiments/20261014_p6_sitl_01/pilot \
  --verification /absolute/path/verification.json \
  --qualification /absolute/path/qualification.json
```

readiness 绑定案例与配置清单、verification、qualification 和完整运行源码清单，覆盖 IMM、robust、adaptive、p6_follow、Planner/Tracker core、launch、脚本及消息。任何后续修改或源码清单增减都会使 readiness 失效，必须先补对应测试/审查，再生成新 readiness；不能仅重写哈希追认旧资格。

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

P6 condition 中的历史合同明确记录为 `baseline_reference_constraint_fingerprint`；`constraint_fingerprint` 与 `actual_constraint_snapshot` 由保存的完整 research 参数、本地 ResearchConfig、原相机常量以及 P6Policy/校准构造，不能用历史 raw 的 fingerprint 冒充新 robust/adaptive 合同。P6 A/B 关闭增强时，实际 fingerprint 必须逐值等于原 raw；不相等则在消耗尝试之前拒绝。已对真实 R11 research/condition 做该一致性测试。无 P6 overlay 的旧路径继续保持原字段与行为。

开发证据在 `data/experiments/20261014_p6_development_01/sitl_driver/`：首轮 RED，以及部分窗口、record 开关、配置资格绑定、完整参数和实际合同的 RED→GREEN。`green_full_parameters.log` 为 83 passed（P6 driver、P5/P53 和 pep257）；最后再联合 IMM 数学/Node 复核，`green_final_binding_parameters.log` 为 147 passed。flake8 与 diff 检查通过。测试包括完整配置复制/默认物化、五模式、A/B/C/D、校准同源、独占目录、旧默认委托、60 秒后半段 owner 丢失、readiness 变更拒绝、先行真实报告门禁、显式执行和环境恢复。

测试中的合成外部进程返回值仅用于验证驱动拒绝错误，不能用作任何真实 pilot、baseline、IMM shadow 或飞行资格。该驱动为本人实现，测试与代码核对属于自检；独立审查由根任务另行完成。
