# P6 匹配离线规划消融与压力测试

入口为 `scripts/p6_follow_ablation.py`，执行实际 KF+BCTRA、IMM、P6Runtime、P47/C 主规划和 P44/D 后备。所有结果仅代表**合成观测下的独立规划窗口与理想 MINCO 曲线**；没有 PX4 响应、实际图像失锁、Tracker 接管或真实闭环资格。

## 预注册条件

正式协议使用独立噪声 seed **1913**，不同于最终预测研究的 calibration=127 和 heldout=3731；程序读取两个校准文件并拒绝种子重叠。五模式全部使用当前 `baseline.yaml` 的实际运动/KF/BCTRA参数及完整 IMM 配置。原始位置噪声为 xyz=(0.1,0.1,0.05) m iid Gaussian、20 Hz、2%丢帧；同一原始位置序列同时进入 KF+BCTRA 与 raw-position IMM，不向估计器提供路线或真值。

固定规划时刻为启动后 **8、16、24、40、56 s**，各时刻此前 1.6 s 以真实 5 Hz epoch 推进观察点 profile。五模式、五时刻和下列协议产生 400 个尝试，丢帧/超时/无候选均保留。虚拟 UAV 初值对所有预测器共用因果 KF 估计后方5 m、飞行高度-5 m；它是受控的合成导航边界，不被描述为实测导航或连续飞行。

| 组 | 预测器 | 风险几何 | 自适应观察点 |
|---|---|---|---|
| A | KF+BCTRA | 原名义 | 关闭 |
| B | IMM | 原名义 | 关闭 |
| C | KF+BCTRA | 校准经验包络 | 开启 |
| D | IMM | 校准经验包络与全部条件模型 | 开启 |

每组均匹配相同原始观测 hash、初始 UAV 边界 hash、源 epoch、预测采样窗、P47 H1.6/QVA参数和 P44后备。额外两类压力事先固定为初始侧向偏移2 m、初始yaw偏差0.25 rad，A/B/C/D全部覆盖。它们不根据结果挑选，也不是实际视觉yaw执行误差的识别结果。

正常条件另外运行 D 分别移除 covariance、条件模型、yaw/姿态工程界及 adaptive 四种组件干预。移除条件模型保留 IMM 身份，触发本地缺失分支检查及显式 nominal fallback，不能改名绕过门禁。移除 covariance 或误差界的结果明确 `robust_claim_eligible=False`，不继承完整增强资格。

## 时效与评价

两个预测器的实际 CPU 耗时计入统一递送延迟，所有对照组等待二者均准备好的同一边界。最终 prepare 与 solver 的真实 `perf_counter` 流逝继续消耗原125 ms输入期限；没有固定墙钟、延长过期时间或重试失败轮。只有与目标无关的固定 MINCO 线性映射在全体组执行前统一预热。

规划完成后才以隐藏合成真值评价理想曲线。记录全目标可见比例、最小水平/垂直角裕度、原5 m后方任务误差、增强参考误差、加速度、jerk、yaw rate、prepare/solve/total耗时、风险报告及原始拒绝原因。结果保存原始请求、策略、预测身份、曲线系数、约束指纹和校准文件SHA。

summary 中所有尝试均计入拒绝率；飞行误差/FOV均值显式给出合法曲线条件下的分母，不能把被拒绝窗口删掉后宣称整体100%。失败轮若存在未通过的系数，仍在逐尝试记录中保留其诊断。最终校准为 xyz 合成经验包络；小概率条件模型的统计适用范围缺口、理想姿态和缺少真实视觉回路限制持续保留。

## 重放与产物

```bash
cd /home/qin/data/uav_usv_mpc
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
python3 scripts/p6_follow_ablation.py \
  --include-lease-prefix \
  --include-fixed-profile \
  --output data/experiments/20261014_p6_development_01/visibility/ablation_02
```

非空目录直接拒绝覆盖。后续重放应指定新目录；`protocol.json`、`attempts.jsonl`、`*_prediction_snapshots.jsonl`、`raw_observations.jsonl`、`*_parameters.json`、`summary.json`与`SHA256SUMS`提供复算入口。

`visibility/ablation_smoke_01` 是早期 linear/8s 的16条端到端烟雾测试。它使用经验半径修正前的校准SHA，原始结果保留，仅验证管线可执行，不作为正式风险或效果资格。`ablation_01` 则使用 `prediction_02` 的 xyz 修后包络（`empirical_scale=2.7954834829151074`），完成400条；其后独立 CTRA 极低速 Jacobian 和停止过程可观测性修复改变了预测器版本，最终代码须使用 `prediction_04` 并另存 `ablation_02`，不得覆盖旧对照。

`--include-lease-prefix` 在同次冻结预测/递送epoch/执行起点上额外增加 C_lease_prefix、D_lease_prefix，各正常条件25条。原400条矩阵显式固定 `risk_horizon_seconds=0`，额外50条设`.45`。新增scope报告不声称完整1.6 s鲁棒通过；它只评价原不可续期`.45 s`实际执行权对应的工程策略。

`--include-fixed-profile` 另增加正常条件25条 `D_fixed_profile`，关闭观察点选择，显式固定 `d=5 m,beta=0`，保留解析参考 PVA 与全部 cov/model/yaw 项，风险仍为全1.6 s。增强参考的转弯速度包含 `-d*heading_rate*n`，原 FOLLOW 前馈只有目标速度；该对照用于分辨参考运动学前馈与实际 d/beta 变化的贡献，不能把 D/B 全部差异归因于自适应距离。两项扩展均启用时共475条，原400条矩阵保持。

`ablation_01` 的400条均通过原硬门禁，但正常C/D各25条新增鲁棒检查全部失败，全部是显式 nominal fallback。D相对B的理想5 m任务RMSE均值为0.425/0.465 m，FOV均100%；C相对A为0.576/0.489 m，反而变差。D的P95计算约40.6 ms、B约13.6 ms。清零covariance可让25条几何通过，但该组件干预明确不具完整资格；不能将其宣称为鲁棒性收益。

同一冻结curve的纯诊断 `ablation_01_lease_prefix_diagnostic.json` 保留全部covariance/yaw/模型及原33点风险配置，只查询真实0.45 s执行租约前缀。正常D五模式各5/5前缀几何通过，完整1.6 s仍各0/5；C/BCTRA前缀仍各0/5。D前缀约2.8 ms、全窗约5.3–5.5 ms。此结果支持研究风险scope与实际短租约是否应一致，尚未改变policy，也不授予整条轨迹概率保证或实飞资格。

完整旧版数据见 `ablation_01/summary.json`、`ablation_01_analysis.json` 与 `ablation_01_failure_attribution.json`。真实五模式60 s闭环、实际失锁、控制权及可靠性结论仍待根任务单独验收。

## 最终冻结研究 ablation_02

最终475条均通过原名义硬门禁；75个匹配场景键的原始输入、共同UAV边界、source/observation/execution epoch、now与state逐项相同。runner和所列依赖源码在研究前后SHA一致，`ablation_02/SHA256SUMS`全部通过。校准为 `prediction_04`：BCTRA SHA `0d2351bc12a98aa877329c63f9f39f9bcc8e22c6867be722fcaa9428f2415784`，IMM SHA `162b42c5d4396b93ef81d8d3c690af216429f0d37a1e0c17588ca8d10ab38cdf`。详细分析在 `visibility/ablation_02_analysis.json`。

以下均为正常条件25条的均值；耗时为本机完整prepare+solve的P95。经验包络通过与原名义准入分开统计。

| 组 | 经验包络通过 | 理想真值5m误差RMSE (m) | 理想全目标FOV比例 | P95总耗时 (ms) |
|---|---:|---:|---:|---:|
| A | 名义策略 | 0.4892 | 0.99394 | 16.69 |
| B | 名义策略 | 0.4650 | 1.00000 | 14.14 |
| C，全1.6s | 0/25 | 0.5764 | 0.99515 | 36.12 |
| D，全1.6s | 0/25 | 0.4252 | 1.00000 | 43.07 |
| C，0.45s前缀 | 0/25 | 0.5774 | 0.99515 | 35.70 |
| D，0.45s前缀 | 25/25 | 0.4318 | 1.00000 | 39.92 |
| D，固定解析参考，全1.6s | 0/25 | 0.4318 | 1.00000 | 32.15 |

正常D前缀在五模式均为5/5；C前缀各0/5。主400条中所有C/D全窗检查均未通过，保留其全部 nominal fallback；侧移2m和初始yaw0.25rad压力也未建立全窗包络资格。C相对A的任务误差变差，不能将BCTRA增强组宣称为整体改进。D的全窗误差低于B，但仍没有全窗风险通过，二者必须同时报告。

固定解析参考与D前缀25条的全部xyz/yaw/duration系数逐元素完全相同（最大差0），说明本协议中D前缀选择保持原5m/beta0，约7.14%的B→固定解析参考误差下降来自增强参考分支及其解析前馈，不能归因于实际距离/侧角调整。D全窗相对固定解析参考的附加收益只有约0.0066m，且其全窗风险仍失败。

清零covariance组25/25几何通过，但标记无完整资格；移除条件模型组25/25显式 fallback；移除yaw/姿态界仍0/25通过。上述干预支持保留完整不确定性并明确风险范围，不支持缩小协方差、丢弃低概率模型或把经验包络当作真实统计保证。研究没有模拟车辆响应，原名义门禁预测目标通过也不等于隐藏真值全程可见，实际闭环仍须单独验证。
