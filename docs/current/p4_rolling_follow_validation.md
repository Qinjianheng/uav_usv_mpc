# P3.3 → P4 滚动 FOLLOW 研究验证

## 本轮结论与验收等级

实际起点为干净 master `6a25c195e6019fcaafb9cf19140140ad5c088c31`，不依赖旧报告推断版本。本轮完成进展型短时域研究、完整 FOLLOW 消息、纯协议、实际 Tracker 拒绝 ACK、同输入消融和原 FOLLOW/影子 SITL。**达到研究算法与影子验证层；未达到受控 MINCO 接管或稳定 MINCO 闭环层。** 安全门禁失败后按用户终止规则关闭接管，继续完成可独立验证的工作。

原 FOLLOW 的未来 P/V/A 参考尚不存在，首次桥接没有证明；目标预测、PX4 跟踪、姿态和连续时间 FOV 的独立持有包络尚不存在。冻结 489 请求中理想规划姿态与配对实测姿态角差 P95=0.07381 rad，超过假设 0.03 rad；这还是经验差异，不能替代认证误差界。150 ms 执行 lead 比原始输入期限晚 51–146 ms。未增大 125 ms、未将 valid_until 改写到曲线末端、未通过真值修正在线数据。

`follow_minco_enabled=false` 保留原行为；即使请求 true，资格变量仍 false。没有安装实际 MINCO 执行分支，也没有声称实现 MINCO→原 FOLLOW 的连续命令回退。当前可继续推进持有资格、共享 epoch 与第一段桥接，暂不具备实际控制接管条件。

## 架构与兼容性

BCTRA/KF/相机/旧截获数值语义不变。新增 ProgressFollowSolver 仅在新 P4 配置中选择；原 ShortHorizonFollowSolver 保留对照。利用既有 MINCO 系数求解与完整验收，没有新增 MPC 求解器或 PX4 发布者。

FollowTrajectory 提供完整局部 NED XYZ/yaw 曲线，FollowPlanAck 由实际 Tracker 发布。消息与旧 InterceptTrajectory 隔离。Tracker 既有方法按 AST 比较只有构造函数新增 opt-in 接收端；新增两个回调/上下文方法，timer、原 FOLLOW、yaw 和截获路径保持原 AST。控制状态不增加虚假的 ACTIVE 分支。详见 [接口契约](p4_follow_interface_contract.md)。

规划器最多一个 worker，busy 时不排队；最新订阅缓存供下一次构造请求。10 ms 完成轮询，Tracker 原 20 Hz。完成、序列化、发布分别保留原始 age/version/start 门禁。实际活动轨迹为零，因此当前没有可消费的正式 active tail；若将来启用，必须另接收端拥有的计划同步。

## 进展型目标与候选

后方参考 `r=p_target−5 e(heading)`，侧后方通过 beta=±0.6 的偏移及解析参考导数生成。目标包含归一化末端 P/V/A、积分 jerk、jerk/视点切换和视场软惩罚。P/V/A/jerk 基准为 5 m、3 m/s、3 m/s²、6 m/s³；权重分别 4、2、1、0.03、0.01，视场软权重 0.05（角度归一化 0.1 rad）。终端加速度惩罚到零。

在常 jerk 局部族内，`pH=p+vH+aH²/2+jH³/6`、`vH=v+aH+jH²/2`、`aH=a+jH`，积分 `∫||j||²dt=H||j||²`。解析二次最优 j 后进行有限限幅/候选搜索，映射到既有三段 MINCO；不是一般非线性规划全局最优。后方 j 的 1/0.5/0 倍、两个侧后方候选和可选旧数学 j，最多六个候选。三点几何只排序，最终全部走独立完整采样 FOV 与解析 V/A/J 极值验收。

A/B 诊断当前是联合验收通过的充分证据，不分别计数“仅动态合法、但 FOV 失败”的中间候选。C 为末端位置改善>1 cm，或相对速度改善>1 cm/s，或已进入 0.3 m/0.2 m/s 邻域。C 是进展诊断，并非强制每轮位置单调下降；连续五次没有进展会标记 stall。D（实际长期闭环稳定）始终 false。数学 j 初值只在同 mission/frame/generation、来源和版本向前时使用；不授予执行权限。

## 单请求同输入消融

冻结 f3_short 全部 489 个请求，逐条保留来源行、SHA、原始 epochs/测量/相机。原输入 TTL 不变，离线墙钟冻结在原请求时刻，算力预算 0.5 s；所以可行率不能当成在线接纳率。最初回放使用优化前代码，源文件保存在 pre_bounds_sources/；后续包络实验单独对照。

|算法|H(s)|联合动态/FOV可行|C有进展|暖初值命中|总计算P95(ms)|
|---|---:|---:|---:|---:|---:|
|p32|0.8|344/489|未评估|未评估|22.18|
|cold|0.8|424/489|419|0|40.18|
|warm|0.8|424/489|419|423|40.43|
|p32|1.2|370/489|未评估|未评估|22.92|
|cold|1.2|423/489|423|0|39.90|
|warm|1.2|423/489|423|422|40.46|
|p32|1.6|393/489|未评估|未评估|25.45|
|cold|1.6|423/489|420|0|20.57|
|warm|1.6|423/489|420|422|21.98|
|p31|2.4|239/489|未评估|未评估|99.16|

P32/P31没有本轮progress诊断，C、zero-jerk及warm列“未评估”不能解释为零；当前不提供按统一C准则的旧算法对照。P31 使用不同的 2.4 s 强终点问题，239/489 不能用于宣称同边界胜出。新方法 1.2 s 可行 423/489，旧 P32 为 370/489；这是有限输入集上的算法结果。进展方法可行候选零 jerk 计数为零；合成稳态测试允许非零速度、零 jerk 合理保持。数学暖初值已命中但没有可验证的提速，暂未实现 Q/T 平移或真实 active tail 暖启动。

## 因果多周期回放

相邻预测按原时刻推进，每次从自己的上一条完整假想曲线采样 P/V/A/yaw/rate；保留原始测量 epochs。后续录像中真实 UAV 状态绝不充当新算法执行状态。旧假想曲线有效时新解失败继续取旧曲线，耗尽/数据间断即结束 episode，再明确以记录投影初始化新 episode。该开环理想模型无实际 PX4、执行误差、独立持有或接管。

|算法(H=1.2)|请求Hz|实际间隔中位(s)|可行/请求|episode|最长episode样本|自生成参考误差RMSE(m)|
|---|---:|---:|---:|---:|---:|---:|
|p32|2.0|0.639|136/164|29|21|3.883|
|cold|2.0|0.639|142/164|23|51|3.603|
|warm|2.0|0.639|142/164|23|51|3.602|
|p32|5.0|0.218|402/486|25|104|7.603|
|cold|5.0|0.218|440/486|17|470|4.648|
|warm|5.0|0.218|440/486|17|470|4.648|
|p32|10.0|0.218|409/489|25|105|7.506|
|cold|10.0|0.218|443/489|17|473|4.648|
|warm|10.0|0.218|443/489|17|473|4.648|

“10 Hz”只表示抽样请求目标，原记录约 4.6 Hz，不能制造缺失的新预测；2 Hz 实现约 1.56 Hz。连续假想交接最大残差为零，只证明完整数学曲线采样一致。5 Hz 进展方法仍约 4.65 m 参考误差，未证明始终收敛到 5 m 后方。各 episode 长度、首末 20 s 均值、最大误差与视点切换见 rolling_diagnostics.json；不是有界稳定性证明，不能与原控制实际 0.7 m 直接排名。提高频率会增加 CPU 与版本竞争，不保证更好效果。

![假想滚动和同输入计算对照](../../data/experiments/20261009_p4_rolling_follow/rolling_and_compute.png)

## profiler 与实时优化

独立 profile 20 请求：0.465 s solver 中 0.276 s 为完整验收，解析 derivative_peak 0.179 s，1386 次 polyroots；MINCO 矩阵求解不是已证实的唯一瓶颈。新增幂基→Bernstein 导数控制向量，凸包最大范数给出保守全时域上界，保留所有高次小系数及舍入 padding。上界足够小才免根搜索，不能认证时回退原极值；FOV 完整检查不省略。仅新进展 solver 默认启用，原 FastConfig 默认 false。

489 同输入 1.2 s 配对测量：原根搜索 P50/P95/P99=10.54/37.49/40.36 ms，新包络=9.46/33.35/35.64 ms，P95 降约 11.0%。可行数均423，准入状态及可行 XYZ/yaw 系数逐项相同。结果是该冻结集、当前平台一次配对测量，不是平台最坏时延保证。

在线直线影子整周期 P50/P95/P99=35.26/55.51/57.84 ms；八字35.29/59.02/76.96 ms，最大98.01 ms。初始化 P95 约28–30 ms，新增视点和粗几何带来成本，不能把此前 P32 3.5 ms 初始化直接当相同算法。详细时间层、接收/ACK、CPU和实际执行见 [仿真比较](p4_closed_loop_comparison.md)。BCTRA 八字纯 Python compute P95 3.02 ms，而图像采集到预测接收 P95 43.76 ms，主要还有采集/接收、pose因果等待、调度；本轮没有靠改戳或放宽等待“消除”这些延迟。

## 验证、故障与未执行项

纯测试覆盖完整系数/非法布局、原始 TTL、绝对 epoch 精度、未来输入、任务/时钟/预测变化、拒绝保留、旧曲线采样交接、PVA/yaw/rate、非有限资格、missed start、到期、不续期、目标锁定/模式与唯一接收端。正常 ACCEPTED/ACTIVE 生命周期仅用显式合成 verifier/bridge fixture。另测进程 PID/start/PGID 重用保护及原生 Ruby Gazebo 启动识别。

实际故障只注入研究节点 SIGINT：原 FOLLOW 继续输出。没有在获准 MINCO 下测试失锁、停止预测、OFFBOARD异常或连续回退；这些只做纯协议输入测试。原 Tracker 对过期 UAV 状态返回 WAITING_FOR_STATE、停止当轮 setpoint 的既有问题尚未解决，不得声称所有故障不会断流。C–F 接管/更复杂机动和实际MINCO故障按门禁停止，不能用影子通过冒充。

初次仿真 localhost-only DDS 发现失败保留；改为与原环境一致的 DDS 传输、独立 domain 后重试。三轮 recorder 在完成采集后出现 Gazebo pybind 清理 abort，原始日志保留；新增显式 unsubscribe 后隔离 5 s/140 clock 样本 probe 退出码0。该修复未重新执行整轮飞行，不能追溯宣称三轮干净退出。333 有效研究事件重新编码后无 INVALID_INTERVAL/TTL_EXTENSION，旧现场数值拒绝仍保留，见 wire_encoding_recheck.json。

## 最终十二项回答

1. **真正接管：否**。达到研究/影子层，未达到中间/最终等级。
2. **唯一控制权：是，限本轮已观测图与源审计**；三个 PX4 输入话题始终只有原 Tracker，没有外部硬件操作。
3. **125 ms：保持**。原采集戳、完整 raw TTL 检查和新鲜时发布/接收拒绝测试均在，未以未来覆盖代替。
4. **ACK与持有：实际 REJECTED ACK 完成，独立持有未完成**；实际 ACCEPTED/ACTIVE 均0。
5. **持续跟随：具备局部进展和理想滚动连续性，实际长期能力未证明**。
6. **切换冲击：数学 PVA/yaw/rate 残差通过；未执行新的实际切换，不能判断冲击**。
7. **八字实际误差：原 FOLLOW 20–60 s 水平参考 RMSE 0.703 m**，不是新 MINCO 效果。
8. **优于原 FOLLOW：未证明**。同输入可行率和解析验收耗时改善只适用于算法；两轮真实直线都用原控制。
9. **jerk/FOV/安全：研究曲线接受完整名义验收；实际误差/姿态/整艇/连续 FOV尚无保证**。原基线个别时刻整球裕度为负；不可宣称所有时刻硬约束满足。
10. **平台稳定性：5 Hz shadow 与原20 Hz输出完成有界运行，有可复核尾延迟与负载；不保证10 Hz或最坏实时性**。
11. **SITL边界：红球、指定光照/场景、理想姿态、有限窗口与本机负载**；不外推无标记整艇、风浪遮挡或实机稳定性。
12. **实机阻碍：独立误差包络和持有、首次桥接、共享时钟代、20 Hz调度交接、预测更新重验、姿态滞后、控制超调、失效有界输出和完整接管/故障对照**。

## 文件与复现入口

新增代码：controllers/progress_follow_solver.py、p4_follow_planner_node.py、follow_transport.py；guidance/follow_contract.py、polynomial_bounds.py；FollowTrajectory/FollowPlanAck 消息；专用 P4 yaml/launch；6 个 test_p4/progress/contract/hypothetical/bounds 测试文件；p4_follow_replay/bounds_benchmark/follow_analysis/shadow_monitor/sitl_session/terminal_wrapper 脚本。

现有修改只涉及：setup entry、研究 solver 选择、可选 Bernstein 预检、hypothetical_request 类型、yaw 起始 rate 接口、Tracker opt-in 拒绝接收端、消息 CMake、测试分层脚本及文档索引。baseline.yaml、PX4、模型、KF/BCTRA、MissionManager和原 tracker control 函数未改。完整新增/修改清单及参数备份来源见 evidence 的 final_sources.json / parameter_backup_sources.json。

验证命令：`./scripts/test_research_planners.sh algorithm|related|full`（分别执行），项目范围 `python3 -m flake8 src/uav_control/uav_control src/uav_control/test scripts`，仓库 build_workspace.sh、ROS接口/launch、所有 scripts/*.sh 的 bash -n、git diff --check / --cached --check。本轮最终计数在 verification_summary.json；原始日志分别 algorithm_final.txt、related_final.txt、full_final.txt、flake8_final.txt、build_final.txt。中途失败及 RED 日志不删除。

证据：`data/experiments/20261009_p4_rolling_follow/`，每轮保存 config、环境、PID/PGID/start、原始JSONL、CSV、stdout、ROS日志、ULog、watchdog_end。P32 1668、P31 445、冻结基线46 manifest条目本轮逐项SHA验证通过。最终 SHA256SUMS 从该证据目录验证；旧目录与历史原始数据不改写。无自动Git提交、推送或暂存。

## 本轮最终检查与审查

最终构建3包成功（31.0 s）；algorithm 294通过/3取消选择，related 455通过，full 1330通过/1跳过/2条依赖弃用warning。flake8规定范围、shell语法、ROS消息生成、launch参数/default-off、工作区prefix、Git差异检查通过。独立审查发现的大有限系数数值异常和未计算指标误记零已分别RED→GREEN修复；review_fix_tests.txt 33通过。全部证据是本轮运行结果，失败/RED不删除。

当前研究验收是离散FOV与名义动力学，不是独立持有资格。SafetyApproval的有限期限检查、不可计算系数的拒绝不会创建接管路径。现场附加节点源码快照缺口及monitor修复未重跑全飞行的边界明确保留；参见比较报告。
