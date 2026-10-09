# P3.1 快速可见性 MINCO：实现与验证

本轮从干净的 master / `d8f3dc2d0f97e5349cc36ea8d17cb64d34a22b6a` 开始。
完成纯算法、冻结输入回放、实际 PX4/Gazebo 影子验证。原 P3 默认算法保留；**尚不具备进入 P4 在线 FOLLOW 接管的条件**。
直线出现合法研究发布，但八字仍被真实计算预算和原始数据新鲜度阻断；所有 Tracker 接纳标志始终为 false。

证据目录：`data/experiments/20261009_p31_fast_minco/`，顶层 `manifest.json` / `SHA256SUMS`。
原 P3 的152个证据文件逐个重新计算 SHA256，全部一致，见 `frozen_p3_check.json`。
本轮未提交、暂存、推送或访问远端仓库；旧工作区只读。

## 原 P3 逐轨迹归因

`audit_v2/per_cycle.jsonl` 保存 F3 全部67个完成事件的原始请求、完整原事件、重建初值、初值验收和解析极值。
`candidate_costs.jsonl` / `candidate_summary.json` 保存候选位置、速度、加速度、方向、投影裕度、代价和拒绝原因。
历史没有记录 worker 的 retained direction、所有候选和中间迭代，因此它们是**以 retained=0 重建**，不是虚构的历史内部记录。
历史最终 Q/T/yaw/系数、源戳、任务/时钟 generation、发布拒绝均来自原 core_result 和事件。
历史未记录项（单次线搜索细节、超时中间系数等）仍标记不可恢复。

|互斥分类|数量|含义|
|---|---:|---|
|A|18|初始化无粗可达可见候选|
|B|0|重建初值可行，但历史优化后不可行|
|C|3|重建初值不可行，历史优化后核心可行|
|D|38|两者不可行|
|E|6|重建初值及历史核心可行，但发布时效拒绝|
|F|2|历史优化超时|

C 的3条也全部发布被拒；时间轴上共9条核心可行、0条发布有效。
原先 `audit/` 错读了 publication output（其指标被准入裁剪），明确由读取 core_result 的 `audit_v2/` 替代。

**38条动力学失败的最大水平 jerk 全部出现在轨迹起点。**
有最终系数的47条中，46条峰值在起点，1条在内部。不能把它们统一归因于末端速度设为目标速度。
初始 P/V/A 与固定5m航点的连接、时间分配及终端耦合必须共同分析。
极值由 jerk 二次多项式模平方的导数实根求得；检查每段端点、内部根和交界两侧，不依赖采样最大值。

重建共519个候选（包括被强制成同一终点的重复候选）：后方279、侧后方240，粗可行253。
粗拒绝计数允许一候选多原因：水平加速度259、水平速度131、yaw rate65、倾角40、整球可见性40、推力16。
原 retained 始终来自0/±0.6，因而没有新增第四种方向，只影响切换代价；末端全部强制后方。
三固定角可以保留为对照，没有证据表明它们是唯一必要选择。

## 性能 profile 与矩阵

`audit_v2/legacy.prof` / `profile.txt` / `summary.json`：选前8个冻结请求，用 simple 初值依次运行 fixed/Q/QT，共24次，profile专用预算3秒。
这组 profile 耗时包含 cProfile 开销，不能当作线上500ms性能结果。实际同输入计时来自独立回放。
首批 profile：872次 evaluate、74次有限差分梯度、14,461次 planned_attitude/完整相机投影；有限差分累计约1.55秒，完整相机投影约1.18秒。
矩阵构造、系数映射、jerk积分、yaw构造、采样和验收计数/累计时间均保存在函数统计中；嵌套时间不相加。
历史单次线搜索次数未保存，profile也没有将 FORTRAN 内部试探伪装成可观测次数。
新引擎记录 objective/gradient、局部 complex-step、矩阵构造/伴随求解、yaw构造、批量几何、迭代及 nfev/njev。
运行时不启用 cProfile，仅使用轻量整数计数和阶段计时；完整 profiler 通过独立审计工具启用，不放入 ROS 回调。

18×18、三维 RHS、300次串行测量的中位数（µs）：

|方法|未缓存逆|NumPy solve|SciPy LU|banded|缓存逆|缓存 LU|
|---|---:|---:|---:|---:|---:|---:|
|耗时|6.59|4.73|7.63|8.55|0.92|2.84|

残差见 `offline_solver/matrix.json`。Q变化可复用原缓存映射，T变化会失效；小系统并不是原主要瓶颈。
原 MincoS3Trajectory API、数学映射和截获路径均未修改。
本轮不引入 C++/Eigen 构建。先处理候选初始化、Python/GIL、重复序列化和交接协议；有明确 kernel 负载证据后再考虑迁移。

## 实现与数学

新增纯模块：

- `polynomial_extrema.py`：速度/加速度/jerk 的实根极值和峰值位置。
- `follow_reference.py`：显式观测点 P/V/A、局部预测拟合和低速退化。
- `adaptive_follow_initializer.py`：A后方、B三固定角、C自适应角、D角+距离；最多15个候选/航点，3个航点。
- `fast_minco_objective.py`：批量完整规划姿态/相机外参代理和混合伴随梯度。
- `fast_follow_minco.py`：先验收固定曲线、保留可行解、快速返回和有限预算的 Q→QT 升级。

新的默认研究入口参数单独在 `p31_follow_research.yaml`；原 `follow_research.yaml` 和普通飞行入口不变。
通过旧研究节点的 `minco_engine=p31` 显式选择，复用一个 worker、同一 adapter/runner/发布检查。
没有新 ROS 消息、控制节点或 Offboard 发布者。
原优化器仅增加默认关闭的 `feasible_priority` / `fast_feasible_seed`，供隔离消融；旧数值默认路径不变。

设 `e=(cos θ,sin θ,0)`，`n=(-sin θ,cos θ,0)`，`θ=ψ+β`：

```
P = P_target - d e
V = V_target - d_dot e - d theta_dot n
A = A_target + (d theta_dot²-d_ddot)e
    - (2 d_dot theta_dot+d theta_ddot)n
```

恒定高度使 `Pz=-5, Vz=Az=0`。固定后方时得到题设的 `V_target-dωn` 与 `A_target+dω²e-dαn`。
纯函数接受显式 d/β 及其导数，不猜测真实平台参数。
当前预测消息只有 P/V，没有可信角加速度：使用±0.4秒、至少3个有效预测点的二次 heading/speed 拟合，记录残差与裁剪。
研究导数上界取 UAV 加速度/jerk限值除以速度；这是**明确的参考模型约束**，并非已校准的 USV 物理上界。
低速<0.2m/s回退当前参考 yaw、零角速度/角加速度。
该参考是局部模型的微分状态，不声称分段线性 P/V 插值全局具有同样导数。
终端候选假定 β/d 在终端局部不再变化；航点之间由实际 MINCO 曲线衔接。

距离研究默认区间3–10m，作为独立 GreedyConfig/ROS研究参数暴露，可在含5m的该区间内收缩。
相机原轴向0.05–25m、目标半径0.25m、安装平移(0.18,0,0.39) FLU、下倾28°、水平/垂直裕度0.07rad均保留。
距离不能替代整球视场、海面和动态验收；它也不是已验证的真机跟随距离范围。
统一跟随误差代价仍相对原5m后方参考，候选变化另报 RMSE，不补造新的飞行验收阈值。
评分含相对运动、粗速度/加速度、参考侧向机动、yaw变化、切换惩罚和局部终端 PVA 兼容性。
本轮采用线性数量局部兼容检查，未做指数搜索或声称完成全局最优前瞻。

规划姿态仍为 `b3=normalize(g e_down-a)`、`b2=normalize(b3×heading)`、`b1=b2×b3`。
实际 PX4 姿态、该理想规划姿态、相机安装姿态分别保留；完整最终验收使用原 P1 投影。
代理用完整旋转、安装平移和整球到四个安全视锥面的距离，另含前方深度与近/远距离；其可见符号与 P1 测试一致。
代价采用每段两点 Gauss积分；动力学/海面平方违约和 softplus 可见性只用于搜索，不能代替硬验收。
最终维持原稠密/自适应采样、PVA边界和C0–C4连续性、yaw解析峰值验收，并增加平移导数解析极值预拒绝。
动态预拒绝不计算完整 FOV 的结果明确标记 `full_projection_validated=false`。
仍不提供遮挡或连续时间 FOV 安全证明。

对 `M(T) C=B(Q)` 解 `Mᵀλ=∂J/∂C`，Q梯度为对应两个航点 RHS 行的 λ 之和；
T梯度包含显式 jerk积分边界、积分权重、物理采样时间、目标插值时间变化以及 `-λᵀ(dM/dT)C`。
局部 smooth代价偏导用 complex-step，避免每个优化变量重建整条轨迹；clamped yaw 的 T敏感度剩余6次小样条中心差分。
L-BFGS-B显式 `jac=True`，代价/梯度共用系数和中间量。
这属于混合梯度，不冒称所有环节纯解析。
五类独立中心差分最大绝对误差分别约 `6.5e-11/7.8e-7/1.43e-5/3.24e-10/1.10e-8`，最大归一化相对误差 `1.67e-7`。
另测不等 T、logit边界±1.499、yaw wrap、FOV边缘、高jerk、静止/低速。
插值节点、yaw unwrap分支、范数零点和平方违约分界采用局部单侧/零子梯度约定，不声称在不可微点存在唯一梯度。
首末 yaw rate仍为0；现有接口缺少可信实际/交接 yaw rate，未达到 Tracker 偏航动态一致。

## 离线消融（真实同输入串行）

20个冻结生成场景涵盖静止、直线、左右转弯、八字、加减速、转率突变、错误yaw、视场边缘/失视、距离、陈旧/epoch错误等。
F2取冻结前20条完成请求，F3取全部67条。重复梯度报告错误通过 post-only补算，没有重跑已完成的优化。
原P2 selector的20条上限保留；P31审计工具自有最大100条选择器，仅用于完整研究审计。

|变体|synthetic20|F2前20|F3全67|F3优化器P95秒|
|---|---:|---:|---:|---:|
|A 原P3|2|11|11|0.147|
|B 仅原代价+可行保留|2|11|11|0.181|
|C 改动态终端|2|11|11|0.147|
|D 自适应角、同终端|2|11|13|0.158|
|E 自适应距离、同终端|2|11|13|0.155|
|F 自适应终端+原优化器|2|11|23|0.165|
|G 混合梯度、同终端|5|11|17|0.177|
|H 快速验收/可行优先、同终端|5|11|17|0.085|
|I 全部改进、改终端|5|11|29|0.085|
|simple+快速|5|11|17|0.086|

表内同边界组不混入改变终端/时域组；F/I增益不能单独归因于求解器。
计时为 optimizer.solve，初始化另存；总体含无效输入早拒，不能用混合P50宣称所有有效规划只有微秒。
质量、整球比例/角裕度、速度/加速度/jerk、积分、RMSE、Q/T/yaw、调用计数、失败/超时均在 records.jsonl。
解析预拒绝案例没有伪造 FOV/RMSE；缺失指标需按有该指标的样本数分析。

额外 `offline_fastpaths` / `replay_f3_fastpaths` 隔离原代价的快速路径、保留、两者结合及动态终端。
原版/保留/快速/两者/动态终端在 synthetic均2/20、F3均11/67；单加初值验收还会增加失败曲线的验证成本。
旧 MPC 本轮 synthetic通过9/20；F3 FutureRequest全部接口拒绝0/67，未伪造匹配的时间戳。

终端策略A/B/C/D在synthetic各5/20；D延长到2.8秒为7/20，属于时域变化组。
2.8秒仍检查原预测窗覆盖，未扩大预测数据窗口；这种收益不能混入同2.4秒结果。
代价消融（均保留同样最终完整FOV硬验收）：jerk+dynamic、+follow、horizontal、joint-yaw、visibility尺度2各5/20；固定yaw/full为4/20。
两次短迭代下，soft可见性、yaw耦合和不同量纲尺度会改变搜索方向；强罚项不保证可行率提高。
有限差分误差不是唯一解释；可行保留、采样/积分设计、终端选择和迭代受限也有贡献。

## 实际 PX4/Gazebo 影子实验

只启动一个研究规划器，原飞行配置逐字复制P3对应F2/F3配置；没有Y操作。
每次 watchdog记录四组件PID/PGID/start identity，有界结束；没有广泛pkill。
F2为160秒原完整图→只关闭三个已拥有诊断节点；F3首次180秒失败，修复后独立180秒轻量图复跑。

|本轮|完成轮|核心可行|最终研究发布|全周期P50/P95/P99秒|初始化P50/P95秒|
|---|---:|---:|---:|---|---|
|F2直线|122|17|16|0.0623/0.0833/0.0907|0.0137/0.0285|
|F3首次|0|0|0|研究节点退出|不可用于性能结论|
|F3修复复跑|146|0|0|0.0712/0.0954/0.1280|0.0529/0.0731|

F2核心：10无粗候选、10无新鲜预算、17可行、13动态失败、72 deadline；17条均跳过优化，1条最终发布被拒。
F3复跑：102无新鲜预算、21 deadline、23动态失败。初始化后的剩余新鲜度中位数仅约6ms（146条完整统计），自适应初始化在实际GIL/回调负载下已消耗大部分预算，不能将离线29/67迁移成在线结果。
原P3发布0，本轮F2出现16条真实发布；运动、负载与时间长度不同，不能据此声称严格统计提升或八字已解决。

F2首次 CLI daemon未发现 offboard topic，相关后续probe/load未执行；失败保留，后续F3 no-daemon重试成功。
F3首次节点exit1，完整Python stderr没有保存。测试复现 `bool_` JSON序列化异常；转换原生bool后19项专项测试覆盖，复跑节点存活。
不虚构历史异常堆栈。
F3 moving_target日志还含Gazebo pose更新超时警告，未验证整段标记模型严格执行理想八字；保留原始日志，未用真值补偿研究输入。
红球测试仍只验证感知接口，不外推无标记非合作目标性能。

实测F2完整14节点、F3轻量11节点。
F2完整20秒CPU合计339.6%（单核100%）、RSS合计1390.6MiB、研究节点CPU63.4%。
F3轻量20秒CPU268.0%、RSS1157.3MiB、研究节点CPU57.0%。两种运动不同，不作严格负载降幅结论。
F2 light窗口与watchdog结束重叠，仅11/20样本在结束前，**不使用其混合均值宣称轻量负载**。
RSS是逐进程求和，不是独占物理内存；QGC/Codex/独立CLI不计入四个owned组。

F3复跑三个 `/fmu/in/{trajectory_setpoint,offboard_control_mode,vehicle_command}` 各1发布者，均为原 Tracker。
研究节点只发布研究JSON/参数事件/rosout，不订阅 `/target/state`；prediction发布/订阅均BEST_EFFORT。
证据为 node info、topic info和 prediction_qos.txt；原主视觉/KF/BCTRA/Planner/Tracker链未删除。

## 新鲜度与未来交接

```
fresh_until = min(nav_stamp+.125, observation_stamp+.125,
                  prediction_source_stamp+.125, prediction_valid_until)
remaining = fresh_until - current_time
```

初始化后使用实际系统时间再计算，预算为 `min(original_budget, remaining-20ms)`，预留约25ms完整验收。
这些是研究调度预留，不是保证最坏时延；publication继续独立检查原125ms门限、任务/generation、最新预测失效与执行起点。
预算不足/完整验收未完成/过期必须拒绝，不能以500ms配置当作线上可用时间。
原始戳、工作开始/结束、发布、remaining与执行起点差值保存在原事件及 freshness_analysis.json。

F2全部122轮、F3复跑全部146轮，**prediction_valid_until早于150ms未来执行起点**。
研究publication valid只表示发布时输入/候选满足现有研究准入；它不授权未来执行。当前研究数据的expires_at仍早于执行起点。
必须分开：原始观测新鲜度、未来预测覆盖、已接纳轨迹合法持有/接替；4秒预测覆盖不自动延长125ms有效期。

P4之前需设计并验证：

1. Tracker提供实际交接时刻及已接纳轨迹期望P/V/A、yaw/yaw rate、轨迹ID/generation；不以旧未来终点统一重启。
2. 由实际剩余预算安排候选完成和交接时间，保证准入与预测有效契约可同时满足；不能固定150ms然后假装TTL满足。
3. 候选持有、失效、预测更新再验收必须是明确协议；失效候选不延长旧轨迹deadline、不刷新原观测戳。
4. 若要引入独立预测可信时域/已验证轨迹持有契约，需另立安全设计及连续性实验。本轮没有直接延长TTL或绕过检查。
5. 批量化候选几何/初始化、JSON复制与GIL调度，再按同输入真实负载测最坏耗时；未标定姿态误差、风/拖曳和yaw边界仍需解决。

因此本轮按任务要求停止继续优化；下一阶段优先交接协议与实际初始化预算，不接管FOLLOW，也不继续PNG–IBVS。

## 检查、文件与可复现入口

新增文件为上述5个算法模块、`scripts/p31_minco_audit.py` / `p31_minco_experiment.py`、19项专项测试、独立研究YAML及本报告。
既有改动仅涉及研究solver/node、原研究优化器的默认关闭开关、研究launch参数路径、分层测试脚本、文档索引。
源码快照添加 COLCON_IGNORE，避免证据被当成重复包；冻结历史152文件不变。
最后将既有3/10距离常数暴露成参数，默认值与实测代码相同；20条本轮F3请求的新/快照初始化输出逐项完全相同，见 default_parameter_equivalence.json。

最终新鲜检查结果见下方验收记录（历史1211通过不作为本轮证据）。
首次完整pytest为1226通过/3失败/1跳过：continuation格式、两项D213文档字符串以及快照包扫描问题；都保留并修复，未删除旧测试。
本轮构建用现有 `build_workspace.sh --packages-select uav_control uav_usv_bringup`，2包通过；最终overlay前缀在新工作区。

复现示例（输出须是新的目录，不覆盖现有证据）：

```bash
OPENBLAS_NUM_THREADS=1 python3 scripts/p31_minco_audit.py \
  data/experiments/20261008_p3_follow/f3/shadow/*.jsonl <new-audit-dir>
OPENBLAS_NUM_THREADS=1 python3 scripts/p31_minco_experiment.py <new-offline-dir>
OPENBLAS_NUM_THREADS=1 python3 scripts/p31_minco_experiment.py <new-replay-dir> \
  --input data/experiments/20261008_p3_follow/f3/shadow/*.jsonl --maximum 67
./scripts/test_research_planners.sh algorithm
./scripts/test_research_planners.sh related
./scripts/test_research_planners.sh full
```

采用单一修改者，自审按接口、硬验收、时间戳和默认兼容逐项检查；没有独立代理复审。
裁决：保留旧默认；局部拟合导数明确为研究假设；未来执行TTL不自作放宽。若假设或时序错误，独立验收仍须拒绝，不能影响原控制链。

设计参考：[Adaptive Tracking and Perching](https://arxiv.org/abs/2312.11866)、[Geometrically Constrained Trajectory Optimization](https://arxiv.org/abs/2103.00190)、[作者GCOPTER代码](https://github.com/ZJU-FAST-Lab/GCOPTER)。
本轮仅参考其动态边界/稀疏轨迹思想，通过网页读取原始来源；未clone/fetch或移植外部优化器。

### 九项研究结论

1. 三固定方向不是必要条件；保留为稳定对照，retained不额外增加方向。
2. 自适应角在同边界F3为13/67对11/67，改变终端进一步提高；仍不能宣称在线八字更好。
3. 终端PVA不是这批jerk失败的唯一主因：38条失败峰值均在起点，动态终端单改未提高通过数。
4. 快速组合把同边界F3优化器P95从约147ms降至85ms，但梯度单改未统一加速，线上初始化仍明显拖慢。
5. 已通过完整验收的固定曲线直接返回；F2实际17轮跳过优化。不可行初值不跳过硬验收。
6. 无需每轮完整QT/yaw优化；Q→QT仅在剩余预算足够时升级，当前短迭代不保证困难曲线修复成功。
7. 125ms下直线研究发布16/122，八字0/146；不能以离线可行率作为合法发布率。
8. 不立即引入C++/Eigen；矩阵微秒级，先批量候选/调度/序列化及交接契约。
9. P4暂不准入：未来TTL与150ms执行起点冲突、缺少实际接纳轨迹交接PVA/yaw-rate、八字初始化预算及姿态误差模型尚未解决。

### 最终本轮验收

- 19项新专项测试通过；algorithm 202通过/3 deselected（2.14s）；related 345通过（3.78s）。
- 从 src/uav_control 跑全pytest：**1230通过、1跳过、2条既有deprecated警告，10.71s**。跳过项为既有 copyright 测试的显式装饰器（generated source无版权头），不作为新算法通过数。
- style修复专项3通过；项目范围flake8、shell语法、launch --show-args、diff --check及cached --check通过。
- 2包工作区构建通过；最终包前缀 /home/qin/data/uav_usv_mpc/install/uav_control。
- 未执行飞行接管、真实USV试验、碰撞/遮挡连续时间证明、C++迁移或PNG–IBVS；这些超出本轮范围。
