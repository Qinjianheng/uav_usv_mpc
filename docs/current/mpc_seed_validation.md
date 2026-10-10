# P2：可见性感知 MPC 初始轨迹与影子验证

## 验收结论

P2 算法、独立研究节点和测试已实现；离线对照存在收益。**在线时效验收未通过，不能进入飞行控制集成。**
可以将本接口用于 P3 的离线 MINCO 二次优化研究；若 P3 要求在线 FOLLOW 接入，应先解决共同执行起点/输入配对与完整流程延迟，再复验。
本轮停止于 P2，没有实现新的 MINCO 优化器、MPC 控制接管或 PNG–IBVS。

起始 master/HEAD `b388847b17c8d207042cccc39407e7842e5f05c8`，初始工作区干净；未提交、暂存、推送或访问 GitHub。
旧工作区只读；原控制代码、默认 launch、baseline.yaml、相机/模型、PX4 源码、冻结证据保持原状。

## 文件与实现

新增纯算法：
- `src/uav_control/uav_control/controllers/follow_mpc_seed.py`
- `src/uav_control/uav_control/guidance/planned_attitude.py`

新增研究运行层：
- `src/uav_control/uav_control/controllers/mpc_shadow_inputs.py`
- `src/uav_control/uav_control/controllers/follow_mpc_shadow_node.py`
- `src/uav_usv_bringup/config/mpc_seed_shadow.yaml`
- `src/uav_usv_bringup/launch/mpc_seed_shadow.launch.py`
- `src/uav_usv_bringup/launch/mpc_seed_validation.launch.py`：仅 include 原 modular 入口和研究节点，显式设置新实验日志目录。

新增工具 `scripts/mpc_seed_experiment.py`、`scripts/mpc_seed_replay.py`；新增六个对应测试文件：
`test_follow_mpc_seed.py`、`test_planned_attitude.py`、`test_mpc_shadow_inputs.py`、
`test_follow_mpc_shadow_node.py`、`test_mpc_seed_experiment.py`、`test_mpc_seed_replay.py`。
唯一已有文件修改为 `src/uav_control/setup.py` 的两行 console entry。本文为新增报告。

### 动力学与优化

NED z 向下。状态为 P(3)/V(3)/A(3)/连续 yaw，输入为 jerk(3)/yaw_rate。
常 jerk 的精确离散积分为：

```
p+ = p + v dt + a dt²/2 + j dt³/6
v+ = v + a dt + j dt²/2
a+ = a + j dt
yaw+ = yaw + yaw_rate dt
```

SciPy SLSQP 优化分块控制；预测 12 步、dt=0.2 s（2.4 s），最终默认 2 块/4 次迭代，1 Hz 影子调度、0.5 s 全计算预算。
考虑后方和左右侧后方 ±0.6 rad 观测参考，先评分初值，再优化选中的参考。
代价含位置、速度、展开 yaw、softplus 几何裕度、加速度/jerk/yaw_rate 与相邻控制变化。
权重为当前研究常数：位置1、速度0.4、yaw2、可见性200、加速度0.03、jerk0.02、yaw_rate0.1、控制差0.04。
有限差分目标和约束的共同评估使用有界单轮缓存，直接复用 P1 evaluate_visibility，没有另一套投影模型。
迭代上限结束但严格可行的初值/结果可返回 SAFE；optimizer_success/status/nit/message 单独保存，SAFE 不等于求解收敛或全局最优。

速度 XY≤6.2、|Vz|≤4；加速度 XY≤3、|Az|≤3；jerk XY≤6、|jz|≤4；|yaw_rate|≤1。
海面 barrier 使用 reserve=0.5 m、下降响应0.15 s和制动2.5 m/s²，较保守地拒绝候选。
这些只约束研究轨迹，不改原 Tracker。jerk、推力和倾角范围是明确研究假设，不是 PX4 执行输入或已辨识参数。
最终按 0.1 s 细分网格重新验收动力学和原始 P1 整体目标 FOV/距离；未作连续时间安全证明。

### 姿态和几何

理想机体 b3=(g e_down-a)/norm，g=9.80665；用平坦 yaw 的水平参考轴构造 b2、b1。
比推力限2..18 m/s²，倾角限0.55 rad；自由落体、退化轴、非有限值和不可行推力/倾角显式拒绝。
不引入质量、惯量或内环参数；平坦 yaw 不应混同为大倾角下的严格 Euler heading。
实际首样本必须传入完整测量 FRD→NED SO(3)，未来样本采用理想姿态；缺失实际姿态不能由预测姿态替代。

沿用 [P1 几何与接口设计](mpc_minco_visibility_design.md)：SDF/FLU→FRD→NED 与 optical(+右,+下,+前) 方向保持一致。
当前前视安装向下28°，平移在模型原点约定下为(0.18,0,0.39) m；不能忽略 base_link 高度和相机位移。
640×480/horizontal_fov=1.74；目标为可配置球模型 r=0.25 m、参考偏移(0,0,-0.42) NED m。
FOV 基础裕度0.04 rad另加姿态模型误差0.03 rad；距离沿用 P1 axial 范围0.05..25 m。
模型误差裕度尚未由实际姿态跟踪误差统计标定。红球结果不能外推为无标记 USV 感知验证。

### 时间、热启动、失败

PlanningContext 保存 mission/cycle、导航/姿态/执行起点、预测 source/observation/valid_until、序列和时钟代次。
预测相对时间基于 source_stamp，要求完整覆盖执行起点至终点；不外推、改戳或用真值补样。
影子位置和姿态使用当前导航 sample epoch：保留原始 sample戳、因果 timesync、Gazebo 同消息 sim/system anchors，姿态 history bracket/SLERP。
拒绝 reset、无时钟 bracket、超时、过期、非法姿态和乱序任务/预测；prediction generated≥source≥observation。
目标只来自 tracking BCTRA。真值仅在原 evaluator 内，研究节点没有真值订阅。

热启动只移动历史严格通过解的控制序列，同 mission/clock generation/未越过原候选时域；精确时间节点防浮点落格。
旧预测不作为当前感知输入，首边界总是显式新请求 P/V/A；旧研究解不是 Tracker 已接纳轨迹。
超时返回 DEADLINE_EXCEEDED；输入、STALE_INPUT、PREDICTION_HORIZON、DYNAMIC_INFEASIBLE、VISIBILITY_INFEASIBLE 分开。
初始不可见而未来恢复只返回 RECOVERY_CANDIDATE、valid=false，不导出为安全 MINCO seed。

影子节点单 worker/最多一个在途解，预测 QoS BEST_EFFORT，与实际 publisher 一致；只发布两个 research String JSON 话题（depth1）。
完成、JSON 序列化与发布再次校验125 ms新鲜度、mission和clock代次；迟到后清空研究 marker，并清除warm状态。
日志区分 core_result.valid、最终 admission、逐次 publication_attempts 与 final_marker_valid；消费者必须核对 expires_at_ros_stamp。
研究发布存在普通 ROS 传输延迟，无法用单节点检查证明远端收件时新鲜，后续消费者仍需验收。

## 离线实验

[配置与650个产物的SHA清单](../../data/experiments/20261008_p2_mpc/offline_final/manifest.json)
覆盖18类场景，3次重复×3个滚动周期×4方法=648条，四方法使用相同请求指纹。
synthetic 模式显式开启，仅为算法测试，没有伪称 BCTRA 或在线真值输入。
五次与视点基准分别生成加速度/偏航序列，转为区间 jerk，再用相同精确积分与严格验收；不裁剪基准超限结果。
表中分母9是每场景每方法的次数，包含预设非法输入，不能把全部场景比例当正常场景成功率。

| 场景 | 五次 | 视点插值 | MPC cold | MPC warm |
|---|---:|---:|---:|---:|
| constant_velocity | 9/9 | 9/9 | 9/9 | 9/9 |
| dynamics_infeasible | 0/9 | 0/9 | 0/9 | 0/9 |
| expired_prediction | 0/9 | 0/9 | 0/9 | 0/9 |
| figure_eight | 0/9 | 0/9 | 9/9 | 9/9 |
| fov_edge | 0/9 | 0/9 | 9/9 | 9/9 |
| left_turn | 0/9 | 9/9 | 9/9 | 9/9 |
| nonmonotonic_prediction | 0/9 | 0/9 | 0/9 | 0/9 |
| recoverable_lost_view | 0/9 | 0/9 | 0/9 | 0/9 |
| right_turn | 0/9 | 9/9 | 9/9 | 9/9 |
| short_prediction | 0/9 | 0/9 | 0/9 | 0/9 |
| speed_jump | 9/9 | 0/9 | 9/9 | 9/9 |
| stale_observation | 0/9 | 0/9 | 0/9 | 0/9 |
| stationary | 9/9 | 9/9 | 9/9 | 9/9 |
| too_close | 0/9 | 0/9 | 0/9 | 0/9 |
| too_far | 0/9 | 0/9 | 0/9 | 0/9 |
| turn_jump | 0/9 | 0/9 | 9/9 | 9/9 |
| wrong_frame | 0/9 | 0/9 | 0/9 | 0/9 |
| wrong_yaw | 0/9 | 0/9 | 0/9 | 0/9 |

| 方法 | strict通过 | 完整流程P50/P95/P99 ms | 超时 |
|---|---:|---|---:|
| quintic | 27/162 | 9.4/18.1/23.1 | 0% |
| viewpoint_interpolation | 36/162 | 10.1/18.0/21.1 | 0% |
| mpc_cold | 72/162 | 102.5/144.8/181.3 | 0% |
| mpc_warm | 72/162 | 106.7/143.5/239.5 | 0% |


只统计各方法严格通过的候选（群体不同，不能据此直接比较平均误差）：

| 方法 | 最小H/V裕度rad | 平均跟随RMSE m | 最大H速度/加速度/jerk | 最大倾角rad |
|---|---|---:|---|---:|
| quintic | 0.7611/0.2703 | 0.0402 | 3.198/2.109/5.888 | 0.212 |
| viewpoint_interpolation | 0.2976/0.1121 | 0.9301 | 1.681/0.400/2.000 | 0.041 |
| mpc_cold | 0.1613/0.1193 | 0.7624 | 4.756/2.217/2.233 | 0.223 |
| mpc_warm | 0.1613/0.1193 | 0.7586 | 4.802/2.238/2.233 | 0.224 |

这些候选细分样点的whole-target可见比例均为1，动力学残差最大值为0。时域之外和样点之间未作安全证明。

MPC warm实际使用42次；warm-cold耗时差的中位数约0.03 ms、P95约+14.2 ms，没有证据证明warm稳定降低耗时。
转弯、八字、视场边缘、转弯率突变有可行率收益；静止/匀速的简单初始化已可行，MPC开销未必值得。
不同方法全部轨迹的可见比例、H/V最小裕度、跟随RMSE、速度/加速度/jerk/倾角分位数与失败分布见各场景JSON和 summary.json；非法输出不补造指标。
性能实验部分与构建/回归并行，耗时表示本机本次观测，不能作为独占CPU实时保证。
2块/4迭代和3块/8迭代同场景均16/36通过，前者P95约105 ms、后者246 ms，所以选择较小问题。

[滚动连续性诊断](../../data/experiments/20261008_p2_mpc/rolling_continuity.json)有48对相邻可行候选：
旧候选在新时刻的参考与新首边界，P/V/A差P95约0.00196 m/0.0294 m/s/0.294 m/s²。
这是合成测量状态重新锚定研究候选的差，不是实际接纳轨迹交接证明。

## PX4/Gazebo 实测与真实请求回放

F1经现有uav_lab --no-build和独立wrapper启动，原机动为figure_eight；PX4达到OFFBOARD地面待命。
控制台TTY的X没有产生任务接纳，随后使用脚本相同的合法ROS命令（三次、等待两个订阅者）启动。
原 Tracker 记录 TAKEOFF COMPLETE→FOLLOW；未发Y。实际ROS图确认轨迹setpoint只有 trajectory_tracker_node 一个发布者，影子没有任何 fmu/in 发布者。
初次运行发现预测QoS不匹配，修复后仅重启本轮研究节点，原控制和仿真不重启。
原仿真初段有Gazebo目标pose更新失败和低图像频率；后段取得真实tracking/BCTRA输入，保留全部日志。

[实际日志汇总](../../data/experiments/20261008_p2_mpc/f1/shadow_summary.json)：
55.45 s窗口、317事件、53次completion。核心25 SAFE、21 DYNAMIC_INFEASIBLE、6 DEADLINE_EXCEEDED、1 VISIBILITY_INFEASIBLE。
全部最终有效marker为0；52 completion因新鲜度拒绝，另一次因时钟代次变化拒绝。
完整流程P50/P95/P99=382.0/523.6/527.1 ms；起始导航年龄P95=43.6 ms、观测年龄P95=107.3 ms，125 ms剩余预算不足。
初期/过程中还存在126次 PREDICTION_HORIZON、时钟bracket/reset和预测失效拒绝。起点早于预测首样点也属于coverage拒绝；需要后续共同时刻调度解决，不能外推补造。
在线warm使用0次，因为最终未验收的候选必须清除，不能用离线warm成功替代在线滚动证明。

F1完成了实际FOLLOW数据采集与安全拒绝验证，未通过“新鲜且有效在线轨迹”验收；F3完成重规划失效统计，未证明在线连续有效接替。
F2使用18类离线场景和真实请求回放覆盖，未增加仿真轮次；F4未执行（用户额度限制且本轮未接管控制）。
不能声称影子节点对旧系统性能完全无影响；没有改变控制输出，但实时负载与基线因果对照未做。

[20条真实请求、80个候选的同输入回放](../../data/experiments/20261008_p2_mpc/replay_final/summary.json)：原始epoch、六组模型配置和原节点拒绝完整保存。
只比较历史请求几何/动力学与本次计算，replay_only=true，accepted_by_tracker=false；不回写或修改实际日志。

| 方法 | strict通过 | 完整流程P50/P95/P99 ms |
|---|---:|---|
| quintic | 10/20 | 16.8/18.1/28.4 |
| viewpoint_interpolation | 7/20 | 16.9/18.0/28.4 |
| mpc_cold | 11/20 | 114.9/117.4/118.2 |
| mpc_warm | 15/20 | 117.1/119.3/134.6 |

回放warm比简单初始化可行率高，但P95约119 ms仍几乎耗尽125 ms，尚未计入现场已有的样本年龄与调度负载。
回放离线可行不等于当时在线 ADMITTED。两组产物清单分别650/82项SHA全部复核相符。
所有实验位于新忽略目录 `data/experiments/20261008_p2_mpc/`；root manifest保存最终文件/源码指纹。
本轮进程使用记录的PID/PGID/启动标识限定停止，未误杀已有gnome终端服务器；仅清理本轮组件和研究进程。

## MINCO接口与P3边界

`to_minco_seed(result, waypoint_stride=4)`只允许严格SAFE：选内部第4/8样点作航点，分段时间为原relative_times差，
传入MincoS3Trajectory现有start/end position/velocity/acceleration和intermediate_positions/durations；保存共同context及yaw_refs。
构造现有MINCO只验证接口和边界，不是二次优化或可见性保持证明。改变航点/时间后，P3必须重新以共同绝对时刻查询目标预测、重新预测姿态并验收连续轨迹。

真正重规划交接应显式选t_h，从上一条**Tracker已接纳**轨迹在t_h采样期望P/V/A，MPC与MINCO都使用该边界和执行epoch。
本轮shadow用实际配对状态做研究初始条件，不能把它或上一条研究seed称为实际交接边界，更不能把旧轨迹未来终点作为所有新规划起点。

P3在线前关键问题：压缩完整计算与调度时间；规范未来执行时刻及nav/prediction起点覆盖；评估实际加速度与规划边界兼容；
标定姿态误差/角速度及不确定性裕度；MINCO改变时间后的连续可见性验收；接受新轨迹的独立freshness与唯一Tracker交接。
不需要为此放宽原海面、捕获、速度或125 ms新鲜度限制。

## 本轮验证记录与复现

最终 `src/uav_control` 全pytest：**1177 passed、1 skipped、2个现有linter依赖警告**。
P1本轮独立74 passed，新增测试与原localizer/navigation/strict-visual-control等已包含完整回归。
项目flake8（实际命令如下）、新增launch lint、shell语法、git diff --check和cached --check均通过；两包构建成功。
`ros2 pkg prefix uav_control`实测为 `/home/qin/data/uav_usv_mpc/install/uav_control`。
最终pytest/flake8/build输出保存在实验根目录，未用历史982测试结果替代本轮。
开发中全包曾因未完成节点/两处E501失败，均修复后复跑；初次QoS失败的日志也保留。
输出保护测试曾错误创建冻结目录下3个全新JSON，已仅撤销本轮新文件和目录；未改写原证据。最终diff/新路径检查通过，工具现先拒绝冻结/旧仓库输出路径。

```bash
cd /home/qin/data/uav_usv_mpc
UAV_USV_WS=$PWD scripts/build_workspace.sh --packages-select uav_control uav_usv_bringup
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
(cd src/uav_control && python3 -m pytest -q)
python3 -m flake8 src/uav_control/uav_control src/uav_control/test scripts --max-line-length=99
python3 scripts/mpc_seed_experiment.py --output-dir data/experiments/<新的空目录> --repeats 3 --rolling-cycles 3
python3 scripts/mpc_seed_replay.py --input-jsonl <原始JSONL> --output-dir data/experiments/<新的空目录> --max-records 20
```

已有实验目录拒绝覆盖。独立shadow启动使用 `ros2 launch uav_usv_bringup mpc_seed_shadow.launch.py log_directory:=<新目录>`。
完整仿真仍通过现有uav_lab，设置 UAV_USV_WS、UAV_USV_EXPERIMENT_LAUNCH=mpc_seed_validation.launch.py、独立CONFIG_FILE及ROS_LOG_DIR。
本轮报告给出部分通过与在线阻碍，停止于P2，等待用户审核。
