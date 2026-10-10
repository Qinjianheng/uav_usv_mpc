# P3 贪心与 Visibility-Aware MINCO 研究验证

## 结论和范围

从master/15d9f520e4c09bf26716775d3ca642309b8e8789干净工作区开始。
实现BCTRA快照→贪心初始化→原MINCO映射的Q/T/yaw优化→研究JSON输出；原MPC继续保留。
完成纯算法、ROS适配/合法回放、实际PX4/Gazebo影子验证。新结果没有接入Tracker。
原控制链仍是原BCTRA/MINCO/Tracker，原Tracker独占Offboard。

已看到真实数据上的采样可行轨迹，但尚未得到及时的最终有效发布，不能进入在线FOLLOW接管。
收敛、采样可行、研究发布准入、Tracker接纳是四个不同状态；所有产物accepted_by_tracker=false。
红球场景只验证仿真感知接口，没有验证无标记非合作USV。

## 新算法与数学约定

### 公共输入/规划边界

FollowProblem只组合P1/P2纯模型。传入位置/速度/加速度/完整实际FRD→NED姿态、BCTRA样本和不可变任务上下文。
拒绝非有限值、非法SO(3)、不一致导航/姿态epoch、非tracking在线来源、过期图像/预测、非法frame和不覆盖的预测。
离线synthetic仅在显式allow_synthetic_predictions=True时允许；在线入口强制关闭。

给定实测t_m，当前处理t_n，未来执行t_e=t_n+lead（默认150ms），dt=t_e-t_m：

p_e=p_m+v_m*dt+0.5*a_m*dt²；v_e=v_m+a_m*dt；a_e=a_m；yaw_e=yaw_m。

FutureRequest单独保存measurement_state和boundary_policy，原navigation/attitude/observation/source_stamp均不改写。
这只是短时常加速度研究预测，不是Tracker已接纳轨迹的实际交接状态。
将来必须在真正交接t_e从上一条已接纳轨迹求P/V/A；禁止拿旧轨迹未来终点作为所有新规划的起点。
MPC/MINCO共同的时间基准为t_e；目标查询为t_e+tau-source_stamp，不外推、不夹到预测首/末点。

### 贪心初始化

固定2.4秒、三段初值T=(0.8,0.8,0.8)，两个中间Q。
在0.8/1.6/2.4秒顺序尝试正后方、左右后方±0.6rad及保留方向，最多四候选/节点，不做指数搜索。
参考位置为目标水平位置减5m方向向量，z固定-5m。目标水平速度<0.2m/s时用初始UAV yaw作退化方向。
粗速度=(下一点-上一点)/0.8；粗加速度=(粗速度-上一速度)/0.8。
筛选粗动力学、原海面停车余量、完整规划姿态和P1整球视场。
代价包括预测惯性位移误差、2*(方向切换)²、yaw变化及视场余量奖励。
只有最终采样可行后，运行器才保留倒数第二观测方向用于下一轮；输入/任务失效则清除。

终点统一为正后方参考P、目标水平V、Vz=0、A=0。三个初始化方法都用该终点，侧后方只改变中间Q。
恒高参考的终点Vz必须为零；初版曾取目标Vz，最终审计已修正并增加升沉测试/回放/短影子复验。
水平终点速度暂用目标速度，并非转弯观测点参考的严格导数；这是转弯时jerk可行性需要继续研究的边界假设。
FollowSeed.final_validated始终false；无候选返回具体原因，不能当作最终安全路径。

### MINCO与yaw

唯一xyz实现仍为MincoS3Trajectory，解析control_effort()为积分jerk²，不另拟一套五次多项式。
固定模式直接验收Q/T；q模式优化6个Q坐标和3个后续yaw结点；qt模式再加入2个时间logit。
T_i=2.4*softmax(l_1,l_2,0)_i，保证正时间和固定总时长；本轮没有优化总飞行时长。
Q范围为种子±0.75m，logit∈[-1.5,1.5]，yaw结点为种子±0.8rad。
L-BFGS-B有限差分，默认仅2轮、整个初始化/求解预算500ms；超时输出invalid。

yaw用同一累计T结点上的C2分段三次样条，先unwrap，首末yaw rate暂设0；不是测得的机体角速度。
每段yaw rate是二次函数，对端点及内部极值求全局最大绝对值；保证被验收的yaw参考满足1rad/s。
XYZ和yaw共同优化。joint_yaw_optimization_overlaps_minco是共享的优化阶段，不能与minco_optimization相加。
yaw_construction_within_optimization另计样条构造耗时，含最终一次构造，不是假称独立的yaw求解器耗时。

目标函数为0.003*解析jerk²积分+20*可见性软罚+1*跟随位置MSE+100*动力学违约²
+1000*海面违约²+0.05*yaw rate²。
可见性使用softplus(-12*margin)²/144，margin包含角度h/v和轴向近/远距离；这些罚尺度/权重是研究配置，尚未物理标定。
没有可见性代价的消融只将该权重置零，最终FOV验收保持开启。
固定T优化Q和Q/T实测都会改变变量及降低原目标，验证不只是航点拟合。
qt对simple/greedy/mpc分别有26/14/18条Q变化>1e-6，24/12/18条T变化>1e-6；目标严格改善24/12/18条。
达到迭代上限仍可能采样可行，收敛也可能不满足约束；不混用solver_success与valid。

### 姿态/相机/最终验收

NED世界、FRD机体、Gazebo FLU及光学坐标完全复用P1：光学坐标=(-camera_flu_y,-camera_flu_z,camera_flu_x)。
相机模型原安装SDF平移(0.18,0,0.39)FLU、俯仰向下28°，全姿态旋转；不改相机/模型参数。
640×480、hFOV1.74rad，安全裕度h/v各0.07rad；有效轴向距离0.05–25m；目标球半径0.25m、NED中心偏移(0,0,-0.42)。
这些来自P1/P2当前校准，不能外推为任意USV几何模型。

规划姿态b3=normalize(g*e_down-a)、b2=normalize(b3×h(yaw))、b1=b2×b3，使用P2 planned_attitude。
同刻a/yaw决定相机姿态；速度暂不进入无阻力理想模型。实际PX4完整姿态只用于原实测epoch的几何；FutureRequest使用规划预测姿态。
不假设Tracker实际yaw必然跟随研究参考，也没有在线姿态校正。
g=9.80665、比推力2–18m/s²、倾角0.55rad仍为P2理想研究限制；角误差0.03rad字段是保留元数据，尚未形成鲁棒姿态包络约束。

独立验收0.05秒密集网格并含各段结点；角裕度<0.08或物理裕度<0.1的相邻区间再细分两轮。
检查PVA端点残差<1e-7、独立系数求导得到C0–C4拼接残差<1e-5、数值/预测覆盖/所有整目标FOV。
速度h/v=6.2/4，a=3/3，jerk=6/4；海面停车余量为
0-0.5-z-max(vz,0)*0.15-max(vz,0)²/(2*2.5)，并检查tilt/比推力/yaw rate。
这些采样/自适应检查没有xyz/相机的严格区间界，valid称SAMPLED_FEASIBLE；不是连续时间数学安全保证。
初始不可见、之后恢复最多为RECOVERY_CANDIDATE，仍invalid。

## 离线对照与消融

最终串行主对照：offline_final；P2十八场景+减速+姿态epoch不一致，共20场景×2分析导航时刻=40组请求。
每组3初始化×4消融，共480条记录；两分析时刻不是已执行轨迹闭环。
A/simple使用预测后方视点，B/greedy使用有限候选，C/mpc使用未经修改的P2求解器提取0.8/1.6秒Q和同刻yaw。
C只贡献中间样本，终点强制为共同PVA；不能把它称作完整MPC终端状态原样传给MINCO。
所有方法同模型、同请求哈希、同起点/终点PVA、总时长及MINCO代价/验证器。初始化在各消融间复用，耗时保留。

| 初始化 | 消融 | 采样可行/40 | 总耗时P50/P95/P99 ms | 优化器收敛数 |
|---|---|---:|---|---:|
| simple | fixed | 4 | 33.13/58.87/59.28 | 0 |
| simple | q | 8 | 69.42/142.24/152.73 | 6 |
| simple | qt | 5 | 113.88/158.54/159.28 | 4 |
| simple | qt_no_visibility_cost | 8 | 89.35/147.85/157.91 | 4 |
| greedy | fixed | 4 | 1.94/61.00/61.82 | 0 |
| greedy | q | 8 | 1.87/145.33/155.93 | 4 |
| greedy | qt | 5 | 1.86/157.82/166.96 | 4 |
| greedy | qt_no_visibility_cost | 8 | 1.85/149.06/159.83 | 4 |
| mpc | fixed | 6 | 91.32/141.03/146.55 | 0 |
| mpc | q | 6 | 92.89/233.40/264.37 | 0 |
| mpc | qt | 7 | 92.87/223.69/257.07 | 0 |
| mpc | qt_no_visibility_cost | 7 | 92.87/223.01/223.81 | 0 |

各组超时均0；40包括非法/过期/距离/不可见/动力学故意失败，低可行率不等于有效导航场景下捕获率。
greedy的全组中位数含大量早拒绝，不能把1.86ms当作完整可行求解耗时。
三方法都能初始化的同一16组qt：可行5/5/7，总P95=157.12/164.18/238.23ms。
没有证据说贪心比简单视点更优；qt也没有比q取得更高可行率。
qt_no_visibility_cost可行8/8/7，说明当前联合代价/有限迭代权衡尚需调整；不能因无视场软罚更可行就取消硬验收。

qt分阶段P95（计入该阶段的样本数随初始化成功而异）：

| 方法 | 初始化ms | Q/T/yaw联合ms | yaw构造ms(重叠) | 密集验收ms |
|---|---:|---:|---:|---:|
| simple | 0.46 | 119.55 | 12.78 | 44.32 |
| greedy | 3.67 | 120.94 | 12.96 | 45.82 |
| mpc | 96.77 | 139.42 | 14.22 | 44.02 |

所有阶段P50/P95/P99、FOV比例/角裕度、跟随RMSE、jerk积分、收敛/失败原因保存在summary.json；逐请求曲线/系数保存在records.jsonl。
本轮不能用40组尾部分位数作平台实时性保证。

Level2最终回放replay_final选F2前20条request-bearing完成记录，保留原生测量/观测/预测epoch和FutureRequest边界。
fixed/q/qt/no_visibility分别：simple 10/10/11/11；greedy 10/10/11/11采样可行。
MPC全部20条因旧接口要求execution_start=measurement epoch拒绝；在四种消融中均为0。
这是明确的接口不可匹配样本，不是公平比较里“MPC性能差”的证据。没有将FutureRequest重新标记为测量时刻或伪造新鲜度。
离线重新可行不能反转原live拒绝，回放没有ROS发布/Tracker接纳。

## 实际PX4/Gazebo影子证据

F1完整八字→只关闭本轮三个可选诊断；F2独立轻量直线。flight.yaml是独立完整快照，保护baseline.yaml。
只发X，原系统起飞进入FOLLOW；没有运行Y/捕获/post-capture冻结，本轮不引用历史成功代替该项。
每次一个研究greedy_minco，1Hz单worker，不排队；未同时运行MPC竞争CPU。
F1/F2各由300秒watchdog约束，仅signal记录身份一致的Gazebo/PX4/DDS/ROS进程组。
F1终态一些原rclpy节点在SIGINT后报context shutdown异常/exit1；发生于watchdog关闭阶段，未冒充运行期正常退出。
F1一次短node list仅发现9个名字；实际负载窗口中保留11个节点进程，F2独立图实测11，详见架构审计。

| 指标 | F1八字(206完成) | F2直线(208完成) |
|---|---:|---:|
| 核心采样可行 | 36 | 191 |
| 动力学失败 | 125 | 0 |
| 无粗可行候选 | 38 | 9 |
| 核心超时 | 7 | 8 |
| 最终有效研究发布/Tracker接纳 | 0/0 | 0/0 |
| 周期P50/P95/P99 ms | 346.82/488.35/521.16 | 289.33/485.74/522.66 |
| 优化P50/P95 ms(完成优化子集) | 272.86/399.32 | 246.66/368.39 |
| 验收P50/P95 ms(完成优化子集) | 51.08/86.40 | 15.86/27.68 |
| 跟随RMSE中位数m(有最终几何子集) | 0.155 | 0.0217 |
| jerk²积分中位数(同子集) | 302.43 | 1.250 |
| 可见比例中位数(同子集) | 1 | 1 |

F1动力学失败125条都含horizontal_jerk违约，其中18含水平a、8含yaw rate；不能因FOV好就接纳。
P1角裕度F1 h/v中位数=0.650/0.262rad，F2=0.744/0.304rad；不是全周期最差保证。
F1原始报告P2的53/25/0、周期P95约524ms只作历史对照；本轮F1完整阶段P95=515.82ms、关闭诊断后466.41ms。
场景/配置/候选不同，不能宣称严格同输入性能提升，研究周期仍不符合125ms准入。

直线F2开始求解时observation_age P50/P95=49.58/80.32ms，navigation_age=20.17/21.02ms；
剩余125ms预算通常只有约75ms，优化中位数246.66ms已超出，额外调度/序列化P50/P95=16.16/25.99ms。
贪心初始化约7ms，联合MINCO有限差分才是主要瓶颈；轻量图降低负载但未解决准入。
F1已完成优化的168条及F2的199条最终都因STALE_INPUT拒绝；新鲜度125ms不变。

150ms执行lead和measurement epoch分开；完成时若已错过t_e，额外拒绝FUTURE_EXECUTION_MISSED。
在每次诊断/轨迹发布、JSON序列化前后重新检查原始年龄、valid_until、mission/generation、期限。
valid_until是当前预测快照的准入TTL；样本forecast coverage是source+prediction_times，二者不能混淆。
本轮快照valid_until均早于默认t_e，但能否在较早有效时刻接纳未来轨迹，需要单独的Tracker持有/失效/交接策略；
当前没有该策略，也没有一条通过当前publication检查的未来候选，不通过重写stamp制造有效结果。

### 最终升沉修正复验F3

最终代码固定end.Vz=0；100秒wall bound的独立轻量八字会话，包含启动和起飞。
67次完成：9采样可行、38动态失败、18无粗候选、2核心超时；最终有效0。
周期P50/P95/P99=308.26/440.36/520.80ms；38动态失败都含水平jerk，7含水平a、3含yaw rate。
源代码启动前SHA256保存于f3/source_before_run.json，全部终点速度z检查保持0；原飞行链继续FOLLOW。
F1/F2是修正前的真实数据，保留原结论；最终版本经F3和replay_final验证，不拿旧仿真代替新版本。
offline_final的synthetic目标Vz全部0，升沉修正不改变该对照的数学路径；配置中保留当时来源SHA而不重写。
F3末尾补查offboard topic时已超过watchdog cutoff，探针返回Unknown topic，vehicle_command后续探针未执行。
不把这个失败写成正常通信；控制隔离依据F1完整研究节点发布表和F1/F2唯一Tracker setpoint发布者。

## 验证、成本与复现

| 本轮最终检查 | 结果 |
|---|---|
| algorithm，无ROS setup | 183通过、3个ROS兼容案例转related；pytest 1.11s，shell 1.267s |
| related，Humble+新overlay | 326通过（详见related_test_final.txt） |
| full，src/uav_control | 1211通过、1跳过，9.52s；shell 10.130s |
| 项目flake8 | uav_control/uav_control、uav_control/test、scripts，99列，通过 |
| shell语法 | scripts下全部.sh，通过 |
| Launch | py_compile及--show-args，通过；F1/F2/F3实际启动 |
| 现有构建脚本 | --packages-select uav_control uav_usv_bringup，2包成功，30.7s |
| ros2 pkg prefix uav_control | /home/qin/data/uav_usv_mpc/install/uav_control |
| Git格式与范围 | diff --check / cached --check；受保护范围与HEAD相同、暂存为空 |

日常全入口10.13s对快速入口1.267s是同版入口成本对照；完整回归没有删减。
full的1跳过为既有copyright未声明检查，2个warnings为既有flake8插件SelectableGroups弃用提示。
初次full因新增测试两处E128失败，修正后最终通过。早期RED缺模块/工厂失败是先写测试的正常过程。
早期show-args在构建未结束时缺新文件、随后默认ROS日志目录只读；等待构建并显式ROS_LOG_DIR后通过。
未执行在线接管、真实无标记目标、Y截获/冻结、严格区间证明；不能将这些项目列为成功。

快速测试：

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/test_research_planners.sh algorithm
./scripts/test_research_planners.sh related
./scripts/test_research_planners.sh full
```

独立研究完整/轻量实验（每次一个研究算法）：

```bash
export UAV_USV_WS=/home/qin/data/uav_usv_mpc
export UAV_USV_EXPERIMENT_LAUNCH=follow_research.launch.py
export UAV_USV_RESEARCH_DIRECTORY=/home/qin/data/uav_usv_mpc/data/experiments/NEW_RUN
export ROS_LOG_DIR="$UAV_USV_RESEARCH_DIRECTORY/ros_logs"
export UAV_USV_RESEARCH_MODE=greedy_minco
export UAV_USV_RESEARCH_LIGHTWEIGHT=true
./scripts/uav_lab.sh --no-build
```

lightweight=false保留原三个诊断开关；baseline原参数不变。
已有飞行图只旁挂观察器时，source Humble和新overlay后：

```bash
ros2 launch uav_usv_bringup follow_research.launch.py start_flight_stack:=false \
  research_mode:=greedy_minco research_log_directory:="$UAV_USV_RESEARCH_DIRECTORY/shadow"
```

可选greedy_seed、greedy_minco、mpc_seed；greedy_seed仅粗初始化，永不标valid。
尚未支持mpc_minco在线模式，避免制造没有完整时域接口的能力。
离线工具共享P2场景和校准，不开仿真；输出必须为空的新目录：

```bash
OPENBLAS_NUM_THREADS=1 python3 scripts/follow_minco_experiment.py \
  --cycles 2 --output-dir /home/qin/data/uav_usv_mpc/data/experiments/NEW_COMPARE
OPENBLAS_NUM_THREADS=1 python3 scripts/follow_minco_experiment.py \
  --input-jsonl /ABS/recorded_shadow.jsonl --max-records 20 \
  --output-dir /home/qin/data/uav_usv_mpc/data/experiments/NEW_REPLAY
```

本轮证据根data/experiments/20261008_p3_follow：offline_final、replay_final、f1/f2/f3、测试/构建输出。
早期offline/replay子目录也保留；最终报告明确选择final目录作算法结论，不覆盖初始结果。
benchmark自带manifest.json；根manifest.json覆盖原始日志/CSV/配置、辅助复算脚本及分析JSON，另列PX4参数备份指纹。
实验目录按现有规则忽略，没有自动纳入Git；没有清理/改写任何冻结数据或旧仓库。

## 文件和控制兼容性

原节点全部保留；仅新增follow_research_shadow_node，选择一个实际算法。
旧follow_mpc_shadow_node只增加solver/request_factory及node名/runner工厂注入，默认仍调用旧MPC。
不改ROS消息，研究String JSON沿用P2话题和schema，增加research_mode及minco模型元数据。
旧MPC原数值优化、旧MINCO映射/Fast MINCO、Tracker/MissionManager/感知/KF/BCTRA、camera SDF、PX4补丁、baseline及安全门限均未改。
没有真值进入在线研究输入；无新增Offboard发布者。旧仓库只读，无暂存/提交/推送。

本轮修改/新增文件完整清单：

- `docs/current/README.md` (修改)
- `src/uav_control/setup.py` (修改)
- `src/uav_control/uav_control/controllers/follow_mpc_shadow_node.py` (修改)
- `docs/current/greedy_minco_validation.md` (新增)
- `docs/current/p3_architecture_audit.md` (新增)
- `scripts/follow_minco_experiment.py` (新增)
- `scripts/test_research_planners.sh` (新增)
- `src/uav_control/test/test_follow_minco_experiment.py` (新增)
- `src/uav_control/test/test_follow_research_algorithms.py` (新增)
- `src/uav_control/test/test_follow_research_runtime.py` (新增)
- `src/uav_control/uav_control/controllers/follow_research_shadow_node.py` (新增)
- `src/uav_control/uav_control/controllers/follow_research_solver.py` (新增)
- `src/uav_control/uav_control/guidance/follow_minco_optimizer.py` (新增)
- `src/uav_control/uav_control/guidance/follow_problem.py` (新增)
- `src/uav_control/uav_control/guidance/greedy_follow_initializer.py` (新增)
- `src/uav_control/uav_control/guidance/yaw_trajectory.py` (新增)
- `src/uav_usv_bringup/config/follow_research.yaml` (新增)
- `src/uav_usv_bringup/launch/follow_research.launch.py` (新增)

## 是否进入在线FOLLOW

P3研究实现、工程入口、离线消融和真实影子阶段已完成；最终准入瓶颈未解决，暂不接管在线FOLLOW。
下一阶段先处理：

1. 联合有限差分优化/可见性计算成本，按剩余原始新鲜度预算做实时配置；不能延长125ms或改写stamp。
2. 转弯时观测参考的终点水平V/A、短2.4s三段时域与jerk可行性；在保持共同边界的前提下评估更合适参数化。
3. 严格Tracker已接纳轨迹在交接时刻的PVA和yaw连续性、候选持有/过期/拒绝策略；常加速度预测不能替代实际交接。
4. 理想平坦姿态与PX4实际/Tracker yaw差异、目标/姿态不确定性及相机安装/目标外形鲁棒裕度。
5. 连续时间极值/区间界、闭环实时性和恢复/失锁安全；采样通过不构成严格安全证明。

停止在P3，不继续修改控制器、上线FOLLOW或实现PNG–IBVS。
