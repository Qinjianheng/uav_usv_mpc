# Codex P4.5 长任务：MINCO真实SITL闭环验证与论文式轻量化优化

## 一、任务背景

项目目录：

`/home/qin/data/uav_usv_mpc`

GitHub仓库：

`Qinjianheng/uav_usv_mpc`

最新确认提交：

`6842a9388018a654418073609f9e6e626a810b2e`

本轮用户决定：

**继续使用已经接入的MINCO，不恢复原FOLLOW默认控制配置。**

本轮主要验证MINCO在PX4/Gazebo中的真实闭环跟随效果，并根据实验决定是否进一步采用参考论文中的轻量前端、MINCO后端优化架构。

参考论文：

*Adaptive Tracking and Perching for Quadrotor in Dynamic Scenarios*

## 二、总体任务目标

完成以下工作：

1. 审查现有MINCO实际控制路径。
2. 自动启动隔离PX4/Gazebo SITL。
3. 首先完成低风险直线MINCO实际执行。
4. 验证MINCO连续ACCEPTED/ACTIVE状态与实际控制输出。
5. 修复阻碍持续接管的实现问题。
6. 测量实际控制周期、切换延迟和运行稳定性。
7. 进行F2直线、F3八字实际闭环实验。
8. 与原FOLLOW进行同条件对照。
9. 分析轨迹规划与实际跟踪之间的误差。
10. 如果MINCO效果仍不理想，研究论文式轻量化初值方案。
11. 执行完整回归并生成实验报告。

这是一个连续长任务。

普通代码修改、编译、测试、日志分析、仿真重试不需要中途确认。

允许自行处理可恢复的失败。

不得自动提交或推送Git。

不得修改旧仓库 `/home/qin/data/uav_usv`。

不得清理不属于本任务的仿真进程。

### 本轮控制边界

只允许在经过确认的隔离PX4/Gazebo SITL中进行实际MINCO执行。

保留用户要求的MINCO接入配置，不以恢复原FOLLOW默认值代替实验。

但必须增加或核实运行环境隔离：

- 不连接真实PX4飞控硬件；
- 检查外部串口、飞控和其他仿真进程；
- 使用独立ROS_DOMAIN_ID及Gazebo分区；
- 实验脚本明确记录执行授权来源；
- 普通实机运行环境不能误用SITL专用名义执行模式；
- 实际控制仍由唯一Tracker发布；
- 限定在当前Gazebo场景内执行。

本轮的名义执行lease不等同于经过证明的物理holding界。

不要为得到ACTIVE而伪造SafetyApproval，也不要宣称已取得实机资格。

---

# 阶段A：完整审查实际MINCO执行路径

首先核对：

- 本地HEAD及工作区状态；
- AGENTS.md；
- 当前构建及ROS overlay；
- 最新P4.4代码；
- 原FOLLOW控制实现；
- 新MINCO执行模块。

重点读取：

`src/uav_control/uav_control/control/trajectory_tracker_node.py`

`src/uav_control/uav_control/guidance/follow_minco_execution.py`

`src/uav_control/uav_control/guidance/follow_contract.py`

`src/uav_control/uav_control/controllers/p4_follow_planner_node.py`

`src/uav_control/uav_control/controllers/p44_follow_solver.py`

`src/uav_control/uav_control/guidance/follow_fast_validation.py`

`src/uav_usv_bringup/config/follow_minco.yaml`

`src/uav_usv_bringup/launch/modular_intercept.launch.py`

`scripts/p45_minco_sitl.py`

## A1. 验证控制流程

检查：

Planner生成MINCO轨迹
→ Tracker接收
→ ACCEPTED
→ 等待执行起点
→ ACTIVE
→ MINCO_FOLLOW
→ 发布PX4速度命令
→ 接收下一条规划
→ 连续切换
→ lease到期或轨迹失效
→ 有界回退。

核对每个环节的状态条件。

必须区分：

- 规划有效；
- 研究提案已发布；
- Tracker已接收；
- ACCEPTED；
- ACTIVE；
- 真正向PX4发布MINCO引导的命令。

不能把合法发布、ACK或ACTIVE单独当作已经完成真实跟随。

## A2. 检查执行边界

当前参数：

- Planner 5Hz；
- Tracker 20Hz；
- 未来执行提前量0.15s；
- NominalFollowReceiver lease 0.45s；
- 原始输入期限125ms；
- 默认规划H1.2。

分析：

- 新轨迹能否在旧lease到期前到达；
- 轨迹起点是否满足切换条件；
- 规划失败时能否保持有限时间的合法控制；
- 首次MINCO接管是否引入速度指令突变；
- 规划状态和实际PX4响应是否一致；
- 数据过期时具体执行哪一种控制。

不得为了通过接收而直接放宽原始125ms新鲜度。

## A3. 检查控制模型

当前MINCO执行路径主要使用：

MINCO参考速度 + 位置反馈 + 原FOLLOW速度整形。

检查：

- 是否真正使用了MINCO加速度参考；
- yaw是否来自MINCO还是原视觉控制；
- 当前速度反馈增益是否合理；
- 是否存在不必要的重复限幅；
- 规划参考与实际飞行状态是否出现较大偏差；
- 实际PX4位置、速度和加速度是否滞后于轨迹。

特别检查速度指令输出时的状态同步问题。

不要将参考数学连续性等同于实际控制响应连续。

---

# 阶段B：首次实际MINCO SITL试验

本阶段不修改算法，先运行当前版本。

## B1. 仿真准备

优先复用：

`scripts/p45_minco_sitl.py`

`scripts/p4_sitl_session.py`

`scripts/start_px4_ros2.sh`

确保隔离SITL、正确日志目录、独立域及有界watchdog。

不要直接运行未经隔离检查的默认启动器作为首次试验。

## B2. F2直线首次接管

先运行原FOLLOW基线，再运行名义MINCO。

建议首次MINCO实际执行窗口约20～30s。

仅使用X启动FOLLOW任务。

不得进入Y截击任务。

实验必须记录：

- X命令确实送达；
- PX4实际进入Offboard；
- UAV成功起飞；
- 有效预测输入；
- Planner启动；
- 首条MINCO实际提案；
- 首个ACCEPTED；
- 首个ACTIVE；
- 首次MINCO_FOLLOW；
- 首次向PX4发布MINCO引导速度指令。

如没有ACTIVE，优先检查时间戳、预测版本、动态约束、接收端资格与状态同步。

保留全部拒绝原因。

不能通过删除合法性判断强行产生ACTIVE。

## B3. 初次接管保护

在首次执行前检查：

- 无人机已进入稳定FOLLOW；
- 当前位置和速度有效；
- 高度与海面有足够净空；
- 目标可见；
- 数据新鲜；
- 切换前命令有记录；
- 预期初次MINCO参考与当前运动状态差异已计算。

设置明确的SITL试验停止条件。

例如：

- PX4失去Offboard；
- 导航状态失效；
- 目标失锁；
- 违反既有海面安全边界；
- 飞机位置或速度出现异常发散；
- 控制定时器显著失去实时性；
- 无有效命令可供下一控制周期使用。

触发异常时停止名义MINCO执行，进入已有且已核验的仿真安全回退路径；若回退路径也失效，终止试验并保留日志，不重复盲试。

---

# 阶段C：解决连续MINCO执行问题

F2首次成功后，不立即进入复杂八字。

先验证名义MINCO能连续工作。

## C1. 轨迹生命周期统计

统计每条实际被Tracker接收的曲线：

- proposal ID；
- parent plan ID；
- prediction sequence；
- accepted stamp；
- active stamp；
- execution start；
- execution end；
- lease deadline；
- revocation；
- expiry；
- fallback。

计算：

- ACCEPTED比例；
- ACTIVE比例；
- 实际MINCO控制时间占比；
- 平均连续ACTIVE持续时间；
- 最长连续ACTIVE；
- 原FOLLOW回退次数；
- 相邻轨迹切换间隔；
- 规划器停止后控制权恢复情况。

重点验证：

**5Hz规划和0.45s lease能否长期维持有效接替。**

## C2. 轨迹交接连续性

在每次切换时记录：

- 位置参考跳变；
- 速度参考跳变；
- 加速度参考跳变；
- yaw参考跳变；
- 实际速度命令跳变；
- 实际姿态及角速度响应。

区分三类连续性：

1. MINCO曲线数学连续性。
2. Tracker最终输出命令连续性。
3. PX4真实运动连续性。

如果某一类不连续，查明原因后修复。

禁止只通过提高限制器上限掩盖问题。

## C3. 预测和状态失效

在隔离SITL里进行有界故障试验：

- 短时预测断流；
- 短时规划器停止；
- 目标丢失；
- 新规划迟到；
- lease到期；
- 预测版本变化。

检查：

- 旧lease是否被错误续期；
- 无效曲线是否仍用于产生速度命令；
- 回退到原FOLLOW时是否产生明显控制跳变；
- 状态丢失后是否错误保持高速命令。

所有异常必须有完整时间戳及最终控制输出记录。

---

# 阶段D：F2/F3真实闭环对照

只在F2连续接管可靠后开展F3。

## D1. 对照组

A：原FOLLOW。

B：P4.4/D，H1.2，名义MINCO实际执行。

C：P4.4/D，H1.6，名义MINCO实际执行。

D：P4.4/F，H1.2，名义MINCO实际执行，作为可选消融。

先在F2重复，再进入F3八字。

每组至少完成3次健康、独立的SITL重复。

异常轮不得删去，也不得计为健康完成。

保持：

- 同一个USV运动场景；
- 同一感知模型；
- 同一PX4参数；
- 同一相机外参；
- 同一动力学限制；
- 同一测量与评价规则。

## D2. 实际跟随性能

统一比较进入FOLLOW后的20～60s稳定窗口，并额外报告全程结果。

关键指标：

- 水平相对位置RMSE；
- 三维位置RMSE；
- 实际跟随距离误差；
- 相对速度RMSE；
- FOV整球可见率；
- 最小水平/垂直FOV裕度；
- 实际位置峰值误差；
- jerk估计；
- 速度及加速度峰值；
- 实测yaw rate；
- roll/pitch/yaw姿态；
- 推力控制代理；
- 实际MINCO控制占比；
- 每次控制切换的突变。

特别区分：

- 规划轨迹相对目标误差；
- UAV实际位置相对规划轨迹误差；
- UAV实际位置相对目标观测点误差。

不能混淆这三种RMSE。

## D3. 效果判断

历史原FOLLOW F3稳态水平RMSE约0.69m。

历史H1.6理想MINCO滚动20s后误差约0.505m。

两者原本不是同口径。

本阶段要回答：

1. 实际MINCO能否在F3长期运行？
2. 实际MINCO位置误差是否小于原FOLLOW？
3. 若误差更大，是规划参考本身错误还是PX4跟踪误差？
4. FOV裕度是否改善？
5. jerk或速度命令变化是否更平滑？
6. 规划频率是否足够？
7. 是否频繁切回原FOLLOW？

如果MINCO接管时间占比不足，不能将混合控制结果直接当作纯MINCO结果。

---

# 阶段E：在线延迟与实际控制分析

分别记录：

- 图像到定位；
- 定位到BCTRA；
- BCTRA计算；
- 预测接收；
- 候选初值；
- MINCO构造；
- 严格轨迹验收；
- 预测重验；
- ROS发布；
- Tracker接纳；
- ACTIVE；
- PX4 setpoint输出；
- UAV实测响应。

使用相同cycle/plan及源时间戳建立因果关系。

计算互斥的阶段耗时和真实系统时效。

继续区分：

- 纯求解器P95；
- 在线完整提案P95；
- Tracker接收处理P95；
- 真实执行起点偏差；
- 控制周期抖动。

还要特别测量：

Tracker20Hz定时回调中的 `nominal_curve()` 是否因每周期重新检查未来轨迹而产生过多计算。

若此处导致控制周期超时：

- 先进行profiler和日志归因；
- 研究缓存不变的数学计算；
- 研究独立非阻塞验收与有时效限制的验证结果；
- 每次新预测仍重新执行必须的可见性检查；
- 验证结果过期则不执行名义轨迹。

不得以停止必要验证来提高控制频率。

---

# 阶段F：根据实际效果决定是否采用论文式轻量化

只有在真实MINCO执行数据足够完整后进入此阶段。

## F1. 首先分类瓶颈

若主要问题是：

- Planner计算慢：优化候选初值和MINCO计算；
- Tracker实时性不足：优化执行验证和调度；
- 规划轨迹本身误差大：调整参考轨迹与优化目标；
- 轨迹可行但实际飞行误差大：检查控制器与PX4动态响应；
- 预测误差大：检查BCTRA、时间同步和模型；
- 频繁失效：检查lease、交接与回退。

不要未经过实际归因就直接替换整个规划器。

## F2. 新增论文式轻量规划器

如确认规划初值和候选计算仍是重要限制，新建：

`direct_reference_minco`

保留现有P4.4/D，不覆盖。

新架构：

BCTRA预测
→ 生成未来正后方观测参考
→ 单次MINCO初始化
→ 可选低维Q/T/yaw优化
→ 完整严格验收
→ 有界失败回退。

与参考论文一致：

**轻量前端只负责提供合理初始路线，MINCO后端负责优化轨迹。**

本场景不需要复现论文用于复杂环境遮挡的射线搜索和A*。

## F3. 初值生成

研究两种方式：

1. 直接基于BCTRA目标预测生成未来后方观测位置。
2. 使用原FOLLOW反馈规律对预测USV进行虚拟滚动，生成参考P/V/A。

由预测参考提取：

- 起点P/V/A；
- 两个中间点Q；
- 终端P/V/A；
- yaw初值；
- 三段时间T。

必须使用因果输入。

虚拟UAV后续状态由内部模型传播，不允许利用未来真实UAV日志。

原FOLLOW输出的零加速度字段不能直接当作真实边界加速度。

## F4. MINCO优化

优先实验：

- 单次MINCO映射；
- Q优化；
- Q/T优化；
- Q/T/yaw优化。

保持有界迭代与完整计算预算。

目标包含：

- 相对跟随位置；
- 相对速度；
- 动态参考加速度；
- jerk积分；
- FOV裕度；
- 轨迹切换连续性。

所有优化结果经过独立最终验收。

失败时，在真实时效预算允许的范围内回退P4.4候选方法。

不预设论文式方法一定更快或更好。

## F5. 公平对照

对比：

A：P4.4/D原候选算法。

B：Direct MINCO固定初始化。

C：Direct MINCO + Q优化。

D：Direct MINCO + Q/T/yaw优化。

同输入离线回放后，再选择具备合法时效、名义安全检查及稳定接管条件的方案进行隔离SITL对照。

比较：

- 实际完整在线周期；
- 可行率；
- 实际ACTIVE占比；
- 实际水平RMSE；
- 目标可见性；
- jerk；
- CPU；
- 回退次数。

如果论文式方法没有优势，继续保留P4.4，不强行替换。

---

# 阶段G：完整回归与安全检查

执行：

`./scripts/test_research_planners.sh algorithm`

`./scripts/test_research_planners.sh related`

`./scripts/test_research_planners.sh full`

并完成：

- 四包构建；
- ROS接口；
- flake8；
- shell语法；
- git diff --check；
- 原FOLLOW回归；
- 原截获任务单元回归；
- P4.4轨迹一致性；
- MINCO边界连续性；
- Tracker唯一PX4发布者；
- 控制周期实时性；
- 真实ACK生命周期；
- 实际MINCO控制时间占比；
- 状态丢失；
- 规划迟到；
- lease到期；
- 回退连续性；
- 旧轨迹不可错误续期；
- 仿真隔离；
- 自有进程清理。

任何异常结果必须保留原始数据与修复记录。

不得用单元测试通过代替实际SITL验证。

---

# 阶段H：最终交付

新实验目录建议：

`data/experiments/20261010_p45_minco_execution/`

如果目录已存在，使用新的唯一目录。

输出：

`docs/current/p45_minco_execution_validation.md`

`docs/current/p45_follow_closed_loop_comparison.md`

`docs/current/p45_planning_latency_analysis.md`

如完成论文式研究，再增加：

`docs/current/p45_lightweight_minco_study.md`

报告必须明确回答：

1. MINCO是否真正进入ACTIVE？
2. 实际有多少时间由MINCO控制？
3. 是否能持续滚动接管？
4. 首次接管是否产生速度或姿态冲击？
5. lease到期如何处理？
6. 跟踪误差是否比原FOLLOW更小？
7. 实际FOV是否改善？
8. jerk和控制平稳性是否改善？
9. MINCO规划与PX4实际执行误差分别是多少？
10. 1.2s和1.6s哪个实际效果更好？
11. 规划器真正的在线P95是多少？
12. Tracker实际计算负担是否影响20Hz控制？
13. 若效果差，主要原因是规划、预测还是跟踪控制？
14. 论文式轻量初始化是否带来额外收益？
15. 目前还缺少哪些面向实机的安全资格？

最终提供：

- 完整修改清单；
- 测试结果；
- SITL对照表；
- 时间序列图；
- 实际控制权分配图；
- 跟随轨迹图；
- 预测误差图；
- FOV裕度图；
- 控制指令变化图；
- 论文式算法消融；
- 原始日志与SHA校验清单；
- 下一阶段建议。

## 连续执行及停止规则

本轮允许直接进行名义MINCO SITL执行。

不要求先完成实机级holding认证才允许做隔离SITL研究，但必须保持实验级隔离、有限lease、异常停止与回退检查。

如果首次F2执行不稳定，不允许直接扩大到F3八字。

如果控制或回退存在危险行为，停止实际MINCO执行，继续离线与影子归因，直到问题修复并经过低速SITL复测。

如果F2稳定，自动继续F3与多次重复。

如果原规划器表现足够好，不必为了模仿论文而重写。

如果实际规划结果仍明显落后原FOLLOW，先做误差归因，然后研究Direct MINCO方案。

**最终目标不是单纯得到ACCEPTED/ACTIVE，而是证明MINCO能在PX4/Gazebo中持续跟随USV，并明确量化它相对于原FOLLOW在跟随精度、可见性、平稳性和计算延迟上的真实收益。**