# P3 架构审计与实施记录

起始 master / `15d9f520e4c09bf26716775d3ca642309b8e8789`，工作区干净。
实际主机没有其他 cwd 为本工作区的 Codex 或 PX4/Gazebo 会话。单进程顺序修改，不提交/推送。

## 节点职责和依赖

来源：当前 modular_intercept.launch.py、baseline.yaml、setup.py 与各模块订阅/发布代码。
默认前视理想 RGB-D、下视关闭、shadow/evaluator 开启；原图13节点，附一个研究节点为14；独立轻量入口实测11节点。

| 节点/源码 | 输入 → 主要输出 | 职责/控制角色 | 启动条件 |
|---|---|---|---|
| moving_target / moving_target.py | impact/command、Gazebo服务 → target/state | 模拟环境，真值仅评价 | 默认 |
| target_predictor_node / tracking/target_predictor_node.py | tracking/target_state、mission → planning/target_prediction | 主BCTRA，控制所需预测 | 默认 |
| shadow_target_predictor_node / 同上 | tracking/target_state、mission → shadow预测 | 同输入同参数的重复BCTRA，独立诊断输出 | enable_shadow_perception |
| intercept_planner_node / guidance/同名.py | navigation、主prediction、mission → planning轨迹 | 已验证截获MINCO，保留 | 默认 |
| trajectory_tracker_node / control/同名.py | PX4、planning轨迹、KF、front bearing、mission → fmu/in | 唯一Offboard控制者；不要合并优化负载 | 默认 |
| mission_manager_node / mission/同名.py | command、tracker反馈 → mission/state | X/Y状态机，保留 | 默认 |
| intercept_evaluator_node / evaluation/同名.py | truth、导航、任务、视觉 → 结果/日志/终态 | 评价真值；终态/暂停有系统作用 | enable_evaluator |
| target_kalman_filter / tracking/同名.py | front target_position → tracking/target_state | 主KF，保留原新鲜度与来源 | 默认 |
| dual_tof_image_bridge / ros_gz_image | Gazebo前视RGB/depth → camera/front | 主输入桥，名字不代表启用下视 | 默认 |
| target_bearing_node / perception/同名.py | RGB、Gazebo双时钟 → front/target_bearing | RGB检测、方位/视觉锁；失深度时仍有安全作用 | 默认，必须保留 |
| rgbd_target_localizer / perception/同名.py | RGB/depth、PX4姿态/导航/timesync、Gazebo clock → front/target_position/observation | RGB-D定位，history因果配对 | 默认，必须保留 |
| front_tof_monitor / perception/同名.py | RGB/depth、可选真值 → front诊断 | 检测/ToF状态/评价，非主控制几何输入 | enable_shadow_perception |
| dual_tof_selector / perception/同名.py | front/down诊断 → usv_visible/active_camera等 | 诊断汇聚，本前视控制使用独立bearing/KF | enable_shadow_perception |
| front_tof_depth_model / perception/tof_depth_node.py | 理想depth → 功能ToF depth | 可选传感器模型 | front_depth_model=tof |
| down_camera_image_bridge / ros_gz_image | Gazebo下视 → camera/down | 可选传感器桥 | enable_down_camera |
| down_tof_monitor / front_tof_monitor.py | 下视RGB/depth → down诊断 | 可选诊断 | down且shadow |
| 原follow_mpc_shadow_node / controllers/同名.py | 同步nav/attitude、主BCTRA、mission → research JSON | P2研究，无飞行权 | 独立launch |
| 新follow_research_shadow_node | 同上，复用P2 runner/adapter → 同一research JSON | 单算法greedy_seed/greedy_minco/mpc_seed | 新独立入口，不与旧研究节点同启 |

## 重复计算判断与本轮选择

- 当前主/shadow BCTRA同读KF、参数相同，仅输出topic不同，属于真正重复预测；独立诊断目的合理，轻量研究关闭shadow。
- bearing/localizer都进行RGB检测，但前者不依赖深度且参与搜索/失锁，保留职责隔离；不能为降节点数删除。
- monitor也执行RGB/深度检测；selector汇聚诊断。关闭两者不影响主bearing/localizer/KF链，使用已有enable_shadow_perception开关。
- 同步和任务处理直接复用P2 ShadowInputAdapter/ShadowResearchRunner；新增solver与request factory注入，不复制ROS节点主体。
- 新FOLLOW使用原MincoS3Trajectory映射、解析control_effort、时间softmax，保持旧Fast MINCO及其行为不动。
- 投影/姿态唯一实现为P1 camera_visibility和P2 planned_attitude。新的pure problem组件只组合模型，不实现另一套坐标变换。
- 原感知各进程独立history有合理故障隔离；不在本轮抽取/重写它们的ROS时钟状态。长期可共享无状态数学库，但不要共享可变history。

## 实施计划与约束

按照用户已授权连续执行：

- [x] 纯算法输入/未来执行起点、连续yaw、贪心两个航点；RED→GREEN。
- [x] 原MINCO的固定Q/T、Q优化、Q/T联合优化与严格采样/自适应验收；RED→GREEN。
- [x] 在旧research runner增加注入点，统一三模式入口；旧MPC规则保持。
- [x] 复用P2场景做三初始化同优化器对照及四种消融；单算法顺序计算。
- [x] 增加algorithm/related/full测试脚本，记录耗时；不删除旧安全测试。
- [x] 既有启动脚本启动轻量/完整研究图，核对QoS、唯一控制权、负载、延迟；有界结束，仅清理本轮PID组。
- [x] 最终全pytest/lint/build/diff/范围审计、结果报告，停止P3。

Ruling：用户指定当前工作区并禁止多修改进程，本轮不创建其他checkout/子代理；不执行技能模板中的提交步骤。
Ruling：研究未来起点用明确常加速度短时投影，保留实际sample/observation/source epoch；这是研究预测边界，不是Tracker已接纳交接状态。
原始输入与最终publication仍遵守125ms，未来执行epoch不能使陈旧观测重新有效。后续真实交接必须由Tracker已接纳轨迹提供期望P/V/A。

## 测试分层

Level1不source ROS：pure几何/姿态/MINCO/yaw/新研究算法；P1文件的三个localizer兼容案例单列Level2。
Level2 source ROS和新overlay：原消息/adapter/runner、新入口和合法JSONL回放，均无需运行PX4/Gazebo。
Level3完整仿真只在集成阶段运行；完整pytest保留作为最后验收。
现有scripts没有统一pytest入口，新增test_research_planners.sh三模式；暂不合并或删除既有fixture和安全测试。

## 本轮实际精简与测量

完整入口F1：原13节点加follow_research_shadow_node，共14；启用evaluator。
在同一会话只停止本任务ROS进程组里的shadow predictor、front monitor、selector三个进程，主链继续FOLLOW。
F1负载采样时剩余11个节点进程均存在；后续一次短发现node list只列出9个名字，不能仅凭该快照推断两节点退出。
F2由轻量Launch独立启动，5秒ROS发现明确列出11节点，主prediction通信及FOLLOW状态正常。
因此11的验收依据为独立F2图和F1准确进程清单，不使用不完整发现快照补造名单。

| 20秒窗口 | 节点数 | CPU平均(一个核=100%) | RSS合计MiB | 研究CPU/RSS |
|---|---:|---:|---:|---|
| F1完整八字 | 14 | 265.14 | 1383.07 | 见f1/analysis.json |
| F1关闭三个诊断后 | 11个节点进程 | 224.80 | 1161.41 | 见f1/analysis.json |
| F2独立轻量直线 | 11 | 226.11 | 1155.58 | 见f2/analysis.json |

CPU/RSS涵盖本轮Gazebo/PX4/DDS/ROS四个进程组，不含QGC、CLI探针和Codex。
RSS是各进程RSS的和，共享页可能重复计数；不是整机唯一物理内存。
不同窗口运动状态/求解成功率不同，负载变化不能证明同输入下规划性能的因果改善。
只运行greedy_minco一个研究规划器；旧MPC保留作串行离线对照。
两轮轨迹setpoint均实测只有trajectory_tracker_node一个发布者；研究节点只订阅导航/姿态/timesync/主预测/任务状态。

## 测试工程实际结论

原pytest本就不需要运行SITL；很多ROS包装器测试用消息类型和模拟依赖。
本轮没有发现必须删去的完全等价安全回归，也没有通过删测试降低耗时。
新纯核心不导入rclpy，测试脚本algorithm无需source ROS。
Level2复用现有姿态/预测fixture，回放器复用P2消息配置恢复与场景生成，避免复制另一套数据模型。
同一当前版本的入口对照：algorithm约1.3秒，related约3.7秒，full约10.2秒（含shell/pytest初始化）；
精确最终检查另见greedy_minco_validation.md。这是日常入口的成本对照，不是历史版本性能结论。
完整回归与两轮真仿真仍执行；无需每次纯算法修改重跑约300秒的系统会话。

## 代码复用与后续重构边界

本轮新增四个纯组件：follow_problem、greedy_follow_initializer、follow_minco_optimizer、yaw_trajectory；
复用camera_visibility、planned_attitude、MincoS3Trajectory，没有另一套投影/五次多项式。
研究运行器新增solver/request_factory注入点；原MPC节点默认算法、订阅、QoS、reset/mission/原始stamp语义保留。
FollowResearchShadowNode是小型子类，只有算法参数/工厂；没有复制一套ROS回调。
单个新Launch支持完整、轻量及start_flight_stack=false旁挂；原baseline及旧Launch不变。

以后再处理：感知无状态红球检测可共享库但不要把故障隔离通路合并；研究runner可以迁移到中性命名公共模块；
保留各进程的可变history；研究拒绝日志在非FOLLOW时有大量重复invalid/mission事件，可改善去重；
离线JSONL曲线/系数输出体积和有限差分优化CPU/GIL负载需要独立评估。
不把heavy solver放进20Hz Tracker，不在本轮改动时钟桥、姿态来源、主KF/BCTRA或安全状态机。


最终检查：algorithm183，related326，full1211通过/1跳过；最终构建2包成功。F3固定终点Vz复验详见验证报告。
