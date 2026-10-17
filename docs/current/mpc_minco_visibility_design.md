# P1：MPC–MINCO 共用相机可见性几何与接口设计

## 范围与当前证据

2026-10-08 执行前核实：工作区 `/home/qin/data/uav_usv_mpc`，分支 `master`，
HEAD `04cf9e78f0cd0a37cc42dfb642a6499e7a8cd796`，工作区干净。
未访问 GitHub、fetch 或 pull；旧工作区只读。

当前仍为 RGB-D → KF → BCTRA → Fast MINCO → tracker；tracker 保持唯一 PX4
Offboard 控制权。新模块是未接入节点的纯算法，未来由 MPC 与 MINCO 共用。
本轮不实现优化器、FOLLOW、Y 后 PNG–IBVS、姿态控制或 ROS 消息变更。
`mission/target_visibility.py` 仍负责图像锁定、偏航搜索和失锁决策；本模块只回答
给定姿态下的几何问题，不能替代观测有效性、freshness 或任务安全策略。

当前依据（均读取本地源码）：

- `models/x500_mono_cam/model.sdf`：前视安装下倾 28°，相对 base_link 平移
  `(0.18, 0, 0.15)` m，640×480，水平 FOV 1.74 rad，深度裁剪 0.05–25 m。
- 本机 PX4 的 `Tools/simulation/gz/models/x500_base/model.sdf`：模型 pose 的
  z 偏移 0.24 m。现有 localizer 将模型原点作为导航参考，故总平移为
  `(0.18, 0, 0.39)` m FLU；与当前 `baseline.yaml` 一致。
- `rgbd_target_localizer.py`：光轴深度为 camera-FLU 的 X；图像右/下对应 -Y/-Z。
  `camera_target_to_local_ned` 与 `local_ned_target_to_camera_flu` 是正反变换依据。
- `VehicleAttitude.msg`：Hamilton 四元数 wxyz，FRD→NED，含 `timestamp_sample`。
  localizer 将原始 sample epoch 因果映射后保存在 attitude history。
- `UavState.msg`：物理 sample epoch 下 P/V/A 和 heading，**不含完整姿态**。
- `MincoS3Trajectory.sample` 返回 P/V/A/jerk；Fast MINCO 当前验收速度、加速度和
  海面净空，并未约束视场。tracker 的反馈、限速、加速度整形会改变实际执行状态。

## 坐标系与相机外参

所有旋转采用主动旋转，`R_AB` 将 B 中向量变为 A 中向量，长度单位 m、角度 rad。

| 系统 | 世界轴 | 机体/相机轴 |
|---|---|---|
| PX4 | local NED：北、东、下 | FRD：前、右、下 |
| Gazebo | ENU：东、北、上 | 模型/相机 link FLU：前、左、上 |
| 图像光学 | 无独立世界系 | optical：右、下、前（x、y、z） |

定义 `D=diag(1,-1,-1)`（FLU↔FRD），
`E=[[0,1,0],[1,0,0],[0,0,-1]]`（ENU↔NED），
`C=[[0,0,1],[-1,0,0],[0,-1,0]]`（optical→camera-FLU）。
若显式使用 Gazebo 模型姿态 `R_ENU_FLU`，则 `R_NED_FRD=E R_ENU_FLU D`。
位置需用相同原点变换 `p_NED=E(p_ENU-o_ENU)`；不能把 ENU 数值直接传入 NED API。
Gazebo 数据仅能用于离线一致性检查，在线调用必须来自导航与视觉/KF/预测。

相机安装旋转 `M=R_bodyFLU_cameraFLU=Rz(yaw) Ry(pitch) Rx(roll)`。
正 SDF pitch 下倾相机 +X。本轮用 `CameraExtrinsics.from_sdf_pose` 构造完整安装
旋转，平移必须已相对于导航所用的机体参考点，不在函数中自动补 0.24 m。
未来真实设备/不同参考点必须重新标定，不能直接套用当前仿真外参。

给定机体实际/显式预测姿态 `R=R_NED_FRD`、机体位置 `p_u`、安装平移 `t_FLU`：

```text
p_camera_NED = p_u + R D t_FLU
R_NED_optical = R D M C
c_optical = R_NED_optical.T (p_target + offset_NED - p_camera_NED)
c_cameraFLU = C c_optical
u = fx * c_optical.x / c_optical.z + cx
v = fy * c_optical.y / c_optical.z + cy
```

相机平移随 UAV 姿态旋转；目标的 world/NED offset 不随 UAV 旋转。
当前 KF 的目标位置是船体参考点：localizer 给红球中心加 `target_reference_z_offset=+0.42`
NED z。因此投影红球时显式传 `TargetBoundingSphere(0.25, (0,0,-0.42))`；
禁止把船体参考点误当成红球中心或再次加 +0.42。这里是已知目标几何定义，不是误差补偿。
真实船体可由检测/几何模型提供中心 offset 和尺寸，不自动引用仿真真值。

## 几何模型与数学裕度

`CameraIntrinsics` 接收完整 fx/fy/cx/cy；`from_horizontal_fov` 采用现有 square-pixel
约定 `fx=fy=width/(2 tan(hfov/2))`，`cx=width/2, cy=height/2`。
当前模型无畸变；非针孔或畸变图像必须先矫正或另建显式模型。
视场采用连续传感器边界 `[0,width]×[0,height]`，与现有 hfov 推导一致；
离散像素索引仍为 0…width-1/height-1。边界允许等号，非零安全裕度使目标落在内部。

目标采用保守包围球，半径 r≥0（r=0 是点目标）；
`from_box_dimensions((L,W,H))` 用 `r=sqrt(L²+W²+H²)/2`，覆盖未知船体朝向，
可能比已知朝向的包围盒更保守。offset 在 NED 下显式传入；随船体转动的非中心 offset
须由上层模型先变换，模块不猜测 USV 姿态。

对 optical `(x,y,z)`，中心在前方要求 `z>depth_epsilon`；
整体在前方要求 `z-r>depth_epsilon`。满足后，球的**精确角度投影区间**为：

```text
theta_h = atan2(x,z), alpha_h = asin(r / hypot(x,z))
theta_v = atan2(y,z), alpha_v = asin(r / hypot(y,z))
h_interval = [theta_h-alpha_h, theta_h+alpha_h]
v_interval = [theta_v-alpha_v, theta_v+alpha_v]
h_safe = [atan2(-cx,fx)+margin_h, atan2(width-cx,fx)-margin_h]
v_safe = [atan2(-cy,fy)+margin_v, atan2(height-cy,fy)-margin_v]
mu_h = min(h_interval.low-h_safe.low, h_safe.high-h_interval.high)
mu_v = min(v_interval.low-v_safe.low, v_safe.high-v_interval.high)
```

裕度单位 rad，正数为剩余空间，负数为违反量；支持偏离图像中心的主点及 fx≠fy。
不能简单用 `asin(r/三维距离)` 缩减左右/上下 FOV，它在另一轴偏离时会低估轮廓。
输出投影包围框由各角度端点的 tan 映射到像素，故区分中心在图像内与整个球在安全视场内。

有效量程由调用者显式选择：

- `distance_mode='axial'`：相机光轴深度，球区间 `[z-r,z+r]`。
  当前理想 RGB-D/localizer 的 0.05–25 m 门限属于该语义。
- `distance_mode='radial'`：欧氏距离，区间 `[max(0,||c||-r),||c||+r]`。
  可选 ToF 模型配置的 0.25–25 m 是径向量程，不能与光轴门限混用。

整体安全要求整个保守区间落在量程内；这比只要求部分可见表面能测距更严格。
中心 RGB 投影、中心距离有效和整体安全分别返回，不因深度超量程而丢弃中心投影。
量程只是配置几何门限，不代表已证明真实设备性能。

API：`evaluate_visibility(uav_position_ned, body_frd_to_ned, target_position_ned,
intrinsics, extrinsics, target, constraints)`，姿态为明确的 3×3 SO(3) 矩阵。
`body_frd_to_ned_from_quaternion` 提供 PX4 wxyz 适配，不订阅 ROS。
返回 `CameraVisibilityResult`，包含两个相机坐标位置、中心/包围框像素、前方标志、
中心/整体距离标志、中心可见标志、两个裕度、整体安全标志及原因 tuple。
非法输入返回 `valid_input=False`；几何不可见仍为 `valid_input=True`。
未定义投影/裕度为 `None`，不能把它当 0。独立构造函数/适配器的非法输入抛 ValueError。
原因包括非法位置/姿态/内外参/约束/目标、零深度、后方、跨相机平面、
水平/垂直视场、近/远量程和数值溢出；不将反射/非正交矩阵自动正交化。
模块不负责遮挡、图像内容、检测置信度、时间同步或数据来源认证。

## 姿态接口及后续假设

接口应分为三种不可互换的姿态：

1. **测量姿态**：PX4 VehicleAttitude 的 FRD→NED q，使用原始 sample epoch 的
   因果时钟映射；位置和姿态必须在同一评价时刻有有效历史 bracket，禁止 receipt time 替代。
2. **规划姿态**：未来 MPC/MINCO 根据规划 P/V/A 和 yaw 估计，带模型版本、有效域和误差界。
3. **安装姿态**：标定的 camera-FLU→body-FLU 固定 M；与上述两者相乘得到相机姿态。

未来理想四旋翼平坦性假设可写成（NED，推力沿 -body-z）：

```text
a = g*e_down - (T/m)*b3    # 忽略气动阻力、风和执行误差
b3 = (g*e_down-a) / ||g*e_down-a||
h = (cos(yaw_ref), sin(yaw_ref), 0)
b2 = normalize(b3 × h); b1 = b2 × b3; R_pred = [b1 b2 b3]
```

该式中的 yaw_ref 是水平参考方向，倾斜时 b1 的水平 heading 未必严格等于它；
后续必须选择“平坦输出 yaw”或“严格 Euler heading”的具体定义。
理想方向映射不需要 mass，但推力/角速度/执行可行性需要它。本轮不增加该函数，
因为尚未验证规划 a 与实际姿态的对应关系；速度本身不能确定滚转/俯仰。
需明确重力标定、推力方向与 T/m 上下限、质量/阻力（如采用）、风/模型残差、最大倾角、
角速度/偏航速度限制、jerk 对角速度的影响、控制延迟与误差界。
`a≈g*e_down`（自由落体）或 `b3×h≈0`（航向退化）应拒绝，不能回退为水平姿态。
tracker 反馈、限速、海面保护及 PX4 内环使实际 a≠MINCO a，必须先离线验证模型并量化误差。
当前 `UavState` 的 heading 无法提供实际 roll/pitch；完整姿态上层接口的传输/同步设计
是下一阶段工作，本轮保持 ROS 消息不变。

## 未来 MPC 输出与 MINCO 输入（仅设计）

建议初始 MPC 用运动学三阶积分器，状态 `x=[p_NED,v_NED,a_NED]`，输入 jerk，
另附 yaw/yaw_rate 参考。若 yaw 是优化变量，扩展状态而非假定姿态固定。
离散更新用同一 dt 下精确积分：p+=v dt+0.5 a dt²+j dt³/6，v+=a dt+0.5 j dt²，a+=j dt。
该模型仅为轨迹生成接口建议；带动力学/视场非线性的优化器与求解器尚未选定。
约束包含现有水平/垂直速度与加速度、海面净空、yaw rate、jerk、模型适用域/倾角/推力、
目标整体 mu_h/mu_v≥0、前方和量程。实际限制沿用经审核配置，不在本轮填入新值。

建议用内部不可变结构，暂不增加 ROS 消息：

| 结构 | 建议字段与语义 |
|---|---|
| PlanningContext | mission_id、cycle_id、frame_id、clock_domain、clock/reset_generation；navigation_stamp、attitude_stamp、observation_stamp、prediction_source_stamp、prediction_sequence_id；execution_start_stamp、previous_accepted_plan_id、start_PVA、end_PVA、model/calibration/constraint_version |
| MpcSeedTrajectory | context；严格递增 relative_times（首个为 0）、positions[N,3]、velocities[N,3]、accelerations[N,3]、jerks[N-1,3]、yaw_refs[N]、yaw_rates；每点 visibility_result；状态/输入限制与违反量；solver_status、solve_wall_seconds、residuals、infeasible_reasons |
| MincoSeedInput | 同一个 context 与 MpcSeedTrajectory；选取的内部 waypoint indices/positions；正的 segment_duration_initials；相同 start/end P/V/A；CameraIntrinsics/Extrinsics/TargetBoundingSphere/VisibilityConstraints；同一 target prediction snapshot 及覆盖时间；规划姿态模型版本 |
| RefinedTrajectory | context；分段多项式与 durations；yaw schedule；完整重验可见性/动态约束结果；generation/validation/total_wall_seconds、status/reasons |

MPC 初始 P/V/A 与 MINCO 边界必须精确相同。航点选取保留原相对时刻，分段时间初值为
相邻保留时刻差；改时间后必须在新绝对时刻重新查询目标预测并复验整段约束。
MINCO 即使通过航点，也可能在中间越出视场；不能把 MPC 样点可见当成 MINCO 整段可见。
必须增加自适应采样/区间界和误差膨胀；本轮裕度函数不是连续时间安全证明或解析梯度。
包围球和相机参数由同一版本快照供两阶段使用，禁止各自使用不同外参/裕度。
失败状态建议区分 INVALID_INPUT、STALE_INPUT、PREDICTION_HORIZON、DYNAMICS_INFEASIBLE、
VISIBILITY_INFEASIBLE、ATTITUDE_MODEL_INVALID、DEADLINE_EXCEEDED、HANDOVER_MISMATCH；
成功必须完整验收，未来失败策略另行设计，不改本轮行为。

## 共用执行起点、时间基准与接替

**execution_start_stamp=t_h 是计划执行起点，不是计算开始、生成、发布、收到或测量时刻。**
MPC 与 MINCO 共用 t_h，样点绝对时刻 `t_k=t_h+relative_times[k]`；
查询 BCTRA/未来 IMM 的相对时间为 `t_k-prediction_source_stamp`，不得重写预测 source_stamp。
相机姿态、UAV P/V/A 与目标位置必须在同一个 t_k。几何模块没有时钟，责任在调用适配层。

已有接纳轨迹时，新起点取上一条实际已接纳轨迹在 **t_h** 的期望 P/V/A，
不是它的未来终点，也不是求解时刻下的测量位置。应确认 tracker 的 accepted plan id；
pending/未接纳候选不能成为连续性依据。当前 `select_planning_start_state` 与
`ActivePlanReference` 已提供相应参考，tracker `accept` 已比较交接 P/V/A。
旧轨迹 `sample` 会端点夹取，未来适配层须先检查 t_h 位于旧轨迹有效执行域，
不能靠夹取后的端点制造“连续”。无有效旧轨迹时，使用显式测量状态和经验证的短期状态
传播策略，保留真实 navigation_stamp，禁止将传播状态伪装为测量。

预算包括 MPC、MINCO、复验、传输和接替等待。若实际交接比约定 t_h 晚，必须根据真实
交接时刻比较两条轨迹 P/V/A 并重新验收；不满足则拒绝/重新规划，不能只把 source_stamp
改为 now。当前 `PolynomialTrajectory.source_stamp` 表示轨迹时间原点；与目标预测的
source_stamp 同名但不同义。保持现有消息语义，未来适配层应显式使用 execution_start_stamp。
时钟/位置/姿态 reset、预测失效、覆盖不足应使候选失效，保持图像戳、因果等待及超时拒绝。

## 实现与验收步骤

按照用户提供的 P1 任务范围直接执行；不暂存、提交、推送，不改现有算法或配置。

1. 新增 `test/test_camera_visibility.py`：先用已知正交轴/像素边界、旋转符号、近远距离、
   球轮廓切线和非法输入固定接口，再执行一次缺少模块的失败检查。
2. 新增 `guidance/camera_visibility.py`：数据类、纯坐标变换与完整球视场判断，NumPy/SciPy
   为现有依赖；不导入 localizer/ROS/evaluation，避免把在线几何绑定 ROS 或真值诊断。
3. 增加与现有 localizer 正反变换、内参及当前模型/配置的兼容性测试；独立几何案例仍为主。
4. 在包目录运行新增/相关回归及全 pytest；运行指定 flake8、shell 语法、构建脚本、
   包前缀与新模块无 ROS 导入检查、diff/非授权路径检查；在本文记录本轮结果。

## 验证记录与进入下一阶段的条件

最终只新增模块、测试、本文三个文件，并在 `docs/current/README.md` 增加一行入口。
HEAD/分支未改变，暂存区为空；现有控制、感知、BCTRA/KF、Fast MINCO、ROS 消息、
模型、配置、PX4 补丁和冻结数据均无 Git 改动。没有启动飞行仿真、发布指令或使用真值
进行在线计算。旧仓库未写入。

本轮实际检查：

| 检查 | 最终结果 |
|---|---|
| 新增测试（包目录，source ROS/新 overlay） | **74 passed** |
| 不 source ROS，排除三个 localizer 兼容性参数案例 | **71 passed, 3 deselected**；纯几何可独立使用 |
| 相关既有 RGB-D、ToF、姿态诊断、锁定、MINCO、tracker、真值隔离回归 | **224 passed** |
| 包目录 `python3 -m pytest -q` | **982 passed, 1 skipped, 2 warnings**；既有 copyright 跳过、ament flake8 的 SelectableGroups 弃用告警 |
| 根目录规定范围 flake8 | 退出 0；无错误 |
| 四个现有 shell 脚本 `bash -n` | 退出 0 |
| 根目录 `git diff --check` / `git diff --cached --check` | 退出 0；暂存区为空；新增文件另做 whitespace 检查 |
| `UAV_USV_WS=新工作区 ./scripts/build_workspace.sh` | **4 packages finished**，退出 0 |
| 最后几何修正后 `build_workspace.sh --packages-select uav_control` | **1 package finished**，退出 0 |
| source 后 `ros2 pkg prefix uav_control` | `/home/qin/data/uav_usv_mpc/install/uav_control` |
| 已安装新模块的实际源路径 | resolve 到 `/home/qin/data/uav_usv_mpc/src/uav_control/uav_control/guidance/camera_visibility.py` |
| 改动路径审计 | 仅上述四个授权代码/测试/文档路径，无已有运行文件变更 |

复现命令：

```bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
cd /home/qin/data/uav_usv_mpc/src/uav_control
python3 -m pytest -q test/test_camera_visibility.py
python3 -m pytest -q
cd /home/qin/data/uav_usv_mpc
python3 -m flake8 src/uav_control/uav_control src/uav_control/test scripts
bash -n scripts/build_workspace.sh scripts/start_px4_ros2.sh scripts/uav_lab.sh scripts/sync_github.sh
git diff --check
git diff --cached --check
UAV_USV_WS=/home/qin/data/uav_usv_mpc ./scripts/build_workspace.sh
ros2 pkg prefix uav_control
```

开发中的失败及处理：首次测试按预期因模块尚未存在收集失败；扩展非法类型案例发现
未分类异常，补充验证后通过。一次未 source ROS 的兼容性检查出现三个 rclpy 导入失败，
随后使用正确 overlay 通过；纯算法检查明确排除这些兼容性案例。首次 flake8 报三处
测试续行缩进，首次全 pytest 为 980 passed/1 failed/1 skipped，唯一失败是新模块两处
PEP257 D213；均修正并复跑。最后一次静态命令曾在包目录使用根目录相对路径而报
E902，改回根目录运行通过。只读审查指出“相机处于球内时径向下界不能为负”，先复现
测试失败，再修正下界为 0；新增和完整测试、包构建均在该修正后重新通过。
没有保留失败检查项，也没有以历史结果代替本轮结果。

只读审查未发现 Critical 或算法 Important 问题。几何测试与 localizer 约定兼容；
使用已有 NumPy/SciPy，不新增依赖或入口。非零安装 roll/yaw 也有独立轴向测试。
本轮不执行 Gazebo 闭环、实时优化耗时、规划姿态/误差模型或实机测试：新模块尚未
接入运行链，故以上检查未执行，也不宣称有在线或连续时间安全验收。

可以进入**离线/影子模式 MPC 初始轨迹生成**的开发：纯几何模型和接口不依赖 ROS 控制，
并具备独立测试边界。不能据此进入在线 FOLLOW 或认定视场安全已实机验证。
下一阶段关键问题是完整实际姿态的同步输入、规划姿态模型/误差界、真实目标几何/遮挡与
标定、求解器及实时预算、交接接纳时刻、MINCO 连续时间可见性复验、失效策略。
红球只验证当前仿真感知接口，不能外推无标记非合作 USV 或真实海面 ToF 性能。
