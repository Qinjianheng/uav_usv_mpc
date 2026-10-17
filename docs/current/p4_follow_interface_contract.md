# P4 FOLLOW 接口与接管门禁

本轮实现进展型研究规划、完整 FOLLOW 系数接口和**实际 Tracker 接收端的拒绝 ACK**。没有实现获准的 MINCO 控制接管：独立安全持有模型、首次桥接和连续时间 FOV 误差包络尚不具备。`follow_minco_enabled` 默认 false；即便请求 true，`follow_minco_authorized` 仍为 false。保持原 FOLLOW，执行与 yaw 仲裁函数没有改动。不得通过给消息填写 model_id/holding_until 解锁。

## 数据链与控制权

`/tracking/target_state → 原 BCTRA → /planning/target_prediction → p4_follow_planner_node → /planning/follow_trajectory → trajectory_tracker_node → /control/follow_ack`。

规划器只有研究 JSON 和 FOLLOW 提案发布者，没有 PX4 发布者。`/target/state` 和 Gazebo entity pose 仅由独立离线 recorder/evaluator 读取。原 `InterceptTrajectory`、MissionState、Y 截获、TERMINAL_MINCO、PNG、OffboardControlMode 和 Tracker 控制输出语义不变。

原 FOLLOW 输出速度命令，没有可采样的未来 P/V/A 曲线。命令 V/A 和测量常加速度外推都不能当成旧已接纳 MINCO。首次接入必须由 Tracker 拥有的桥接过程提供未来参考及实际已发命令/yaw-rate 连续性证明；本轮明确拒绝 `INITIAL_BRIDGE_UNAVAILABLE`。

## 消息布局

新增 `FollowTrajectory.msg` 与 `FollowPlanAck.msg`；不新增重复状态消息。提案包含 plan/mission/generation/prediction/parent ID、九个独立时间字段、完整 XYZ/yaw 系数、约束与相机版本、nominal sampled 状态和诊断。接收端使用本地约束，不信任提案里的 dynamic_limits、FOV margins 或验证标签。

N 个分段，durations 为 SI 秒：XYZ 按 `(piece, degree, axis)` 排列，degree=0..5 升幂，axis=x/y/z，长度 N×18。与原 PolynomialSegment 的 axis-major 布局不同，必须走专用转换。yaw 按 `(piece, degree)`，degree=0..3 升幂，长度 N×4；发布时将 scipy CubicSpline 的降幂 `(4,N)` 转为此布局。位置统一 local NED，yaw 为 NED 平面角。P/V/A 与 yaw/rate 由同一绝对执行时刻采样，无外推、无终点别名。

`constraints-p4-v1` / `camera-p1-v1` 是本轮研究政策标签；实验同时保存完整参数、源码和 SHA256。它们尚不是自动认证的配置哈希，未来资格验证器必须绑定完整有效参数与相机校准指纹。

## 四类时间

- 原始输入：`input_until ≤ min(nav+.125, attitude+.125, observation+.125, source+.125)`，还受原 prediction_valid_until 限制。采集戳保留，发布/接收时间不得代替采集时间。
- 预测覆盖：`source + last_prediction_time`，插值不能外推。4 s 覆盖不等于 4 s 可执行权限。
- 执行区间：`[execution_start, execution_end]`，分段时间和区间必须一致；过了起点才收到的提案拒绝。
- 安全持有：独立接收端验证器批准的绝对期限，不可因新规划失败续期，也不得授权超出完整曲线末端的采样。本轮提案 holding_until=0、model_id 为空，明确表示**未建立**，不是自动写成轨迹终点。

冻结的 P32 489 个请求中，150 ms lead 对应执行起点比原输入期限晚约 51–146 ms。这不能通过重写 stamp、改大 125 ms 或仅缩短 lead 来取得安全证明。需要在输入新鲜时进行接纳，并由独立误差包络覆盖接纳后实际执行的短前缀与恢复时间。

## 纯协议与实际 ACK 的边界

`FollowReceiver` 的纯生命周期支持提案验证、ACCEPTED pending、ACTIVE replacement、REJECTED、REVOKED、EXPIRED。接纳前独立检查完整系数、解析动力学极值、内接缝 P/V/A/yaw/rate、版本/时间、任务/时钟、模式/锁定、最新预测版本、父参考及未来交接边界。拒绝候选保留旧 active/pending，不延长期限；预测版本变化先撤销，不能默认沿用旧验证。

后续起点仅从接收端 own active 的系数在新 start 时采样。yaw 角差使用圆周差，rate 单独校验。纯测试使用 1e-6 SI 容差，激活晚于 start 超过 1e-6 s 即拒绝；这属于研究严格门禁，**不是已标定的实际调度或飞行容差**。将来需定义控制 tick 的合法交接时刻及同刻再验收，不能随意放宽。

`SafetyApproval` 只能由构造时注入的接收端独立 verifier 产生，并绑定整个候选指纹和有效期限。单元测试中的 verifier/bridge 是已标记的合成 fixture；实际 ROS Tracker 没有注入它们，因此没有任何实际 ACCEPTED/ACTIVE 路径。控制接管分支也没有提前安装。实际 `/control/follow_ack` 的 REJECTED 来自 Tracker 回调；旁路研究可行结果、JSON 发布和合成 ACCEPTED 都不是接管 ACK。

规划器检查 ACK 的候选身份、当前任务/代、时间及图上的唯一 Tracker 发布者；本轮任何 ACCEPTED/ACTIVE 都记为 unexpected_accept，不转为授权。ROS 图和 receiver 字符串是实验来源审计，不提供密码学身份认证。

## 调度、暖启动与失效

P4 复用原导航/姿态因果配对、时间映射和原预测订阅。一个 ThreadPool worker；busy 时不构造或排队旧请求；回调缓存最新输入，完成时仍做 freshness/mission/generation/prediction/start 等原门禁。P4 的完成轮询为 10 ms，原研究节点保持 20 ms，Tracker 仍 20 Hz。

数学暖启动只复用 jerk 初值及归一化 switching 项，新的 P/V/A、预测和完整验收必须重新建立。数学 hint 可以超过旧测量 TTL，不能获得执行资格；任务/代/来源倒退拒绝跨界使用。P4 对单纯输入等待保留同任务/代数学 cache，实际新结果失败仍清空。本轮没有把 Q/T 尾段复用或“旧正式接纳尾段”伪装成已实现：实际 accepted reference 为零。

因为接管被关闭，新规划失败、拒绝、过期或研究节点退出均继续走原 FOLLOW/搜索/恢复链。原控制 timer/yaw 函数按 AST 保持一致。此结论不等于已经验证 MINCO→原 FOLLOW 的真实命令连续回退，也不掩盖原 Tracker 在 UAV 状态过期时返回 WAITING_FOR_STATE 的既有行为。后者必须在后续获准接管前建立独立有界 contingency，本轮不以未验证状态继续飞行。

## 后续不可跳过的资格工作

需要目标预测误差（包括 KF 初始误差）、机体跟踪误差和实际姿态误差的可审计界，以及误差传播到整目标 FOV、距离、海面净空、V/A/J 的连续时间上界。规划姿态是理想微分平坦映射，PX4 实测姿态和相机安装姿态独立；不能相互替代。相机仍为真实 28° 下倾安装，平移外参仍生效。

当前目标几何是红球 bounding sphere，不构成无标记 USV 整艇可见性认证。预测、风阻、姿态滞后、执行器及 plant-side 超调必须在持有模型中处理。建立资格后才能完成真实 bridge、20 Hz 交接协议、计划 yaw 与 TARGET_LOCK/SEARCH/HOLD 的仲裁、正式接管与有界回退；保持唯一 Tracker PX4 权限。

## 本轮现场时间协议发现

planner 的 adapter.clock_generation=1，Tracker 的局部回退计数=0；独立启动计数不是共享时间代。本轮因此产生真实 CLOCK_GENERATION_CHANGED 拒绝。未来必须提供接收端拥有的时钟/boot epoch 与同步复位协议，不得从提案抄入 generation 以绕过门禁。收到新预测与规划完成竞争也产生 PREDICTION_CHANGED，需对最新预测完整重验。

绝对秒编码先取整数sec，再编码fraction纳秒，避免直接对约1.79e9秒乘1e9所致精度损失。333原有效事件重新编码无误报INVALID_INTERVAL/TTL_EXTENSION；不更改原采集浮点值或125ms限制。消息纳秒字段不恢复原日志已丢失的亚浮点精度。

数值验收以受控浮点异常和LinAlgError转为INVALID_COEFFICIENTS，有限但不可计算的系数不能逃出实际Tracker回调。独立SafetyApproval期限必须有限；NaN/Inf不得取得授权。
