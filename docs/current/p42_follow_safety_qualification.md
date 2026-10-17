# P4.2 独立安全资格与接口结论（2026-10-09）

结论：共享 epoch 和新版本名义重验已实现；实际 holding、首次桥接及状态失效回退尚无足够证据，MINCO 接管保持关闭。
`follow_minco_authorized=False` 没有被翻转；实际 Tracker 未注入 `SafetyApproval` 或 bridge fixture，没有新增 PX4 setpoint 发布者。

## 接收端拥有的时间代

新增 `FollowReceiverState` 只读心跳：Tracker boot identity、复位 generation、mission、当前预测版本以及资格状态。
Tracker 启动生成一次随机 boot identity；普通消息不重置身份，ROS 时间倒退或原始导航 native sample 复位才增加 generation。
规划器独立 boot identity 区分重启；其导航配对 adapter generation 继续保留，不能当作接收端 generation。
每次任务 dispatch 保存接收端 epoch，完成时要求与最新心跳一致，并验证唯一实际 Tracker 发布者、心跳时效、mission 和本地导航 generation。
只有 proposal 的 wire generation 使用这份已核实的只读身份；Tracker 从不采纳 proposal 的 generation 作为自身状态。
ACK 绑定 receiver boot、planner boot、mission、generation、plan，并要求实际 Tracker 唯一发布者与发布/收到时间一致。
旧 receiver boot、旧 planner boot、错误 ACK、未来心跳、mission/clock 改变均有测试。
这依赖受控 ROS 图身份，尚无 DDS 安全认证；不构成面对恶意同名节点的加密信任协议。
任何 native sample 倒退均保守撤销该代；后续应区分乱序与真正重启，避免多余拒绝，但不可因此放宽门禁。

## 预测版本策略

A：默认 `latest_only`，求解中版本更新直接拒绝旧候选。
B：专用 P4.1 配置使用 `full`，保持导航采集戳、规划起点和完整多项式不变，仅用新预测的明确 source/observation/generated/valid_until/sequence/P/V 替换预测输入。
对整条曲线重新计算完整姿态、整球水平/垂直 FOV、有效距离、预测覆盖、海面、动力学、yaw 和期限，并记录新预测完整快照及重验耗时。
全轨迹采样和两级临界区间细分仍是名义几何验收，不构成连续时间 FOV 证明。动力学另用 Bernstein 上界或解析极值。
旧导航过期不能被新预测刷新；重验完成及消息分配后再查原始 125ms、未来起点、mission 和共享 epoch。
新增重验、消息分配和最终 proposal 发布均计入完整 cycle 预算及独立最终审计；过期不发 proposal，发布跨限显式记录。接收端仍要求当前 prediction sequence 完全匹配；消息在传输中再次被更新时继续 `PREDICTION_CHANGED` 拒绝。
C 的差异界增量证明未实现，没有固定距离阈值放行。

## 连续时间和不确定性

现有 XYZ 五次多项式导数使用解析极值；可接受的 Bernstein 凸包上界可提前通过，界过宽时仍回到解析验证。
这些界只约束名义曲线，不能证明 PX4 实际加速度、jerk 或姿态满足同样限制。
整球目标、28° 下倾、实际平移外参复用 P1 几何，不改变相机/世界模型。

新增纯研究函数 `interval_visibility`：给定整个区间的 UAV/目标位置误差界、外参平移误差界、相对位移界和总旋转角界 θ，令

`E = Euav + Etarget + Emount + Emotion + 2 (||q|| + ||t_mount|| + Euav + Etarget + Emount + Emotion) sin(θ/2)`。

这是保守的光学中心球界，显式计入安装位置杠杆；θ 必须包含规划/实际姿态偏差、安装旋转误差和区间内名义相机转动。
将目标球半径从 r 膨胀为 r+E。对四个安全视锥单位法向 n 检查 `n·q >= r+E`，并检查 axial/radial 距离与整球前方条件。
这些平面条件是完整球与视锥边界的连续几何条件，不是将采样密度增加后宣称连续安全。
输入 `relative_motion` 必须覆盖整个待证区间，可由已证明的相对速度界乘区间长度形成；旋转变化也需连续界。缺失、非有限或负误差界均拒绝。
独立已知几何测试覆盖中心、边缘、大位置误差、大姿态误差、安装平移误差、区间运动和无界输入。
即使数学界通过，函数仍返回 `holding_qualified=False`，不生成 SafetyApproval，不设置 holding_until，也不订阅真值。

尚缺：因果目标预测误差的有效区间界、实机执行位置/姿态包络、控制响应超调/滞后、外参标定误差、区间旋转界、海面/制动不确定性及恢复时间。
历史/本轮实际测量的分位数或最大误差不能自动成为未来物理保证。本轮没有为了放行虚构这些数值。
没有可信界就不能以 `holding_until=trajectory_end` 授权；原 proposal holding 继续为零。

## 首次桥接与回退门禁

原 FOLLOW 输出速度反馈，没有可直接采样的未来位置/速度/加速度多项式。
当前测量 P/V/A 是实际状态估计；当前已发速度命令是控制输入，二者不可当作同一个起点。
合法桥接需要 Tracker 同刻拥有测量 P/V/A、已发命令及其整形状态、yaw/rate、控制模式、锁定状态，并证明未来短区间内实际响应误差和模式切换有界。
MINCO 五次边界插值只能证明参考 P/V/A 连续；无法单靠它证明当前速度模式下测量与已发命令一致。
本轮没有形成具有真实误差依据的首条 bridge，未接纳第一条 MINCO，未更改原控制状态。

原 Tracker 状态过期分支仍为 WAITING_FOR_STATE，不发布新 setpoint；没有把它称作 MINCO 的已验证安全回退。
从 MINCO 返回 FOLLOW、搜索或等待所需的有界命令、撤销时刻、有限保持与 PX4 failsafe 策略仍须独立设计验证。
不能无限维持最后速度，也不能在无可信状态时随意命令爬升/刹车并声称安全。
因此第七阶段实际执行分支和实验 C/D/E 的 MINCO 接管按用户安全条件停止；继续影子/协议/故障研究。

## 门禁状态

消息生成、共享 epoch、125ms 原始期限、预测覆盖、P/V/A 与 yaw/rate 纯协议、名义 FOV、名义动力学已可测试。
独立 holding、真实首次 bridge、实际模式切换、状态失效有界 contingency 未通过。
实际跟随仍为原 FOLLOW，默认开关为 false，baseline.yaml 与原截获语义保持不变。
纯协议的 ACCEPTED/ACTIVE 合成测试只验证状态机，永远不能计入真实 Tracker 接纳次数。

P4 FOLLOW 消息追加 boot identity 并新增只读状态消息，需要 planner/tracker/interfaces 一起重新构建。本轮四包构建已完成。原 InterceptTrajectory 等控制消息未修改，原 FOLLOW 的 timer、命令整形和 yaw 方法 AST 与基线一致。
