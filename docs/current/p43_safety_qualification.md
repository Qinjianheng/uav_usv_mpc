# P4.3 真实执行安全资格

本轮基点 `97c8c8c9e52b0d9f7f10595dcd7557ea2525592d`。只修改新工作区；未提交、暂存或推送。实验根目录：`data/experiments/20261009_p43_follow_optimization/`。历史报告只作背景，本页数值来自本轮原始 JSONL。红球验证仍属于仿真感知接口验证。

## 当前结论

**不能进入MINCO实际接管。** 真实Tracker可接收、解析和拒绝研究提案，未安装独立holding、首次bridge或有限安全回退依据；无SafetyApproval，`follow_minco_authorized=False`。H/I/J未执行，未通过夹具或强制变量制造ACTIVE。红球与本场景成功不能外推无标记非合作USV性能。

## 接收端可信约束

新增ResearchConfig仅允许研究XY加速度≤4.5、yaw≤1.5，其余旧限制不放宽。接收端由本地只读研究参数构造完整标定/限值快照；消息携带快照及SHA256声明，不能决定本地阈值。任何字段或指纹不一致明确拒绝，配置变化清除active/pending与旧批准。默认纯算法夹具可省略快照以保持旧测试，真实ROS接收端始终使用本地快照。

消息增加`FollowTrajectory.constraint_snapshot/constraint_fingerprint`与`FollowPlanAck.receiver_compute_seconds`；planner、tracker与接口包须一起重建，旧二进制线协议不保证兼容。本轮已重建全部4包及消息接口。ACK处理保留真实boot/mission/clock身份和唯一Tracker发布者检查。

## 实际PX4与模型证据

`px4_readonly/`保存本机PX4 HEAD、已有修改状态、4001参数、DDS、x500_base、x500电机模型和commander参数；只读采集，没有修改PX4。每轮ULog另保留真实启动参数，不能把airframe默认值代替启动快照。

实测参数包括MPC_ACC_HOR=3、MPC_ACC_HOR_MAX=5、MPC_TILTMAX_AIR=45°、MC_ROLLRATE_MAX/MC_PITCHRATE_MAX=220°/s、MC_YAWRATE_MAX=200°/s、MPC_THR_MAX=1；它们不是允许把研究yaw上限直接扩到机体p/q/r的证明。电机模型时间常数上升12.5ms/下降25ms、最大转速1000、motorConstant=8.54858e-6。忽略所有损失的4电机静态推力约34.19N，结合base、旋翼和两个相机的SDF质量约2.164kg，对应约15.80m/s²；现有名义比推力上限18未经物理校准，不能视为执行能力。本轮接受曲线远低于该名义上限。

ULog以原始PX4 boot sample time分析稳定20–60s：实际p/q/r、倾角、测量yaw rate、归一化推力、motor control代理、Offboard比例均保留。归一化推力不等于牛顿。导航与姿态不使用拟合延迟平移；在ULog原生时间上以≤20ms因果匹配计算名义平坦性推力轴与测量轴的角误差。这是基于实际加速度的模型残差，**不是尚未执行的MINCO预测姿态误差**。

修复后B轮，该角误差P95约0.0499rad、最大约0.0513rad，已大于AttitudeConfig预留0.03rad。预留值原本没有用于holding授权，本轮也未通过固定偏置或扩大门限来“修复”它。需要解释阻力、滤波、姿态控制与安装误差，并建立独立可复算界。

## 响应、预测与误差包络

`p43_analysis.json:response_model`对真实原FOLLOW速度命令做因果receipt≤125ms配对，前半拟合一阶a_xy=K(v_cmd−v)，后半留出检验。B轮tau约0.648s，留出加速度残差P95约0.584m/s²、最大约0.678m/s²；这是经验模型，不是可保证延迟或响应上界。命令receipt配对也不能用于重建导航原始sample epoch。

位置/速度评价、目标各时域预测误差和控制消息延迟另见在线报告。没有实际执行MINCO，因此没有该控制分支的跟踪超调、恢复时间或模式转换误差界；影子提高加速度/yaw只改变提案，不改变飞行响应。相机安装误差没有独立标定分布。全部经验分位数/实验最大值与可保证物理上界分开，holding误差源仍不充分。

## 首次bridge与回退设计（未获资格）

进入桥接需要同刻真实测量P/V/A、已发速度/加速度及yaw/yawspeed、控制模式、实际姿态、未来执行epoch，并验证PX4在该命令历史下的响应包络。多项式与旧期望曲线P/V/A连续只证明参考连续，不能证明首次切换的真实机体连续性。不得用旧轨迹未来终点作为统一起点。

| 事件 | 当前证据/行为 | 进入MINCO前所需有限协议 |
|---|---|---|
| 状态短延迟/缺失 | Tracker WAITING_FOR_STATE/STATE_STALE返回，不发送新Offboard/setpoint | 有期限的状态可信度与已发命令租约；不能依赖无限保持上一速度 |
| 预测断流/目标失锁 | 研究提案拒绝，原FOLLOW原恢复逻辑保留 | 在有效holding窗内提交独立已验收减速/安全高度曲线，过期进入明确PX4模式 |
| 已接纳曲线/控制命令到期 | 研究receiver拒绝或撤销；无真实MINCO active | 有限截止时间、连续制动距离、海面裕度和接管失败闭锁 |
| mission/clock/boot改变 | 身份不符拒绝，旧批准作废 | 中止旧租约并重建可信状态，禁止跨epoch沿用曲线 |
| Offboard异常 | ULog记录本轮正常Offboard；未在MINCO分支故障注入 | 在独立模式切换试验验证实际PX4模式与可用导航条件 |

实际COM_OF_LOSS_T=1.0s、COM_OBL_RC_ACT=0（Position）。这说明停止消息流后的PX4配置，不证明ROS状态陈旧时导航仍足以安全进入Position，也不能代替有限安全回退的响应试验。本轮不更改原Tracker失效逻辑。

## 资格表与后续顺序

| 条件 | 状态 |
|---|---|
| 本地完整约束一致/真实boot时钟身份/125ms门槛 | 已有实现、单测及真实拒绝ACK证据 |
| Tracker唯一PX4控制发布者 | 本轮图记录核验 |
| 独立holding误差界与有限窗口 | 不具备 |
| 从原FOLLOW进入MINCO的真实bridge响应 | 未执行 |
| 状态失效、命令过期、Offboard异常有限回退 | 未获独立验证 |
| 连续时间FOV/真实误差鲁棒包络 | 不具备；目前是名义完整目标采样验收 |

下一阶段先独立标定响应/姿态/安装误差并设计带期限的制动和模式切换协议；通过受控低速bridge试验后才能进入H，再逐级I/J。当前只具备继续影子算法研究的条件，不能宣称公平实际闭环优于原FOLLOW。

关联：[快速算法](p43_fast_visibility_minco.md)、[在线验证](p43_online_follow_validation.md)、[安全资格](p43_safety_qualification.md)。
