# P3.2 FOLLOW 轨迹交接协议（设计与影子 dry-run）

本轮没有控制接管。协议纯函数位于 `guidance/follow_handover.py`，原 Tracker、MissionManager、PX4 发布接口和 ROS 消息均未修改。`ELIGIBLE_DRY_RUN` 只表示显式给定契约通过纯函数检查；研究 JSON 的发布、几何可行或离线测试通过均不构成实际接纳。

## 三种时间契约

1. 原始输入期限：`min(nav+0.125, observation+0.125, source+0.125, prediction_valid_until)`。使用原采集戳及 ROS/system epoch；准备、各级初始化、验收、优化、完成和序列化后的发布均须检查。monotonic 仅计计算时间。不可重写输入戳或增加原 `valid_until`。
2. 预测覆盖：原 BCTRA source epoch 加原预测样点时域。插值须在覆盖内，不能外推。4 s 覆盖不等于 4 s 信任或执行权限。
3. 未来已接纳轨迹的持有期限：需要独立安全依据、实际接收者确认和非续期的绝对期限。当前 FOLLOW 没有这种研究轨迹契约。纯函数中必须显式提供 `holding_valid_until`、`safety_model_id`，且其覆盖整个候选执行区间并不超过预测覆盖；未建立时明确拒绝。字符串和期限本身不是安全模型的证明，未来可信接收端必须核验它们的来源。

当前保留 150 ms execution lead。其作用是投影研究起点；许多实际请求的原预测 TTL 在这个起点前已经到期。因此本轮即使满足当前时刻的研究发布准入，仍不具备未来执行授权。没有通过修改 lead/TTL 解决这个矛盾。

## 数据与权限

候选字段：轨迹 ID、mission ID、clock generation、原 source stamp、实际执行起终点、输入期限、预测覆盖终点、同刻 P/V/A/yaw/yaw-rate、完整验收状态、预测版本及重验收版本、独立持有期限和安全模型 ID。实际接入还需要单独保留 navigation/attitude/observation stamps、完整系数、约束版本、校准版本、取消原因、接收端确认戳和预测误差界。

旧轨迹字段：实际已接纳 ID、任务/时钟代、原有效执行区间、ACCEPTED/ACTIVE 状态，以及从其系数在**候选交接时刻**采样得到的 P/V/A/yaw/yaw-rate 和 sample stamp。旧轨迹未来终点不能通用地充当下一次规划起点。

原 Tracker 内部已有 `active_trajectory`、plan/mission ID、source/valid_until、系数和 admission/replacement 判断；现有 `/control/diagnostic` 可读 plan ID、源年龄、参考位置、命令 V/A、替换标志和 P/V/A 交接残差。`/control/reference` 是实际发出的 setpoint，原 FOLLOW 通常 plan ID 为 0、XYZ position 为 NaN、加速度字段为 0。它不能提供具有有效区间的旧已接纳研究轨迹。现有发布没有统一的 generation/接受确认、完整旧轨迹快照、yaw-rate 交接证明和持有依据。本轮只读录制上述话题，未改变发布。

`follow_problem.future_request` 的测量常加速度外推仍明确标记为 projected boundary；它不是旧轨迹期望状态，不能用于宣称实际交接连续。运行时研究结果将交接 dry-run 标记为 `OLD_TRAJECTORY_UNAVAILABLE`；禁止伪造旧轨迹。

## 生命周期与拒绝

`PROPOSED → VALIDATED → ELIGIBLE → ACCEPTED/REJECTED → ACTIVE → EXPIRED`。纯 `transition` 检查合法边；ACCEPTED 要求接收者确认，影子端默认不能确认。REJECTED/EXPIRED 的同一身份为终态，不允许改标签恢复；新提案需新身份、原始输入校验和新的完整验证。

`dry_run` 拒绝条件：非法/非有限契约或 PVA/yaw 数据、非法任务/时钟、时间顺序错误、试图扩大原始 source 的 125 ms 期限、候选 EXPIRED、非 FOLLOW、任务切换、clock generation 改变、原输入过期、执行起点已错过、预测不覆盖候选全域、候选尚未完整验证、旧轨迹缺失/未授权/不同任务或时钟、旧轨迹在交接时刻无效、旧轨迹采样 epoch 不一致、P/V/A/yaw/yaw-rate 任一不连续、新预测版本未重新验收，以及持有安全契约未建立。

纯函数默认离线容差为 P/V/A 各 `1e-3`、yaw/yaw-rate 各 `1e-3`（SI），不是经过飞行标定的控制验收参数。yaw 使用圆周差。单元测试同时覆盖合法合成旧轨迹、断续、过期、missed start、mission/generation、覆盖、无旧轨迹和禁止影子确认。

## 未来安全持有模型

P4 前必须设计可信 validator：在标定的目标预测误差、姿态/跟踪误差界内检查整段动力学、目标整体视场和海面安全；说明采样验证如何覆盖连续时间；规定预测更新、失锁、时钟重置、任务切换、bounds 违约后的撤销/停止条件。接收者在实际交接时刻重新核对最新预测及旧轨迹，拒绝错过起点；执行只到原持有期限，拒绝候选不得延长旧期限。撤销后的处理须走原 Tracker/MissionManager 的安全策略，由未来独立控制任务实现，本轮没有实施。

离线时间策略只能比较：在当前输入 TTL 内完成计算并发布；为已证明安全的持有契约预留交接与调度时间；在最新预测到达时重新验收。当前没有持有误差模型，因此不能把第二种策略当成已可执行方案。不同 lead、计算/调度延迟的纯测试仍保留原输入期限，缺少旧轨迹或安全依据时 dry-run 失败。

尚未验证：实际控制接收/确认、原 FOLLOW 研究轨迹交接、持有期间的闭环安全、姿态误差界、预测可信界、风/阻力与执行器角速度、连续时间 FOV 证明。本文件不授权 P4。
