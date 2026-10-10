# P4.4 低延迟MINCO影子验证

本轮基点 `c4f34cf56921`，证据根 `data/experiments/20261010_p44_follow_optimization/`。所有实际飞行仍由原FOLLOW控制，Tracker是唯一PX4指令发布者。新MINCO仅研究提案，没有SafetyApproval、holding、首次bridge或实际接管资格；没有开启follow_minco_authorized。旧仓库只读，baseline、模型、原跟随/截击/偏航代码未改。

## 度量口径

每轮独立SITL、独立DDS domain/Gazebo partition、有界watchdog、每轮源码快照/SHA、原配置复制至新实验目录；退出仅清理PID/PGID/start身份匹配的本轮进程。A原FOLLOW、B本轮P43 H1.2、C P44/D H1.2、D P44/D H1.6、E P44/F动态回退、F重复及受控负载。历史60.80ms只作背景，收益使用本轮B对照。

全部完成周期与合法发布分别统计；失败、发布跨deadline和接收拒绝均保留。发布前TTL从实际提案时间、原导航/姿态及实际重验后预测epoch复算，不把旧预测final日志期限当成当前发布TTL。Tracker到达年龄用真实FollowAck.stamp，即接收回调context.now，匹配plan_id及本次原始请求/重验预测，不用离线monitor receipt冒充到达时间。

`tracker_arrival.after_compute_ttl`是在到达余量上减去单调钟接收处理时长的估计值，不是另采集的回调结束ROS时间；RTF/时钟跳变时须谨慎。精确到达年龄与估计处理后余量分开，合法发布P95下降不证明接收时一定有效。125ms门槛、mission/clock/boot/本地指纹及最终门禁均保留。

## 执行资格

拒绝ACK只能证明真实收到和门禁工作，不能证明可安全执行。holding误差界、实际首桥响应、状态失效时有限制动/模式转换仍未建立；名义推力上限18没有物理认证，采样FOV不是连续鲁棒可见性保证。红球仍仅验证仿真感知接口。默认维持原FOLLOW和H1.2，可继续影子研究，不能接管。
