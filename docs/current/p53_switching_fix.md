# P5.3 最小供应修复与复验

生产修改集中在 `controllers/p4_follow_planner_node.py`：

- 在显式执行/准入模式下，先完成原始freshness、整轮截止、预测重验、消息转换、身份和实际wire发布；
  随后处理大型研究JSON与日志。普通shadow路径保持原先顺序。
- 求解可行性不再代表本轮交付成功。原有单parent、一次提前重试依据真实wire结果；
  未发布时保持重试资格，合法wire之后研究marker晚到不会触发多余重试。
- 日志同时记录wire完成时刻/原输入余量/关键周期耗时，以及之后研究marker原因，避免混淆两种人口。

cycle79记录中发布阶段14.035ms、JSON13.253ms。确定性测试注入原13.253ms开销：
修复前候选在关键wire检查前过期，修复后先依法发布；已过期输入仍拒绝。
另一RED覆盖“求解valid而wire失败”的重试标志；修复后相关原P4/P5.1/P5.2回归通过。
所有指标保留125ms/450ms/50ms，不使用旧目标、授权缓存或时间戳改写。
完整Receiver独立验收与验收完成后的实际时钟检查保留。

P5.2的accepted-start唤醒、近邻1ms保护、等待足够新鲜预算的唯一提前重试不重复实现。
新增唤醒测试覆盖回拨、跳过lease、50ms调度延迟和parent失效取消pending。
真实第三轮证明唤醒延迟改善，但仍有两次合法回退；不以新增测试数量代替实际稳定性。

新增工具：p53_f2_analysis（只读归因/25s同刻评价）、p53_event_replay（记录事件/ACK回放）、
p53_saved_request_replay（原始边界与实测墙钟复算）、p53_minco_sitl（新命名空间一次F2/条件F3）。
测试夹具从第二轮原始证据精选，保存来源SHA；确定性gzip约111KiB，未提交大体量实验或视频。

残留：worker高负载连续超时未由本轮日志优先级修改证明消除；Receiver真实耗时仍可能耗尽余量。
记录prefix值的协议回放不替代新候选完整几何验收，模拟延迟不冒充真实CPU延迟。
原始失败/初次格式检查输出保留；最终全量、构建、静态检查与独立审查见
[audit/p53_follow_minco/verification.json](audit/p53_follow_minco/verification.json)。

最终离线核验：1756 passed、1 skipped、3 warnings；四包构建、flake8、shell、安装接口、launch、包前缀和Git检查通过。688个当前源码SHA、984个冻结保护文件及9组历史清单已核验。独立审查1项Important已RED→GREEN关闭；没有把离线资格当作实际F2通过。

## 2026-10-13 自主续做的后继预算调度

P53第2次实跑已验证关键wire发布未再被研究JSON挡住，但仍有两次lease回退。cycle63/107分别在原始观测age68.83/70.88ms时派发，实际求解超出有效预算；cycle64/108的准备预算分别25.044/33.548ms，略高于最便宜路径提示，但worker dispatch和公共模型准备消耗余量后两路径均未尝试，浪费唯一early retry。

在真实ACTIVE parent、deadline scheduling、无pending时，提交worker前保留原20ms发布reserve，再要求最便宜路径估计之外有20ms准备/dispatch/公共模型及变化余量。缺口只等待新鲜输入；原factory每次先检查实际parent、pending、身份和start余量，不能延长旧lease或多求解重试。shadow和初始进入保持原行为。

等待期间历史成本提示可能高估，因此仅同mission/generation下严格更新的图像采集戳可以使成本提示按原solver恢复策略衰减一次，最低原warm prior15/12ms。重复timer/旧图像不能衰减，worker进行中不改成本。提示仍非保证；真实求解/完成/发布/接收的所有原硬限保持。独立审查的成本尖峰恢复Important已由真实runner请求计数RED→GREEN关闭。

原FOLLOW单帧失锁观察和Gazebo瞬时pose失败修复见 [飞行效果修复](p53_flight_effect_repair.md)。当前后继余量修复尚待P53第3轮实际验收，不将定向17通过当作连续接管通过。
