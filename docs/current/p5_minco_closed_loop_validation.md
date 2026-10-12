# P5 名义 MINCO 实际闭环验证

执行前预定状态（实际结果见下文）：H1.6 离线限定门禁通过，实际执行尚未验证。旧 P4.7 全时域门禁仍失败。
本轮唯一主配置为 P47/C、qva、refinement=false、adaptive、H1.6；P44/D 后备。

执行前预定判据：一次隔离 F2，原 FOLLOW 健康稳定 5s 后启动研究 Planner，
从真实 MINCO_FOLLOW 接管起观察约25s。10s 内无真实接管则停止；不自动重试。
导航/预测原始输入125ms、新提案完整验收及450ms lease保持。
高度偏离-5m超过1m、失去Offboard、导航/预测失效、速度>8.5m/s、姿态角>0.7rad、
持续Tracker callback>50ms达0.5s或连续3次Planner完整周期>125ms、无合法FOLLOW路径均停止。
VehicleStatus监视按PX4稳态500ms发布周期设置750ms丢失界限，不改变导航/预测125ms门槛。

只有F2正常完成且无底层安全中止/权限冲突、记录完整、诊断有效覆盖≥95%、
观察期实际MINCO占比≥80%、最长连续MINCO≥5s，才允许一次F3。
F3同配置，观察60s以覆盖20–60s稳态窗。若F2失败或收益门槛不满足，本轮不执行F3。

运行入口（本轮不可重复执行）：

```bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
cd /home/qin/data/uav_usv_mpc
/usr/bin/python3 scripts/p5_minco_sitl.py \
  data/experiments/20261012_p5_closed_loop/f2_h16_01 \
  --qualification docs/current/audit/p5_20261012/release.json
```

`attempt_f2.json` / `attempt_f3.json` 用排他创建阻止本轮自动重试。授权由原隔离harness真实签发，
Planner和只读健康监视均有独立PID/start/pgid记录，清理INT→TERM→KILL每步验证进程身份。
底层安全中止不能被外层观察期限覆盖为成功；未知或缺失证据不开放F3。

历史原FOLLOW仅作为非严格匹配参考。现有原始数据、PX4、BCTRA、控制限值和捕获半径不改；
没有通过放宽lease或时效增加ACTIVE占比。实际结果与测试证据见下文。

## 本轮实际结果：F2_TOOLING_ABORT_BEFORE_MINCO

**实际执行一次F2，未执行F3，未重试。P5持续MINCO闭环尚未验证。**
本机目录：`data/experiments/20261012_p5_closed_loop/f2_h16_01/`（约40.7MB完整日志、配置、ULog、
运行时源码快照）；完整目录校验入口 `P5_DELIVERY_SHA256SUMS`。
轻量审计：[f2_outcome.json](audit/p5_20261012/f2_outcome.json)。

隔离harness真实签发授权，原FOLLOW正常起飞。新只读健康记录器收到PX4四元数后，
因NumPy `float32`不能直接序列化JSON而退出。外层工具当时只在自己的FOLLOW标志成立后检查健康；
记录器退出使这个标志始终未成立，因而没有自动及时停止。任务监督发现后写入
`health_monitor_failure.json`并触发STOP，原harness完成日志收尾及所有自有进程清理。
这是本轮新增实验工具的缺陷，**不是P47算法失败证据，也没有P47真实执行成功证据**。

| 必需指标 | 实际结果 |
|---|---|
| P47 Planner实际启动 | 否；仅之前只读实例化预检加载成功 |
| 真实执行授权 | harness签发；研究Planner未启用，condition.minco_authorized=false |
| FollowTrajectory / ACCEPTED / ACTIVE | 0 / 0 / 0 |
| MINCO实际控制占比 / 最长连续控制 | 0% / 0s |
| MINCO观察25s窗口 | 未开始 |
| lease到期数 | 0；没有计划，不能解释为回退改善 |
| 接替间隔、MINCO参考高度、参考跟踪误差 | 不可用 |
| Planner完整周期P95、发布/Tracker接收余量 | 不可用 |
| 控制权切换速度跳变 | 未发生MINCO切换 |
| 唯一PX4发布者 | 84个完整发现图样本均为原Tracker，冲突0 |
| 清理 | 原harness及新增进程survivors均空 |
| F3门禁 | 失败，不执行 |

原FOLLOW共记录约67.95s，仅作此失败轮的背景诊断。原FOLLOW20–60s同窗：
水平RMSE 0.062657m，高度RMSE 0.007718m，整球标记安全比例100%，最小垂直裕度0.316621rad。
全FOLLOW（含过渡）水平RMSE 4.923451m、整球标记安全比例98.53%，不能以稳态数字掩盖过渡。
这里全部是**原FOLLOW**，场景速度约2m/s；离线F2预测速度约4m/s，不能作为P47对原FOLLOW的匹配收益比较。
红球整球几何也不能外推无标记真实USV验证。

原始`p5_run_summary.json`的reason为`OUTER_WATCHDOG`，这是当时外层默认标签，
并非真实运行达到295s看门狗期限；总session约120.94s。原始记录按字节保留，
`failure_attribution.json`解释实际中止原因，不覆写原标签。

修复：四元数逐项转原生float；外层在FOLLOW前也检查监视进程退出、首样本超时和快照停滞；
保留真实ACCEPTED/ACTIVE与PX4命令证据作为健康执行门禁。针对实际生成的
`px4_msgs/VehicleAttitude` float32数组、监视器死亡及停滞均完成RED→GREEN测试。
没有改动原Tracker、BCTRA、PX4、lease、125ms时效或旧P47门禁。修复后未重新SITL。

## 最终问题逐项回答

| 问题 | 结论 |
|---|---|
| P4.7是否封版 | H1.6离线候选封版；非全时域、非生产资格 |
| P5候选 | P47/C + qva + false + adaptive + H1.6 |
| P44/P46如何保留 | P44主后备和显式切回；P46历史对照、复现和测试保留 |
| 精简哪些入口 | 只将新P5选择列表缩为P47/P44；算法文件删除0 |
| Git整理是否完成 | 授权范围内非破坏性整理、标签/本地分支/跟踪引用完成；main因PR保留 |
| 新MINCO是否持续控制PX4 | 没有，本轮0 ACTIVE |
| 高度漂移是否解决 | 离线参考通过；实际P47尚无证据 |
| lease回退是否减少 | 无法判断，没有MINCO计划 |
| F2/F3是否超过原FOLLOW | 无证据；F2工具中止，F3未执行 |
| 何时开始预测/FOV创新 | 另行授权的实际闭环验证健康持续通过后 |

完整测试和来源入口：[verification.json](audit/p5_20261012/verification.json)。
本轮唯一F2次数已消耗，后续真实复验需要新的明确授权；本次不自动继续。

最终修复后全量pytest：**1612 passed、1 skipped、3 warnings（16.27s）**；新增测试44项通过。四包构建、项目flake8、5个shell语法、ROS消息/launch、包前缀及diff/cached diff检查通过。未提交、未推送。
