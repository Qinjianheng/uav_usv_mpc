# P4.2 本轮闭环与影子验证（2026-10-09）

实际执行始终为原 FOLLOW。MINCO 的真实 ACCEPTED/ACTIVE 均为零，holding、真实 bridge 和状态失效 contingency 未通过，按用户门禁未实施接管实验 C/D/E。
证据根目录：`data/experiments/20261009_p41_rolling_follow/`。没有自动提交、推送或暂存。

## 实验与来源

每轮独立启动 Gazebo/PX4/DDS/ROS，以 X 开始，不发 Y。F2 使用 2m/s 直线、F3 使用 4m/s 八字，升沉均沿用 0.15m。完整配置在各轮 config/，场景参数未为改善结果而调整。
任务拥有的 PID/PGID/start、DDS domain、GZ partition、watchdog、ROS/Gazebo 日志和 ULog 均独立保存；启动前 freeze_sources 复制源码/配置并生成 SHA256。source_snapshot.json 与 working_tree.patch 是对应现场版本，不以最终源码倒推初期实验。
其中 f2_original_1 启动超时、f2_original_2 在很晚就绪后被 watchdog 截断，均保留并排除有效重复；retry_1/2 为替代的独立轮次。失败时主机 load≈20、IO wait≈29%，有传感器超时、world 服务延迟；没有停止用户应用。
早期四轮 shadow 的 completion.whole_cycle_time 截止于原 JSON 发布，漏计新增重验和 proposal；已补最终发布前/后完整周期门禁及 follow_publication_final。final_1/2 是修复后的独立 F3 现场验证。final_1 后补充纯 API 的 NaN generated epoch 守卫，final_2 冻结该最终代码；正常 ROS 预测入口原已拒绝这种输入。
两次 original 和两次 shadow 分别构成 F2/F3 对照，额外 final 轮用于工程修复确认；没有宣称做了 MINCO 实际闭环重复。

## 实际运动结果

稳定窗口以首次 FOLLOW 后 20–60s 计算。速度列为 UAV 与艇体的速度差，转弯时不等于旋转后方参考的速度跟踪误差。全 FOLLOW、初始状态/负载、距离、相对速度、高度、FOV、命令差分等均保留在各轮 analysis_extended.json。原生 usv_target pose 已是渲染球心，离线评价已纠正重复施加 -0.42m 偏移的旧工具问题；在线目标基准仍保留原 offset，未改模型/控制。已用光轴中心独立案例回归，未改写旧 P4 报告或原始数据。

| 轮次 | 实际控制器 | 水平 RMSE m | 距离 RMSE m | 速度误差 m/s | 高度 RMSE m | 整球安全比例 | 真实拒绝 ACK |
|---|---|---:|---:|---:|---:|---:|---:|
| f2_original_retry_1 | 原 FOLLOW | 0.0593 | 0.0542 | 0.0238 | 0.0078 | 1.0000 | 0 |
| f2_original_retry_2 | 原 FOLLOW | 0.0682 | 0.0668 | 0.0219 | 0.0077 | 1.0000 | 0 |
| f2_shadow_1 | 原 FOLLOW | 0.0907 | 0.0891 | 0.0219 | 0.0081 | 1.0000 | 222 |
| f2_shadow_2 | 原 FOLLOW | 0.4151 | 0.4136 | 0.3107 | 0.0097 | 0.9950 | 106 |
| f3_original_1 | 原 FOLLOW | 1.4916 | 1.1796 | 1.4622 | 0.0159 | 0.9589 | 0 |
| f3_original_2 | 原 FOLLOW | 0.6875 | 0.1103 | 1.0454 | 0.0073 | 1.0000 | 0 |
| f3_shadow_1 | 原 FOLLOW | 0.7101 | 0.1036 | 1.0603 | 0.0084 | 1.0000 | 141 |
| f3_shadow_2 | 原 FOLLOW | 0.7037 | 0.1151 | 1.0548 | 0.0088 | 1.0000 | 138 |
| f3_shadow_final_1 | 原 FOLLOW | 0.7267 | 0.1045 | 1.0776 | 0.0074 | 1.0000 | 162 |
| f3_shadow_final_2 | 原 FOLLOW | 0.7174 | 0.1312 | 1.0725 | 0.0098 | 1.0000 | 155 |

F3 original 的较差轮次不删除；其失锁/原生渲染 FOV 下降、速度变化和现场服务异常说明独立重复存在场景/负载偏差。样本数少且未严格随机交叉负载，不能把 shadow 条件下的 RMSE 变化归因于新规划器优化。
此处真值只用于离线评价：在线 predictor/planner 接收 tracking 状态；native Gazebo pose/clock 仅进入 recorder/analysis。整球指标使用原生同刻 UAV/目标 pose 与实际安装外参；只验证红球接口，不代表无标记非合作艇感知、遮挡处理或实机可部署。

## 实际动态与名义约束

| 轮次 | 实际水平速度 max m/s | 实际水平加速度 max m/s² | 差分 jerk P99 m/s³ | 差分 jerk max m/s³ |
|---|---:|---:|---:|---:|
| f2_original_retry_1 | 2.0314 | 0.0898 | 0.9679 | 1.3536 |
| f2_original_retry_2 | 2.0393 | 0.0875 | 0.9491 | 1.1993 |
| f2_shadow_1 | 2.0277 | 0.1010 | 0.9866 | 1.5357 |
| f2_shadow_2 | 3.4733 | 2.8980 | 5.4735 | 7.0908 |
| f3_original_1 | 6.0677 | 2.9841 | 7.1052 | 8.1636 |
| f3_original_2 | 4.2372 | 1.8558 | 1.5539 | 1.8241 |
| f3_shadow_1 | 4.2213 | 1.8856 | 1.6900 | 3.0931 |
| f3_shadow_2 | 4.1992 | 1.8511 | 1.3724 | 2.0961 |
| f3_shadow_final_1 | 4.2063 | 1.8707 | 1.3800 | 1.7189 |
| f3_shadow_final_2 | 4.3414 | 1.8829 | 1.6442 | 2.4318 |

jerk 是原导航加速度按实际有效相邻间隔的差分估计，受估计噪声影响。部分实际原 FOLLOW 轮次估计峰值超过研究轨迹 6m/s³，不能用 MINCO 名义曲线最大 jerk 来证明 PX4 实际响应满足该限制。没有实际 MINCO 执行数据可建立其位置误差、姿态滞后或超调包络，故 holding 不成立。

## 时序、ACK 与版本竞争

图像/导航/预测 acquisition、generated、proposal、ACK 使用各自保留的 ROS/system epoch；计算耗时使用 perf_counter/monotonic。UavState 保留 native sample；Gazebo native clock 单独记录，未伪造 timestamp 或在线使用真值对齐。
各轮 prediction_latency.json/analysis_extended.json 包含图像→KF epoch→BCTRA generated→研究进程收到，以及 observation processed/published 分段。完成、proposal、ACK、未来 execution_start 分别记录；因没有 ACTIVE，不存在实际交接执行时刻，不能用计划 start 冒充执行。

| 影子轮次 | completion 周期 P95 ms（旧口径） | 最终 proposal 周期 P95 ms（新口径） | 新版本重验 通过/次数 | proposal→ACK P95 ms |
|---|---:|---:|---:|---:|
| f2_shadow_1 | 54.74 | 未测 | 118/118 | 0.95 |
| f2_shadow_2 | 64.03 | 未测 | 72/72 | 1.17 |
| f3_shadow_1 | 58.74 | 未测 | 77/77 | 0.98 |
| f3_shadow_2 | 59.72 | 未测 | 65/65 | 1.17 |
| f3_shadow_final_1 | 60.56 | 73.16 | 93/93 | 0.99 |
| f3_shadow_final_2 | 57.76 | 66.96 | 86/86 | 1.09 |

实际 ACK 状态合计：`{'REJECTED': 924}`；原因合计（同一 ACK 可含多个原因）：`{'HOLDING_DOES_NOT_COVER': 924, 'PREDICTION_CHANGED': 196, 'NO_QUALIFIED_HOLDING_MODEL': 924, 'INITIAL_BRIDGE_UNAVAILABLE': 924}`。
共享时间代通过实际 Tracker 只读心跳建立；本轮 CLOCK_GENERATION_CHANGED 拒绝为零。独立 holding/bridge 缺失继续拒绝；新预测重验后到 Tracker 再更新的竞争仍由 PREDICTION_CHANGED 拒绝。没有删门禁来增加 ACK。
所有有效图样本的 PX4 三类输入发布者均为单一 trajectory_tracker_node；没有新增 Offboard 发布者。实际 active/pending 始终为零。MINCO 连续执行时长、MINCO→FOLLOW 回退次数均为“未执行”，不记成已验证的零失败。

## 预测时域的实测依据

以下预测误差仅在新现场 shadow completion 的原预测 source+h 与独立理想目标时间序列之间离线复算，拒绝外推/跨 >0.15s 缺口。各时域样本数不同，含初始运动；不是严格同人口消融或未来误差保证。

| 轮次 | 0.2s 水平 RMSE m | 0.8s | 1.2s | 1.6s | 2.4s |
|---|---:|---:|---:|---:|---:|
| f2_shadow_1 | 0.0558 | 0.0665 | 0.0834 | 0.1171 | 0.2388 |
| f2_shadow_2 | 0.1024 | 0.1239 | 0.1593 | 0.2256 | 0.4468 |
| f3_shadow_1 | 0.3240 | 0.5177 | 0.6883 | 0.8780 | 1.3684 |
| f3_shadow_2 | 0.1673 | 0.3674 | 0.5174 | 0.6794 | 1.1268 |
| f3_shadow_final_1 | 0.2044 | 0.4119 | 0.5672 | 0.7427 | 1.2332 |
| f3_shadow_final_2 | 0.1540 | 0.3724 | 0.5290 | 0.7010 | 1.1911 |

这些数据支持避免盲目延长预测与规划。滚动短轨迹可利用最新输入更新并限制过时预测影响，但仅降低名义预测依赖，不能凭此证明闭环更优或续原始 TTL。1.2s 为本轮保守研究配置；1.6s 的理想滚动收益值得后续独立验证。

## 故障与检查

f2_shadow_2 在 X 后 65s 向本轮研究 planner 子进程发送 SIGINT；记录自身 PID/start/PGID。停止后原 FOLLOW 继续发布 reference；间隔统计见 planner_exit。此实验验证影子退出不夺控制权，不能称为 MINCO 安全回退。
预测断流/失锁/输入过期/轨迹过期、mission/clock 变化、错误 ACK、重启/旧 boot 重放、OFFBOARD 不满足和 holding 到期由纯协议/实际回调测试覆盖；没有在尚未获准的 MINCO 控制中注入故障。无真实桥接、连续 MINCO、PNG/IBVS 新控制或实机测试。
所有会话 watchdog_end 记录监视器退出码、launcher 退出码及仍存活的自有组。12 轮均无仍存活自有组；11 轮 monitor 退出码为0，final_2 在 X 后85.17s收尾瞬间发生 take_message RuntimeError，退出码1。稳定窗口数据保留，不将它称为干净收尾。最后改为显式停止 spin 后再关闭 ROS context，并只在已收到停止信号时记录退出异常；主动运行时 RuntimeError 仍上抛。独立短时退出 probe 结果见 monitor_shutdown_probes.json；未宣称该补强已在新完整飞行中复验，也未断言已独立复现原生异常根因。没有广泛 kill 或删除历史数据。PX4 参数备份原路径和复制清单见 parameter_backup_sources.json。
本轮 fresh reviewer 只返回了完整周期门禁遗漏这一具体发现后因账户额度中断；该问题有 RED/GREEN 回归及最终现场复验。没有声称完整独立审查完成，也没有再次派发消耗额度。

最终构建/algorithm/related/full/flake8/shell/launch/interface/diff/保护审计结果见 verification_summary.json。原截获功能做自动回归，本轮没有新的 Y 截击飞行。397 个保护文件和原控制方法 AST 单独核验；旧数据 SHA256 重新校验。

## 十三个结论

1. 4.65m 包含初始追赶、侧后方参考位移和动态滞后，不能全解释为长期发散。
2. 动态终端加速度在冻结 F3 上有收益，F2 无全面收益；未放宽动态限制。
3. 新策略在该组约 101s 理想 episode 内保持跟随并降低误差，转弯仍有周期误差；未证明渐近收敛或任意目标长期稳定。
4. 首次 bridge 未真实验证。
5. holding 没有独立真实误差依据；纯误差球数学函数不发许可。
6. receiver-owned boot/clock 协议已实测消除独立 generation 比较错误。
7. A/B 已实现；新的传输竞争继续拒绝，C 未实现。
8. Tracker 仅作真实拒绝 ACK，没有真正接纳新轨迹。
9. 未完成实际 MINCO 闭环。
10. 实际八字误差见逐轮表，控制器为原 FOLLOW。
11. 不能宣称新 MINCO 真正优于原 FOLLOW。
12. 名义动力学和整球几何可测试；真实连续误差界、bridge 与有界安全恢复不足。
13. 实机前须解决误差 tube、可信预测界、接收端持有、首次模式桥接、状态失效 contingency、时序预算及 DDS 身份/复位乱序问题。

到此停止。下一阶段建议先做接收端误差与控制响应资格研究，资格通过后才实现/验证低速首次桥接和 Tracker 实际接纳。

## 修改文件

本轮功能文件共 32 项；实验产物另见证据根目录 SHA256SUMS 和 parameter_backup_sources.json。无暂存、提交或推送。

- `docs/current/README.md`（modified）
- `docs/current/p41_follow_tracking_optimization.md`（new）
- `docs/current/p42_closed_loop_validation.md`（new）
- `docs/current/p42_follow_safety_qualification.md`（new）
- `scripts/experiment_source_snapshot.py`（new）
- `scripts/p41_follow_replay.py`（new）
- `scripts/p4_follow_analysis.py`（modified）
- `scripts/p4_shadow_monitor.py`（modified）
- `scripts/p4_sitl_session.py`（modified）
- `scripts/test_research_planners.sh`（modified）
- `src/uav_control/test/test_follow_epoch.py`（new）
- `src/uav_control/test/test_follow_revalidation.py`（new）
- `src/uav_control/test/test_follow_safety_bounds.py`（new）
- `src/uav_control/test/test_p4_receiver_node.py`（modified）
- `src/uav_control/test/test_p4_session_safety.py`（modified）
- `src/uav_control/test/test_tracking_progress.py`（new）
- `src/uav_control/uav_control/control/trajectory_tracker_node.py`（modified）
- `src/uav_control/uav_control/controllers/follow_research_shadow_node.py`（modified）
- `src/uav_control/uav_control/controllers/follow_transport.py`（modified）
- `src/uav_control/uav_control/controllers/p4_follow_planner_node.py`（modified）
- `src/uav_control/uav_control/controllers/progress_follow_solver.py`（modified）
- `src/uav_control/uav_control/controllers/short_follow_solver.py`（modified）
- `src/uav_control/uav_control/guidance/follow_contract.py`（modified）
- `src/uav_control/uav_control/guidance/follow_epoch.py`（new）
- `src/uav_control/uav_control/guidance/follow_reference.py`（modified）
- `src/uav_control/uav_control/guidance/follow_revalidation.py`（new）
- `src/uav_control/uav_control/guidance/follow_safety_bounds.py`（new）
- `src/uav_usv_bringup/config/p41_follow_research.yaml`（new）
- `src/uav_usv_interfaces/CMakeLists.txt`（modified）
- `src/uav_usv_interfaces/msg/FollowPlanAck.msg`（modified）
- `src/uav_usv_interfaces/msg/FollowReceiverState.msg`（new）
- `src/uav_usv_interfaces/msg/FollowTrajectory.msg`（modified）
