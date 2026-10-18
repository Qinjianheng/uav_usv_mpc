# P7 FOLLOW 失效归因与八字捕获修复

本轮对应“原 4 m/s 八字捕获，以及变速八字/S/Z 跟随失败修复”。原失败数据和冻结基线保持原样。各轮保留完整配置、源码快照、原始记录及 SHA256；本文件区分 FOLLOW、Y 后制导与终态暂停。

后续已完成[三种变速模式捕获验证](p7_variable_capture_20261018.md)。下方保留本阶段结果；后续恢复接口修复与直线连续性不足另列，不能沿用本阶段 F2 资格。

## FOLLOW 根因与修复

**变速 S/Z：起飞时静止爬升导致目标逃出深度量程。** P7 未开启已经实现的 `takeoff_visual_tracking`，默认起飞时不给水平目标。按原图像采集时刻，以不超过 50 ms 的原生 Gazebo 位姿 bracket、完整姿态、真实相机外参独立复算：S 的 47 个匹配 `DEPTH_RATIO_LOW` 样本中，46 个球体整体超过 25 m 前向深度；Z 为 69/69。有效观测的中心深度残差 p95 分别为 0.052/0.078 m。S 的一个边界样本未归为纯量程原因，没有隐藏。详见[独立归因](../../data/experiments/20261018_p7_follow_repair_audit_01/README.md)及[深度曲线](../../data/experiments/20261018_p7_follow_repair_audit_01/depth_boundary.png)。

修复只在 P7 Tracker 配置开启既有起飞视觉跟随：仍要求新鲜且锁定的 KF 输入，0.50 m 高度后才开始水平运动，保留高度混合、1.50 m/s² 水平加速度限制及失效拒绝。没有扩大深度范围。

**变速八字：转弯时缺少 FOLLOW 视线角速度前馈，阻碍重锁。** 原始记录在一个输入超时后恢复到新鲜 KF，目标仍可见、位置观测有效，但约 0.21 rad 稳态偏角超过 0.15 rad 重锁门限，居中帧数一直为零。已有同刻 LOS 前馈原来仅用于 `rear_point` 且 MINCO active；P7 使用 `target` 参考，实际未走该分支。

修复将既有 KF/原生导航同刻 LOS 前馈用于 P7 的 FOLLOW，包括正常 MINCO、原 FOLLOW 回退及新鲜观测下的重锁。保留 125 ms 图像/状态有效期、50 ms 原生导航 bracket、原偏航上限和失锁时搜索上限。Y 后立即返回独立的 IBVS 偏航分支，没有叠加 FOLLOW LOS 前馈。

**Z 复测暴露的观察器异步竞态。** 第一轮已经进入 MINCO；一次 RGB 未配对后，新失效预测先于下一控制诊断到达。观察器把新预测与上一周期 MINCO 状态一起判为 `PREDICTION_INPUT_STALE`。原始记录中下一周期预测恢复，Tracker 已依法回退到 FOLLOW。P7 观察器现在只在“失效预测比控制诊断新”时最多等待现有一个 50 ms 控制周期；更新的 MINCO 诊断、超时及任何物理故障仍终止。此调整不授予控制权，也不改变在线 125 ms 时效、租约或接管统计；短暂回退仍按实际记录计入结果。

## 八字捕获的独立问题

第一次原 4 m/s 八字已通过 FOLLOW 并接收 Y；接近约 1 m 时仍有约 0.5–0.7 m/s 横向相对速度，IBVS 饱和，随后失锁并恢复。原始失败及[曲线](../../data/experiments/20261018_p7_follow_repair_figure_eight_analysis_01/capture.png)保留。

原制导积分世界坐标加速度，持续转弯时缺少目标速度变化的前馈；靠近后切换的速度匹配也存在加速目标跟踪滞后。独立恒转弯反例采用 4 m/s、1 m/s²、40 ms 观测年龄及独立的一阶速度/偏航响应，原控制在 20 s 内无法进入 0.50 m。左右转弯测试均先失败。修复仅在显式捕获中，将相邻合法 KF 速度的变化量加入水平加速度积分前馈，并在原加速度、速度和步长限幅之前处理；首帧不施加差分，失效输入清空历史。没有读取真值加速度、未来轨迹或增加偏航前馈。离线反例及输入指纹见[转弯诊断](../../data/experiments/20261018_p7_capture_turn_audit_01/counterexample.json)，离线结果不作为飞行通过证据。

## FOLLOW 实际复测

以下均为第一次实际 MINCO 接管后固定 25 s 窗口，统计包括全部控制者；不是 proposal、ACK 或 shadow 资格。运动参数和原 2–4 m/s 规律未修改。

| 场景 | 实际连续 MINCO | 比例 | 可见/锁定 | 回退 | 轮次 |
|---|---:|---:|---:|---:|---|
| 直线回归 | 24.9501 s | 100% | 100% / 100% | 0 | [F2 01](../../data/experiments/20261018_p7_follow_repair_f2_01/) |
| 变速八字 | 24.9502 s | 100% | 100% / 100% | 0 | [eight 01](../../data/experiments/20261018_p7_follow_repair_variable_eight_01/) |
| 变速 S | 24.9572 s | 100% | 100% / 100% | 0 | [S 01](../../data/experiments/20261018_p7_follow_repair_variable_s_01/) |
| 变速 Z | 24.9708 s | 100% | 100% / 100% | 0 | [Z 02](../../data/experiments/20261018_p7_follow_repair_variable_z_02/) |

完整指标、原生导航曲线、各轮源码指纹及原始 SHA256 见[交付目录](../../data/experiments/20261018_p7_follow_repair_delivery_01/)。首次 Z 的异步停止轮次另行保留，没有替换为通过结果。

## 原 4 m/s 八字捕获

[修复后轮次 02](../../data/experiments/20261018_p7_follow_repair_figure_eight_02/)完成 X→实际 MINCO→Y→IBVS/PNG→接触半径暂停。Y 前实际 MINCO 输出跨度 6.7879 s；Y 后 **18.8201 s** 捕获，评价距离 **0.498785 m**，半径保持 **0.50 m**。独立按原生导航戳和评价真值插值复算的离散最小三维距离为 0.492115 m，支持已进入原半径；与评价器同步/分段相交统计分列，不混用定义。详见[独立指标](../../data/experiments/20261018_p7_follow_repair_figure_eight_analysis_02/metrics.json)及[捕获曲线](../../data/experiments/20261018_p7_follow_repair_figure_eight_analysis_02/capture.png)。

制导及检测可见比例均为 100%，实际水平速度峰值 4.882 m/s、水平加速度峰值 1.646 m/s²、最低高度 0.354 m；Y 指令水平速度步长 0.09981 m/s。一个 BCTRA 主预测器、一个最终 Tracker、纯速度接口，Y 后无旧 FOLLOW ACTIVE 或旧截击 proposal。IBVS 偏航请求峰值 0.518 rad/s；PNG 仍执行原 2 m/s² 加速度及 6.2 m/s 速度请求上限，未把理论值当作实际响应。

暂停后 46 个世界统计样本覆盖 4.5899 s，暂停标志有效、仿真时钟跨度 0、目标位置跨度 0、目标速度为零：`freeze.world_verified=true`。未取得暂停期间重复 ECS 当前位姿，补充 `native_pose_verified=false`，不宣称该补充审计通过。末端检测可见不等于球心或整球始终位于帧内：捕获窗口内垂直归一化图像误差峰值 2.112。原 summary 的 2.647 还包含结果事件后两条有效制导记录，原文件不改写；按物理捕获戳截止的[分窗指标](../../data/experiments/20261018_p7_follow_repair_delivery_01/capture_scoped_metrics.json)单独保存。本轮仍为红球仿真感知接口验证，不外推为无标记 USV 或完整捕获率。

## 验证与保留问题

最终源码通过 **358 项定向测试**、项目范围 flake8、shell 语法、两包 symlink 构建及 diff 检查；`ros2 pkg prefix uav_control` 指向本工作区。捕获代码修改后重新取得[当前源码 F2 03](../../data/experiments/20261018_p7_follow_repair_f2_03/)资格：连续 MINCO 24.9000 s、100%、无回退。变速三轮的 FOLLOW 源码/config 相同；后续源码变化仅为 Y 后显式捕获的局部分支，三种变速捕获本轮未运行、未宣称通过。

重新取得资格前的 F2 02 记录一次定位节点导航与观测同时停止输出：Gazebo 时钟和 DDS 姿态仍更新，Tracker 在原 125 ms 门限下正确终止。此轮失败保留，未以观察器例外掩盖。后续 F2 03 和八字 02 增加仅被动注册的线程栈诊断；捕获前没有再次触发，首次中断根因仍未确认。八字暂停后的预期 `STATE_STALE` 触发过栈记录，明确与飞行中断区分，见交付目录。这个边界不改变本轮捕获与三种 FOLLOW 的实际记录。

原失败、归因脚本、每轮配置/源码/原始 SHA256、验证日志和自审结论均保留。没有改写冻结证据、扩大捕获半径或在线使用真值；阶段提交不推送远程。
