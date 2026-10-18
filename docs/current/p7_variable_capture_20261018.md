# P7 三种变速模式捕获验证

后续独立直线回归的两次回退已另行分析并修复确定性的后继误拒绝：同配置两次 25 秒复测均实际 MINCO 100%、零回退，见 [FOLLOW 连续性报告](p7_follow_continuity_20261018.md)。下文保留捕获阶段原始源码、负结果与资格边界。

本轮在原 4 m/s 八字捕获和三种变速 FOLLOW 已验证的基础上，按用户授权执行变速八字、S、Z 的完整 X→实际 MINCO FOLLOW→Y→IBVS/PNG→半径捕获→暂停。失败先保留并归因，修复后再有界复测；单智能体执行，不推送远程。

## 验收口径

保持原 2–4 m/s 变速规律、0.15 m 升沉、0.50 m 三维接触半径、125 ms 在线输入有效期、50 ms 原生导航 bracket、原锁定/动力学/海面门限。真值只用于评价及独立离线分析。达到半径后暂停世界，不需要碰撞事件。

每轮检查实际 MINCO 接管后 Y 的控制权切换、唯一 Tracker/BCTRA、纯速度接口、旧 FOLLOW/截击权限撤销、实际 PX4 运动和检测可见性；成功后核对世界暂停、固定仿真时钟、固定目标位置及零目标速度。重复 ECS 当前位姿属于单独补充审计，不冒充世界暂停证明。检测有效不表示目标中心或整目标始终在帧内。

## S 首轮失败与修复

[S 首轮](../../data/experiments/20261018_p7_variable_capture_s_01/)在 Y 后 10.8699 s、0.479015 m 达到原半径并暂停，但 `healthy=false`。捕获前约 0.1 s 目标在相机近场消失，制动/爬升保护正确触发；`FAR_GUIDANCE` 的 P7 分支先保持纯速度，下一周期 `SAFE_RECOVERY` 却使用旧 `command_to_setpoint` 默认位置模式，留下 1 条有限位置/加速度字段。这是控制接口验收失败，不改写为通过。

修复仅让 P7 空中搜索/恢复分支继续输出同一受保护的制动/爬升速度，位置和加速度字段为 NaN，Offboard 选择速度模式。非 P7 行为保留。没有调整 PNG、相机/视觉、KF、运动参数、捕获半径、125 ms 时效或锁定门限。3 个 P7 恢复场景先失败，3 个非 P7 对照先通过；修复后 6 个场景全通过。完整原始时序、独立升沉诊断中未采用的候选方案和 SHA256 见[归因目录](../../data/experiments/20261018_p7_variable_capture_s_audit_01/)；恢复接口阶段提交为 `f2f4a20`。

完整测试首轮记录了格式失败，修正后重新运行：**2147 passed、2 skipped、3 warnings**。两项跳过与弃用/日志配置警告保留在日志中。项目范围 flake8、shell 语法、两包构建与 diff 检查通过；日志见[交付目录](../../data/experiments/20261018_p7_variable_capture_delivery_01/verification/)。

修复后[直线 F2 回归](../../data/experiments/20261018_p7_variable_capture_f2_01/)安全完成且锁定/可见 100%，但 MINCO 接管比例仅 54.8019%，最长连续 8.1002 s：一次 `TARGET_STATE_STALE`、一次 `LEASE_EXPIRED_NO_SUCCESSOR`，故 `f3_eligible=false`。没有把安全完成混作持续接管资格，也未运行原 4 m/s F3。该窗口未进入本次修改的恢复分支，底层输入交付与后继供应中断仍未宣称修复；完整时序另存[连续性边界](../../data/experiments/20261018_p7_variable_capture_delivery_01/f2_continuity_limit.json)。后续变速捕获使用既有 F2 场景入口，各轮仍须通过实际 MINCO/Y 门禁；这不授予 F3 资格。

## 实际结果

三轮最终结果均绑定源码指纹 `bed597c09761a2feadc9848a81f5d773dead5390cd38dcd98419cc3b68ca519d`，`healthy=true`，均因 `CAPTURE_RADIUS_REACHED` 暂停。耗时为评价器从接受 Y 起计算；距离为其同步/分段相交统计，不与离散采样或插值点混称。

| 模式 | Y 后捕获耗时 | 判定距离 | 结果与证据 |
|---|---:|---:|---|
| 变速八字 | 13.4815 s | 0.491775 m | [原始 02](../../data/experiments/20261018_p7_variable_capture_eight_02/)、[独立曲线](../../data/experiments/20261018_p7_variable_capture_eight_analysis_02/capture.png) |
| 变速 S | 18.9309 s | 0.495210 m | [修复后 02](../../data/experiments/20261018_p7_variable_capture_s_02/)、[独立曲线](../../data/experiments/20261018_p7_variable_capture_s_analysis_02/capture.png) |
| 变速 Z | 10.7411 s | 0.499527 m | [原始 01](../../data/experiments/20261018_p7_variable_capture_z_01/)、[独立曲线](../../data/experiments/20261018_p7_variable_capture_z_analysis_01/capture.png) |

独立在物理结果时刻，以原始导航/目标戳插值复算的距离分别为 **0.487611 / 0.491129 / 0.484784 m**，均支持已进入原半径；导航 bracket 为 16–24 ms，目标在结果时刻有原始同戳样本，不做 extrapolation、不拟合时间偏移。详细 bracket、原始 SHA256 及分窗指标见[独立汇总](../../data/experiments/20261018_p7_variable_capture_delivery_01/final_analysis/validation.json)，并保留[三模式对比图](../../data/experiments/20261018_p7_variable_capture_delivery_01/capture_comparison.png)。所有离线曲线均止于物理结果时刻，结果后的导航停更不作为飞行失效。

| 模式 | 实际水平速度峰值 | 实际水平加速度峰值 | 最低高度 | 暂停统计覆盖 |
|---|---:|---:|---:|---:|
| 变速八字 | 4.450 m/s | 1.653 m/s² | 0.349 m | 50 样本 / 4.992 s |
| 变速 S | 4.818 m/s | 1.133 m/s² | 0.359 m | 46 样本 / 4.579 s |
| 变速 Z | 4.615 m/s | 0.649 m/s² | 0.371 m | 47 样本 / 4.687 s |

三轮从首次实际 MINCO 到首次 P7 指令的观测窗口，实际 MINCO 分别为 6.881 / 6.342 / 6.403 s、均 100%；这是各轮接近前局部窗口，不替代上述固定 25 s F2 连续性结果。Y 后均为唯一 Tracker、单 BCTRA、纯速度接口，无旧 FOLLOW ACTIVE/旧截击 proposal。三轮控制诊断中的 `visible` 比例均 100%，不等同于每条 RGB-D 位置观测均有效；S 的 P7 制导比例为 99.736%，捕获前有一周期合法 `P7_BRAKING`，没有把它隐藏为持续制导。

三轮 `freeze.world_verified=true`：暂停后的时钟和目标位置跨度均为 0，目标速度均为零；未取得重复暂停 ECS 当前位姿，补充 `native_pose_verified=false`。捕获窗口内垂直图像误差峰值分别约 1.640 / 3.528 / 1.519，末端检测有效不等于球心或整球始终位于图像内。结果仅限当前红球仿真接口与各一轮最终复测，不宣称无标记目标能力或总体捕获率。

## 原件、清理与复现

首次变速八字 01 通过（13.1649 s / 0.481009 m）；因随后修改共享恢复映射，最终版本重新运行 02，首轮未覆盖或删除。S 首轮的半径成功与接口失败也分别保留，没有改写原始 `healthy=false`。实验及交付目录均位于忽略的 `data/experiments/`，冻结基线与视频未改动。

最终三轮按仓库约定选入[新的轻量基线](../../data/baselines/20261018_p7_variable_capture/README.md)，仅复制完整配置、原始 summary/终态回执及来源/SHA256 清单。原始 JSONL、完整源码快照与大日志仍留在忽略的实验目录，不把此轻量包声称为它们的 Git 备份。

沿用[运行入口](p7_capture.md)的 `--guidance d --capture`；`--motion` 取 `variable_speed_eight`、`variable_speed_s`、`variable_speed_z` 中的一个，每轮选不存在的新目录。单轮制导最多 60 s，外层会话有界；失败先检查本轮 `summary.json`、`bounded_stop.json`、原生输入和控制时序，再做有证据的定向修复。不能因为本轮三模式捕获通过而跳过 `require_f2` 对原 4 m/s F3 的独立资格检查。
