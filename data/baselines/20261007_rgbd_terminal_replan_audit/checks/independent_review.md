# RGB-D 末端执行独立静态审查记录

日期：2026-10-07。工作区：`/home/qin/data/uav_usv_mpc`。

本记录保存已完成的独立审查及实际验证结果。审查期间未编辑生产代码；本次仅保存记录，没有新增仿真或扩大审查范围。

## 结论与范围

最终复查结论：已提出的 Important 项均已闭环，未发现新的 Critical、Important 或 Minor 问题。

本轮对照 `data/experiments/current/rgbd_terminal_20261007/source_before/` 审查 RGB-D 默认入口、末端新鲜重规划验收、tracker/mission 的执行期限交接，以及近距保速边界。主要涉及：

- `src/uav_control/uav_control/control/trajectory_tracker_node.py`
- `src/uav_control/uav_control/control/trajectory_tracking.py`
- `src/uav_control/uav_control/mission/mission_manager.py`
- `src/uav_control/uav_control/mission/mission_manager_node.py`
- `scripts/start_px4_ros2.sh`
- `src/uav_usv_bringup/launch/modular_intercept.launch.py`
- `src/uav_control/test/test_terminal_execution.py`
- `src/uav_control/test/test_camera_profile.py`

启动脚本与 launch 的默认前视深度模式均为 `ideal`；ToF 实现保留供显式选择。本轮未改 planner、KF、predictor 或 Z safety 算法。

## Important 闭环证据

### 1. XY 重合但尚未捕获时的非保护减速

独立复现先通过 `_evaluate_trajectory(fresh_replacement)` 得到 `NONE`，随后在 `t=10.5` 令新鲜 KF 与 UAV 的 XY 均为 `(3.25, 0)`，垂直距离为 `0.56m`。KF、prediction 与 visual lock 均有效，但原逻辑同时退出承诺入口和 cruise 计算，结果为 `TRACKING`、`SAFE`、无 commitment，XY 指令由 `6.5m/s` 降到 `6.288m/s`。

修复后，零 XY 距离及零预测 aim 向量保留上一条实际有限 XY 指令；初始无历史时使用当前状态速度。承诺入口移除零距离下限，保留原 `2.5m` 上限及所有新鲜度、来源、任务和期限检查。

独立执行零 XY 回归的两种剩余时间设置均通过：已有多项式使用 `5.5m/s` 仍保持 `6.5m/s`；最后 `0.7s` 内随后丢失深度，保速也仅持续至原有效期限。

### 2. tracker 与 mission 的剩余时间验收时刻不一致

独立复现使用配置允许的 `0.30s` 新计划：导航/轨迹起点为 `10.285`，当前控制时刻为 `10.4`，导航年龄 `0.115s`，`valid_until/contact_stamp=10.585`。原 core 按导航采样时刻检查剩余 `0.30s >= 0.20s` 并真正替换 plan 7→8；mission 收到当前剩余 `0.185s` 后拒绝接纳，保留 plan 7。下一条 `TERMINAL_COMMITTED(plan 8)` 因身份不匹配触发 `SAFE_RECOVERY`。

修复在调用原 `tracker.accept` 前，按当前控制时刻检查过期及原 `minimum_remaining_time=0.20s` 门槛。导航采样时刻仍用于原几何与连续性验收，没有重写或外推观测时间戳。新增 delayed-navigation 回归返回 `INSUFFICIENT_REMAINING_TIME`，并保留旧 active trajectory 与旧 execution context。

### 3. 此前终端审查中已关闭的异步与重置问题

- KF 先失效而 prediction 尚未超时：所有 committed 调用允许保速回退，core 仅在新鲜 steering 不可用时使用回退；有效 steering 继续执行。
- 明确的图像 epoch reset（`stamp=raw_stamp=0`）：立即撤销旧 commitment、active/pending plan，并锁存恢复。

上述两条曾分别独立复现并执行新增回归通过，相关测试也包含在最终专测中。

## 诊断顺序与期限核对

独立使用实际 `_publish_diagnostic` 与 `MissionManagerNode.controller_callback` 验证：

1. 真正通过完整验收的新 plan 8 清除旧 tracker context，并先发 `PLAN_ACCEPTED` / `TARGET_LOCK`；mission 接纳 plan 8 后清除旧 execution deadline。
2. 下一条 `TERMINAL_COMMITTED(plan 8)` 为该已接纳计划建立新的 deadline，tracker 保存对象与 active trajectory 为同一对象。
3. 提交 plan 9 的冗余候选而 core 未实际替换时，诊断仍报告 active plan 8，tracker context 与 mission deadline 均保持不变。

拒绝、失效及未实际替换的候选不能续期旧承诺。新鲜候选仍经过原 frame、source、状态几何、目标 endpoint、参考连续性和海面安全验收。盲执行仅限最终 `0.7s` 和已接纳轨迹原期限；`125ms` freshness、`0.5m` capture radius 与 sea guard 保持。

## 实际验证结果

独立在包目录执行：

```bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
cd /home/qin/data/uav_usv_mpc/src/uav_control
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  test/test_terminal_execution.py \
  test/test_trajectory_tracking.py \
  test/test_camera_profile.py
```

实际输出：`69 passed in 0.24s`，退出码 `0`。独立 `git diff --check` 无输出，退出码 `0`。

上述实际诊断交接、新计划/冗余计划身份、期限保持检查及零 XY 回归也独立执行通过。所有检查均未修改生产源码。

## 仍有限制

本独立审查确认代码入口、边界和上述专测结果，未独立执行或审计最终两轮完整仿真。主流程报告两轮 RGB-D 全仿真均首轮捕获且接触前指令未减速；该仿真结论应以主流程保存的原始数据与验收报告为证，不属于本记录的独立验证结果。

本记录也不将 RGB-D 仿真结果外推为 ToF 标定、真机海面测距覆盖或无标记非合作 USV 的已验证能力。完整回归、flake8、四包构建与最终实验验收结果由主流程分别记录。
