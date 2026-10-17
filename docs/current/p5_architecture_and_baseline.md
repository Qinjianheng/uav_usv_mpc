# P5 架构与研究基线

P4.7/C H1.6 为唯一主候选，P4.4/D 是明确对照和 fallback。MPC 尚未实现；
本阶段验证既有 BCTRA → FOLLOW rollout → MINCO → Tracker → PX4 速度控制链。
独立完整验收、原始输入 125ms TTL、450ms nominal lease、ACK/parent/boot/epoch 协议均保留。
Tracker 仍是唯一 PX4 指令发布者。真值只进入评价和离线分析，红球仅验证仿真感知接口。

研究入口：`src/uav_usv_bringup/config/p5_follow_research.yaml` 和
`src/uav_usv_bringup/launch/p5_follow_research.launch.py`。
新选择列表只允许 `p47_follow_guided` / `p44_adaptive`，原 launch 与历史 engine 不改。
默认研究输出不获得执行权。P44/D 一键切回（影子研究）：

```bash
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
ros2 launch uav_usv_bringup p5_follow_research.launch.py engine:=p44_adaptive
```

实际执行只能使用独立 P5 实验入口 `scripts/p5_minco_sitl.py`。
它核验新 H1.6 证据 SHA 和限定门禁、排除其他仿真/串口硬件/占用 DDS 端口，
复用原隔离 harness 签发的真实进程身份授权。原 FOLLOW 收敛稳定 5s 后才启动研究 Planner，
不通过动态改参冒充已加载 engine，不强制 ACTIVE。
Planner 仅发布 FollowTrajectory；Tracker 自主验收、发送真实 ACCEPTED/ACTIVE/EXPIRED 并执行。
20Hz 只读健康监视保留 sample/source/observation 时间，检查原始输入期限。

日志 `condition.json`、完整配置、`source_snapshot/`、`environment.json`、真实
`sitl_execution_grant.json`、Planner JSONL、ACK/proposal/原生 PX4 setpoint 记录 engine、H、
terminal_mode、refinement、fallback、Git commit、约束指纹、源码及配置 SHA、实际授权状态。
未提交改动由实验 source snapshot 标识，不能仅凭 commit 认定运行源码。

本地恢复标签：`baseline-p47-before-p5`、`baseline-main-before-p5`、
`baseline-p5-research-start`。P5 本地分支为 `codex/p5-h16-closed-loop`；标签未推送。
两个研究标签指向原 HEAD `a35e405`，不意味着新增未提交配置已进入 Git 基线；
新配置的实际版本以实验 source snapshot、SHA 和当前 diff 为准。

后续仅在持续实际执行验证通过后研究预测误差、自适应机动、FOV 裕度和不确定性。
本轮不增加新优化器、视觉网络、目标、障碍物或学习控制。

2026-10-12 续做：原先仅以健康状态计时，已根据实际启动过渡数据补为共同 epoch 的
位置/相对速度/高度连续收敛门禁；仅只读 KF 与 native navigation，不外推或使用真值。
同时修复快照计时、环境文件发布竞态及晚启动组件清理。最后稳态门禁尚未重新 SITL。
当前脚本固定目录为 `data/experiments/20261012_p5_revalidation_02`，F2次数已消耗；
完整判据、已有用户授权和实际失败结果见 [复验报告](p5_revalidation_20261012.md)。
