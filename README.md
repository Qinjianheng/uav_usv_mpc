# UAV–USV 开发工作区

P7 专用配置已完成视觉/KF → BCTRA → MINCO FOLLOW，以及 Y 后的 IBVS＋PNG 安全接近；Tracker 是唯一 PX4 指令发布者。运行、真实飞行曲线和边界见 [P7 使用与验证](docs/current/p7_progress.md)。原 baseline/IMM 研究入口保留；MPC 尚未作为在线控制器接管。

当前捕获判据按用户要求恢复为 **0.5 m**，默认使用 28°／5 m 与旧 P7。2026-10-20 单次恒速八字在 Y 后约 14.48 s 捕获并暂停世界；独立原生模型冻结复核记录缺失，完整边界与 0.25 m 失败原因见[本次验证](docs/current/p7_radius05_validation_20261020.md)。显式三维候选也同步为 28°／5 m／0.5 m，但未由本轮验收。

历史[P53同配置F2/F3验收](docs/current/p53_comprehensive_validation.md)作为成功对照保留；本轮 P7 独立恢复与飞行结果见上述入口。普通 baseline、P53 与 P7 配置分别保留，不混称同一轮验证。

## 日常构建与轻量启动

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/uav_lab.sh
```

脚本自动构建并加载 ROS/本工作区；已经构建时可用 `./scripts/uav_lab.sh --no-build`。默认进入 P7 手动捕获，不固定变速模式，也不自动发送 X/Y。

- 输入 `1` 直线、`2` 八字、`3` 变速八字、`4` 变速 S、`5` 变速 Z，再回车。数字只选择，USV 保持静止；未选择时初始为直线。
- 等待 `FLIGHT READY` 后输入 `X`：UAV 起飞、USV 开始运动，视觉锁定后进入 MINCO FOLLOW。
- 稳定跟随后输入 `Y`：进入 IBVS/PNG 捕获；达到当前 0.5 m 原点三维半径才由评价器暂停仿真。
- `Q` 结束并清理本轮，`R` 完整重启；R 后默认八字，可在 X 前重新选择。每轮手动会话上限 280 s。

直线和恒速八字均为 **4 m/s**，按原 1 m/s² 起步限制加速；三种变速模式保持原 2–4 m/s 规律。

P7 默认运行 10 个节点，关闭闲置的旧截击规划器；主 RGB-D、bearing、KF、BCTRA、FOLLOW 规划、任务与评价保留。下视、额外感知诊断、功能 ToF 按需启动。默认使用随实验结束的轻量 GCS 心跳，不自动打开 QGroundControl。旧 11 节点 IMM 基线入口保留为 `./scripts/uav_lab.sh --baseline --no-build`。详见[启动说明](docs/current/startup_profile.md)。

## 开发与资料入口

- [当前文档](docs/current/README.md)：架构、环境、基线、最新验收与未来MPC范围。
- [系统精简与恢复](docs/current/system_simplification.md)：保留依据、旧工具恢复、检查结果。
- [研究历史](docs/history.md)：各阶段设计、对照、失败归因及旧验收；[更早归档](docs/archive_index.md)。
- `src/uav_control/`保留控制、感知、研究模块与测试；`src/uav_usv_bringup/`保留launch、配置和模型；`scripts/`保留当前运行、回放、诊断与构建工具。

主predictor/tracker使用`/tracking/target_state`；`/target/state`真值仅评价、显式shadow和离线分析。图像采集戳、因果pose等待、原始导航epoch、安全门禁及原生三维捕获语义保持；当前半径为0.5m。

新实验与视频留在忽略的`data/experiments/`、`data/videos/`；冻结证据位于`data/baselines/`，不得改写。只暂存明确路径，不使用`git add -A`或全量推送脚本。原仓库`/home/qin/data/uav_usv`保持只读，两远端均指向[本仓库](https://github.com/Qinjianheng/uav_usv_mpc)。
