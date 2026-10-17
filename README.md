# UAV–USV 开发工作区

当前运行视觉/KF → BCTRA预测 → MINCO规划 → Tracker速度控制，Tracker是唯一PX4指令发布者。MPC研究模块保留，尚未作为在线控制器接管。

最新[P53同配置F2/F3验收](docs/current/p53_comprehensive_validation.md)已完成：两场景MINCO占比100%、零回退；F3水平观察点RMSE约0.70m。普通baseline启动与P53有界验收的运行配置分别保留，不混称同一轮验证。

## 日常构建与轻量启动

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/build_workspace.sh
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
export UAV_USV_WS=/home/qin/data/uav_usv_mpc
UAV_USV_ENABLE_SHADOW_PERCEPTION=false ./scripts/uav_lab.sh --no-build
```

该命令使用已有开关关闭shadow predictor、front monitor、selector三个诊断节点，主RGB-D、bearing、KF、预测、规划、任务与评价保留。需要完整诊断时将`false`改为`true`；启动器原默认值未改。当前前视理想RGB-D、下视关闭，ToF选项保留。X开始起飞/FOLLOW，Y开始截击，R重启，Q退出控制台。

## 开发与资料入口

- [当前文档](docs/current/README.md)：架构、环境、基线、最新验收与未来MPC范围。
- [系统精简与恢复](docs/current/system_simplification.md)：保留依据、旧工具恢复、检查结果。
- [研究历史](docs/history.md)：各阶段设计、对照、失败归因及旧验收；[更早归档](docs/archive_index.md)。
- `src/uav_control/`保留控制、感知、研究模块与测试；`src/uav_usv_bringup/`保留launch、配置和模型；`scripts/`保留当前运行、回放、诊断与构建工具。

主predictor/tracker使用`/tracking/target_state`；`/target/state`真值仅评价、显式shadow和离线分析。图像采集戳、因果pose等待、原始导航epoch、安全门禁及0.50m捕获球保持。

新实验与视频留在忽略的`data/experiments/`、`data/videos/`；冻结证据位于`data/baselines/`，不得改写。只暂存明确路径，不使用`git add -A`或全量推送脚本。原仓库`/home/qin/data/uav_usv`保持只读，两远端均指向[本仓库](https://github.com/Qinjianheng/uav_usv_mpc)。
