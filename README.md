# UAV–USV MPC 开发工作区

当前仍运行原有视觉/KF → BCTRA 预测 → MINCO 规划 → 速度跟踪控制器，**MPC 尚未实现**。本仓库从原项目 `85e13461` 建立，2026-10-06 的整理只调整数据、文档和构建路径，保持现有控制行为。

优先阅读 [当前文档](docs/current/README.md)、[架构与控制边界](docs/current/architecture.md)、[成功基线](docs/current/baseline.md)、[环境与启动](docs/current/environment.md)。后续工作见 [MPC 范围](docs/current/mpc_scope.md)，历史材料见 [归档索引](docs/archive_index.md)。

当前事实以 [baseline.yaml](src/uav_usv_bringup/config/baseline.yaml) 和代码为准：

- 2026-10-07：默认仅前视 RGB-D（`front_depth_model=ideal`），下视暂时关闭；目标升沉幅值 0.15 m、频率 0.25 Hz。ToF 代码与配置保留，可显式开启，详情见 [相机文档](docs/current/camera_simulation.md)。冻结成功基线仍为原 0.05 m 场景。当前末端重新验收修复及验证状态见 [RGB-D 验证](docs/current/terminal_replan_rgbd_20261007.md)；早期带噪定位及失败记录见 [ToF 复验](docs/current/tof_fit_validation_20261007.md)。
- 主 predictor 和 tracker 从 `/tracking/target_state` 使用 KF 状态；真值 `/target/state` 供仿真评价及明确的 shadow/离线审计。
- tracker 自行做位置反馈、限速和加速度整形，向 PX4 发送速度指令；`use_velocity_control: true`。
- BCTRA 预测窗口为 4.0 s；MINCO 规划最大时域为 1.5 s，常规/末端调度分别为 5/10 Hz。
- 现有 0.50 m 捕获判据、125 ms freshness、海面保护和末端速度修复保持。

## 构建与启动

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/build_workspace.sh
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
export UAV_USV_WS=/home/qin/data/uav_usv_mpc
ros2 pkg prefix uav_control
./scripts/uav_lab.sh --no-build
```

包前缀应为 `/home/qin/data/uav_usv_mpc/install/uav_control`。启动控制台中 X 同时开始起飞与目标运动，视觉锁定 FOLLOW 后 Y 开始截击；Q 仅退出控制台，仿真终端仍保留。完整环境要求和独立 launch 命令见 [环境文档](docs/current/environment.md)。

## 数据与提交

[20261006_pre_mpc](data/baselines/20261006_pre_mpc/README.md) 保存一轮失败对照和三轮连续成功，包含原始 CSV、vision CSV、evaluator 配置快照、summary、完整基线配置及速度/冻结证据。原始记录内容不变。

新实验写入 `data/experiments/`，参数备份和视频默认忽略；审计后选择必要证据放入 `data/baselines/`，附来源与 SHA256 再明确暂存。`.gitignore` 不会取消旧文件跟踪，本轮已经按 [逐文件清单](docs/current/audit/retention_manifest.tsv) 处理旧副本。

只暂存明确审核的路径，不使用 `git add -A`。现有 `sync_github.sh` 会全量暂存并推送，不作为精选基线的默认提交入口。`origin` 和 `github` 的 fetch/push 均指向 [新仓库](https://github.com/Qinjianheng/uav_usv_mpc)。本轮检查与规模见 [整理交付记录](docs/current/repository_cleanup.md)。
