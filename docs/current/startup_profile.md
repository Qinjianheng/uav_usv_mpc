# 仿真按需启动

2026-10-18：一键入口接入 P7 手动捕获，只启动当前模式实际使用的进程；旧基线与研究配置保留。

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/uav_lab.sh
# 已构建时：./scripts/uav_lab.sh --no-build
```

脚本自动构建并加载 ROS/本工作区。数字键输入后需回车：1 直线、2 八字、3 变速八字、4 变速 S、5 变速 Z。**数字只选模式，USV 保持静止；X 才启动 UAV 起飞和 USV 运动。** 等待 `FLIGHT READY` 后输入 X，视觉锁定并稳定进入 MINCO FOLLOW 后输入 Y；捕获至 0.50 m 三维接触半径内时评价器暂停 Gazebo。初始不选择时为直线；R 完整重启后默认八字，可在 X 前重新选择。启动后不能在线换模。

Q 结束并清理当前 P7 会话，R 完整重启，每轮手动会话上限 280 s。输出保存在独立 `data/experiments/p7_manual_*`；脚本不自动发送 X/Y。已有显式 `UAV_USV_EXPERIMENT_LAUNCH` 的研究驱动仍直接进入原后端，避免递归启动 P7。

当前直线与恒速八字的水平速度均为 **4 m/s**，起步加速度限制仍为 1 m/s²；三种变速模式保留原 2–4 m/s 规律。实验准备从 R11 复制场景几何后，以当前 `p7_guidance.yaml` 的 `horizontal_speed` 覆盖旧恒速设置，避免 F2 历史的 2 m/s 覆盖当前一键配置。完整实际参数仍逐轮另存；历史场景文件不改写。

| 入口 | 默认在线节点 | 旧终端截击规划器 |
|---|---:|---|
| `scripts/uav_lab.sh` → P7 手动捕获 → `p4_follow_research.launch.py` | 10 | 不启动，Y 由 Tracker 执行 IBVS/PNG |
| `scripts/uav_lab.sh --baseline` → `modular_intercept.launch.py` | 11 | 保留，普通基线的 Y 阶段需要 |
| `p7_guidance.launch.py` | 10 | 不启动，Y 由 Tracker 执行 IBVS/PNG |
| P7 实验脚本 → `p4_follow_research.launch.py` | 10 | `UAV_USV_P7_MODE=true` 时不启动 |

P7 的 FOLLOW 规划器仍然运行。前视 RGB-D 桥、方位检测、定位、KF、唯一主 predictor、任务管理、唯一 Tracker 和评价器均保留。评价器负责捕获判定与暂停；方位检测在深度失效时仍用于搜索与锁定。下视传感器及桥、额外感知诊断、功能 ToF 模型继续默认关闭，原有显式开关保留。

旧基线与 P7 的主 predictor 分别为 IMM 与 BCTRA。一键入口默认调用 `p7_follow_sitl.py --manual --capture`，启用半径捕获；底层 P7 实验入口仍通过 `--capture` 显式选择，省略时保持安全接近。

QGroundControl 默认不自动打开。P7 自动/手动实验驱动提供自己的轻量心跳，并设置 `UAV_USV_HEADLESS_GCS=true`，后端不重复启动。旧基线后端若已有 QGC 则复用，否则由 `scripts/run_with_sitl_gcs.py` 管理仅发送 GCS 心跳的本地辅助进程，不发送飞行或参数指令；随 ROS launch 结束、实验终端退出或 R 重启一起停止。旧基线模式 Q 只关闭命令台，因此不会中断仍运行的实验心跳。

旧基线入口需要自动打开 QGroundControl 时：

```bash
UAV_USV_START_QGC=true ./scripts/uav_lab.sh --baseline --no-build
```

需要恢复旧规划器用于专门诊断时，可在对应 `ros2 launch` 命令中显式传入 `enable_intercept_planner:=true`。这仅恢复节点进程，P7 自身的控制门禁仍然有效。

一键入口本次验证为本地回归，未新增飞行：完整测试 2226 passed、2 skipped、3 warnings，含五种模式选中后静止、X 后才运动，以及数字选择不发布 X 的控制台检查。`uav_control`、`uav_usv_bringup` 构建，脚本语法、flake8、差异检查均通过；日志位于 `data/experiments/20261018_one_click_p7_01/`。此前单次 S 捕获见[六项修复与实际验证](audit_repair_validation_20261018.md)，不作为新包装器已经飞行的证据。
