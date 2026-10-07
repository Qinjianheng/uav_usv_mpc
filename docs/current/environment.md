# 环境、构建与启动

2026-10-06 在本机读取的环境：Linux x86_64、Python 3.10.12、ROS 2 Humble（`/opt/ros/humble`）、colcon-core 0.21.0；NumPy 1.24.4、SciPy 1.8.0、PyYAML 5.4.1、pytest 6.2.5、flake8 4.0.1、ament-flake8 0.12.15、pyulog 1.2.3。版本是当前核对值，不代表依赖版本锁。

工作区包含 `px4_msgs`、`uav_usv_interfaces`、`uav_control` 和 `uav_usv_bringup`。系统需要 ROS launch/rclpy/消息依赖、`ros_gz_image`、Gazebo Transport 13 / Msgs 10 的 Python 模块，以及 `gz`、`MicroXRCEAgent`、`gnome-terminal`、`timeout`、`rg`。离线速度绘图另需 matplotlib。声明见 [package.xml](../../src/uav_control/package.xml) 和 [bringup package.xml](../../src/uav_usv_bringup/package.xml)。

PX4 SITL 源码默认在 `/home/qin/Projects/PX4-Autopilot`，当前读取 HEAD 为 `de8158101c96ad6b04170dc91f087148104c58eb`。HEAD 不包含未提交改动的证明；启动还依赖已构建 SITL、现有导航/磁场/GNSS 修复和 rootfs。工作区 [patches/px4](../../patches/px4) 本轮逐文件保留。`PX4_ROOT` 可覆盖外部 PX4 路径；QGroundControl AppImage 路径由 `QGC_APPIMAGE` 覆盖。本轮没有重新安装环境或改动外部 PX4。

## 构建

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/build_workspace.sh
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
export UAV_USV_WS=/home/qin/data/uav_usv_mpc
ros2 pkg prefix uav_control
```

构建脚本默认从自身位置推导工作区，`UAV_USV_WS` 仍可覆盖；启动包装器 `uav_lab.sh` 将工作区固定为该脚本所在仓库。请在新终端先 source ROS，再 source 新工作区，避免旧 overlay 顺序干扰；ROS setup 前使用 `set -eo pipefail`，不启用 `set -u`。预期包前缀为 `/home/qin/data/uav_usv_mpc/install/uav_control`。

## 完整仿真

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/uav_lab.sh --no-build
```

启动器使用新仓库路径，并打开 Gazebo、PX4、DDS 和实验终端。X 同步启动 UAV 起飞/USV 运动；视觉锁定 FOLLOW 后发送 Y。成功后现有评价器暂停 Gazebo 并冻结目标。R 通过启动器恢复干净会话；Q 只关闭指令控制台，其他终端仍运行。

当前默认前视 RGB-D、下视关闭、目标升沉 0.15 m。ToF 暂停使用，代码保留。物理传感器开关需要重启完整仿真；详细配置与切换见 [相机文档](camera_simulation.md)。

已有 PX4/Gazebo/DDS 会话时只启动 ROS 图可用：

```bash
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
export UAV_USV_WS=/home/qin/data/uav_usv_mpc
ros2 launch uav_usv_bringup modular_intercept.launch.py   config_file:=/home/qin/data/uav_usv_mpc/src/uav_usv_bringup/config/baseline.yaml   log_directory:=/home/qin/data/uav_usv_mpc/data/experiments/current
```

直接 launch 时显式设置工作区/日志路径：现有 launch 未设置 `UAV_USV_WS` 时仍有原仓库 fallback。`enable_shadow_perception` 只控制部分诊断，不应当用它推断主视觉链是否关闭。

## 本轮静态验证入口

```bash
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
cd /home/qin/data/uav_usv_mpc/src/uav_control
python3 -m pytest -q
cd /home/qin/data/uav_usv_mpc
python3 -m flake8 src/uav_control/uav_control src/uav_control/test scripts
bash -n scripts/build_workspace.sh
bash -n scripts/start_px4_ros2.sh
bash -n scripts/uav_lab.sh
bash -n scripts/sync_github.sh
git diff --check
```

仓库整理时的输出见 [交付记录](repository_cleanup.md)。后续 RGB-D/控制修改及新增仿真实际检查见 [当前验证](terminal_replan_rgbd_20261007.md)，不能将原 0.05 m 冻结成功直接作为当前 0.15 m 场景的验收。
