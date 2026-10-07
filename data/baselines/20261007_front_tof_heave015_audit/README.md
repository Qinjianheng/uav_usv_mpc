# 前视 ToF / 0.15 m 升沉审计包（不是成功基线）

来源：`/home/qin/data/uav_usv_mpc/data/experiments/current/tof_heave_20261007/`；2026-10-07 选入并按字节复制，原始实验记录保留。当前 ToF 两轮均失败；第 3 轮是理想深度单次成功对照。

- `run1/`：修正性能问题前，图像配对失败、任务超时。
- `run2/`：配对修复后，带噪球面定位拒绝、任务超时。
- `run3_ideal_control/`：0.15 m / 仅前视的理想深度对照成功，包含误差及冻结核对。
- 每轮保留 CSV、vision CSV、evaluator config、summary、完整运行参数副本、probe 与节点日志；ToF 两轮附噪声配置。
- `same_sphere_noise_audit.json`：同球面加噪前后定位有效性对照。
- `final_source_fingerprint.json`：最终工作区源码指纹。不是第 1 轮修改前的完整源码快照，不能冒充历史记录。
- `checks/`：实际测试、构建、DDS 冒烟及旧冻结包校验日志。
- `auto_probe.py` / `stop_owned.py`：本轮仿真记录与定向清理工具副本；`ros_image_smoke.py` 为隔离 ROS 域消息测试副本，不是产品默认启动入口。

详细分析：[验证报告](../../../docs/current/tof_validation_20261007.md)。当前限制与恢复：[相机文档](../../../docs/current/camera_simulation.md)。

本包建立时的文件校验：在本目录运行 `sha256sum -c SHA256SUMS`。原 `20261006_pre_mpc` 冻结包未改。
