# 带噪 ToF 球面适配复验：失败证据包

2026-10-07 运行 `20261007_104013_014381`，0.15 m / 0.25 Hz 升沉、仅前视、带噪 ToF。结果为 FAILURE / TIMEOUT；最近距离 0.504192 m，捕获半径 0.50 m。这是末端连续执行改动之前的记录，不是成功基线。

- [验证报告](validation_report.md)：实际精度、阶段切分、未接触即减速与限制。
- `run1/raw/`：原始 CSV、vision CSV、evaluator 配置快照和 summary，逐字节复制自新工作区 `data/experiments/current/`；[来源与 SHA256](run1/raw_copy_manifest.json)。
- `run1/full_config.yaml` 和 `run1/front_tof_simulation.yaml`：本轮完整主配置和 ToF/拟合配置；运行前 8 文件快照及指纹在 `run1/source_snapshot/` 和 `run1/source_fingerprint.json`。
- `run1/probe.jsonl`、ROS 日志、话题/节点/真值消费者记录：原运行证据。原始 CSV 包含超时后记录，分析显式截止结果时刻。
- [同刻误差统计](analysis/analysis.json)、[减速时序](analysis/deceleration_trace.csv)、[近距深度诊断](analysis/near_depth_trace.json)。
- `checks/`：红/绿测试、877 passed / 1 skipped、flake8 和构建记录。141 个原有受保护文件未改变，见 `run1/protection_check.json`。
- `run1/tools/analyze_recording.py` 支持复算至独立输出目录，命令见报告。

`SHA256SUMS` 覆盖本包其余文件。20261006 冻结基线及此前相机失败/理想对照证据包不改写。所有复制均来自 `/home/qin/data/uav_usv_mpc` 与本轮临时检查目录，没有更改原仓库。
