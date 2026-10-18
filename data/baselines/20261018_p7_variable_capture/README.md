# P7 三种变速捕获：轻量基线选入

选入最终同源码的变速八字 02、S 02、Z 01，源码修复提交 `f2f4a20`。三轮均在原 0.50 m 三维半径捕获并暂停；完整结果、失败归因和未获 F3 资格的独立直线回归见[当前报告](../../../docs/current/p7_variable_capture_20261018.md)。冻结的 `20261006_pre_mpc` 未改动。

各子目录按字节复制本轮完整 `config/flight.yaml`、`config/research.yaml`、原始 `summary.json` 与 `terminal_result.json`。来源、逐文件 SHA256、物理时刻复算指标及原始大日志指纹见 [selection.json](selection.json)。

本包是轻量选入，**不是原始大日志的 Git 备份**。原始 JSONL、完整源码快照、ULog、逐轮 CSV、曲线与所有失败原件仍原位保留在 `data/experiments/` 忽略目录；使用清单中的路径和 SHA256 检查本机证据。没有移动、删改或自动提交这些原始目录和视频。
