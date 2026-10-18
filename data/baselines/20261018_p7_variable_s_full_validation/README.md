# P7 当前版本变速 S 全流程复测

仅选入用户要求的一轮 X→MINCO→Y→半径捕获→暂停。Y 后 10.903095 s 达到原 0.50 m 半径，评价距离 0.493206 m；同刻独立复算 0.477308 m。Y 前局部实际 MINCO 6.285560 s、100%，无异常回退。完整结果见[当前报告](../../../docs/current/p7_variable_s_full_validation_20261018.md)。

本轮捕获及保护验收通过，**不是全程视觉锁定**：捕获前约 0.1296 s 进入 P7_BRAKING→SAFE_RECOVERY，恢复接口保持纯速度，没有复现旧位置/加速度字段错误。控制源码、运行参数、时效和半径没有修改。

按字节复制完整 flight/research 配置、原始 summary/condition/terminal_result 及独立分窗复算；来源和 SHA256 见 selection.json。原始 JSONL、源码快照、ULog、曲线和近场分析留在原 data/experiments 忽略目录，本包不是它们的 Git 备份。冻结旧基线和视频未改动。
