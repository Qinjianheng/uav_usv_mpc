# P7 直线 FOLLOW 连续性：轻量基线

选入修复提交 `fa5bd6e` 的两次相同配置直线 2 m/s F2 回归。固定 25 秒观察内实际 MINCO 均为 100%、零回退；诊断覆盖的连续区间分别为 24.943532 / 24.950008 秒。原 125 ms 输入、50 ms bracket 和 450 ms 租约保持。

根因、原失败时序、修复和证据边界见[当前报告](../../../docs/current/p7_follow_continuity_20261018.md)。第一次 KF 陈旧在本轮未再现，不声称穷尽了偶发交付延迟的全部内部原因；没有执行 F3 或新捕获轮。

每个子目录按字节选入完整 flight/research 配置、原始 summary/condition 及离线 metrics。来源和 SHA256 见 [selection.json](selection.json)。原始 JSONL、源码快照、ULog、所有旧失败和新曲线仍原位保留于忽略的 `data/experiments/`；本包不是这些大日志的 Git 备份。冻结旧基线和视频未改动。
