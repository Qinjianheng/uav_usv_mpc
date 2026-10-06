# 历史归档索引

这个工作区用于后续修改。原仓库 `/home/qin/data/uav_usv` 仍完整保存实验历史；原始文件逐字节备份也在 `/home/qin/data/uav_usv_mpc_archive/20261006_pre_mpc/repository_before.tar.gz`。本轮只精简新目录副本。

当前入口：[README](../README.md)、[当前文档](current/README.md)。逐文件去向、SHA256、大小、原仓库关系及原仓库 `85e13461` 文件链接统一放在 [保留清单](current/audit/retention_manifest.tsv)，无需在开发文档中重复列数千条历史文件。

| 历史材料 | 原仓库 85e13461 链接 |
|---|---|
| 原始实验 | [data/experiments/current](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/data/experiments/current) |
| 更早基线 | [data/experiments/baseline_legacy](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/data/experiments/baseline_legacy) |
| PX4 参数备份 | [data/experiments/px4_parameter_backups](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/data/experiments/px4_parameter_backups) |
| 视频 | [data/videos](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/data/videos) |
| 跟踪/末端修复报告与证据 | [docs/tracking](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/docs/tracking) |
| 成功收尾报告 | [docs/tracking/terminal_cruise_validation_20261006.md](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/docs/tracking/terminal_cruise_validation_20261006.md) |
| 末端速度、冻结、历史记录工具 | [docs/tracking/y_intercept_evidence_20261005](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/docs/tracking/y_intercept_evidence_20261005) |
| 视觉归因 | [docs/vision](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/docs/vision) |
| 旧性能与基准 | [docs/performance](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/docs/performance) |
| 旧设计与计划 | [docs/superpowers](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/docs/superpowers) |
| 旧 README | [README.md](https://github.com/Qinjianheng/uav_usv/tree/85e13461df62b65d34aa55c9d92576c58085d0b3/README.md) |

GitHub 链接标识原仓库该提交；本轮通过本地 Git 对象核对内容，没有访问远端。恢复步骤见 [分析与恢复](current/analysis_and_recovery.md)。

6 个本机独有文件不在原仓库 Git 内：一轮 `20261006_102846_255769` 的 CSV/vision/config/summary，以及 `enu_caLQn2` 的两份 PX4 参数备份。它们已归档校验，详见 [本机文件清单](current/audit/local_only_files.json)，必须从本地归档恢复。

归档 SHA256 和逐成员检查见 [归档核对](current/audit/archive_verification.json)；初始 HEAD、状态、remote URL 和规模见 [清点快照](current/audit/snapshot_before.json)。当前四轮精选基线见 [数据包](../data/baselines/20261006_pre_mpc/README.md)。大体量成功 JSONL 与历史工具留在原仓库和归档中，可按清单定位。
