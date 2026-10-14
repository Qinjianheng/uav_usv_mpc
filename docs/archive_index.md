# 历史归档索引

最新用户明确无需归档、直接删除旧日志后，2026-10-14另行删除约12.04GB旧实验及非latest日志，未新增归档。最近五个真实场景、冻结基线和当前H1.6/场景配置依赖保留；逐文件删除清单及当前状态见[系统精简](current/system_simplification.md)。下面既有归档不覆盖此次全部删除内容，历史命令不保证原样重跑。

2026-10-14新增[系统精简、复验与恢复](current/system_simplification.md)：14个无当前程序依赖的旧CLI与193个旧colcon日志目录归档至`/home/qin/data/uav_usv_mpc_archive/20261014_system_cleanup_01/`；P53 R1～R10共17个旧场景目录及5份已完成计划归档至同级`20261014_system_cleanup_02/`。逐成员内容、大小、权限和冷恢复校验通过。R11成功对照、旧轮次attempt/summary、报告正文与旧资格保留；旧报告中的场景原路径须先恢复再校验。逐文件清单在各归档目录，导航统一见[研究历史](history.md)。

2026-10-12 新增 [待推送提交体积整理与恢复](current/submission_size_cleanup_20261012.md)：
P4.7 的 470 个新增大体量/重复实验文件保留本机并从候选提交移出，完整原提交
另存为持久 bundle；逐文件清单、归档 SHA256 和独立恢复核验见该入口。

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

## 后续源码冗余清理

原整理清单记录 `c2ef5ae` 的布局。本轮另外移出 3 个源码历史副本，其去向以 [补充清单](current/audit/redundancy_cleanup_20261006.json) 为准；原先快照及冻结实验记录不改写。详情和恢复命令见 [源码冗余清理](current/redundancy_cleanup.md)。

| 移出文件 | 原仓库 85e13461 |
|---|---|
| `src/uav_control/uav_control/guidance/intercept_planner_node.py.bak` | [历史文件](https://github.com/Qinjianheng/uav_usv/blob/85e13461df62b65d34aa55c9d92576c58085d0b3/src/uav_control/uav_control/guidance/intercept_planner_node.py.bak) |
| `src/uav_control/uav_control/archive/pure_pursuit_backup.py` | [历史文件](https://github.com/Qinjianheng/uav_usv/blob/85e13461df62b65d34aa55c9d92576c58085d0b3/src/uav_control/uav_control/archive/pure_pursuit_backup.py) |
| `src/uav_control/uav_control/archive/predictive_intercept_v1.py` | [历史文件](https://github.com/Qinjianheng/uav_usv/blob/85e13461df62b65d34aa55c9d92576c58085d0b3/src/uav_control/uav_control/archive/predictive_intercept_v1.py) |
