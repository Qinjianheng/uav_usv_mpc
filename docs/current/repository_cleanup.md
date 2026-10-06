# 仓库整理交付（2026-10-06）

新目录用于后续修改，保留完整运行功能、四轮精选基线和必要证据。当前仍为原 BCTRA/MINCO/tracker 速度控制器，MPC 尚未实现。本轮只改构建默认目录、忽略规则、文档和数据布局；462 个源码、测试、消息、配置、模型、world 与补丁文件均按整理前 SHA256 核对不变。

## 清点与保护

整理前新旧仓库 HEAD 都是 `85e13461df62b65d34aa55c9d92576c58085d0b3`。原仓库为 master，新仓库为 main。新仓库原有 2339 个跟踪文件，无跟踪文件改动；另外有 6 个未跟踪文件：`20261006_102846_255769` 一轮的四种实验文件和 `enu_caLQn2` 的两份参数 BSON。原仓库工作区干净。6 项本机独有内容均已先归档再移出新目录。

`origin` 和 `github` 的全部 fetch/push URL 均为 `https://github.com/Qinjianheng/uav_usv_mpc.git`。初始详情见 [快照](audit/snapshot_before.json)，全部 2345 个文件的保留/修改/归档/基线去向见 [唯一逐文件清单](audit/retention_manifest.tsv)，引用检索见 [引用审计](audit/reference_audit.json)。

持久归档：`/home/qin/data/uav_usv_mpc_archive/20261006_pre_mpc/repository_before.tar.gz`，大小 521.71 MiB，SHA256 `8e2b72785d197c788bf788d0d64dbb90ad871805dbb8120430cd6f84a1bdf309`。从归档逐个读取 2345 个文件核对成功，最终再次核对归档 SHA256；[核对证据](audit/archive_verification.json)。归档包含初始清单与状态 metadata；工作区快照另明确区分初始六项状态与创建审计文件后的状态。

原目录 `/home/qin/data/uav_usv` 的文件校验值和 Git 工作区状态最终仍一致。历史材料统一由 [归档索引](../archive_index.md) 定位；本机独有项见 [六项清单](audit/local_only_files.json)。

## 整理前后规模

| 范围 | 整理前 | 整理后 |
|---|---:|---:|
| 非生成文件数 | 2345 | 594 |
| 非生成文件内容 | 1435.99 MiB | 14.76 MiB |
| data 内容 | 1264.66 MiB | 11.16 MiB |
| docs 内容 | 169.02 MiB | 1.30 MiB |

计量按普通文件逻辑字节，排除 `.git`、build/install/log、缓存与沙箱元数据，详见 [规模快照](audit/size_snapshot.json)。Git 对象历史约 540 MiB 仍保留，未重写历史；build/install/log 仍为本机生成目录，构建后规模会变化。

旧路径移出 1836 项：1792 项外置，44 项复制到当前基线；另外保留 506 项不变，修改 3 项（README、.gitignore、构建脚本）。原先已跟踪的移出项共有 1830 项，在整理提交中按清单取消跟踪；不是只添加 ignore 规则。

四轮精选记录共有 16 个 CSV/vision/config/summary；连同原成功报告、指纹、速度/冻结证据和两个完整配置，包内校验 46 项。三份大体量 JSONL 和三份历史记录工具外置，可从原仓库/归档恢复；新工作区不重复装入整套历史。配置范围说明见 [基线](baseline.md)。

## 实际检查

| 检查 | 实际结果/证据 |
|---|---|
| 原 pytest | **836 passed / 1 skipped / 2 warnings**；从 src/uav_control 运行，[日志](audit/pytest.log) |
| 项目 flake8 | exit 0；uav_control 源码、测试、scripts，[日志](audit/flake8.log) |
| shell 语法 | 8 个脚本 `bash -n` 全部 exit 0，[日志](audit/shell_syntax.log) |
| 构建 | 从 /tmp 清除 UAV_USV_WS 后直接调用构建脚本，**4 packages finished / exit 0**，[日志](audit/build.log) |
| 路径覆盖 | UAV_USV_WS 的 cwd/colcon 参数 smoke 通过；colcon 使用替身，仅核对覆盖传递，[日志](audit/build_path_override.log) |
| ROS 包前缀 | `/home/qin/data/uav_usv_mpc/install/uav_control`，[日志](audit/ros_package_prefix.log) |
| 原始数据/完整配置 | `sha256sum -c SHA256SUMS` **46/46 OK**，[日志](audit/baseline_checksums.log) |
| 文件/配置/remote/ignore | 静态和 SHA256 验证通过，[结果](audit/verification.json) |
| 文档/历史链接 | 当前相对链接核对通过；2339 个历史 Git 对象存在；原成功报告的 16 个链接在原目录上下文有效 |
| 当前分析 | 四轮 vision 与 P8 视觉归因实跑；[vision 结果](audit/vision_analysis.json)、[P8 结果](audit/visual_bias_analysis.json)、[summary 提取](audit/baseline_summary_analysis.json) |
| 历史入口恢复 | legacy、P4 九个输入、末端速度重放成功；重放速度 JSON 与原证据相同，[恢复步骤](analysis_and_recovery.md) |
| Git whitespace | 工作区和暂存区均 exit 0；[暂存区日志](audit/git_cached_diff_check.log) |

独立只读审查未发现 Critical/Important 问题；[审查记录](audit/review.json)。新生成 TSV 的 CRLF 已修正为 LF，重新暂存后 whitespace 检查通过，原实验字节不变。

两条 pytest warning 为已有 ament/flake8 的 SelectableGroups 弃用提示；P4 实跑另有恒定输入的相关系数 warning。未改写这些分析算法或测试。两个旧 CSV 分析入口不适用于当前 modular 日志字段/phase，限制和正确入口已记录在 [分析文档](analysis_and_recovery.md)，不能把其 0% 当成当前感知失效。本轮没有新增飞行实验，三次成功依据为原冻结证据。

原成功报告及 JSON 按字节保留历史路径，报告的相对链接按原仓库目录解释；当前开发文档使用新路径。必要时通过 manifest/source URL 或恢复目录定位原文件。

## 继续开发

```bash
cd /home/qin/data/uav_usv_mpc
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
export UAV_USV_WS=/home/qin/data/uav_usv_mpc
ros2 pkg prefix uav_control
./scripts/uav_lab.sh --no-build
```

需重新构建时运行 `./scripts/build_workspace.sh`。X 起飞/目标启动，视觉锁定 FOLLOW 后 Y 截击。后续 MPC 范围见 [MPC 文档](mpc_scope.md)，操作约束见 [AGENTS](../../AGENTS.md)。新实验默认忽略，选入新基线后明确暂存；不覆盖 pre_mpc 原始证据。
