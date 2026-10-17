# 2026-10-12 待推送提交体积整理

## 原因与范围

整理前工作区和暂存区均干净。只读访问 GitHub 确认远端 `master` 为
`741000cd8e1faa22ba85292827f59c2fc0a21f99`，本地 `master` 多出一个提交
`43198343a2260392749e979bbc3beae6ce6f20c4`。

该提交引入 267 个新 blob，共 2,977,850,154 字节；主要是 P4.7 正式矩阵
JSONL、逐请求汇总及源码快照。新增实验路径有 475 个，共 3,027,355,469
字节；重复内容的路径共用 blob，故两个数值不同。`.gitignore` 缺少整个
`data/experiments/` 的默认忽略规则，是新增实验被全量收进提交的原因。

本轮仅处理这个未推送提交的提交范围：

- 470 个新增实验文件从暂存树移出，原文件仍留在原路径，共 3,026,681,971 字节。
- 保留 P4.7 README、原交付 SHA256 索引及正式报告直接引用的三张新增图，
  共 673,498 字节。报告另外两张图已经在远端基线提交内。
- 增加 `data/experiments/` 默认忽略规则；后续精选证据按仓库约束放进新的
  `data/baselines/` 子目录，附来源和 SHA256。
- 保留全部代码、测试、运行配置、模型、PX4 补丁和冻结基线字节；没有修改
  原仓库 `/home/qin/data/uav_usv`。

已在远端的历史实验文件继续保持原跟踪状态。此次不改写已发布历史，
也不清理本地 Git 对象，因此 `.git` 的磁盘占用不能代表这次待推送大小。

## 归档与逐文件索引

持久归档：

```text
/home/qin/data/uav_usv_mpc_archive/20261012_submission_cleanup_4319834/
  repository_before.bundle
  original_files.tsv
  excluded_files.tsv
  excluded_paths.nul
  snapshot_before.json
  SHA256SUMS
```

`repository_before.bundle` 保存原本地提交及全部可达历史，没有先决提交依赖。
`original_files.tsv` 记录整理前所有跟踪文件的 SHA256、Git blob、模式和大小。
不依赖旧 Git 工作区即可恢复这些已跟踪文件；未跟踪视频、外部 PX4 源码或
ULog 不在此次 Git 归档范围内，本轮也未操作它们。

- [470 个保留本机文件的逐文件清单](audit/submission_cleanup_20261012/excluded_files.tsv)
- [整理前快照](audit/submission_cleanup_20261012/snapshot_before.json)
- [独立恢复与归档核验](audit/submission_cleanup_20261012/archive_verification.json)
- [暂存内容与文件保护核验](audit/submission_cleanup_20261012/verification.json)

本机恢复引用为 `refs/archive/submission-cleanup-20261012-4319834`，不会由
普通 `git push github master` 推送。持久 bundle 是独立备份；该本地引用
仅提供快捷定位。

归档独立恢复后，4,686 个跟踪文件、6,048,342,913 字节逐文件 SHA256
均通过；bundle 的 SHA256 为
`f8b85f01e4db1757594093533803484438d1a001db6365898c0cf9d8b30f4e1d`。
四包构建和四个启动/构建脚本的 shell 语法检查通过，`ros2 pkg prefix
uav_control` 指向当前工作区。

定向测试保留完整两轮记录：并行归档时为 83 通过、1 个有效性验收失败
（213.23 s）；归档结束后失败用例单独复核为 1 通过（0.18 s），
全部 P4.7 定向测试独立复跑为 84 通过（0.99 s）。执行条件变化与
复跑差异支持负载影响时效预算的判断，未修改源码、测试或任何门限。
见 [首次测试](audit/submission_cleanup_20261012/pytest_p47_with_archive.txt)、
[单用例复核](audit/submission_cleanup_20261012/pytest_failed_isolated.txt)、
[独立定向测试](audit/submission_cleanup_20261012/pytest_p47_isolated.txt) 和
[构建日志](audit/submission_cleanup_20261012/build.txt)。

## 在独立目录恢复

先校验归档，再恢复到新建临时目录，避免覆盖当前工作区：

```bash
cd /home/qin/data/uav_usv_mpc_archive/20261012_submission_cleanup_4319834
sha256sum -c SHA256SUMS
RECOVERY_WS=$(mktemp -d /tmp/uav_usv_submission_restore.XXXXXX)
git clone --bare repository_before.bundle "$RECOVERY_WS/repository.git"
git --git-dir="$RECOVERY_WS/repository.git" archive \
  43198343a2260392749e979bbc3beae6ce6f20c4 \
  data/experiments/20261010_p47_follow_guided \
  | tar -x -C "$RECOVERY_WS"
```

恢复文件与 `original_files.tsv` 的 SHA256 对照。P4.7 每个证据包的
`SHA256SUMS` 按所属目录校验；原交付指纹对应整理前的交付状态，
本轮不重写这些指纹。当前工作区的实验数据仍在原位置，可直接分析。

## 改写提交边界

原 P4.7 任务明确要求“不自动 commit、push”。因此本轮先提供归档、
暂存结果和核验记录，等待单独授权后才改写本地提交；没有自动提交或推送。

**必须替换未推送的 `4319834`，不能在它之后再追加一个删除提交。**
否则该提交引入的大 blob 仍属于推送历史。确认期间如果出现新的提交、
暂存变化或远端更新，须先重新核对，不能直接沿用本轮改写边界。

在 HEAD、远端和暂存内容复核无变化并获授权后，使用
`git commit --amend --no-edit` 替换这个本地提交。新提交的父提交仍是
`741000c`，可普通快进推送，无需强制推送。此文及核验 JSON 记录的是
准备完成时的状态，改写与推送结果须由后续核验单独记录。
