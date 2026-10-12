# P5 入口精简与 Git 审计

源码和测试引用图见 [dependencies.tsv](audit/p5_20261012/dependencies.tsv)；
engine 字面引用见 [engine_references.tsv](audit/p5_20261012/engine_references.tsv)。
这是静态 import/字面引用图；动态加载、ROS entry point 和 launch 也按入口人工核对，
“未发现静态引用”不能作为删除依据。

| 入口 | 处理 | 原因 |
|---|---|---|
| P47/C H1.6 | 纳入新 P5 默认配置 | 限定离线验收通过 |
| P44/D | 保留并纳入新 P5 切回列表 | 可靠对照与真实后备 |
| P46 FOLLOW-guided | 保留历史实现/测试/脚本；移出 P5 列表 | 历史复现和对照仍依赖 |
| legacy/P3/P4.1/P4.3 | 保留原入口；移出 P5 列表 | 研究工厂、旧测试/消融仍引用 |
| P47 额外 SLSQP/D、q/qt 等旧消融 | 保留复现入口；P5 默认关闭 | 无持续执行收益证据 |
| FOLLOW rollout、MINCO 映射、完整验收、BCTRA、Tracker、ACK/lease/隔离授权 | 全部保留 | 当前主链/后备/安全依赖 |

本轮实际删除算法文件数为 **0**；新 P5 列表缩为两个 engine。没有格式化或改写冻结历史工具、
旧配置、模型、PX4 补丁、预测器或 Tracker。当前工作树的旧功能文件保持原字节。

Git 审计见 [git_and_dependencies.json](audit/p5_20261012/git_and_dependencies.json)。
远端 main 是 master 祖先、独有提交 0；PR #1 仍以 main 为 base，默认分支仍为 main。
两远端均指向要求的仓库。仅刷新过期 origin/main；保留 main/master，
创建并 peel 校验三个本地恢复标签；使用新本地研究分支。没有默认分支切换、远端删除或 push。
本任务完成可授权范围内的非破坏性分支整理，保留有 PR 依赖的 main。

master 可达 blob 原始总大小约 **7.340GB**（包含历史版本，不是传输压缩量）；
当前已跟踪文件约 **3.022GB**，本机实验文件约 **14.126GB**，`.git` 磁盘约 **2.5GiB**。
4319834 引入的约 3.027GB 新实验虽在 a35e405 从当前树取消跟踪，仍在发布历史。
远端 master 已确认指向 a35e405，故这些历史对象仍由其可达；GitHub 实际物理占用无法从本地推算。
本轮精选证据约数 MB，GB JSONL 继续仅保留本机。不会因 `.gitignore` 而取消旧跟踪。

完整历史恢复包：
`/home/qin/data/uav_usv_mpc_archive/20261012_submission_cleanup_4319834/repository_before.bundle`。
SHA256 `f8b85f01e4db1757594093533803484438d1a001db6365898c0cf9d8b30f4e1d`；
上轮已做冷恢复 fsck 与逐文件校验，记录见 `submission_size_cleanup_20261012.md`。
本轮没有移走原始数据。若需压缩发布历史，须另行评估协作者、PR、标签、归档恢复和强推影响，
单独获得授权后开展，本任务不执行 filter-repo 或历史重写。

最终待审改动约2.7MB（以轻量摘要和匹配样本为主），新增40.7MB本机F2日志继续忽略；暂存区为空。原始source snapshot保留失败轮实际工具版本，修复后版本另见 `audit/p5_20261012/repair_source_fingerprints.json`，不能混称同一运行版本。
