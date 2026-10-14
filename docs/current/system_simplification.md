# 系统精简、仿真复验与恢复（2026-10-14）

本次是保功能维护，基点master/2467c8a；前一任务的P53修复及未提交文件保留。前两次清理累计移出约944MiB旧文件并保存归档；最新用户明确“需要清理的文件无需归档，直接删除”，本轮据此直接删除其余旧日志，不新增归档。当前控制、感知、预测、规划、测试、launch/配置、模型和PX4补丁保持字节一致；不新增算法或改变接口。

新F2首轮出现一次预测时效回退；同配置诊断F2及条件F3均通过预先固定的比较标准。不能把源码等价或一次复测通过写成持续接管稳定性已彻底解决。

## 最新：仅保留最近有效日志

已直接删除56301个旧文件、3839个空目录，文件逻辑大小12036033710字节（12.04GB），范围为其余旧`data/experiments/`和5个非latest构建/列表日志目录。此次没有创建tar、外部副本或迁移符号链接；之前已存在的归档不动。

完整保留五个真实场景：R11成功F2/F3、清理后首F2失败、同配置诊断F2与条件F3；相应分析、资格绑定驱动、源码快照及SHA保持。保留62条旧轮次声明/摘要等小文件，防止重开已消费实验；旧目录中仅剩这些声明时，不代表仍保留完整旧日志。`data/experiments/current/`作为新实验输出目录为空。

两类旧数据属于当前运行依赖而非可删日志：H1.6 formal数学资格数据约2.33GB（227个manifest成员及原SHA256SUMS），以及P41原始F2/F3的两份`config/flight.yaml`。P5/P53准备入口实际读取它们；删除后会阻断新仿真。两份配置按原字节保留，未从evaluator快照补造。删除前后实际调用prepare，完整F2/F3飞行与研究参数均与R11语义一致；当前资格及formal manifest校验通过。

源码、配置、模型、冻结基线及近期证据共8469个保护文件保持；原仓库、Git历史、`data/videos/`和功能迁移patch保留。当前工作区约5.82GB，其中Git对象库约2.59GB、H1.6资格数据约2.33GB，是主要剩余占用。

本轮逐文件删除清单、保护清单与操作记录位于`data/experiments/20261014_recent_logs_cleanup_01/`；轻量验证入口见[最新清理审计](audit/recent_logs_cleanup_20261014/)。此次不提供新的旧数据恢复包。依赖已删原始输入的历史离线消融/分析不能保证原样重跑；新开发需采集新输入并重新资格审查。下面两个归档的恢复命令只覆盖前两次已归档项目，不覆盖本轮直接删除内容。

删除后全量pytest为1848 passed、1 skipped，flake8、shell语法和Git空白检查通过，ROS包前缀仍为本工作区。此轮不重复启动SITL或重新构建未改动的源码；保留上一轮真实F2/F3实测及首失败，并检查源码等价、完整准备参数与运行依赖。原有预测时效可靠性风险未因日志删除而修复。

## 已精简

| 项目 | 精简前 | 精简后 |
|---|---:|---:|
| scripts顶层工具 | 90 | 76 |
| 当前文档导航 | 55项 | 10项 |
| 旧构建日志目录 | 193个旧目录 | 已外置归档，latest目标保留 |
| P53 R1～R10旧场景日志 | 17个完整场景目录 | 已外置归档；attempt/summary保留 |
| 无当前引用的已完成计划 | 5份 | 已外置归档 |
| 日常轻量启动图 | 原完整图14节点 | 现有开关解析为11节点 |

归档移出14个旧阶段CLI，共1999行/90058字节；均无保留程序、测试、launch或安装入口引用。包括旧P31/P32分析、P43/P44/P45批量实验入口与早期视觉/PX4离线审计。逐文件SHA与去向见[retention_manifest.tsv](audit/system_cleanup_20261014/retention_manifest.tsv)。

另外移出193个旧colcon日志顶层目录、1456个日志文件，逻辑大小307357625字节（约293MiB）。包括其432个完整目录成员及权限，空目录也可恢复。新构建会正常产生少量新日志，不宣称log目录永久为空。

第二次清理移出21098个文件、1148个目录成员、682893799字节（约651MiB），包含17个P53旧场景和5份计划。逐文件SHA/大小/权限及完整目录清单位于第二份归档目录；[精简结果](audit/system_cleanup_flight_20261014/applied_cleanup.json)记录实际移出数量。R1～R10原有41条attempt/summary等声明保持原字节，避免重开已消费轮次；R11完整成功对照、P52回放及H1.6资格原始证据保留。

P4/P5公共研究runner、P47主候选/P44后备、共享MINCO/完整验收、未来PID/MPC接口、通用采集/性能工具、当前P51/P52/P53回放与全部测试仍保留。没有按阶段编号或当前尚未调用删除有开发用途的模块。历史报告正文及旧资格包未移动；当前README只保留当前事实，阶段入口统一到[研究历史](../history.md)。

## 日常运行

```bash
cd /home/qin/data/uav_usv_mpc
source /opt/ros/humble/setup.bash
source install/setup.bash
export UAV_USV_WS=/home/qin/data/uav_usv_mpc
UAV_USV_ENABLE_SHADOW_PERCEPTION=false ./scripts/uav_lab.sh --no-build
```

使用已有开关关闭`shadow_target_predictor_node`、`front_tof_monitor`、`dual_tof_selector`；主RGB-D、bearing、KF、BCTRA、MINCO、Tracker、任务与评价保留。需完整诊断时改为`true`。启动器默认仍为`true`，本次没有改变运行配置。14/11是源码launch条件解析结果；下面的新SITL沿用R11的研究配置，不用于宣称该日常开关的CPU收益。

普通baseline与P53 R11实验配置不同；精确复现实验按[最新验收](p53_comprehensive_validation.md)读取该轮完整config。未来MPC开发位置见[架构](architecture.md)和[范围](mpc_scope.md)。

## 可恢复归档

完整归档：`/home/qin/data/uav_usv_mpc_archive/20261014_system_cleanup_01/before_cleanup_complete.tar.gz`。
SHA256：`6513ea1ae835e43cf5bd30f8df291b395d092774848c4e385ca0be577cb54774`。
1474个原始文件和432个日志目录已全部冷恢复，逐文件内容、大小、文件/目录权限一致。外部`archive_members.json`、`directory_metadata.json`记录完整成员；最初文件版归档也保留，但恢复使用complete版本。

恢复单个历史工具到原路径（拒绝覆盖已有文件，保留原权限）：

```bash
ARCHIVE_DIR=/home/qin/data/uav_usv_mpc_archive/20261014_system_cleanup_01
(cd "$ARCHIVE_DIR" && sha256sum -c COMPLETE_SHA256SUMS)
tar --same-permissions --keep-old-files -xzf "$ARCHIVE_DIR/before_cleanup_complete.tar.gz" \
  --strip-components=1 -C /home/qin/data/uav_usv_mpc \
  repository/scripts/p31_minco_audit.py
```

按清单替换最后的原始路径即可。原脚本在原路径恢复后，仍须提供对应阶段的真实输入；归档不补造数据。只查看时可改为新的`mktemp -d`恢复目录；有基于`__file__`寻找工作区的脚本须恢复至完整布局再运行。

第二份完整归档：`/home/qin/data/uav_usv_mpc_archive/20261014_system_cleanup_02/historical_logs_and_plans.tar.gz`。
SHA256：`93f582ebc1039031d09a03555b941dd39d5737bdcfc3551551f1149481cbd941`；压缩179171480字节。
21098个文件和1148个目录成员均已冷恢复核验内容、大小和权限；另经独立流式逐成员复核。[归档核验](audit/system_cleanup_flight_20261014/archive_validation.json)保存结论。

恢复一个旧场景：

```bash
ARCHIVE_DIR=/home/qin/data/uav_usv_mpc_archive/20261014_system_cleanup_02
(cd "$ARCHIVE_DIR" && sha256sum -c SHA256SUMS)
tar --same-permissions --keep-old-files -xzf "$ARCHIVE_DIR/historical_logs_and_plans.tar.gz" \
  --strip-components=1 -C /home/qin/data/uav_usv_mpc \
  repository/data/experiments/20261013_p53_closed_loop_10/f2_h16_01
```

原始路径列表见外部`archive_roots.json`；逐文件SHA/大小/权限见`archive_members.json`，目录权限见`archive_directories.json`。计划同样按`repository/docs/superpowers/plans/<文件名>`选择性恢复。旧root的SHA256SUMS仍引用已迁移场景，应恢复对应内容后再完整校验，不改写旧清单。`kept_root_metadata.tar.gz`是仍保留在工作区的轮次声明额外备份，不用于覆盖或重置轮次。

归档包含清理前快照和工作树补丁，供审计或单独恢复；勿整包覆盖当前目录。视频、冻结基线、Git历史、build/install、原仓库未移动。新构建和仿真会产生必要日志，累计移出量不等于含新验证产物的净磁盘节省。

## 功能保护与验证

首次清理保存6685个保护文件SHA，移出旧CLI/构建日志后全部一致；第二次明确将R1～R10旧日志归档，留下3901个当前程序/基线/R11保护文件和41条旧轮次声明，移出后全部一致。R11快照中的599个src/patches文件与当前源码一致，两份新生成的飞行/研究YAML与R11语义一致。清单和独立重复驱动经只读复核，无Critical/Important。

第二次清理前后全量回归均为1848 passed、1 skipped；四包构建、flake8、所有shell语法、Git空白检查通过。ROS包前缀仍为`/home/qin/data/uav_usv_mpc/install/uav_control`。首次证据见[audit目录](audit/system_cleanup_20261014/)及[source_equivalence.tsv](audit/system_cleanup_20261014/source_equivalence.tsv)；本次测试/构建及新源码资格见[新审计](audit/system_cleanup_flight_20261014/)。

不修改旧P53 readiness：其指纹包含README和已归档脚本，整理后按设计失效，仍是历史运行证据。本次新实验针对当前源码和独立驱动生成新资格；一次诊断复测使用[单独资格](audit/system_cleanup_repeat_20261014/)，不把旧资格追认为整理后的当前资格。

## 新实际仿真与限制

比较标准在首轮启动前固定：F2/F3分别取首次MINCO后完整25/60s、包含全部owner；MINCO占比≥95%，最长连续≥20/55s、零回退/违规、全目标FOV100%；水平RMSE相对R11最多增加0.05/0.10m，高度RMSE≤0.020/0.025m，参考跟踪误差增加≤0.03/0.06m。没有放宽在线时效、安全门禁或捕获半径。真值只作离线评价。

| 实际场景 | MINCO占比 | 最长连续 | 水平RMSE | 高度RMSE | 参考跟踪3D RMSE | FOV | 回退 |
|---|---:|---:|---:|---:|---:|---:|---:|
| R11 F2对照 | 100% | 24.950s | 0.0546m | 0.0090m | 0.0283m | 100% | 0 |
| 清理后首F2 | 67.19% | 11.213s | 0.0464m | 0.0081m | 0.0249m | 100% | 1 |
| 同配置诊断F2 | 100% | 24.950s | 0.0520m | 0.0096m | 0.0242m | 100% | 0 |
| R11 F3对照 | 100% | 59.950s | 0.6977m | 0.0116m | 0.2310m | 100% | 0 |
| 清理后条件F3 | 100% | 59.952s | 0.7735m | 0.0121m | 0.2363m | 100% | 0 |

条件F3水平RMSE比R11增加0.0758m，在运行前固定的0.10m比较范围内，不能写成误差完全相同。F2复测、条件F3的诊断覆盖均100%、审计违规0，发现后的控制发布者均为唯一Tracker。完整窗口native采样分别1250/3000条，参考跟踪误差分别使用431/1143条同epoch可用参考样本；两者分母不同。F3实测水平速度峰值4.535m/s、水平加速度峰值1.695m/s²，旁路记录的1482条原生setpoint发布消息均有有限水平加速度前馈，峰值1.752m/s²。这不等同于1482条PX4内部接收确认，也不是推力或任意场景安全证明。

首轮F2唯一撤销为plan31的`PREDICTION_STALE`。两次有效观测epoch相隔100.015ms；撤销时Tracker持有prediction738，旧观测年龄134.659ms，超过原125ms期限9.659ms。新观测已到KF，但新预测尚未进入该次Tracker执行快照。`generated_stamp`是计算开始时间，不能据此推断已发布或可消费；没有全量Tracker出队或传感器发布记录，不能进一步断言DDS、回调竞争或哪一段丢帧。原FOLLOW安全接替，8.19s后重新准入MINCO。

已确认源码/参数一致，未观察到缺文件/import错误或对已归档文件的在线依赖；这支持临界输入时序失败的定位，不等于证明全部调度根因。首失败完整封存，复测按预先固定上限只增加一次F2，条件F3必须先通过原门禁和全部比较；不能反复运行筛选成功。即使条件F3通过，也保留该接管连续性风险，不宣称“各种情况下效果不变”。

新原始证据为`data/experiments/20261014_cleanup_sitl_01/`及`20261014_cleanup_sitl_02/`；分析、诊断、预定比较与F3准入记录位于`data/experiments/20261014_cleanup_validation_01/`。实际误差采集时刻对齐，参考误差只统计存在MINCO参考的样本；不把独立实际运行视为逐样本确定性对照。

最终轻量结果与恢复清单见[结果审计](audit/system_cleanup_results_20261014/)，其中[完整比较](audit/system_cleanup_results_20261014/comparison_actual.json)保留首失败、复测和F3，未筛掉失败轮。三个新场景完整原始文件均封SHA，任务拥有的仿真进程已正常清理，未留下survivor。当前源码/参数未因维护改变；仍需另立算法任务解决临界预测时序与恢复耗时，才能提高持续接管的重复可靠性。

本次没有commit/push、分支切换、Git历史重写、视频暂存或原仓库写入。
