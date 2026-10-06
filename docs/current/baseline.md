# 2026-10-06 MPC 前基线

冻结数据包：[data/baselines/20261006_pre_mpc](../../data/baselines/20261006_pre_mpc/README.md)。来源为原仓库提交 `85e13461df62b65d34aa55c9d92576c58085d0b3`；复制后逐文件 SHA256 与整理前相同。

| 运行 ID | 角色 | summary 结果 | 最小三维距离 m | recovery_count |
|---|---|---|---:|---:|
| 20261006_093918_367847 | 修复前失败对照 | FAILURE / TIMEOUT | 0.557574 | 6 |
| 20261006_095151_101120 | 修复后首次成功 | SUCCESS | 0.464702 | 0 |
| 20261006_095428_701507 | 成功重复 1 | SUCCESS | 0.482564 | 0 |
| 20261006_095732_501270 | 成功重复 2 | SUCCESS | 0.461706 | 0 |

三次成功均首次末端进近命中、捕获前无恢复，自动暂停与 UAV/目标共同冻结核对通过。原验证报告的结果事件 elapsed_time 为 4.413230、4.622916、4.484602 s；summary 的 `intercept_elapsed_time` 使用日志生命周期时刻，数值略有不同，不应混用。

现有 tracker 指令上限为 6.5 m/s，三轮捕获前最后收到的实测水平速度为 6.803260、6.783259、6.828834 m/s。PX4 存在跟踪超调；这些值是物理 sample epoch 下的最后导航样本，不是精确碰撞瞬间速度。最后 0.7 s 的 14 个样本中实测水平速度持续增加。原始速度曲线、JSON 和冻结证据保留于数据包；大体量记录 JSONL 留在原仓库/归档，可恢复重放。

这三次属于同一目标参数和任务入口的连续验证，不推断任意运动或真实无标记 USV 的总体捕获率。失败对照发生在末端修复前，不能将成功版本指纹误标为失败轮完整代码指纹。

每轮保存四类文件：主 CSV、vision CSV、`*_config.yaml`、`*_summary.json`。**每轮 config 只有 evaluator 设置**；完整成功版本配置另存为 [config/baseline.yaml](../../data/baselines/20261006_pre_mpc/config/baseline.yaml) 与 [visual_geometry_diagnostics.yaml](../../data/baselines/20261006_pre_mpc/config/visual_geometry_diagnostics.yaml)。这些完整配置与成功验证指纹一致；失败轮不补造未记录的完整参数。

[原始成功报告](../../data/baselines/20261006_pre_mpc/evidence/terminal_cruise_validation_20261006.md)、[重复验证 JSON](../../data/baselines/20261006_pre_mpc/evidence/terminal_cruise_repeat_validation.json)、[代码指纹](../../data/baselines/20261006_pre_mpc/evidence/terminal_cruise_code_fingerprint.json) 按字节保存。报告和 JSON 中旧路径反映原始证据位置，恢复方法及路径映射见 [分析与恢复](analysis_and_recovery.md)。原报告已从工作区移到证据包，因此其历史相对链接要在原仓库目录上下文解读；当前文档中的链接以新工作区为准。

复核命令与实跑结果见 [整理交付](repository_cleanup.md)。
