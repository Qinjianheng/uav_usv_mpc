# MPC 前冻结基线（2026-10-06）

本包保存失败对照 `093918_367847` 和连续成功 `095151_101120`、`095428_701507`、`095732_501270` 四轮实验。主目录共有 16 个原始文件：每轮 CSV、vision CSV、evaluator config YAML 与 summary JSON。

| 子目录/文件 | 内容 |
|---|---|
| `config/` | 成功版本完整 baseline 与视觉几何配置；与成功指纹一致 |
| 原仓库/归档 | 三次成功原始 JSONL 和历史工具，按 manifest 的 external 条目恢复 |
| `evidence/` | 原成功报告、重复验证、速度/冻结 JSON/PNG、日志与源码指纹 |
| [manifest.json](manifest.json) | 46 个原始文件/配置及 6 个外部录制/工具的源路径、目的地、SHA256 与大小 |
| [SHA256SUMS](SHA256SUMS) | 原始数据逐文件校验清单 |

所有证据来自原仓库 `85e13461df62b65d34aa55c9d92576c58085d0b3`，复制前后字节一致。失败对照属于修复前版本；包中完整配置及成功源码指纹不能被解释成失败轮完整配置。每轮 `*_config.yaml` 只记录 evaluator 参数。

原始报告的相对链接、JSON 里的源路径和工具的硬编码路径保留历史上下文。新入口见 [当前基线说明](../../../docs/current/baseline.md) 与 [分析/恢复步骤](../../../docs/current/analysis_and_recovery.md)。

```bash
cd /home/qin/data/uav_usv_mpc/data/baselines/20261006_pre_mpc
sha256sum -c SHA256SUMS
```

不要在本包改写原始文件或重新生成报告/图；派生分析写入独立输出目录。
