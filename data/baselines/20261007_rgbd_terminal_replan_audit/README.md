# RGB-D 末端重新验收证据包

2026-10-07：默认前视 ideal RGB-D、下视关闭、升沉幅值 0.15 m / 0.25 Hz。MPC 尚未实现。用户授权在新鲜感知下重新验收末端轨迹，感知失效不能延长原轨迹。最终代码在两轮独立全仿真中均成功捕获；按用户要求首轮成功后仅追加一轮。

详见 [当前修复报告](../../../docs/current/terminal_replan_rgbd_20261007.md)。本包与旧冻结基线分开；所有 CSV、evaluator YAML 和 summary JSON 按字节复制，未补造缺失配置。大体积采集 JSONL 使用无损 gzip，原始未压缩 SHA256 另存 provenance.json，采集原件仍在本工作区忽略的实验目录。

| 目录 | 内容 / 结果 |
|---|---|
| user_observed_before/ | 用户记录 114222、114314、114512：成功、TIMEOUT 失败、成功；12 个原始文件 |
| user_before_audit.json | 按一次性结果截断失败记录后的归因，避免混入失败后的再次接近 |
| before_run1/ | 修改前补录，FOLLOW 3 s 后 Y，SUCCESS / 10.697 s / 0.441 m |
| before_run2/ | 修改前补录，FOLLOW 5 s 后 Y，SUCCESS / 9.022 s / 0.459 m |
| after_run1/ | 151119，最终代码，SUCCESS / 10.448 s / 0.460 m；实际 committed 接替路径 |
| after_run2/ | 151710，独立重复，SUCCESS / 10.876 s / 0.404 m；继续原接纳轨迹路径 |
| source_before/ | 修改前代码副本，含用户已改为 ideal 的启动脚本 |
| source_after/ | 两轮修复后共同的完整选定源码；每轮 source_fingerprint.json 可核对 |
| config/ | 最终 baseline、几何诊断及保留的 ToF 完整配置 |
| checks/ | pytest、flake8、构建、shell、基线保护及代码审查输出 |
| tools/ | 只读原始输入的速度、捕获、冻结和接替复算工具 |
| provenance.json | 每个复制输入的来源、SHA256、压缩说明及仓库 HEAD |
| SHA256SUMS | 本包逐文件校验清单；不包含清单自身 |

after_run1 的最终源码副本在捕获后补齐；生产修改全部早于该轮启动，之后未修改，其 snapshot_note.txt 明确记录时机。after_run2 的副本在启动前保存。HEAD eb9c092 不包含本轮未提交修改，以 source_after 和指纹表示实际验证版本。

复算输出放在独立实验目录，避免改写本包：

```bash
cd /home/qin/data/uav_usv_mpc
sha256sum -c data/baselines/20261007_rgbd_terminal_replan_audit/SHA256SUMS
python3 data/baselines/20261007_rgbd_terminal_replan_audit/tools/analyze_run.py \
  data/baselines/20261007_rgbd_terminal_replan_audit/after_run1 \
  data/experiments/rgbd_terminal_reanalysis/after_run1
python3 data/baselines/20261007_rgbd_terminal_replan_audit/tools/audit_replans.py \
  data/baselines/20261007_rgbd_terminal_replan_audit/after_run1 \
  data/experiments/rgbd_terminal_reanalysis/after_run1
```

将 after_run1 改为 after_run2 可复算重复轮。工具直接读压缩 JSONL，不需恢复解压件。原生冻结解析使用本机已安装的 gz.msgs10；脚本不会把真值送入在线控制。

捕获距离来自一次性 InterceptResult，低频任务 CSV 的缓存距离不代表同一瞬间。红球只验证仿真感知接口；两轮结果不覆盖所有升沉相位或真实无标记 USV，保护规则仍可能触发减速退出。
