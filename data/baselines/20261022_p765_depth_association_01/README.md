# P765 前景深度关联：离线修复与合并验证

来源：`../20261022_p765_unarmed_01/raw_session.tar.gz`；新输出仅属于离线反事实分析，没有新仿真。源码提交 `cd3189a79b403e4dace98eac9086c0aa68f3e979`，最终仅追加 D213 文档字符串修正。详情见 [当前报告](../../../docs/current/p765_depth_association_merge.md)。

| 文件 | 含义 |
|---|---|
| `roi_comparison.json` | 最终源码的十份原合同/候选合同对比、输入哈希、完整诊断和实际 loaded-code 身份 |
| `roi_comparison_pre_review.json` | 审查前的对比输出；同一输入接受结果相同，工具曾未保留更严格支持率，不能作为最终合同实现 |
| `source_identity.json` | 完整回归前冻结的 7 个源码/配置/测试哈希，原基线和 master 位置 |
| `source_final.json` | 最终哈希；与完整回归仅差文档字符串，已检查去除文档字符串后的 AST 相同 |
| `checks/directed_pytest.txt` | 最终定向 158 passed |
| `checks/full_pytest.txt` | 一次完整可用回归原文：2583 passed、2 failed、2 skipped、3 warnings |
| `checks/focused_after_full.txt` | 两项失败处理后定向复验：33 passed、2 warnings |
| `checks/colcon*.txt`、`checks/ros_prefix.txt` | 构建及新工作区前缀 |
| `checks/previous_evidence_integrity.json` | 旧四包共 273 项 SHA 检查结果 |
| `checks/static_checks.txt`、`checks/diff_checks.json` | 静态与 diff 检查范围/原始输出；冻结字节不格式化 |
| `checks/contract_audit.json` | 参数合同有效；flight_qualified=false |
| `checks/verification_commands.json` | 验证命令、排除范围和原因 |

原始重放 0/10，候选 10/10，只有 1 组不同的 depth/mask payload。每帧原有效深度 268，拟合使用 263，排除远背景 5；球半径约 0.248334 m，P95 残差约 0.001947 m。不能据此授予新的在线三维、协方差或飞行资格。

重放命令（输出必须为新路径）：

```bash
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
python3 /home/qin/data/uav_usv_mpc/scripts/p765_depth_association_analysis.py --output /tmp/p765_new_roi_comparison.json
```

`SHA256SUMS` 从本目录核验。E0/E1/E2/A1 仍阻断；单次未解锁采集 claim 保留，没有自动重启、合成数据冒充实测或推送 GitHub。
