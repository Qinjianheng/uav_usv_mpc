# P4.5 两项失败的 P4.6 回归审计

本子任务只修改 `src/uav_control/test/test_follow_revalidation.py`，未修改生产重验代码、在线预算、模型/控制约束或冻结证据，未启动 SITL，未提交或推送。

## 原日志与当前事实

- 原始 `20261010_p45_minco_execution/full_initial.log`：1428 passed、1 skipped、2 failed。两项失败为 tracker 第 341 行 E501，以及 behind-camera 几何拒绝断言收到 `REVALIDATION_DEADLINE`。
- 当前 HEAD 已把 tracker 该行换行；本子任务未改 tracker。配置了仓库 99 字符规范后的局部 flake8 通过。
- 当前首次复跑原相关测试：5 passed、test_flake8 failed；当时 7 个 lint 错误均在主任务正在修改的 direct_reference_minco.py，已报告给主任务处理。本子任务未修改该文件。

## 根因与修复

`revalidate_prediction` 默认用真实 perf_counter 测量 30 ms 截止时间。几何/因果单测未注入时钟，因此主机调度或计算负载可先触发正常的截止拒绝，覆盖它想断言的几何拒绝原因；生产代码的失败关闭行为正确。

诊断脚本保留实际几何计算，只在每次 assess 后补足真实 40 ms 耗时。修复前，同几何与 behind-camera 两项均因 REVALIDATION_DEADLINE 失败；修复后，同一脚本返回正确的几何验收/拒绝并通过。

所有几何/因果单测显式注入恒定测量时钟。另增加默认 30 ms 截止时间的独立测试，覆盖同几何与 behind-camera，并分别覆盖恰好 30 ms 与 31 ms：必须拒绝且返回原请求，不能刷新预测/导航上下文。没有增大生产预算。

## 实跑验证

- `semantic_stall_before.log`：两项预期复现失败，elapsed 约 41 ms，reason=REVALIDATION_DEADLINE。
- `semantic_stall_after.log`：同几何有效、behind-camera 原因=PREDICTION_REVALIDATION_FAILED；两项通过。
- `related_after.log`：test_follow_revalidation.py、test_follow_fast_validation.py、test_p4_receiver_node.py，27 passed（0.69 s）。
- `scoped_flake8_configured.log`：从仓库根目录读取 .flake8 的 99 字符规范，修改测试与原失败 tracker 文件通过。
- `git diff --check -- src/uav_control/test/test_follow_revalidation.py`：通过。
- ROS 包前缀实查：/home/qin/data/uav_usv_mpc/install/uav_control。
- `fingerprints.json`：旧 full_initial.log 与生产 follow_revalidation.py 均逐字节匹配 HEAD。

一次局部 flake8 从包目录误用了默认 79 字符，输出保存在 scoped_flake8.log；已从仓库根目录使用现存 .flake8 重新验证，没有据此格式化文件。

按主任务协调，算法修改期间不运行完整回归；完整 pytest/项目 flake8/构建由主任务在整合后统一运行，本子任务不宣称全量通过。
