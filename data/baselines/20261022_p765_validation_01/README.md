# P765 离线修复与 NO-GO 证据包

基线 `33a267f2c05ea087422a254cc385e8b15ba2edd5`，分支 `codex/p76-paper-faithful-yolo-dkf`。本轮无新 Gazebo/PX4、解锁或 A1–D。E0 BLOCKED、E1/E2 FAIL、A1 NO-GO；不是飞行/非合作 USV 验证。

完整中文分析：[p765_remaining_blockers.md](../../../docs/current/p765_remaining_blockers.md)。`file_origins.json` 保留所选文件的原始绝对来源和哈希。JSON 内部原始路径不改写；本包副本按下表对应。`SHA256SUMS` 覆盖整个包除自身，原 P762/P764 校验均通过。

| 路径 | 内容 |
|---|---|
| geometry/analysis.json、histlineage.jsonl | 362拒绝重算；140严格几何失败均精确匹配RGB/depth/detection；全部362对应DKF receive。无旧像素，不虚构子因 |
| geometry/synthetic_roi/、synthetic_capture_replay.json | 有界float32采集/重放：2已知球接受、平面拒绝，仅合成工具验证 |
| dynamics/dynamics_analysis.json | 七档77瞬态/49热身后样本、五协议、条件拟合；240ms仅模式退出，最大力/同接口响应/制动仍未知 |
| dynamics/e2_analysis.json | 保留500ms末误差0.658108m；角速度链主导；局部连续tau<0.4375s，400ms最大已测试域内PASS；不宣称实际稳定范围 |
| qualification/ | 当前A/B缺失、C/D未运行；旧3/5计数不变；原始回调重算，无非法授权、Y后有效三维0 |
| timing/historical_timing.json | 旧545次实际推理、阶段误标签分解，原157丢周期未改写；未测新性能改善 |
| preflight_plan.json | 唯一推荐后续30s未解锁ROI/时序方案及只读预检；尚无授权/会话 |
| config/ | 本轮不变的完整P76配置/合同副本 |
| checks/ | 所有选入的红/绿测试与中间失败、独立审查、完整回归原结果、构建、shell/lint、source指纹 |

完整可用回归**仅一次**：2568 passed、2 failed(style)、2 skipped、3 warnings，47.05s。随后仅修改两处测试格式/文档字符串，相关及lint测试33 passed、2 warnings，5.16s；运行源码字节相同，不重新跑全套、不伪造全绿日志。缺失历史R11输入的两个测试模块显式排除，不计通过。`source_frozen.json` 对应完整回归，`source_final.json` 列明仅两个测试文件的差异。

两包构建通过（2min1s），prefix为 `/home/qin/data/uav_usv_mpc/install/uav_control`。最终flake8/shell/diff通过。默认P7入口测试在功能回归中；Y公式、增益、合同、PX4补丁未修改。未合并、未push。

复算需先source ROS和本工作区，从 `src/uav_control` 运行pytest。历史动力学输入仍保存在原P761实验目录，其原始压缩副本和恢复索引见 `../20261021_p762_calibration_01`；历史模型完整包见 `../20261021_p764_validation_01/historical_E0_models.tar.gz`。本包不重复大体量原始数据，也不重写旧包。几何逐帧输入是旧P764冻结的 `nonflight_01/synchronized_packets.jsonl.gz` 与 `actual_callback_inputs.jsonl.gz`。

后续严格限定一次新未解锁采集：先经用户单独授权，再用 `scripts/p765_nonflight_session.py` 执行。默认只预检；执行有持久单次claim，无自动重试。新增ROI/资源监测改变负载，不能将之后改善单独归因于EOF或worker；报告必须声明条件差异。原始帧/epoch/receipt/实际detector代码身份、正常结束和数据哈希均核验，失败保持NO-GO。

源码提交：`3cc13bb`。`source_identity.json` 记录当前完整功能源码SHA；最终预检与该指纹一致。

控制台原始输出以 `.txt.gz` / `.log.gz` 无损保存，避免pytest原始尾空白影响Git检查；`raw_log_identity.json`逐文件绑定原始SHA/长度。可用 `gzip -dc checks/full_regression_frozen.txt.gz` 查看，未修改原始文本。
