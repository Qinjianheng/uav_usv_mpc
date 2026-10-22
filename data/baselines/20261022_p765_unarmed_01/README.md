# P765：唯一授权未解锁采集

授权来源：2026-10-22本对话用户回复“授权”，范围为一次30s未解锁ROI/时序诊断。执行HEAD `f5c8338`，实际一次、无解锁/起飞/A1–D、无重试。源码/配置/模型未变；本包记录数据与离线分析，不宣称生产算法已修复。

中文完整报告：[授权采集与根因](../../../docs/current/p765_authorized_unarmed_capture.md)。主要结论：采集完整保存10组ROI，但真实有效3D为0；本次mask包含263球面点＋5远背景有效点，原fit半径10.864m被正确拒绝。固定已知直径分层后的反事实fit为0.248334m，只是离线验证。十帧像素相同，不能外推动态可靠性或协方差资格。实际200Hz影子丢周期567，E1仍FAIL，A1仍NO-GO。

| 文件 | 作用 |
|---|---|
| raw_session.tar.gz | 原始会话全部63文件（排除另存的source_snapshot）；含typed原始packets/callback、run绑定timing/graph/phase、ROI原始float32/mask/RGB/metadata、清理、控制台与ROS日志 |
| raw_files.json | 逐原始文件长度/SHA256；已逐项从归档读取复核 |
| executed_sources.tar.gz、executed_source_identity.json | 启动时实际快照中的206个相关源码/launch/msg/合同/profile及入口；逐项与快照SHA复核，HEAD f5c8338 |
| analysis/capture_analysis.json、analyze_capture.py | 两轮检测/时序对比、原始像素根因、原阈值反事实、同run与E1独立复算；纯离线，不启动ROS/PX4/Gazebo |
| analysis/roi_replay.json | 原始mask与严格fit原样重放：仍全部拒绝 |
| analysis/roi_background_evidence.png、plot_roi.py | 五个蓝色背景像素与原始远深度的直接可视证据 |
| analysis/model_identity_proof.json、model_manifest.json | 实际/tmp模型27文件完整校验，与历史冻结模型无差异；完整字节复用旧P764历史模型归档，避免重复9MB |
| validation_summary.json | 采集完成与E1失败分开；无控制publisher、未解锁、清理及最终预检 |
| authorized_capture_20261022.json、authorized_preflight_20261022.json、one_unarmed_attempt_claim.json | 用户授权范围、实际预检、永久保留单次claim；不代表后续飞行预算 |

归档保留原字节、文件模式和相对结构；未格式化或改写原失败输出。不要执行归档里的会话/授权脚本；它们只供审计。仅用于离线读取时：

```bash
mkdir -p /tmp/p765-unarmed-replay
tar -xzf raw_session.tar.gz -C /tmp/p765-unarmed-replay
```

原始目录在仓库 `data/experiments/p765_offline_validation/nonflight_01/`，分析在其同级 `nonflight_01_analysis/`。分析工具的常量路径保留实际执行上下文；恢复至其他目录须显式调整其副本路径。ROS/PX4均未在线调用，函数导入需先source原ROS/工作区环境。`executed_sources.tar.gz`用于读取精确运行版本，不能把当前HEAD或文件修改后的副本冒充当时进程；实际detector/localizer发布的loaded-code身份在ROI元数据及typed原始流中另存。

模型完整恢复入口：`../20261021_p764_validation_01/historical_E0_models.tar.gz`。27个实际模型文件与该历史源逐一匹配；本包model_manifest字节SHA与原p76_session_model绑定一致。旧P762/P764/P765离线包未修改。

本轮相同场景/模型，但新增ROI/资源/身份记录和更多二维有效更新，负载不同：454/504有效检测不意味着推理加速；567丢周期是共进程诊断执行器的失败，不授予或否定未经本轮实测的生产独立Tracker性能。影子Y不是实际Y飞行。没有去背景在线修复、没有新可信3D协方差，没有捕获验证。

下一项工作应先离线设计并验证mask-depth前景关联，保留原门槛、原mask及原统计，歧义/低支持/不可观测继续拒绝；再处理诊断高速影子与DKF的进程隔离。单次采集预算已用完，任何下一真实会话须另行授权。
