# P7.6.5 三维感知离线结论

E1 继续 NO-GO。本轮没有启动 Gazebo/PX4，没有解锁或飞行，也没有取得新的真实有效 3D 测量或同步可用的真实 2D/3D 状态。合成数据通过不能授予真实感知或控制资格。

## 历史 362 条观测

最终可复算输出为 `final/analysis.json` 和 `final/histlineage.jsonl`。输入保持冻结，SHA256 为 `38d041d35c0e175d04ff8f999a75bb93af42d876358d8ac49ef561dacaaad0ec`。全部 362 条原始失败均保留；派生严格 JSON 仅将非有限数写作 null，原始 NaN 仍保留在冻结输入中。

- 140 条 `TARGET_VECTOR_INVALID`：各对应不同的原始 acquisition key，RGB/depth header 同时刻，全部找到该帧 detector RLE，尺寸与 640×480 图像一致。
- 121 条 detector timeout、92 条 inference 后过期、3 条 source stale：216 条在原实现刷新当前帧诊断前返回，raw 字段可能来自先前帧。历史数据明确标为 `UNPROVEN_EARLY_REJECTION_RAW_FIELDS`，不能按收包最近原则强行指定图像。
- 5 条 position history empty、1 条 position timestamp before history。
- 362/362 条 TargetObservation 通过完整 payload SHA256 对上 `actual_callback_inputs.jsonl.gz` 中的 `SpatialDkfNode.receive`；无需猜测接收顺序。历史有效 TargetObservation 为 0，有效 spatial output 为 0。

140 条几何失败的 detector box center 均为 `(319.2374831, 76.3182303)` pixel；实际 RLE mask 质心均为 `(322.0547945, 76.0547945)` pixel，292 个 mask pixel、18×18 mask bbox。该证据仅覆盖同一画面位置，且没有成功样本作比较，因此不能建立位置相关风险或距离相关结论。`target_range` 无有效测量，不能用配置目标距离或真值补造距离。

## 140 次具体为什么失败

只能定位到“严格球心几何没有返回可接受结果”，不能进一步确定是无效深度、背景/海面混入、曲率不足、数值条件、半径或残差失败。原始 RGB/depth 像素均未保存，失败消息中的深度点数/统计默认零不是测量到的零。合法 RLE 尺寸、相同 raw header 并不能证明像素级 RGB-depth 对准、mask 内纯净度或深度实现正确。

当前没有原始点云支持确定某个几何/深度/坐标数值错误并修复；不将此归咎于 YOLO 模型质量。历史 `executed_sources.tar.gz` 没有 core localizer/ToF 的运行时副本；另有收尾 source SHA，不能代替启动时实际加载代码身份。

## 本轮修复与诊断

已用连续两帧 timeout 的红绿测试证明并最小修复了诊断 raw 字段复用：每次 pair 处理开始即清空旧诊断并设置当前 RGB/depth 原始时间和收包时间。这只修复诊断归属，不改变有效测量或任何时效门槛。

ideal 与 ToF 拟合新增可选诊断字典：有效点数、深度比例、min/median/MAD（m）、fit mode、rank、condition、fitted radius（m）、radius residual（每点相对于指定半径的绝对径向误差 P95，m）、fit position std（m）与明确拒绝子原因。尚未计算或不可证明的字段为 null。ideal 路径没有新增协方差模型，其 fit position std 为 null；不能将默认零称作可信拟合协方差。ToF 保留原有 largest-axis std 与所有曲率/残差/条件数门槛。

`require_valid_sphere=True`、原有残差、半径、曲率、深度比例与 125 ms 门槛均保持。独立 ray/sphere intersections 验证 forward-X depth、FLU 左/上符号和 off-axis 已知球心。没有证据支持修改这些坐标公式。

## 有界采集与离线重放已准备

localizer 参数 `sphere_fit_diagnostics_enabled` 默认为 false；开启后向 `/diagnostics/p765_sphere_fit` 发布当前精确 raw sec/nanosec、原收包时间、geometry contract、失败详情，以及构造时实际加载函数的 code-object SHA 与源码路径/启动源码 SHA。`target_geometry` 返回 None 时同样发布详细原因。源身份同时包含 localizer、ToF、真实导入的 YOLO infer/decode、mask encode/decode。

`scripts/p765_roi_capture.py --output PATH --duration 30 --max-samples 60 --max-per-stratum 10 --max-bytes 67108864 --capacity 24` 是独立进程，只创建订阅，不创建任何 PX4 command publisher。它按 raw integer key 连接 RGB/depth/detection/fit，保留完整原始 RLE、ROI offset 和 1:1 坐标映射、源头/收包/诊断时间、原因、代码身份与 SHA256。不同原始 key 绝不互配。保存 float32 原始值和 byte order，包含 NaN/Inf 位模式；有 image row padding/大小端测试。压缩和写盘均不在 200 Hz 控制回调内。

上限按总样本、成功/各失败原因 strata、payload bytes、队列和事件数约束。未匹配和达到限额的事件保留失败元数据。`--max-bytes` 约束 ROI+sample JSON payload；事件/manifest 另由事件数和固定 source image extent 约束。当前 collector 保存的是完整 target mask bbox ROI，bbox 外像素未保存；重放不声称重建完整原始图像。

`scripts/p765_roi_replay.py --input CAPTURE_DIR --output NEW_JSON` 验证数据 SHA、mask 与 offset，先还原完整原始图像坐标，再按保存的 strict geometry contract 拟合，避免把 crop 宽高当作原相机内参。

已实际运行纯离线合成采集→保存→重放：两个已知球体有效（ideal、modeled ToF），一个已知平面继续拒绝，见 `synthetic_capture_replay.json`、`synthetic_cli_replay.json` 和 `synthetic_roi/SHA256SUMS`。未运行 live recorder。

## 验证和下一步

从 `src/uav_control` 执行：

```
python3 -m pytest -q test/test_p765_geometry.py test/test_p765_roi_capture.py test/test_tof_sphere_localization.py test/test_rgbd_target_localizer.py
```

最终结果 `126 passed in 2.90s`，日志 `final/directed_tests.txt`。上述变更范围 flake8、`git diff --check`、脚本/模块 py_compile 通过。各 TDD 失败及中间 lint 证据均保留。本子任务未运行完整回归或构建，由主线冻结后统一执行。

唯一建议的下一步是单独批准后，在与 P764 可比的同场景、同模型、未解锁会话内，执行一次最多 30 s 的有界 ROI/时间诊断；先冻结实际导入的源码与 runtime identity。没有深度像素复算和真正有效且协方差可信的 3D 测量前，E1 保持 NO-GO，不运行 A1 或飞行。

## 独立复核后的采集防伪补充

最终几何范围测试更新为 **138 passed in 2.90s**，见 `review_final_tests.txt`；`review_flake8_final.txt`、`review_final_diff_check.txt`、`review_final_compile.txt` 均通过，旧 126 项记录完整保留。

独立复核发现：在 localizer/recorder 进程导入 YOLO 函数只能说明本进程的代码，不能证明运行模型的 venv detector。现在 live ROI matcher 必须额外收到 `/diagnostics/p76_yolo` 上 detector 自己发出的 `runtime_identity`，按 exact `{sec,nanosec}` raw stamp、frame sequence 与 model SHA256 绑定；保存其 PID、executable、已加载 process/infer/decode/encode 代码身份、原 status 与收包时间。缺失/不符不保存。测试使用的远程身份明确标记 synthetic，只有显式 `allow_synthetic=True` 才接收；live CLI 没有该选项。

每个 raw key 只消费一次并删除对应缓存；已消费的重复输入只记录 duplicate 事件，不再保存。尚未消费的重复 raw key 可能表示时钟回绕后的像素替换，直接 FAIL CLOSED。各输入流内 raw key 回退、已观察 clock epoch 变化、detector/localizer epoch 不一致、原始收包→发布顺序不成立或 recorder pair 等待超过 0.5 s，均清空缓存并退出该采集会话，不自动恢复、不跨 epoch 互配。0.5 s 仅约束采集证据等待，不改变 125 ms 控制数据有效门槛。终态 `SESSION_END` 在所有未匹配失败元数据之后写入，作为最后事件。
