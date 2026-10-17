# 2026-10-07 前视 ToF / 0.15 m 升沉验证

**本报告记录带噪球面适配前的早期验证。最新结果见 [定位适配与闭环复验](tof_fit_validation_20261007.md)。当时的带噪 ToF 配置未通过任务验收。** 消息积压问题已修复，但现有红球球面定位算法拒绝带噪深度，无法提供有效三维目标位置。单次理想深度对照成功，不能记为 ToF 成功或新场景重复验证。

完整证据包：[20261007_front_tof_heave015_audit](../../data/baselines/20261007_front_tof_heave015_audit/README.md)。原始产物同时保留在 `data/experiments/current/` 和 `data/experiments/current/tof_heave_20261007/`，无原始记录改写；20261006 冻结包不变。

## 实际完整仿真

三轮均重新启动 Gazebo/PX4/DDS/ROS，前视相机、下视关闭、幅值 0.15 m / 0.25 Hz；原控制器、真值边界、0.50 m 捕获半径、海面保护不改。X 启动起飞及目标运动，在 FOLLOW 连续保持 3 s 后发送 Y；带噪两轮的 Y 请求等待有效位置锁定，没有进入有效截击。第 1 轮自动 X 提前触发，启动器随后错过 flight_ready 并超时退出控制台；节点继续运行并产出任务失败结果。第 2 / 3 轮等待就绪至少 10 s 后发送 X，启动器就绪检查通过。

| 轮次 / 输出 ID | 输入 | 有效位置观测 | 结果 |
|---|---|---|---|
| 1 / `20261007_101607_439432` | 带噪 ToF，旧 bytes 封装 | 0 / 413 | Y 后 30.048 s 超时，最近距离 4.910 m |
| 2 / `20261007_101922_226956` | 带噪 ToF，封装修正后 | 0 / 1483 | Y 后 30.049 s 超时，最近距离 4.943 m |
| 3 / `20261007_102124_727094` | 理想深度对照 | 362 / 469 | Y 后 10.953 s 捕获，距离 0.458 m |

有效率分母包括起飞、跟随、失去可见性等记录，不代表纯可见目标的检测率；CSV 的 `observation_valid` 与有同刻真值的 `valid` 在上述数据中计数一致。第三轮捕获水平距离 0.360 m、竖直误差 0.283 m；本轮是诊断对照，没有运行两次成功重复，不据此声明 0.15 m 场景稳定验收。

## 根因及已修正项

1. 新节点 `Image.data = bytes` 的 Python 元素校验耗时约 100–144 ms，大于相机 50 ms 周期。第 1 轮 412 条 `DEPTH_FRAME_UNMATCHED`，采集戳偏差中位数 100 ms。改为等价 `array('B')` 封装后，离线同尺寸测距加封装 p50 / p95 / max 约 9.6 / 16.1 / 31.2 ms。第 2 轮采集戳偏差中位数 0，无图像配对失败；未放宽配对或时序规则。
2. 第 2 轮仍无有效位置：`TARGET_VECTOR_INVALID` 871 条、`IMAGE_INVALID` 495 条、`DEPTH_RATIO_LOW` 117 条。前者是主阻塞。现有 [validation_sphere_center](../../src/uav_control/uav_control/perception/rgbd_target_localizer.py) 拟合球半径及残差门控面向理想球面：半径允许差 10%，95% 残差需不超过 `0.05 × radius`（本场景 1.25 cm）。当前 ToF 径向噪声标准差为 `0.01 + 0.001r` m，不能直接继续使用该理想深度算法。
3. 用同一个合成球面（同掩膜、同内参、同距离），在 1 / 3 / 5 / 8 / 15 / 24 m 各比较 20 帧：理想深度在六个距离均可定位，当前 ToF 噪声下各为 0 / 20。见 [同球面加噪对照](../../data/baselines/20261007_front_tof_heave015_audit/same_sphere_noise_audit.json)。这是感知算法兼容性测试，非真实相机精度测试。
4. ROS DDS 实测曾暴露 BEST_EFFORT 输出与原 RELIABLE localizer 不兼容，现已统一使用原 `aligned_camera_qos`。修正后的 DDS 冒烟测试接收到 640×480 测量，原 stamp/frame 保持，有效深度像素约 99.01%。这只证明接口可达，不能证明目标定位可用。
5. 原 shadow ToF 诊断绕过噪声读取理想 Gazebo 深度，已在 ToF 模式改为订阅真实模拟输出；缺测 NaN 不用理想深度替代。下视 sensor、桥与诊断均关闭，原模型几何/质量/惯量不变。

## 精度与冻结证据

当前带噪 ToF 两轮没有有效三维观测，**无定位 RMSE 可计算，不能报告“识别精准”**。当前红色掩膜仅检测验证球，没有无标记船体识别能力。

理想深度对照中的同刻真值误差，按现有 CSV 的阶段单列：PREPARATION 为起飞/跟随等混合段，332 条有效同刻样本，跨度 21.95 s，3D RMSE 0.171 m、P95 0.450 m；TERMINAL_APPROACH 为 29 条、跨度 1.40 s，3D RMSE 0.114 m、P95 0.164 m。它们是理想深度对照结果，不能转称为 ToF 精度，也不是单独稳态跟随窗口。

第三轮成功后排除首 1 s，记录 60 条世界统计、121 条目标状态：暂停、世界时间不变、目标位置不变、速度为零。两次 `/world/default/state` 的实时 ECS Pose 状态进一步确认 UAV 与目标姿态共同冻结；`scene/info` 只包含初始建模姿态，已明确排除为冻结依据。见 [freeze_audit.json](../../data/baselines/20261007_front_tof_heave015_audit/run3_ideal_control/freeze_audit.json)。

probe 退出后实际 `/target/state` 唯一订阅者为 evaluator；tracker/predictor 仍使用 KF。`gazebo_topics.txt` 无下视 RGB/深度，`ros_nodes.txt` 无下视桥/监视器。本次结束后仅清理记录的会话 PID 及其子进程。

## 检查及后续

- pytest：859 passed、1 skipped、2 项既有依赖警告；项目 flake8、shell 语法、`git diff --check` 通过。
- 构建：4 packages finished；`ros2 pkg prefix uav_control` 指向 `/home/qin/data/uav_usv_mpc/install/uav_control`。
- 原冻结包 46 项 SHA256 全通过；原控制器、KF、predictor、planner、evaluator、任务代码与 PX4 补丁未改。
- 相机/噪声及球面定位下一步需独立适配带噪测量，验证误差、拒绝边界和输出协方差，再运行前视 ToF 的任务及两次成功重复；不能以关闭噪声或放宽捕获半径替代验收。真实硬件还需具体型号、内外参/噪声/缺测标定和户外海面测试。

此阶段改动未提交或推送；当时默认 ToF 场景受定位拒绝阻塞，后续已另立定位适配与执行连续性任务。成功旧基线的恢复命令见 [相机文档](camera_simulation.md)。
