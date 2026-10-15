# P6 五模式匹配预测研究：最终冻结版本

最终输出为 `data/experiments/20261014_p6_development_01/prediction_04/`。
本报告仅描述离线合成传感器研究，不能等同 RGB-D、SITL 或真实捕获率。
最终数学版本冻结于 `CTRA_stop_Q_recovery_conservative_posterior_motion_priors_v4`，
训练 seed127、独立保留测试 seed3731；未用 heldout 残差拟合参数、尺度或 floor。
之前 `prediction/`、`prediction_02/`、`prediction_03/` 的中间结果与失败全部保留。

## 输入、时间与对照

五模式为直线、原八字、变速八字、变速 S、变速 Z，各 60 s。原位置观测 20 Hz，
预测评分 5 Hz，warmup 2 s；评价时域为 0.2/0.5/0.8/1.2/1.6/2.4/4.0 s。
位置噪声独立零均值 Gaussian，xyz 标准差 0.1/0.1/0.05 m；单帧丢失率 2%，
数据年龄 20–25 ms。两 split 共享确定性运动路径、使用不相交噪声 seed，
因此保留测试只验证新噪声实现，不能代表新运动分布。

完整链比较使用完全相同的 raw timestamp/position/covariance：原 raw→KF→BCTRA，
raw→locked CV/CTRV/CTRA，以及 raw→IMM。locked 模型采用 identity Markov 矩阵
和 one-hot 初始概率；每个采集戳只更新一次，所有预测显式传播到相同 source 历元，
原 acquisition stamp 不改写。未来 reference 只在离线评分中使用，由运动生成器
20 Hz 位置/速度样本插值；从未送入在线滤波器。

原 KF/BCTRA 参数读取当前 `baseline.yaml`，KF 过程加速度标准差 1.5 m/s²、
默认测量位置标准差 0.25 m、初始速度标准差 5 m/s；实际 raw covariance 覆盖默认 R。
原 BCTRA filtering、机动衰减、限幅和有界垂向模型保持原样。

IMM 修复了减速跨零后的物理截停与停止分支可恢复 P/V 过程噪声。
当前 posterior 先验为水平速度 5 m/s、omega ±0.7 rad/s、a_long ±2 m/s²、
alpha ±1.5 rad/s²；速度先验来自声明目标最大 4 m/s 加 25% 测量估计余量，
latent 范围取原 BCTRA 工程范围。所有三个模型采用同一 posterior 先验，
投影后增加均值位移外积协方差，原 Q/R 与模型概率更新未调参；未来路径没有被
强制设为统一终端速度。更快目标必须重新声明先验。

完整链差异同时包含原始观测滤波、垂向处理、运动先验与模型交互，不能解释为
单纯 IMM 模型切换收益。独立 `matched_initial_state_model_only` 分支以同刻真实
P/V/omega/a/alpha 初始化纯模型，只用于分离运动积分误差，不影响任何在线 estimator。
该 oracle 分支不报告协方差覆盖资格。

## 主要窗口结果

下表为 0.5/0.8/1.2/1.6 s 合并水平 RMSE，单位 m。
每链每模式 1,164 次评价尝试，其中 1,144 次有效、20 次丢帧失效，有效率 98.28%；
失效保留在分母。主要窗口所有有效预测均未超过 5 m。

| 模式 | CV | CTRV | CTRA | 原 KF+BCTRA | IMM | IMM 相对 BCTRA RMSE 降幅 |
|---|---:|---:|---:|---:|---:|---:|
| 直线 | 0.373 | 0.465 | 0.771 | 0.659 | 0.445 | 32.5% |
| 原八字 | 0.955 | 0.596 | 0.827 | 0.817 | 0.615 | 24.8% |
| 变速八字 | 0.806 | 0.581 | 0.815 | 0.731 | 0.584 | 20.1% |
| 变速 S | 0.571 | 0.524 | 0.776 | 0.661 | 0.514 | 22.3% |
| 变速 Z | 0.576 | 0.574 | 0.825 | 0.679 | 0.549 | 19.1% |

| 模式 | 位置 P95：BCTRA / IMM m | 速度 RMSE：BCTRA / IMM m/s | 速度 P95：BCTRA / IMM m/s |
|---|---:|---:|---:|
| 直线 | 1.443 / 0.922 | 0.929 / 0.429 | 1.889 / 0.816 |
| 原八字 | 1.721 / 1.316 | 1.038 / 0.713 | 2.079 / 1.518 |
| 变速八字 | 1.505 / 1.194 | 0.926 / 0.662 | 1.833 / 1.388 |
| 变速 S | 1.396 / 1.015 | 0.857 / 0.545 | 1.711 / 1.051 |
| 变速 Z | 1.487 / 1.134 | 0.895 / 0.606 | 1.858 / 1.233 |

直线 CV 最佳，原八字与变速八字 CTRV 的 RMSE 略优于 IMM。
模型概率大多温和变化，不能宣称明确识别了每个真实运动模式。
锁定 CTRA 在含噪完整链中较弱，保留所有误差而不删除该模型。

全部模式合并逐时域结果：

| 时域 s | BCTRA 水平 RMSE m | IMM 水平 RMSE m | 有效误差 >5 m：BCTRA / IMM |
|---:|---:|---:|---:|
| 0.2 | 0.1281 | 0.1251 | 0 / 0 |
| 0.5 | 0.2384 | 0.2208 | 0 / 0 |
| 0.8 | 0.3973 | 0.3458 | 0 / 0 |
| 1.2 | 0.7058 | 0.5617 | 0 / 0 |
| 1.6 | 1.1468 | 0.8381 | 0 / 0 |
| 2.4 | 2.4193 | 1.5840 | 35 / 4 |
| 4.0 | 5.4940 | 3.8671 | 545 / 256 |

4 s 仍有大量大误差，长时域轨迹不能直接解释为可靠拦截点。
在预注册“当前正在转弯、未来 1.6 s 内转弯率反号”标签下，主要窗口每链
180 个有效样本：BCTRA RMSE/P95=0.636/1.369 m，IMM=0.506/1.105 m，
CV=0.427/0.889 m、CTRV=0.558/1.217 m、CTRA=0.887/1.720 m；仍由 CV 最佳。
阶段标签非互斥，`summary.json.by_stage` 还包含匀速直线、恒转弯率、增长/下降、
纵向加/减速与复合机动，避免仅按整轮平均掩盖阶段差异。

![最终五模式逐时域 RMSE](../../data/experiments/20261014_p6_development_01/prediction_04/heldout_prediction_rmse.png)

## 同初值与误差分解

同历元真实初值下的纯模型主要窗口水平 RMSE 如下，单位 m。

| 模式 | CV | CTRV | CTRA | BCTRA | IMM 使用因果后验概率融合 |
|---|---:|---:|---:|---:|---:|
| 直线 | 0.121 | 0.121 | 0.053 | 0.047 | 0.073 |
| 原八字 | 0.639 | 0.166 | 0.068 | 0.067 | 0.177 |
| 变速八字 | 0.521 | 0.221 | 0.044 | 0.081 | 0.183 |
| 变速 S | 0.332 | 0.209 | 0.034 | 0.074 | 0.156 |
| 变速 Z | 0.331 | 0.222 | 0.051 | 0.083 | 0.159 |

这组 oracle 结果与完整链不能混报：纯 CTRA 在变速机动中积分误差较小，
而其含噪状态估计较差；完整链改进不等价于纯模型更精确。

保留测试 raw 观测 6,005 次、125 次丢帧；有效 raw 水平 RMSE=0.1408 m，
同采集历元 KF 当前位置 RMSE=0.0716 m、当前速度 RMSE=0.2815 m/s。
BCTRA 误差向量可分解为初始 P/V、导数估计和纯有界模型误差，
`decomposition_audit.json` 复算 10,010 条有效记录的向量恒等式，最大绝对差
1.78e-15 m。分量 RMS 有相关交叉项，不能直接相加。

| 模式 | 初始 P/V 影响 RMS m | 导数估计影响 RMS m | 纯模型误差 RMS m |
|---|---:|---:|---:|
| 直线 | 0.230 | 0.519 | 0.047 |
| 原八字 | 0.451 | 0.540 | 0.067 |
| 变速八字 | 0.393 | 0.477 | 0.081 |
| 变速 S | 0.297 | 0.447 | 0.074 |
| 变速 Z | 0.301 | 0.481 | 0.083 |

在本噪声假设下，状态及导数估计影响大于纯模型积分误差；这不意味着原 BCTRA
积分数学有错，也没有据此修改原 KF Q/R。

## 三维经验包络与加载接口

同目录 `bctra_calibration.json`、`imm_calibration.json` 可由 root 风险加载器读取：
schema `p6-calibration-v1`，family bctra/imm，distribution `empirical_envelope`，
calibration_domain **xyz**。训练使用完整三维 residual 与三维 covariance；
IMM 条件残差包含独立 z 项，不将二维覆盖误报为整体球三维覆盖。
训练逐时域取 95% Mahalanobis² 分位数，以 χ²95,3=7.814727903251179 归一，
再使用全部 0.2–4 s 最坏方差尺度，且 scale>=1。实际 radius multiplier
`empirical_scale=sqrt(χ²95,3)=2.7954834829151074`，必须与 covariance_scale 一起使用。
χ² 在这里仅作经验参考尺度，没有据此声称真实 Gaussian 或概率保证。

| family | covariance_scale | covariance_floor_m2 | empirical_scale | heldout 融合覆盖 | heldout 条件覆盖 |
|---|---:|---:|---:|---:|---:|
| BCTRA | 78.3861017546 | 0 | 2.7954834829 | 99.23% | 不适用 |
| IMM | 1.0000000000 | 0 | 2.7954834829 | 99.95% | 99.25% |

每 family 训练 9,975 条、heldout 10,010 条有效三维 residual。IMM 条件校准只包含
posterior probability>=0.05 分支；鲁棒检查可检查全部分支，但低概率分支没有该
离线覆盖声明。原 BCTRA `(0.02+0.10t)² I` 仅为经验基础，未成为可信传播协方差。
全时域 scale 由 4 s 决定，BCTRA heldout 4 s 覆盖 95.10%；IMM 协方差在此合成
噪声假设下明显偏宽，近 100% 覆盖不能称为精确 95% 概率校准。

两个文件 SHA256：

- BCTRA：`0d2351bc12a98aa877329c63f9f39f9bcc8e22c6867be722fcaa9428f2415784`
- IMM：`162b42c5d4396b93ef81d8d3c690af216429f0d37a1e0c17588ca8d10ab38cdf`

加载方绑定文件实际 bytes SHA 与 family；不能只按文件名赋资格。
经验包络推荐用于 shadow 或明确的 `nominal_fallback`。它不含实测姿态、yaw、
外参偏差、遮挡和 burst dropout；重叠时域样本相关，无整条轨迹联合概率、
连续时间保证或真实 95% 安全结论。真实 strict 资格须另有独立实际残差证据。

## 稳定性修复、耗时与可复算证据

中间研究发现的 locked CTRA 停止后失去速度恢复和过大 latent/current-speed
估计均保留。修复按次序只处理一个根因：物理截停与 Jacobian，停止侧 CV 型
平移过程 covariance，保守 posterior latent 先验，最后独立声明当前水平速度先验。
没有用真值修正在线状态，没有盲调 Q/R，也没有修改原 KF/BCTRA。
最终同一五模式×两个 seed 的 10 条短回放全部通过固定恢复检查：当前位置最大
误差 1.587 m、source speed 最大 5 m/s、4 s 位移最大 25.405 m，PSD 检查通过。
停止 covariance 另有独立 20,000 粒子 Monte Carlo 和持续移动位置观测恢复测试。

七时域预测 P95：BCTRA 各模式 0.168–0.261 ms，IMM 1.486–2.912 ms。
这些包含离线研究运行时争用，不能与 41 点在线工作量混报。最终冻结版本独立
41 点×3 模型、300 帧 CPU benchmark：update+forecast+消息 P95 4.20 ms，
包含完整 JSON 的 P50/P95/P99=4.90/5.18/8.14 ms，最大 10.03 ms，诊断 43,585 bytes。
该结果满足离线 p95<10 ms 目标，不代表 DDS/Gazebo/PX4 负载下的硬实时上界。

数学/包装/校准/ROS 契约 targeted 93 项 GREEN，root 集成复审 47 项 GREEN；
日志与 TDD RED→GREEN 历史位于 `imm/`，最终 CPU 结果为 `imm/final_04/benchmark.json`。
真实 Engine→ROS message→planner parser→条件诊断身份测试覆盖小 epoch、真实大 epoch、
非纳秒输入和相对 0.3 s，验证 ROS 1 ns 序列化契约。

`provenance.json` 保存完整输入参数；`source_snapshot/` 保存七份实际源码/配置 bytes，
`source_snapshot_manifest.json` 与递归 `SHA256SUMS` 可核对版本。数学源码 SHA256 为
`2bd650c7f160de1250bacc0f5294df68b38b5fca91da5e799590cfdad9af3eb3`；
包装源码 SHA256 为 `c1419eba3daa763ad6ac1532b1a049bc7b60a912b6acccddc10b64dca2b8bab4`。
完整 `*_observations.jsonl`、`*_forecasts.jsonl`、`*_model_probabilities.jsonl`、
`*_timings.jsonl` 保留原输入、真值评分、失效、conditional covariance 与投影次数，
支持重新计算指标。图已人工核对，无裁切/标签缺失。

当前 root 集成独立审查结果为 `imm/root_integration_review_04.json`，支持首轮
BCTRA 主链+IMM shadow、linear、名义可见性、固定观测点的 60 s pilot。
尚未据此授予 IMM 主链、robust strict 或联合飞行成功资格。
