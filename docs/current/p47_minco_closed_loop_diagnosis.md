# P4.7：闭环与导数不可行归因

本轮基于 HEAD `353c3e1a9dba15277b156896e1270bd0abe74f97`，严格先分析 P4.5
实际执行，再研究 P4.6 冻结输入。实际执行结论见
[执行归因](p47_execution_root_cause.md)；新算法对照见
[可行性研究](p47_follow_guided_feasibility.md)。旧实验和原 checkout 均保持只读。

## 实际执行与理想滚动是不同证据

四轮 F2 真正执行过 MINCO，H1.6 的 ACTIVE 高度误差主要在参考曲线：参考/实际跟踪
Z RMSE 为 1.516/.053 m 和 .523/.059 m。全 owner 稳定窗高度 RMSE 是 2.246/1.091 m，
包含回退恶化，不能只选 ACTIVE 子集。实际参考 PVA 连续不等于与实际状态重合，也不
保证始终在期望高度。P44/D rank 的 P/V 只含 XY，XYZ jerk 缩放和 parent 非零 vz/az
继承形成了有日志支持的高度漂移机制；c_f2_2 同时证明零 jerk 不是全部偏差的唯一原因。

131 次回退直接源于 .45 s lease 到期，上游为 116 次无接替 proposal、9 次接替被拒、
6 次 accepted 接替 START_MISSED。没有 active REVOKED，回退前 prefix 均 VALID。
不能归咎于 prediction id 更新，也不能为了提高 ACTIVE 比例延长 lease。

P4.7 继续使用原 Tracker 和唯一 PX4 setpoint 发布者；修改参考生产者及计算调度。
下面的 frozen request / own-curve rolling 是预测模型中的理想研究，不是植物执行结果。

## 精确复现历史 jerk 人口

读取原 P4.6 B single JSONL，并按原 source path/line 绑定。H1.2 的历史 168 个 initial
DYNAMIC 拒绝中精确找到 162 个 horizontal jerk 违约；另外 33 个原来未尝试的请求
单列。充分离线预算下全部 489 请求有 175 个 hJ 违约，这个数字不能冒充历史 162。

每条记录保存起点/终点 PVA、两个 Q、三段 T、10 ms FOLLOW 区间 jerk、MINCO 六类
精确范数极值及 piece/local/global time、yaw 解析极值、姿态与完整独立验收结果。
使用 `polynomial_extrema.derivative_peak` 的驻点和端点；不以采样 jerk 代替严格峰值。

| H | 全请求 | 历史 hJ 人口 | 第一段峰值 | t=0 峰值 | 内点峰值 | 全请求起始 V/A 已越界 |
|---|---:|---:|---:|---:|---:|---:|
| 1.2 | 489 | 162 | 162 | 155 | 7 | 32 |
| 1.6 | 489 | 112 | 112 | 112 | 0 | 32 |

H1.2 历史 162 人口同时含 121 个 yaw、20 个 speed、4 个 acceleration 和 1 个
vertical jerk 违约；22 个起始 V/A 已不合法。横向 jerk 峰值中位数 7.7364 m/s³。
改变终点不能把原本超速的起点变成合法轨迹。FOLLOW 离散响应 jerk 有界也不保证
插值该响应的全局最小 jerk 五次多项式峰值有界。

## 先做单变量反事实，再选择变量

所有反事实保持 start、末端 P 与总 H，不改硬约束。A 分量域 ±1.5 m/s²、V ±.5 m/s、
Q ±.2 m 是显式搜索域，不是新的物理阈值。纯 jerk 二次问题用原 MINCO 线性映射及
三点 Gauss 精确积分。T-only 是各段 ±20% 的有限网格、总 H 不变；yaw-only 是三个
非起始 knot ±.3 rad 的离线诊断。它们没有加入在线新候选搜索。

| 变量 | H1.2 解除 hJ / 162 | H1.2 完整验收 / 162 | H1.6 解除 hJ / 112 | H1.6 完整验收 / 112 |
|---|---:|---:|---:|---:|
| A | 1 | 1 | 1 | 0 |
| V+A | 4 | 3 | 4 | 3 |
| Q+V+A | 155 | 38 | 110 | 24 |
| Q | 151 | 38 | 100 | 24 |
| T | 0 | 0 | 0 | 0 |
| yaw | 0 | 0 | 0 | 0 |

结论是**两个中间位置约束对第一段 jerk 更关键**，终端 A 或 V+A 单独调整不足以
解释或解决多数失败。QVA 的微小增益说明终端导数仍值得保留有界调整；但解除 hJ
不等于完整可行，固定 yaw 和不合法起点仍是明显瓶颈。有限 T/yaw 反事实不能证明任意
T/yaw 优化无效。主比较在看到此表后、运行新算法矩阵前确定 C/D 都使用 QVA，分别
关闭/开启低维修正，避免根据比较结果选择有利主组。A/VA/QVA 单独消融全部保留。

## 独立发现的旧积分报告问题

`MincoS3Trajectory.control_effort()` 的交叉项系数应为 144、240、720，旧实现为
72、120、360。单段 1 s、0→1、两端 V/A=0 的标准 quintic，正确 jerk 能量为 720，
旧函数报告约 36720；分段表达还会改变这个错误报告值。因此本轮不能直接比较旧字段
`jerk_integral` 与正确积分。

`FastMincoObjective.jerk()` 的 Gram 积分是正确的；硬导数峰值验收也没有使用这个
错误的能量报告值。本轮为保持 P44/D 与 P4.6 的冻结行为，没有全局改写旧函数。
新目标和离线统计独立精确积分，原值命名为 `legacy_reported_jerk_integral` 保留。
这项遗留问题仍影响旧 fixed objective/报告的物理解释，后续修复需独立比较，不能
从历史旧积分宣称新方法降低了多少百分比能量。

## 证据与复算入口

- `data/experiments/20261010_p47_follow_guided/derivative_diagnosis/`：
  `f3_h1.2.jsonl`、`f3_h1.6.jsonl`、两个 summary、逐请求 CSV、源码快照、来源 manifest。
  `legacy_reported_attempt/` 保留发现积分问题前的首轮分析，正式统计使用顶层修正版。
- `jerk_and_boundary_h1.2.png`、`jerk_and_boundary_h1.6.png`：极值与修正分布。
- `boundary_pva_h1.2.png`、`boundary_pva_h1.6.png` 与 `plot_witness_*.json`：明确请求的
  起始/末端、Q、导数时间曲线及可复算证据。
- 工具 `scripts/p47_derivative_diagnosis.py`：新增独立诊断；未改写 P4.6 输入或输出。

阶段 D/E 的最终验收、是否启动实际 SITL 和遗留项，以可行性报告中的同窗对照及五项
门禁结论为准；本诊断没有宣称已经修复植物闭环或保证达到研究目标。
