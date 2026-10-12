# P4.7 Stage B 冻结 FOLLOW→MINCO 导数诊断

本目录对 F3 两个 H 各 489 个同源原始请求做离线初始化及独立验收。没有控制接管、在线参数放宽、真值在线输入或 SITL。全部失败保留。正式统计文件为根目录 `f3_h1.2.jsonl` / `f3_h1.6.jsonl`；`legacy_reported_attempt/` 是首次诊断发现旧积分错误前的保留尝试，不作正式积分统计。

## 人口与时序边界

原 P4.6 的 162 项来自 H1.2 B 单次原记录：initial_status=DYNAMIC_INFEASIBLE 的168项中，162项同时违反精确 horizontal jerk。充分离线验证将原 NOT_ATTEMPTED 的33项也计算，故全489出现175项 horizontal jerk；这13项额外发现不混入原162。H1.6相应原始人口112，全489为118。

|H|原P4.6 hJ人口|起点极值|第一段内部极值|该人口起点V/A已不可行|充分离线initial拒绝数|
|---:|---:|---:|---:|---:|---:|
|1.2|162|155|7|22|216|
|1.6|112|112|0|13|176|

所有这些 jerk 极值均在 piece 0。H1.2 原162项同时含 yaw 121、horizontal speed20、horizontal acceleration4、vertical jerk1项。起点已越界是必要可行性障碍，终端 V/A/Q 修正不能改写真实起点。

## 有界诊断消融

保持原始 start PVA、终点 P 和总 H。A/VA/QVA/Q 均用现有 MINCO 线性映射加三点 Gauss-Legendre quadrature 求有界最小平方 jerk；该积分对每段二次 jerk 的四次平方精确。终端 A 每分量±1.5 m/s²、终端 V 每分量±0.5 m/s、Q每坐标±0.2m。它们是显式诊断域，不是安全阈值。T-only 固定 Q/端PVA，三段各±20%、10%步长有限网格且总 H 不变；yaw-only 保持 XYZ/T，用未来三个yaw knot各±0.3rad最小平方 yaw rate。

每个被选修正调用原 validate_seed，离线预算10s，硬约束保持。下表只使用相同原 P4.6 hJ 人口；“解除hJ”与完整验收分开。

|mode|H1.2 解除hJ /162|H1.2 全验收 /162|H1.6 解除hJ /112|H1.6 全验收 /112|
|---|---:|---:|---:|---:|
|A|1|1|1|0|
|VA|4|3|4|3|
|QVA|155|38|110|24|
|Q|151|38|100|24|
|T|0|0|0|0|
|yaw|0|0|0|0|

在该明确消融域内，单独终端 A 或 V+A 的初段 jerk 收益很小，放开 Q 明显有效。该结果支持“Q插值约束与初段jerk冲突”作为主要排查方向，但没有证明任何 QVA 修正都安全或替代控制必然成功。纯积分最优不等于 peak-constrained 最优，也不等于 FOV/姿态/海面/起点约束通过。QVA 后剩余 hJ 为7/2项，其中4/1项起点已不可行；仍有同时 yaw 失败。

## 旧 control_effort 的独立复现

现有 MincoS3Trajectory.control_effort 的三个 cross term 系数少一半：c3·c4 应144而旧值72；c3·c5 应240而旧值120；c4·c5 应720而旧值360。标准 1s 零端V/A、P:0→1 的 quintic（Q=17/81、64/81）独立平方jerk积分是720，旧方法报告2231.1111111111。此问题被定向测试实测复现，未改生产函数。

正式每个 assessment 的 `jerk_integral` 来自独立精确 Gauss 积分；`legacy_reported_control_effort` 原样单列。旧 P4.4/P4.6 的 jerk_integral 数字不能当可信积分，不直接与新 exact 积分比较排名。硬 derivative peak 仍由原 derivative_peak 给出，本问题不等于旧硬 jerk 门限失效。

## 产物与验证

- `f3_h*.jsonl`：全部原始请求、同源line/hash、原P4.6状态、initial start/endPVA/Q/T/yaw、FOLLOW10ms区间jerk极值、MINCO六项 exact峰值及piece/local/global time、yaw峰值、同时违约、start必要可行性、全部失败/反事实及原完整验证outcome。
- `per_request_diagnosis.csv`：同一初始/修正人口的扁平索引，3330个assessment行；嵌套数值保留JSON cell。
- `jerk_and_boundary_h*.png`：原人口分布和中位案例jerk/FOLLOW间隔jerk；`boundary_pva_h*.png`：固定start/terminalP下的terminalV/A和Q位移；`plot_witness_h*.json`保留案例完整边界。
- `source_manifest.json`：F3原始shadow及两份原P4.6比较JSONL前后SHA相同；本次计算源码指纹。`analysis_source_used.py` 是正式计算实际执行的精确脚本快照（在后续plot/style增补前复制）。
- `code_baseline_353c3e1.tar.gz`：本任务初始干净HEAD的库/脚本冻结副本；只用于复算旧population。生产算法在本次计算完成后才获准继续修改。
- `SHA256SUMS`：所有产物包括保留失败尝试的SHA256。

```bash
cd /home/qin/data/uav_usv_mpc/src/uav_control
python3 -m pytest -q test/test_p47_derivative_diagnosis.py test/test_p47_execution_timeline.py
cd /home/qin/data/uav_usv_mpc
python3 -m flake8 scripts/p47_derivative_diagnosis.py src/uav_control/test/test_p47_derivative_diagnosis.py
PYTHONPATH=src/uav_control:scripts OPENBLAS_NUM_THREADS=1 python3 scripts/p47_derivative_diagnosis.py --output /tmp/p47_reproduction --horizons 1.2 1.6
MPLCONFIGDIR=/tmp/p47_mplconfig PYTHONPATH=src/uav_control:scripts python3 scripts/p47_derivative_diagnosis.py --plots-only
cd data/experiments/20261010_p47_follow_guided/derivative_diagnosis
sha256sum -c SHA256SUMS
```

若生产库已经更新，用 `bash reproduce_frozen.sh /tmp/p47_frozen_reproduction` 精确复算。该包装器在独立 /tmp 目录展开冻结源码、复制实际执行脚本、以只读用途链接原 data 目录，并将输出写到指定新目录。追加 `--limit 1 --horizons 1.2` 可做快速入口检查；正式结果不使用该limit。定向测试先RED后GREEN，最终7项derivative测试及9项timeline测试通过；无全套回归、构建或SITL。
