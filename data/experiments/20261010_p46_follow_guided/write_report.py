"""Write P4.6 tables from saved results, without rerunning or rewriting raw evidence."""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
WS = ROOT.parents[2]
RUN = ROOT/'comparison'
SUMMARY = json.loads((RUN/'summary.json').read_text())
POST = json.loads((ROOT/'postanalysis.json').read_text())
CONFIGS = SUMMARY['configurations']


def table(headers, rows):
    return '\n'.join(['| '+' | '.join(headers)+' |',
                      '| '+' | '.join(['---']*len(headers))+' |',
                      *['| '+' | '.join(map(str, row))+' |' for row in rows]])+'\n'


def fmt(value, scale=1., digits=3):
    return 'N/A' if value is None else f'{value*scale:.{digits}f}'


def quant(value, scale=1., digits=3):
    return 'N/A' if value is None else '/'.join(
        fmt(value[q], scale, digits) for q in ('p50', 'p95', 'p99'))


def count_rate(n, d):
    return f'{n}/{d} ({n/d*100:.1f}%)' if d else '0/0 (N/A)'


def distribution(values):
    if not values:
        return None
    return dict(count=len(values), minimum=min(values), maximum=max(values),
                **dict(zip(('p50', 'p95', 'p99'), np.percentile(values, (50, 95, 99)))))


def extra_metrics():
    additional = dict(p44_profile={}, independent_final_fit={}, boundary_errors={})
    front_stages = ('candidate_reference', 'candidate_jerk', 'candidate_yaw',
                    'candidate_seed_construction', 'candidate_proxy_geometry',
                    'candidate_proxy_score', 'candidate_sort')
    for key in CONFIGS:
        is_a = key.endswith('A_single')
        independent, boundary, profiles = {}, {}, []
        for line in (RUN/(key+'.jsonl')).open():
            row = json.loads(line)
            if row['result'] is None:
                continue
            result = row['result']
            detail = result['metrics'].get('follow_guided', {})
            if is_a:
                stages = result['metrics']['cycle_profile']['stages']
                attempts = result['metrics']['p43']['attempts']
                profiles.append(dict(
                    candidate_front_exclusive=sum(stages.get(s, {}).get('seconds', 0.)
                                                  for s in front_stages),
                    coefficient_construction=stages.get('minco_coefficients', {}).get(
                        'seconds', 0.),
                    validation_inclusive=sum(a['strict_seconds'] for a in attempts)))
            if not result['valid']:
                continue
            for name, value in detail.items():
                if name.startswith('boundary_') and isinstance(value, (int, float)):
                    boundary.setdefault(name, []).append(value)
            independent_valid = key.split('_')[-2] == 'A' or detail.get('independent_valid')
            fit = (row.get('evaluation') or {}).get('final_curve_to_follow_fit')
            if independent_valid and fit:
                for name, value in fit.items():
                    independent.setdefault(name, []).append(value)
        if profiles:
            additional['p44_profile'][key] = {
                name: distribution([p[name] for p in profiles]) for name in profiles[0]}
        additional['independent_final_fit'][key] = {
            name: distribution(values) for name, values in independent.items()}
        additional['boundary_errors'][key] = {
            name: distribution(values) for name, values in boundary.items()}
    additional['p44_front_stages'] = front_stages
    return additional


EXTRA_PATH = ROOT/'additional_metrics.json'
if EXTRA_PATH.exists():
    EXTRA = json.loads(EXTRA_PATH.read_text())
else:
    EXTRA = extra_metrics()
    EXTRA_PATH.write_text(json.dumps(EXTRA, indent=2, allow_nan=False)+'\n')

parts = [r'''# P4.6：原 FOLLOW 引导的 MINCO 初始化与优化

日期：2026-10-10。起始 HEAD：`a00aebb0af6d89acf94751d083f2ba5830ecd0fb`。
实施位置：`/home/qin/data/uav_usv_mpc`。本轮未提交、未推送、未运行 SITL。

## 结论与十二项回答

**架构已实现，离线验证已完成；新方法尚未达到替代 P4.4/D 的计算效率、总体可行率和持续跟随质量目标。**
原 FOLLOW 的完整速度命令逻辑已经复用，三段 MINCO 能合理拟合假设响应模型下的 FOLLOW P/V/A；
但小拟合误差不能保证 jerk 可行。优化提高部分独立可行率，同时占用后备求解预算。
保持新研究模式默认 `refinement=none`、H=1.2 s，保留 P4.4/D 一行切回入口。
该默认值只表示本轮研究配置选择，不代表具备实际闭环优越性或新增执行授权。

| 问题 | 本轮回答 |
|---|---|
| 1. 真正复用原 FOLLOW 吗？ | 是，私有 `FlightGuidanceCore.command('FOLLOW', ...)`，没有复制简化水平公式；原生产 FOLLOW 未修改。 |
| 2. 垂向是否一致？ | 是，独立使用 `1.0 × (-5-z_UAV)`，经过原垂向限速、加速度整形和海面保护；目标升沉不改变 UAV 高度参考。 |
| 3. 未来轨迹是什么？ | 20 Hz 原 FOLLOW 命令驱动虚拟速度滞后模型，10 ms 区间内常 jerk 精确积分的 P/V/A 序列。模型未由 PX4 实测辨识。 |
| 4. MINCO 初值如何构造？ | 当前合法执行起点 P/V/A；Q=P_ref(H/3,2H/3)；终点 P/V/A=P_ref/V_ref/A_ref(H)；三段等时；复用已有 yaw 解缠、样条及起始 yaw rate。 |
| 5. 是否取消五候选主初始化？ | 新引擎仅一次 FOLLOW 初始化；五候选只存在于显式标记的有界 P4.4/D 后备。旧引擎仍可选择。 |
| 6. Q 优化更好吗？ | 部分独立可行率提高，如 F3/H1.2 259→294；总体可行数却从396降到368，滚动只维持3.56 s，不能判定更好。 |
| 7. Q/T/yaw 值得默认使用吗？ | 当前不值得。F3/H1.6 滚动比固定方案更久，但仍中断，耗时更高；无四组共同20 s后样本。 |
| 8. 独立可行率？ | 固定方案 F2两时域均70.3%；F3/H1.2为53.0%，H1.6为60.7%。其余消融见下表。 |
| 9. 多少请求回退？ | 固定方案 F2每时域6/118；F3/H1.2为158/489，H1.6为122/489。回退成功与独立成功分别统计。 |
| 10. 时间下降了吗？ | 没有。固定方案 P50约9.85–11.65 ms，P4.4约5.39–5.80 ms；Q和QT更慢。 |
| 11. 精度/高度改善了吗？ | F2短共同窗口略好；F3/H1.2完整A/B窗口，20 s后水平RMSE由1.351升至2.359 m，高度误差下降但水平明显退化。 |
| 12. 值得立即开展实际SITL闭环吗？ | 当前不建议。先解决三段拟合 jerk 过冲和剩余预算分配，再重做同输入比较；本轮没有用SITL掩盖离线负结果。 |

## 实现与边界

### FOLLOW 滚动

入口：`uav_control.guidance.follow_rollout.rollout_follow(problem, config, initial_command=None, deadline=None, clock=...)`。
`FollowRolloutConfig` 默认位置增益0.8、高度增益1.0、控制周期0.05 s、积分步长0.01 s、响应时间常数0.5 s。
`problem` 先由现有 `FollowProblem` 核验时戳、状态、预测覆盖与执行边界合法性。

每个控制周期调用未修改的 `FlightGuidanceCore`：

```
v_cmd,xy = v_target,xy + 0.8 (p_target,xy - 5 heading_xy - p_UAV,xy)
v_cmd,z  = 1.0 (-5 - z_UAV)
a_des    = bound_accel((v_cmd-v_UAV)/0.5)
Δa       = bound_jerk(a_des-a_UAV, dt)
j        = Δa/dt
p_next   = p + v dt + a dt²/2 + j dt³/6
v_next   = v + a dt + j dt²/2
a_next   = a + j dt
```

公式中的命令先经过原 FOLLOW 的水平/垂向限速、加速度限制、速度整形及海面保护。
虚拟响应又约束加速度与 jerk 的变化，不把命令瞬时当作真实速度。
保留请求中的真实可用起始加速度；不使用命令中占位的零加速度。
若物理起始速度超限，只限制合法初始化命令，绝不把请求 P/V/A 裁剪成合法状态，最终轨迹仍拒绝。
每个积分点检查截止时间；控制/积分最多各10000步，先限制再分配数组。

`initial_command` 支持调用者提供合法已发命令。本轮冻结输入和现有研究请求没有对应时刻的实际命令，
在线工厂与离线比较均显式采用合法执行起点速度初始化，并记录 `command_source=execution_velocity_initialization`。
不能将这一假设称为已识别的 PX4 响应，也不能据此认定真实闭环会重现理想滚动。
未来目标仅来自请求内已记录的 `/tracking` 预测；无 ROS/PX4 输出，无未来 Gazebo 真值。

H=1.2 s的前10个可用请求分别以0.01/0.005 s积分，控制周期不变；整个粗网格上的最大P/V/A差：
''']
parts.append(table(['场景', '样本', 'max ΔP (m)', 'max ΔV (m/s)', 'max ΔA (m/s²)'], [
    [scene.upper(), v['count'], *[fmt(x, digits=6) for x in v['maximum_pva']]]
    for scene, v in POST['convergence'].items()]))
parts.append(r'''
这支持本轮短时域10 ms离散的数值稳定性；并非响应模型真实性验证或无限步长收敛证明。
额外单测覆盖恒转率、低速、大初始误差、yaw跨±π以及海面保护。

### 三段 MINCO 与 Accepted 边界

`follow_guided_seed(request, problem, reference)` 返回现有 `FollowSeed`；
系数构造完全复用 `MincoS3Trajectory`。起点直接取合法 `request.state`，终点采样完整参考 P/V/A。
yaw 初值复用 `forecast_viewpoint` 方向、`np.unwrap` 和现有 `YawTrajectory`；
`reference_yaw_rate` 继续由现有执行接口传递。总时域保持H，初始三段各H/3。

`AcceptedFollowRequest` 的共享执行时刻由原边界验证核实，parent coefficients/context未被修改。
记录起点与原测量的P/V/A差，以及将测量因果投影到共享执行时刻后的差。
前一差包含原导航戳到执行戳的时间距离，不能直接解释为空间偏置。
离线 `HypotheticalRequest` 仅采样本组上一条曲线；不是Tracker实际ACK。

### 优化代价、变量及回退

新增 `FollowGuidedMinco` 扩展 `DirectReferenceMinco`。旧 `virtual_follow` 也改为调用上述原 FOLLOW 滚动。
`TrackingMincoObjective` 新增显式冻结参考序列、权重及是否优化yaw；旧调用默认行为保持。
新模式通过 `reference=reference` 追踪本轮滚动序列，时间变量的梯度包含物理时间参考插值导数。

```
J = wp ∫||P-P_ref||²dt + wv ∫||V-V_ref||²dt
  + (0.03/36) ∫||jerk||²dt
  + 20 J_visibility + (0.01/36) ∫||jerk-jerk_prior||²dt
  + 100 J_dynamic + 1000 J_sea + 0.005 ∫yaw_rate²dt
```

沿用既有求积与MINCO伴随梯度，求积内部采用P/V权重 `H×4/25`、`H×2/9`，加速度跟踪权重为0。
`jerk_prior` 来自上一条曲线在同一执行时刻的jerk；没有parent时取零。
连续性项是整个时域对prior jerk的软惩罚；起点P/V/A、yaw及yaw rate连续性由已有硬边界接口保证。
完整球体相机姿态代理仅用于优化排序；所有输出仍经现有 `validate_seed` 严格验收，代理不授予准入。

| refinement | 变量 | 边界与预算 |
|---|---|---|
| none | 无 | 单一FOLLOW初值→完整验收 |
| q | 6维Q | 每个坐标初值±0.25 m；yaw与段时间固定 |
| qt | 6维Q+2维时间logit+3维yaw，共11维 | logit±1.5，固定总H及既有段时长下界；yaw初值±0.3 rad |

L-BFGS-B默认最多2次迭代、maxls=3、优化预算25 ms，留15 ms给后续步骤；每次目标调用前检查截止。
单次NumPy/SciPy调用不可硬中断，因此预算不是操作系统实时保证；最终到期检查会撤销有效结果。
初值即使违反动力学/FOV仍允许有预算的优化，只有完整验收通过的优化结果能替换初值。
若优化失败而固定初值合法，保留固定初值；若独立失败且剩余预算>25 ms，调用P4.4/D后备。
初始预留25 ms与后备触发条件是本轮保守实现选择，导致实测可行率退化，见失败分析。

总截止仍取原输入有效期与配置求解预算的较早者，保留20 ms发布储备。
未放宽125 ms输入年龄、v/a/jerk、yaw rate、倾角、推力、整球FOV或海面净空。
Tracker/ACK、有限lease、隔离授权及异常撤销逻辑未修改；没有新增setpoint发布者。

## 冻结输入、统计口径与溯源

实验根目录：`data/experiments/20261010_p46_follow_guided/`。
正式矩阵：`comparison/`，完整32组JSONL、`manifest.json`、`summary.json`、`SHA256SUMS`。
本轮完整运行未使用 `--limit`，607个原始请求×4组×2时域=4856次独立求解；
滚动按共同初始化、实际间隔及至少0.2 s的采样过滤，共4608行，停止后行保留为未尝试，绝不计作成功。

| 场景 | 原请求数 | 独立窗口跨度(s) | 共同滚动记录数 | 原完成间隔中位数/最大值(s) |
|---|---|---|---|---|
| F2 | 118 | 118.364 | 106 | 1.020/1.040 |
| F3 | 489 | 104.580 | 470 | 0.218/0.560 |

F2原日志不是5 Hz，不能补造中间预测。滚动只在原记录时刻重规划、采样自己的曲线；
候选拒绝保留旧curve及原deadline，首次曲线覆盖中断后终止该episode，不从下一条日志导航重置。
RMSE是这些共同epoch的样本加权统计，不是均匀时间积分。

| 输入 | 工作区内原路径 | SHA256 |
|---|---|---|
| F2 | `data/experiments/20261009_p32_realtime_follow/f2_retry/shadow/mpc_seed_shadow_1791515236281436128_111866.jsonl` | `7641b2b5453839ef92968fd84f9f74f86d1808a0b83abcabe557ce49f91a3c95` |
| F3 | `data/experiments/20261009_p32_realtime_follow/f3_short/shadow/mpc_seed_shadow_1791517694786638951_126448.jsonl` | `f7fc246f85f75632925b83b358a126034c8be2b929a520c01b04c2409b191fed` |

共同初始化直接复用P4.3已有文件 `data/experiments/20261009_p43_follow_optimization/common_initial/{f2,f3}_initial.json`。
F2源行597，epoch=1791515268.9016387，行SHA=`075df73e4b1ad7122c4563503edf884c79c3a364a3d56b080abdbb090ac5aacd`；
F3源行436，epoch=1791517716.6230757，行SHA=`e545ce14d664f18c37a315b6091a2a06c520b10a7c71d784780929acb257a891`。
文件SHA、全部选入源行SHA和本轮源码SHA见manifest，未补写旧配置。

独立比较每次创建新solver，禁用warm hint；四组使用相同原始请求、相机、模型约束、H及冻结 `raw.now_stamp`，
四组输入一致性检查均通过。单调计算时钟真实运行；不是实际跨进程端到端到达年龄。
滚动仅初态相同，后续执行起点来自各组自己的曲线，这是理想滚动定义所需的状态差异。
所有行 `hypothetical_only=true, tracker_accepted=false`；真实测量只作边界差诊断，无真值纠偏。

冻结约束：水平/垂向速度6.2/4.0 m/s，加速度3/3 m/s²，jerk6/4 m/s³，yaw rate1 rad/s；
倾角0.55 rad，比推力2–18 m/s²，净空储备0.5 m，响应延迟0.15 s，制动加速度2.5 m/s²。
相机640×480，fx=fy=269.968184，主点(320,240)，外参及完整目标球半径0.25 m/中心偏移见每行 `model_config`；
水平/垂向FOV配置储备均0.07 rad。模型、输入和相机均未改动。

主机：Python3.10.12，NumPy1.24.4，SciPy1.8.0，Linux6.8.0-138；BLAS/OMP/MKL均1线程。
矩阵串行求解，运行期间未并行pytest/build。

## 四组独立求解结果

A=P4.4/D；B=FOLLOW固定；C=FOLLOW+Q；D=FOLLOW+Q/T/yaw。
独立/触发回退/总体的分母均为全部原请求；回退成功率分母为回退触发数。
全部超期及未尝试独立初始化的请求仍计入分母，不能仅在有预算子集上报告可行率。
''')
parts.append(table(['场景/H', '组', '独立可行', '回退触发', '回退成功', '最终可行', '总wall P50/P95/P99(ms)'], [
    [key.rsplit('_', 2)[0], key.split('_')[-2], count_rate(v['independent_valid'], v['count']),
     count_rate(v['fallback_used'], v['count']), count_rate(v['fallback_valid'], v['fallback_used']),
     count_rate(v['feasible'], v['count']), quant(v['solver_wall_seconds'], 1000)]
    for key, v in CONFIGS.items() if key.endswith('_single')]))
parts.append(r'''
Q/QT提高F3部分独立可行率，但剩余后备预算减少；最终可行率全部低于固定组及P4.4。
H1.6固定、Q、QT独立可行数分别297、316、313；总体398、379、337。
当前优化同时改变跟踪、平滑、动态和可见性权衡，目标下降不等于最终跟随质量改善。

### 分阶段耗时

下表为每组全体独立请求的P50(ms)，包含提前拒绝记录。
系数是 `CycleProfile.minco_coefficients` 的实测互斥子阶段总和，含验证、优化、后备内的构造；
优化/验收/后备是各自包含子阶段的wall时间，因此不能把这些列相加作为总耗时。
A的初始化列是下述已记录候选阶段之和，其他组是完整滚动初始化wall，口径有差异。
A验收是全部候选 `strict_seconds` 之和，不仅最后候选。
''')
phase_rows = []
for key, v in CONFIGS.items():
    if not key.endswith('_single'):
        continue
    t = v['timing']
    if key.endswith('A_single'):
        p = EXTRA['p44_profile'][key]
        vals = [p['candidate_front_exclusive'], p['coefficient_construction'], None,
                p['validation_inclusive'], None]
    else:
        vals = [t.get(s) for s in ('initialization', 'coefficient_construction',
                                  'optimization', 'validation', 'fallback')]
    phase_rows.append([key.rsplit('_', 2)[0], key.split('_')[-2],
                       *[fmt(None if x is None else x['p50'], 1000) for x in vals]])
parts.append(table(['场景/H', '组', '初始化*', '系数', '优化', '验收', '后备'], phase_rows))
parts.append(r'''
A的初始化*：`candidate_reference/jerk/yaw/seed_construction/proxy_geometry/proxy_score/sort`互斥时间和；
未把共享预测插值、执行上下文和未归因开销伪造成完整初始化wall。
A没有优化或外部后备，表中N/A表示不适用；B的优化0表示明确未执行。
各列P95/P99保存在 `summary.json` 和 `additional_metrics.json`。

区分两个拟合诊断计时：求解器内 `result.timing.fit_analysis` 属于求解wall（约0.37–0.81 ms中位数）；
比较脚本求解后重新构造共同FOLLOW参考及最终曲线误差的 `analysis_seconds` 不属于求解wall。
manifest中“fit_analysis outside solver timer”指后者，不是将求解器内诊断排除。
首次诊断用的系数构造另记 `initial_coefficient_construction`，不是全部系数成本；
优化组可能为最终拟合又构造一次，不能将该字段解释为唯一初始化MINCO。

### FOLLOW → 初始 MINCO 拟合误差

完整10 ms网格上的3D误差，各请求RMSE的P50/P95；不只检查端点或Q残差。
下表使用B的全部可获得初值，包含随后被硬约束拒绝的初值；C/D使用相同初值。
''')
fitrows = []
for scene in ('f2', 'f3'):
    for h in (1.2, 1.6):
        key = f'{scene}_h{h}_B_single'
        fit = CONFIGS[key]['follow_guided_fit']['initial_fit']
        fitrows.append([f'{scene}/H{h}', fit['position_rmse']['count'], *[
            '/'.join(fmt(fit[n][q], digits=6) for q in ('p50', 'p95'))
            for n in ('position_rmse', 'velocity_rmse', 'acceleration_rmse',
                      'position_vertical_rmse')]])
parts.append(table(['场景/H', 'n', 'P(m)', 'V(m/s)', 'A(m/s²)', '垂向P(m)'], fitrows))
parts.append(r'''
F3/H1.6初值位置RMSE最大0.003506 m；位置拟合良好，但仍须检验曲线导数极值。
下面的最终拟合只取**独立成功**输出，不混入P4.4后备；表项为各请求RMSE的P50/P95。
''')
finalrows = []
for key, fit in EXTRA['independent_final_fit'].items():
    if not key.endswith('_single') or key.endswith('A_single'):
        continue
    finalrows.append([key.rsplit('_', 2)[0], key.split('_')[-2], fit['position_rmse']['count'], *[
        '/'.join(fmt(fit[n][q], digits=6) for q in ('p50', 'p95'))
        for n in ('position_rmse', 'velocity_rmse', 'acceleration_rmse', 'height_rmse')]])
parts.append(table(['场景/H', '组', 'n', 'P(m)', 'V(m/s)', 'A(m/s²)', '绝对高度RMSE(m)'], finalrows))
parts.append(r'''
完整最终曲线（含A和后备）相对FOLLOW的P/V/A最大差、垂向差和高度误差均在每行 `evaluation`；
混合总体分布在 `final_curve_to_follow_fit`，不能据此替代新方法独立拟合质量。

## 理想滚动与共同窗口

“覆盖率”表示原epoch中能采样有效曲线的比例，旧curve仍有效时即使本次拒绝也有覆盖；
“尝试/合法”是实际重规划次数/新合法结果；独立及后备仅计新合法结果。
覆盖中断后保留未尝试行，连续维持时间取旧curve实际截止或原窗口终点。
''')
parts.append(table(['场景/H', '组', '尝试/合法', '独立合法', '回退触发/成功', '采样覆盖%', '连续维持(s)'], [
    [key.rsplit('_', 2)[0], key.split('_')[-2], f"{v['attempted']}/{v['feasible']}",
     v['independent_valid'], f"{v['fallback_used']}/{v['fallback_valid']}",
     fmt(v['coverage_fraction'], 100, 2), fmt(v['continuous_survival_seconds'])]
    for key, v in CONFIGS.items() if key.endswith('_rolling')]))
parts.append(r'''
F2新方法在相同无预算请求后曲线到期，约10 s后停止；不能将前10 s较小误差外推至全程。
F3/H1.2固定新方法与A均覆盖全程，但固定组405条合法中152条来自后备；
Q/QT在3.56 s中断。H1.6的Q/QT持续92.54/94.44 s，优于固定3.76 s，但仍不是完整成功。

### 四组共同样本（水平/高度分开）

每条源行SHA、epoch相同且四组均有自己曲线的样本才进入比较。
水平RMSE相对固定后方5 m预测点，绝对高度RMSE相对z=-5 m。
“所选观测点”与固定后方点、距离、相对速度误差另存summary，不与高度混成单一指标。
''')
commonrows = []
for key, m in SUMMARY['matched_rolling'].items():
    for g, e in m['groups'].items():
        commonrows.append([key, g, f"{m['count']}/{m['aligned_count']}", m['steady_count'],
                           fmt(e['all']['fixed'], digits=6), fmt(e['all']['height'], digits=6),
                           fmt(e['after20']['fixed']), fmt(e['after20']['height'])])
parts.append(table(['场景/H', '组', '共同/原样本', '共同20s后n', '水平RMSE(m)',
                    '高度RMSE(m)', '20s后水平', '20s后高度'], commonrows))
parts.append(r'''
四组共同窗口均不足20 s，因此四组20 s后指标全部N/A，不能记成0。
F3此表11–12 m主要来自共同初始化后的早期追赶，仅说明同一短过渡窗口，不能当作稳定跟随误差。

### 额外A/B完整窗口：F3/H1.2

这是唯一A/B均持续覆盖且有20 s后样本的窗口，共470点，20 s后376点；
不与上面四组短共同窗口混用。
''')
pair = POST['paired_fixed']['f3_h1.2']['groups']
parts.append(table(['组', '窗口', '水平固定后方RMSE(m)', '所选点RMSE(m)', '距离RMSE(m)',
                    '相对速度RMSE(m/s)', '高度RMSE(m)'], [
    [g, window, *[fmt(e[window][n], digits=8 if n == 'height' else 6)
                  for n in ('fixed', 'selected', 'distance', 'relative_velocity', 'height')]]
    for g, e in pair.items() for window in ('all', 'after20')]))
parts.append(r'''
结论：固定新方法高度稳定，但20 s后水平和相对速度都退化，不能宣称整体改善。
微米级理想高度差接近模型/数值量级，不代表真实PX4达到该精度；高度反馈仍为原增益，未提高。

### 动力学、完整目标FOV与交接

下表是每组合法滚动输出（含后备）的全曲线最大值/最小裕度，样本窗口按上表覆盖各异，
用于检查合法输出，不能直接当作相同窗口的性能排名。
加速度与jerk采用原求解器极值/界报告；FOV采用原完整球体、自适应验收样本裕度。
FOV裕度已扣除配置角度储备；不把离散FOV检查称为连续时间证明。
''')
physicalrows = []
for key, v in CONFIGS.items():
    if not key.endswith('_rolling'):
        continue
    b = v['solver_dynamic_and_fov']
    names = ('maximum_horizontal_acceleration', 'maximum_vertical_acceleration',
             'maximum_horizontal_jerk', 'maximum_vertical_jerk',
             'minimum_horizontal_margin', 'minimum_vertical_margin')
    physicalrows.append([key.rsplit('_', 2)[0], key.split('_')[-2], *[
        fmt(b[n]['minimum'] if n.startswith('minimum') else b[n]['maximum'], digits=6)
        for n in names]])
parts.append(table(['场景/H', '组', 'a_xy(m/s²)', '|a_z|', 'j_xy(m/s³)', '|j_z|',
                    'FOV水平(rad)', 'FOV垂向(rad)'], physicalrows))
parts.append(r'''
各组速度、yaw rate、倾角、比推力与海面检查保持；详细极值见summary与每行physical。
例如F3/H1.2固定组最大水平速度6.194574、yaw rate0.963436、倾角0.296874、比推力10.255261，均在原限制内。
F3/H1.6的Q优化最小水平FOV裕度0.000999 rad，虽通过当前验收，仍没有额外鲁棒余量结论。

所有成功接替在同一执行epoch的P/V/A、wrapped yaw、yaw rate最大残差均为0（数值保存精度）。
F3/H1.2的A/B分别467/404次交接，H1.6的A/B/C/D分别460/7/369/377次；
这是共享边界构造与验收结果，不是实际导航零误差。
原测量与执行参考偏差另存 `follow_guided.boundary_measurement_*` / `boundary_projected_measurement_*`，
以及 `additional_metrics.json`。未重置parent或用测量直接覆盖已接纳边界。

F3/H1.2固定组合法输出的原测量P/V/A差，中位数分别0.834382 m、0.313200 m/s、0.290736 m/s²；
投影到共享epoch后的中位数为0.598097 m、0.355831 m/s、0.290736 m/s²，最大位置差10.068077 m。
这里比较的是本组理想曲线与历史另一个实际运行的导航，不能据此认定真实执行连续或准入合法。
冻结日志中请求为FutureRequest，不含实际AcceptedFollowRequest；Accepted边界由专门单测覆盖，
本轮没有在线ACK/真实测量闭环验证。

## 失败案例与下一步边界

1. **预算拒绝**：F2每个新方法都有24/118条`NO_FRESHNESS_BUDGET`，F3有30/489条，另3条最终截止。
   新实现先要求至少25 ms验收储备，P4.4只要求正总预算且每候选>3 ms；这会拒绝P4.4能在约6 ms完成的请求。
   F2滚动首次中断对应源行608；其前一请求被提前预算拒绝，旧curve截止未延长。
   这是预算分配实现的保守性代价，不是需要增大125 ms TTL的证据。
2. **拟合导数过冲**：F3/H1.2固定初值168条动力学拒绝，其中162条水平jerk超限；
   拒绝初值最大水平jerk10.799882 m/s³（限6）。H1.6仍有127条动力学拒绝，其中112条水平jerk超限，最大9.411876。
   虚拟响应的区间jerk受限不等于三段五次MINCO的jerk受限；端点PVA与两Q精确不足以控制整个区间导数。
   相同记录可能同时违反多项限制，归因计数不能相加作为失败请求数。
3. **起始超速**：F2两时域各9条初值水平速度超限，F3各26条；未裁剪实际起始PVA。
4. **yaw**：F3/H1.2、H1.6固定初值分别29、32条yaw rate拒绝；解缠避免角度跳变但不保证样条斜率合规。
5. **优化与后备竞争**：优化提高部分独立可行数，但更少请求有25 ms剩余后备预算。
   F3/H1.2 B触发158次/成功137，C仅88/74，D仅53/43；不能把总体下降解释为回退本身的成功率提高。

后续应保持同一主问题：先处理参考→三段MINCO的导数保真与预算分配，重做本矩阵。
当前没有证据支持增加控制增益、放宽jerk/FOV、扩展TTL、经验轴偏置或真值修正。
没有辨识真实速度响应，也未验证新方法实际Tracker到达年龄和PX4执行误差，暂不推进实际闭环效果结论。
红球仍只验证仿真感知接口，不外推为无标记非合作USV验证。

## 测试、旧失败审计与构建

新增算法测试30项、比较脚本测试10项；覆盖原FOLLOW整周期命令一致性、高度/升沉隔离、
速度响应非瞬时、直线/恒转率/低速、大误差、yaw跨π、PVA/航点/系数、Q6/QT11梯度、
初值非法仍优化、预测版本绑定、Accepted parent不变、完整FOV/海面/动力学拒绝、
真实P4.4回退分开计数、总截止撤销、积分内部截止、步数上限与CLI配置元数据。
测试工厂覆盖新engine与原P4.4选择；既有Planner/ACK/Tracker门禁由全回归覆盖。

旧 `data/experiments/20261010_p45_minco_execution/full_initial.log` 中两失败重新核对：
tracker第341行E501已在本轮起始HEAD修复，未再次修改tracker；
behind-camera测试被真实主机计时先触发30 ms截止，生产失败关闭正确。
几何/因果单测注入固定时钟，独立增加30/31 ms边界测试；没有增大生产重验预算。
真实40 ms诊断延迟保留实际几何计算，修复前复现失败、修复后语义断言通过。
证据：`regression_audit/report.md`、before/after日志及fingerprints；旧日志/生产重验代码未改写。

| 检查 | 实跑结果 | 日志/证据 |
|---|---|---|
| 全量pytest（包目录） | 1484 passed、1 skipped、3 warnings，15.77 s | `verification/full_final.log` |
| 四包最终构建 | px4_msgs、uav_usv_interfaces、uav_control、uav_usv_bringup全部成功，约61 s | `verification/build_final.log` |
| 项目flake8 | 通过 | `verification/flake8_final.log` |
| shell语法与diff检查 | 通过 | `verification/final_checks.log` |
| ROS包前缀 | `/home/qin/data/uav_usv_mpc/install/uav_control` | `verification/final_checks.log` |
| 正式矩阵SHA256 | 32 JSONL、manifest、summary全部通过 | `verification/comparison_checksums.log` |
| 本轮源码与矩阵指纹 | 全部匹配 | `verification/final_checks.log` |

warnings为2条SelectableGroups弃用提示、1条测试重复配置日志提示；无失败。
构建/pytest/lint不等于离线性能验收通过；本报告明确保留算法负结果。

## 修改文件与复现

| 文件 | 用途 |
|---|---|
| `src/uav_control/uav_control/guidance/follow_rollout.py` | 新增原FOLLOW命令+虚拟响应纯函数 |
| `src/uav_control/uav_control/controllers/direct_reference_minco.py` | 原virtual_follow修正、完整PVA初值、fit、独立新solver与后备 |
| `src/uav_control/uav_control/guidance/p43_minco_objective.py` | 显式参考、权重及Q-only维度，原调用默认保持 |
| `src/uav_control/uav_control/controllers/follow_research_shadow_node.py` | 新engine工厂与研究参数；p4 planner继承入口 |
| `src/uav_usv_bringup/config/follow_minco.yaml` | 新研究默认固定、保留P4.4选择及原门禁 |
| `scripts/p45_minco_sitl.py` | 显式engine/refinement CLI与真实元数据；本轮未调用SITL |
| `scripts/p46_follow_study.py` | 同输入独立+连续理想滚动、完整原始请求/曲线/参考/溯源 |
| `src/uav_control/test/test_follow_guided_minco.py` | 新算法及工厂/CLI测试 |
| `src/uav_control/test/test_p46_follow_study.py` | 同输入、窗口、停止/拒绝、计时及度量测试 |
| `src/uav_control/test/test_follow_revalidation.py` | 修复时间敏感单测、补截止独立覆盖 |
| `docs/current/p46_follow_guided_minco.md`、`docs/current/README.md` | 报告和索引 |
| `docs/superpowers/plans/2026-10-10-p46-follow-guided-minco.md` | 实施/验证记录 |

P4.4切回：在专用研究YAML把 `minco_engine: follow_guided_minco` 改回 `p44_adaptive`，
保留 `p44_ablation: D, p44_refinement: none`；不涉及Tracker/setpoint逻辑。
优化消融使用 `follow_guided_refinement: q` 或 `qt`，默认 `none`。
以下命令只做构建/离线比较，不启动Gazebo或PX4；输出路径须未存在：

```bash
cd /home/qin/data/uav_usv_mpc
set -eo pipefail
./scripts/build_workspace.sh
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 scripts/p46_follow_study.py --output /tmp/p46_comparison_new
# 快速接口检查可另指定 --limit 1 --horizons 1.2；不可代替正式统计。
cd /tmp/p46_comparison_new
sha256sum -c SHA256SUMS
cd /home/qin/data/uav_usv_mpc/src/uav_control
python3 -m pytest -q
cd /home/qin/data/uav_usv_mpc
python3 -m flake8 src/uav_control/uav_control src/uav_control/test scripts
git diff --check
git diff --cached --check
ros2 pkg prefix uav_control
```

原始JSONL包含raw_request、实际求解request、完整model_config及constraint fingerprint、
result与P44候选尝试、独立/回退标记、系数、FOLLOW完整参考、seed、拟合误差、
阶段时间、physical、handover、sampled_state/error、连续覆盖与停止原因。
`postanalysis.py/json`补充匹配A/B窗口、失败导数归因和步长比较；
`additional_metrics.json`补充独立输出拟合、A的已记录profile和边界差。
`write_report.py`从上述证据生成本报告表格，不重跑算法。
实验根目录 `DELIVERY_SHA256SUMS` 从工作区根目录校验本轮数据、报告与功能文件。
新增约519 MB离线证据保留本地，没有全量暂存或自动提交实验文件。
原仓库 `/home/qin/data/uav_usv` 保持只读，冻结pre-MPC基线和历史原始输入未改写。
''')
(WS/'docs/current/p46_follow_guided_minco.md').write_text('\n'.join(parts))
