import json
from pathlib import Path

WS = Path('/home/qin/data/uav_usv_mpc')
ROOT = WS/'data/experiments/20261010_p47_follow_guided'
S = json.loads((ROOT/'comparison/formal_20261012_2/summary.json').read_text())
CFG = S['configurations']
lines = []


def add(text=''):
    lines.append(text)


def fmt(x, digits=3):
    return '—' if x is None else f'{x:.{digits}f}'


def table(headers, rows):
    add('| '+' | '.join(headers)+' |')
    add('|'+'|'.join('---' for _ in headers)+'|')
    for row in rows:
        add('| '+' | '.join(map(str, row))+' |')
    add()


def configurations(suite, mode):
    for scene in ('f2', 'f3'):
        for h in (1.2, 1.6):
            arms = ('A', 'B', 'C', 'D') if suite == 'main' else (
                ('a', 'va', 'qva') if suite == 'terminal' else ('legacy', 'adaptive'))
            for arm in arms:
                yield scene, h, arm, CFG[f'{scene}_h{h}_{suite}_{arm}_{mode}']


add('## 单请求可行率与耗时')
add()
add('分母包含所有原始请求，不能排除短预算或不合法起点。后备触发数与后备成功数分别列出。')
add('耗时是本次 fresh replay 的包含 profile 开销的外层实测 wall time；不与历史不同运行的耗时作百分比比较。')
add()
table(['场景', 'H', '组', '独立通过 / N', '最终通过 / N', '最终率 %', '后备触发 / 成功'], [
    [scene.upper(), h, arm, f"{v['independent_valid']}/{v['count']}",
     f"{v['feasible']}/{v['count']}", fmt(100*v['final_feasible_fraction'], 2),
     f"{v['fallback_used']}/{v['fallback_valid']}"]
    for scene, h, arm, v in configurations('main', 'single')])

def phase(v, field, q):
    stat = v['timing'].get(field)
    return fmt(1000*stat[q]) if stat else '—'

table(['场景', 'H', '组', '初始化 P50/P95 ms', '总求解 P50 ms', 'P95 ms', 'P99 ms'], [
    [scene.upper(), h, arm, phase(v, 'initialization', 'p50')+'/'+phase(v, 'initialization', 'p95'),
     *[fmt(1000*v['solver_wall_seconds'][q]) for q in ('p50', 'p95', 'p99')]]
    for scene, h, arm, v in configurations('main', 'single')])
add('A 的初始化未按 FOLLOW 单独记录，故显示未知。B/C/D 包含未尝试请求的 0，不能把它们当作一次成功初始化的成本。')
add('独立 rollout 性能包另给纯滚动耗时：F3/H1.2 P50 4.225→0.599 ms、P95 4.452→0.654 ms；')
add('H1.6 P50 5.388→0.751 ms。完整初始化、MINCO 构造和严格验收仍有成本。')
add()

add('## 拒绝原因与历史 jerk 人口')
add()
add('下面是 F3 的独立方法阶段拒绝分类，可重叠；不等于所有同时存在的物理违约。')
add('原验证器只返回泛 DYNAMIC 时，必须有精确导数证据才细分；A 的此类拒绝保留 unresolved。')
add('预算/时效列包含对应状态码；P47 的 22 个 `SHORT_BUDGET_P44_PRIORITY` 是主动转后备，单独列明。')
add()
table(['H', '组', 'jerk', 'speed', 'accel', 'yaw', '预算/时效', 'dynamic 未细分', '其他'], [
    [h, arm, *[v['rejections']['independent'][key] for key in
               ('jerk', 'speed', 'acceleration', 'yaw', 'deadline', 'dynamic_unresolved', 'other')]]
    for scene, h, arm, v in configurations('main', 'single') if scene == 'f3'])
add('F3/H1.2 最终失败仍为 39：35 个原后备 DYNAMIC、1 个 yaw、3 个时效。')
add('初值还未尝试就转后备的 22 条计入独立方法未成功，不能冒称 jerk 修复或额外输入。')
add()
table(['H', '历史 hJ 人口', 'QVA 解除 hJ', '完整独立通过', '含后备最终通过'], [
    [h, d['historical_population'], d['groups']['C']['horizontal_jerk_resolved'],
     d['groups']['C']['independent_valid'], d['groups']['C']['final_valid']]
    for h in (1.2, 1.6)
    for d in [json.loads((ROOT/f'final_evidence_20261012/historical_jerk_h{h}_summary.json').read_text())]])
add('155/162（95.7%）解除 hJ，不等于 155 条全部可执行：该人口同时存在 yaw、速度和起点不可行。')
add('第 436 行新版能量为 9.0047，hJ 为 6.1240，仍超过原 6.0 上限并被拒；能量降低不能替代峰值验收。')
add()

add('## 终端与预算消融')
add()
table(['场景', 'H', '变量', '独立通过', '最终通过', '后备触发', 'P95 ms'], [
    [scene.upper(), h, arm.upper(), v['independent_valid'], v['feasible'], v['fallback_used'],
     fmt(v['solver_wall_seconds']['p95']*1000)]
    for scene, h, arm, v in configurations('terminal', 'single')])
add('A/VA/QVA 消融均开启同一有界完整目标修正，和主组 C 的关闭修正另作区分。')
add('最终率被后备托住时，A/VA 的不足会被掩盖：H1.6 理想滚动 A/VA 不能完整覆盖，QVA 可以。')
add('阶段 B 同一历史人口的单变量反事实表是判断 Q 必要性的主要依据。')
add()
table(['场景', 'H', '调度', '独立通过', '最终通过', '后备触发 / 成功', '滚动覆盖 s'], [
    [scene.upper(), h, arm, v['independent_valid'], v['feasible'],
     f"{v['fallback_used']}/{v['fallback_valid']}",
     fmt(CFG[f'{scene}_h{h}_budget_{arm}_rolling']['continuous_survival_seconds'])]
    for scene, h, arm, v in configurations('budget', 'single')])
add('压力消融只延迟求解器到达时刻，原输入、expiry 和 20 ms reserve 不变；旧门槛八组均 0 通过。')
add()
table(['场景', 'H', 'N', 'adaptive 8 ms', '12 ms', '20 ms', '24 ms'], [
    [scene.upper(), h, CFG[f'{scene}_h{h}_budget_stress_adaptive_8ms_single']['count'],
     *[CFG[f'{scene}_h{h}_budget_stress_adaptive_{ms}ms_single']['feasible']
       for ms in (8, 12, 20, 24)]]
    for scene in ('f2', 'f3') for h in (1.2, 1.6)])
add('8 ms 等紧预算仍会出现真实超期，24 ms/H1.6 也有一次失败；经验 P95/P99 不是保证。')
add()

add('## 连续理想滚动：覆盖与同窗误差')
add()
table(['场景', 'H', '组', '新 proposal 通过 / N', '连续覆盖 s', '完整覆盖', '后备成功'], [
    [scene.upper(), h, arm, f"{v['feasible']}/{v['count']}",
     fmt(v['continuous_survival_seconds']), '是' if v['fully_covered'] else '否', v['fallback_valid']]
    for scene, h, arm, v in configurations('main', 'rolling')])
add('四组共同 after20 仅 F3/H1.2 非空（N=376）；该窗口 A/B/C/D XY RMSE 分别为')
v=S['matched_rolling']['f3_h1.2_main']['groups']
add('/'.join(fmt(v[g]['after20']['fixed'],6) for g in 'ABCD')+' m。')
add('F2 两个 H 与 F3/H1.6 的四组共同 after20 均为空，不能比较各自不同窗口。以下另列 A/C/D 完整同窗。')
add()
table(['场景', 'H', '组', '全程 XY RMSE m', 'after20 XY m', 'after20 相对 V m/s', 'after20 Z m'], [
    [scene.upper(), h, g, fmt(q['all']['fixed'],6), fmt(q['after20']['fixed'],6),
     fmt(q['after20']['relative_velocity'],6), fmt(q['after20']['height'],8)]
    for scene in ('f2', 'f3') for h in (1.2, 1.6)
    for g,q in S['matched_rolling_acd'][f'{scene}_h{h}_main']['groups'].items()])
add('F2 同窗 N=106/after20=86，F3 为 470/376。F2/H1.2 C/D 的稳态 XY 由 .0893 增至 .1876 m，')
add('相对速度由 .0671 增至 .2187 m/s；这是本轮未通过 SITL 门槛的明确退化，不能只展示 F3 的改善。')
add('有界最小能量初值在直线场景改变了已较好的 FOLLOW 局部响应；目前证据支持这一质量取舍，尚无植物反事实证明。')
add()

add('## 动力学、完整目标 FOV 与边界')
add()
add('下表为完整理想滚动中所有最终验收通过曲线的独立精确导数最大值，含成功后备；')
add('FOV 是原完整目标验收的最小裕度，非目标中心点可见性。各组被接受曲线人口可以不同。')
add()
table(['场景', 'H', '组', 'hV ≤6.2', 'vV ≤4', 'hA ≤3', 'vA ≤3', 'hJ ≤6', 'vJ ≤4', 'min FOV rad'], [
    [scene.upper(), h, arm, *[fmt(v['coefficient_quality']['derivative_extrema'][key]['maximum'],6)
          for key in ('horizontal_speed','vertical_speed','horizontal_acceleration',
                      'vertical_acceleration','horizontal_jerk','vertical_jerk')],
     fmt(v['minimum_complete_target_fov_margin_rad'],6)]
    for scene,h,arm,v in configurations('main','rolling') if arm in ('A','C','D')])
add('yaw/姿态/推力/海面仍走原完整验收。F3/H1.2 C 的最大 yaw rate .9634 rad/s、tilt .2505 rad、')
add('specific thrust 10.1023 m/s²；所有通过曲线保持原门槛。**标准未放宽，但实际 FOV 最小裕度下降**：')
add('同场景 A 为 .2711 rad，C/D 为 .2315 rad，不能声称 FOV 裕度完全没有损失。')
add()
table(['场景', 'H', '组', 'P 偏差 P50/P95 m', 'V 偏差 P50/P95 m/s', 'A 偏差 P50/P95 m/s²'], [
    [scene.upper(),h,arm,*['/'.join(fmt(v['boundary_measurement_offsets']['projected_measurement'][k][q],6)
                 for q in ('p50','p95')) for k in ('position','velocity','acceleration')]]
    for scene,h,arm,v in configurations('main','rolling') if arm in ('C','D')])
add('该偏差比较理想 own-curve 边界与同一请求所保存导航 PVA 的显式 CA 投影，不能叫作真实跟踪误差。')
add('原始 measurement 与 projected_measurement 两套数值均保留。全部 C/D 交接的 P/V/A/yaw/rate')
add('参考间误差为 0；这与实测 PVA 偏差并不矛盾，植物证据仍以阶段 A 同 native epoch 分解为准。')
add()
add('## 修正收益与剩余算法问题')
add()
add('D 没有新增独立通过数量。F3/H1.2 single 的 334 次优化中，333 次到达 6 ms 有界停止，')
add('仅 1 次返回并通过验收；rolling 442 次中也仅 1 次返回。H1.6 rolling 437 次中返回并保留 23 次，')
add('XY RMSE 仅 .431856→.431732 m；P95 却 10.698→19.600 ms。因此 D 的完整目标修正目前不值得默认启用。')
add('C 比 B 改善了可行率和 F3 长期误差，但单请求仍有 38.2%（H1.2）/33.3%（H1.6）触发后备。')
add('F3/H1.2 rolling 后备降至 47/470，不能外推为实际 ACTIVE 比例。')
add('剩余方向是固定 yaw/合法起点之外的联合约束质量，以及在降低 jerk 时保留已良好的直线 FOLLOW 响应；')
add('本轮不新增 yaw/T 贪心搜索，不盲调权重，不延长名义 lease，也不以更宽约束掩盖负结果。')
add()

path = WS/'docs/current/p47_follow_guided_feasibility.md'
text = path.read_text()
assert '<!-- FINAL_TABLES -->' in text
path.write_text(text.replace('<!-- FINAL_TABLES -->', '\n'.join(lines)+'\n'))
(ROOT/'final_evidence_20261012/report_tables_used.py').write_text(Path(__file__).read_text())
print('report tables generated',len(lines),'lines')
