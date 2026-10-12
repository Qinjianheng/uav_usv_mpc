# P4.7 Stage A 控制归属与回退时间轴

输入为 P4.5 四轮原始 JSONL、真实 Tracker ACK、planner shadow JSONL、ULog 与配置；仅写入此独立目录。算法与控制代码未修改，未启动仿真。所有 JSON/CSV 的绝对时间戳保持原始 ROS 时钟；导航 PVA 以 debug.state_stamp 精确配对。

## 直接触发与上游更新

|run|H(s)|MINCO→FOLLOW|EXPIRED ACK|同 tick 重建|租约内无替代提案|替代被拒|已接受但 START_MISSED|
|---|---:|---:|---:|---:|---:|---:|---:|
|b_f2_1|1.2|35|35|0|32|3|0|
|b_f2_2|1.2|31|31|0|28|1|2|
|c_f2_1|1.6|28|28|0|25|2|1|
|c_f2_2|1.6|37|35|2|31|3|3|

131 次回退的直接分支均为 nominal lease 到期。129 次有 EXPIRED ACK；c_f2_2 的旧 plan 127 和 169 在同 tick 先到期，然后 pending 129 / 171 因 START_MISSED 返回 REJECTED，覆盖了函数中的旧 EXPIRED 返回值。两例依据原始执行 tick 越过旧 deadline、上一轮 prefix VALID、同 tick 的已接受 pending START_MISSED 及相同源码重建；没有补造 ACK。

PREDICTION_CHANGED 不直接撤销 nominal active：tick 显式排除该错误，再依最新 prediction 重验 prefix。四轮没有 REVOKED ACK，回退前旧 plan 的最新 prefix 均为 VALID。prefix invalid、parent invalid、prediction changed、measured divergence 作为 active 回退直接触发的观测次数均为 0。parent invalid / input expired / start missed 是候选更新失败，独立写在逐切换上游字段。没有 measured-PVA divergence gate，误差大小不能替代分支证据。

## 更新节奏与预算

配置 execution_lead=0.15 s，lease=0.45 s。配置 lead 从研究请求构建起算，实际 publication→start 是求解/发布后的剩余预算。

|run|提案/接受/激活数|提案间隔 median / p95 (s)|提案间隔 ≥0.45 s|ACTIVE间隔 median / p95 (s)|publication→start median(s)|
|---|---:|---:|---:|---:|---:|
|b_f2_1|149/140/137|0.222783 / 1.416807|38|0.226450 / 1.547913|0.096662|
|b_f2_2|154/145/140|0.219103 / 1.498093|29|0.207802 / 1.475860|0.100802|
|c_f2_1|171/166/165|0.215828 / 1.029104|29|0.203468 / 0.848809|0.095363|
|c_f2_2|159/152/148|0.219473 / 1.063593|33|0.204102 / 1.048646|0.097471|

每次切换均记录最后 ACTIVE 时刻、距该时刻的间隔、旧 reference 年龄、deadline 超时量、租约内全部 replacement proposal、各候选 ACK、parent ID、输入 TTL 与可否在旧 deadline 前启动。shadow publication-final 非发布原因也按旧 ACTIVE 到切换 tick 区间列入。上游事件可复算，但不被写成唯一物理根因。

## 信号边界

- 262 个切换前/后实际 PVA 均有精确同 stamp 的原始 navigation 样本；加速度直接取记录，未微分/外推。
- MINCO 前一回调的参考 PVA 来自 execution_debug.reference，参考时刻为 max(nav stamp, active start)。原 FOLLOW 内部几何参考 P/V/A 未记录，after.reference_pva=null；不能把速度命令当作该几何参考。
- final emitted velocity 来自 tracker 在 setpoint_pub.publish 后记录的 /control/reference。velocity 模式的 position/acceleration 是 NaN，输出用 null 表示。
- ULog trajectory_setpoint 约 5 Hz，而 Tracker 20 Hz。只在原生时间落在 navigation 对应区间内映射，并以 125 ms 窗口及 ≤1e-6 m/s 的同向量交叉核验；73/262 个切换前后指令命中。其余明确未知，不能声称全部 PX4 接收瞬间或精确时延已验证。
- 四轮独立 shadow ACK recorder 与 monitor ACK identity/stamp/state 完全一致，无仅单侧 ACK。
- configured_height_error_m 是实际 NED z - (-5 m)；reference_vertical_error_m 是实际 z - 当前 MINCO reference z，二者不可混用。c_f2_2 后半段配置高度误差显著振荡，即使当前 MINCO reference tracking error 较小。
- flight.yaml 单独记录 follow_minco_enabled=false/shadow_check=true；真实运行要结合 launch 覆写、condition 与已执行的 final diagnostic/ACTIVE ACK，不能将该 YAML 单独当有效全参数。
- 此审计不修改 .45 s 租约、.05 s 激活窗口、freshness、捕获判据，也不证明 robust holding 或替代控制的闭环表现。

## 产物与复算

- 每轮 `plans.json`：proposal、parent、名义 deadline、完整 ACK history、Tracker 使用的 prefix/prediction 版本、最终 owner diagnostic epochs。
- 每轮 `switches.json` / `switches.csv`：完整逐切换直接分支、上游更新与前后 PVA/命令/高度；CSV 嵌套证据以 JSON cell 保存。
- 每轮 `callbacks.json`：全部 callback 的原时钟、实际 navigation PVA、参考与最终命令。
- 每轮 `original_clock_timeline.jsonl` / `timeline.png`：完整原时钟生命周期与可视化。
- 每轮 `source_inputs_sha256.json`：10 个实际读入原始文件的前后 SHA256 相同；顶层 `audit_code_sha256.json` 保存审计时控制源与脚本指纹。
- `SHA256SUMS`：本审计目录全部交付产物指纹。

```bash
cd /home/qin/data/uav_usv_mpc/src/uav_control
python3 -m pytest -q test/test_p47_execution_timeline.py
cd /home/qin/data/uav_usv_mpc
python3 -m flake8 scripts/p47_execution_timeline.py src/uav_control/test/test_p47_execution_timeline.py
MPLCONFIGDIR=/tmp/p47_mplconfig python3 scripts/p47_execution_timeline.py
cd data/experiments/20261010_p47_follow_guided/execution_timeline
sha256sum -c SHA256SUMS
```

定向测试已实际执行 RED→GREEN：先 7 项因缺少审计实现失败，再通过；另两项分别先捕获不充分的 START_MISSED 归因和缺少 plan history，再修正通过。最终 9 项测试通过，新增两文件 flake8 通过；未运行全套 pytest/build。
