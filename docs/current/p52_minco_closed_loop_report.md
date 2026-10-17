# P5.2 MINCO 闭环准备与验证报告

> 授权续做更新：P52第1轮无ACTIVE；第2轮35.47%/最长5.35s；第3轮27.41%/最长6.35s，均未达到80%，F3未准入。三轮已封存。本文以下保留原离线交付的当时状态；最新完整窗口、修复与待授权复验见 [P5.3验收](p53_f2_validation.md)。


基线：master `413b54fefa35c01f8d235aa553249d2071de23f5`，开始时干净；日期 2026-10-13。
本轮修复执行资格、身份失效和合法候选供应的已复现缺陷，完成离线故障回放。
**没有启动新 SITL，不能宣布实际连续 MINCO 控制、F2/F3 通过或性能优于 FOLLOW。**

研究参数仍为 P47/C `p47_follow_guided`、H1.6、qva、refinement=false、adaptive、P44/D。
125ms 原始输入期限、450ms lease、50ms 激活窗口、原完整验收保持。
原 FOLLOW 主控制律、BCTRA、PX4 参数、相机标定和捕获半径未修改。

## 本轮证据

新原始输出：`data/experiments/20261013_p52_offline_01/`；精简审计：`audit/p52_follow_minco/`。
旧 P4/P5/P5.1 数据保留原字节。资格绑定当前代码，不继承 P5.1 的飞行授权或尝试目录。
检查结果和文件 SHA 以审计包最终 verification.json、SHA256SUMS 为准。
本轮最终1704 passed、1 skipped、3 warnings；四包构建、flake8、5个shell语法、
安装消息、launch参数加载和两种Git diff检查通过。原984个保护文件哈希全部未变。
单次独立审查无Critical、1项Important已完成RED→GREEN修复，无未决Important/Minor。

| 人口 | 结果 | 能证明的范围 |
|---|---|---|
| 当前源码三条历史启动请求 | 第三条由预算跳过恢复为合法输出；前两条仍拒绝 | 同输入调度修复，不是在线成功率 |
| 20s 直线共速解析夹具 | 1 次初始 ACTIVE、71 次后继 ACTIVE、0 次回退 | 生产 solver→wire→Receiver 协议可连续衔接 |
| 两次求解失败 | 两次均保留合法旧曲线，70 次后继，0 次回退 | 此时序相位有余量；不保证任意两次失败都能衔接 |
| 输入中断/性能偏差/前缀失效 | 各 1 次回退，7.75s 后重新 ACTIVE | 冷却、稳定和探测没有被跳过 |
| 全部扩展夹具 | 无过期 ACTIVE、无非法 parent、无立即重入 | 虚拟时钟、解析导航，无 ROS/PX4 发布 |
| 本轮实际 F2/F3 | 未执行、未消耗新次数 | 需用户另行授权 |

正常夹具 72 条已 ACCEPTED 后继中，71 条已在观察窗内 ACTIVE，1 条尚未到 start；
完成窗内 cohort 为 71/71，不能将右删失 pending 算成失败，也不能称真实在线供应率 100%。
最终命令调用生产限幅/海面保护，最大速度约 2m/s、切换步长约数值舍入误差；
它仅检查共速平衡输入，真实机体跟踪、非平衡命令平滑和飞控执行必须以新 F2 诊断/ULog 验收。

## 十五项回答

| 问题 | 当前结论 |
|---|---|
| 1 FOLLOW 稳定快速接近 | 进入判据已验证；本轮没有实际接近时间 |
| 2 MINCO 首次合法接管 | 离线生产 Receiver 协议通过；本轮实际接管未知 |
| 3 真正连续控制 PX4 | 尚无新实测证据 |
| 4 历史反复切换是否解决 | 协议缺陷和预算饥饿已修复；实际切换改善未验证 |
| 5 合法后继供应率 | 解析夹具成熟 cohort 71/71；实际未知 |
| 6 450ms 是否频繁回退 | 正常夹具 0 次；历史 131 次 lease 相关；本轮实际未知 |
| 7 125ms 是否满足 | 保留原期限并拒绝过期；实际供应持续满足未知 |
| 8 在线超时是否减少 | 第三条同输入恢复供给；前三条中仍有 1 次截止失败，不能给在线 P95 |
| 9 实际参考高度 | 夹具约 1.4e-13m 数值误差；实际未验证 |
| 10 实际参考跟踪误差 | 无本轮实际 RMSE，不用解析夹具替代 |
| 11 F2 实际验收 | 未执行 |
| 12 F3 实际验收 | 未执行；须真实 F2 完整通过及单独授权 |
| 13 优于原 FOLLOW | 尚不能作性能优势结论 |
| 14 已解决/未验证 | 见故障修复报告；实际供应尾延迟、跟踪和高度仍待 F2 |
| 15 可进入创新研究 | 可以准备离线研究；实际闭环基线尚不能封版 |

## 下一轮唯一目录与命令

任务书阶段 F 明确要求新次数授权。下列命令仅供授权后执行；当前未运行，也未创建尝试根。

```bash
cd /home/qin/data/uav_usv_mpc
set -eo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/p52_minco_sitl.py \
  data/experiments/20261013_p52_closed_loop_01/f2_h16_01 \
  --scene f2 --authorize-f2 \
  --readiness docs/current/audit/p52_follow_minco/readiness.json \
  --qualification docs/current/audit/p52_follow_minco/qualification.json
```

每个 scene 持久记录最多一次，F2 先独立 FOLLOW 稳定再接管，接管后有界观察 25s。
运行前重查环境冲突、资格哈希、唯一发布者和健康条件；任何源码改变会使资格失效。
F2 未通过不自动重复，不自动进入 F3；可以继续离线归因修复。
若 F2 完整通过且获得 F3 单独授权，使用同命令将输出改为 `f3_h16_01`，
scene 改为 `f3`，授权选项改为 `--authorize-f3`，观察 60s；入口另检查实际 F2 全部指标。

本轮没有自动 commit/push、切换/删除分支或修改旧仓库。
