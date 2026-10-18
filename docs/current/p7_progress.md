# P7：BCTRA＋MINCO FOLLOW 与 IBVS＋PNG

2026-10-17。本轮已完成指定 SITL 场景下的 P7.0～P7.3；红球仍只验证仿真感知接口，不外推到真实无标记 USV。

2026-10-18 的显式半径捕获与三种变速验证见 [P7 半径捕获](p7_capture.md)。下文保留 10 月 17 日安全接近阶段的结果与边界。

## 使用

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/build_workspace.sh
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
python3 scripts/p7_follow_sitl.py data/experiments/p7_manual_01 --manual
```

每次使用新的实验目录。该入口创建隔离的 DDS/Gazebo 会话和现有 SITL 执行授权，加载 `p7_guidance.yaml`：

- 1～5：X 前选择直线、八字、变速八字、S、Z；沿用现有参数服务 ACK。
- X：起飞、视觉锁定、原 FOLLOW 初始接近、自动 MINCO FOLLOW。
- Y：仅新鲜导航与当前锁定的 FOLLOW 可接受；立即撤销 FOLLOW 轨迹，进入 IBVS/PNG 安全接近。
- Q：发送任务中止并清理本会话；R：沿用原启动器的干净重启。

手动会话上限 280 秒。直接 ROS launch 仍须具备现有隔离 SITL 授权，不应禁用该保护。手动控制台未另加一轮飞行；自动 C/D 使用相同配置和控制代码验证。原 baseline/IMM 研究入口、P5/P6 代码与实验均保留。

## 控制与关键变更

| 位置 | 结果 |
|---|---|
| `tracking/maneuvering_target_predictor.py`、`target_prediction.py`、`target_predictor_node.py` | 从本机 R11 完整源码快照恢复 BCTRA；原 KF 未改；4 s/0.1 s 预测，原采集戳、原状态戳与 125 ms 有效期 |
| `modular_intercept.launch.py`、`p7_guidance.yaml` | 主预测器互斥选择 BCTRA/IMM；P7 默认 BCTRA，普通研究入口保留 IMM 默认 |
| FOLLOW 配置 | R11 P47/C、qva、H1.6、5 Hz、adaptive、无 refinement、P44/D 后备与水平加速度前馈；保留后续安全修复 |
| `control/p7_visual_guidance.py` | 纯计算 IBVS、水平 PNG、有界纵向接近与实际 dt 积分 |
| Tracker、MissionManager、旧截击 Planner | Y 撤销旧执行权；只有 Tracker 发布 PX4 setpoint；P7 不求解/发布旧截击轨迹，也不接受其状态转移 |
| `scripts/p7_follow_sitl.py`、`p7_follow_analysis.py` | 单轮有界运行、源/config 指纹、原始记录、离线曲线、仅清理本任务进程 |

真实相机 FLU 射线给出 `e_u=-y/x`、`e_v=-z/x`，等价于标定针孔模型归一化图像误差。IBVS 独占偏航：死区 0.03，指令上限 0.6 rad/s、变化率 1.2 rad/s²；不叠加旧 LOS 偏航前馈。垂向误差记录用于检查，固定相机没有云台，垂向保持 -5 m NED 并经过原海面安全保护。

PNG 使用 N=3，积分器速度/加速度上限为 6.2 m/s、2 m/s²；期望纵向闭合速度上限 0.75 m/s，目标水平间距 4.5 m。接管时继承的实际闭合速度可以更高，随后按上限减速，不能将其瞬间置零。近安全间距或非闭合时使用有界速度匹配；这不属于 PN 横向加速度分支。C 与八字诊断分别实际执行了 88、104 个 PNG 周期。

保留原导航 sample epoch、50 ms 因果 bracket、125 ms 时效、450 ms 执行租约、激活门限、Q/R、失锁恢复和 Offboard 保护。失效图像不继续驱动 IBVS。真值只进入评价和离线分析；P7 保持高度并在安全间距受控接近。

## 实际结果

| 实验 | 实际证据与核心结果 |
|---|---|
| A 直线 FOLLOW | MINCO 100%，连续 24.948 s、零回退；1250 条 native 状态；水平观察点 RMSE 0.0534 m，高度 RMSE 0.0111 m |
| B 八字 FOLLOW（复测） | MINCO 100%，连续 59.950 s、零回退；3000 条 native 状态；水平观察点 RMSE 0.7524 m，高度 RMSE 0.0141 m，执行参考 3D RMSE 0.2154 m |
| C 直线制导 25 s | 实际水平距离 14.43→4.56 m；全程可见、连续 P7 控制；Y 速度指令步长 0.100 m/s；实测水平加速度最大 1.964 m/s² |
| D 完整 X→Y | Y 前实际 MINCO 连续观测 8.736 s；之后距离 5.06→4.59 m；Y 速度指令步长 0.0148 m/s；无旧控制权恢复 |
| 低速八字偏航诊断 25 s | 距离 14.46→4.52 m；图像水平误差峰值 0.331，随后收敛；偏航指令峰值 0.301 rad/s，PX4 heading 的 0.2 s 差分峰值 0.310 rad/s；横向指令加速度峰值 1.587 m/s²，实测水平加速度最大 2.164 m/s² |

C/D/偏航诊断全部为单一 BCTRA、单一 Tracker、速度接口，未混发位置/加速度 setpoint；可见性与 P7 控制占比均为 100%，旧截击/旧 FOLLOW 活动事件为零。全部成功轮清理无错误、无残留。最终在线源码与这三轮源快照逐文件一致，见交付包 `source_verification.json`。

速度 setpoint 时间差计算的瞬时加速度最大约 2.01 m/s²，包含回调与时间戳相位误差；它与积分器的 2 m/s² 上限、实测 PX4 加速度是不同量。未修改原物理安全要求。八字 FOLLOW 仍是约 0.75 m 观察点误差，不宣称厘米级跟随。

首轮八字 `20261017_p7_f3_01` 失败记录完整保留：29 ms 接收验证占用单线程回调期间，125 ms 门限正确撤权；目标仍可见，但旧 P53 观察器把 `INPUT_WAIT` 按 0.2 s 失锁停止。P7 观察器按控制器已有 0.45 s 输入等待窗区分，物理/时效门限不变。成功复测未触发该分支；接收回调调度仍可能引发合法回退，不声称消除了所有时序风险。

## 曲线和验证

- FOLLOW：[直线](../../data/experiments/20261017_p7_f2_analysis_01/follow.png)、[八字](../../data/experiments/20261017_p7_f3_analysis_01/follow.png)。
- 制导：[直线接近](../../data/experiments/20261017_p7_c_analysis_01/guidance.png)、[八字 IBVS/PNG](../../data/experiments/20261017_p7_yaw_analysis_01/guidance.png)。
- 完整任务：[X/Y 轨迹与所有权](../../data/experiments/20261017_p7_d_analysis_01/xy_mission.png)、[Y 前后指令/实际速度](../../data/experiments/20261017_p7_d_analysis_01/handover.png)。
- 每个分析目录保留指标 JSON、CSV 和 SHA256；原始 flight 目录保留全参数、源码快照、ROS/native 记录和 ULog。
- [交付验证](../../data/experiments/20261017_p7_delivery_01/)：263 项受影响核心测试通过；五模式/实际 ROS 参数服务 ACK 的 55 项测试通过，合计 318 passed。最初节点测试受只读 ROS 日志目录限制，已改用独立 DDS 域 191 和 `/tmp` 日志完成实测。
- 初始四包构建、集成两包构建、项目 flake8、shell 语法、Git 空白检查通过；`ros2 pkg prefix uav_control` 指向本工作区。

按用户要求单智能体执行并自审，未启动独立审查智能体。分支 `codex/p7-visual-guidance` 保留阶段提交，不自动推送。尚未对其余三种变速模式做完整闭环飞行；五种模式的选择、启动与状态接口已验证。下一步优先扩展这些场景，再单独研究更小八字观察点误差或最终接触包络。
