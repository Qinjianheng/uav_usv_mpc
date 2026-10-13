# P53 全闭环代码与实测审查

2026-10-13。用户要求全面检查并允许结构调整；失锁搜索只是例子，不预设为根因。
当前HEAD5a783ca，所有原始轮次保持。R9新F2/F3已完成且清理，尚未通过全面修复后的新验收。

## 分层证据

| 层次 | 当前证据 | 结论/待办 |
|---|---|---|
| 图像/pose同步 | R4、R9 F3各1199条有效RGB-D，MAPPED_EXACT；无pose括区间越界/未来/倒序 | 本组记录未发现需改几何或时间戳的缺陷 |
| KF/预测 | source保持图像采集戳；R9观察发布延迟19.6～79.2ms，较R4的14.5～46.7ms增大 | 分离测量新鲜度与视觉锁；不据此盲调Q/R或换模型 |
| 视觉/任务状态 | R9全部60s仅1条locked=false而visible=true，约49.6ms；3D观测全有效 | KF短时不新鲜会清视觉锁并输出REACQUIRE；需拆分语义 |
| 名义MINCO接收器 | plan10时prediction688的图像年龄128.096ms；prefix VALID、视觉locked/visible true | 正确撤销过期执行，但错误标成TARGET_NOT_LOCKED；旁路689到达不证明Tracker本地交付 |
| 后继调度 | F2计划113后，114有效worker预算26.81ms而耗时33.39ms；115预算41.01ms而耗时43.76ms | 调度cost hint经等待衰减到15/12ms，低估近期真实路径成本；需修复并回放 |
| CPU/日志 | R9 solver中位36.13ms/R4为24.13ms；完整周期71.05ms/R4为53.65ms | 单次跨运行比较不足以将运行变慢归因于前馈；需区分实际成本估计、ROS回调与日志开销 |
| 执行控制 | 前馈仅显式名义ACTIVE水平轴，竖直NaN；最终速度/加速度整形受约束 | F3参考跟踪0.481→0.233m、两次偏差退出消失；F2连续性仍回退，不能总体宣称修复完成 |
| 实验工具 | 启动顺序、GUI world、旁路采集延迟已有独立修复与反例 | 观察300ms与在线执行125ms分开记录，不能互相充当资格 |

## 必须正确区分的两种事件

R9 F3 plan10的TARGET_NOT_LOCKED回退发生于1791894295.1609073；同刻视觉仍锁定。
Tracker随后debug使用prediction688，图像年龄128.096ms，执行context.visible=false。
约下一控制周期context恢复。现有`_follow_context`把native/预测新鲜度与视觉锁合成visible，
`context_errors`统一报TARGET_NOT_LOCKED，使输入等待被误读为视觉搜索。

另一条1791894318.611247的REACQUIRE由KF source年龄125.414ms触发；二维visible仍true，
下一诊断1791894318.660881重新锁定。现有视觉状态在`not kf_fresh`时清_locked。
当前记录未保存原始TargetBearing，不能补造无效RGB帧原因；也无证据表明发生突兀旋转或骤停。

两次F3回退恢复7.939s/12.720s，包括冷却、稳定与合法探测。先修复状态语义及供应原因，
暂停曾提出的统一缩短恢复等待策略。

## 文献对照（用户提供，本地PDF只读）

1. Wang等，2022，Geometrically Constrained Trajectory Optimization for Multicopters。
   PDF11页V-D采用积分罚函数处理轨迹约束；17页VII-B明确有限采样/罚函数与严格零违反的局限。
   MINCO的低维参数化和优化效率不自动解决ROS交付时序、所有权、或控制跟踪。
2. Gao等，2024，Adaptive Tracking and Perching for Quadrotor in Dynamic Scenarios。
   PDF4页III-A将定位/估计预测、约束、轨迹生成、控制分层；5页IV将距离、角度和遮挡作为跟随规划指标；
   9页Algorithm1以可见观察点和路径初始化后优化，不能把其path SearchPath当失锁yaw搜索算法。
   16页VIII-A报告其DJI/标签与差分GPS融合系统20Hz重规划，这不证明本仓库应直接调到20Hz。
   本组记录完整目标FOV100%，目前不应以新增遮挡优化替代时效与恢复结构修复。

文献路径为用户指定Zotero的QFIQJHY8与ZVS7T6XB PDF，正文与相关页已提取，图示已检查。
文献只作设计参考，实际结论来自本仓库封存源码和日志。

## 后续修复顺序

1. 修复成本估计在等待时抹去近期真实成功路径成本的问题；保留成本异常后的恢复能力。
2. 分离RGB锁定、3D状态可用性、预测/执行准入与撤销原因；保留过期轨迹立即撤销和新鲜输入完整验收。
3. 用明确故障反例和固定观察窗口验证。仅基于实际证据决定是否需要视觉搜索/恢复算法改动。
