# P6.1 五种 USV 运动模型

已实现纯 Python 运动模块、运动节点参数选择 ACK，以及独立 ROS 服务测试。默认仍为原 `figure_eight`；未改 `FigureEightTrajectory`、MissionManager、Tracker 或原基线。本文的 120 秒结果是离线运动数学与节点接口证据，不是五模式闭环飞行结论。

源码为 `src/uav_control/uav_control/usv_motion_profiles.py` 与 `moving_target.py`。纯接口 `UsvMotionProfile(mode, MotionParameters(...))` 的 `state()` / `advance(dt)` 返回 `MotionSample`：当前位置、速度、加速度、连续航向、转弯率、曲率、阶段、纵/横加速度和角加速度。没有向在线预测器或规划器发布未来路径。

| 编号 | `trajectory_type` | 数学与阶段 |
|---|---|---|
| 1 | `linear` | 原方向归一化、原纵向速度递增和半隐式位置积分；从静止加速至原 4 m/s |
| 2 | `figure_eight` | 原 40/20 m Gerono、原相位 π、原 +Y 初始方向、原弧长 RK4 和启动递增 |
| 3 | `variable_speed_eight` | 原八字几何；平滑变速，按 Simpson 弧长增量调用原空间 RK4 |
| 4 | `variable_speed_s` | 平滑周期航向与启动包络，真正连续左右曲率交替；按当前时刻速度积分 |
| 5 | `variable_speed_z` | 7 s 直线、5 s 左转、8 s 斜向、5 s 右转、7 s 直线，周期 32 s |

坐标为 NED，正 yaw 为顺时针；S/Z 的第一次转弯为负 yaw，从 +Y 向 +X 左转。八字航向做跨 ±π 展开。八字 `phase` 为原路径弧度，S 为周期相位弧度，Z 为周期内秒数，直线为累计秒数；评估阶段同时记录在 `stage`。

新模式速度先以五次 smoothstep 在 8 s 内从 0 升至 4 m/s，然后采用 `3 + cos(2π(t-8)/24)` m/s，在 2～4 m/s 间反复变化。速度及纵向加速度在启动连接处连续。S 的航向为 `yaw0 - 0.6 sin(2πt/24) smoothstep(t/8)`；Z 的转向使用五次航向插值，其航向、转弯率和角加速度在段连接处连续。

各新模式在启动时核验完整动力学包络，不通过则拒绝选择；不会通过在线截断速度/曲率悄悄改变路径。八字曲率与曲率弧长导数由解析几何计算，准入使用全路径 8192 点采样并加 0.1% 余量；该数值包络不是任意极端幅值配置的解析证明。S/Z 使用解析保守上界。

## 参数

原参数保留，包括位置、方向、水平速度、纵向加速度、转弯率、横向加速度、八字幅值及升沉。下列新参数在节点声明，可由独立 YAML 或启动覆盖配置；运动运行期间仅支持模式选择参数，其余运动参数要求重启，以免节点与 ROS 参数存储不一致。

| 参数 | 默认 | 含义 |
|---|---:|---|
| `maximum_horizontal_acceleration` | 3.2 m/s² | 总水平加速度上界；和原横向上界取同一保守场景量级，同时检查纵/横合成 |
| `maximum_angular_acceleration` | 0.35 rad/s² | 有限转弯角加速度场景界；高于旧八字约 0.177 和新默认 Z 约 0.185 的实际峰值，尚非真实 USV 辨识值 |
| `variable_speed_min` / `variable_speed_max` | 2 / 4 m/s | 新模式稳态速度范围 |
| `variable_speed_period` | 24 s | 速度周期 |
| `variable_startup_duration` | 8 s | 平滑启动时间；默认峰值纵向加速度 0.9375 m/s² |
| `s_heading_amplitude` / `s_period` | 0.6 rad / 24 s | S 航向幅值与周期 |
| `z_heading_change` | 0.8 rad | Z 直线与斜线之间的航向变化 |
| `z_straight_duration` | 7 s | Z 周期首尾各自直线时间 |
| `z_turn_duration` | 5 s | 每次平滑转向时间 |
| `z_diagonal_duration` | 8 s | 斜向直线时间 |

纯 API 的 `linear_vx` / `linear_vy` 对应节点原 `velocity_x` / `velocity_y`，`integration_step` 对应 `1/update_rate_hz`。其他字段与节点参数同名。新模式限制与路径参数独立；若降低上界或加大机动导致不可行，节点会给出明确的包络拒绝原因。

## 模式 ACK 与时间契约

选择接口是已有 `/moving_target/set_parameters`，请求只包含一个字符串 `trajectory_type`。例如：

```bash
ros2 service call /moving_target/set_parameters rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: trajectory_type, value: {type: 4, string_value: variable_speed_s}}]}"
```

成功/失败均返回 `SetParametersResult.successful` 和 JSON `reason`，包含 `mode`、`id`、`mode_id`、`started`、`ack`、`successful` 与可读 `reason`。`/simulation/impact/motion_mode` 使用 reliable / transient-local / depth=1 发布同一状态。启动前合法切换立即生效；重复同模式幂等；启动后切换其他模式拒绝，同模式仍返回成功 ACK。非法模式或不可行配置保持原模式。模式对象仅在 ROS 全部参数校验成功后提交。

ROS Humble 会清空成功参数回调的 `reason`，因此节点在公开 `set_parameters_atomically()` 提交后补充 ACK；标准 ParameterService 原样保留。`set_parameters()` 同样逐参数委托该入口。混合模式选择与运动参数更改的原子请求被拒绝，防止部分切换。控制台数字键、X 前 ACK 和 R 进程重启由外层集成，运动节点不改变 MissionManager 的 X/Y/R/Q 状态机。节点保留原 flight-ready 与 TAKEOFF 启动入口，Y/R/Q 不直接改变运动状态。

节点每次回调只取一个当前 ROS 时间戳。正向有效 `dt` 分成不超过名义积分步长的子步；正常 20 Hz 旧八字逐值不变。首次未锚定、相同时间、回退及大于 1 s 的不连续跳变均冻结并重新锚定；原生 Gazebo 暂停期间记录真实时刻，恢复后只积分实测间隔，不补名义时间。旧 `measured_motion_step()` 保留为历史兼容辅助，实际 timer 使用严格的 `elapsed_motion_step()`。

原 `z=z0+vz*t+0.15 sin(2π·0.25t)` 及解析垂向速度保留。`/target/state` 现在同时记录解析当前加速度；`/target/position`、`/target/velocity`、同次真值消息和 Gazebo 更新使用同一当前样本。渲染仍使用原 `z - gazebo_visual_height_offset`。`/simulation/impact/motion_truth` 的 JSON 仅供评价，记录当前样本与阶段、运动学分量、原始 `stamp_ns` 和 `evaluation_only=true`；没有未来计划或在线反馈用途。

## 实际验证

证据目录为 `data/experiments/20261014_p6_development_01/motion/`，含 RED 日志、GREEN 日志、五模式逐样本 CSV、七幅曲线、`summary.json`、源码指纹、生成脚本与 `SHA256SUMS`。

- 28 项初始数学测试先因模块缺失失败；随后通过。后续独立 NED 左转及零方向加速度用例先失败，再修正通过。
- 旧八字在 0.05、1、4、15、60、120 s 的预先冻结 `(x,y,vx,vy)` 浮点值完全相等；另对 120 s 全部 2400 个 `(x,y,z,vx,vy,vz,phase,time)` 样本做冻结 SHA256 字节精确回归。其源文件未改。旧线性启动、现有时间基和评价测试均通过。
- 120 s / 20 Hz 检查速度、总/纵/横加速度、转弯率、角加速度、确定性、变步长、时间异常和段连接。以 5 ms 独立位置/速度有限差分核对导数。
- 隔离 `ROS_DOMAIN_ID=73`、无 PX4/Gazebo 的真实 Node 与 DDS 服务测试通过，包括 GetParameters、SetParameters、成功 JSON ACK、启动后拒绝，以及晚加入订阅者接收到启动时 transient-local 状态。
- `green_motion_final.log`：132 passed；包括其他 ROS 参数验证器在末尾拒绝时，已准备的模式也不会提前提交。指定四个代码/测试文件按仓库 `.flake8` 配置验证，以及 `git diff --check` 通过。集成全量 pytest、构建及完整控制台/真实 Gazebo/SITL 由根任务统一验证。

| 新模式 | 8 s 后速度 m/s | 总水平加速度峰值 m/s² | 纵向峰值 m/s² | 横向峰值 m/s² | 转弯率峰值 rad/s | 角加速度峰值 rad/s² |
|---|---|---:|---:|---:|---:|---:|
| 变速八字 | 2～4 | 1.8368 | 0.9375 | 1.8339 | 0.4684 | 0.1727 |
| 变速 S | 2～4 | 0.9997 | 0.9375 | 0.5638 | 0.1641 | 0.0955 |
| 变速 Z | 2～4 | 1.1817 | 0.9375 | 1.1777 | 0.3000 | 0.1847 |

表中为默认参数离线全窗口采样峰值，含启动；浮点容差内速度上界为 4 m/s。没有因此宣称任意配置、真实无标记 USV 或复杂闭环已验收。
