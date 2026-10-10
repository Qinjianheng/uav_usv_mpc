# 前视 RGB-D、保留的 ToF 与 0.15 m 升沉场景

**当前按用户要求暂用前视 RGB-D（理想深度），ToF 实现和配置保留。末端重新验收修复及当前结果见 [RGB-D 验证](terminal_replan_rgbd_20261007.md)；早期带噪定位的失败记录见 [ToF 复验](tof_fit_validation_20261007.md)。**

2026-10-07 按用户最新要求，默认感知输入为前视 RGB-D，下视传感器暂时关闭。RGB 用于现有红球检测，渲染深度用于定位；切换 ToF 时才启用测距误差模型。现有 BCTRA/MINCO/tracker 框架保持，MPC 尚未实现。

当前 [baseline.yaml](../../src/uav_usv_bringup/config/baseline.yaml) 的目标升沉幅值为 **0.15 m**，峰峰值 0.30 m；频率仍为 0.25 Hz，周期 4 s。几何诊断配置同步使用该幅值。冻结的 [20261006_pre_mpc](../../data/baselines/20261006_pre_mpc/README.md) 仍为 0.05 m / 理想 RGB-D 成功基线，不能将它的成功直接视为当前场景结果。捕获半径、海面保护、KF Q/R 和控制参数不变。

## 感知链路

默认：Gazebo 前视 RGB-D 渲染 → 图像/深度桥 → 原理想深度 localizer → KF → 原预测/规划/控制。

可选 ToF：RGB-D 渲染 → RGB 桥 + 理想深度桥 → ToF 测距误差模型 → 带噪球面 localizer → KF → 同一预测/规划/控制。以下带噪深度话题和参数只描述显式启用的 ToF 模式。

- RGB：`/camera/front/image_raw`。
- 渲染器理想深度：`/camera/front/depth/ideal`，仅作 ToF 模型输入。
- 模拟测量：`/camera/front/depth/image_raw`，由 ToF 节点唯一发布，定位和 ToF 诊断均使用它。
- 图像采集戳、frame、640×480 / 20 Hz、相机外参和对齐方式保留。模拟器在径向距离上加噪、量化及丢失回波，再转换回现有定位接口所需的光轴深度；无效值为 NaN。
- 模型不订阅真值、目标状态或无人机姿态。定位仍使用因果姿态历史和原时间戳规则；真值只作评价及离线误差分析。
- 默认启动不桥接下视、不启动下视诊断。启动器生成临时模型副本，移除下视 sensor，保留外壳、质量、惯量、关节和前视外参；原双相机 SDF 不改写。

## 参数及真实硬件边界

[front_tof_simulation.yaml](../../src/uav_usv_bringup/config/front_tof_simulation.yaml) 是未标定测试配置：径向量程 0.25–25 m；逐像素高斯测距标准差 `0.01 + 0.001 × 距离[m]`；独立回波丢失概率 1%；量化步长 1 mm；随机种子 0。它们是可更改的工程假设，未对应某款实物相机。RGB 可见而深度超量程时，原系统只能获取图像方位，不能由该模型获得有效三维位置。

此模型覆盖测距接口和缺测行为，未模拟阳光、光子统计、多径、水面反射、材质反射率或 RGB/深度基线遮挡。海面在当前世界中仍为视觉平面。25 m 是仿真测试上限，不是已经验证的真机海面能力。

[Gazebo RGB-D API](https://gazebosim.org/api/sensors/7/classgz_1_1sensors_1_1RgbdCameraSensor.html) 描述渲染式 RGB/深度接口。真实设备量程随型号与条件变化，例如 [Basler blaze-101](https://docs.baslerweb.com/blaze-101) 标称 0.3–10 m；[ifm O3M 官方资料](https://www.ifm.com/binaries/content/assets/microsites/o3m-smart-sensor/en/ifm-3d-sensors-o3m-mobile-machines-gb.pdf) 描述更长距离的不同目标条件。后续需选择具体型号，标定内外参、模式、距离噪声与有效率，再验证户外海面表现。

检测器目前仍是红球接口测试，未实现无标记船体识别。本次仿真定位误差和捕获结果只能说明当前测试场景，不能证明真实非合作 USV 识别精度或普遍成功率。

## 启动与恢复

```bash
cd /home/qin/data/uav_usv_mpc
./scripts/build_workspace.sh
./scripts/uav_lab.sh --no-build
```

完整重新启动后生效；已有 Gazebo 中的传感器不会因 ROS launch 开关自动删除。仅直接 launch 会关闭下视桥与诊断，但物理 sensor 是否存在取决于已加载模型。

显式恢复仅前视 ToF 功能仿真：

```bash
UAV_USV_FRONT_DEPTH_MODEL=tof ./scripts/uav_lab.sh --no-build
```

恢复双相机理想深度模式（仍为当前 0.15 m 场景）：

```bash
UAV_USV_ENABLE_DOWN_CAMERA=true UAV_USV_FRONT_DEPTH_MODEL=ideal ./scripts/uav_lab.sh --no-build
```

复现冻结的 0.05 m 配置，须另外显式指定基线完整参数，不能只恢复相机开关：

```bash
UAV_USV_ENABLE_DOWN_CAMERA=true UAV_USV_FRONT_DEPTH_MODEL=ideal \
UAV_USV_EXPERIMENT_CONFIG_FILE=/home/qin/data/uav_usv_mpc/data/baselines/20261006_pre_mpc/config/baseline.yaml \
./scripts/uav_lab.sh --no-build
```

保留当前只前视、临时对照理想深度：`UAV_USV_FRONT_DEPTH_MODEL=ideal ./scripts/uav_lab.sh --no-build`。独立 ROS launch 可设置 `front_depth_model:=tof enable_down_camera:=false tof_config_file:=/绝对路径/配置.yaml`；须显式设置新工作区和日志目录，且与 Gazebo 模型开关一致。

初始失败与理想对照见 [相机初始验证](tof_validation_20261007.md)，带噪定位适配后的实际结果见 [闭环复验](tof_fit_validation_20261007.md)。原基线证据不改写。
