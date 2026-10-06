# 仓库协作约束

- 当前入口依次为 `README.md`、`docs/current/README.md`、`docs/current/architecture.md`、`docs/current/baseline.md`、`docs/current/environment.md`。历史入口是 `docs/archive_index.md`，历史报告不作为当前配置事实。
- 本工作区为 `/home/qin/data/uav_usv_mpc`；先读当前文件、Git 状态和终端输出。`origin`/`github` 的 fetch/push 都应指向 `https://github.com/Qinjianheng/uav_usv_mpc.git`。
- 当前仍为原 BCTRA/MINCO/tracker 速度控制，MPC 尚未实现。整理任务不得顺带改动控制代码、测试、运行配置、模型或 PX4 补丁；算法修改另立明确任务。
- 主 predictor/tracker 使用 `/tracking/target_state`。`/target/state` 真值只作仿真评价、显式 shadow 诊断和离线分析，禁止进入在线几何反馈/预测/规划；评价终态结束任务/暂停仿真与真值在线纠偏须区分。
- 红球只验证仿真感知接口，不宣称已验证无标记非合作 USV。保留图像采集戳、因果 pose history 等待、显式超时拒绝和原始导航 sample epoch。
- 每次只解决一个主要问题；禁止固定轴偏置、经验延迟、真值校正、盲调 Q/R 或放宽捕获半径。
- `data/baselines/20261006_pre_mpc/` 为冻结证据，原始 CSV/YAML/JSON/JSONL、报告、指纹和 PNG 不改写。新增分析输出放在独立目录，新增实验选入新的基线子目录，附来源和 SHA256。
- 每轮 `*_config.yaml` 是 evaluator 快照；完整参数另存于基线包 `config/`。不要补造失败轮未记录的完整配置。
- 新实验、PX4 参数备份与视频在 `data/experiments/` / `data/videos/` 默认忽略。忽略规则不取消已有跟踪；移出文件必须先归档、校验，并更新逐文件清单和恢复入口。
- 保持原仓库 `/home/qin/data/uav_usv` 只读。持久本地归档位于 `/home/qin/data/uav_usv_mpc_archive/20261006_pre_mpc/`；本机独有文件只能从该归档恢复，不能假定 Git 有备份。
- 不使用 `git add -A`，不自动提交视频、生成目录或所有实验；明确列出暂存路径。`sync_github.sh` 会全量暂存并推送，不作为默认提交方式。
- ROS setup 前使用 `set -eo pipefail`；构建脚本从脚本位置推导路径并允许 `UAV_USV_WS` 覆盖。直接 launch 须显式设置新工作区/日志目录。
- pytest 从 `src/uav_control` 运行；项目 flake8 范围为 `src/uav_control/uav_control src/uav_control/test scripts`，原始历史证据工具按字节保存，不格式化。提交前跑必要测试、shell 语法、构建及 `git diff --check` / `git diff --cached --check`，确认 `ros2 pkg prefix uav_control` 指向新工作区。
