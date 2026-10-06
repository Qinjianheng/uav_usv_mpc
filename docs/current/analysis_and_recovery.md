# 分析入口与历史恢复

开发目录只留四轮精选 CSV/vision CSV、配置、summary 和关键证据。历史大体量录制、旧报告和输入仍在原目录 `/home/qin/data/uav_usv` 及持久归档中；逐文件路径映射见 [清单](audit/retention_manifest.tsv)，当前包映射见 [manifest](../../data/baselines/20261006_pre_mpc/manifest.json)。

## 当前基线可用入口

```bash
cd /home/qin/data/uav_usv_mpc
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
mkdir -p /tmp/uav_usv_mpc_analysis
python3 -m uav_control.evaluation.vision_error_analysis \
  /home/qin/data/uav_usv_mpc/data/baselines/20261006_pre_mpc/*_vision.csv \
  > /tmp/uav_usv_mpc_analysis/vision.json
python3 scripts/p8_visual_bias_analysis.py \
  --vision data/baselines/20261006_pre_mpc/modular_intercept_20261006_095151_101120_mission_1_vision.csv \
  --output /tmp/uav_usv_mpc_analysis/visual_bias.json
```

四轮 vision 分析实跑有 1249/713/714/712 个观测，其中有效且有真值用于离线评价的观测为 1117/630/630/646。结果读取范围、样本有效条件和时刻见原分析实现；这些是全程统计，不作为同窗口系统偏差的因果结论。

结果与恢复次数直接读取每轮 `*_summary.json`；末端速度/冻结核对读取包中 `evidence/terminal_cruise_*`。已有 `intercept_outcome_analysis` 只选 `phase == 'intercept'` 的旧格式行，当前 modular CSV 使用 `MINCO_TRACKING` 等状态，实跑会输出“No experiment CSV found”；`usv_estimation_analysis` 的旧 camera/KF 字段在当前主 CSV 已拆到 vision 日志，输出的 0% 不能当作当前感知失效证据。本轮保持这些历史入口和源码原样，当前数据使用上面的 vision 入口与原 summary。

## 恢复一般历史入口

在独立临时目录恢复需要的文件，避免向开发目录重新塞入整套历史。下面 `mktemp` 每次新建目录：

```bash
cd /home/qin/data/uav_usv_mpc
HISTORY_WS=$(mktemp -d /tmp/uav_usv_history.XXXXXX)
git archive 85e13461 -- scripts src data/experiments/baseline_legacy \
  | tar -x -C "$HISTORY_WS"
source /opt/ros/humble/setup.bash
source /home/qin/data/uav_usv_mpc/install/setup.bash
ros2 run uav_control intercept_outcome_analysis \
  "$HISTORY_WS/data/experiments/baseline_legacy"
```

旧分析入口对恢复后的 legacy 数据实跑通过。其他历史工具若接收 `--csv`、`--root` 或输入目录，指向相应恢复路径即可。默认读取脚本所在根目录的工具，需要在恢复目录运行对应脚本；外部 ULog、bag 等输入仍须提供其原始文件，清单不会补造它们。

P4 几何分析在导入时读取 `p3_20260927_pose_counterfactuals.json`，执行时再读取其中九个 `source_csv`，仅恢复 JSON/汇总 CSV 不够。可恢复这组输入：

```bash
git archive 85e13461 -- scripts src \
  data/experiments/current/p3_20260927_pose_counterfactuals.json \
  data/experiments/current/p3_20260927_pose_counterfactuals.csv \
  'data/experiments/current/vision_static_capture_20260923_*.csv' \
  | tar -x -C "$HISTORY_WS"
python3 "$HISTORY_WS/scripts/vision_p4_geometry_gate_analysis.py"
```

本轮按 JSON 的九个 source 路径实际恢复并跑通；常量数组的 Spearman 相关分析可能给出原有 constant-input warning。输出写在恢复目录，原仓库和基线包保持原样。

## 重放原始末端速度工具

三份 JSONL 和原工具已移出开发目录，来源路径在基线 manifest 的 `external_recordings_and_tools`。示例恢复首次成功，其他两轮只替换运行名：

```bash
git archive 85e13461 -- src \
  data/experiments/current/y_intercept_20261006_terminal_cruise_user_phase_1.jsonl \
  docs/tracking/y_intercept_evidence_20261005/terminal_speed_audit.py \
  | tar -x -C "$HISTORY_WS"
export HISTORY_WS
python3 - <<'PY'
import os
from pathlib import Path
root = Path(os.environ['HISTORY_WS'])
p = root / 'docs/tracking/y_intercept_evidence_20261005/terminal_speed_audit.py'
q = p.with_name('terminal_speed_audit_scratch.py')
q.write_text(p.read_text().replace('/home/qin/data/uav_usv', str(root)))
PY
python3 "$HISTORY_WS/docs/tracking/y_intercept_evidence_20261005/terminal_speed_audit_scratch.py" \
  terminal_cruise_user_phase_1
```

仅临时副本替换历史绝对路径，计算算法不变；原工具、原录制和已提交基线不改写。本轮首次成功速度重放 JSON 与冻结证据 JSON 完全一致。

## 本地归档与独有文件

归档含 `repository/<原始路径>` 与 `metadata/`。恢复到独立目录：

```bash
ARCHIVE=/home/qin/data/uav_usv_mpc_archive/20261006_pre_mpc/repository_before.tar.gz
RECOVERY_DIR=$(mktemp -d /tmp/uav_usv_recovery.XXXXXX)
tar -xzf "$ARCHIVE" -C "$RECOVERY_DIR" \
  repository/data/experiments/current/modular_intercept_20261006_102846_255769_mission_1.csv
```

校验恢复文件时与 [逐文件清单](audit/retention_manifest.tsv) 的 SHA256 比对。该轮另外三个文件和 `enu_caLQn2` 的两份参数 BSON 同样按源路径恢复；它们不在原仓库 Git 中。归档本身的 SHA256 见 [归档核对](audit/archive_verification.json)。
