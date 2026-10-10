# 源码冗余清理（2026-10-06）

目标：清理已确认没有当前功能依赖的历史副本，不改变现有控制行为；对测试依赖、诊断职责和未来复用意图不明确的代码予以保留。清理前基点为 `c2ef5ae58a9219b54e9719a2d4f31e869d0bc888`，分支为 master。

## 已移出

| 原始路径（相对仓库） | 行数 | 依据 |
|---|---:|---|
| `src/uav_control/uav_control/guidance/intercept_planner_node.py.bak` | 822 | 编辑备份，不是 Python 模块；源码/测试/脚本无引用 |
| `src/uav_control/uav_control/archive/pure_pursuit_backup.py` | 269 | 旧版本已在构造首句抛出退役异常，无入口注册、导入或测试引用 |
| `src/uav_control/uav_control/archive/predictive_intercept_v1.py` | 550 | 同上；当前主 predictor 使用独立 tracking 模块 |

共移出 **3 个文件、1641 行、49,262 字节**，没有改写任何剩余运行实现。`archive/__init__.py` 保留原包命名空间；已有控制器、控制接口、任务状态机、测试、参数、模型、world、补丁及冻结基线保留。

删除前已核对四份内容完全一致：当前文件、原仓库文件、`85e13461` Git blob、持久归档对应成员。归档自身 SHA256 也重新核对。逐文件校验值、历史链接与恢复路径见 [补充清单](audit/redundancy_cleanup_20261006.json)。之前的 [整理清单](audit/retention_manifest.tsv) 描述 c2ef5ae 的布局；上述三项现在以补充清单为准，先前审计快照不改写。

## 本轮保留

- `trajectory_impact_sim.py`、`pure_pursuit.py`、`predictive_intercept.py`：当前飞行入口已退役，但回归测试仍引用；不删除实现或缩成壳。
- 主/shadow predictor：同源同参数，但 shadow 输出仍被 evaluator 用于独立诊断；不删除节点或改变日志。
- 重复时间转换、正数和向量检查：位于当前运行路径；这次不引入新公共模块或改动时间处理。
- `finite_horizon_intercept_planner.py`：是当前 FastMincoPlanner 的父类，属于必要依赖。
- PID、旧 rolling/solver、离线分析、手动起飞/位置监听工具：默认 launch 未调用不能证明其无用，保留未来复用与手动入口。

## 恢复

原仓库 `/home/qin/data/uav_usv` 保持只读，完整保留这些文件；Git 与持久归档也均可恢复。先恢复到独立目录，避免覆盖当前代码：

```bash
cd /home/qin/data/uav_usv_mpc
RESTORE_DIR=$(mktemp -d /tmp/uav_usv_source_restore.XXXXXX)
git archive 85e13461 -- \
  src/uav_control/uav_control/guidance/intercept_planner_node.py.bak \
  src/uav_control/uav_control/archive/pure_pursuit_backup.py \
  src/uav_control/uav_control/archive/predictive_intercept_v1.py \
  | tar -x -C "$RESTORE_DIR"
```

本轮已实际执行并核对三份恢复文件 SHA256。若需要从归档恢复，成员路径为 `repository/<上表路径>`，归档为 `/home/qin/data/uav_usv_mpc_archive/20261006_pre_mpc/repository_before.tar.gz`；校验值见补充清单。

## 验证

| 检查 | 实际结果 |
|---|---|
| 清理前后完整 pytest | 均为 836 passed、1 skipped、2 warnings |
| 项目 flake8、8 个 shell 脚本语法 | 全部通过 |
| 工作区构建 | 4 个包完成，退出码 0 |
| ROS 包前缀 | `/home/qin/data/uav_usv_mpc/install/uav_control` |
| 安装入口 | 15 个 console entry point 均可加载 |
| 内容保护 | 剩余 459 个源码/测试/配置/模型/补丁文件及 46 份冻结证据校验一致 |
| 文档与恢复 | 69 个当前相对链接有效；3 个文件实际恢复并通过 SHA256 核对 |
| 独立只读审查 | 未发现需修正的问题，可按当前范围提交 |

完整结果记录在补充清单的 `verification` 中。没有新增飞行实验；运行实现保持字节一致，飞行证据沿用冻结基线。
