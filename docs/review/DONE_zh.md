# 已完成清单（DONE）—— 按主题

**文档职责**：记录"已经做完并且有实测验收"的事情（含 commit 与关键数字）。
未完成的在 `TODO_zh.md`；每条缺陷/特性的现象→原因→修正→结果在 `DEFECT_LOG_zh.md`。

**维护约定**：见 `templates/DOC_TEMPLATE_zh.md`。完成任务时从 TODO 迁到这里，
**保留日期与 commit**；只写结论与验收数字，过程细节写进 DEFECT_LOG。

## 更新记录

| 日期 | 更新内容 | 相关 commit / 分支 |
|---|---|---|
| 2026-09-20 | 初版：按主题整理 `origin/main..main` 的 21 个提交；补"当前基线实测"一节 | `docs/review-spec` |

---

## 0. 当前基线（2026-09-20 实测）

* `main = 40744b5`，领先 `origin/main` 21 个提交，工作区干净（上一轮的调试改动已
  存到 `wip/old-ckpt-arm-coupling-debug @ 8de683c`）。
* **编译**（容器 `m20_piper_ros`）：

  ```bash
  source /opt/ros/humble/setup.bash
  colcon build --packages-up-to m20_sdk_deploy \
    --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON
  # → Summary: 2 packages finished [19.3s]，无 error
  ```

* ⚠️ **`main` 上的策略是旧 checkpoint**：`policy/history_adaptation_full.onnx`
  对应 `M20_adjusted` 资产、`policy_obs 86 / history 10×77 / action 23`。
  最新训练 run（`2026-09-20_00-50-31`，`M20_Piper_own`）是
  `83 / 10×70 / 16`，**尚未接进部署侧**（见 `TODO_zh.md` P0-1）。

---

## 一、Sim2sim 骨架（2026-08-20，`9095957`~`ee5c264`）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-08-20 | 引入 M20+Piper 的 MJCF 模型与 mesh（`M20_Piper_description/`） | — | `9095957` |
| 2026-08-20 | MuJoCo 仿真支持 24 DOF（16 腿轮 + 6 臂 + 2 夹爪），双话题：腿部走官方 `/JOINTS_CMD`，臂走新增 `/ARM_JOINTS_CMD`；模型补 `imu_site`/`base_site` 与 framequat/accelerometer/gyro 传感器、按训练配置补力矩限幅（腿 76.4 / 轮 21.6 / 臂 100 / 夹爪 10） | 仿真能加载 24 DOF 并自动把机器人落到轮子着地高度 | `efdc2d8` |
| 2026-08-20 | WBC 部署框架：`UserCommand` 增加机身高度/俯仰/横滚、EE 增量、夹爪、`ee_reset`；新增 `PiperArmInterface`（8 DOF，单位标定）；新增第一版 `M20PiperPolicyRunner`（obs 79、action 16）；键盘加姿态/numpad EE/夹爪/复位 | — | `0b8acbb` |
| 2026-08-20 | 新增 Piper 臂 IK 节点 `arm_controller.py`：50 Hz 绝对 EE 目标、DLS（λ=0.01）与训练 `DifferentialIKController` 对齐；MDH 参数优先取 `pyAgxArm`，无 SDK 时用内嵌 fallback；订阅 `/ARM_TELEOP`、发布 `/ARM_JOINTS_CMD` 与 `/ARM_TELEOP_STATE` | — | `66bcab1` |
| 2026-08-20 | 支持 `ActorCriticHistory` 策略（history encoder + actor）的导出与部署：`scripts/export_history_policy_onnx.py` 从 checkpoint 重建 TCN + actor | 导出的 ONNX 输入 `obs`/`obs_history`、输出完整动作 | `ee5c264` |

## 二、sim2sim 稳定性与安全网（2026-09-04，`e5e1917`）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-04 | **SIM2SIM 恒等标定接口** `M20SimInterface`（腿轮 dir=1、offset=0），CMake 加 `-DSIM2SIM=ON`；不加这个选项会走真机 `M20Interface`，仿真坐标系会被多圈 offset 标定污染 | 加 `-DSIM2SIM=ON` 后仿真坐标系与 MJCF 一致 | `e5e1917` |
| 2026-09-04 | **物理子步**：原 1 ms 单步让 USD 派生的 MJCF 出现 ~500 Hz 机身震荡（IMU 读到 ±2 rad/s 假角速度），改成 0.2 ms × 5 子步，控制 tick 仍是 1 ms、反馈仍 200 Hz | 机身震荡消失，话题频率不变 | `e5e1917` |
| 2026-09-04 | **执行器 armature**：臂/夹爪 0.01、轮 0.00243216（按训练 actuator 配置），避免显式 PD 下机械臂/夹爪失稳 | 臂/夹爪不再发散 | `e5e1917` |
| 2026-09-04 | **History 关节顺序修正**为"12 腿 → 4 轮 → 6 臂 → 2 夹爪"（对齐当时在用的 `M20_adjusted` 老模型的 `find_joints(".*")` 实测输出） | 见 `DEFECT_LOG_zh.md` DEF-005 | `e5e1917` |
| 2026-09-04 | **动作限幅 ±3**（轮速 ≈ ±15 rad/s）作为部署安全网，避免原始 ONNX 输出发散导致的轮子 runaway / NaN；清理高频打印 | 输出有界、不再 NaN | `e5e1917` |

## 三、仿真功能与遥操作（2026-09-04，`b464ce8`~`7b51a35`）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-04 | 新增 `/reset_sim` 服务（`std_srvs/Empty`）：恢复默认站立姿态、清零速度与腿/臂命令缓冲 | `ros2 service call /reset_sim std_srvs/srv/Empty` 可用 | `b464ce8` |
| 2026-09-04 | `/ARM_TELEOP` 协议重设计（增量 + 绝对偏移两种模式）；抽出独立的 `arm_teleop_node.py`（键盘 hub），`rl_deploy` 不再转发臂命令 | 臂可在进 RL 之前单独动 | `5c92378` `a48c415` |
| 2026-09-04 | 内置 XLeVR，新增独立 `/VR_TELEOP` 节点；VR 接管腿/机身（与键盘仲裁：VR 活跃时键盘忽略并打印原因） | VR 按下 B 启动，掉线 0.5 s 自动交还键盘 | `774ac91` `c186773` `7b51a35` |
| 2026-09-04 | VR 的臂命令走 `arm_teleop` hub；VR 的 X/Y 触发 `/reset_sim` 并清空策略 history 窗口 | 复位后 history 整窗重填，不喂"摔之前的历史" | `994d1eb` `186cdd9` |
| 2026-09-04 | **真机臂传输**：`arm_real_adapter.py` 把 `arm_joint1..6 + gripper` 映射到 `agx_arm_ros` 的 `/control/joint_states`（夹爪宽度 = `q6 − q7`），并把 `/feedback/joint_states` 映回 `/ARM_JOINTS_DATA` | 话题级验证通过；真机未验（见 TODO P2-2） | `b884d90` `cccbef2` |

## 四、状态机与易用性（2026-09-08）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-08 | 按键被状态守卫忽略时打印原因；idle 拒绝站立请求时报告原因 | 日志可读 | `979636d` `ba36fc3` |
| 2026-09-08 | `safe_control_mode` 在所有来源恢复后清零 | 不会卡在安全模式 | `c9e08e5` |

## 五、策略观测一致性（2026-09-16）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-16 | `last_action` 观测改用"训练口径"的原始动作（`env.action_manager.action`，被 `clip_actions=100` 截断），而不是换算后的关节目标 | 见 `DEFECT_LOG_zh.md` DEF-002 | `40744b5` |

## 六、上一轮未提交的调试工作（已另存分支）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | 针对**旧 checkpoint** 的臂→轮耦合诊断：`M20_LAST_ACTION_MODE`（applied/raw/processed）、`M20_IK_FEEDBACK`（mean/noisy/zero/hold）、`M20_FREEZE_EE_GOAL`/`M20_FREEZE_ARM_OBS`、`M20_ARM_DEFAULT_TRAINED`、`M20_ZERO_CMD_WHEEL_BRAKE`；history 关节序从"分组序"重写；`arm_controller` 的 `ee_goal` 改发 root（body）系；README 大改 | `last_action=raw` 时仿真里能站住，`processed` 时 ~3 s 翻倒（轮速钉在 ±15 rad/s）；这些现象记录在分支上 | `wip/old-ckpt-arm-coupling-debug @ 8de683c` |

> 该分支**不并入 `main`**：它的结论（尤其是"history 用分组序"）来自另一个资产
> （`M20_adjusted`），对新 checkpoint 需要重新判定（`TODO_zh.md` P0-2）。

## 七、基础设施

* `scripts/export_history_policy_onnx.py`：从 checkpoint 重建 TCN history encoder + actor，
  导出成部署态 ONNX（**未提交的后续版本**在训练仓库侧，导出脚本已升级为
  `export_deploy_policy.py`，同时出 `policy.pt` / `policy.onnx` / `policy_layout.json`
  并带数值自检）。
* `scripts/probe_policy_nominal.py` / `probe_policy_ee_goal.py`：离线探针，
  喂标称/扫掠观测看 ONNX 输出的量级与敏感性（当前针对旧 checkpoint 的 86 维布局）。
* `README.md`：记录了编译选项、4~5 终端运行方式、按键表、臂工作空间限位、
  遥操作约定、真机传输方式与容器创建命令。
