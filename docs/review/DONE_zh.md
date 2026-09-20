# 已完成清单（DONE）—— 按主题

**文档职责**：记录"已经做完并且有实测验收"的事情（含 commit 与关键数字）。
未完成的在 `TODO_zh.md`；每条缺陷/特性的现象→原因→修正→结果在 `DEFECT_LOG_zh.md`。

**维护约定**：见 `templates/DOC_TEMPLATE_zh.md`。完成任务时从 TODO 迁到这里，
**保留日期与 commit**；只写结论与验收数字，过程细节写进 DEFECT_LOG。

## 更新记录

| 日期 | 更新内容 | 相关 commit / 分支 |
|---|---|---|
| 2026-09-20 | 初版：按主题整理 `origin/main..main` 的 21 个提交；补"当前基线实测"一节 | `docs/review-spec` |
| 2026-09-20 | 新增第八节：L3 测试基础设施（遥测 + 一键冒烟）+ 两条 sim2sim 修复（DEF-012/013） | `fix/sim2sim-bringup` |
| 2026-09-20 | 新增第九节：策略接口切到 83/700/16（布局驱动）+ L1 验收 + 走起来了 | `feat/policy-layout-v2` |

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

## 八、L3 测试基础设施 + sim2sim 进场修复（2026-09-20）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **遥测落盘**：仿真节点新增 `M20_SIM_TELEMETRY=<path>`（默认 200 Hz）+ `M20_SIM_TELEMETRY_PERIOD`，列含 `t / base xyz / quat / rpy / omega_b / q[24] / dq[24] / tau[24] / 四轮 z / 四轮接触力` | 12 s 跑出 2653 行（≈200 Hz），可直接算高度/倾角曲线 | `fix/sim2sim-bringup` |
| 2026-09-20 | **一键 L3 冒烟** `tests/sim2sim_smoke.py`：起仿真 + `rl_deploy`，用管道喂键盘走 idle→standup→RL（可选按住 `w` 前进），按契约文档的判据（高度 = `root_z − mean(轮 z) + 0.09`、倾角 = `acos(-g_z)`；摔倒 = 倾角 > 0.8 rad 或高度 < 0.30 m）打印 PASS/FAIL | `--mode hold` PASS；`--mode rl` 判据可复现（见下） | `fix/sim2sim-bringup` |
| 2026-09-20 | **修 DEF-012**：仿真的初始腿部保持命令从 `LEG_INIT["M20"]`（旧机型姿势）改成 `LEG_INIT["M20_Piper_own"]`（与初始 qpos 同源） | `--mode hold --duration 12`：height 0.095 → **0.498 m**，tilt 2.2° → 1.4°，FAIL → **PASS** | `fix/sim2sim-bringup` |
| 2026-09-20 | **修 DEF-013**：`/JOINTS_CMD`、`/ARM_JOINTS_CMD` 只接受 `control_word = kIndexMotorControl(4)` 的帧（`DdsInterface` 的构造函数会发 4 帧 `kp=0/kd=0/pos=0` 的电机状态帧，仿真侧原来把它们当零增益命令执行） | `--mode rl --duration 25`（旧 checkpoint）：修复前 t≈3.4 s 触发摔倒判据（height −0.271 m、tilt 147°）→ 修复后 height min **0.502 m**、tilt max **1.7°**、**PASS** | `fix/sim2sim-bringup` |

> 注意：这两条修复让 sim2sim 的"进场"（裸模型站立 → 站立 → 进 RL）达标了，
> **不等于**策略接口已经对齐 —— 策略侧仍是旧 checkpoint 的 86/770/23
> （`DEF-010` / `TODO_zh.md` P0-1）。

## 九、策略接口切到新 checkpoint（83/700/16，布局驱动）(2026-09-20)

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **策略产物**：删除旧 checkpoint 的三个 onnx 与三个旧探针脚本；新增 `policy/m20_piper_history_20260920/{policy.onnx, policy.pt, policy_layout.json}`（来自 run `2026-09-20_00-50-31` / `model_19999.pt`），并在 layout 里补 `joint_order_native` / `joint_order_action`（部署侧交叉断言用） | 目录即"一次训练的部署产物"，换策略 = 换目录 | `feat/policy-layout-v2` |
| 2026-09-20 | **runner 布局驱动**（`M20PiperPolicyRunner` 重写）：维度全部来自 `policy_layout.json` 并与 ONNX 形状/名字交叉断言；`joint_pos/joint_vel` 换成 **24 维原生序**（交错序），`joint_pos` 只把 4 个轮子列（15..18）置零；history 每步 **70**；动作 **16**（没有 `ee_ik` 槽位）；`ee_goal` 按训练 clip ±3；喂回 `actions` 观测的就是**实际下发**的动作 | 启动打印 `layout kind=history obs=83 history=10x70 latent=32 action=16` | `feat/policy-layout-v2` |
| 2026-09-20 | **L1 离线验收** `scripts/check_policy_interface.py`：layout 自洽 + ONNX 形状/名字 + 原生序与 C++ 表交叉断言 + 标称状态输出有限 + ONNX↔TorchScript **相对**误差 | **PASS**；标称状态 `|a|max = 1.459`（远小于训练 clip 100）；相对误差 **9.8e-07**（独立复核了训练侧导出时 1.87e-07 的结论） | `feat/policy-layout-v2` |
| 2026-09-20 | **L3 三档全绿**（`tests/sim2sim_smoke.py`，判据见契约文档） | `hold`：height 0.498 m / tilt 1.4°；`rl`（零命令）：height 0.515 m / tilt 1.0°；**`walk`（按住 w）**：height 0.515 m / tilt 1.1°、后半段平均 **+0.58 m/s**（键盘命令 +0.7 m/s），四轮转速 ≈ −7 rad/s（= 0.63 m/s） | `feat/policy-layout-v2` |
| 2026-09-20 | **修 DEF-014**（ORT C++ API 生命周期：nullptr allocator 段错误、`GetTensorTypeAndShapeInfo()` 悬垂 → `std::length_error`）**与 DEF-015**（SIGINT 退出时 `Stop()` 不调 `OnExit()` → 策略线程 joinable 析构 → abort） | 退出日志干净；`tests/sim2sim_smoke.py` 新增"日志里必须出现 `M20PiperPolicyRunner` 与 `rl_control`、且不许有 Aborted/Segmentation fault"的断言 | `feat/policy-layout-v2` |

> 结论：**新 checkpoint 在 sim2sim 里已经能站住并且能按命令前进**。
> 下一步是 `TODO_zh.md` P0 里剩下的命令语义（height 度量、复位初值）、
> 安全接管阈值，以及 P1 的仿真一致性（armature 仍未生效 = DEF-011）。
