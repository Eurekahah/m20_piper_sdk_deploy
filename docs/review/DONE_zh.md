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
| 2026-09-20 | 新增第十节：臂增益/armature/高度区间三条对齐 + `--mode arm` 用例 | `fix/arm-gains-armature-height` |
| 2026-09-20 | 新增第十一节：安全接管（倾角+腿折叠）+ 扰动注入用例 + 进 RL 偶发摔倒修复 | `fix/safety-takeover` |
| 2026-09-20 | 新增第十二节：入口瞬态定位（DEF-018）+ `ee_goal` 坐标系修正；第十三节：臂链路端到端；第十四节：轮子速度伺服 + 末端跟踪验收 | `feat/actuator-accuracy-tests` |

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

## 十、与训练对齐的三条参数修正 + 机械臂用例（2026-09-20）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **DEF-011 armature 真正生效**：MJCF 里 `armature` 从 `<motor>`（MuJoCo 静默忽略）挪到 `<joint>`（轮 0.00243216、臂/夹爪 0.01） | `check_mjcf_contract.py` 从 8 PASS/**2 FAIL** 变成 **10 PASS / 0 FAIL / 3 UNKNOWN** | `fix/arm-gains-armature-height` |
| 2026-09-20 | **DEF-007 臂增益统一到 300/20**（`arm_controller.py` 的 40/8 → 300/20；仿真侧 `ARM_DEFAULT_KP/KD` 同步） | 新增 `--mode arm`：臂相对默认位姿最大偏差 **0.0153 rad**、臂关节最大力矩 **4.6 N·m**（限幅 100）、底盘 height 0.515 m / tilt 1.0° | `fix/arm-gains-armature-height` |
| 2026-09-20 | **DEF-009 高度命令区间收敛**到训练终值 (0.33, 0.55)（键盘与 VR 都改），并把"高度是**相对足端**度量 `root_z − mean(四轮 z) + 0.09`"写进代码注释 | 默认姿态着地 = 0.5266 m（`check_mjcf_contract.py` 实测），与训练侧文档给的数字一致 | `fix/arm-gains-armature-height` |
| 2026-09-20 | **L3 第四档 `--mode arm`**：额外起 `arm_controller.py`（IK + `/ARM_JOINTS_CMD` + `/ARM_TELEOP_STATE`），断言臂保持默认位姿、力矩不触限幅、底盘不受扰 | 四档 `hold / rl / walk / arm` **全 PASS** | `fix/arm-gains-armature-height` |
| 2026-09-20 | 顺带修 `arm_controller.py` 退出时二次 `rclpy.shutdown` 抛 `RCLError`；`tests/sim2sim_smoke.py` 增加"上一次的 sim/rl_deploy 还在跑就拒绝开跑"（仿真的控制循环是墙钟驱动，并发会让结果不可复现） | 四档连续跑全绿；退出日志干净 | `fix/arm-gains-armature-height` |

## 十一、安全接管 + 扰动注入用例 + 进 RL 偶发摔倒修复（2026-09-20）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **DEF-016 安全接管落地**：`PostureUnsafeCheck()` 从空实现变成两条判据 —— 倾角 `acos(cos(roll)cos(pitch)) > 0.8 rad`（= 训练终止阈值，`M20_TILT_TAKEOVER` 可覆盖）+ 兜底"腿折叠" `|q − q_default| > 1.2 rad`；触发打印 `[TAKEOVER!]` 并切 `kJointDamping` | 见下 | `fix/safety-takeover` |
| 2026-09-20 | **仿真扰动注入** `M20_SIM_PUSH_FORCE/_AT/_DURATION`（`xfrc_applied` 作用在 `base_link`），让"接管"可自动测试 | `--mode push` 默认 800 N：`[TAKEOVER!] leg fold = 1.228 rad` → `joint_damping`，PASS | `fix/safety-takeover` |
| 2026-09-20 | 关掉折叠判据复测倾角通路（`M20_LEG_FOLD_TAKEOVER=0`） | `[TAKEOVER!] tilt = 0.818 rad` → `joint_damping`，PASS（两条通路都验证） | `fix/safety-takeover` |
| 2026-09-20 | **DEF-017 进 RL 偶发摔倒**：`OnEnter()` 先起策略线程、观测缓冲还没填过一帧 ⇒ 第一拍可能用"全零关节角 + 单位姿态"算动作，而且这一垃圾帧会进 history 最旧端影响 200 ms。修法：加 `rbs_ready_` 门禁 + `OnEnter` 先采一帧 | `--mode rl` 连续 3 次全 PASS（修复前连续跑时约 1/3 概率在进 RL 后 1~3 s 摔倒） | `fix/safety-takeover` |
| 2026-09-20 | L3 用例扩到 **5 档**：`hold / rl / walk / arm / push`，并加"上一轮残留进程"检查与"日志里必须有 runner/rl_control、不许有 Aborted"断言 | 五档序列全绿（见本节表格） | `fix/safety-takeover` |

### 五档 L3 的典型数值（2026-09-20，新 checkpoint）

| 模式 | 判据 | 实测 |
|---|---|---|
| `hold`（裸模型站立，12 s） | 不触发摔倒判据 | height min 0.493 m、tilt max 1.4° |
| `rl`（零命令，25 s） | 同上 | height 0.502~0.515 m、tilt max 1.0° |
| `walk`（按住 w，25 s） | 前进速度 ∈ [0.3, 1.2] m/s | **+0.576 m/s**（命令 +0.7）、height 0.515 m、tilt max 1.1° |
| `arm`（带 IK 节点，25 s） | 臂偏差 ≤ 0.1 rad、力矩 < 100 N·m | 偏差 **0.0043~0.0245 rad**、最大力矩 **1.3~7.0 N·m** |
| `push`（800 N 侧推） | 必须触发 `[TAKEOVER!]` 并切 `joint_damping` | leg fold 1.228 rad / tilt 0.818 rad（分别单独验证） |

## 十二、进 RL 的入口瞬态：定位与交接（2026-09-20）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **DEF-019 修 `ee_goal` 坐标系**：`RLControlState::ee_goal_` 与 `UserCommand` 的默认值原来是臂基座系的 `(0.1092, 0, 0.3439)`，改成 root 系的 `(0.3492, 0, 0.4327)` | `M20_PIPER_DEBUG` dump 的第一拍 `obs[73:80]` 从 `0.1092...` 变成 `0.3492...` | `fix/entry-transient` |
| 2026-09-20 | **`run_cnt_` / `decimation_` 未初始化**（策略线程先于 `OnEnter()` 启动）→ 就地初始化 + 调整启动顺序；**软启动** `M20_SOFT_START_TICKS`（默认 10 tick = 200 ms） | 真 bug，修掉；软启动对失败率无影响，但契约要求保留 | `fix/entry-transient` |
| 2026-09-20 | **遥测加 `wall` 列 + 控制循环追帧** → 可算实时因子 | 全程 RTF **1.000**（最差 1 s 窗口 0.97）⇒ 排除"仿真跑慢" | `fix/entry-transient` |
| 2026-09-20 | **执行器延迟仿真**（训练 `DelayedPD` 每执行器 0~5 物理步 = 0~25 ms）做成开关 | `M20_SIM_ACTUATOR_DELAY_TICKS=25`：失败率 5/12（更差）⇒ 默认关，留作 P1-1 的对照工具 | `fix/entry-transient` |
| 2026-09-20 | **入口瞬态的 4 组对照实验**（见 `DEFECT_LOG_zh.md` DEF-018 的矩阵） | 全部不是解：软启动长度 0/10/25/50 → 0/6、1/6、2/6、2/6；轮子执行器语义改动 7/12（更差）；执行器延迟 5/12；站立腿增益 4/12。基线 **3/12 ≈ 25%** | `fix/entry-transient` |
| 2026-09-20 | **结论**：入口发散是**策略在该动力学下的边缘稳定性**（失败运行里轮速命令自己长到 ±3~5），与训练侧 s3 阶段 0.09~0.15/20 s 的终止率同源 → 已按训练仓库 `TODO_zh.md` P1-2 的口径交接 | `tests/sim2sim_smoke.py --repeat N` 现在报失败率（`rl` 档判据 ≤ 1/3） | `fix/entry-transient` |

## 十三、机械臂链路端到端 + 坐标系修正（2026-09-20）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **DEF-020**：`arm_controller` 发布的 `/ARM_TELEOP_STATE` 是**臂基座坐标系**的值（0.1092, 0, 0.3439），而策略观测要 root 系 ⇒ 开着臂节点时策略一直看到偏 24 cm 的 EE 目标。修：发布前加回臂座偏移 `(0.24, 0, 0.0888)`，留 `M20_EE_GOAL_BODY_FRAME=0` 回旧行为 | `ros2 topic echo --once /ARM_TELEOP_STATE`：默认 `(0.3492, 0, 0.4327)`、旧行为 `(0.1092, 0, 0.3439)`；`--mode arm` 下第一拍 `obs[73:80]=(0.3492,0,0.4327)` | `fix/ee-goal-frame-arm-node` |
| 2026-09-20 | **DEF-021**：`arm_teleop_node` 在非终端 stdin 下崩（`termios.error: Inappropriate ioctl for device`）⇒ 非 TTY 时跳过 raw 模式 | 管道驱动可用（自动化测试与 launch 拉起都不再崩） | `fix/ee-goal-frame-arm-node` |
| 2026-09-20 | **新增端到端用例 `--mode arm_move`**：起 sim + rl_deploy + arm_controller + arm_teleop，进 RL 后按住 numpad 8（EE +x） | 臂相对默认位姿动 **2.31 rad**、最大关节力矩 **11.4 N·m**（限幅 100）、底盘 height 0.491 m / tilt max **1.7°** —— 即"臂在动、底盘不摔" | `fix/ee-goal-frame-arm-node` |
| 2026-09-20 | `tests/run_all.sh` 覆盖六个模式（`hold / rl / walk / arm / arm_move / push`） | 见 `DONE_zh.md` 第十一节表格 | `fix/ee-goal-frame-arm-node` |

## 十四、执行器/末端精度验收（2026-09-20）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **P1-2 轮子速度伺服阶跃**：仿真侧新增 `M20_SIM_WHEEL_STEP_RAD_S` / `_AT`（直接覆盖四个轮子的执行器语义为 kp=0/kd=0.6 + 速度目标），新增 `--mode wheel_step` | 阶跃 +5 rad/s：四轮稳态 **+5.00 +5.00 +5.00 +5.00 rad/s**（同向、误差 < 0.2%），上升时间 ~1 s（受 21.6 N·m 力矩限幅约束，要把整机加速） | `feat/actuator-accuracy-tests` |
| 2026-09-20 | **遥测增加末端位姿与轮速指令列**：`ee_x/y/z`、`ee_qw..qz`（gripper_base 相对 base_link，root 系）、`wcmd_fl/fr/hl/hr` | 默认姿态下 `ee = (0.3492, 0, 0.4326)`，与 `check_mjcf_contract.py` 的 FK 实测一致 | `feat/actuator-accuracy-tests` |
| 2026-09-20 | **`--mode arm_move` 增加末端位移判据**：按住 numpad 8（EE +x），参考点取默认姿态的 `ee_x = 0.3492`（不能用"中间某刻"——臂几秒内就走到工作空间边界） | 末端 x：**0.3492 → 0.8336 m（Δ = +0.48 m）**，臂关节最大偏差 2.31 rad、最大力矩 11.6 N·m，底盘 tilt ≤ 1.2° | `feat/actuator-accuracy-tests` |
| 2026-09-20 | `tests/run_all.sh` 扩到 **7 档**（`hold / wheel_step / rl / walk / arm / arm_move / push`） | 一键全绿 | `feat/actuator-accuracy-tests` |
