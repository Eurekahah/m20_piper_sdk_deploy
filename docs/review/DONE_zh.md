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
| 2026-09-20 | 新增第十二节：入口瞬态定位（DEF-018）+ `ee_goal` 坐标系修正；第十三节：臂链路端到端；第十四节：轮子速度伺服 + 末端跟踪验收；第十五节：策略版本管理 + 物理对照；第十六节：无臂部署方案 | `feat/no-arm-stub` |
| 2026-10-09 | 新增第二十三节：仿真新代码同步（臂/夹爪增益 300/8、286/5；臂速度上限 3 rad/s）+ 默认策略切到多地形 run `cloud_slowvx20k` | `sync/2026-10-09-sim-delta` |
| 2026-10-12 | 新增第二十四节：VR 臂跟随修复（DEF-044）——柱坐标位置 + 手腕关节偏移的姿态映射、IK 收敛检查（解不出来不下发）、零空间姿态正则、关节指令限速默认 3 rad/s；新增 `tests/vr_arm_chain_check.py` 回归用例（对旧行为 FAIL、新行为 PASS） | `fix/vr-arm-ik-reachability` |
| 2026-10-13 | 第二十四节续：MuJoCo 窗口里的可视化验收（12 个具名末端位姿指令、跟踪 ~5 mm / 0.8°）+ 补修"半径下限把标定锚点钳住" + 定位 DEF-045（残留 `arm_stub_node` 造成 /ARM_JOINTS_DATA、/ARM_TELEOP_STATE 双发布者） | `fix/vr-arm-ik-reachability` |
| 2026-10-13 | 修复 **DEF-046**：VR 复位语义 = 机身姿态回默认 + 臂末端位姿回默认 + 速度归 0 + **交还控制权**（再按 B 把左右手柄当前位姿重采成校准原点）。改动 4 个文件（C++ 机身锚点/共享指令、键盘内部底座归位、设备侧 `RESET_KEEPS_ACTIVE` 默认 1→0），新增回归 `tests/vr_reset_semantics_check.py`（设备级 + 端到端两场景，含旧实现 A/B 复现）并接入 `tests/run_all.sh` | 工作区（建议 `fix/vr-reset-defaults`） |
| 2026-10-13 | 新增第二十五节：头显页面**离线化**（DEF-058）——页面不再从公网 CDN 拉 A-Frame，改成本地 `aframe.min.js` + 缺失时页内红条 + `scripts/fetch_aframe.sh` 一键获取 + `sync_to_robot.sh`/启动横幅双重提示 + HTTPS 服务改多线程。**操作员当日实测通过：头显 VR 遥操正常**。新增 `docs/OFFLINE_ASSETS_zh.md`（离线资源清单：要下载什么） | `sync/2026-10-13-vr-ip` |
| 2026-10-13 | 新增第二十六节：真机"**左前侧摆关节过温 100 ℃ → 安全接管 → 趴下 → 10 s 后恢复**"的仿真侧排查（**DEF-059**）——结论：**仿真没有电机热模型**（温度写死 40/45 ℃ ⇒ 温度接管路径在 sim2sim 不可达），等价可观测的**关节力矩**是**长期静载**（站立状态机 `fl_hipx` +12.0 N·m / RL 站立 +15.1 N·m，150 s 稳态不变）；新增离线探针 `tests/joint_torque_report.py`；后续动作登记为 **P1-7** | `investigate/left-hipx-temp-sim2sim` |

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
| 2026-09-20 | **DEF-022 修键盘增量路径的目标限速**：`arm_teleop` 以 200 Hz 发增量，原实现逐条累加 ⇒ 按 0.1~0.4 s 就把目标甩到工作空间边界。改成"累计增量 → 在 50 Hz tick 里按 `M20_EE_MAX_LIN_SPEED`(0.35 m/s) 推进请求位姿 → 被跟踪目标再限速逼近"；VR 路径用 2.0 m/s / 5.0 rad/s | 短按 0.4 s：末端 **+0.246 m**、**IK 位置残差 0.18 mm**（修复前 +0.49 m 撞限位、残差 21.8 mm） | `feat/actuator-accuracy-tests` |
| 2026-09-20 | **IK 精度验收通道**：`arm_controller` 增加 `M20_ARM_DEBUG=1` 的 IK 残差打印（目标 vs 解出关节角的 FK），`--mode arm_move` 自动解析并断言 | 可达目标**稳态**残差 **0.18~0.23 mm**；运动过程中最大 54~71 mm（= 限速 0.35 m/s 下约 0.15~0.2 s 的**跟踪滞后**，不是误差发散），`--mode arm_move` 只对末尾稳态做断言 | `feat/actuator-accuracy-tests` |

## 十五、策略版本管理 + 物理对照定案（2026-09-20）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **`policy/README.md`**：定下"一个子目录 = 一次训练的部署产物"的约定 —— 命名 `<实验名>_<日期>`、换策略 6 步（导出 → 复制 → 补 `joint_order_native`/`joint_order_action` → L1 → `M20_POLICY_DIR` 试跑 → 默认值 + `run_all.sh`）、以及两条硬性约束（别用 actor-only 的 `exported/policy.pt`；onnx 与 layout 必须成对且维度一致） | 当前默认目录 `policy/m20_piper_history_20260920/`（policy.onnx + policy.pt + policy_layout.json） | `edde463` 之后 |
| 2026-09-20 | **P1-1 物理对照定案**：契约文档第 5 节从"待办清单"变成逐项结论（控制周期/积分步长/执行器延迟/增益/armature/限幅/重力/摩擦/恢复系数/求解器/基座惯性/初始位姿/碰撞/地形） | `base_link` 显式惯性 = URDF（质量 15.882、三个主惯量一致，仅丢 ~0.2% 的非对角项）；摩擦取训练分布内的固定值 1.0、恢复系数 0 | `edde463` |

## 十六、无臂部署方案（2026-09-20）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-20 | **DEF-023**：臂没装时策略会看到"臂不在默认姿态"（接口把臂反馈初始化为 0，而默认姿态是 arm2 0.5 / arm3 -0.5 ⇒ 观测与 `ee_goal` 矛盾）。修：接口初值改成默认姿态 + `HasData()` + `EnterRL` 时打印无臂警告 | 见下 | `feat/no-arm-stub` |
| 2026-09-20 | 新增 **`arm_stub_node.py`（假臂节点）**：无臂部署时以 200 Hz 发 `/ARM_JOINTS_DATA`（默认姿态、零速度/力矩）、50 Hz 发 `/ARM_TELEOP_STATE`（root 系默认位姿 `(0.3492,0,0.4327)`），订阅丢弃 `/ARM_JOINTS_CMD`；`M20_ARM_STUB_POSE` / `M20_ARM_STUB_EE` 可改 | `ros2 topic echo --once /ARM_JOINTS_DATA` → `position: 0.0 / 0.5 / -0.5 ...`；`/ARM_TELEOP_STATE` → `(0.3492, 0, 0.4327, 0.7373, 0, 0.6756, 0)` | `feat/no-arm-stub` |
| 2026-09-20 | 仿真侧新增 `M20_SIM_NO_ARM_DATA=1`（照常跑，只是不发臂反馈）用来模拟"臂没装"；新增 L3 用例 `--mode no_arm` | 第一拍观测臂项 = `[0,0,0,0,0,0,0,0]`、`ee_goal` = 默认位姿；`--repeat 6` 失败率 17%（仅入口瞬态）；不起假节点时启动打印无臂警告且观测仍自洽（接口初值兜底） | `feat/no-arm-stub` |

## 十七、sim2sim 的腿/臂分离调试（2026-09-20/21）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-21 | **`scripts/run_sim2sim.sh` 三套配置**：`full`（腿臂都来自 MuJoCo）/ `legs`（臂=假臂默认姿态）/ `mixed`（腿来自 MuJoCo、臂走独立传输）。三者下 `rl_deploy` 看到的接口完全一致，只是"谁在发布"不同 | Ctrl-C 能收干净所有子进程（实测无残留）；`legs` 起 sim+stub、`mixed` 起 sim+stub(follow)+arm_controller | `feat/no-arm-stub` 之后 |
| 2026-09-21 | **假臂节点增加"跟随模式"**（`M20_ARM_STUB_FOLLOW=1`，`M20_ARM_STUB_MAX_VEL=3.0 rad/s`）：把 `/ARM_JOINTS_CMD` 的位置目标一阶跟随出来当反馈 = **假 CAN 驱动**，于是没有真臂也能调 `arm_controller`/`arm_teleop` 这条链路 | 见下 | 同上 |
| 2026-09-21 | **新增 L3 用例 `--mode arm_mixed`**：sim 不发臂数据 + 假 CAN 驱动 + IK + 键盘，按住 numpad 8 后断言**策略观测里的臂项真的变化** | 变化量 **1.037 rad**（tick0 → tick299），底盘 height 0.511 m / tilt ≤1.1° ⇒ 数据链路通了 | 同上 |
| 2026-09-21 | `tests/run_all.sh` 扩到 **9 档**（新增 `no_arm`、`arm_mixed`） | 见第十一节表 | 同上 |

## 十八、真机部署前检查工具（2026-09-22）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-22 | **`scripts/preflight_real_deploy.py`**：一次回答"策略要不要臂数据 / 能不能上真机 / 该跑哪些命令"。检查：① 策略目录与观测构成（能否从维度反推 24 关节含臂 / 16 关节不含臂）；② `build/.../flags.make` 里有没有 `-DSIM2SIM`（真机必须没有）；③ 四个臂相关脚本是否齐全；④ `/JOINTS_DATA`、`/ARM_JOINTS_DATA`、`/ARM_TELEOP_STATE` 等话题有没有发布者。给"必须处理"的阻断项并 `exit 1` | 在当前容器（SIM2SIM 构建、无真机）实测：正确报出"策略需要臂数据"、"SIM2SIM 构建不能上真机"、"没有 /JOINTS_DATA"，并提示起 `arm_stub_node.py` | `preflight` |

## 十九、真机启动自检不再卡死（2026-09-22）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-22 | **DEF-024**：`M20Interface::ResetPositionOffset()` 原来要求 16 个关节（含 4 轮）全部变化且**无限等待**，真机上站着不动/没有反馈时就是每 10 s 打印一次 `joint data update is not finished` 并卡在启动里（期间机器人是零增益=软的）。改为有限等待 `M20_JOINT_UPDATE_TIMEOUT_S`（默认 5 s，`=0` 跳过）+ 每秒进度 + 收不到数据时的明确诊断 + hipy/knee 未更新则跳过 ±360° 偏移校正 + 结束打印 16 个实测角 | 用真机接口（`-DSIM2SIM=OFF`）复现"无反馈"：3 s 后打印 `完全没有收到 /JOINTS_DATA（收到 0 条）` 与三条排查建议，不再卡住；`--mode hold` 回归 PASS | `fix/real-startup-selfcheck` |
| 2026-09-22 | **DEF-025**：CMake 缓存 `SIM2SIM` —— 先编仿真版再编真机版必须显式 `-DSIM2SIM=OFF`，否则仍是恒等标定的仿真接口（日志里看不出来） | `preflight_real_deploy.py` 的 `[2]` 会读 `flags.make` 并在发现 `-DSIM2SIM` 时报阻断项 | 同上 |

## 二十、真机臂桥接（两台机器）+ FK 发布 ee_goal（2026-09-22）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-22 | **`arm_real_adapter` 增加 FK 发布 `/ARM_TELEOP_STATE`**（`M20_ADAPTER_PUBLISH_EE`，默认开）：臂动了但没人更新 `ee_goal` 时，策略会一直以为臂在默认位姿（观测分布外）。现在从 `/feedback/joint_states` 的关节角算 FK（复用 `arm_controller` 的 MDH 表），换算到 root 系后以 50 Hz 发布 | 假反馈喂默认姿态 → `/ARM_TELEOP_STATE` = `(0.3492, 0, 0.4327, 0.7373, 0, 0.6756, 0)`，与实测默认位姿差 **0.04 mm / 0.000°** | `feat/real-arm-bridge` |
| 2026-09-22 | **确定两台机器的拓扑**：Jetson（10.21.31.201）只跑 `agx_arm_ctrl`；M20 电脑（10.21.31.103）跑 `arm_real_adapter` + `arm_controller` + `arm_teleop_node` + `rl_deploy`。adapter 收发 drdds，所以放在 M20 上（Jetson 不用装 drdds） | 见下 | 同上 |
| 2026-09-22 | **新增 L2 验收 `tests/arm_real_bridge_check.py`**（不需要真臂/Jetson）：假 `/feedback/joint_states` → 检查 `/ARM_JOINTS_DATA`（8 关节逐位一致）；假 `/ARM_JOINTS_CMD` → 检查 `/control/joint_states`（joint2 0.8、夹爪宽度 0.04） | 上行 8 关节最大差 **1.5e-10**；ee_goal 位置差 0.04 mm；下行 joint2=0.8000、gripper=0.0400 ⇒ **PASS** | 同上 |
| 2026-09-22 | 文档：README 增加"真机 + 真臂：两台机器怎么接线"（拓扑图、两台机器各自的命令、方案 A/B、网络与 DDS 发现要求、验证步骤、安全提示）；`run_all.sh` 增加 L2 桥接检查；真机检查单阶段 B 重写为两台机器流程 | — | 同上 |
| 2026-09-22 | **"只调臂"（不开策略）**：新增 `run_sim2sim.sh arm` 配置与 L3 用例 `--mode arm_only`（仿真 + IK + 键盘，**不起 rl_deploy**）。确认臂链路与策略解耦；真机上对应"只起 adapter+IK+键盘"的用法（腿由"不进 SDK 模式"或"rl_deploy 停在 idle"撑着） | 轻点 numpad：末端 **+0.20 m**、IK 稳态残差中位数 **0.63 mm**、底盘 tilt 2.4°，连跑 2 次都 PASS | `feat/arm-only` |

## 二十一、"只调臂"（不开策略）+ 进 RL 不抢臂（2026-09-22）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-22 | **`run_sim2sim.sh arm` + `--mode arm_only`**：仿真 + `arm_controller`(IK) + `arm_teleop`(键盘)，**不起 `rl_deploy`** —— 臂链路与策略完全解耦，可以先单独调；真机对应"只起 adapter+IK+键盘" | 轻点 numpad：末端 **+0.20 m**、IK 稳态残差中位数 **0.63 mm**、底盘 tilt 2.4°，连跑 2 次 PASS | `feat/arm-only` |
| 2026-09-22 | **DEF-026**：`PiperArmInterface` 现在也监听 `/ARM_JOINTS_CMD`；进 RL 时若最近 0.5 s 有别人在发（`arm_controller` 在控制臂），就跳过"保持默认位姿"，不再把臂拽一下 | `--mode arm/arm_move`：出现跳过日志，`arm_move` 仍 PASS（+0.239 m / 残差 0.18 mm）；`--mode no_arm`：跳过日志 0 次（该发还是发），PASS | `f456e0e` |
| 2026-09-22 | IK 残差判据改成"末尾中位数"（运动瞬态单拍可能撞 20 次迭代上限，属跟踪滞后而非误差） | `arm_only` / `arm_move` 均稳定 PASS | `feat/arm-only` |

## 二十二、命令速查文档 + 臂键盘退出修复（2026-09-29）

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-09-29 | **DEF-028**：`arm_teleop_node` 的 `Ctrl+C` 关不掉（`tty.setraw` 清掉 ISIG，`0x03` 被当普通按键丢掉）。改用 `setcbreak` + 显式处理 `0x03` + `main` 捕获 `KeyboardInterrupt` | 真 TTY 实测：`Ctrl+C` → `[arm_teleop] stopping ... / stopped (terminal restored)`；`ESC` 仍然可用 | `fix/arm-teleop-exit` |
| 2026-09-29 | **两份命令速查**：`src/M20_sdk_deploy/COMMANDS_SIM2SIM_zh.md`（全仿真/只调腿/只调臂/腿仿真+臂独立传输/VR/一键验收/换策略）与 `COMMANDS_SIM2REAL_zh.md`（只部署 M20/只调臂/只感知真臂/IK+键盘合并/DDS 两种修法/排障），每个场景一张"哪个终端敲什么"的表；README 顶部与 `docs/README_zh.md` 都已挂索引 | — | 同上 |

## 二十三、仿真新代码同步 + 换多地形策略（2026-10-09）

来源：训练仓库 `docs/deploy_code_delta_2026-10-09_zh.md`（基线 `2d49f47` → `f663ee1`）。
逐条落点见 `docs/deploy_code_delta_2026-10-09_zh.md`；缺陷条目见 DEFECT_LOG 的
DEF-029~DEF-032。

| 日期 | 内容 | 关键实测 | commit |
|---|---|---|---|
| 2026-10-09 | **DEF-029**：臂 PD **300/20 → 300/8**（DEF-064）+ 臂关节 1~6 速度上限 **3 rad/s**（DEF-065）。同步三处常量（`arm_controller.py` / `mujoco_simulation_ros2.py` / `piper_arm_interface.hpp`）与 `check_mjcf_contract.py` 的断言 | 训练侧：饱和 23.5% → 0%、跟踪 4.14 → 2.19 cm、关节速度 p99 5.0 → 3.00 rad/s | `sync/2026-10-09-sim-delta` |
| 2026-10-09 | **DEF-030**：夹爪 PD **4000/200 → 286/5**（DEF-058）；开合指令 ±0.035 本来就对 | 训练侧：饱和 88~100% → 0%、稳态误差 ≤0.0005 | 同上 |
| 2026-10-09 | **DEF-031**：默认策略切到多地形 run `2026-10-02_00-44-42_cloud_slowvx20k`，新目录 `policy/m20_piper_rough_20261002/`（补齐 `joint_order_native`/`joint_order_action`）；`rl_control_state.hpp`、`check_policy_interface.py`、`preflight_real_deploy.py` 默认目录同步；旧平地目录保留 | 用本仓库 onnxruntime 编的最小校验器：两目录 I/O 名/形状一致、标称输出有限、动作幅值上限 0.958 / 1.459（<10）、输出各不相同（不同权重） | 同上 |
| 2026-10-09 | **DEF-032**：核对训练侧其余 3 条——IK 输出夹限位（DEF-054）**已满足**（`arm_controller.ik_step` 已 clip）、EE 命令复位锚定（DEF-034）**已满足**、EE 碰撞盒机体局部系（DEF-066）**部署侧不适用** | 见 `docs/deploy_code_delta_2026-10-09_zh.md` §0 对照表 | 同上 |
| 2026-10-09 | **容器实测（L0~L3）**：容器 `m20_piper_ros` 里重建 + `tests/run_all.sh`。L0 编译 ✅、**L1 PASS**（ONNX↔TorchScript 5.96e-07）、**L3' 10 PASS / 0 FAIL**、L2 ✅；hold/wheel_step/no_arm/arm_only/arm_mixed/push 全 PASS，rl/walk/arm_move 失败率 0~17%（<1/3）。`arm` 档偶发倾角接管 = 训练侧 DEF-018（新策略 9/12 vs 旧平地策略 8/12 通过，臂力矩全程 ≤5 N·m） | 完整数字见 `docs/deploy_code_delta_2026-10-09_zh.md` §7；测试口径问题见 DEF-033 | `sync/2026-10-09-sim-delta` |
| 2026-10-09 | **DEF-034**：手工 `z`→`c` 后"乱动/摔倒"复现并定位为**宿主机电池降频**（`platform_profile=low-power`、全核 0.8~1.0 GHz）⇒ 实时因子 1.00→0.67 ⇒ 墙钟 50 Hz 策略被压缩成 ~1.5 倍速。仿真新增 `[SIM-RT]` 1 Hz 实时因子自检（<0.95 打印警告）与 `M20_SIM_RENDER_INTERVAL` 渲染间隔开关 | 同一命令：插电 1.000 / 0 失败 vs 电池 0.67 / **3-3 失败** | 同上 |
| 2026-10-09 | **`COMMANDS_SIM2SIM_zh.md` 整体改写为"宿主机一行式"**：所有命令带 `docker exec`（含 `m20()` 便捷函数、`xhost` 放行、无头/窗口两种、二进制直调避免 `ros2 run` 孤儿进程）；新增 §8 键盘速查（含"速度键是阶跃、想柔和不按"与臂"轻点"语义）与 §9 排障（`[SIM-RT]` 实时因子表、残留/孤儿进程检查与 `pkill -f` 自杀坑） | 文档 | `sync/2026-10-09-sim-delta` |
| 2026-10-09 | **平地模型 `policy/m20_piper_flat_20260930/`**（run `2026-09-30_19-36-00_cloud_cap12_20k`）入库：补齐部署侧必需的 `joint_order_native` / `joint_order_action`（与另两个目录逐项相同），并把它写进 commands/README/policy-README/契约文档作为"平地"选项 | `check_policy_interface.py` **PASS**（83/700/16、名字/形状一致、`\|a\|max` 1.093、ONNX↔TorchScript 1.36e-06） | `sync/2026-10-09-sim-delta` |
| 2026-10-09 | **DEF-035：默认策略切到平地模型**（`rl_control_state.hpp` + 两个脚本的默认目录；多地形改 `M20_POLICY_DIR` 切回）+ `--mode push` 默认推力 800→1500 N | 插电、实时因子 **1.000**：L1 PASS、L3' 0 FAIL、hold/arm/wheel_step/no_arm/arm_only/arm_mixed PASS、**rl 0/6**、walk 2/6（容差内）、**arm_move 0/6**（ee_x +0.25 m）；push 800 N 被扛住（不触发接管）→ 1500 N **3/3** 触发 | 同上 |
| 2026-10-09 | **新增 `tests/command_sweep.py`（22 个命令阶段：vx 0.2~1.5 / vy / wz / 组合 / 机身姿态），并挂进 `tests/run_all.sh`**；用它把平地与多地形两套模型各跑 3 遍 | 两模型各 **3/3 PASS、66 次命令暴露 0 摔倒**、实时因子 1.000；两套模型 10 档 L3 均"全部通过"（详见验收报告） | `sync/2026-10-09-sim-delta` |
| 2026-10-09 | **验收报告 + 全代码走查**两份文档：`docs/review/ACCEPTANCE_2026-10-09_zh.md`（平地 vs 多地形全套数字 + 侧移/转向方向不对称等发现）、`docs/review/CODE_REVIEW_2026-10-09_zh.md`（缺漏/不优雅，带 `文件:行`） | 走查结论：安全层两个检查未接线、`liedown` 限位夹取失效、默认站姿抄 8 遍、死代码约 700 行；对应 TODO 新增 P4-1~P4-9 | 同上 |
| 2026-10-10 | **VR/臂链路一整批修复**（详见 `DEFECT_LOG_zh.md` **DEF-052~DEF-056**；这 5 条在 2026-10-13 重编号过，原为 039~043）：VR 速度限幅与默认手感、臂链路自检脚本、双发布者检测、复位语义（末端目标回默认位姿 + 保持 VR 激活）、IK 节点不再因单拍异常退出、软死区 | 实测：VR vx 1.5→1.01 m/s；复位后臂与默认位姿差 0.015 rad 且不按 B 可继续操控；干净输入下 `dx` 跟手 1:1（+0.1276 m）、IK 0.08 mm；`arm_*` 回归全过 | `sync/2026-10-09-sim-delta` |
| 2026-10-10 | **新增真机 + VR 完整部署流程文档** `docs/DEPLOY_REAL_VR_zh.md`（10 节：拓扑/前提 → 编译 arm 版与 scp → SDK 模式与标定核对 → 只上腿 → 接真臂 → 接 VR（按键分工/复位语义）→ 手感与工作空间 → 安全回滚 → 排障 → 速查），并从 `docs/README_zh.md`、`sim2real_checklist_zh.md`、`COMMANDS_SIM2REAL_zh.md` 三处挂索引 | 文档 | 同上 |
| 2026-10-10 | **DEF-056（原 DEF-043）：真机离线依赖**——VR 链路不再依赖 scipy（新增纯 numpy 的 `xlevr/_rotation_compat.py`，两处导入改成 scipy 优先 + 兼容实现兜底）；`requirements_teleop.txt` 标注 scipy 可选、yaml/websockets 必需；`DEPLOY_REAL_VR_zh.md` §1.5 离线装包（跨平台 wheel 下载参数 / `pip3 --no-index` / 无 pip 时解压 wheel + PYTHONPATH） | 与真 scipy 对照 2000 组随机旋转 **误差 ≤1e-14**；屏蔽 scipy 后 `vr_teleop_node.py` 正常启动（8443/8442 都起） | 同上 |
| 2026-10-12 | **新增 `scripts/sync_to_robot.sh`（真机增量同步）**：rsync 优先（第二次起只传改动）、默认排除**仿真专用 mesh（47 MB / 2202 个 STL）**、开发机专用 `onnxruntime/x86`（62 MB）、`gamepad/example`（3.9 MB）、`__pycache__`；无 rsync 时退化为 `tar+ssh` 并**远端自检文件数**；`--with-meshes/--full/--dry-run/--delete/--build` + `ROBOT=`/`DEST=` | 实测：**4539 → 2038 个文件、215 MB → 104 MB、STL 0 个**（rsync 与 tar 两条路径都验过）；`DEPLOY_REAL_VR_zh.md` §1 改用该脚本 | 同上 |
| 2026-10-12 | **离线依赖包改放在仓库 `tmp/wheels/`（`.gitignore` 已忽略 `tmp/`，重启不丢）**：`PyYAML-6.0.3` / `websockets-13.1`（cp38+aarch64）+ 新增 `pylibs_yaml_websockets_cp38.tar.gz`（两个包的纯 Python 源码，真机没 pip 时解压挂 `PYTHONPATH` 即可）；确认真机 numpy 1.17.4 够用、**无需传 numpy**；`DEPLOY_REAL_VR_zh.md` §1.5 更新为两条安装路径 | 方式 B（vendored + 屏蔽 scipy）实测能起 `vr_teleop_node.py`（8443/8442 均监听） | 同上 |
| 2026-10-13 | **DEF-057：VR 节点打印 `https://localhost:8443` 修掉**——IP 探测原来依赖"连外网取出口 IP"（真机内网没有默认路由 ⇒ 抛 `ENETUNREACH` ⇒ 退回 localhost）；改为**离线枚举所有网卡 IPv4**（`ip -4 -o addr show` → `hostname -I` → UDP 兜底；无线优先）并打印所有候选 + 支持 `M20_VR_IP=<ip>` 指定；顺带修 `xlevr/config.py` 的 `config.yaml` 相对路径（警告消失、改 yaml 真生效）。服务端一直绑 `0.0.0.0`，页面里的 `wss://<host>:8442` 跟着页面地址走 | 单测（模拟真机 `ip` 输出：lo/end0/wlan0/docker0）：`wlan0 10.21.41.1` 排第一、回环被跳过；三种打印路径（自动/覆盖/识别不到）逐条验过 | `sync/2026-10-13-vr-ip` |

---

## 二十四、VR 臂跟随修复（DEF-044，2026-10-12）

来源：操作员反馈"sim2sim 里腿臂同时操作时臂动作不对，键盘正常、VR 很怪"。
完整条目见 `DEFECT_LOG_zh.md` 的 **DEF-044**；调参说明见 `COMMANDS_SIM2SIM_zh.md` §5.2。

| 日期 | 内容 | 关键实测 | commit / 分支 |
|---|---|---|---|
| 2026-10-12 | **定位**：默认站姿 `(0, 0.5, -0.5, 0, 0, 0)` 里 joint5 = joint6 = 0 ⇒ joint4 与 joint6 同轴 ⇒ **手腕在奇异位形**（Jacobian 最小奇异值 3.6e-18）。VR 的"侧移 + 手腕转动"在这个位形上**没有近邻解**（约束优化求最近姿态：侧移 5 cm 要 2.67 rad、工具 yaw 10° 要 2.47 rad；键盘 ±x 只要 0.30 rad），局部 IK 走不动，而节点又把"顶在关节限位上的解"无条件下发 ⇒ 臂被驱到 joint1 = 2.168（上限）、20 rad/s、139 N·m。腿是无关项（腿站着不动同样复现） | 6 个用例对照（下表的"修改前"列）；单变量 A/B：键盘 + 旋转键（numpad `+`）同样坏 ⇒ 不是 VR 传输问题 | `fix/vr-arm-ik-reachability` |
| 2026-10-12 | **修复**：① 位置改**柱坐标**（前后→半径、左右→**基座摆动**、上下→z）；② 姿态改**手腕关节偏移**（工具姿态跟着臂摆，不再要求手腕补偿整条臂）；③ **IK 收敛检查**：残差 > 10 mm / 3° 就不下发（沿用上一拍 + 目标拉回实际位姿）；④ **零空间姿态正则**（压手腕沿自由方向漂到限位）；⑤ 关节指令限速默认 **3 rad/s**（= 真机驱动器口径）。旧映射可用 `M20_VR_ARM_MAP=cartesian` 回退 | 见下表"修改后"列 | 同上 |
| 2026-10-12 | **回归用例** `tests/vr_arm_chain_check.py`（假头显：侧移 + 前伸 + 上下 + 手腕三轴转动；约 25 s），挂进 `tests/run_all.sh`（`VRA=0` 跳过） | **对旧行为 FAIL**（IK 残差 100.5 mm、joint5 停限位 8.46 s、21.3 rad/s、156.8 N·m）／**对新行为 PASS**（1.9 mm、0 停留、3.5 rad/s、15 N·m、跟手 23 mm） | 同上 |
| 2026-10-12 | **不改动键盘路径**（正则与限速只在 VR 路径生效），回归：`--mode arm_move` / `arm_mixed` / `no_arm` 全 PASS，`check_policy_interface.py` **L1 PASS**，`check_mjcf_contract.py` 10 PASS / 0 FAIL | 见下 | 同上 |

**实测对照（同一套 rig、同一段假头显动作，实时因子 1.000）**

| 指标 | 修改前（VR） | 修改后（VR） | 键盘（对照） |
|---|---|---|---|
| IK 位置残差 > 10 mm 的采样占比 | 31.8% | **0.0%** | 0.0% |
| IK 位置残差 p95 / max | 104 / 114 mm | **2.8 / 9.8 mm** | 0.2 / 0.4 mm |
| 跟手误差（请求 vs 实际末端）p95 | 99.9 mm | **26 mm** | — |
| 关节速度 max | **20.8 rad/s** | **1.9 rad/s** | 1.9 rad/s |
| 指令力矩 max | **139.5 N·m** | **8.1 N·m** | 12.6 N·m |
| 停在关节限位的关节 | joint1 / 4 / 5 | **无** | 无 |

**只加安全网、不改映射**（A/B：`M20_VR_ARM_MAP=cartesian` + 关正则/限速）：
残差看着是 0，但安全网**拒发 965 拍**（≈ 一半时间臂不动，因为它满足不了请求）
⇒ 映射才是修复本身，安全网只是兜底。

### 二十四续：窗口可视化验收 + 两个补充修复（2026-10-13）

| 日期 | 内容 | 关键实测 | commit / 分支 |
|---|---|---|---|
| 2026-10-13 | **开 MuJoCo 窗口做可视化验收**：`sim(M20_USE_VIEWER=1)` → `arm_controller` → `rl_deploy`（`z` 站立 → `c` 进 RL）→ 新增工具 `src/M20_sdk_deploy/scripts/arm_pose_demo.py`（12 个具名末端位姿指令，循环下发并打印实测） | 实时因子 **1.000**、base 高 0.508 m、倾角 <1°、**0 次接管**；12 个位姿逐个跟踪：位置 **~5 mm**、姿态 **~0.8°**（IK 残差 0.1~1.6 mm），关节 ≤0.06 rad/s、力矩 ≤4.6 N·m。明细表见 DEFECT_LOG 的 DEF-044 §6 | `fix/vr-arm-ik-reachability` |
| 2026-10-13 | **补修：半径下限把标定锚点自己钳住**——`M20_VR_RADIUS_MIN` 默认 0.12 m 高于默认站姿半径 0.1092 m，"手停在标定点"回不到锚点，实测"中位"与"后收 8 cm"目标完全相同（都是 x = 0.360），往回拉 <1.1 cm 无反应。改为 `min(M20_VR_RADIUS_MIN, r_anchor)`（默认 0.05） | 已提交；**窗口复跑待做**（预期：中位回到 0.3492、后收 4 cm 到 0.3092） | 同上 |
| 2026-10-13 | **发现并定位 DEF-045**：11 小时前残留的 `arm_stub_node.py` 与 `arm_controller` **同时**发 `/ARM_JOINTS_DATA` + `/ARM_TELEOP_STATE` ⇒ IK 反馈与策略 `ee_goal` 观测被掺入常量（表现"目标读数不变、臂不听使唤"）。清掉残留后立即恢复 | 清掉后：目标随指令变化、拒发计数不再增长、逐拍目标/实际对得上（5 mm / 0.8°） | 同上 |

> **流程提示（本次踩到）**：跑任何 sim2sim/真机用例**之前**先确认没有残留的同名节点
> （`arm_stub_node` / `arm_real_adapter` / 第二个 `arm_teleop` / 第二个 `vr_teleop`），
> 否则会出现"两个发布者交替"的伪故障；另外**只靠仿真自己的腿部默认保持，机器人会缓慢
> 摔倒**，必须 `rl_deploy` 走到 `rl_control`。两项都被记进 `TODO_zh.md`（P2-8）与
> `COMMANDS_SIM2SIM_zh.md` §5.3。

---

## 二十五、头显页面离线化：不再依赖公网 CDN 的 A-Frame（DEF-058，2026-10-13）

来源：操作员"真机上 `https://10.21.31.103:8443` 能打开，但进去就卡住、只有中间一张
`VR Teleoperation` 卡片、点什么都没反应"。完整条目见 `DEFECT_LOG_zh.md` 的 **DEF-058**；
部署步骤见 `docs/DEPLOY_REAL_VR_zh.md` §1.6。

**结论**：不是上位机算力问题（HTTPS:8443 / WS:8442 / 静态文件都在正常服务）。
是页面 `<head>` 里从 `https://aframe.io/...` **公网 CDN** 现拉的 `aframe.min.js`
拉不到 ⇒ 阻塞解析 + `vr_app.js` 第一行就 `AFRAME is not defined` ⇒ `<a-scene>` 与
两个启动按钮全都不存在；而 `interface.js` 照常执行、检测到设备支持 `immersive-vr`
就只显示那个静态居中卡片（`z-index:9999`）⇒ 表现就是"卡住 + 点不动"。

| 日期 | 内容 | 关键实测 | commit / 分支 |
|---|---|---|---|
| 2026-10-13 | `web-ui/index.html` 改为本地 `aframe.min.js`（`defer`，顺序在 `vr_app.js` 之前），删掉阻塞式 CDN 引用；加兜底脚本：本地缺失 → 页顶红色横幅（头显看不到控制台）+ 若头显能上网就从 CDN 补一份并只刷新一次 | 页面内联脚本 `node` 逐段解析 **2/2 OK**；`defer` 顺序 `aframe.min.js → vr_app.js → interface.js`；CDN 地址仅存在于兜底列表 | `sync/2026-10-13-vr-ip` |
| 2026-10-13 | 新增 `scripts/fetch_aframe.sh`：6 个镜像依次尝试 → 校验（够大/非 HTML/含 `AFRAME`）→ 落到 `web-ui/`；`--check` 自检、`--url`/`--dest` 可指定 | `--check`：真缺 → `exit 1` 且指出路径；假文件 585 KB → `OK/exit 0`；HTML 错误页 → 拒绝 | 同上 |
| 2026-10-13 | `sync_to_robot.sh` 同步前打印 A-Frame 是否就位（缺了给出"头显里会怎样 + 该跑哪条命令"）；`vr_teleop_device.py` 启动横幅加 `头显页面: 已自带离线 A-Frame（xxxx KB）` / ⚠️+修法；HTTPS 服务 `HTTPServer → ThreadingHTTPServer`（`daemon_threads=True`，浏览器并发 6 条连接不再互相堵） | `_display_info()` 两分支本机模拟通过（缺 → ⚠️+修法；有 → `已自带离线 A-Frame（390 KB）`）；`py_compile` OK | 同上 |

> **闭环**：2026-10-13 操作员把 `aframe.min.js` 放到 `web-ui/` 后头显页面恢复正常，
> **机械臂遥操正常**（原"待复核"项 `TODO_zh.md` P2-9 已勾掉）。
>
> **顺带产出**：把所有"必须先在**有外网**的机器上下载再搬过去"的东西集中成
> [`docs/OFFLINE_ASSETS_zh.md`](../OFFLINE_ASSETS_zh.md) —— A-Frame、上位机的
> `PyYAML`/`websockets` wheel、明确**不用**下的（scipy / numpy / pynput / torch）、
> 各自放哪 / 怎么下 / 怎么自检 / 缺了是什么症状 / 换机器后要重做哪些。

---

## 二十六、真机"左前侧摆关节过温 100 ℃"的仿真侧排查（DEF-059，2026-10-13）

**问题（操作员）**：真机进入**站立状态机**或 **rl_control** 一段时间后，
**左前侧摆关节（`fl_hipx`）温度升到 100 ℃** ⇒ 触发安全保护 ⇒ 趴下；
约 **10 s** 后温度掉回阈值下 ⇒ 又能恢复。问：这条现象在 sim2sim 里会不会出现？
仿真有没有温度模拟？或者该关节的力矩是否异常？

**结论一：仿真侧没有温度模拟，这条现象在 sim2sim 里既不会出现、也无法验证。**

* `mujoco_simulation_ros2.py:836-837`（腿）/ `:874-875`（臂）把
  `motion_temp = 40.0`、`driver_temp = 45.0` **写死**（`# Dummy normal temp`）；
* `SafeController::IsMotorTempertureNormal()`（`safe_controller.hpp:190`）判的是
  `max_temp > 100`（`:202`）⇒ **永远不成立** ⇒ 温度那条 `safe_control_mode=2` 接管路径
  在 sim2sim 里不可达；
* 真机那条链路已逐行核对：`:202` 置 `safe_control_mode = 2` ⇒ `standup_state.hpp` /
  `rl_control_state.hpp` 的 `LoseControlJudge()` ⇒ `JointDampingState`（`joint_damping_state.hpp`
  只发 `kd = 2/1`、无位置目标 = **卸力**）⇒ **趴下**；温度掉回后
  `safe_controller.hpp:396-399` 打印 `Safe control recovered: safe_control_mode -> 0`
  ⇒ **又能恢复**。与操作员描述的现象链**逐环对应**。

**结论二：力矩确实"异常"——是长期**静载**，不是抖动。**

用 `M20_SIM_TELEMETRY` 落 200 Hz 遥测，再跑新增的 `tests/joint_torque_report.py`：

| 状态（用例） | 腿 kp/kd | `fl_hipx` 稳态力矩 | `fr` / `hl` / `hr_hipx` | 轮接触力 fl/fr/hl/hr | 站高 |
|---|---|---|---|---|---|
| 站立状态机（`--mode stand`，47 s） | 200 / 4 | **+12.0 N·m** | −11.9 / +8.0 / −7.9 | 106.5/105.9/86.2/85.6 N | 0.514 m |
| 仿真自带保持（`--mode hold`，42 s） | 80 / 2 | **+18.6 N·m** | −18.5 / +8.7 / −8.6 | 114.1/113.6/78.5/77.9 N | 0.479 m |
| RL 站立零命令（`--mode rl`，64 s / 45 s / 154 s 三次） | 80 / 2 | **+15.1 N·m** | −6.6 / +4.6 / −13.6 | 127.0/80.5/70.2/106.5 N | 0.510 m |

> 窗口口径：`stand`/`hold` 取 t≥15 s、`rl` 取 t≥25 s（`--t0`）。进 RL 后要 ~30 s 才收敛，
> 窗口取早（t≥15 s）会读到 fl 15.24 / hr −12.44 这样的"还在收"的值。
> `hold` 档（仿真自己的 kp=80/2 保持**训练默认姿态**）会**下沉到 0.479 m、倾角 4.4°**，
> 前腿侧摆因此更高（18.6±3.0 N·m、|max| 23.2）—— 站立状态机（kp=200）把它保持得更紧
> （0.514 m），所以"进 RL 前"和"进 RL 后"是两个不同的工况。

* **时间演化**：进 RL 后 `fl_hipx` 先冲到 +20.2 N·m（t≈8 s 入口瞬态），t≈30 s 起**钉在
  +15.0 N·m**；154 s 的长用例在 **t≥70 s 之后完全不变**（fl +15.03 / fr −6.44 /
  hl +4.60 / hr −13.68，站高 0.5097 m）⇒ **稳态静载**，不是漂移。
* **热负荷**（`mean(τ²)`，与铜损同量纲）：腿上前四名是
  `fl_knee 460.7 > fl_hipy 266.5 > hr_knee 240.7 >` **`fl_hipx 227.2`**；
  四个侧摆里 **`fl_hipx` 是最大的，是 `fr_hipx`(43.0) 的 5.3 倍**。
* **频谱**：四个侧摆 >5 Hz 的幅值全 **< 0.001 N·m** ⇒ 仿真里**没有颤振**，
  热全部来自"直流顶力矩"（角速度≈0 ⇒ 无机械功率，只有 `P ≈ (τ/k_t)²·R`）。
* **机理（可独立复算）**：`fl_hipx` 轴在机体侧（MJCF `y = +0.0685`），
  轮子接地点在 `y ≈ 0.2266` ⇒ 力臂 ≈ 0.158 m；前腿承担 ~106 N（前后 55/45）
  ⇒ 侧摆关节必须顶住 `106 × 0.158 ≈ 16.7 N·m`，扣掉腿自重 ≈2 N·m 后与实测 12~15 N·m
  同量级。**这是 M20 腿型 + 站立姿态的固有静负荷**，与 kp 无关（kp 只决定"让多少角度"：
  状态机 kp=200 停在 0.060 rad 误差上、RL kp=80 停在 0.188 rad）。

**交付物**：`tests/joint_torque_report.py`（离线看 τ 统计 / 热负荷排序 / 频谱 / 饱和 /
1 Hz 时间序列；用法见文件头）、`DEFECT_LOG_zh.md` 的 **DEF-059**、
`TODO_zh.md` 的 **P1-7**（真机核对 / 给仿真加一阶热模型 / 姿态降载三条路）。
原始遥测 CSV 在 `tmp/temp_probe/`（`tmp/` 已 gitignore，单次 10~33 MB）。

**遗留（未验证，别当成结论）**：① 真机 `fl_hipx` 是"纯静载"还是"静载 + 抖动/额外损耗"
（仿真测不到，要查电流环/减速箱/装配/线束）；② 仿真的三次 RL 用例**共用一份确定性策略**，
"重的一侧永远是左前"不能当独立样本；③ 站立状态机里左右其实基本对称（12.0 / −11.9）
⇒ 若操作员看到的是"刚站起来"阶段就左前先热，那更可能是**真机自身左右差异**。
