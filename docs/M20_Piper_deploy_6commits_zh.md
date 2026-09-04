# M20 + Piper sim2sim 部署：6 次本地提交总结（中文）

> 说明：本文对应 `origin/main..HEAD` 的 6 个本地提交，尚未 push 到远程。
> 分支：`main`

| # | Commit | 日期 | 提交信息 | 主题 |
|---|--------|------|----------|------|
| 1 | `9095957` | 2026-08-20 | add: 上传M20_Piper的mjcf模型，缺少传感器 | 模型资产 |
| 2 | `efdc2d8` | 2026-08-20 | feat(sim): M20+Piper 24-DOF mujoco sim with dual topics | 仿真节点 |
| 3 | `0b8acbb` | 2026-08-20 | feat(rl): WBC policy runner, arm interface, teleop keys, 24-dof types | RL 部署框架 |
| 4 | `66bcab1` | 2026-08-20 | feat(arm): Piper arm controller node (pyAgxArm MDH + DLS IK) | 机械臂 IK 节点 |
| 5 | `ee5c264` | 2026-08-20 | feat(rl): support history_adaptation policy (history encoder + actor) | history 策略导出与部署 |
| 6 | `2fcab7c` | 2026-09-04 | fix(sim): stabilize M20+Piper sim2sim physics and bound policy actions | sim2sim 稳定性整理提交 |

---

## 第 1 次提交：`9095957`

**上传 M20 + Piper 的 MuJoCo 模型（当时缺少传感器）**

新增 `src/M20_sdk_deploy/M20_Piper_description/`：

- `mjcf/M20_Piper_own.xml`：M20 四轮腿 + Piper 六轴臂 + 夹爪的 MJCF 模型；
- `meshes/*.STL`：base、四条腿/轮、机械臂各连杆与夹爪的网格文件。

这个提交只是把模型带进 SDK 仓库，传感器、执行器参数和仿真脚本由后续提交补齐。

---

## 第 2 次提交：`efdc2d8`

**让 MuJoCo 仿真支持 M20 + Piper 共 24 DOF，并使用双 topic 通信**

改动集中在：

- `M20_Piper_own.xml`
  - 增加 `imu_site` / `base_site` 以及 framequat / accelerometer / gyro 传感器；
  - 按 Isaac Lab 训练配置增加各执行器力矩限制（腿 76.4、轮 21.6、臂 100、夹爪 10）。
- `mujoco_simulation_ros2.py`
  - 加载 24 DOF 模型（16 腿轮 + 6 臂 + 2 夹爪）；
  - `/JOINTS_CMD` 驱动 16 个腿轮关节，新增 `/ARM_JOINTS_CMD` 驱动 8 个臂/夹爪关节；
  - 发布 `/JOINTS_DATA`、`/ARM_JOINTS_DATA`、`/IMU_DATA`；
  - 自动把机器人放到轮子着地的高度；
  - 在第一条机械臂命令到来前，用默认位姿保持机械臂。

此时腿轮仍使用官方 16 关节的 dir/offset 标定；机械臂/夹爪使用单位标定（raw rad）。

---

## 第 3 次提交：`0b8acbb`

**加入 WBC 策略部署框架、机械臂 ROS2 接口、遥操作按键和 24 DOF 类型**

主要改动：

- `common_types.h`：`UserCommand` 增加机身高度/俯仰/横滚、机械臂 EE 增量、夹爪与 `ee_reset`、绝对 EE 目标等字段；
  `RobotBasicState` 默认 24 DOF；
- 新增 `PiperArmInterface`：8 DOF 的 `/ARM_JOINTS_CMD` 发布与 `/ARM_JOINTS_DATA` 订阅，
  使用单位标定，默认位姿保持增益为 Isaac Lab 的 300/20、夹爪 4000/200；
- 新增第一版 `M20PiperPolicyRunner`（obs 79、action 16）：
  - 12 腿 + 4 轮 + 6 臂的观测顺序；
  - 轮子位置在位置观测中置零；
  - last_action 使用处理后的动作；
  - 默认位姿为 hipy ±0.6、knee ∓1.0、arm2=0.5、arm3=-0.5；
- `RLControlState`：把 16 腿轮和 8 臂/夹爪合成 24 DOF 观测，转发 `/ARM_TELEOP` 并接收 `/ARM_TELEOP_STATE`；
- 键盘（stdin）增加机身姿态、numpad EE、夹爪、复位等按键。

---

## 第 4 次提交：`66bcab1`

**新增 Piper 机械臂 IK 控制器节点 `arm_controller.py`**

- 50 Hz 绝对 EE 目标跟踪，DLS（damped least squares）IK，lambda=0.01，与 Isaac Lab `DifferentialIKController` 对齐；
- MDH 参数优先从 `pyAgxArm.utiles.mdh_kinematics` 获取，无 SDK 时使用内嵌 fallback；
- 订阅 `/ARM_TELEOP`（增量 + 夹爪 + reset），反馈 `/ARM_JOINTS_DATA`；
- 发布 `/ARM_JOINTS_CMD`（目标 + 增益）和 `/ARM_TELEOP_STATE`（供策略 obs 使用）；
- README 增加 M20+Piper sim2sim 架构、三终端运行流程和按键说明。

---

## 第 5 次提交：`ee5c264`

**支持 `ActorCriticHistory` 策略（history encoder + actor）的部署**

由于 rsl_rl 标准 ONNX 导出只包含 actor MLP（输入 118 = policy obs 86 + latent 32），
本次提交加入：

- `scripts/export_history_policy_onnx.py`：从 checkpoint 重建 TCN `HistoryEncoder` + actor，
  导出一个输入为 `obs [1,86]`、`obs_history [1,770]`，输出 `actions [1,23]` 的完整 ONNX；
- `policy/history_adaptation_full.onnx`：由 `model_19999.pt` 生成的完整模型；
- `M20PiperPolicyRunner` 升级为：
  - policy obs 86：角速度(3) + 重力(3) + 指令(3) + 22 关节位置/速度 + 23 last action + 7 EE goal + 3 body pose；
  - history obs：10 × 77 的滚动窗口；
  - action 23：12 腿位置 + 4 轮速度 + 7 ee_ik（部署时只使用前 16 维控制腿轮）；
- `RLControlState` 默认加载 `policy/history_adaptation_full.onnx`。

> 注意：该提交中的 history 关节顺序按“arm first”编写，后续第 6 次提交已按老 checkpoint
> （`M20_adjusted`）实际顺序修正为“12 腿 → 4 轮 → 臂/夹爪”。

---

## 第 6 次提交：`2fcab7c`

**当前工作区整理的提交：sim2sim 稳定性修复 + 动作限幅**

这是在本次任务中整理并提交的改动：

1. **SIM2SIM 单位标定接口**
   - CMake 增加 `SIM2SIM` 选项；
   - 新增 `M20SimInterface`，腿轮 dir=1、offset=0，避免真机多圈关节标定污染仿真坐标系。
2. **MuJoCo 物理子步**
   - 原 1 ms 单步会让模型产生约 500 Hz 机身震荡，IMU 读到 ±2 rad/s 的假角速度；
   - 仿真改为 0.2 ms × 5 子步，仍然每 1 ms 控制 tick、5 ms 发布一次反馈。
3. **执行器 armature**
   - 按 Isaac Lab actuator 配置补齐：臂/夹爪 0.01、轮 0.00243216；
   - 避免机械臂/夹爪因惯量过小在 MuJoCo 显式 PD 下失稳。
4. **History 关节顺序**
   - 按实际使用的 `history_adaptation_full.onnx`（`M20_adjusted` 老模型）改为
     “12 腿 → 4 轮 → 6 臂 → 2 夹爪”。
5. **动作限幅**
   - 原始 ONNX 输出在标称状态也会发散，部署侧把 raw action 限制在 ±3
     （轮速约 ±15 rad/s），避免轮子位置 runaway、NaN 和关节指令爆炸。
6. **清理**
   - 删除 `SetJointCommand publish!!` 等高频打印和临时 TEMP DEBUG；
   - README 补充编译参数 `-DSIM2SIM=ON` 与稳定性说明；
   - 补入 `policy/history_adaptation.onnx`（actor-only，调试用）。

---

## 整体上部署需要修改/注意的点

### 1. 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to m20_sdk_deploy \
  --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON
```

必须加 `-DSIM2SIM=ON`，否则 `qw_state_machine` 会走真机 `M20Interface`，
仿真坐标系会再次被多圈 offset 标定破坏。

### 2. 运行流程

三个终端（机械臂需要动作时）：

```bash
# Terminal 1: RL 部署
export ROS_DOMAIN_ID=1
source install/setup.bash
ros2 run m20_sdk_deploy rl_deploy

# Terminal 2: MuJoCo 仿真
export ROS_DOMAIN_ID=1
source install/setup.bash
M20_USE_VIEWER=1 python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py

# Terminal 3: 机械臂 IK（可选；机械臂静止时可不开）
export ROS_DOMAIN_ID=1
source install/setup.bash
python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py
```

交互：`z` 站立，站立完成后 `c` 进入 RL。

### 3. 仿真/模型相关

- `M20_Piper_own.xml` 必须保留：传感器、执行器力矩限制、本轮提交补上的 armature；
- `mujoco_simulation_ros2.py` 使用 0.2 ms 物理子步，不要回退成 `model.opt.timestep = 0.001`；
- 可视化需要容器/主机有 X11，并设置 `DISPLAY`、`QT_X11_NO_MITSHM=1`、`MUJOCO_GL=glfw`；
- 纯无头调试设 `M20_USE_VIEWER=0`。

### 4. 策略部署侧

- 策略文件：
  - `policy/history_adaptation_full.onnx`（history encoder + actor，部署实际加载）；
  - `policy/history_adaptation.onnx`（actor-only，调试/对比用）；
- 输入：`obs [1,86]` + `obs_history [1,770]`；输出 `actions [1,23]`；
- 动作前 16 维用于腿轮，后 7 维 ee_ik 在部署中被忽略但仍会进入 last-action 观测；
- runner 已对原始动作做 ±3 限幅，这是防 NaN/runaway 的安全网。

### 5. 已知问题 / 需要进一步处理

当前 `history_adaptation_full.onnx` 在“标称默认状态”下第一拍就不会输出接近零的动作，
限幅后命令有界、不会 NaN，但机器人仍会被策略压到侧躺，无法稳定平衡。

如果目标是稳定站立/行走，建议：

- 用当前 `M20_Piper_own` 模型重新训练并导出新的 `history_adaptation_full.onnx`；
- 或确认现有 checkpoint 的观测/动作语义与训练侧完全一致后再继续调部署侧。

### 6. 本机部署环境（不在 git 内）

- 已 commit 镜像：`m20-piper-deploy:latest`（由旧容器保存依赖）；
- 新容器：`m20_piper_dev`，挂载当前仓库到 `/root/m20_piper_ws`，
  `--privileged --network=host` + X11；
- 旧容器 `ros2_mujoco_dev`（`71775a4d39ca`）已停止。

---

## 文件清单

主要改动目录：

- `src/M20_sdk_deploy/interface/robot/simulation/`：MuJoCo 仿真、arm_controller；
- `src/M20_sdk_deploy/interface/robot/hardware/`：M20SimInterface、PiperArmInterface；
- `src/M20_sdk_deploy/run_policy/`：M20PiperPolicyRunner；
- `src/M20_sdk_deploy/state_machine/quadruped_wheel/`：idle/standup/rl_control 状态；
- `src/M20_sdk_deploy/M20_Piper_description/`：MJCF 模型与 mesh；
- `src/M20_sdk_deploy/policy/`：ONNX 策略文件。
