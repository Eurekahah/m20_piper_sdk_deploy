# sim2sim 运行命令速查（MuJoCo）

**这份文档只管"在仿真里跑，哪个终端敲什么命令"。** 真机请用
[`COMMANDS_SIM2REAL_zh.md`](COMMANDS_SIM2REAL_zh.md)。

> 每个场景都是"**一个标题 + 一张终端表 + 检查点**"，照抄即可。
> 原理/接口/缺陷记录见 [`docs/`](../../docs/README_zh.md)，本文不重复。

## 0. 通用约定（所有场景都适用）

| 项 | 值 |
|---|---|
| 仓库根目录 | `/root/m20_piper_ws`（容器 `m20_piper_ros` 内；宿主机是仓库所在目录） |
| 每个终端先做 | `source /opt/ros/humble/setup.bash && source install/setup.bash` |
| `ROS_DOMAIN_ID` | **1**（和真机隔离；所有终端必须一致） |
| 编译（改过 C++ 后） | `colcon build --packages-up-to m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON` |
| 无头跑 | 加 `M20_USE_VIEWER=0`（默认带窗口；需要 X11） |
| 启动容器 | `docker start m20_piper_ros && docker exec -it m20_piper_ros bash` |

下面用 **T1/T2/T3/T4** 表示"第几个终端"。

---

## 1. 场景 A：全仿真（腿 + 臂都来自 MuJoCo）——最常用

```bash
bash src/M20_sdk_deploy/scripts/run_sim2sim.sh full      # T1：只起 MuJoCo
```

| 终端 | 命令 | 说明 |
|---|---|---|
| T1 | `bash src/M20_sdk_deploy/scripts/run_sim2sim.sh full` | MuJoCo（同时提供腿和臂的数据） |
| T2 | `ros2 run m20_sdk_deploy rl_deploy` | 策略 + 状态机：键盘 `z` 站立 → `c` 进 RL |
| T3（可选） | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py` | 臂键盘；**要先起 `arm_controller`** 才能真正动臂（见下） |
| T3'（可选） | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py` | 臂 IK；用臂就必须起它 |

**检查点**

```bash
ros2 topic hz /JOINTS_DATA            # ~200 Hz
ros2 topic hz /ARM_JOINTS_DATA        # ~200 Hz
ros2 topic echo --once /ARM_TELEOP_STATE   # 默认位姿时 ≈ (0.3492,0,0.4327,0.7373,0,0.6756,0)
```

`rl_deploy` 里：`z` 站立、`c` 进 RL、`w/a/s/d/q/e` 走、`h/j` 高度、`b/n` 俯仰、`[`/`]` 横滚、`r` 阻尼、`x` 趴下。

---

## 2. 场景 B：只调腿（臂不参与，假臂保持默认姿态）

和真机"没装臂"完全同构：MuJoCo **不发**臂反馈，由假臂节点提供默认姿态。

```bash
bash src/M20_sdk_deploy/scripts/run_sim2sim.sh legs       # T1：MuJoCo + 假臂
```

| 终端 | 命令 |
|---|---|
| T1 | `bash src/M20_sdk_deploy/scripts/run_sim2sim.sh legs` |
| T2 | `ros2 run m20_sdk_deploy rl_deploy`（`z` → `c`） |
| 自动化 | `python3 tests/sim2sim_smoke.py --mode no_arm` |

**检查点**：策略第一拍观测里的 8 个臂项应为 0（= 默认姿态）、`ee_goal` = 默认位姿
（`--mode no_arm` 会自动断言并打印）。

---

## 3. 场景 C：只调机械臂（**不起 rl_deploy**）

臂链路和策略完全解耦，可以先单独调。

```bash
bash src/M20_sdk_deploy/scripts/run_sim2sim.sh arm        # T1：MuJoCo + IK
```

| 终端 | 命令 |
|---|---|
| T1 | `bash src/M20_sdk_deploy/scripts/run_sim2sim.sh arm` |
| T2 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py` |
| 自动化 | `python3 tests/sim2sim_smoke.py --mode arm_only` |

臂键盘：numpad `8/2` x、`4/6` y、`7/9` z、`1/3` roll、`0/.` pitch、`+/-` yaw、`G` 夹爪、`L` 复位目标。
**退出**：`Ctrl+C`（或 `ESC`）。

**检查点**：轻点一次 numpad 后 `ros2 topic echo --once /ARM_TELEOP_STATE` 的数值应变化；
`--mode arm_only` 的实测参考：末端 +0.20 m、IK 稳态残差 0.63 mm。

---

## 4. 场景 D：腿仿真 + 臂走"独立传输"（和真机拓扑一致）

模拟真机结构：腿来自 MuJoCo，臂走"外部传输"（假 CAN 驱动 / 真机由 agx 提供）。

```bash
bash src/M20_sdk_deploy/scripts/run_sim2sim.sh mixed      # T1：MuJoCo(腿) + 假CAN + IK
```

| 终端 | 命令 |
|---|---|
| T1 | `bash src/M20_sdk_deploy/scripts/run_sim2sim.sh mixed` |
| T2 | `ros2 run m20_sdk_deploy rl_deploy`（`z` → `c`） |
| T3 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py` |
| 自动化 | `python3 tests/sim2sim_smoke.py --mode arm_mixed` |

**检查点**：按住 numpad 8 后，策略观测里的臂项应跟着变（`--mode arm_mixed` 实测变化 1.03 rad），
底盘不摔（tilt ≤ 1.2°）。

---

## 5. 场景 E：VR 遥操作（腿/机身 + 臂）

| 终端 | 命令 |
|---|---|
| T1 | `bash src/M20_sdk_deploy/scripts/run_sim2sim.sh mixed`（或 `full`） |
| T2 | `ros2 run m20_sdk_deploy rl_deploy` |
| T3 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py` |
| T4 | `python3 src/M20_sdk_deploy/interface/robot/simulation/vr_teleop_node.py` |

依赖：`pip3 install -r src/M20_sdk_deploy/requirements_teleop.txt`；头显里打开打印出的 `https://<ip>:8443`。
按右 B 启动/重标定；左 X/Y = 复位仿真（调 `/reset_sim`）。

---

## 6. 场景 F：一键自动化验收（不需要人看）

```bash
bash tests/run_all.sh                      # L0 编译 + L1 + L3' + 全部 L3 用例
SKIP_BUILD=1 MODES="hold rl" bash tests/run_all.sh     # 只跑子集
```

| 用例 | 测什么 | 判据 |
|---|---|---|
| `--mode hold` | 裸模型站立 | 高度 ≥0.30 m、倾角 ≤46° |
| `--mode wheel_step` | 轮子速度伺服阶跃 | 四轮稳态 ≈ +5 rad/s（±10%） |
| `--mode no_arm` | 无臂部署口径 | 观测里的臂项 = 0、ee_goal = 默认 |
| `--mode arm_only` | 只调臂（无策略） | 末端位移 >0.03 m、IK 残差中位数 <3 mm |
| `--mode arm_mixed` | 臂走独立传输 | 按键后观测里的臂项变化 >0.05 rad |
| `--mode rl` / `walk` / `arm_move` | 进 RL / 行走 / 动臂 | 见 `docs/review/DONE_zh.md`（含入口失败率） |
| `--mode push` | 800 N 侧推 | 必须触发 `[TAKEOVER!]` 并切 `joint_damping` |

---

## 7. 场景 G：换策略（换一个 run 的 ONNX）

```bash
# 1) 把训练侧 <run>/exported_deploy/ 整个目录复制成 policy/<实验名>_<日期>/
# 2) 给它补 joint_order_native / joint_order_action（见 policy/README.md）
python3 src/M20_sdk_deploy/scripts/check_policy_interface.py policy/<新目录>   # L1 必须 PASS
# 3) 试跑
M20_POLICY_DIR=policy/<新目录> ros2 run m20_sdk_deploy rl_deploy
# 4) 确认无误后改 rl_control_state.hpp 里的默认目录，再跑 tests/run_all.sh
```

---

## 8. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| MuJoCo 起不来 / 窗口报 GL 错 | 需要 X11：宿主机 `xhost +local:docker`，容器里 `DISPLAY`/`QT_X11_NO_MITSHM=1`；或直接 `M20_USE_VIEWER=0` 无头跑 |
| `rl_deploy` 报 `Policy dim mismatch` | 策略目录与 runner 布局不符（例如拿了旧的 86/770/23 模型）；用 `check_policy_interface.py` 查 |
| 按 `c` 后偶发抖动/摔倒 | 已知的"进 RL 入口瞬态"（`DEF-018`，策略侧边缘稳定性）；用 `--repeat` 看失败率 |
| 仿真运行变慢/结果不可复现 | 同一台机器别同时跑两个 MuJoCo 实例（`sim2sim_smoke.py` 开跑前会自动检查残留） |
| 想复位机器人 | `ros2 service call /reset_sim std_srvs/srv/Empty`（会清腿/臂命令缓冲） |
