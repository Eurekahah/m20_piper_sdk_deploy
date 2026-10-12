# sim2sim 运行命令速查（宿主机一行式）

**这份文档只管"在仿真里跑，在宿主机粘哪条命令"。** 真机请用
[`COMMANDS_SIM2REAL_zh.md`](COMMANDS_SIM2REAL_zh.md)。

> **与旧版的区别**：主命令全是**宿主机一行式**（前面带 `docker exec`），
> 不用先 `docker exec -it bash` 进容器再敲。原理/接口/缺陷见
> [`docs/README_zh.md`](../../docs/README_zh.md)，本文不重复。

下面用 **A1/A2/A3…** 表示"第几个终端"，每个终端粘**一条**命令即可。

---

## 0. 通用约定

| 项 | 值 |
|---|---|
| 容器名 | `m20_piper_ros` |
| 宿主机仓库 | `/home/eureka/code/m20_piper_sdk_deploy` |
| 容器内仓库 | `/root/m20_piper_ws` |
| 窗口显示 | 容器里 `DISPLAY=:1`、`MUJOCO_GL=glfw`、`QT_X11_NO_MITSHM=1` 都已配好；宿主机只需放行 X11（§0.1） |
| `ROS_DOMAIN_ID` | `1`（`run_sim2sim.sh` 自己会设；自己敲 `ros2` 命令时要带上） |
| 无头跑 | 在 `exec` 前加 `M20_USE_VIEWER=0` |
| 实时因子 | 仿真那个终端每秒打印 `[SIM-RT] ... 实时因子 x.xx`；**≈1.00 才正常**，见 §9.1 |

### 0.1 只做一次的准备

```bash
docker start m20_piper_ros
DISPLAY=:1 xhost +local:
```

撤销 X11 放行：`DISPLAY=:1 xhost -local:`。窗口报
`Authorization required, but no authorization protocol specified` /
`Failed to open display :1` 就是这条被清掉了，重跑一次即可。

### 0.2 少打字：定义 `m20()` 函数（**可选**）

```bash
m20() { docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec "$@"' _ "$@"; }
```

定义之后就能写 `m20 ros2 topic hz /JOINTS_DATA` 这种短形式（等价于把 `m20` 换成
`docker exec -it m20_piper_ros bash -lc '...'` 那一长串）。
**本文所有命令都给的是完整形式，不定义这个函数也能直接粘**；
用 `"$@"` 透传是为了让**带空格的参数**（例如
`m20 env MODES="hold rl" bash tests/run_all.sh`）也能正确送到容器里。

### 0.3 改过 C++ 之后重编

```bash
docker exec m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && colcon build --packages-up-to m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON'
```

---

## 1. 场景 A：全仿真（腿 + 臂都来自 MuJoCo）—— 最常用

| 终端 | 说明 |
|---|---|
| A1 | MuJoCo（同时提供腿和臂的数据，带窗口） |
| A2 | 策略 + 状态机：键盘 `z` 站立 → `c` 进 RL |

**A1：**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh full'
```

**A2：**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ./install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy'
```

> A2 直接调编译出来的二进制，不用 `ros2 run`：`ros2 run` 是个包装进程，
> `kill` 它**不会**杀掉真正的 `rl_deploy`，会留下孤儿实例继续发 `/JOINTS_CMD`
> （表现：机器人自己乱动、必倒）。配 `exec` 用 `Ctrl+C` 收工最干净。

**检查点**（另开一个终端）：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 topic hz /JOINTS_DATA'                 # ~200 Hz

docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 topic hz /ARM_JOINTS_DATA'             # ~200 Hz

docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 topic echo --once /ARM_TELEOP_STATE'   # 默认位姿 ≈ (0.3492,0,0.4327,-0.7373,0,-0.6756,0)（四元数=训练/链式 FK 的符号分支，DEF-039）
```

**想动臂**：A2 的 numpad 键**不驱动臂**，臂只认独立的 `arm_teleop_node`（见场景 C/D）。

---

## 2. 场景 B：只调腿（臂不参与，假臂保持默认姿态）

和真机"没装臂"完全同构：MuJoCo **不发**臂反馈，由假臂节点提供默认姿态。

```bash
# A1
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh legs'

# A2
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ./install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy'
```

**检查点**：策略第一拍观测里的 8 个臂项应为 0（= 默认姿态）、`ee_goal` = 默认位姿。
自动化：`--mode no_arm`（见 §6）。

---

## 3. 场景 C：只调机械臂（**不起 `rl_deploy`**）

臂链路和策略完全解耦，可以先单独调；臂是开环 IK，**不受实时因子影响**，
开着窗口看最舒服。

```bash
# A1：仿真 + arm_controller(IK)
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh arm'

# A2：臂键盘（numpad）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py'
```

**检查点**：轻点一次 numpad 后，另开终端看 `ros2 topic echo --once /ARM_TELEOP_STATE` 数值应变化：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 topic echo --once /ARM_TELEOP_STATE'
```

自动化：`--mode arm_only`（实测参考：末端 +0.20 m、IK 稳态残差 0.63 mm）。

---

## 4. 场景 D：腿仿真 + 臂走"独立传输"（和真机拓扑一致）

```bash
# A1：腿来自 MuJoCo，臂走假 CAN 驱动
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh mixed'

# A2：策略
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ./install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy'

# A3：臂键盘
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py'
```

想用真臂替代假 CAN 驱动：A1 前面加 `M20_ARM_TRANSPORT=real`（需要先起
`agx_arm_ctrl`，见真机文档）。

**检查点**：按住 numpad 8 后策略观测里的臂项应跟着变（`--mode arm_mixed` 实测 1.03 rad）。

---

## 5. 场景 E：VR 遥操作（腿/机身 + 臂）

四个终端都要开，全部是宿主机一行式。

**A1：MuJoCo（带窗口）**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh full'
```

> 想和真机拓扑一致（腿来自仿真、臂走"独立传输"）就把结尾的 `full` 换成 `mixed`。

**A2：策略 + 状态机（键盘 `z` 站立 → `c` 进 RL；VR 接管速度/机身）**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ./install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy'
```

**A3：臂 IK（不动臂的话可以不起）**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py'
```

**A4：VR 节点（会打印 `https://<ip>:8443`，用头显打开这个地址）**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/interface/robot/simulation/vr_teleop_node.py'
```

**首次使用要先装 VR 依赖**（下面这条只需跑一次；装完再起 A4）：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec pip3 install -r src/M20_sdk_deploy/requirements_teleop.txt'
```

操作：头显里按**右 B** 启动 / 重标定（把 VR 坐标系锚到当前机器人姿态），
**左 X/Y** 复位（调 `/reset_sim`，**同时把末端目标重置回默认位姿**）。

> **复位后 VR 保持激活**（校准原点自动重采）⇒ **不用再按 B**，手再动就能继续操控
> （操作员 2026-10-10 反馈"复位后不能操控了"就是因为老语义会把 VR 置为 inactive）。
> 要回训练侧老语义（复位即停、需再按 B）：`M20_VR_RESET_KEEPS_ACTIVE=0`。
> 复位完成时 `arm_teleop` 会打印 `[reset] /reset_sim 完成；EE 目标已重置到默认位姿`。

VR 一旦激活（B 按下），
键盘就不再写速度/机身字段（由 VR 独占），但 `z`/`c`/`r` 这些模式键仍由 A2 的键盘负责。

**左右手的分工（最容易踩的坑）**：

| 手柄 | 管什么 | 说明 |
|---|---|---|
| **右手柄** | **机械臂**（EE 位置 + 姿态） | 只有右手柄的位姿会进 `EE dx/dy/dz / droll/dpitch/dyaw` |
| **左手柄** | 机身 / 底盘（摇杆给 vx/vy/wz，位置/姿态给机身高度·俯仰·横滚） | 动左手柄**不会**让臂动 |
| 右手柄**扳机** | 夹爪（>0.5 闭合） | 不影响臂的运动 |

**速度限幅**（VR 是连续量，必须有上限）：默认 `vx ±1.0`、`vy ±0.5`、`wz ±0.7`
（`M20_VR_MAX_VX` / `_VY` / `_WZ` 可改），启动时 `rl_deploy` 会打印一行 `[VR] 速度限幅: ...`。

**臂不动时先做这一步链路自检**（8 秒采样，直接把"卡在哪一环"指出来；
采样期间请动一下右手柄）：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py --seconds 8'
```

它会打印 `/VR_TELEOP`（active、EE 偏移有没有在变）、`/ARM_TELEOP`、
`/ARM_JOINTS_CMD`、`/ARM_JOINTS_DATA` 的流量，并给出结论。判据速查：

| 现象 | 说明 |
|---|---|
| `[VR] Button B -> START + calibrate` 没打印 | 头显的 B 没上报（手柄不在视野 / 按钮映射）⇒ `active=0`，腿和臂都不会听 VR |
| `active=1` 但采样期 `\|EE 偏移\|` 恒为 0 | 右手柄没在动 / 丢跟踪 / 动得小于 4 mm 死区 |
| `/ARM_TELEOP` 没有数据 | `arm_teleop_node` 没起或崩了 |
| `/ARM_JOINTS_DATA` 没有数据 | 臂反馈没来（仿真/真臂侧没跑） |
| **某个话题有 2 个以上发布者** | ⚠️ 最坑的一种：两路命令**交替到达**，目标在两帧之间来回跳，表现就是"动作特别快 / 不跟手 / 夹爪乱拍"。常见原因：起了两个 `vr_teleop_node`（或两个 `arm_teleop_node`）。自检脚本会把发布者名字列出来 |

### 5.1 手感调节（VR 动得太快/太钝就调这两处）

**为什么 VR 下臂很快**：VR 的手臂跟随是 **1:1**（手柄动 10 cm ⇒ EE 目标也走 10 cm，
再乘 `M20_VR_ARM_POS_SENS`），所以"手快 → 臂快"；上限由 `arm_controller` 的
**VR 限速**决定，默认 **0.4 m/s / 1.0 rad/s**（键盘路径 0.35 m/s / 0.8 rad/s）。
每个 20 ms 的控制周期里最多走 `0.4 × 0.02 = 8 mm`。

两个旋钮含义不同，通常一起用：

| 想改什么 | 环境变量 | 默认 | 加在**哪个终端** |
|---|---|---|---|
| **速度上限**（削峰，手感变"钝"） | `M20_VR_MAX_LIN_SPEED` / `M20_VR_MAX_ANG_SPEED` | `0.4` m/s / `1.0` rad/s | **A3 `arm_controller`** |
| **灵敏度**（整体缩放，更细腻） | `M20_VR_ARM_POS_SENS` / `M20_VR_ARM_ROT_SENS` | `0.5` / `0.5` | **A4 `vr_teleop_node`** |
| 底盘/机身灵敏度（可选） | `M20_VR_VX_SENS` / `_VY_SENS` / `_WZ_SENS`、`_HEIGHT_SENS` / `_PITCH_SENS` / `_ROLL_SENS` | `2.0` / `1.0` / `1.0`、`0.5` / `0.35` / `0.25` | A4 |
| 夹爪"像真夹爪"（默认直接阶跃） | `M20_GRIPPER_SLEW` [stroke/s] | `0`（建议 `0.15~0.2`） | A3 |
| 关节指令限速（= 真机驱动器口径） | `M20_ARM_MAX_JOINT_VEL` [rad/s] | `3.0`（`0` = 不限，不建议） | A3 |
| VR 臂映射（见 §5.2；一般不要动） | `M20_VR_ARM_MAP` | `cylindrical`（`cartesian` = 旧行为） | A3 |
| 臂摆动角度上限 | `M20_VR_SWING_MAX_DEG` | `60` | A3 |
| IK 拒绝阈值（解不出来就不下发） | `M20_ARM_IK_MAX_POS_ERR` / `_ANG_ERR` | `0.010` m / `3.0` ° | A3 |
| IK 姿态正则（压手腕漂移） | `M20_ARM_IK_POSTURE_GAIN` | `0.3`（`0` = 关） | A3 |

> 上面这组默认值是 **2026-10-10 操作员实测确认手感合适** 后固化进代码的（原来是
> 2.0 m/s / 5.0 rad/s、灵敏度 1.0/1.0、油门 5.0）。要回到训练侧原值：
> `M20_VR_MAX_LIN_SPEED=2.0 M20_VR_MAX_ANG_SPEED=5.0 M20_VR_VX_SENS=5 M20_VR_ARM_POS_SENS=1 M20_VR_ARM_ROT_SENS=1`。

**想让仿真的臂/夹爪更像真机**（可选，默认都不限速）：

```bash
# A3 臂 IK：夹爪按真夹爪的行程时间走（关节限速现在默认就是真机的 3 rad/s）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 M20_GRIPPER_SLEW=0.2 && exec python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py'
```

起 A4 时 VR 节点会打印一行 `灵敏度（可用 M20_VR_*_SENS 覆盖）: ...`，可以核对是否生效。

### 5.2 VR 臂的映射（2026-10-12 改，DEF-044）

**为什么改**：默认站姿 `(0, 0.5, -0.5, 0, 0, 0)` 里 joint5 = joint6 = 0，joint4 与
joint6 同轴 ⇒ **手腕正好在奇异位形**。旧实现把 VR 偏移当"笛卡尔绝对偏移"（训练侧语义）
直接给局部 IK，而"EE 侧移 5 cm / 工具转 10°"在这个位形下需要 **2.4~2.7 rad** 的关节重排
（键盘的 ±x 只要 0.30 rad）⇒ IK 解不出来，节点又把"顶在限位上的解"发出去，
表现就是**臂自己甩到 joint1 = 2.168（上限）、关节速度 20 rad/s、指令力矩 139 N·m**。

**现在怎么做**（默认，全部在 `arm_controller`）：

| 手的动作 | 现在的含义 |
|---|---|
| 前伸/后收 | 沿臂当前朝向**伸缩**（半径 r） |
| 左/右移 | **基座摆动 Δθ**（臂自然摆过去，工具跟着摆；上限 `M20_VR_SWING_MAX_DEG=60°`） |
| 上/下 | 高度 z |
| 手腕自转/俯仰/侧倾 | **手腕关节** j6 / j5 / j4 的增量（永远可达；不再要求手腕去补偿整条臂） |

配套的两道保险：

* **IK 收敛检查**：位置残差 > 10 mm 或姿态残差 > 3° 时**不下发**（沿用上一拍指令 +
  把目标拉回实际位姿），日志里出现 `IK 未收敛…沿用上一拍指令`；
* **零空间姿态正则**：防止手腕沿"不影响任务但会漂到限位"的方向慢慢拧过去。

**回退**：`M20_VR_ARM_MAP=cartesian` 回到旧语义（只在做 A/B 时用）。

**验收**：`python3 tests/vr_arm_chain_check.py`（约 25 s；对旧行为会 FAIL）。

---

## 6. 场景 F：一键自动化验收（不需要人盯）

```bash
# 完整：L0 编译 + L1 + L3' + L2 + command_sweep + 10 档 L3
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash tests/run_all.sh'

# 只跑子集（跳过编译、跳过命令组合扫描）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec env SKIP_BUILD=1 SWEEP=0 MODES="hold rl" bash tests/run_all.sh'
```

想让脚本代打键盘、你只看窗口：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 tests/sim2sim_smoke.py --mode arm_move --duration 25 --viewer'

docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 tests/sim2sim_smoke.py --mode rl --duration 25 --viewer'

# 命令组合扫描（22 个阶段：vx/vy/wz/组合/机身姿态）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 tests/command_sweep.py --repeat 3'
```

| 用例 | 测什么 | 判据 |
|---|---|---|
| `--mode hold` | 裸模型站立 | 高度 ≥0.30 m、倾角 ≤46° |
| `--mode wheel_step` | 轮子速度伺服阶跃 | 四轮稳态 ≈ +5 rad/s（±10%） |
| `--mode no_arm` | 无臂部署口径 | 观测里的臂项 = 0、`ee_goal` = 默认 |
| `--mode arm_only` | 只调臂（无策略） | 末端位移 >0.03 m、IK 残差中位数 <3 mm |
| `--mode arm_mixed` | 臂走独立传输 | 按键后观测里的臂项变化 >0.05 rad |
| `--mode rl` / `walk` / `arm_move` | 进 RL / 行走 / 动臂 | 见 `docs/review/DONE_zh.md`（按失败率判，含入口瞬态） |
| `--mode push` | **1500 N** 侧推 | 必须触发 `[TAKEOVER!]` 并切 `joint_damping`（800 N 打不倒当前平地模型，见 §9.3） |

---

## 7. 场景 G：换策略（换一个 run 的 ONNX）

当前默认目录 = `policy/m20_piper_flat_20260930/`（**平地** run
`2026-09-30_19-36-00_cloud_cap12_20k`）；多地形用 `policy/m20_piper_rough_20261002/`
（run `2026-10-02_00-44-42_cloud_slowvx20k`）。这两个 + 更早的
`m20_piper_history_20260920/` 接口逐字段相同（同一资产、同为 83/700/16），换哪个都能跑。

```bash
# 1) 把训练侧 <run>/exported_deploy/ 整个目录复制成 policy/<实验名>_<日期>/
# 2) 给它补 joint_order_native / joint_order_action（见 policy/README.md）
# 3) L1 验收（路径按容器内 cwd 写，即 /root/m20_piper_ws 下的相对路径）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 src/M20_sdk_deploy/scripts/check_policy_interface.py src/M20_sdk_deploy/policy/<新目录>'

# 4) 试跑（把 §1 的 A2 换成下面这条）
# ⚠️ M20_POLICY_DIR 是**按进程 cwd 解析**的（代码默认目录却是按 __FILE__ 解析的，
#    两者口径不同）—— 所以这里必须写绝对路径，写 `policy/xxx` 会找不到：
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec env M20_POLICY_DIR=/root/m20_piper_ws/src/M20_sdk_deploy/policy/<目录> ./install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy'

# 5) 确认无误后改 rl_control_state.hpp 里的默认目录，再跑 tests/run_all.sh
```

---

## 8. 键盘速查

### 8.1 腿 / 机身（在 `rl_deploy` 的终端里按）

| 键 | 作用 |
|---|---|
| `z` / `c` / `r` / `x` | 站立 / 进 RL / 阻尼 / 趴下 |
| `w` `s` | 前进 / 后退 **±0.7 m/s** |
| `a` `d` | 左 / 右平移 **±0.5 m/s** |
| `q` `e` | 左 / 右转 **±0.7 rad/s** |
| `h` `j` | 机身抬高 / 降低 **±2 mm / 次** |
| `b` `n` | 机身俯仰 **±0.01 rad / 次** |
| `[` `]` | 机身旁滚 **±0.01 rad / 次** |
| `l` | 复位（机身姿态 + 臂目标 + 夹爪开） |

⚠️ **速度键是阶跃的**：按一下就立刻给到满量程，没有渐变。想"柔和"就别按它们
（只动 `h/j/b/n/[/]` 这类增量键），或者点一下马上松开、隔一会儿再点。

### 8.2 臂（在 `arm_teleop_node.py` 的终端里按，NumLock 打开）

| 键 | 作用 |
|---|---|
| numpad `8` / `2` | EE x + / − |
| numpad `4` / `6` | EE y + / − |
| numpad `7` / `9` | EE z + / − |
| numpad `1` / `3` | EE roll ± |
| numpad `0` / `.` | EE pitch ± |
| numpad `+` / `-` | EE yaw ± |
| `G` | 夹爪开 / 合 |
| `L` | 复位臂目标 + 张开夹爪 |

⚠️ 按一下会被当作"**按住 500 ms**"，一次轻点末端就走 ~0.1 m；按住不放会一路顶到
工作空间边界。**轻点**即可。退出 `Ctrl+C` 或 `ESC`。

### 8.3 两个通用注意

* 键盘只认**当前焦点终端**：要按 `rl_deploy` 的键，先点一下 A2 那个终端。
* 臂必须**先起 `arm_controller`**（IK）才会动。

---

## 9. 收工 / 排障

### 9.1 实时因子：策略能不能正常跑的前提

`rl_deploy` 的策略是**按墙钟 50 Hz** 跑的（`state_machine_base.h` 用 1 kHz timerfd，
每 4 tick 一个策略周期），设计上要求"**仿真 ≈ 实时**"。仿真落后墙钟时，同一个动作序列
在仿真时间里被压缩 ⇒ 训练分布外 ⇒ 底盘"乱动/发抖"然后翻倒。

仿真节点每秒打印一条：

```
[SIM-RT] 仿真 27.00s / 墙钟 37.70s -> 实时因子 0.72  <== 仿真落后墙钟，策略速率会被压缩（进 RL 易乱动）
```

| 因子 | 处理 |
|---|---|
| ≈1.00 | 正常 |
| 0.9~1.0 | 还能看；偶发入口瞬态属已知 DEF-018 |
| 持续 <0.95 | **别在窗口里评估策略**。逐条查：① 宿主机是否在用电池（`cat /sys/class/power_supply/BAT0/status`；电池降频会掉到 0.7 以下）；② 有没有别的程序抢 CPU；③ 加大渲染间隔 `M20_SIM_RENDER_INTERVAL=200`；④ 直接 `M20_USE_VIEWER=0` 无头跑 |

### 9.2 残留 / 孤儿进程（务必看）

```bash
# 有没有残留（空 = 干净）
docker exec m20_piper_ros bash -lc 'ps -eo pid,args | grep -E "rl_deploy|mujoco_simulation|arm_controller|arm_teleop|run_sim2sim" | grep -v grep'

# 全清（按精确路径杀；^ 锚点很关键）
docker exec m20_piper_ros bash -lc "pkill -9 -f '^/root/m20_piper_ws/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy'; pkill -9 -f '^python3 src/M20_sdk_deploy/interface/robot/simulation/'"
```

> ⚠️ 用 `pkill -f <关键字>` 时，如果关键字也出现在**你自己这条命令**里，会把执行这条
> 命令的 shell 一起杀掉（表现为命令瞬间退出、退出码 137）。上面用 `^` 锚定命令行开头
> 就是为了避免这个坑。

### 9.3 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 窗口报 `Authorization required` / `Failed to open display` | 宿主机 X11 没放行：`DISPLAY=:1 xhost +local:` |
| 按 `c` 后偶发抖动/摔倒 | 已知"进 RL 入口瞬态"（`DEF-018`，策略侧边缘稳定性），用 `--repeat` 看失败率；**先看 §9.1 的实时因子** |
| 没按键盘机器人就自己动 / 乱动 | 十有八九是**残留的 `rl_deploy` 孤儿实例**在抢 `/JOINTS_CMD`，见 §9.2 |
| MuJoCo 起不来 / GL 错 | `M20_USE_VIEWER=0` 无头跑；或按 §0.1 放行 X11 |
| `rl_deploy` 报 `Policy dim mismatch` | 策略目录与 runner 布局不符（例如拿了旧的 86/770/23 模型）；用 `check_policy_interface.py` 查 |
| `arm_controller`（IK 那个终端）**自己没了** | 两个已知来源：① `run_sim2sim.sh` 收工时的清理（2026-10-10 已改成只收它自己启动的进程；若你显式设了 `M20_SIM2SIM_CLEANUP_ALL=1` 仍会连带杀掉手起的节点）；② 回调里抛异常（已加三层兜底，不会再死）。真出现时把终端里那条 traceback 贴出来 |
| 想复位机器人 | 见下方命令（会清腿/臂命令缓冲） |
| `--mode push` 不触发接管（机器人扛住了） | 说明策略更抗推了，不是 bug：默认已调到 **1500 N**；仍扛得住就再加大 `M20_SIM_PUSH_FORCE`（实测 2500 N 必倒），或显式给 `_AT` / `_DURATION` |

**复位仿真**（清掉腿/臂的命令缓冲，让机器人回到初始位姿）：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 service call /reset_sim std_srvs/srv/Empty'
```

**相关文档**：接口契约 [`docs/sim2sim_layout_contract_zh.md`](../../docs/sim2sim_layout_contract_zh.md)、
缺陷与实测 [`docs/review/DEFECT_LOG_zh.md`](../../docs/review/DEFECT_LOG_zh.md) /
[`docs/review/DONE_zh.md`](../../docs/review/DONE_zh.md)、开关登记
[`docs/review/WORKFLOW_zh.md`](../../docs/review/WORKFLOW_zh.md) §6.1。
