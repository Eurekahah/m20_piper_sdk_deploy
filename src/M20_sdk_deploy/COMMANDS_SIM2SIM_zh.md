# sim2sim 运行命令速查（宿主机一行式）

**这份文档只管"在仿真里跑，在宿主机粘哪条命令"。** 真机请用
[`COMMANDS_SIM2REAL_zh.md`](COMMANDS_SIM2REAL_zh.md)。

> **与旧版的区别**：主命令全是**宿主机一行式**（前面带 `docker exec -it`），
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

### 0.2 改过 C++ 之后重编

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && colcon build --packages-up-to m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON'
```

### 0.3 统一键盘控制终端

`rl_deploy` 和 `arm_teleop_node.py` 不能同时直接读同一个终端。统一监督器只读一次
stdin，再按业务分流：

| 按键 | 转发目标 |
|---|---|
| `W/S/A/D/Q/E`、`Z/C/R/X`、`H/J/B/N/[/]` | `rl_deploy`（腿、状态机、机身姿态） |
| 小键盘 `0~9 . + -` | `arm_teleop_node.py`（机械臂末端） |
| `G/L` | 两边都发（夹爪切换 / 整体复位） |

查看帮助：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --help'
```

默认 `--profile sim` 会启动 `arm_controller`、`arm_teleop_node` 和 `rl_deploy`。
日志分别带 `[ik]`、`[arm]`、`[rl]` 前缀；任一子节点异常退出时，其余节点会一起停止。
`Ctrl+C` 或 `ESC` 统一退出。

---

## 1. 场景 A：全仿真（腿 + 臂都来自 MuJoCo）—— 最常用

### 1.0 一条命令看完整流程：**开始 → 站立 → RL**（开窗口，最省事）

`tests/sim2sim_smoke.py` 自己就会起 MuJoCo（带窗口）+ `rl_deploy`，并自动按键
（`z` 站立 → 过 4 s 按 `c` 进 RL），所以不用开两个终端：

```bash
# 宿主机（只做一次）：放行容器连 X11
DISPLAY=:1 xhost +local:

# 宿主机一条命令（窗口出现在 :1 上；时长 150 s，到期自动收工）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && M20_SIM_INIT_POSE=M20 M20_IDLE_ZERO_GAIN=1 M20_STANDUP_VENDOR=1 exec python3 tests/sim2sim_smoke.py --mode rl --viewer --duration 150 --out /root/m20_piper_ws/tmp/temp_probe/visual'
```

**你会看到的时序**（= 厂商流程）：

1. 打开就在**原版初始坐姿**上趴着（高度 ≈0.10 m、`hipx ∓0.438 / knee ±2.76`）、腿不使力（idle 零增益）；
2. `z` → 站立状态机**厂商两段式**：第一段 1.5 s 折腿，第二段 1.5 s 按高度样条抬升；
   这是修改前已验证速度快、观感连续的行为；
3. `c` → 进入 RL，策略从厂商站姿收敛到训练默认姿态；稳定后站高 ≈0.51 m、
   `fl_hipx τ ≈ +15 N·m`（机器人保持水平，pitch ≈0°）。

三个环境变量就是要复现"完全和原版一致"的那三处（见
[`docs/DEFECTS_zh.md`](../../docs/DEFECTS_zh.md) DEF-060）：

| 变量 | 这里设成 | 含义 |
|---|---|---|
| `M20_SIM_INIT_POSE` | `M20` | 初始关节角 = 厂商原版折叠坐姿（`training` = 训练站姿） |
| `M20_IDLE_ZERO_GAIN` | `1` | idle 不使力（机器人就地塌着，按 `z` 再起） |
| `M20_STANDUP_VENDOR` | `1` | 站立 = 厂商两段式（想从**训练站姿**进 RL 就设 `0`） |

> 想换成"训练姿态档进 RL"（= 策略的入口姿态，DEF-018 入口瞬态更稳）：
> `M20_STANDUP_VENDOR=0 M20_SIM_INIT_POSE=training`。
> 收工：`pkill -INT -f rl_deploy`（窗口会一起退），或等 `--duration` 到期。
> 实时因子看仿真终端里的 `[SIM-RT]`：开着窗口也应是 **≈1.00**（真慢了就把
> `M20_SIM_RENDER_INTERVAL` 调大，例如 200）。


| 终端 | 说明 |
|---|---|
| A1 | MuJoCo（同时提供腿和臂的数据，带窗口） |
| A2 | 统一控制：策略 + IK + 臂键盘 |

**A1：**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh full'
```

**A2：**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim'
```

> 监督器直接启动编译产物，不经过 `ros2 run` 包装进程。按 `z` 站立 → `c` 进 RL；
> 小键盘直接控制机械臂，不需要切换终端。

**检查点**（另开一个终端）：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 topic hz /JOINTS_DATA'                 # ~200 Hz

docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 topic hz /ARM_JOINTS_DATA'             # ~200 Hz

docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 topic echo --once /ARM_TELEOP_STATE'   # 默认位姿 ≈ (0.3492,0,0.4327,-0.7373,0,-0.6756,0)（四元数=训练/链式 FK 的符号分支，DEF-039）
```

**想动臂**：在 A2 直接按小键盘。`G/L` 会同时作用于腿侧状态和臂侧目标。

---

## 2. 场景 B：只调腿（臂不参与，假臂保持默认姿态）

和真机"没装臂"完全同构：MuJoCo **不发**臂反馈，由假臂节点提供默认姿态。

```bash
# A1
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh legs'

# A2
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile legs'
```

**检查点**：策略第一拍观测里的 8 个臂项应为 0（= 默认姿态）、`ee_goal` = 默认位姿。
自动化：`--mode no_arm`（见 §6）。

---

## 3. 场景 C：只调机械臂（**不起 `rl_deploy`**）

臂链路和策略完全解耦，可以先单独调；臂是开环 IK，**不受实时因子影响**，
开着窗口看最舒服。

```bash
# A1：只启动仿真
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh arm'

# A2：IK + 臂键盘（不起 rl_deploy）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile arm'
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

# A2：策略 + IK + 臂键盘（同一个 stdin，不再切终端）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim'
```

想用真臂替代假 CAN 驱动：A1 前面加 `M20_ARM_TRANSPORT=real`（需要先起
`agx_arm_ctrl`，见真机文档）。

**检查点**：按住 numpad 8 后策略观测里的臂项应跟着变（`--mode arm_mixed` 实测 1.03 rad）。

---

## 5. 场景 E：VR 遥操作（腿/机身 + 臂）

开三个终端。VR 节点保持独立；策略、IK 和臂键盘合并到同一个控制终端。

**A1：MuJoCo（带窗口）**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash src/M20_sdk_deploy/scripts/run_sim2sim.sh full'
```

> 想和真机拓扑一致（腿来自仿真、臂走"独立传输"）就把结尾的 `full` 换成 `mixed`。

**A2：统一控制终端（键盘 `z` 站立 → `c` 进 RL；VR 接管速度/机身，同时保留臂键盘节点）**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim'
```

**A3：VR 节点（会打印 `https://<ip>:8443`，用头显打开这个地址）**

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/interface/robot/simulation/vr_teleop_node.py'
```

**首次使用要先装 VR 依赖**（下面这条只需跑一次；装完再起 A3）：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec pip3 install -r src/M20_sdk_deploy/requirements_teleop.txt'
```

> **头显页面还依赖一个离线文件 `web-ui/aframe.min.js`**（DEF-058）：它原本是从公网
> CDN 现拉的，本机/真机都没有外网 ⇒ 拉不到时头显里只会显示中间一张 `VR Teleoperation`
> 卡片、**点哪都没反应**（页面顶部会有红色报错条）。有外网的机器上跑一次即可：
>
> ```bash
> bash src/M20_sdk_deploy/scripts/fetch_aframe.sh          # 下载到 third_party/XLeVR/web-ui/
> bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --check  # 自检
> ```
>
> 起 A3 时终端还会打印 `头显页面: 已自带离线 A-Frame（xxxx KB）`（缺了会打 ⚠️ + 修法）。

操作：头显里按**右 B** 启动 / 重标定（把 VR 坐标系锚到**当前左右手柄位姿**），
**左 X/Y** 复位。

> **复位的语义（2026-10-13 起，DEF-046）**——按 X/Y 后四件事一起发生：
> ① 机身姿态回默认（`0.513 / 0 / 0`，与键盘 `l` 同一个口径）；
> ② 机械臂末端位姿回默认（`arm_teleop` 发一帧 flag=2 的"重置到默认末端位姿"，
>    同时调 `/reset_sim` 把腿和臂都复位）；
> ③ 速度归 0；
> ④ **VR 交还控制权**（设备侧 `_started=False` ⇒ `/VR_TELEOP` 持续发全 0 + `active=0`，
>    机身/速度字段由 `rl_deploy` 的键盘接口按默认值续写）。
>
> 要接着用就**再按一次右 B**：它会把那一刻的左右手柄位姿重新采成校准原点（干净的新零点），
> 不需要再手工摆姿势。复位时 `rl_deploy` 会打印 `[VR] RESET -> 机身姿态回默认 ...`，
> `arm_teleop` 会打印 `[reset] /reset_sim 完成；EE 目标已重置到默认位姿`。
> 想回到 2026-10-10 的"复位后保持激活、不用再按 B"：`M20_VR_RESET_KEEPS_ACTIVE=1`。

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
| **速度上限**（削峰，手感变"钝"） | `M20_VR_MAX_LIN_SPEED` / `M20_VR_MAX_ANG_SPEED` | `0.4` m/s / `1.0` rad/s | **A2（启动 `arm_controller` 前）** |
| **灵敏度**（整体缩放，更细腻） | `M20_VR_ARM_POS_SENS` / `M20_VR_ARM_ROT_SENS` | `0.5` / `0.5` | **A3 `vr_teleop_node`** |
| 底盘/机身灵敏度（可选） | `M20_VR_VX_SENS` / `_VY_SENS` / `_WZ_SENS`、`_HEIGHT_SENS` / `_PITCH_SENS` / `_ROLL_SENS` | `2.0` / `1.0` / `1.0`、`0.5` / `0.35` / `0.25` | A3 |
| 夹爪"像真夹爪"（默认直接阶跃） | `M20_GRIPPER_SLEW` [stroke/s] | `0`（建议 `0.15~0.2`） | A2 |
| 关节指令限速（= 真机驱动器口径） | `M20_ARM_MAX_JOINT_VEL` [rad/s] | `3.0`（`0` = 不限，不建议） | A2 |
| VR 臂映射（见 §5.2；一般不要动） | `M20_VR_ARM_MAP` | `cylindrical`（`cartesian` = 旧行为） | A2 |
| 臂摆动角度上限 | `M20_VR_SWING_MAX_DEG` | `60` | A2 |
| IK 拒绝阈值（解不出来就不下发） | `M20_ARM_IK_MAX_POS_ERR` / `_ANG_ERR` | `0.010` m / `3.0` ° | A2 |
| IK 姿态正则（压手腕漂移） | `M20_ARM_IK_POSTURE_GAIN` | `0.3`（`0` = 关） | A2 |

> 上面这组默认值是 **2026-10-10 操作员实测确认手感合适** 后固化进代码的（原来是
> 2.0 m/s / 5.0 rad/s、灵敏度 1.0/1.0、油门 5.0）。要回到训练侧原值：
> `M20_VR_MAX_LIN_SPEED=2.0 M20_VR_MAX_ANG_SPEED=5.0 M20_VR_VX_SENS=5 M20_VR_ARM_POS_SENS=1 M20_VR_ARM_ROT_SENS=1`。

**想让仿真的臂/夹爪更像真机**（可选，默认都不限速）：

```bash
# A2 统一控制：夹爪按真夹爪的行程时间走（关节限速默认就是真机的 3 rad/s）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 M20_GRIPPER_SLEW=0.2 && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim'
```

起 A3 时 VR 节点会打印一行 `灵敏度（可用 M20_VR_*_SENS 覆盖）: ...`，可以核对是否生效。

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

### 5.3 开窗口看"几个末端位姿指令"（`scripts/arm_pose_demo.py`）

想把臂单独看清、又不想戴头显时用这个（2026-10-13 第一次这么验收的流程）：

```bash
# A1 仿真（开窗口；实时因子要 ~1.00）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 M20_USE_VIEWER=1 && exec python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py'

# A2 腿策略 + 臂 IK。这里必须关掉臂键盘，避免 /ARM_TELEOP 出现第二个发布者。
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 M20_ARM_DEBUG=1 && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim --no-arm-teleop'

# A3 依次下发 12 个具名位姿指令（默认一直循环；每步 2.5 s）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec python3 src/M20_sdk_deploy/scripts/arm_pose_demo.py --hold 2.5'
```

在 A2 按 `z` 站立、`c` 进 RL。

指令序列（相对按下 A3 那一刻的实际位姿）：中位 → 前伸 8 cm → 后收 4 cm →
**左移 8 cm（基座摆动）** → 右移 8 cm → 抬高 6 cm → 降低 6 cm →
手腕自转/俯仰/侧倾各 +25° → 组合（前伸+左移+自转）→ 回中位。
A3 每步结束会打印 `目标 / 实际 / 位置误差 / 姿态误差 / max|dq| / max|tau|`；
2026-10-13 的实测是 **~5 mm / ~0.8°**（IK 残差 0.1~1.6 mm）、关节 ≤0.06 rad/s、
力矩 ≤4.6 N·m。

> ⚠️ **三个坑（都踩过）**：
> 1. **A2 必须真的进 RL**：只靠仿真自己的腿部默认保持，机器人会**缓慢摔倒**
>    （窗口里表现为高度慢慢掉、最后趴下）。进 RL 后窗口里应看到它站住（高度 ~0.51 m）。
> 2. **A2 已用 `--no-arm-teleop`**。不要再同时起 `arm_teleop_node` / `arm_stub_node` /
>    `arm_real_adapter`：会和
>    `arm_controller` 在 `/ARM_TELEOP`（或 `/ARM_JOINTS_DATA`、`/ARM_TELEOP_STATE`）
>    上形成**两个发布者**，两路数据交替到达 —— 表现是"目标读数不变、臂不听使唤"，
>    见 [`DEFECTS_zh.md`](../../docs/DEFECTS_zh.md) DEF-045
>    （本次就是一个 11 小时前残留的 `arm_stub_node` 干的）。
>    起之前可以先 `python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py` 看发布者数量。
> 3. 想看"腿也在走"的组合：A2 里按住 `w`（每 0.2 s 重复一次，键盘是"按住"语义）。

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
| `--mode rl` / `walk` / `arm_move` | 进 RL / 行走 / 动臂 | 见 [`docs/STATUS_zh.md`](../../docs/STATUS_zh.md)（按失败率判，含入口瞬态） |
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

# 4) 试跑（把 §1 的 A2 换成下面这条；M20_POLICY_DIR 会传给监督器里的 rl_deploy）
# ⚠️ M20_POLICY_DIR 是**按进程 cwd 解析**的（代码默认目录却是按 __FILE__ 解析的，
#    两者口径不同）—— 所以这里必须写绝对路径，写 `policy/xxx` 会找不到：
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 M20_POLICY_DIR=/root/m20_piper_ws/src/M20_sdk_deploy/policy/<目录> && exec python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim'

# 5) 确认无误后改 rl_control_state.hpp 里的默认目录，再跑 tests/run_all.sh
```

---

## 8. 键盘速查

### 8.1 腿 / 机身（在统一控制终端里按）

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

### 8.2 臂（仍在同一个统一控制终端里按，NumLock 打开）

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

* A2 是唯一键盘焦点，不需要在腿和臂之间切换终端；监督器按上表自动分流。
* `--profile sim` 会先启动 `arm_controller`，再延迟 1 s 启动 `rl_deploy`；
  臂链路未就绪时不要直接按小键盘。

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
docker exec -it m20_piper_ros bash -lc 'ps -eo pid,args | grep -E "rl_deploy|mujoco_simulation|arm_controller|arm_teleop|run_sim2sim" | grep -v grep'

# 全清（按精确路径杀；^ 锚点很关键）
docker exec -it m20_piper_ros bash -lc "pkill -9 -f '^/root/m20_piper_ws/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy'; pkill -9 -f '^python3 src/M20_sdk_deploy/interface/robot/simulation/'"
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

**相关文档**：接口契约 [`docs/CONTRACT_zh.md`](../../docs/CONTRACT_zh.md)、
当前状态 [`docs/STATUS_zh.md`](../../docs/STATUS_zh.md)、开发约定
[`docs/DEVELOPMENT_zh.md`](../../docs/DEVELOPMENT_zh.md)、
历史缺陷 [`docs/DEFECTS_zh.md`](../../docs/DEFECTS_zh.md)。
