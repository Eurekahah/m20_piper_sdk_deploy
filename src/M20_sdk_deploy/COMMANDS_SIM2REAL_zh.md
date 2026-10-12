# sim2real 运行命令速查（真机）

**这份文档只管"真机上跑，哪个终端敲什么命令"。** 仿真请用
[`COMMANDS_SIM2SIM_zh.md`](COMMANDS_SIM2SIM_zh.md)。

> 每个场景都是"**一个标题 + 一张终端表 + 检查点**"，照抄即可。
> 原理/接口/排障细节见 [`docs/sim2real_checklist_zh.md`](../../docs/sim2real_checklist_zh.md)。
>
> **用 VR 操作真机**（腿 + 臂）请走这条完整流程：
> [`docs/DEPLOY_REAL_VR_zh.md`](../../docs/DEPLOY_REAL_VR_zh.md)。

## 0. 硬件拓扑与通用约定

```
[Jetson AGX Orin  10.21.31.201]            [M20 电脑  10.21.31.103]
  agx_arm_ctrl（CAN → Piper 臂）             rl_deploy（策略）/ arm_controller（IK）
      ↑ /control/joint_states  (sensor_msgs)      ↑ /ARM_JOINTS_DATA (drdds)
      ↓ /feedback/joint_states (sensor_msgs)      ↑ /ARM_TELEOP_STATE (std_msgs)
            └───────── 有线网络 / 同一个 DDS domain ─────────┘
                    arm_real_adapter.py 放在 M20 上
```

| 项 | 值 |
|---|---|
| M20 电脑 | `10.21.31.103`（另一网段 `10.21.41.1`）；仓库在 `~/M20_sdk_deploy` |
| Jetson | `10.21.31.201`；`agx_arm_ros` 在容器里，容器必须 `--network host`（见场景 5） |
| ROS | M20 上 `source /opt/ros/foxy/setup.bash && source /opt/robot/scripts/setup_ros2.sh` |
| `ROS_DOMAIN_ID` | **两边一致**；机器人自带服务用默认值（M20 上就是空 = 0） |
| 臂的默认位姿 | `arm2=+0.5, arm3=-0.5`，其余 0（= 训练 `init_state`）——起策略前先摆到这儿 |
| 急停 | 一切真机动作前确认：软件 `r`（阻尼）+ 硬件急停按钮都可用 |

下面用 **T1/T2/T3…** 表示"第几个终端"。

---

## 场景 0：一次性准备（只做一次 / 换版本时做）

### 0.1 在 M20 上编译真机版（**必须显式 `-DSIM2SIM=OFF`**）

```bash
cd ~/M20_sdk_deploy
source /opt/ros/foxy/setup.bash
colcon build --packages-select m20_sdk_deploy \
  --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF
grep -c SIM2SIM build/m20_sdk_deploy/CMakeFiles/rl_deploy.dir/flags.make   # 应为 0
```

> `SIM2SIM` 是 CMake 缓存变量：先编过仿真版再编真机版，**必须**显式 OFF，
> 否则还是恒等标定的仿真接口（`DEF-025`）。

### 0.2 只更新某个 Python 脚本（不用重编）

```bash
# 开发机 → M20
scp src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py \
    user@10.21.31.103:~/M20_sdk_deploy/src/M20_sdk_deploy/interface/robot/simulation/
```

### 0.3 进 SDK 模式（每次上电后都要做）

```bash
# M20：用手柄进入 SDK 模式（需要授权码）
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic hz /JOINTS_DATA          # 应 ~200 Hz
```

**检查点**：`ros2 topic list` 能看到 `/JOINTS_DATA /IMU_DATA /BATTERY_DATA /JOINTS_CMD`。
若只看到 `/rosout`、`/parameter_events` → `ROS_DOMAIN_ID` 不一致（`unset ROS_DOMAIN_ID` 再试）、
RMW 不一致、或机器人服务没起。

---

## 场景 1：只部署 M20（**没装机械臂**）

策略观测里含臂，所以必须给一份"默认姿态"的假反馈。

| 终端 | 命令 | 说明 |
|---|---|---|
| T1 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py` | 假臂：200 Hz 发 `/ARM_JOINTS_DATA`（默认姿态）+ 50 Hz 发 `/ARM_TELEOP_STATE` |
| T2 | `ros2 run m20_sdk_deploy rl_deploy` | `z` 站立 → `c` 进 RL |

**检查点**

```bash
ros2 topic echo --once /ARM_JOINTS_DATA     # 8 个 position = 0/0.5/-0.5/0/0/0/0/0
ros2 topic echo --once /ARM_TELEOP_STATE    # (0.3492,0,0.4327,-0.7373,0,-0.6756,0)  ← 四元数=训练/链式 FK 符号分支（DEF-039）
# rl_deploy 启动日志里【不该】出现 "[ARM] 没有收到 /ARM_JOINTS_DATA"
```

**安全**：进 RL 后 1~2 s 别放手（`DEF-018` 入口瞬态）；更保守可以先用
`M20_TILT_TAKEOVER=0.5 M20_LEG_FOLD_TAKEOVER=0.5 ros2 run m20_sdk_deploy rl_deploy`。

---

## 场景 2：真机 **只调机械臂**（策略不在场）

臂链路和策略解耦，先单独把它调顺。

| 终端 | 机器 | 命令 |
|---|---|---|
| T1 | Jetson | `ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper fast_mode:=true` |
| T2 | M20 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py` |
| T3 | M20 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py`（IK；不给输入时把臂带到默认位姿并保持） |
| T4 | M20 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py`（键盘；`Ctrl+C` 或 `ESC` 退出） |

**腿怎么办（真机特有）**：

* **不进 SDK 模式** → M20 自己的控制器撑着腿（推荐，最省事）；
* 进了 SDK 模式又不跑 `rl_deploy` → 腿零增益会塌：要么把机器人架起来，要么让
  `rl_deploy` 起来停在 idle（**不要按 `c`**）。

**检查点**

```bash
python3 src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py --check
ros2 topic echo --once /ARM_JOINTS_DATA     # arm2≈0.5 arm3≈-0.5（默认位姿）
ros2 topic echo --once /ARM_TELEOP_STATE    # ≈ (0.349,0,0.433,0.737,0,0.676,0)
# 按 numpad 8/2/4/6/7/9，上面两个话题的数值应跟着变
```

---

## 场景 3：真机"**只感知**"机械臂（控制仍用 agx 自带键盘/示教）

只想让策略知道臂的状态、不换控制方式：**不用起 `arm_controller`**。
`arm_real_adapter` 会用 FK 从当前关节角算出末端位姿发到 `/ARM_TELEOP_STATE`
（= 策略观测里的 `ee_goal`），所以观测与真机自洽。

| 终端 | 机器 | 命令 |
|---|---|---|
| T1 | Jetson | agx 驱动（同场景 2 的 T1） |
| T2 | Jetson | 你原来的 agx 键盘/示教（保持不变） |
| T3 | M20 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py` |
| T4 | M20 | `ros2 run m20_sdk_deploy rl_deploy`（`z` → `c`） |

**检查点**：手动画臂时 `/ARM_JOINTS_DATA` 与 `/ARM_TELEOP_STATE` 都跟着变
（`M20_ADAPTER_PUBLISH_EE=0` 可关掉 FK 那条）。

---

## 场景 4：真机 **用我们的 IK + 键盘控制臂**，同时策略感知（合并）

和 sim2sim 的 `mixed` 配置同构：先在仿真里调好再上真机
（`bash src/M20_sdk_deploy/scripts/run_sim2sim.sh mixed`）。

| 终端 | 机器 | 命令 |
|---|---|---|
| T1 | Jetson | agx 驱动（同场景 2 的 T1） |
| T2 | M20 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py` |
| T3 | M20 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py` |
| T4 | M20 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py` |
| T5 | M20 | `ros2 run m20_sdk_deploy rl_deploy`（`z` → `c`） |

**注意**：进 RL 时 `rl_deploy` **不会**再抢 `/ARM_JOINTS_CMD`（检测到 `arm_controller`
在发就跳过"默认位姿保持"命令，`DEF-026`），日志里会打印那一行。

**安全**：臂大幅摆动时底盘会被扰动（训练里 s3 阶段本身有 9~15% 的 20 s 摔倒率），
先从**小幅度、慢速**开始；出现持续倾斜或 `[TAKEOVER!]` 就停手。

---

## 场景 5：网络/DDS 不通时的两种修法

**症状**：Jetson 上 `ros2 topic echo /feedback/joint_states` 有数据，但 M20 上
`ros2 node list` 看不到 agx 节点、`ros2 topic info` 的 Publisher count = 0。

### 5.1 先确认 Jetson 的容器是 host 网络

```bash
docker inspect -f '{{.HostConfig.NetworkMode}}' <容器名>   # 必须是 host
docker exec -it <容器名> ip -brief addr                    # 应能看到 end0 10.21.31.201
```

bridge 网络下 DDS 出不了容器：用 `--network host` 重建容器。

### 5.2 组播被挡 → 强制单播发现 + 钉死有线网卡

```bash
# 两台机器各跑一次（IP / 网卡名换成自己的）
source src/M20_sdk_deploy/scripts/setup_dds_wired.sh 10.21.31.103 10.21.31.201 <M20有线网卡>
source /tmp/setup_dds_wired.sh 10.21.31.201 10.21.31.103 end0     # Jetson（先把脚本拷过去）
ros2 daemon stop
# 然后重启各自的节点（agx launch / adapter / rl_deploy 都要在设过变量的 shell 里起）
```

### 5.3 兜底：UDP 直连（不依赖 DDS 发现，只要 ping 通）

```bash
# Jetson 容器内
python3 arm_udp_bridge.py --tx --peer 10.21.31.103
# M20（会发 /ARM_JOINTS_DATA 与 FK 的 /ARM_TELEOP_STATE）
python3 src/M20_sdk_deploy/scripts/arm_udp_bridge.py --rx
```

用兜底时**不要**同时跑 `arm_real_adapter`（两个都会发 `/ARM_JOINTS_DATA`）。

---

## 场景 6：排障速查

```bash
# 反馈有没有到（M20）
ros2 topic list | grep -E "ARM|feedback|control"
ros2 topic info -v /feedback/joint_states      # Publisher count / Reliability
ros2 topic hz  /feedback/joint_states          # ~200 Hz

# ros2 图对不对（M20）
ros2 node list                                 # 应看到 agx_arm_ctrl_single_node
ros2 daemon stop                               # daemon 有时缓存旧图

# 两个自带自检
python3 src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py --check
python3 src/M20_sdk_deploy/scripts/preflight_real_deploy.py
```

| 现象 | 处理 |
|---|---|
| `ros2 topic echo` 能收到、脚本收不到 | 更新 `arm_real_adapter.py`（反馈订阅已改 BEST_EFFORT）；比对两个终端的 domain/RMW（adapter 启动会打印自己的环境） |
| 启动时 `no data on /feedback/joint_states yet` | 只是"第一帧还没到"（DDS 发现要 1~2 s）；超过 2 s 才提示，收到会打印"收到第一帧…启动后 X.Xs" |
| `/ARM_JOINTS_DATA` 一直没数据 | adapter 没收到 agx 反馈就不会发；先解决发现（场景 5） |
| 臂不在默认位姿 | 起 `arm_controller`（不给输入时会回到默认位姿），或用 agx 键盘/示教摆到 `0, 0.5, -0.5, 0, 0, 0` |
| `rl_deploy` 卡在 `joint data update is not finished` | 旧版本；现已改为有限等待 + 诊断（`DEF-024`），会告诉你是不是收不到 `/JOINTS_DATA` |
| `arm_teleop_node.py` 按 Ctrl+C 关不掉 | 旧版本（`raw` 模式吃掉了 Ctrl+C）；新版本用 `setcbreak`，`Ctrl+C` 或 `ESC` 都能退（`DEF-028`） |
