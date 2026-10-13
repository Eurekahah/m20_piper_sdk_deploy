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

### 0.1 把代码同步到 M20（**在开发机**上做，别用 `scp -r`）

```bash
# 在**开发机**的仓库根目录执行
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh
```

**常用开关**

```bash
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --dry-run      # 先看"会传什么"，不真传
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --build        # 同步完顺手在 M20 上 colcon build
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --with-meshes  # 要在 M20 上跑 sim2sim/check_mjcf_contract 时才加
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --full         # 不排除任何东西（= 旧的 scp -r）
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --delete       # 让远端与本地完全一致（**会删远端多余文件**，慎用）
ROBOT=user@10.21.41.1 bash src/M20_sdk_deploy/scripts/sync_to_robot.sh   # 第二 WiFi 换 IP
DEST='~/M20_sdk_deploy/src/M20_sdk_deploy' bash src/M20_sdk_deploy/scripts/sync_to_robot.sh  # 换目标目录
```

| 项 | 说明 |
|---|---|
| 为什么不用 `scp -r` | 整包 **215 MB / 4539 个文件**，其中**仿真 mesh 47 MB / 2202 个 STL**、`third_party/onnxruntime/x86` 62 MB、`gamepad/example` 3.9 MB —— 真机**运行与编译都不需要**。scp 每次都重传（慢在"文件个数"的 ssh 开销） |
| 默认排除后 | **104 MB / 2038 个文件**；**第二次起 rsync 只传改动过的那几个 .py/.hpp（通常几 KB）** |
| 前提 | 开发机与 M20 都有 `rsync`；**任一端没有就自动退化成 `tar + ssh`**（带同样的排除，并在远端数文件数自检） |
| 目标位置 | `M20:~/M20_sdk_deploy/src/M20_sdk_deploy`（与旧 scp 流程一致，M20 上 `~/M20_sdk_deploy` 是**仓库根**） |

> 密码是**一个单引号** `'`（首次会问；配好免密就不用输了）：
> `ssh-copy-id user@10.21.31.103`。

> **头显页面要的离线引擎别忘了**（DEF-058）：`third_party/XLeVR/web-ui/aframe.min.js`
> 1247 KB，仓库里**默认不自带**（要联网下，或从别的机器拷）——缺了它头显里只会显示中间一张
> `VR Teleoperation` 卡片、点哪都没反应。同步脚本会打印它是"已就位"还是缺失：
>
> ```bash
> bash src/M20_sdk_deploy/scripts/fetch_aframe.sh          # 在有外网的机器上跑一次
> bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --check  # 自检（存在/大小/不是错误页）
> ```

### 0.2 在 M20 上编译真机版（**必须显式 `-DSIM2SIM=OFF`**）

```bash
cd ~/M20_sdk_deploy
source /opt/ros/foxy/setup.bash
colcon build --packages-select m20_sdk_deploy \
  --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF
grep -c SIM2SIM build/m20_sdk_deploy/CMakeFiles/rl_deploy.dir/flags.make   # 应为 0
```

> `SIM2SIM` 是 CMake 缓存变量：先编过仿真版再编真机版，**必须**显式 OFF，
> 否则还是恒等标定的仿真接口（`DEF-025`）。

### 0.3 M20 上的 Python 依赖（**离线**，只装一次）

> 📥 本节与 §0.1 的备注（还有头显要的 `aframe.min.js`）汇总在
> [`docs/OFFLINE_ASSETS_zh.md`](../../docs/OFFLINE_ASSETS_zh.md)：
> 「要下什么 / 放哪 / 怎么下 / 怎么自检 / 缺了什么症状 / 换机器后重做哪些」，一页看完。

VR 节点要 `yaml` 与 `websockets`（`xlevr/config.py`、`xlevr/inputs/vr_ws_server.py` 各用一个）。
**scipy 已经不需要**（2026-10-10 起用纯 numpy 实现替代）；**numpy 也不用传**
（M20 自带 `numpy 1.17.4`，实测够用）。

```bash
# 0) M20 上先确认版本（应为 Python 3.8 —— cp38 的 wheel 就是按 Foxy/Ubuntu20.04 选的）
python3 -V
python3 -c "import numpy, yaml, websockets; print('deps ok')"    # 报缺哪个就装哪个

# 1) 开发机：传现成的包（放在仓库 tmp/wheels/，tmp/ 已被 gitignore，重启不丢）
scp tmp/wheels/*.whl tmp/wheels/*.tar.gz user@10.21.31.103:~/

# 2a) M20：有 pip（--no-index = 不上网找包）
python3 -m pip install --no-index --no-deps ~/PyYAML-*.whl ~/websockets-*.whl

# 2b) M20：没有 pip / wheel 装不上（纯 Python 兜底，最省事）
mkdir -p ~/pylibs && tar xzf ~/pylibs_yaml_websockets_cp38.tar.gz -C ~/pylibs
export PYTHONPATH=~/pylibs:$PYTHONPATH    # 起 vr_teleop_node 的终端都要带上

python3 -c "import numpy, yaml, websockets; print('deps ok')"     # 两条路都要走到这行
```

> 目录里放的就是这三个：`PyYAML-6.0.3-cp38-…-aarch64.whl`、`websockets-13.1-cp38-…-aarch64.whl`、
> `pylibs_yaml_websockets_cp38.tar.gz`（兜底源码包）。
> 若 M20 的 Python 不是 3.8，用这条按实测版本重下：
>
> ```bash
> python3 -m pip download PyYAML websockets --only-binary=:all: \
>   --platform manylinux2014_aarch64 --python-version 38 --implementation cp --abi cp38 -d tmp/wheels
> ```
>
> 细节（含"没有 pip 时怎么装"的更多说明）见
> [`docs/DEPLOY_REAL_VR_zh.md`](../../docs/DEPLOY_REAL_VR_zh.md) §1.5。

### 0.4 只更新某个 Python 脚本（不用重编）

```bash
# 开发机 → M20（单个文件；也可以用 0.1 的同步脚本，它本来就只传改动的文件）
scp src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py \
    user@10.21.31.103:~/M20_sdk_deploy/src/M20_sdk_deploy/interface/robot/simulation/
```

### 0.5 进 SDK 模式（每次上电后都要做）

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

---

## 场景 7：录关节数据（`torque` + 两个温度）—— 排查"某关节长期顶力矩 / 过温"

用途：核对真机上"某个关节（如左前侧摆 `fl_hipx`）站久了温度冲到 100 ℃ ⇒ 安全接管趴下 ⇒
10 s 后恢复"这类现象（背景与判读口径见 `docs/review/DEFECT_LOG_zh.md` **DEF-059**、
待办 `docs/review/TODO_zh.md` **P1-7 / P1-8**）。

**录什么**：`/JOINTS_DATA` 的 16 个关节 `position / velocity / torque / motion_temp / driver_temp`
（两个温度**都要**：代码判的是 `motion_temp`，`driver_temp` 是驱动器板温，混淆会判错病因），
外加 `/IMU_DATA`（看有没有摔倒/被抬动）。

> 为什么只能在真机录：仿真侧**没有电机热模型**（`mujoco_simulation_ros2.py` 把
> `motion_temp/driver_temp` 写死 40/45 ℃），温度这条路在 sim2sim 里根本走不到。

### 7.1 前置检查（在 M20 上）

```bash
cd ~/M20_sdk_deploy
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh

ros2 topic hz /JOINTS_DATA                                   # 应 ~200 Hz
python3 -c "from drdds.msg import JointsData; print('drdds ok')"   # 脚本要用
```

> 提示：**用与跑 `rl_deploy` 相同的终端环境**（domain/RMW 一致）。
> 若第二条报 `No module named drdds`，说明这个 shell 没 source 到带 drdds 的 overlay
> ⇒ 用 7.4 的 `ros2 bag` 兜底。

### 7.2 录制（M20 上；脚本随仓库同步，见场景 0.1）

```bash
# A）站立状态机：按 z 站起来、**不要**按 c，录 10 分钟
python3 src/M20_sdk_deploy/scripts/record_joint_data.py --duration 600 \
        --out ~/joint_logs/stand_$(date +%m%d_%H%M).csv

# B）RL 站立、零速度命令：另一个终端跑 rl_deploy 按 z 再按 c，这里录 10 分钟
#    （过温 → 接管趴下 → 冷却恢复 会完整落在这一份里）
python3 src/M20_sdk_deploy/scripts/record_joint_data.py --duration 600 \
        --out ~/joint_logs/rl_$(date +%m%d_%H%M).csv

# C）过温趴下之后，继续录 60 s 的**冷却段**（看温度多久掉回阈值以下）
python3 src/M20_sdk_deploy/scripts/record_joint_data.py --duration 60 \
        --out ~/joint_logs/cool_$(date +%m%d_%H%M).csv
```

* `--duration 0` = 一直录到 `Ctrl+C`（`Ctrl+C` 也会先打印汇总再退出）；
* **不会覆盖**：`--out` 指向的文件已存在时，脚本自动改写到 `…_2.csv`、`…_3.csv` 并在终端
  提示（要覆盖得显式加 `--force`）。⚠️ 真机上时钟可能是错的（实测录到 `2024-08-11`），
  所以**别靠时间戳区分**，文件名用有意义的 tag 更可靠；
* 终端每 5 s 打印一行"最热的 3 个关节（motion/driver）"，**边录边看**有没有在升温：
  ```
  [  125.3s]  200.0 Hz  n=25060  最热: fl_hipx 87.4(drv 79.1) fl_hipy 71.2(drv 68.0) ...
  ```
* 退出时打印每个关节的 τ 均值/极值 + 温度首/末/峰，并标出 `motion_temp > 100 ℃`
  的区段（第几秒开始、持续多久）。

**录制期间**：急停别离手；B 档一定要亲眼看到"过温 → 趴下 → 恢复"这一轮（这才是要复现的），
顺便记下操作员看到的**时间点**（对照 CSV 里的 `wall` 列）。

### 7.3 取回 + 分析（开发机上）

```bash
# 下载（M20 的 SSH 密码是一个单引号 '）
mkdir -p tmp/real_logs
scp user@10.21.31.103:'~/joint_logs/*.csv' tmp/real_logs/
scp user@10.21.31.103:'~/joint_logs/*.meta.json' tmp/real_logs/

# 分析（容器里跑；也可以用开发机上带 numpy 的 python）
docker exec m20_piper_ros bash -lc 'cd /root/m20_piper_ws &&
  python3 src/M20_sdk_deploy/scripts/joint_log_report.py tmp/real_logs/rl_xxxx.csv --ref rl --spectrum'
```

报告直接给四刀（对应 DEF-059 的排查清单）：

1. **逐关节 `τ` / `q` 与仿真参考值并列**（`--ref rl` 或 `--ref standup`）⇒ 差得多 = 负载/标定问题；
2. **两个温度分开看 + >100 ℃ 事件时序** ⇒ 是电机绕组还是驱动器板；
3. **谁最热 vs 谁力矩最大**（若不是一个关节 ⇒ 纯铜损解释不了）+ 左右镜像对比
   （`|左|/|右| > 1.5` 会标出来）；
4. `--spectrum`：`τ` 的高频带能量（仿真里 >5 Hz 都 < 0.001 N·m；真机若有 ⇒
   纹波/极限环，仿真测不到）。

### 7.4 兜底：`ros2 bag`（Python 端 import 不到 drdds 时）

```bash
# M20 上录（只依赖 ROS 自带的 ros2bag）
ros2 bag record -o ~/joint_logs/bag_rl /JOINTS_DATA /IMU_DATA
# 开发机上取回，再把 .db3 给需要的人（我们这边可以转成上面的 CSV 格式）
scp -r user@10.21.31.103:'~/joint_logs/bag_rl' tmp/real_logs/
```

---

## 场景 8：站立/idle 用"**厂商原版档**"做 A/B（DEF-060 / TODO P1-9）

**背景**：本仓库把厂商的状态机改过（`e5e1917`，2026-09-04）——
站立从"两段高度规划（0.12 m 蹲姿 → 0.48 m）"改成"一步到训练默认姿态并**永久保持**"，
idle 从"**零增益不使力**"改成"`kp 80/2` 主动保持"。真机实测（DEF-059 §5）在站立状态机里
保持力矩是仿真模型的 **2.0~3.7 倍**、65 s 内 3 次过温 ⇒ 这两个改动是**首要嫌疑**之一，
因此加了两个开关做 A/B（默认关闭 = 本仓库行为不变，且**不碰 `rl_control_state` / `run_policy` / 安全层**）。

### 8.1 三档怎么跑（跑之前先做场景 0.1 同步 + 0.2 编译）

> ⚠️ **开关是代码改动 ⇒ 真机上必须先重新同步 + 重编**（否则开关不起作用）：
>
> ```bash
> # 开发机（仓库根目录）：同步顺手在 M20 上编真机版
> bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --build
> ```
>
> 起来后**先确认开关生效**：`rl_deploy` 启动日志里应出现
> `[STANDUP] M20_STANDUP_VENDOR=1 ⇒ 厂商原版两段式站立…`（档 B）
> 或 `[IDLE] M20_IDLE_ZERO_GAIN=1 ⇒ 厂商原版 idle…`（档 C）。**没有这行就是没编进去。**

```bash
cd ~/M20_sdk_deploy
source /opt/ros/foxy/setup.bash && source /opt/robot/scripts/setup_ros2.sh

# 档 A：本仓库行为（默认，什么都不用设）—— 对照组
ros2 run m20_sdk_deploy rl_deploy            # 按 z 站立，录 10 分钟（场景 7.2）

# 档 B：厂商原版站立（蹲 0.12 m → 升到 0.48 m，轮子 kp=0）
M20_STANDUP_VENDOR=1 ros2 run m20_sdk_deploy rl_deploy

# 档 C：厂商原版 idle（零增益、不使力）
M20_IDLE_ZERO_GAIN=1 ros2 run m20_sdk_deploy rl_deploy
```

**档 B 的完整录法（两个终端，注意顺序：先开录、再起 rl_deploy、最后按 `z`）**

```bash
# T1（M20）：先把录制挂上（文件名带 tag；已存在不会覆盖，会写成 …_2.csv）
python3 src/M20_sdk_deploy/scripts/record_joint_data.py --duration 600 \
        --out ~/joint_logs/standvendor_B1.csv

# T2（M20）：用你平时那条启动命令，**前面加环境变量**（`ros2 run` 或直接跑二进制都行）
M20_STANDUP_VENDOR=1 ros2 run m20_sdk_deploy rl_deploy
#   等它打印 [STANDUP] … 之后，按 z 站立；保持 10 分钟（或直到过温趴下 3~4 次）
```

**要注意的**（都写进了启动日志）：

* 档 B 的**结束站位不是训练默认姿态**（仿真实测站高 0.559 m vs 训练姿态 0.514 m）⇒
  **别带着 `M20_STANDUP_VENDOR=1` 按 `c` 进 RL**（入口瞬态会变差，DEF-018）。
  它的用途是"同一个人、同一个位置、同一份录制脚本，只换站立策略"，比完就撤；
* 档 C 进 SDK 模式后**腿就没人撑了**（厂商原版就是零增益，它假定站着的是别人）⇒
  机器人会塌。想复现厂商整条链路，应该在**不进 SDK 模式**的情况下用机器人自带控制器站
  （这也就是 P1-9 步骤 1 的"厂商模式 A/B"，不需要我们的代码）；
* 仿真里跑 `--mode stand` 时，档 B 会被冒烟脚本判成"摔倒 @ t≈4.6 s"——那是**故意的蹲姿**
  （0.12 m < 0.30 m 的摔倒阈值），不是异常。

### 8.2 仿真侧的对照结果（2026-10-13，`--mode stand`，t≥25 s 稳态）

| | 档 A（训练姿态，默认） | 档 B（厂商两段式） |
|---|---|---|
| 站高（判据口径 root_z−mean(轮 z)+0.09） | 0.514 m | **0.559 m** |
| `fl_hipx` / `fr_hipx` | +11.99 / −11.89 N·m | **+11.78 / −11.68 N·m** |
| `hl_hipx` / `hr_hipx` | +8.02 / −7.94 | **+8.50 / −8.42** |
| `fl_knee` / `fr_knee` | −9.52 / −9.46 | **−7.07 / −7.02** |
| 前/后轮接触力 | 106.2 / 85.9 N | 104.2 / 87.9 N |

> 读法：仿真里两档的**侧摆力矩几乎一样**（差 1.8%），膝盖轻了 ~26%，站位高了 4.5 cm。
> 所以**站立改动解释不了真机那 2.0~3.7 倍**（那是真机负载/几何/摩擦 vs 模型的差异）；
> 但膝盖那 26% 在真机上值不值得，只有真机 A/B 能给答案——这也是这个开关存在的意义。
