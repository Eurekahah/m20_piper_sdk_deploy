# 真机 + VR 完整部署流程（M20 + Piper）

**这份文档管**：从开发机把代码送到真机，到**用 VR 手柄操作系统（腿 + 臂）**跑起来的完整步骤，
每一步都带"怎么验证"。
**不管**：仿真怎么跑（见 [`../src/M20_sdk_deploy/COMMANDS_SIM2SIM_zh.md`](../src/M20_sdk_deploy/COMMANDS_SIM2SIM_zh.md)）、
接口契约（见 [`sim2sim_layout_contract_zh.md`](sim2sim_layout_contract_zh.md)）、
逐条打勾版的上线检查单（见 [`sim2real_checklist_zh.md`](sim2real_checklist_zh.md)，
本文是它的"VR 版执行流程"）。

---

## 0. 拓扑与前提

| 角色 | 硬件 / 系统 | 跑什么 |
|---|---|---|
| **上位机**（M20 电脑 `10.21.31.103`，第二 WiFi 下 `10.21.41.1`） | ROS 2 **Foxy**，仓库 `~/M20_sdk_deploy` | `rl_deploy`（策略+状态机）、`arm_controller`（IK）、`arm_teleop_node`、`vr_teleop_node`、`arm_real_adapter`、`arm_stub_node` |
| **Jetson**（`10.21.31.201`） | ROS 2 + `agx_arm_ros`（容器必须 `--network host`） | `agx_arm_ctrl`（CAN ↔ Piper 臂/夹爪） |
| **VR 头显** | Quest 类浏览器 | 打开 `https://<上位机IP>:8443` 做位姿/按键采集 |
| 开发机（本仓库） | 容器 `m20_piper_ros`（Humble） | 编译 **arm 版**、`scp` 送文件 |

**网络（最容易卡住的一步）**：头显、上位机、Jetson 必须在同一个 WiFi / DDS 域：

* 机器人自带 WiFi：`CA9B********` / 口令 `12345678`（错了找技术支持）；
* 上位机在该网段的 IP：`10.21.31.103`，或第二 WiFi 下的 `10.21.41.1`
  —— 头显里打开的地址要用"上位机**在当前网段**的那个 IP"；
* 登录：`ssh user@10.21.31.103`，密码是**一个单引号** `'`。

**前提条件（先确认，不满足就先解决）**：

1. `[ ]` 机器人 OTA 已升级到支持 SDK 模式的固件（README 要求 ≥ 1.1.8），并拿到**授权码**；
2. `[ ]` `drdds` 在机器人 `/opt/ros/foxy` 下**已经装好**（**别**在机器人上重编 drdds）；
3. `[ ]` **臂侧驱动器把 joint1~6 的速度上限设成 3 rad/s**（joint6 本来就是 3）。
   这是训练侧 DEF-065 的约束，**别按 5 放开**；它属于 `TODO_zh.md` 的 **P2-6（未做）**，
   上真机前请先在 agx 侧确认；
4. `[ ]` 上位机 Python 依赖（**离线**，见 §1.5）：
   `python3 -c "import numpy, yaml, websockets; print('deps ok')"`
   —— **scipy 已经不需要了**（2026-10-10 起用纯 numpy 实现替代，见 §1.5）。

---

## 1. 准备：编译 arm 版 + 送到真机（在**开发机**上做）

```bash
# 1.1 编译 aarch64 / 真机版（必须显式 -DSIM2SIM=OFF：CMake 会缓存上一次的 SIM2SIM，见 DEF-025）
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF'

# 1.2 确认编出来的确实是"真机版"（应输出 0；非 0 = 还是 SIM2SIM 构建）
grep -c SIM2SIM build/m20_sdk_deploy/CMakeFiles/rl_deploy.dir/flags.make

# 1.3 传文件 —— 用同步脚本（rsync 增量 + 默认排除仿真 mesh/x86 库；密码是一个单引号 '）
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh                 # 日常就用这条
#   ROBOT=user@10.21.41.1 bash src/M20_sdk_deploy/scripts/sync_to_robot.sh   # 第二 WiFi 换 IP
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --dry-run     # 先看"会传什么"
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --with-meshes # 要在真机上跑 sim2sim 时才加
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --build       # 同步完顺手 colcon build
```

> **为什么不用 `scp -r`**：整包 **215 MB / 4539 个文件**，其中仿真专用 mesh 占
> **47 MB / 2202 个 STL**、开发机专用的 onnxruntime x86 库 62 MB、gamepad 自带的 fmt
> 示例 3.9 MB —— 这些真机**运行与编译都不需要**。scp 每次都会重新传一遍（慢在"文件个数"）；
> 脚本默认用 **rsync（只传改动过的文件）** 并排除上述目录 ⇒ 首次约 **104 MB / 2038 个文件**，
> **之后每次通常只有几个 .py/.hpp（几 KB）**。
>
> 旧写法（脚本不可用时照旧）：`ssh user@10.21.31.103 'mkdir -p ~/M20_sdk_deploy/src'` +
> `scp -r src/M20_sdk_deploy user@10.21.31.103:~/M20_sdk_deploy/src`

**在真机上编译**（机器人上直接编更省事）：

```bash
ssh user@10.21.31.103
cd ~/M20_sdk_deploy
source /opt/ros/foxy/setup.bash
colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF
```

### 1.5 上位机的 Python 依赖（**真机不能联网，这一步必须离线做**）

先看缺什么：

```bash
python3 -V                                            # Foxy/Ubuntu 20.04 通常是 3.8
python3 -c "import numpy, yaml, websockets; print('deps ok')"
```

* **`scipy` 不再需要**：2026-10-10 起 VR 侧的旋转换算改用纯 numpy 实现
  （`third_party/XLeVR/xlevr/_rotation_compat.py`，有 scipy 时仍优先用 scipy，
  行为与之前逐项数值一致）⇒ 离线机器人不用再装 scipy；
* **`yaml`（PyYAML）和 `websockets` 是必需的**（`xlevr/config.py` 与
  `xlevr/inputs/vr_ws_server.py` 各用一个）。

**离线安装**（在有网机器上下载 → scp → 用 `pip3 --no-index` 装）：

```bash
# 【有网的机器】按"真机的 Python 版本 / 架构"下 wheel（真机是 aarch64 + py3.8）
python3 -m pip download PyYAML websockets \
  --only-binary=:all: --platform manylinux2014_aarch64 \
  --python-version 38 --implementation cp --abi cp38 -d wheels

# 【scp 到真机】
scp -r wheels user@10.21.31.103:~/

# 【真机】离线安装（--no-index 表示不上网找包）
python3 -m pip install --no-index --find-links=~/wheels PyYAML websockets
```

> **本次已经替你把包装好了**，在仓库里 `tmp/wheels/`（`tmp/` 已被 `.gitignore` 忽略，
> 不会进 git、存在仓库目录里所以**重启不会丢**；之前放在 `/tmp` 的那份随重启没了 —— 对，就是这次的情况）：
>
> | 文件 | 说明 |
> |---|---|
> | `PyYAML-6.0.3-cp38-…-aarch64.whl` | 必需（`xlevr/config.py`） |
> | `websockets-13.1-cp38-…-aarch64.whl` | 必需（`xlevr/inputs/vr_ws_server.py`） |
> | `pylibs_yaml_websockets_cp38.tar.gz` | **兜底**：两个包的纯 Python 源码，真机没 pip 时直接解压挂 `PYTHONPATH` |
>
> **numpy 不用传**：真机自带 `numpy 1.17.4`（Ubuntu 20.04 的 apt 版），VR/IK 只用到基础接口，够用。
>
> ```bash
> # 先确认真机的 Python 版本（Foxy/Ubuntu20.04 应为 3.8；不是就按下面的命令重下）
> python3 -V
>
> # 传（在开发机仓库根目录）
> scp tmp/wheels/*.whl tmp/wheels/*.tar.gz user@10.21.31.103:~/
>
> # 真机上：方式 A —— 有 pip
> python3 -m pip install --no-index --no-deps ~/PyYAML-*.whl ~/websockets-*.whl
> python3 -c "import numpy, yaml, websockets; print('deps ok')"
>
> # 真机上：方式 B —— 没有 pip / wheel 装不上（纯 Python 兜底，最省事）
> mkdir -p ~/pylibs && tar xzf ~/pylibs_yaml_websockets_cp38.tar.gz -C ~/pylibs
> export PYTHONPATH=~/pylibs:$PYTHONPATH        # 起 vr_teleop_node 的终端都带上
> python3 -c "import numpy, yaml, websockets; print('deps ok (vendored)')"
> ```
>
> 这两条路我都实测过：方式 B 在一个"屏蔽了 scipy 且只有这个目录"的环境里能正常起
> `vr_teleop_node.py`（8443/8442 都监听上）。

**万一真机上没有 pip**（也能离线装）：wheel 就是 zip，直接解压到自己的目录再挂 PYTHONPATH：

```bash
mkdir -p ~/pylibs && cd ~/pylibs
python3 -m zipfile -e ~/PyYAML-*.whl .
python3 -m zipfile -e ~/websockets-*.whl .
export PYTHONPATH=~/pylibs:$PYTHONPATH      # 起 vr_teleop_node 的终端里都要带上
python3 -c "import yaml, websockets; print('deps ok (vendored)')"
```

> ⚠️ 若 `python3 -V` 不是 3.8，就按实测版本重下（把 `--python-version 38` 改成 3x，
> 并相应改 `--abi cp3x`）：
>
> ```bash
> python3 -m pip download PyYAML websockets --only-binary=:all: \
>   --platform manylinux2014_aarch64 --python-version 38 --implementation cp --abi cp38 -d tmp/wheels
> ```
>
> 纯 Python 包的另一种偷懒做法：在**开发机容器**里找到 `yaml/`、`websockets/` 目录
> （`python3 -c "import yaml,websockets;print(yaml.__file__, websockets.__file__)"`）
> 直接拷到真机再挂 `PYTHONPATH` —— 注意**版本要支持真机的 Python**
> （例如 websockets ≥14 要求 Python ≥3.9，别拷过来）。

---

## 2. 上电与前置自检（在**真机**上做）

```bash
# 2.1 进 SDK 模式（用机器人自带手柄 + 授权码）
sudo su
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh
echo "domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"   # 记下这两个值，后面所有终端保持一致

# 2.2 必须能看到机器人的 ROS 图
ros2 topic list | grep -E "JOINTS|IMU|BATTERY"
ros2 topic hz /JOINTS_DATA        # ≈200 Hz；没有数据就别往下走
```

**一键前置检查**（本仓库自带，会读编译选项 / 策略 / 话题）：

```bash
cd ~/M20_sdk_deploy
source install/setup.bash
python3 src/M20_sdk_deploy/scripts/preflight_real_deploy.py
```

它必须报：**不是 SIM2SIM 构建**、策略目录完整、`/JOINTS_DATA` 有发布者。

**腿部标定核对（最关键的一条）**：把机器人摆成训练默认姿态
（hipy ∓0.6、knee ±1.0、轮子 0），读 `/JOINTS_DATA`：

```bash
ros2 topic echo --once /JOINTS_DATA
```

16 个 `position` 应接近 `(0, -0.6, 1.0, 0)`（后腿 `+0.6, -1.0`）。偏差 > 0.05 rad
说明 `{dir, offset}` 表与这台机器人不匹配 ⇒ **先修表再继续**。

---

## 3. 阶段 A：先只上腿（臂没装 / 先不接）

策略观测里**本来就有臂**，所以"只部署 M20"要起**假臂**节点（`DEF-023`）。

| 终端 | 命令 | 说明 |
|---|---|---|
| T1 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py` | 假臂：200 Hz 发 `/ARM_JOINTS_DATA`（默认姿态）+ 50 Hz 发 `/ARM_TELEOP_STATE` |
| T2 | `ros2 run m20_sdk_deploy rl_deploy` | `z` 站立 → `c` 进 RL |

**检查点**

```bash
ros2 topic echo --once /ARM_JOINTS_DATA    # 8 个 position = 0/0.5/-0.5/0/0/0/0/0
ros2 topic echo --once /ARM_TELEOP_STATE   # (0.3492, 0, 0.4327, -0.7373, 0, -0.6756, 0)
# rl_deploy 日志里【不该】出现 "[ARM] 没有收到 /ARM_JOINTS_DATA"
```

**第一次进 RL 的顺序（务必照做）**：

1. 先把机器人**吊起来或架住**，或至少清空周围、手放在急停上；
2. 更保守的启动：
   `M20_TILT_TAKEOVER=0.5 M20_LEG_FOLD_TAKEOVER=0.5 ros2 run m20_sdk_deploy rl_deploy`；
3. `z` 站立 →（等姿态与仿真一致，height ≈ 0.50 m）→ `c` 进 RL；
4. **进 RL 的头 1~2 s 别松手**（已知入口瞬态 `DEF-018`）；给它一点**小速度**
   （`w` 轻点一下 / vx ≈ 0.1~0.2）帮助脱离静止点；
5. 按 `r`（joint damping）与硬件急停各试一次，确认都能立刻接管；
6. 确认无误后把 `M20_TILT_TAKEOVER` / `M20_LEG_FOLD_TAKEOVER` 调回默认（0.8 / 1.2）。

---

## 4. 阶段 B：把真臂接进来（先用键盘跑通）

| 终端 | 机器 | 命令 |
|---|---|---|
| T3 | Jetson | `ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper fast_mode:=true` |
| T4 | 上位机 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py` |
| T5 | 上位机 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py` |
| T6 | 上位机 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py`（numpad 动臂） |

**跨机 DDS 要通**（T4 与 T3 之间）：两边 `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` 一致、
`ROS_LOCALHOST_ONLY≠1`；在上位机 `ros2 topic list | grep feedback/joint_states` 能看到
Jetson 的话题。看不到就按 [`COMMANDS_SIM2REAL_zh.md`](../src/M20_sdk_deploy/COMMANDS_SIM2REAL_zh.md)
场景 5 钉网卡 + 静态对点。

**检查点**

```bash
ros2 topic echo --once /ARM_JOINTS_DATA     # arm2≈0.5 arm3≈-0.5（默认位姿）
ros2 topic echo --once /ARM_TELEOP_STATE    # ≈ (0.3492, 0, 0.4327, -0.7373, 0, -0.6756, 0)
```

* 按 T6 的 numpad，臂应**按预期方向**小步动，且底盘 tilt < 2°；
* 观察 `/ARM_JOINTS_DATA` 的 `torque` 与两个温度字段：默认姿态保持力矩 1.3~7 N·m、
  遥操作时 ≤ 11.4 N·m（限幅 100 N·m）；
* ⚠️ 若方向反了或关节序号错位 —— **先停下来修映射**，别继续。

---

## 5. 阶段 C：接 VR（本文重点）

### 5.1 起 VR 节点（上位机，T7）

```bash
cd ~/M20_sdk_deploy
source /opt/ros/foxy/setup.bash && source install/setup.bash
python3 src/M20_sdk_deploy/interface/robot/simulation/vr_teleop_node.py
```

启动后会打印：

```
HTTPS server on port 8443
VR WebSocket server on port 8442
VR teleop ready
  Open in your headset browser:  https://<上位机IP>:8443
  灵敏度（可用 M20_VR_*_SENS 覆盖）: chassis vx 2 / vy 1 / wz 1; body h 0.5 / p 0.35 / r 0.25; arm pos 0.5 / rot 0.5
```

### 5.2 头显侧

1. 头显连上**和上位机同一个 WiFi**（见 §0）；
2. 浏览器打开打印出来的 `https://<上位机IP>:8443`：
   * 自签证书 → 浏览器会警告，选择"继续 / 信任"；
   * 打不开：先在上位机
     `curl -k -s -o /dev/null -w "%{http_code}\n" https://127.0.0.1:8443` 看是不是 `200`，
     再确认头显与上位机同网段（`ping <上位机IP>`）；
3. **右手柄**按 **B**：启动 + 标定（VR 终端会打印 `[VR] Button B -> START + calibrate`）。

### 5.3 谁控制什么（最容易搞错的一条）

| 手柄 / 按键 | 控制 |
|---|---|
| **右手柄** | **机械臂**（位置 + 姿态）；**扳机** = 夹爪（>0.5 闭合） |
| **左手柄** | **腿 / 机身**：摇杆给 vx/vy/wz；位置/姿态给机身高度·俯仰·横滚 |
| 右手柄 **B** | 启动 / 重新标定（把当前手柄位姿记成偏移原点） |
| 左手柄 **X / Y** | **复位**：末端目标**回到默认位姿**（并保持 VR 激活，可继续操作） |

> 腿仍然由 `rl_deploy` 的策略控制，VR 只是把**速度/机身命令**喂给它；
> 臂**不经过策略**：`vr_teleop_node → arm_teleop_node → arm_controller → arm_real_adapter → agx`。

### 5.4 第一次动的顺序（安全）

1. T2 里 `z` 站立 → `c` 进 RL（按 §3 的注意事项）；
2. 先**只动左手柄摇杆**：vx 小一点（推 10~20%），确认腿的方向/速度正确；
3. 再用**右手柄**小幅度移动：确认臂跟着手走、前后/左右/上下方向都对；
4. 试一次**左 X/Y 复位**：臂应回到默认位姿并停住，之后**不用再按 B** 就能继续操控；
5. 试**扳机**开合夹爪；
6. 一切正常后再做组合动作（边走边动臂）。

**链路自检（随时可跑，只读）**：

```bash
python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py --seconds 8
```

它会打印 `/VR_TELEOP → /ARM_TELEOP → /ARM_JOINTS_CMD → /ARM_JOINTS_DATA` 的流量、
**每个话题的发布者数量**，并直接给出"卡在哪一环"的结论。

---

## 6. 阶段 D：VR 手感与工作空间

**为什么 VR 的范围比键盘小**：键盘是**增量积分**（可以把目标一直推远），
VR 是**绝对偏移**（目标 = 标定锚点 + 手柄位移 × 灵敏度）⇒ VR 的范围
≈ **你的手能移动多远 × 灵敏度**，"手够不到"的位置就够不到。

| 想改什么 | 环境变量（加在对应终端） | 默认 |
|---|---|---|
| 臂**速度上限**（削峰） | `M20_VR_MAX_LIN_SPEED` / `M20_VR_MAX_ANG_SPEED`（**T5** `arm_controller`） | `0.4 m/s` / `1.0 rad/s` |
| 臂**灵敏度**（范围缩放） | `M20_VR_ARM_POS_SENS` / `M20_VR_ARM_ROT_SENS`（**T7**） | `0.5` / `0.5` |
| 底盘油门 | `M20_VR_VX_SENS` / `_VY_SENS` / `_WZ_SENS`（T7） | `2.0 / 1.0 / 1.0` |
| 底盘速度上限 | `M20_VR_MAX_VX` / `_VY` / `_WZ`（**T2** `rl_deploy`） | `1.0 / 0.5 / 0.7` |
| 夹爪"像真夹爪" | `M20_GRIPPER_SLEW`（T5） | `0`（建议 `0.2`） |
| 关节指令限速 | `M20_ARM_MAX_JOINT_VEL`（T5） | `0`（建议 `3.0`，与驱动器一致） |
| 复位后是否保持 VR 激活 | `M20_VR_RESET_KEEPS_ACTIVE`（T7） | `1`（保持） |

**想把范围做大**：把 `M20_VR_ARM_POS_SENS` 提到 `1.0`（速度由限速兜着，两者解耦）；
或者**拿 B 当"离合器"**：手柄移到头了按一下 B（锚点重设到当前手位姿、臂停在原地），
手回中间继续推 —— 范围可以无限延伸。

---

## 7. 安全与回滚

* **急停优先级最高**：软件 `r`（joint damping）→ 硬件急停按钮；上机前两个都要试一次。
* **顺序**：先腿（吊装 / 架住）→ 再臂（键盘）→ 最后 VR；别第一次就上 VR 组合动作。
* 进 RL 的 1~2 s 是已知弱区（`DEF-018`），手不要离开急停。
* 出现持续倾斜 / `[TAKEOVER!]` → 松手，按 `r`。
* **退出顺序**：`r`（阻尼）→ `x`（趴下）→ 停 `vr_teleop_node`（Ctrl+C）→ 停 `arm_teleop` /
  `arm_controller` → 停 `arm_real_adapter` → 停 `rl_deploy` → 退出 SDK 模式。
* 怀疑参数改坏了：`git checkout <上一个 tag/commit>` 重编；真机改过的参数记进
  `DEFECT_LOG_zh.md`（写"真机实测值"）。

---

## 8. 排障速查

| 现象 | 先查什么 |
|---|---|
| 头显打不开 `https://<ip>:8443` | 头显与上位机是否同网段（`ping`）；上位机 `curl -k https://127.0.0.1:8443`；防火墙；打印的 IP 是不是**当前网段**那个（第二 WiFi 是 `10.21.41.1`） |
| VR 里**腿和臂都不动** | 头显里按 B 了吗（VR 终端应有 `[VR] Button B -> START + calibrate`）；`ros2 topic hz /VR_TELEOP` 有没有 50 Hz |
| **只有臂不动** | 跑 `check_vr_arm_chain.py`：`/ARM_TELEOP` 没数据 ⇒ `arm_teleop_node` 没起/崩了；`/ARM_JOINTS_DATA` 没数据 ⇒ 臂反馈没来 |
| 臂**乱动 / 特别快 / 不跟手** | **先看发布者数量**（脚本会列）：`/VR_TELEOP` 或 `/ARM_TELEOP` 上有 2 个发布者时两路命令会交替到达（`DEF-041`）。常见原因：起了两个 `vr_teleop_node` |
| `arm_controller` 终端自己没了 | 已加三层兜底不会再死（`DEF-042`）；若是被别的终端 Ctrl+C 连带杀了，注意清理脚本的 `pkill` 范围；把 traceback 贴出来 |
| 复位后臂被"拽"一下 | 现在已是确定性的"重置到默认位姿"（`DEF-042`）；正常时会打印 `[reset] /reset_sim 完成；EE 目标已重置到默认位姿`（真机上没有 `/reset_sim`，会打一条 service not available 的警告，但末端目标仍会重置） |
| `Policy dim mismatch` / 找不到策略 | 策略目录必须含 `policy.onnx` + `policy_layout.json`；用 `M20_POLICY_DIR` 时**写绝对路径**（按进程 cwd 解析，`DEF-038`） |
| 策略周期被拉长 / 动作卡顿 | 上位机上 `top` 看谁在抢 CPU；`[JVEL]` 打印应 1 Hz；仿真侧那条 `[SIM-RT]` 自检**真机上没有** |

---

## 9. 还没在真机验证过的（别当成验过）

* `[ ]` **腿部标定实测核对**（§2 那条，必须先做）；
* `[ ]` **臂侧 3 rad/s 速度上限**在 agx / 驱动器里设好（`TODO_zh.md` P2-6）；
* `[ ]` `arm_real_adapter` 的实机联调（只在话题层验证过）；
* `[ ]` 端到端时延 / 抖动（P2-3）；
* `[ ]` **VR 跨机链路**：本文假设 VR 节点与 `rl_deploy` **同在**上位机（最稳）。
  若改成在开发机上跑 `vr_teleop_node`，需要跨机 DDS 且两边 ROS 版本不同
  （上位机 Foxy vs 开发机 Humble），**不推荐**；
* `[ ]` 真机急停链路（软件 `r` + 硬件按钮）与固件侧力矩/速度限幅是否与训练一致。

---

## 10. 命令速查（复制用）

```bash
# —— 开发机 ——
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF'
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh          # 增量同步 + 排除仿真 mesh（≈104 MB/2038 文件，之后只传改动）

# —— 上位机（Foxy，先 sudo su + source setup_ros2.sh）——
cd ~/M20_sdk_deploy && source /opt/ros/foxy/setup.bash && source install/setup.bash
python3 src/M20_sdk_deploy/scripts/preflight_real_deploy.py          # 前置检查

# T1 假臂（臂没装时）
python3 src/M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py
# T2 策略（z → c）
ros2 run m20_sdk_deploy rl_deploy
# T4/T5/T6 臂链路（真臂）
python3 src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py
M20_GRIPPER_SLEW=0.2 M20_ARM_MAX_JOINT_VEL=3.0 python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py
python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py
# T7 VR
M20_VR_ARM_POS_SENS=1.0 python3 src/M20_sdk_deploy/interface/robot/simulation/vr_teleop_node.py
# 自检
python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py --seconds 8
```
