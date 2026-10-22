# M20 + Piper 部署与运行

**职责**：从开发机编译、同步到真机，按“只上腿 → 接机械臂 → 接 VR”的顺序完成部署。
本文件是可执行主流程；具体命令速查见
[`COMMANDS_SIM2SIM_zh.md`](../src/M20_sdk_deploy/COMMANDS_SIM2SIM_zh.md) 和
[`COMMANDS_SIM2REAL_zh.md`](../src/M20_sdk_deploy/COMMANDS_SIM2REAL_zh.md)。
接口参数见 [`CONTRACT_zh.md`](CONTRACT_zh.md)，安全风险见 [`STATUS_zh.md`](STATUS_zh.md)。

## 1. 拓扑

| 角色 | 运行内容 |
|---|---|
| 开发机 + 容器 `m20_piper_ros` | x86 sim2sim、aarch64 交叉编译、测试 |
| M20 上位机（ROS 2 Foxy） | `keyboard_teleop_supervisor.py` 统一托管 `rl_deploy`、`arm_controller`、`arm_teleop_node`、`arm_real_adapter`；`vr_teleop_node` 独立 |
| Jetson | `agx_arm_ctrl`，负责 Piper 机械臂 CAN 通信 |
| VR 头显 | 浏览器访问上位机的 `https://<IP>:8443` |

网络要求：

- 头显、M20 上位机和 Jetson 在同一网段。
- 各节点 `ROS_DOMAIN_ID` 与 `RMW_IMPLEMENTATION` 一致。
- `ROS_LOCALHOST_ONLY` 不能为 1。
- 上位机可能同时有 `10.21.31.103` 和机器人 WiFi 侧 `10.21.41.1`，头显必须使用当前网段地址。

## 2. 离线资源

现场机器均不能联网。必须提前在有外网的机器准备以下资源：

| 资源 | 用途 | 准备方式 |
|---|---|---|
| `aframe.min.js` | VR 页面离线渲染 | `bash src/M20_sdk_deploy/scripts/fetch_aframe.sh` |
| PyYAML aarch64 wheel | `xlevr/config.py` | `pip download`，见下 |
| websockets aarch64 wheel | VR WebSocket 服务 | 同上 |
| 两个包的纯源码 tar.gz | 上位机无 pip 时兜底 | 解压后挂 `PYTHONPATH` |

```bash
python3 -m pip download PyYAML websockets \
  --only-binary=:all: --platform manylinux2014_aarch64 \
  --python-version 38 --implementation cp --abi cp38 -d tmp/wheels

bash src/M20_sdk_deploy/scripts/fetch_aframe.sh
bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --check

scp tmp/wheels/*.whl tmp/wheels/*.tar.gz user@10.21.31.103:~/
```

上位机安装与自检：

```bash
python3 -V
python3 -m pip install --no-index --no-deps ~/PyYAML-*.whl ~/websockets-*.whl
python3 -c "import numpy, yaml, websockets; print('deps ok')"
```

无需额外安装：

- `scipy`：VR 旋转换算已有纯 numpy 兜底。
- `numpy`：上位机自带版本足够。
- `torch`：部署使用 ONNX Runtime。
- `pynput`：不在部署链路中。

## 3. 编译与同步

真机版必须显式关闭 SIM2SIM：

```bash
docker exec -it m20_piper_ros bash -lc '
  cd /root/m20_piper_ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select m20_sdk_deploy \
    --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF
'
```

检查构建类型后再同步：

```bash
grep -c SIM2SIM build/m20_sdk_deploy/CMakeFiles/rl_deploy.dir/flags.make
# 真机版应为 0

bash src/M20_sdk_deploy/scripts/sync_to_robot.sh
# 第二 WiFi：ROBOT=user@10.21.41.1 bash src/M20_sdk_deploy/scripts/sync_to_robot.sh
# 先预演：bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --dry-run
# 同步后直接编译：bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --build
```

不要整包使用 `scp -r`。同步脚本会排除仿真 mesh、x86 onnxruntime 和示例文件。

上位机编译：

```bash
cd ~/M20_sdk_deploy
source /opt/ros/foxy/setup.bash
colcon build --packages-select m20_sdk_deploy \
  --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF
source install/setup.bash
```

## 4. 上电与预检

```bash
sudo su
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh

echo "domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"
ros2 topic list | grep -E 'JOINTS|IMU|BATTERY'
ros2 topic hz /JOINTS_DATA    # 约 200 Hz

cd ~/M20_sdk_deploy
source install/setup.bash
python3 src/M20_sdk_deploy/scripts/preflight_real_deploy.py
```

预检必须确认：

- 二进制不是 SIM2SIM 构建。
- 策略目录包含 `policy.onnx` 和 `policy_layout.json`。
- `/JOINTS_DATA` 等基础话题有发布者。
- 机械臂相关话题只有一个预期发布者。

### 4.1 腿部标定

把机器人摆到训练默认姿态，读取 `/JOINTS_DATA`：

```bash
ros2 topic echo --once /JOINTS_DATA
```

前腿目标应接近 `(0, -0.6, 1.0, 0)`，后腿 `(0, 0.6, -1.0, 0)`。
误差大于 0.05 rad 时先修 `M20Interface` 的 `dir/offset`，不要继续进 RL。

## 5. 阶段 A：只上腿

策略观测包含机械臂，所以无机械臂时必须启动假臂节点。

| 终端 | 命令 |
|---|---|
| T1 | `python3 src/M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py` |
| T2 | `python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile legs` |

检查假臂数据：

```bash
ros2 topic echo --once /ARM_JOINTS_DATA
# 8 个位置：0 / 0.5 / -0.5 / 0 / 0 / 0 / 0 / 0

ros2 topic echo --once /ARM_TELEOP_STATE
# (0.3492, 0, 0.4327, -0.7373, 0, -0.6756, 0)
```

### 5.1 状态机

当前默认：

- idle 零增益，机器人处于软状态。
- 按 `z` 使用厂商两段式站立：两段各 1.5 s，腿增益保持厂商 `200/4`。
- 按 `c` 进入 RL，策略从厂商站姿收敛到训练默认姿态。
- 按 `r` 进入关节阻尼，按 `x` 趴下。

厂商站立结束位姿与训练 RL 位姿不同，按 `c` 后会主动调整姿态。该切换是当前已知风险，
真机首跑必须吊装或清场，并尝试收紧接管阈值：

```bash
M20_TILT_TAKEOVER=0.5 M20_LEG_FOLD_TAKEOVER=0.5 \
  ros2 run m20_sdk_deploy rl_deploy
```

进入 RL 的 1~2 秒不要松开急停，先给小速度帮助脱离静止点。已知失败率见
[`STATUS_zh.md`](STATUS_zh.md)。

## 6. 阶段 B：接机械臂

先停止阶段 A 的统一控制终端，避免同时出现两个 `rl_deploy`。Jetson 驱动仍单独占
T1，上位机的 adapter、IK、臂键盘和策略合并到 T2：

| 终端 | 机器 | 命令 |
|---|---|---|
| T1 | Jetson | `ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper fast_mode:=true` |
| T2 | 上位机 | `M20_GRIPPER_SLEW=0.2 M20_ARM_MAX_JOINT_VEL=3.0 python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile real` |

监督器依次启动 adapter、IK、臂键盘，延迟 1 s 后启动 `rl_deploy`。日志前缀分别为
`[adapter]`、`[ik]`、`[arm]`、`[rl]`。直接在 T2 按键盘：

- `z` 站立、`c` 进 RL、`r` 阻尼、`x` 趴下。
- `W/S/A/D/Q/E` 控制底盘速度，`H/J/B/N/[/]` 控制机身姿态。
- 小键盘控制机械臂末端，`G` 切换夹爪，`L` 同时复位机身/臂目标。

任一节点异常退出时监督器会停止其余节点；正常收工仍先按 `r`、`x`，再按
`Ctrl+C` 或 `ESC` 停止整组。

跨机检查：

```bash
ros2 topic list | grep feedback/joint_states
python3 tests/arm_real_bridge_check.py
```

验收顺序：

1. 机械臂默认姿态约 `0, 0.5, -0.5, 0, 0, 0`。
2. `/ARM_TELEOP_STATE` 的默认末端位姿与合同文档一致。
3. 小步移动单关节，方向正确，底盘倾角小于 2°。
4. 检查 `/ARM_JOINTS_DATA` 的 torque、`motion_temp` 和 `driver_temp`。
5. 驱动侧确认 joint1~6 速度上限为 3 rad/s。

如 DDS 发现失败，先检查 domain/RMW，再使用
`scripts/setup_dds_wired.sh` 固定网卡；必要时使用
`scripts/arm_udp_bridge.py` 的 UDP 直连模式。

## 7. 阶段 C：VR

```bash
cd ~/M20_sdk_deploy
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 src/M20_sdk_deploy/interface/robot/simulation/vr_teleop_node.py
```

节点会打印 HTTPS 地址和全部候选网卡。也可指定：

```bash
M20_VR_IP=10.21.41.1 python3 src/M20_sdk_deploy/interface/robot/simulation/vr_teleop_node.py
```

头显打开打印的 `https://<IP>:8443`：

- 自签证书警告是预期行为。
- 页面顶部出现 A-Frame 缺失红条时，回到离线资源准备步骤。
- 正常页面应有 `Enter VR` 和 `Start Controller Tracking`。

### 7.1 控制分工

| 输入 | 作用 |
|---|---|
| 右手柄 | 机械臂位置与姿态 |
| 右手柄扳机 | 夹爪 |
| 右手柄 B | 启动 / 重新标定 |
| 左手柄摇杆 | 机器人速度 |
| 左手柄位置/姿态 | 机身高度、pitch、roll |
| 左手柄 X / Y | 复位 |

复位语义：机身回默认、机械臂回默认、速度归零并交还控制权。再次按 B 重新标定后继续控制。

### 7.2 VR 首次操作

1. 先只测试左手柄小速度。
2. 再测试右手柄小范围移动。
3. 测试夹爪开合。
4. 测试复位和重新接管。
5. 最后做行走 + 机械臂组合动作。

链路自检：

```bash
python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py --seconds 8
```

该脚本应显示关键话题各只有一个发布者。

## 8. 常用调节参数

| 参数 | 默认 | 作用 |
|---|---:|---|
| `M20_VR_MAX_VX/_VY/_WZ` | 1.0 / 0.5 / 0.7 | 底盘速度上限 |
| `M20_VR_ARM_POS_SENS` | 0.5 | VR 机械臂位置灵敏度 |
| `M20_VR_ARM_ROT_SENS` | 0.5 | VR 机械臂姿态灵敏度 |
| `M20_VR_MAX_LIN_SPEED` | 0.4 m/s | 机械臂目标线速度 |
| `M20_VR_MAX_ANG_SPEED` | 1.0 rad/s | 机械臂目标角速度 |
| `M20_ARM_MAX_JOINT_VEL` | 0（现场建议 3.0） | 关节指令速度上限 |
| `M20_VR_ARM_MAP` | `cylindrical` | VR 机械臂映射，不建议改回 `cartesian` |

## 9. 安全与退出

上真机前必须确认：

- 软件 `r` 可进入 joint damping。
- 硬件急停可立即切断危险动作。
- 进入 RL 时人员与设备处于安全距离。
- 所有关键话题只有一个发布者。

退出顺序：

```text
r 关节阻尼 → x 趴下 → 停 VR → 在统一控制终端按 Ctrl+C/ESC（一并停臂键盘/IK/adapter/rl_deploy）→ 退出 SDK 模式
```

## 10. 排障

| 现象 | 优先检查 |
|---|---|
| 真机完全看不到话题 | SDK 模式、domain、RMW、drdds 服务 |
| 构建仍像仿真 | `flags.make` 中是否残留下 `-DSIM2SIM` |
| 策略加载失败 | `M20_POLICY_DIR` 使用绝对路径；layout/ONNX 成对 |
| 仿真正常但真机慢慢塌 | 策略未加载或仍在 idle；检查启动日志 |
| 机械臂目标不变、反馈混入常量 | 是否有残留 `arm_stub_node` / 第二个 adapter |
| VR 页面只有一张卡片 | `aframe.min.js` 是否被同步并可返回 200 |
| VR 只有机械臂不动 | 运行 `check_vr_arm_chain.py`，检查 `/ARM_TELEOP` |
| 机械臂自己甩 | 先查发布者数量，再查 IK 残差和关节限位 |
| 仿真跑不住 | 检查实时因子是否接近 1.0；电池降频时结果无效 |
| 真机站立持续升温 | 立即记录数据；当前状态见 [`STATUS_zh.md`](STATUS_zh.md)，证据见 [`DEFECTS_zh.md`](DEFECTS_zh.md) DEF-059/DEF-060 |

## 11. 真机数据录制

一条命令录制 16 关节与 IMU：

```bash
bash src/M20_sdk_deploy/scripts/record_joint_data.sh stand_test 600
```

离线报告：

```bash
python3 src/M20_sdk_deploy/scripts/joint_log_report.py \
  tmp/real_logs/stand_test.csv --ref standup
```

该工具会区分保持段、卸力段和过温事件，并换算到策略关节方向。

## 12. 尚未在真机验证

- 腿部标定残差。
- `arm_real_adapter` 实机联调。
- agx 驱动侧 3 rad/s 限速。
- 端到端时延与抖动。
- 硬件急停和固件限幅的一致性。
- 新 VR 机械臂映射的真机复核。
- 无机械臂策略布局。
