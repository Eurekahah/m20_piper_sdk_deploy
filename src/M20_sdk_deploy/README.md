# M20 SDK Deploy

M20 轮足机器人 + AgileX Piper 机械臂的 ROS 2 部署包，覆盖 sim2sim、sim2real、
策略部署、机械臂 IK、键盘和 VR 遥操作。

## 文档入口

- [文档索引](../../docs/README_zh.md)
- [当前状态与待办](../../docs/STATUS_zh.md)
- [部署运行](../../docs/DEPLOY_zh.md)
- [接口契约](../../docs/CONTRACT_zh.md)
- [开发与测试](../../docs/DEVELOPMENT_zh.md)
- [缺陷历史](../../docs/DEFECTS_zh.md)
- [策略目录约定](policy/README.md)

日常复制命令：

- [sim2sim 命令速查](COMMANDS_SIM2SIM_zh.md)
- [sim2real 命令速查](COMMANDS_SIM2REAL_zh.md)

## 主要能力

| 模块 | 说明 |
|---|---|
| 策略 | 83 维观测、10×70 history、16 维动作；平地为默认模型 |
| 状态机 | idle、站立、RL、趴下、关节阻尼和安全接管 |
| 腿部接口 | 真机 drdds 接口与 MuJoCo 恒等标定接口 |
| 机械臂 | Piper 6 关节 + 两指夹爪，DLS IK，键盘和 VR 命令 |
| 真机臂 | `arm_real_adapter.py` 连接 `agx_arm_ros` |
| VR | XLeVR 页面、离线 A-Frame、机身 + 机械臂遥操作 |
| 工具 | 预检、同步、遥测、命令扫描、关节温度录制和离线分析 |

当前接口和参数以 [CONTRACT_zh.md](../../docs/CONTRACT_zh.md) 为准。

## 目录

```text
src/M20_sdk_deploy/
  interface/                 机器人、DDS、MuJoCo、机械臂和 VR 节点
  state_machine/             idle/standup/RL/liedown/damping 状态机
  run_policy/                策略加载、观测和动作映射
  policy/                    训练部署产物
  scripts/                   预检、同步、测试和数据分析
  M20_Piper_description/     MJCF 与 mesh
  third_party/               XLeVR、Eigen、ONNX Runtime、手柄等
```

## 环境

- 开发机建议 ROS 2 Humble 容器。
- M20 上位机使用 ROS 2 Foxy。
- 策略文件必须成对存在：
  `policy.onnx` + `policy_layout.json`。
- 真机版必须显式使用 `-DSIM2SIM=OFF`。

## 编译

### sim2sim

```bash
cd /root/m20_piper_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to m20_sdk_deploy \
  --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON
source install/setup.bash
```

### 真机

```bash
colcon build --packages-select m20_sdk_deploy \
  --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF
```

确认构建类型：

```bash
grep -c SIM2SIM build/m20_sdk_deploy/CMakeFiles/rl_deploy.dir/flags.make
# 真机应为 0
```

## 快速运行

### 全仿真

宿主机一行式命令：

```bash
docker exec -it m20_piper_ros bash -lc '
  cd /root/m20_piper_ws &&
  source /opt/ros/humble/setup.bash &&
  source install/setup.bash &&
  export ROS_DOMAIN_ID=1 &&
  exec python3 tests/sim2sim_smoke.py --mode rl --viewer --duration 60
'
```

启动后按：

- `z`：站立。
- `c`：进入 RL。
- `w/s/a/d/q/e`：速度命令。
- `r`：关节阻尼。
- `x`：趴下。

实际部署时，仿真单独一个终端；策略、IK 和臂键盘由
`src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py` 合并到另一个终端。
完整分场景流程见 [COMMANDS_SIM2SIM_zh.md](COMMANDS_SIM2SIM_zh.md)。

### 真机

先完成编译、同步、SDK 模式和预检：

```bash
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh
python3 src/M20_sdk_deploy/scripts/preflight_real_deploy.py
python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile real
```

`--profile real` 会统一启动真臂 adapter、IK、臂键盘和策略；VR 节点仍单独运行。
真机首跑、机械臂和 VR 的完整步骤见
[DEPLOY_zh.md](../../docs/DEPLOY_zh.md)。

## 测试

```bash
# 完整仿真验收
bash tests/run_all.sh

# 策略接口
python3 src/M20_sdk_deploy/scripts/check_policy_interface.py

# 统一键盘监督器按键分流与退出
python3 tests/keyboard_teleop_supervisor_check.py

# MJCF 与训练参数对照
python3 src/M20_sdk_deploy/scripts/check_mjcf_contract.py

# 22 阶段命令扫描
python3 tests/command_sweep.py --repeat 3
```

测试判据、实时因子要求和当前测试口径问题见
[DEVELOPMENT_zh.md](../../docs/DEVELOPMENT_zh.md)。

## 真机关节数据

只订阅、不控制，因此原厂控制器站立时也可录制：

```bash
bash src/M20_sdk_deploy/scripts/record_joint_data.sh <tag> 600
```

输出位于 `~/joint_logs/<tag>.csv`。分析：

```bash
python3 src/M20_sdk_deploy/scripts/joint_log_report.py \
  tmp/real_logs/<tag>.csv --ref standup
```

## 已知限制

- 站立热安全尚未完全闭环，见 [STATUS_zh.md](../../docs/STATUS_zh.md)
  和 [DEFECTS_zh.md](../../docs/DEFECTS_zh.md) 的 DEF-059/DEF-060。
- 厂商站立到训练 RL 姿态仍存在入口瞬态。
- 安全层的低电量、命令心跳和部分超时保护未完整接线。
- 真机标定、时延、急停和机械臂实机联调仍待验收。

完整状态和优先级见 [STATUS_zh.md](../../docs/STATUS_zh.md)。

## 上游与许可证

本项目基于 Deep Robotics M20 SDK 和 XLeVR 等第三方组件。
许可证和第三方说明以仓库 `LICENSE` 及各 `third_party/` 目录为准。
