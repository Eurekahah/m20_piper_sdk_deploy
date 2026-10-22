# 部署接口与仿真/真机契约

**职责**：记录部署侧必须遵守的输入输出、关节顺序、坐标系、增益、限幅和仿真差异。
运行步骤见 [`DEPLOY_zh.md`](DEPLOY_zh.md)，当前风险见 [`STATUS_zh.md`](STATUS_zh.md)。

## 1. 策略接口

当前部署策略统一使用以下布局；更换策略时必须同时更换 `policy.onnx` 和
`policy_layout.json`。

| 项 | 值 |
|---|---|
| 控制频率 | 50 Hz |
| 观测 | `policy_obs[83]` |
| 历史 | `history_flat[700] = 10 × 70`，最旧 → 最新 |
| 动作 | 16：12 个腿关节 + 4 个轮子 |
| 机械臂 | 不占动作维度；由 `ee_goal` 经 IK 驱动 |
| 默认策略 | `policy/m20_piper_flat_20260930/` |
| 可用策略 | 平地 20260930、多地形 20261002、历史平地 20260920 |

权威来源按以下优先级使用：

1. `policy_layout.json`、ONNX I/O 形状和当前部署代码。
2. 训练 run 的 `params/env.yaml`。
3. `scripts/check_policy_interface.py` 与 `scripts/check_mjcf_contract.py` 的实测断言。
4. 训练仓库部署文档仅作参考。

## 2. `policy_obs` 布局

顺序为：计算 → clip → scale；部署不加噪声，但必须保持 clip 与 scale 的顺序。

| 切片 | 内容 | 维度 | scale / 说明 |
|---|---|---:|---|
| 0..2 | 机体系角速度 | 3 | 0.25 |
| 3..5 | 投影重力 | 3 | 1.0 |
| 6..8 | `vx, vy, wz` | 3 | 1.0 |
| 9..32 | 关节位置相对默认值 | 24 | 原生序，轮子列置零 |
| 33..56 | 关节速度 | 24 | 0.05 |
| 57..72 | 上一步动作 | 16 | 喂回实际下发的动作 |
| 73..79 | `ee_goal` | 7 | root 系位置 + 四元数 `wxyz`，clip ±3 |
| 80..82 | `height, pitch, roll` | 3 | 高度使用相对足端定义 |

### 2.1 高度

```text
height = root_z - mean(z_fl_wheel, z_fr_wheel, z_hl_wheel, z_hr_wheel) + 0.09
```

训练命令范围是 `(0.33, 0.55)`。默认 RL 指令为 0.513 m。

### 2.2 `ee_goal` 四元数

- 必须使用训练侧链式 FK 产生的符号分支，默认姿态为
  `(-0.7373, 0, -0.6756, 0)`，不能随意归一化成 `w >= 0`。
- 所有发布 `/ARM_TELEOP_STATE` 的节点必须使用同一符号约定并在姿态跨 180° 时保持连续。
- 符号错误会改变策略输入分布，历史实测会造成零速度下持续前向漂移。

## 3. History 布局

每一步 70 维，使用原始值，不乘 policy scale；整条 history 只做 `clip ±100`。

| 切片 | 内容 | 维度 |
|---|---|---:|
| 0..2 | 原始角速度 | 3 |
| 3..5 | 投影重力 | 3 |
| 6..29 | 24 个关节位置相对值，含轮子 | 24 |
| 30..53 | 24 个关节速度原始值 | 24 |
| 54..69 | 上一步动作 | 16 |

- 展平顺序为最旧到最新。
- 每 20 ms 推一步。
- 进入 RL 或复位后，用当前帧填满整个 history 窗口。

## 4. 动作映射

| 动作槽 | 关节 | 目标 |
|---|---|---|
| 0..11 | `fl/fr/hl/hr` 的 `hipx/hipy/knee` | `q_default + scale·a` |
| 12..15 | 四轮 | `ω_des = 5·a`，`τ = 0.6(ω_des-ω)` |

- hipx 动作 scale 为 0.125，其余腿关节为 0.25。
- 若部署侧增加安全限幅，`actions` 观测必须喂回限幅后的同一个值。

## 5. 关节顺序

动作序是四腿分组：`fl, fr, hl, hr`，每腿 `hipx/hipy/knee/wheel`。

24 维观测使用的是训练 articulation 原生交错序：

```text
 0..3   fl_hipx, fr_hipx, hl_hipx, hr_hipx
 4      arm_joint1
 5..8   fl_hipy, fr_hipy, hl_hipy, hr_hipy
 9      arm_joint2
10..13  fl_knee, fr_knee, hl_knee, hr_knee
14      arm_joint3
15..18  fl_wheel, fr_wheel, hl_wheel, hr_wheel
19..21  arm_joint4, arm_joint5, arm_joint6
22..23  gripper_joint1, gripper_joint2
```

不要按位置猜测关节顺序。必须通过 `policy_layout.json` 和关节名映射。

## 6. 默认关节姿态

| 部位 | 默认值 |
|---|---|
| 前腿 | `hipx 0 / hipy -0.6 / knee 1.0 / wheel 0` |
| 后腿 | `hipx 0 / hipy 0.6 / knee -1.0 / wheel 0` |
| 机械臂 | `0, 0.5, -0.5, 0, 0, 0` |
| 夹爪 | `0, 0` |

默认姿态下 `gripper_base` 相对 `base_link` 约：

```text
position = (0.3492, 0, 0.4327) m
quaternion_wxyz = (-0.7373, 0, -0.6756, 0)
```

## 7. 执行器参数

| 执行器 | 力矩上限 | 速度上限 | kp / kd | armature |
|---|---:|---:|---:|---:|
| 腿 | 76.4 N·m | 22.4 rad/s | 策略 80/2 | 0 |
| 轮 | 21.6 N·m | 79.3 rad/s | 0/0.6 | 0.00243216 |
| 机械臂 | 100 N·m | 3.0 rad/s | 300/8 | 0.01 |
| 夹爪 | 10 N·m | 1.0 rad/s | 286/5 | 0.01 |

注意：

- `76.4 N·m` 是训练/仿真的峰值 effort limit，不是电机连续热额定。
- 站立状态机仍使用厂商 `200/4`，这是当前过温专项的重点。
- MuJoCo 的 `armature` 必须写在 `<joint>`，写在 `<motor>` 会被静默忽略。

## 8. sim2sim 与 sim2real 差异

| 项 | sim2sim | sim2real | 状态 |
|---|---|---|---|
| 编译 | `-DSIM2SIM=ON` | 必须显式 `-DSIM2SIM=OFF` | CMake 会缓存，必须检查 |
| 腿接口 | `M20SimInterface` 恒等标定 | `M20Interface` 的 `dir/offset` | 真机待核对 |
| IMU | 仿真 deg，由 DDS 层转 rad | 同样转换 | 已一致 |
| 机械臂 | IK 输出直接进 MuJoCo | IK → adapter → agx/CAN | 真机待联调 |
| 轮子 | 仿真 kp=0/kd=0.6 | 固件速度环 | 符号和量纲待真机复核 |
| 温度 | 固定 40/45 ℃ | 真实绕组/驱动器温度 | 仿真无热模型 |
| 接触摩擦/齿隙 | 简化模型 | 有静摩擦、齿隙和装配差异 | 当前仿真姿态负载预测不可靠 |
| 执行器延迟 | 可开关模拟 0~25 ms | 真实端到端时延未测 | 真机待验收 |
| 急停 | 可使用 `r` / 软件接管 | 软件 `r` + 硬件急停 | 真机待实测 |

## 9. 仿真物理差异的定案

- 控制周期：训练和部署均为 50 Hz。
- MuJoCo 物理积分步长更细：`0.0002 s × 5` 为 1 ms 控制 tick。
- 训练侧执行器延迟为 0~25 ms；仿真默认关闭，仅作对照工具。
- 摩擦固定为训练分布内的 1.0，恢复系数固定为 0；训练侧有随机化。
- PhysX 与 MuJoCo 求解器无法逐项等价。
- `base_link` 显式惯性已与 URDF 对齐；质量 15.882 kg，主惯量一致。

这些差异不阻断接口验证，但会降低动力学和热行为的可外推性。真机站姿/热安全必须真机实测。

## 10. 修改接口时的最低检查

```bash
python3 src/M20_sdk_deploy/scripts/check_policy_interface.py
python3 src/M20_sdk_deploy/scripts/check_mjcf_contract.py
python3 tests/sim2sim_smoke.py --mode rl --duration 20 --repeat 6
```

验收要求：

- ONNX 名称、形状、layout 和 C++ 关节顺序交叉一致。
- 喂回观测的动作与真正下发动作逐元素一致。
- 默认末端位姿误差 < 1 mm，旋转误差可解释。
- 仿真中零命令能稳定站立，且没有持续方向漂移。
