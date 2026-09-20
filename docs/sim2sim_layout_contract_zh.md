# 部署接口契约（sim2sim / sim2real）

**文档职责**：把"部署侧必须照抄的接口"写死，并记录**每一条的核对状态**。
管：观测/动作布局、关节顺序、坐标系、增益、限幅、命令语义、MuJoCo 侧参数。
不管：待办（`docs/review/TODO_zh.md`）、缺陷来龙去脉（`docs/review/DEFECT_LOG_zh.md`）、
流程规范（`docs/review/WORKFLOW_zh.md`）。

**核对状态标记**：`✅ 已核对`（在本仓库或训练仓库有实测/源码证据）/
`⚠️ 与我方不一致`（两边说法不同，必须处理）/
`❓ 待实测`（只能推断，需要跑一次才知道）/ `❌ 训练侧文档有误`。

## 更新记录

| 日期 | 更新内容 | 相关 commit / 分支 |
|---|---|---|
| 2026-09-20 | 初版：核对训练侧 `docs/deploy_sim2sim_sim2real_zh.md` 的每一条；加入新 checkpoint（83/700/16）的布局 | `docs/review-spec` |

---

## 0. 一眼速查（针对最新 checkpoint）

| 量 | 值 | 状态 |
|---|---|---|
| 策略 run | `logs/rsl_rl/history_adaptation/2026-09-20_00-50-31`，checkpoint `model_19999.pt` | ✅ |
| 部署产物 | `<run>/exported_deploy/{policy.pt, policy.onnx, policy_layout.json}`；本仓库镜像在 `src/M20_sdk_deploy/policy/m20_piper_history_20260920/` | ✅ |
| 资产 | `deep_robotics_model/M20_Piper_own/usd/M20_Piper_own.usd`（= 本仓库 MJCF 的同一份 URDF） | ✅ |
| 策略频率 | **50 Hz**（`sim.dt=0.005` × `decimation=4`） | ✅ |
| 输入 | `policy_obs (batch,83)` + `history_flat (batch,700)` | ✅ |
| 输出 | `action (batch,16)`：12 腿位置 + 4 轮速度；**动作里没有机械臂** | ✅ |
| 腿目标 | `q_des = q_default + gain·a`，hipx **0.125**，其余腿关节 **0.25** | ✅ |
| 轮目标 | `ω_des = 5.0·a`，力矩 = `0.6·(ω_des − ω)`（kp=0） | ✅ |
| 机械臂 | 由 **IK** 从 `ee_pose` 命令驱动（不占动作维度） | ✅（需部署侧自己实现） |
| history | 10 步 × 70 维，**最旧→最新** | ✅ |
| 终止阈值（建议当接管阈值） | 倾角 > 0.8 rad、`root_z − mean(wheel z) + 0.09` < 0.30 m | ✅ |

### 权威来源（改部署脚本时照着看）

| 文件 | 内容 |
|---|---|
| `<run>/exported_deploy/policy_layout.json` | 部署态接口的**机器可读**版本（维度、ONNX 输入输出名/形状、history 语义、数值自检结论） |
| `<run>/params/env.yaml` | 训练配置的**完整快照**：`init_state`、actuators、observations 的 scale/clip/noise、`sim.dt`、`decimation`、`clip_actions` |
| `source/.../config/wheeled/deeprobotics_m20/rough_env_cfg.py` | 动作 term 的 `joint_names` 与 `scale`（`__post_init__`，约 290~365 行） |
| `source/.../config/wheeled/deeprobotics_m20/flat_env_wbc_cfg.py` | 本任务的命令与课程终值（`WBCCommandsCfg` / `WBCCurriculumCfg`） |
| `source/.../velocity/mdp/observations.py::history_single_step_obs` | history 每步 70 维的**唯一定义** |
| `source/.../velocity/mdp/commands.py` | `HeightInvariantEECommand`（EE 采样 + 插值 + `command_local`）、`BodyPoseCommand` |
| `source/.../velocity/mdp/actions.py::CommandDrivenIKAction` | IK：读 `command_manager` 的 7 维目标、`action_dim=0` |
| `deep_robotics_model/M20_Piper_own/urdf/M20_Piper_own.urdf` | 关节轴与限位的权威来源 |
| 本仓库 `scripts/check_mjcf_contract.py` | **在本仓库实测**：MJCF vs 训练配置逐项比对（可重复运行） |
| 训练侧 `docs/deploy_sim2sim_sim2real_zh.md` / `scripts/.../probe_deploy_layout.py` | 训练侧写的部署手册与布局探针（**参考，不是权威** —— 见第 1 节） |

---

## 1. 对训练侧部署文档的逐条核对

结论：**大部分正确且很有用；3 处需要修正/存疑**（下表 ⚠️/❌/❓）。

| # | 训练侧文档的说法 | 核对结果 | 证据 |
|---|---|---|---|
| 1 | 策略 50 Hz（`sim.dt=0.005` × `decimation=4`） | ✅ | `env.yaml`: `sim.dt 0.005`、`decimation 4` |
| 2 | 输入 83 + 700，输出 16 | ✅ | `policy_layout.json`；ONNX 图（opset 17，`policy_obs['batch',83]`+`history_flat['batch',700]`→`action['batch',16]`） |
| 3 | 腿目标 `q_default + gain·a`，hipx 0.125、其余 0.25 | ✅ | `rough_env_cfg.py`: `joint_pos.scale = {".*_hipx_joint": 0.125, '^(?!.*_hipx_joint)(?!.*arm_joint).*': 0.25}` |
| 4 | 轮 `ω=5.0a`，力矩 `0.6(ω_des−ω)` | ✅ | `joint_vel.scale = 5.0`；`env.yaml` wheel actuator `stiffness 0.0 / damping 0.6` |
| 5 | 默认关节角（前后腿 hipy/knee 反号、arm2 0.5 / arm3 −0.5） | ✅ | `env.yaml` `init_state.joint_pos` 逐项一致 |
| 6 | 动作里没有机械臂（`ee_ik.action_dim = 0`） | ✅ | `mdp/actions.py::CommandDrivenIKAction.action_dim` 返回 0 |
| 7 | `policy_obs` 分片与 scale/clip（ang_vel×0.25、joint_vel×0.05、`ee_goal` clip ±3） | ✅ | `env.yaml` `observations.policy.*` 逐项 |
| 8 | `joint_pos` 24 维原生序、轮子列置零；`joint_vel` 24 维 | ✅（维度与语义） | `joint_pos_rel_without_wheel` + `joint_names=".*"`，`wheel_asset_cfg` = 4 轮 |
| 9 | **24 维"原生序"到底是哪一个** | ❓ **待实测** | 训练侧给的是**交错序**（探针在 `M20_Piper_own` 上的输出）；本仓库只有**另一个资产**（`M20_adjusted`）的实测（分组序）。见第 3 节 |
| 10 | history 每步 70 = `[ang_vel3, grav3, joint_pos24, joint_vel24, last_action16]`，原始值不乘 scale | ✅ | `history_single_step_obs` 源码；`HistoryCfg.history_obs` 只设 `clip=(-100,100)` |
| 11 | history 展平顺序"最旧→最新"，推窗节奏 = 策略周期，复位后整窗填同一帧 | ✅ | `ActorCriticHistory.history_length=10`（`agent.yaml`）+ 训练侧 `DEF-014` 的实测 |
| 12 | `base_velocity` 终值 vx (−5,5)/vy (−1,1)/wz (−1,1) | ✅ | `WBCCurriculumCfg.base_velocity_lin_vel_x_s7 = (-5,5)`；vy/wz 初值 ±1.0 |
| 13 | `body_pose` 终值 height (0.33,0.55)、pitch ±0.35、roll ±0.25 | ✅ | `WBCCommandsCfg.body_pose.height_range=(0.33,0.55)` 等 |
| 14 | `body_pose.height` = `root_z − mean(4 轮 z) + 0.09` | ✅ **且本仓库实测复核过** | `check_mjcf_contract.py`：默认姿态着地后 `rel_height = 0.5266`，与文档给的 0.5266 完全一致 |
| 15 | `ee_pose` 球坐标范围 l (0.30,0.52)、pitch (−0.785,0.628)、yaw ±1.257、o_roll/o_pitch ±0.3927、o_yaw ±π | ✅ | `WBCCurriculumCfg.ee_goal_stages` 的 s3 = 原分布 |
| 16 | "锁在举臂默认位姿比放开更差；起步建议低位锚点 l=0.41,pitch=−0.08,姿态 0" | ✅（引用训练侧实测） | `flat_env_wbc_cfg.py` 注释里的 1.0% / 55.5% / 25.8% 对照 |
| 17 | IK：DLS `lambda=0.01`、绝对位姿、`arm_joint1..6`、末端 `gripper_base`、root 系、50 Hz | ✅ | `CommandDrivenIKAction.process_actions` 读 `command_manager` 的 root 系目标；`command_local = pose_command_b` |
| 18 | MuJoCo 模型：默认姿态 `gripper_base` 相对 `base_link` = `(0.3492,0,0.4327)`，四元数 `(−0.7373,0,−0.6756,0)` | ✅ **本仓库实测** | `check_mjcf_contract.py`：`pos=[0.3492,0,0.4326]`（差 5.7e−05 m）、旋转角差 **0.000°** |
| 19 | MJCF `timestep=0.002` vs 训练 `0.005` | ✅ | `model.opt.timestep = 0.002`（实测） |
| 20 | 关节轴与 URDF 一致、无符号翻转 | ✅ **本仓库实测** | MJCF 的 `axis` 与 URDF 逐条相同（hipx `-1 0 0`；hipy/knee/wheel `0 -1 0`；arm `0 0 1`） |
| 21 | 关节限位表：`hipx` 前腿 [−0.436,0.611]／后腿 [−0.611,0.436] | ❌ **写错了** | URDF 与 MJCF 都是 **左右镜像**：`fl/hl` = [−0.436,0.611]、`fr/hr` = [−0.611,0.436]。hipy/knee 那两行是对的（前后镜像） |
| 22 | 执行器：腿 80/2、轮 0/0.6、臂 **300/20**、夹爪 4000/200；armature 轮 0.00243216、臂/夹爪 0.01 | ✅（训练值本身） | `env.yaml` `actuators` 四组 |
| 23 | 训练侧文档没有提"armature 要写在 `<joint>` 上" | ⚠️ **本仓库踩坑** | 本仓库 MJCF 把 armature 写在 `<motor>` 上，**MuJoCo 静默忽略** ⇒ 轮/臂/夹爪的 `dof_armature` 实际是 0。见 `DEF-011` |
| 24 | 时延：训练 `DelayedPD` 每个执行器 0~5 个物理步（0~25 ms） | ✅（训练侧） | `env.yaml` `min_delay 0 / max_delay 5`；**部署侧未实现**（TODO P2-3） |
| 25 | 摩擦/恢复系数：训练随机化 0.35~1.5 / 0~0.7 | ❓ | `env.yaml` 里 `sim.physics_material` 是 1.0/1.0/1.0（基础值），随机化区间在 events 配置里；**未逐项核对** |
| 26 | 根初始高度 0.55（spawn） | ⚠️ | `env.yaml` `init_state.pos = (0,0,0.55)`；本仓库 MJCF 的 `base_link pos = 0.5901`，但仿真节点会按"轮子着地"重新落位（`M20_Piper_own` 默认站姿落地后 base z = 0.5266） |

---

## 2. 新 checkpoint 的接口契约（细节）

### 2.1 `policy_obs`（83 维）

处理顺序（IsaacLab `observation_manager`）：`compute → 加噪声 → clip → 乘 scale`。
**部署不加噪声**，但 clip 与 scale 必须照做，且顺序不能反（先 clip 后 scale）。

| 切片 | 项 | 维度 | 公式 / 来源 | clip | scale |
|---|---|---|---|---|---|
| 0..2 | `base_ang_vel` | 3 | 机体系角速度（IMU 直接给） | ±100 | **0.25** |
| 3..5 | `projected_gravity` | 3 | `R_bᵀ·[0,0,−1]` | ±100 | 1.0 |
| 6..8 | `velocity_commands` | 3 | `[vx, vy, wz]`（机体系） | ±100 | 1.0 |
| 9..32 | `joint_pos` | **24** | `q − q_default`，**原生序**，**4 个轮子列置零** | ±100 | 1.0 |
| 33..56 | `joint_vel` | **24** | `q̇`，原生序 | ±100 | **0.05** |
| 57..72 | `actions` | **16** | 上一步的策略输出（原始值，`clip_actions=100`） | ±100 | 1.0 |
| 73..79 | `ee_goal` | 7 | `[pos_b(3), quat_b(4) wxyz]`（**root 系**目标） | **±3** | 1.0 |
| 80..82 | `body_pose_cmd` | 3 | `[height, pitch, roll]` | — | — |

> ⚠️ 与旧 checkpoint 的差别：`joint_pos`/`joint_vel` 从 22 维（leg+wheel+arm）
> 变成 **24 维**（含夹爪）；`actions` 从 23 维变成 **16 维**（`ee_ik` 槽位消失）。

### 2.2 `history_flat`（700 = 10 × 70）

每步 70 维（`history_single_step_obs`，**原始值，不乘 scale / 不加噪声**，只 `clip ±100`）：

| 切片 | 项 | 维度 |
|---|---|---|
| 0..2 | `base_ang_vel`（原始，不乘 0.25） | 3 |
| 3..5 | `projected_gravity` | 3 |
| 6..29 | `joint_pos_rel`（**全部 24 个关节，含轮子，不置零**） | 24 |
| 30..53 | `joint_vel_rel`（原始，不乘 0.05） | 24 |
| 54..69 | `last_action` | 16 |

* 展平顺序：**最旧 → 最新**；推窗节奏 = 策略周期（20 ms），不是物理步。
* 复位后第一帧：**用同一帧填满整窗**（不要用 0 填）。

### 2.3 动作（16 维）→ 关节

| 槽位 | 关节（动作序，不是原生序） | 目标 | 增益 |
|---|---|---|---|
| 0..2 | `fl_hipx / fl_hipy / fl_knee` | `q_default + gain·a` | 0.125 / 0.25 / 0.25 |
| 3..5 | `fr_...` | 同上 | 0.125 / 0.25 / 0.25 |
| 6..8 | `hl_...` | 同上 | 0.125 / 0.25 / 0.25 |
| 9..11 | `hr_...` | 同上 | 0.125 / 0.25 / 0.25 |
| 12..15 | `fl/fr/hl/hr_wheel` | `ω = 5.0·a`（力矩 `0.6(ω_des−ω)`，kp=0） | 5.0 |

动作 term 级的 `clip_actions` 在训练里是 **±100**（`agent.yaml: clip_actions: 100`）。

### 2.4 部署侧必须在"实际下发"与"喂回观测"之间保持一致

训练里 `actions` 观测 = `env.action_manager.action` = **控制用的同一个值**。
部署侧如果再加一层安全限幅（例如 ±3），就必须把**限幅后**的值喂回 `actions`，
否则策略会看到"我命令了 X，机器人收到 Y"（`DEF-006`）。

---

## 3. 关节顺序（已实测判定 ✅）

24 维 `joint_pos` / `joint_vel` 用的是 **articulation 原生序**（PhysX 上报顺序），
它既不是动作序、也不是 MJCF 序。历史上有三种写法（`DEF-005`），
**2026-09-20 由训练侧 `probe_deploy_layout.py` 在 `M20_Piper_own` 上实测判定**：

```
idx  0- 3  fl_hipx, fr_hipx, hl_hipx, hr_hipx
idx  4     arm_joint1
idx  5- 8  fl_hipy, fr_hipy, hl_hipy, hr_hipy
idx  9     arm_joint2
idx 10-13  fl_knee, fr_knee, hl_knee, hr_knee
idx 14     arm_joint3
idx 15-18  fl_wheel, fr_wheel, hl_wheel, hr_wheel
idx 19-21  arm_joint4, arm_joint5, arm_joint6
idx 22-23  gripper_joint1, gripper_joint2
```

即"**交错序**"（= 训练侧部署文档第 4 节那张表），`wheel = 15..18`。
这份顺序同时写进了两个地方，两边必须一致：

* `policy/<run>/policy_layout.json` 的 `joint_order_native` 字段；
* `M20PiperPolicyRunner::NativeOrder()`（C++ 侧）。

交叉校验由 `scripts/check_policy_interface.py` 的第 [3] 项自动完成；
**动作序**（12 腿 fl,fr,hl,hr + 4 轮）另记在 `joint_order_action`。

探针同一次输出还顺带核实了：默认角（hipy ∓0.6 / knee ±1.0 / arm2 0.5 / arm3 −0.5）、
硬限位（**hipx 是左右镜像**：`fl/hl`=[−0.436,0.611]、`fr/hr`=[−0.611,0.436]）、
软限位 = 硬限位中点 ± 半宽×0.9、轮子无位置限位（±inf）——
与 `scripts/check_mjcf_contract.py` 在 MJCF 上测到的完全一致。

> 遗留（`TODO_zh.md` P0-8）：目前"顺序错了"只能靠端到端摔倒或人工比对发现，
> 下一步可以把 `joint_order_native` 做成运行时断言（例如用一处已知的关节角
> 偏移做闭环自检）—— 但那需要真机或更细的仿真探针，先记录为待办。

---

## 4. 部署侧必须自己实现的部分

| 项 | 说明 |
|---|---|
| **机械臂 IK** | 训练侧臂不由动作驱动：`ee_pose` 命令 → 50 Hz DLS IK（λ=0.01，绝对位姿，`arm_joint1..6` → `gripper_base`）→ 位置目标。本仓库已有 `arm_controller.py`，但增益与训练不一致（`DEF-007`）、解的一致性未核对（TODO P1-4） |
| **三个命令的生成** | `base_velocity`（机体系）、`body_pose`（**相对足端**的高度 + pitch/roll）、`ee_pose`（root 系位姿）。部署侧由键盘/VR/手柄产生 |
| **复位语义** | 进 RL / 收到 reset 时：history 整窗用当前帧填满、`ee_goal` 用当前 EE 位姿、`body_pose` 用当前实测值（`DEF-008`） |
| **反馈来源** | `base_ang_vel` / `projected_gravity` 由 IMU 给（重力投影只需 roll/pitch，yaw 无关）；单位 `deg/s → rad/s`；`q`、`q̇` 由编码器给，**减默认角要用同一张默认角表** |
| **安全** | 力矩/速度限幅（腿 76.4 N·m / 22.4 rad/s；轮 21.6 / 79.3；臂 100 / 3.0；夹爪 10 / 1.0）、通信超时兜底、软启动、急停、训练终止阈值当接管阈值 |

---

## 5. MuJoCo 侧：本仓库实测数据（`scripts/check_mjcf_contract.py`）

在容器 `m20_piper_ros` 里跑：

```bash
docker exec m20_piper_ros bash -lc \
  'cd /root/m20_piper_ws && python3 src/M20_sdk_deploy/scripts/check_mjcf_contract.py'
```

当前输出（2026-09-20）：

| 项 | 结果 |
|---|---|
| 关节集合（24：22 hinge + 2 slide 夹爪） | ✅ 与训练一致（夹爪是 **slide**，不是 hinge） |
| 关节限位 | ✅ 与 URDF 一致（**hipx 是左右镜像**） |
| 执行器力矩限幅（`ctrlrange`） | ✅ 24 个执行器全部匹配 |
| **armature** | ❌ **全部为 0**（写在 `<motor>` 上被 MuJoCo 忽略）—— `DEF-011` |
| `gripper_base` FK（默认姿态，相对 `base_link`） | ✅ `pos=[0.3492, 0, 0.4326]`（差 5.7e−05 m）、旋转角差 **0.000°** |
| 默认姿态着地后的"相对足端高度" | ✅ `0.5266 m`（与训练侧文档给的 0.5266 一致） |
| `imu_site` / `base_site` / `base_quat`+acc+gyro | ✅（sensordata 布局：`[0:4]` 四元数、`[4:7]` 加速度、`[7:10]` 角速度） |
| `timestep` | ⚠️ MJCF 0.002、仿真节点用 0.0002 × 5 子步、训练 `sim.dt=0.005` |
| MJCF 里额外的 `base_link` 显式惯性（`mass=15.882`） | ❓ 训练侧 USD 是否同值**未核对** |

### MuJoCo 侧与训练侧的已知差异（需要在 TODO P1-1 里逐条定案）

1. `timestep` 三者不同（MJCF 0.002 / 仿真 0.0002×5 / 训练 0.005）；
2. **armature 未生效**（`DEF-011`）；
3. 执行器延迟（训练随机 0~5 物理步）仿真侧未实现；
4. 摩擦 / 恢复系数的随机化未复刻；
5. `base_link` 的显式惯性来源未核对；
6. 仿真里"首条 `/ARM_JOINTS_CMD` 到达前"的臂默认保持增益是 **40/8**（应为 300/20，`DEF-007`）；
7. 仿真节点的初始腿部保持命令用的是 `LEG_INIT["M20"]`（旧机型的站立位姿），
   而初始 qpos 用的是 `LEG_INIT["M20_Piper_own"]` —— 两者不是同一个姿态（见 `DEF-012`）。

---

## 6. 还没核对、必须实测的清单

| 项 | 怎么测 | 通过判据 |
|---|---|---|
| 24 维原生序（第 3 节） | ✅ 已由训练侧探针判定，并写进 layout + runner（交叉断言在 L1） | — |
| 轮速符号与尺度 | 给 `a[12..15]=+1` | 四轮同向、稳态 ω ≈ 5 rad/s（±10%） |
| `ee_goal` 坐标系的端到端一致性 | 手动把臂移动一段，比较 `arm_controller` 反馈的 root 系位姿与 IK 目标 | 差 < 1e-3 m |
| IK 解 vs 训练 DLS | 20 个随机目标上比关节角 | `max|Δq| < 0.02 rad`（或写清偏差来源） |
| height 命令的度量 | 把机器人抬起/压下，看命令与实测是否同向同幅 | 稳态误差 < 0.03 m |
| 动作限幅与观测一致性 | 打印"喂回 16 维" vs "实际下发" | 逐元素相等 |
