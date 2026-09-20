# sim2real 上线检查单（M20 + Piper）

**文档职责**：从"sim2sim 已通过"走到"真机可跑"要做的核对与实测，逐条可打勾。
管：接口差异（仿真 vs 真机）、上线步骤、判据、回滚方式。
不管：待办与优先级（`docs/review/TODO_zh.md`）、缺陷来龙去脉
（`docs/review/DEFECT_LOG_zh.md`）、接口契约（`docs/sim2sim_layout_contract_zh.md`）。

**维护约定**：见 `docs/review/templates/DOC_TEMPLATE_zh.md`。每条实测都要写
**命令 + 数字 + 日期**；没做过的标 `[ ]`。

## 更新记录

| 日期 | 更新内容 | 相关 commit / 分支 |
|---|---|---|
| 2026-09-20 | 初版：接口差异表（sim vs real）+ 分阶段上线步骤 + 判据 | `main @ 0990850` |

---

## 0. 一句话

sim2sim 已经能站、能按命令走、能带臂遥操作（见 `review/DONE_zh.md` 第九~十三节）。
转真机的主要风险**不在策略**，而在三件事：**腿部标定链**、**臂的传输链路**、
**安全兜底**。本文就是按这三件事组织的。

---

## 1. 仿真 ↔ 真机的接口差异表（唯一一张要对着改的表）

| 项 | sim2sim | 真机 | 状态 |
|---|---|---|---|
| 腿部接口实现 | `M20SimInterface`（dir=1、offset=0，恒等标定） | `M20Interface`（16 关节 `dir/offset` 表 + 启动期 ±360° 多圈补偿） | `[ ]` 上线前逐条核对 |
| 选择方式 | 编译选项 `-DSIM2SIM=ON` | 同一个二进制不带该选项（`-DBUILD_PLATFORM=arm`） | `[ ]` |
| 关节命令话题 | `/JOINTS_CMD`（16 腿轮）+ `/ARM_JOINTS_CMD`（8 臂/夹爪） | 同左（drdds 消息没改，臂走独立话题） | ✅ 已一致 |
| 反馈话题 | `/JOINTS_DATA`、`/ARM_JOINTS_DATA`、`/IMU_DATA` | 同左 | ✅ 已一致 |
| IMU 单位 | 仿真发 deg（`ImuData`），`DdsInterface::HandlerIMU` 统一 deg→rad | 真机同样 deg→rad | ✅ 已一致 |
| 关节反馈坐标系 | MJCF 原始帧 = 策略帧 | **经过 dir/offset 标定**，与仿真不同 | `[ ]` 重点 |
| 轮子执行器 | kp=0 / kd=0.6 速度伺服（力矩限幅 21.6 N·m） | 由固件/驱动器实现速度环，力矩限幅在硬件 | `[ ]` 核对符号与量纲 |
| 臂传输 | `arm_controller` → `/ARM_JOINTS_CMD` → MuJoCo | `arm_real_adapter.py` → `agx_arm_ros` 的 `/control/joint_states`（`fast_mode`） | `[ ]` 见第 3 节 |
| `ee_goal` 坐标系 | root 系（`DEFECT_LOG_zh.md` DEF-019/020 已修） | 同左（同一份代码） | ✅ 已一致 |
| 仿真独有的东西 | `/reset_sim`、遥测 CSV、扰动注入、执行器延迟开关 | 无 | — 不影响 |

---

## 2. 阶段 A：只上腿（臂固定）

前提：`colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm`
（**不要**带 `-DSIM2SIM=ON`），scp 到机器人，按 `src/M20_sdk_deploy/README.md`
的"Sim-to-Real"一节启动 SDK 模式。

1. `[ ]` **标定核对（最关键）**：把机器人摆成与训练默认姿态一致的姿势
   （hipy ∓0.6、knee ±1.0、wheels 0），读 `/JOINTS_DATA`：
   * 期望：16 个关节的 `position` 接近 `(0, -0.6, 1.0, 0)`（后腿 `+0.6, -1.0`）；
   * 若偏差 > 0.05 rad，说明 `${dir, offset}` 表与本次机器人不匹配，先修表。
2. `[ ]` **假站立（不使能策略）**：`M20_TILT_TAKEOVER=0.5 M20_LEG_FOLD_TAKEOVER=0.5`
   先把安全阈值收紧启动 `rl_deploy`，只做 `z` 站立、不进 RL，确认站立姿态与仿真
   （height ≈ 0.50 m）一致。
3. `[ ]` **进 RL，零命令**：按 `c` 进 RL，**双手悬在急停上**，观察 20 s：
   * 判据：不触发 `[TAKEOVER!]`；高度稳定；没有持续单方向漂移。
   * 已知风险：入口 1~2 s 的瞬态（`DEFECT_LOG_zh.md` DEF-018，sim2sim 里约 25%
     发散）—— 真机首跑务必先给**小速度命令**（vx 0.1~0.2）帮助它脱离静止点，
     或先接受"可能第一下会晃"。
4. `[ ]` **小速度行走**：vx ≈ 0.2 → 0.5 m/s，每次 5 s，检查方向、轮速符号、
   是否走直线。
5. `[ ]` **急停 / 阻尼**：按 `r`（joint damping）、按硬件急停各一次，确认能立刻接管。
6. `[ ]` **安全阈值定档**：确认后把 `M20_TILT_TAKEOVER` / `M20_LEG_FOLD_TAKEOVER`
   调回默认（0.8 / 1.2），再重复第 3 步。

---

## 3. 阶段 B：接机械臂

1. `[ ]` CAN 与驱动：`agx_arm_ctrl start_single_agx_arm.launch.py
   can_port:=can0 arm_type:=piper effector_type:=agx_gripper fast_mode:=true`。
2. `[ ]` `arm_real_adapter.py` 桥接（映射 `arm_joint1..6 + gripper` ↔
   `/control/joint_states`；夹爪宽度 = `q6 − q7`）。
3. `[ ]` **臂单关节小步进**：`arm_teleop_node` 按住 numpad，看关节是否按预期方向动、
   有没有把底盘带歪（仿真里的判据是 tilt < 2°）。
4. `[ ]` **力矩与温升**：观察 `/ARM_JOINTS_DATA` 的 `torque` 与两个温度字段；
   仿真里默认姿态保持力矩 1.3~7 N·m，遥操作时 ≤ 11.4 N·m（限幅 100）。
5. `[ ]` **`ee_goal` 一致性**：`ros2 topic echo --once /ARM_TELEOP_STATE` 在默认姿态下
   应为 `(0.3492, 0, 0.4327, ...)`（root 系）。若拿到 `(0.1092, ...)` 说明跑的是旧版
   `arm_controller`（`DEFECT_LOG_zh.md` DEF-020）。
6. `[ ]` 臂 + 腿同跑：进 RL 后按住 numpad 移动臂，确认底盘姿态不失控
   （仿真 `--mode arm_move` 的结果是 tilt ≤ 1.7°）。

---

## 4. 阶段 C：测量与回填

| 项 | 怎么测 | 回填到哪 |
|---|---|---|
| 端到端时延（IMU/关节 → 策略 → 命令） | 打时间戳记录，统计均值/95 分位 | `TODO_zh.md` P2-3；契约文档第 5 节 |
| 实际控制周期抖动 | 策略 tick 间隔直方图 | 同上 |
| 站立高度（对比训练的 0.513） | 用腿 FK 或人工量测轮心到机体距离 | `TODO_zh.md` P0-5 的遗留项 |
| 关节标定残差 | 姿态摆正后读 `/JOINTS_DATA` | 本文第 2 节第 1 条 |

---

## 5. 出问题时的回滚顺序

1. `r`（joint damping）→ 或者硬件急停；
2. 退出策略状态：按 `x`（lie down）；
3. 如果怀疑参数改坏了：`git checkout <上一个 tag/commit>` 重新编译；
4. 真机参数（标定表、超时阈值）改过的话，在 `DEFECT_LOG_zh.md` 里加一条并写明
   "真机实测值"。

---

## 6. 本仓库**没有**做过的验证（别当成已验证）

* `[ ]` 真机腿部标定的实测核对（本文第 2 节第 1 条）；
* `[ ]` `arm_real_adapter` 的实机联调（只在话题层验证过）；
* `[ ]` 真机时延/抖动测量；
* `[ ]` 真机上的力矩/速度限幅是否与训练一致（仿真侧由 MJCF `ctrlrange` 保证，
  真机由固件保证 —— 两边都要核）；
* `[ ]` 真机急停链路（软件 `r` 与硬件按钮）。
