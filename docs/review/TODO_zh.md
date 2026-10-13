# 总待办清单（TODO）—— 按优先级

**文档职责**：这是**唯一**的"未完成任务"清单。已完成的事情在 `DONE_zh.md`，
每个缺陷/特性的来龙去脉在 `DEFECT_LOG_zh.md`，接口契约在
`docs/sim2sim_layout_contract_zh.md`，流程规范在 `WORKFLOW_zh.md`。

**维护约定**：见 `templates/DOC_TEMPLATE_zh.md`。每次更新在下面"更新记录"加一行；
条目用 `- [ ]`/`- [x]`；完成即迁到 `DONE_zh.md` 并留一行"→ 已迁至"。

## 更新记录

| 日期 | 更新内容 | 相关 commit / 分支 |
|---|---|---|
| 2026-09-20 | 初版：把部署仓库现状拆成 P0~P3；确认"`main` 上跑的是旧 checkpoint"是首要问题 | `docs/review-spec` |
| 2026-09-20 | 新增 P0-8（进场已达标后的下一步）；P1-1 部分完成（DEF-012/013 已修）；P3-1/P3-2 部分完成（遥测 + 一键冒烟已可用） | `fix/sim2sim-bringup` |
| 2026-09-20 | **P0-1 / P0-2 / P0-3 完成**（接口切到 83/700/16；原生序由训练侧探针判定；`actions` 观测=实际下发）；新增 DEF-014/015；P1-1 再加一条（armature） | `feat/policy-layout-v2` |
| 2026-09-20 | **P0-5 / P0-6 完成、DEF-011 完成**（高度区间收敛、臂增益 300/20、armature 真正生效）；L3 增加 `--mode arm`；P1-1 的 armature 一条清掉 | `fix/arm-gains-armature-height` |
| 2026-09-20 | **P0-7 完成**（安全接管 + `--mode push` 用例）；新增 DEF-016/017/018，其中 **DEF-018（连续跑偶发摔倒）未修 → 新 P0-9** | `fix/safety-takeover` |
| 2026-09-20 | **P0-4 完成**（`ee_goal` 默认值 + `arm_controller` 发布的坐标系都修到 root 系）；DEF-018 定位为**策略侧**边缘稳定性并交接训练侧；新增 DEF-020/021 与 `--mode arm_move` | `fix/entry-transient` / `fix/ee-goal-frame-arm-node` |
| 2026-09-20 | **P1-1 完成**；**P3-1/P3-2/P3-3/P3-4/P1-5 完成**；清理 TODO 占位条目（P0 全部清空） | `main` |
| 2026-09-20 | **P1-2 完成**（轮子速度伺服阶跃验收）；新增 `--mode wheel_step`；`arm_move` 增加末端位移判据；仿真遥测增加末端位姿与轮速指令列；**DEF-022 修键盘目标限速**，IK 精度实测 0.2 mm | `feat/actuator-accuracy-tests` |
| 2026-10-09 | 同步仿真新代码：臂增益 300/20 → **300/8**、夹爪 4000/200 → **286/5**、臂速度上限 **3 rad/s**；默认策略切到多地形 `cloud_slowvx20k`；新增 **P1-6**（容器内补跑 L1/L3）与 **P2-6**（真机臂速度上限 3 rad/s 落地） | `sync/2026-10-09-sim-delta` |
| 2026-10-12 | **DEF-044 修复完成**（VR 臂跟随：柱坐标 + 手腕关节偏移映射、IK 收敛检查、零空间姿态正则、关节限速默认 3 rad/s）、新增回归用例 `tests/vr_arm_chain_check.py`；**P2-7**（`arm` 档判据，DEF-033）仍未动 | `fix/vr-arm-ik-reachability` |
| 2026-10-13 | 窗口可视化验收（12 个具名位姿指令，跟踪 ~5 mm / 0.8°，`scripts/arm_pose_demo.py` 新增）；补修半径下限钳住标定锚点；新增 **P2-8**（跑用例前先清残留 + 断言"话题只有一个发布者"，DEF-045） | `fix/vr-arm-ik-reachability` |
| 2026-10-09 | 全代码走查（`docs/review/CODE_REVIEW_2026-10-09_zh.md`）后新增 **P4 代码质量** 一节（安全层接线、单一常量来源、死代码、CMake 警告、并发约定、单元测试）；默认策略已改回**平地**模型 | 同上 |
| 2026-10-13 | 真机"左前侧摆过温 100 ℃"排查：仿真侧**没有热模型**（温度写死 40/45 ℃），等价量**关节力矩**实测为长期静载（站立状态机 `fl_hipx` +12.0 N·m、RL 站立 +15.1 N·m）⇒ 新增 **DEF-059** 与 **P1-7**；新增离线探针 `tests/joint_torque_report.py` | `investigate/left-hipx-temp-sim2sim` |
| 2026-10-13 | 反问修正：**静载只是必要条件**（同腿 `knee/hipy` 同量级、`76.4 N·m` 是 `effort_limit` 不是热额定）；补做延迟压力测试（0/5/25 ms 都不产生抖动）⇒ 新增 **P1-8**（"为什么偏偏左前侧摆"未闭合，四刀真机取数） | `investigate/left-hipx-temp-sim2sim` |
| 2026-10-13 | 真机取数工具就位并入 P1-7：`scripts/record_joint_data.py`（录）+ `scripts/joint_log_report.py`（四刀分析），命令见 `COMMANDS_SIM2REAL_zh.md` §7 | `investigate/left-hipx-temp-sim2sim` |
| 2026-10-13 | **真机 65 s 数据到手**：保持段**纯静载无抖动**、力矩是仿真的 **2.0~3.7 倍**（`τ = kp × 落后量` 逐个对上）、65 s 内 **3 次过温**（保护→卸力→冷却→再站）；电机 100 ℃ 而 driver 只 53 ℃ ⇒ 绕组。另查明**站立/idle 状态机确实被本仓库改过**（`e5e1917`：厂商两段高度规划 + idle 零增益 → 训练姿态 + 主动保持）⇒ 新增 **DEF-060** 与 **P1-9**（厂商模式 A/B） | `investigate/left-hipx-temp-sim2sim` |

**优先级定义**：P0 = 挡在"sim2sim 能稳定跑"前面；P1 = 决定 sim2sim 与训练的一致性上限；
P2 = sim2real 落地；P3 = 工具与文档。

---

## 0. 现状一句话

接口已经切到新 checkpoint（**83 / 700 / 16**、资产 `M20_Piper_own`），
sim2sim 三档冒烟（`hold` / `rl` / `walk`）全绿：**站得住、按命令走得动**
（walk 后半段 +0.58 m/s，命令 +0.7）。剩下的 P0 是"命令语义 + 安全接管"，
P1 是仿真与训练的一致性（armature 仍未生效 = DEF-011）。

---

## P0 —— 挡在"sim2sim 能稳定跑"前面

> **P0-1 / P0-2 / P0-3 已完成**，见 `DONE_zh.md` 第九节（`feat/policy-layout-v2`）：
> 接口切到 83/700/16 且布局驱动；24 维原生序由训练侧探针实测判定（交错序，
> `wheel=15..18`）并写进 layout + runner，L1 交叉断言；`actions` 观测 = 实际下发的动作。
> 下面保留原来的验收口径，供以后换策略时复用：
>
> * P0-1 验收 = `scripts/check_policy_interface.py` 通过 + L3 零命令 20 s 不摔；
>   （实测：L1 PASS、`hold/rl/walk` 三档 L3 全 PASS，walk 后半段 +0.58 m/s）
> * P0-2 验收 = 顺序写进 `policy_layout.json::joint_order_native` 且与
>   `M20PiperPolicyRunner::NativeOrder()` 一致；
> * P0-3 验收 = 喂回 `actions` 的 16 维与实际下发逐元素相等（现在是同一个变量）。

- [x] **P0-4 复位/进入 RL 时不要喂零命令** → 已迁至 `DONE_zh.md` 第十二/十三节
  （默认值改成 root 系的当前默认位姿；`arm_controller` 从第一拍起就发布状态且坐标系已对齐；
  `--mode arm` 用 dump 验证第一拍 `obs[73:80] = (0.3492, 0, 0.4327)`）
  - 要做什么：进入 `RLControlState` 的那一帧就把 `ee_goal` 设成"当前 EE 位姿（root 系）"、
    `body_pose` 设成"当前实测 height/pitch/roll"，history 整窗用同一帧填满。
  - 依据（为什么）：训练侧 `TODO_zh.md` P1-3 ⑫（reset 后第一帧 `pose_command_b` 全 0）
    与 `docs/deploy_sim2sim_sim2real_zh.md` 第 5 节坑 2。本仓库现在用的是
    `UserCommand` 的硬编码默认值 `(0.1092, 0, 0.3439)`（臂基座坐标系、且是**另一个**
    资产的默认姿态），既不是"当前位姿"也不是 **root 系**。
  - 验收：L3 里从 idle 进 RL 的那一帧，日志打印的 `ee_goal`/`body_pose` 与
    `arm_controller` FK 算出的当前位姿一致（差 < 1e-3）。
  - 预估：0.5 个工作日。

- [~] **P0-5 `body_pose.height` 的度量与取值范围** → 区间已收敛 + 语义写进注释（`fix/arm-gains-armature-height`，见 `DONE_zh.md` 第十节）；剩余：真机上核对我方实测高度与训练默认是否一致（并入 P2-1）
  - 要做什么：把"机身高度命令"从"绝对 `root_z`"改成训练定义
    `height = root_z − mean(四轮 body 的 z) + 0.09`；命令范围收敛到训练区间
    `(0.33, 0.55)`（现在键盘/VR 上限是 `0.60`）。
  - 依据（为什么）：训练侧 `mdp/utils.py::compute_base_height_rel_to_feet`；
    `flat_env_wbc_cfg.py` 的 `body_pose` 课程终值 height (0.33, 0.55)。
    本仓库 `keyboard_interface.hpp:36` 用 `0.513f` 当绝对高度、
    `rl_control_state.hpp` VR 分支 `clamp(..., 0.33f, 0.60f)`。
  - 验收：L3 打印的"实测 height"在默认姿态下 ≈ 0.513（±0.01），
    且把命令改到 0.55/0.60 时不会超出策略训练区间（超出要打警告）。
  - 预估：0.5 个工作日。

- [x] **P0-6 机械臂 PD 增益对齐到本次训练（40/8 → 300/20）** → 已迁至 `DONE_zh.md` 第十节（`fix/arm-gains-armature-height`）
  - ⚠️ **2026-10-09 更新**：训练侧又把臂阻尼 20 → **8**、夹爪改成 **286/5**，本仓库三处常量
    已同步（见 `DONE_zh.md` 第二十三节 / `DEFECT_LOG_zh.md` DEF-029/030）。以下内容保留作历史。
  - 要做什么：`arm_controller.py` 的 `ARM_KP/ARM_KD`、仿真侧"首条命令前的默认保持"
    `ARM_DEFAULT_KP/KD`，统一取本次 run 的 `piper_arm` 执行器配置；
    把它做成单一常量来源，避免三处各写一份。
  - 依据（为什么）：`2026-09-20_00-50-31/params/env.yaml` 的
    `piper_arm.stiffness=300.0 / damping=20`、`piper_gripper=4000/200`。
    当前仓库三处不一致：`arm_controller.py:84` 是 `40/8`、
    `piper_arm_interface.hpp:78` 是 `300/20`、
    `mujoco_simulation_ros2.py:116` 又写了一份 `40/8`。
  - 验收（已达成）：`--mode arm` 下臂最大力矩 4.6 N·m、偏差 0.0153 rad；
    `arm_controller.py` / `mujoco_simulation_ros2.py` / `piper_arm_interface.hpp` 三处一致。

- [x] **P0-7 安全接管阈值与限幅** → 已迁至 `DONE_zh.md` 第十一节

- [~] **P0-9 定位 DEF-018：进 RL 的入口瞬态（约 25% 发散）**
  - 现状：部署侧 4 组对照实验全部排除（软启动长度 / 轮子执行器语义 / 执行器延迟 /
    站立腿增益），仿真实时因子 1.000；结论是**策略在该动力学下的边缘稳定性**，
    已按训练侧 P1-2 的口径交接（见 `DEFECT_LOG_zh.md` DEF-018 的矩阵）。
  - 剩余可做（不阻塞）：① 与训练侧一起看"reset 后前 0.2~0.5 s 的动作分布"；
    ② 用 `--repeat 20` 建立统计基线，改训练后回归对比。
  - 验收：`--mode rl --repeat 12` 失败率 < 10%。

  - 要做什么：按 `DEFECT_LOG_zh.md` DEF-018 的三条候选逐一排除；
    建议先做"给仿真加固定时延（0/1/2 ms）看退化"和"dump 进 RL 前 300 tick 的
    obs/action 与离线复算对照"。
  - 依据：五档连续跑约 1/3 概率 FAIL，单独跑全过 —— 这种"只在连续跑时出现"
    的问题最可能是时序/调度，而不是策略。
  - 验收：五档连续跑 3 轮（15 次）零失败。
  - 预估：0.5~1 个工作日。
  - ⚠️ 在它修掉之前，`tests/run_all.sh` **不能**当合并门槛（P0-8 顺延到它之后）。

- [x] **P0-7 安全接管阈值与限幅** → 已迁至 `DONE_zh.md` 第十一节（`fix/safety-takeover`；
    倾角 0.8 rad + 腿折叠 1.2 rad 两条通路都用 `--mode push` 验证过）
  - 要做什么：① 把 `tests/sim2sim_smoke.py --mode hold` 与 `--mode rl`
    写成 `tests/run_all.sh`，作为合并前的必跑项；② 覆盖"进 RL 前机器人已经
    被撞歪/输入抖动"的起点；③ 让 `--mode walk` 也能给出方向与速度的判据。
  - 依据（为什么）：`DEF-012/013` 都是"策略没错、进场流程错"的典型；
    这类问题的成本极低但会伪装成"策略不行"。
  - 验收：`tests/run_all.sh` 在 main 上全绿；故意把 `control_word` 检查去掉时
    `--mode rl` 必须重新 FAIL。
  - 预估：0.5 个工作日。
---

## P1 —— 决定 sim2sim 与训练的一致性上限

> 已完成的子项：**DEF-012**（初始保持位姿用错机型）、**DEF-013**（把非控制字帧当
> 零增益命令执行）、**DEF-011**（armature 写到 `<motor>` 上被忽略）—— 这三条修完，
> `tests/sim2sim_smoke.py` 的 `hold / rl / walk / arm` 四档全 PASS，
> `check_mjcf_contract.py` 从 2 个 FAIL 变成 0 个。
> 遥测（P3-2）与一键冒烟（P3-1 的一半）也已可用。
> 仍待处理：`timestep` 取舍、执行器延迟、摩擦/恢复系数、`base_link` 显式惯性来源。

- [x] **P1-1 MuJoCo 物理与训练对齐（结构性差异清单）** → 表已写进
    `docs/sim2sim_layout_contract_zh.md` 第 5 节（控制周期/积分步长/延迟/增益/armature/
    限幅/重力/摩擦/恢复系数/求解器/基座惯性/初始位姿/碰撞/地形 逐项定案）。
    要点：接口类参数全部对齐且有断言；物理类参数（积分步长、执行器延迟、摩擦与恢复系数
    随机化、求解器）是结构性差异，已取训练分布内的固定值或留开关。
    `base_link` 显式惯性已核对 = URDF（质量 15.882、主惯量一致）。
    剩余可选：把摩擦/恢复系数做成按 env 随机（要复刻训练随机化时再做）。

- [~] **P1-4 IK 与训练侧 DLS 的一致性**（精度侧已完成：`--mode arm_move` 实测可达目标残差 **0.2 mm**，见 `DONE_zh.md` 第十四节；剩余是与训练侧 `CommandDrivenIKAction` 的逐目标对照）
  - 要做什么：对比 `arm_controller.py` 的 DLS 解与训练侧
    `velocity/mdp/actions.py::CommandDrivenIKAction`（λ=0.01、绝对位姿、
    `gripper_base`、root 系）在若干目标位姿上的解，记录关节角差。
  - 依据（为什么）：臂虽不占动作维度，但它的位姿进入 `joint_pos` 观测与
    `ee_goal`，解不一致会持续给策略"错误的现实"。
  - 验收：随机 20 个目标上 `max |Δq| < 0.02 rad`（或在文档里写清偏差来源）。
  - 预估：1 个工作日。

- [x] **P1-5 清理旧 checkpoint 的调试开关** → 已随新 runner 一起删除（`grep M20_LAST_ACTION|M20_IK_FEEDBACK|M20_FREEZE|M20_ARM_DEFAULT_TRAINED|M20_ZERO_CMD_WHEEL_BRAKE` 在 `src/` 下已无命中）；`M20_ARM_TRAINED_DEFAULT` 也已移除
  - 要做什么：逐条判定 `WORKFLOW_zh.md` 第 6.1 节里"旧 checkpoint 期"引入的开关
    （`M20_LAST_ACTION_MODE` / `M20_IK_FEEDBACK` / `M20_FREEZE_*` /
    `M20_ARM_DEFAULT_TRAINED` / `M20_ZERO_CMD_WHEEL_BRAKE`）在新 checkpoint 下是否还需要，
    不需要的连同代码一起删。
  - 依据（为什么）：新 checkpoint 动作是 16 维、臂默认角就是 `0.5/-0.5`，
    这些开关的适用前提已经不存在。
  - 验收：删掉后 L1 + L3 仍通过；开关登记表同步更新。
  - 预估：0.5 个工作日。

- [x] **P1-6 在容器里补跑 L1 + L3（2026-10-09 新增）** → 已完成（容器 `m20_piper_ros`）
  - 结果：L0 编译 ✅；**L1 PASS**（ONNX↔TorchScript 5.96e-07）；**L3' 10 PASS / 0 FAIL**；
    L2 ✅；hold / wheel_step / no_arm / arm_only / arm_mixed / push 全 PASS；
    rl / walk / arm_move 重复 6 次失败率 0~17%（< 1/3）。完整数字见
    `docs/deploy_code_delta_2026-10-09_zh.md` §7。
  - 遗留：`arm` 档偶发触发倾角接管（新策略 9/12、旧平地策略 8/12 通过）——属训练侧
    DEF-018，不是本次改动引入；`run_all.sh` 对 `arm` 用"0 失败"判据与 DEF-018 不自洽。
    是否把 `arm` 也纳入"失败率 ≤ 1/3"见 **P2-7**。

- [ ] **P1-7 真机"左前侧摆关节过温 100 ℃"闭环（2026-10-13 新增，DEF-059）**
  - 现状：仿真侧**没有热模型**（`mujoco_simulation_ros2.py:836-837/874-875` 把温度写死
    40/45 ℃ ⇒ 温度那条安全接管路径在 sim2sim 不可达）。能观察的等价量是**力矩**：
    站立状态机 `fl_hipx ≈ +12.0 N·m`、RL 站立零命令 `+15.1 N·m`（四个侧摆里最大，
    是 `fr_hipx` 的 2.3 倍），且 150 s 内完全稳态 ⇒ 长时间"顶力矩"、角速度≈0、
    铜损 ∝ τ² 持续加热。
  - 要做什么（三选一或都做，**先做 (1)**）：
    1. **真机核对**：`ros2 topic echo /JOINTS_DATA` 记 16 关节 `torque` + `motion_temp`
       的长曲线，确认 `fl_hipx` 是"纯静载"还是"静载 + 抖动/额外摩擦"（后者仿真测不到，
       要查电流环/减速箱/装配/线束）；
    2. 给仿真加**一阶热模型**（`dT/dt = (T_amb−T)/τ_th + k·τ²`，用 `tau`/`dq` 算铜损），
       让"过温 ⇒ 自动接管 ⇒ 倒 ⇒ 冷却恢复"这条路径能被 `tests/` 回归；
    3. 若确认是静载：从**姿态**降载（站高 / 髋外展角 / 前后配重）——属训练侧改动，
       必须与训练侧同步（会改策略分布）。
  - 依据（为什么）：真机现象已复现（过温 ⇒ 趴下 ⇒ ~10 s 后恢复，与
    `safe_controller.hpp:202/396-399` + `joint_damping_state.hpp` 的链路一致），
    但现在**仿真无法验证**，且分不清"固有静载"与"真机额外损耗"。
  - 验收：真机上 `fl_hipx` 的 `torque × temperature` 曲线能定量对上本节数字（±20%），
    或说明多出来的损耗来源；若加 (2)，则 sim2sim 里能复现"过温 ⇒ 接管"。
  - 预估：真机核对 0.5 个工作日；一阶热模型 0.5 个工作日。
  - 备注：本轮的三次 RL 用例共用一份确定性策略，**不能**当成独立样本
    （"重的一侧是否永远是左前"待真机确认）。
  - **工具已就位**（2026-10-13 补）：真机 `scripts/record_joint_data.py`
    （200 Hz 录 16 关节 `pos/vel/tau/motion_temp/driver_temp` + IMU）→ `scp` 取回 →
    `scripts/joint_log_report.py`（四刀报告：仿真参考值对照 / 两个温度分开看 + 过温时序 /
    谁最热 vs 谁力矩最大 + 左右镜像 / `--spectrum`）。命令见
    `src/M20_sdk_deploy/COMMANDS_SIM2REAL_zh.md` **§7**；
    两个脚本已在仿真 `/JOINTS_DATA` 上自测通过（含人造过温数据的过温事件检测）。

- [ ] **P1-8 "为什么偏偏是左前侧摆"未闭合（2026-10-13 反问后新增，DEF-059）**
  - 立论：`fl_hipx ≈ 15 N·m` **不是离群值**（同腿 `knee 21.4` / `hipy 16.3` 同量级，
    后腿 `hr_hipx 13.6`；`76.4 N·m` 是训练 `effort_limit` 而不是热额定），
    所以"静载"只是**必要条件**，选不出具体关节。
  - 已排除的：仿真里**没有**抖振（>5 Hz < 0.001 N·m）；把执行器延迟开到 5 / 25 ms
    也不产生抖振（`fl_hipx` std 0.139 → 0.115 → 0.111 N·m）
    ⇒ "延迟 ⇒ 极限环 ⇒ 发热"在这份模型里不成立，**仿真测不了**这类真机现象。
  - 要做什么：四刀（详见 DEF-059 §2 末）——
    ① 真机 16 关节静止 `torque` vs 仿真表；
    ② `motion_temp` 与 `driver_temp` 分开看；
    ③ 站立 10 分钟记全部 16 关节温度曲线（区分"整条腿贴上限"与"单关节异常"）；
    ④ 真机 `torque` 的 200~1000 Hz 记录做频谱（用 `tests/joint_torque_report.py` 的口径）。
  - 验收：能明确回答"是负载（含标定）还是该关节自身（电机/驱动器/线束）"，
    并给出对应处置（改标定 / 换件 / 改姿态与训练侧同步）。
  - 预估：真机取数 0.5 个工作日；取决于①②③④的结果再决定后续。

- [ ] **P1-9 站姿/保持改动 vs 厂商原版 的 A/B（2026-10-13 操作员提问后新增，DEF-060）**
  - 事实：**站立/idle 状态机是本仓库改过的**（`e5e1917`，2026-09-04）：厂商版是
    "站立 = 两段高度规划（0.12 m 蹲姿 → 0.48 m）；**idle = 零增益不使力**"，
    现版是"**训练默认姿态（hipx 0 / hipy ∓0.6 / knee ±1.0）+ 永久主动保持**；
    idle 也主动保持（kp 80/2）"。同一份文件在真机上编 ⇒ **真机行为也变了**。
  - 要做什么（**先做 1，它可能直接结案**）：
    1. **厂商模式 A/B**：不跑我们的 deploy，用机器人自带模式（不进 SDK 模式，让 M20 自己的
       控制器撑腿）站 10 分钟，录 `motion_temp`。若**同样**冲到 100 ℃ ⇒ 是机器人/姿态/负载
       本身（我们只是把关节推进了这个高力矩工况）；若**不冲** ⇒ 我们的 idle 主动保持 /
       站立目标改动是可疑点，回头做 2。
    2. 若 1 说明是我们这边：把 `standup` 改回厂商的两段高度规划、`idle` 改回零增益
       （可用环境变量做成开关，默认保持现行为），跑同一份 65 s 录数据对比
       `保持段 τ` 与温升速率。
    3. 顺带量两个数：**整车重量**、**站立时前轮中心距（轮距）** —— 用来解释"真机为什么需要
       2 倍以上力矩"（仿真模型总重 384 N ≈ 39 kg；前轮距 ≈0.45 m）。
    4. 核对 `M20Interface` 的 `init_pos_offset` 里 hipx 那两列（−25°/25°，见 P2-1）：
       现在无法从数据判断它是否标定正确（`τ = kp × 落后量` 两侧都用同一张表，误差会互相抵消）。
  - 依据：真机实测（DEF-059 §5）表明"保持段是 28 N·m 的纯静载、无抖动、2 倍于仿真"，
    而 `DEF-060` 恰好解释了"为什么真机一直在顶力矩"（idle 也主动保持）。
  - 验收：能回答"过温是机器人固有工况，还是我们改的状态机引入的"；若是后者，给出改回/开关方案。
  - 预估：A/B 半个工作日（不需要动代码）。
  - **开关已就位（2026-10-13）**：`M20_STANDUP_VENDOR=1`（厂商两段式站立）、
    `M20_IDLE_ZERO_GAIN=1`（厂商 idle 零增益）——**只改 `standup_state.hpp` / `idle_state.hpp`，
    默认行为不变，未碰 `rl_control_state.hpp` / `run_policy` / 安全层**；命令见
    `COMMANDS_SIM2REAL_zh.md` **§8**。仿真实测：厂商档站位高 4.5 cm、侧摆力矩差 1.8%、
    膝盖轻 26% ⇒ 站立改动不是那 2~3.7 倍的原因，但膝盖那 26% 值得真机验一下。
  - **A/B 已跑完（2026-10-14 记）**：档 A（训练姿态）vs 档 B（厂商姿态）真机各一份
    （DEF-059 §7）：侧摆 **29.0 → 10.7 N·m（−63%）**、膝盖 22→16（−27%）、**hipy 变重**、
    过温从"几十秒 3 次"变成"冷起 ~4.4 min 一次"⇒ **延长但没解决**，最后跳到膝盖上。
    且仿真对姿态**不敏感**（扫站高 0.42/0.48/0.54 → 侧摆 11.0/10.9/10.3）与真机**相反**
    ⇒ 仿真不能用来外推站姿。
  - **下一步（新增两个扫描旋钮）**：`M20_STANDUP_HEIGHT_M=<m>`（配 vendor 档，扫站高）
    与 `M20_STANDUP_HIPX_DEG=<deg>`（镜像偏置，扫侧摆目标）——**真机上一个一个扫**，
    每次记"稳态 τ + 升温速率"，判据 = **最热的那个关节**。命令见 `COMMANDS_SIM2REAL_zh.md` §8.3。
    目标：找到让"最热关节"明显低于今天的最优/次优站姿；如果扫遍仍然过温 ⇒
    该工况超出电机连续能力，转为找厂商要**连续额定力矩/热阻**并谈运营口径（别长时间挂着站）。

---

## P2 —— sim2real 落地

- [ ] **P2-4 无臂部署上线确认（新增）**
  - 现状：代码/工具层已完成（`DEF-023`：接口默认姿态兜底 + `HasData()` 警告 +
    `arm_stub_node.py` 假臂节点（含**跟随模式=假 CAN 驱动**）+ 仿真 `M20_SIM_NO_ARM_DATA`
    + `scripts/run_sim2sim.sh {full|legs|mixed}` 三套配置 + `--mode no_arm` / `--mode arm_mixed` 用例）。
  - 待做：真机上**只装腿**跑一次完整流程（起 `arm_stub_node.py`，进 RL、走、
    急停），确认没有臂时策略行为与"臂装好且静止"一致。
  - 验收：真机无臂行走不触发 `[TAKEOVER!]`；`ros2 topic echo /ARM_TELEOP_STATE`
    为默认位姿；对比"有臂静止"跑的日志无差异（除臂话题）。
  - 工具：先跑 `scripts/preflight_real_deploy.py`（会检查 SIM2SIM 编译选项、
    策略是否需要臂、话题发布者），再按输出执行。

- [ ] **P2-5 支持"观测里不含机械臂"的策略（待用户提供 layout 再动手）**
  - 现状：`M20PiperPolicyRunner::LoadLayout()` 硬断言 `obs = 3+3+3+24+24+action+7+3`，
    所以一个"真·无臂策略"（例如 16 关节、无 `ee_goal`）现在会被直接拒绝加载
    （报 `Policy dim mismatch`，不会静默算错）。
  - 要做什么：把关节表/是否含 `ee_goal` 改成**从 `policy_layout.json` 推导**，
    并让 `RLControlState` 在无臂布局下不创建 `PiperArmInterface`。
  - 依据/前置：需要那个策略的 `policy_layout.json`（关节表、是否有 `ee_goal`），
    因为观测构成必须与训练一致。
  - 验收：用一份合成 ONNX（按无臂布局的维度生成）跑通 L1 + 端到端；现有
    83/700/16 路径回归不退化。

- [ ] **P2-1 真机腿部标定链核对**
  - 要做什么：核对 `M20Interface`（真机 dir/offset 标定、启动期 ±360° 多圈补偿）
    与仿真 `M20SimInterface`（恒等标定）的差异清单，确认"仿真里跑通的策略
    换到真机只需要换接口"。
  - 依据（为什么）：`m20_sim_interface.hpp` 的注释已经指出真机接口会在启动期
    调整偏移，若机器人启动时漂移会污染坐标系。
  - 验收：出一张"仿真 vs 真机"接口差异表 + 上线检查单。
  - 预估：1 个工作日。

- [~] **P2-2 机械臂真机传输**（软件侧已完成：adapter 双向 + FK 发布 ee_goal + L2 验收；剩余是**真机联调**）
  - 要做什么：把 `arm_real_adapter.py`（`agx_arm_ros` 的
    `/control/joint_states` ↔ `/ARM_JOINTS_DATA`）跑通并验收：
    关节名映射、夹爪宽度 = `q6 − q7` 的符号、`fast_mode` 下的伺服频率。
  - 依据（为什么）：commit `b884d90` 已写好适配层，但只做过话题级验证。
  - 验收：真机上单关节小步进跟踪误差有记录；急停可用。
  - 预估：1 个工作日（含真机时间）。

- [ ] **P2-3 时延与同步测量**
  - 要做什么：测"IMU/关节反馈到策略、策略到关节命令"的端到端时延与抖动，
    对照训练 `DelayedPD` 的 0~25 ms；必要时在仿真里加同样延迟做退化测试。
  - 验收：给出时延分布（均值/95 分位）+ 加延迟后的 L3 结果。
  - 预估：1 个工作日。

- [ ] **P2-6 臂关节速度上限 3 rad/s 的真机落地（2026-10-09 新增）**
  - 背景：训练侧新增约束 DEF-065 —— 臂 joint1~6 速度上限按 **3 rad/s**
    （旧 USD 是 5）；仿真已由 `align_joint_velocity_limits` 对齐。真机上这条由
    `agx_arm_ctrl` / Piper 驱动器限制，**不在本仓库代码里**，别按 5 放开。
  - 要做什么：在 Jetson 的 agx 驱动配置里把 arm joint1~6 的速度上限设成 3 rad/s
    （joint6 本来就是 3），然后实测一条小步进/大范围的关节 `|qd|`。
  - 验收：真机 `|qd|` p99 ≤ 3.0 rad/s；臂跟踪不因此变差（对比改前日志）。
  - 参考：`docs/deploy_code_delta_2026-10-09_zh.md` §1/§2。

- [ ] **P2-7 `run_all.sh` 的 `arm` 档判据与 DEF-018 对齐（2026-10-09 新增）**
  - 现状：`tests/run_all.sh` 把 `arm` 当"0 失败"档单跑，但 `arm` 会进 RL、同样吃
    DEF-018 的入口瞬态 ⇒ 以 ~25~33% 概率整体报红（实测：新策略 9/12、旧平地 8/12）。
    真正按失败率判的只有 `rl / walk / arm_move / no_arm / arm_mixed`
    （`tests/sim2sim_smoke.py` 容忍列表 = `rl / walk / arm_move / no_arm`）。
  - 建议改法（一行）：`run_all.sh` 的 `if` 条件加 `arm`；`sim2sim_smoke.py` 的
    `limit` 元组加 `"arm"`，即用同样的"失败率 ≤ 1/2"。
  - 依据：`DEFECT_LOG_zh.md` DEF-018（已定位为策略侧，部署侧对照实验已排除）。
  - 验收：`MODES="arm" SKIP_BUILD=1 bash tests/run_all.sh` 在 6 次里不再因偶发坍塌报红。

- [ ] **P2-8 跑用例/上机前的"残留进程 + 发布者唯一性"检查（2026-10-13 新增）**
  - 现状：本次演示被一个 **11 小时前残留的 `arm_stub_node.py`** 干扰——它和
    `arm_controller` 同时发 `/ARM_JOINTS_DATA` 与 `/ARM_TELEOP_STATE`（常量默认位姿），
    于是 IK 的反馈种子与策略的 `ee_goal` 观测都被掺入常量，表现是"目标读数不变、
    臂不听使唤、安全网拒发计数猛涨"（DEF-045）。上真机同理：多起一个假臂节点或
    `arm_real_adapter` 就会踩到。
  - 建议改法：① `tests/run_all.sh` 开跑前调用 `tests/sim2sim_smoke.py` 里已有的
    `leftover_processes()` / `force_cleanup()`（把"本仓库节点"清干净）；
    ② 启动后加一条前置断言：`/ARM_JOINTS_DATA`、`/ARM_TELEOP_STATE`、`/ARM_TELEOP`、
    `/VR_TELEOP` **各只有一个发布者**（`scripts/check_vr_arm_chain.py` 已经在打印
    发布者数量，可直接复用其判定）。
  - 依据：`DEFECT_LOG_zh.md` DEF-045（与 DEF-054 2026-10-10 同类的"双发布者"坑）。
  - 验收：故意留一个 `arm_stub_node.py` 在跑时，`run_all.sh` 在启动阶段就报红并指出
    是哪个话题、哪个进程。

- [x] **P2-9 头显页面离线化后的真机复核（2026-10-13 新增，DEF-058）** → 已完成：
  操作员 2026-10-13 实测——把 `aframe.min.js` 放到 `web-ui/` 后头显页面恢复正常，
  **可正常遥操**。详见 `DONE_zh.md` 第二十五节、`DEFECT_LOG_zh.md` DEF-058。
  - 遗留（非阻塞）：`aframe.min.js` 约 1.4 MB 的第三方文件**建议随仓库提交**
    （与 `web-ui/` 其它文件一致），否则换机器/重新 clone 后要重跑一次
    `fetch_aframe.sh`——"要下载什么"总表见 `docs/OFFLINE_ASSETS_zh.md`。

---

## P3 —— 工具与文档

- [x] **P3-1 `tests/` 一键验收** → 已迁至 `tests/run_all.sh`（L0+L1+L3'+7 档 L3，`DONE_zh.md` 第十一~十四节）
  - 要做什么：`tests/run_all.sh` 依次跑 L0 → L1 → L3（无头 + 数值判据），
    输出 PASS/FAIL 与关键数字；CI 或人工合并前只跑这一条命令。
  - 验收：故意引入一个错误（改错关节顺序）时脚本必须 FAIL。

- [x] **P3-2 遥测落盘** → 已完成：`M20_SIM_TELEMETRY` 写出 `t/wall/base 位姿/rpy/omega/q[24]/dq[24]/tau[24]/轮 z/轮接触力/末端位姿/轮速指令`，200 Hz（`DONE_zh.md` 第八/十四节）
  - 要做什么：仿真侧提供 `M20_SIM_TELEMETRY=<path>`，按 200 Hz 写 CSV：
    `t, base_pos, base_quat, rpy, omega, q[24], dq[24], tau[24], 轮接触力`，
    供 L3 判据与离线画图使用（现在只能靠 `[JVEL-SIM]` 打印肉眼判断）。
  - 验收：CSV 行数 = 时长 × 200（±1%），能直接算出高度/倾角曲线。

- [x] **P3-3 清理过时文档** → 已完成：`docs/M20_Piper_deploy_6commits_zh.md` 头部加历史归档横幅、`docs/README_zh.md` 的过时描述改掉、悬空引用指向现行文档
  - 要做什么：`docs/M20_Piper_deploy_6commits_zh.md` 描述的是早期 6 个提交
    （`9095957`~`2fcab7c`），与 `main`（领先 `origin/main` 21 个提交）已经不符；
    按新规范改写成"历史归档"，或并入 `DONE_zh.md` 并在原处留指针。
  - 验收：`rg "M20_Piper_deploy_6commits" docs/ README.md` 的引用都指到现行文档。

- [x] **P3-4 把策略文件纳入版本管理的方式定下来** → 已迁至 `src/M20_sdk_deploy/policy/README.md`（一个目录=一次训练的部署产物；命名/换策略 6 步/硬性约束都在里面）+ `DONE_zh.md` 第十五节
  - 要做什么：现在 `policy/*.onnx` 是直接提交进仓库的（旧 checkpoint）。
    需要定规则：新策略放哪、命名规则（含 run 与 checkpoint 版本）、
    是否连 `policy_layout.json` 一起提交、如何在部署机上换策略。
  - 验收：`README.md` 里有"换策略"的标准步骤，且 runner 会在缺文件时打印该步骤。

---

## P4 —— 代码质量（2026-10-09 全代码走查新增）

> 依据：`docs/review/CODE_REVIEW_2026-10-09_zh.md`（每条都有 `文件:行` 与影响面）。
> 排序按"低风险 → 高收益"。

- [ ] **P4-1 删死代码（零行为变化）**
  - 内容：`run_policy/m20_policy_runner.hpp` + `rl_control_state.hpp:44` 的 `m20_policy_`（~265 行）、
    `interface/user_command/keyboard_interface_sim.hpp`（283 行，且与在用键盘接口**同名类**）、
    `qw_state_machine.hpp:33 car_move_controller_`、`main.cpp:3-9` 的 `USE_SIMULATION`+`backward.hpp`
    死分支（`backward.hpp` 仓库里根本没有）、`Sign()`/`GetAbsPath()`/gait/msfb error setters、
    `control_parameters.h` 里 ~10 个没人用的成员、`standup_state` 里那对**从未调用**的
    `GetHipY/KneePosByHeight` 与 `default_pos`/`init_hipx_pos_`/`set_wheel_kd_`。
  - 验收：L0 编译 + `SKIP_BUILD=1 MODES="hold rl" bash tests/run_all.sh` 通过。

- [ ] **P4-2 安全层接线 + 时间单位统一**（走查 §1.1~§1.3）
  - 内容：`SafeController::Run()` 接上 `IsBatteryNormal()` / `IsUserCommandNormal()`；
    决定 `imu_update_overtime` / `joint_update_error` / 电池·CPU 温度三个"永远为 0"的标志
    是**实现**还是**删除**；把 `current_time_`（秒）与 `> 1000.0`（ms 语义）统一；
    把 `safe_control_mode` 的"模式/错误码"重载拆开或改注释。
  - 验收：仿真里用假驱动状态/假电量/停止发 `/JOINTS_DATA` 三种注入，确认能进
    `joint_damping` 且日志给出原因。

- [ ] **P4-3 修 `liedown_state.hpp` 的限位夹取 bug**（走查 §1.6）
  - 内容：`:48`/`:61` 的 `LimitNumber(...)` 丢掉返回值（standup 里同一份代码是接的），
    导致趴下时膝目标 3.02 rad 超出上限 2.758 rad、硬顶限位。
  - 验收：`--mode` 新增或用现有用例走一遍 `x`（趴下），确认膝角不超过限位。

- [ ] **P4-4 单一常量来源**（走查 §3.1）
  - 内容：默认站姿（8 处）、默认 EE 位姿（6 处）、臂/夹爪增益（4 处）抽成
    `deploy_contract.json`（+ 生成的 `.h`），C++/Python 都从它读。
  - 验收：`check_mjcf_contract.py` 与 `check_policy_interface.py` 改为读契约文件；
    故意改错契约时 L1/L3' 必须 FAIL。

- [ ] **P4-5 打开编译警告**（走查 §5）
  - 内容：去掉 `add_definitions(-w)`，改 `-Wall -Wextra` 并逐条消；顺手合并
    `CMakeLists.txt` 里两个完全相同的 `if/else` 分支、把 `file(GLOB_RECURSE)` 换成显式列表。
  - 验收：全量重编无新增 warning（或每条都有明确豁免理由）。

- [ ] **P4-6 策略目录解析口径统一 + 安装**（走查 §1.4/§5）
  - 内容：`M20_POLICY_DIR` 与默认目录用同一基准解析（或只接受绝对路径）；
    `install(DIRECTORY policy ...)` 到 `share/`，按可执行文件位置解析，
    让"装到别的机器/移动目录"也能跑。
  - 验收：把 `install/` 整体拷到临时目录，在**没有源码树**的位置起 `rl_deploy` 能加载策略。

- [ ] **P4-7 并发约定写清 / 加固**（走查 §3.3）
  - 内容：`common_types.h` 的 `UserCommand` 上加"字段所有权表"注释（谁写谁读）；
    或把 `body_*`/速度字段改成原子/双缓冲。
  - 验收：至少注释落地；若能加 TSAN 跑一次 `hold` 档无告警更好。

- [ ] **P4-8 单元测试 + 把 `command_sweep` 挂进 `run_all.sh`**（走查 §2）
  - 内容：给纯函数补测试（`GetCubicSplinePos`、`RpyToRm`、`NormalizeAngle`、
    `fk_pose/mat_to_quat`）；`run_all.sh` 增加 `command_sweep` 一档（短冒烟）。
  - 验收：`bash tests/run_all.sh` 里能看到 `command_sweep` 的结果行。

- [ ] **P4-9 （训练侧）查 vy/wz 方向不对称**（走查 §7.1，见验收报告 §5）
  - 现象：单命令阶跃下，平地模型 `wz -0.5` 只跟到 5%、多地形模型 `vy +0.5` 只跟到 9%，
    而各自的反方向都正常 ⇒ 怀疑左右侧轮子符号/增益或训练侧 `yaw_rate` 符号约定。
  - 验收：训练仓库里用同一 MJCF 跑 `vy/wz = ±0.3/±0.7` 探针，给出左右对称性结论。
