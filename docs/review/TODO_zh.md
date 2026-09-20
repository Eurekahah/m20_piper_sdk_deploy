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

**优先级定义**：P0 = 挡在"sim2sim 能稳定跑"前面；P1 = 决定 sim2sim 与训练的一致性上限；
P2 = sim2real 落地；P3 = 工具与文档。

---

## 0. 现状一句话

`main`（`40744b5`）能编译、能起 sim2sim，但**整条链喂给策略的接口是旧 checkpoint 的**
（`policy_obs 86 / history 770 / action 23`，训练在 `M20_adjusted` 资产上）；
最新训练 run `logs/rsl_rl/history_adaptation/2026-09-20_00-50-31`
是 **`83 / 700 / 16`**、资产是 `M20_Piper_own`。这是"部署效果欠佳"的第一嫌疑，
必须先把接口换过来，再谈策略本身。

---

## P0 —— 挡在"sim2sim 能稳定跑"前面

- [ ] **P0-1 策略接口层切到新 checkpoint（83 / 700 / 16）**
  - 要做什么：把 `M20PiperPolicyRunner` 从"写死 86/770/23"改成**读
    `policy_layout.json`** 驱动：`policy_obs_dim` / `history_single_step_dim` /
    `history_length` / `action_dim` 全部来自布局文件并与 ONNX 的输入输出形状交叉断言；
    按新布局重写观测组装（`joint_pos` / `joint_vel` 都是 **24 维原生序**，
    `joint_pos` 只把 4 个轮子列置零）、history 每步 **70 维**、动作 **16 维**
    （12 腿位置 + 4 轮速度，**没有 `ee_ik` 槽位**）。
  - 依据（为什么）：`docs/sim2sim_layout_contract_zh.md` 第 2 节逐项核对结果；
    训练侧 `logs/.../2026-09-20_00-50-31/params/env.yaml` 的 `observations.policy`
    与 `exported_deploy/policy_layout.json`。
  - 验收：`check_policy_interface.py` 通过；L3 的"零位移命令"步骤 20 s 不摔倒。
  - 预估：1 个工作日（含测试）。

- [ ] **P0-2 确认 `M20_Piper_own` 的 articulation 原生关节顺序（当前是 `(推断)`）**
  - 要做什么：把 24 维原生序做成**可切换**（布局文件里的 `joint_order` 字段 + 环境变量
    覆盖），然后在 sim2sim 里 A/B 两种候选序：
    ①训练侧文档/DEF-021 实测的**交错序**（`hipx×4, arm1, hipy×4, arm2, knee×4, arm3,
    wheel×4, arm4-6, gripper×2`）；②按关节类型分组的**分组序**（`hipx×4, hipy×4,
    knee×4, wheel×4, arm×6, gripper×2`）。
  - 依据（为什么）：支持交错序的证据是训练仓库 `DEF-021` / `docs/deploy_sim2sim_sim2real_zh.md`
    第 4 节（在 `M20_Piper_own` 上跑 `probe_deploy_layout.py` 的实测输出，
    "原生关节序 wheel=15..18"）；支持分组序的证据是**另一个资产**
    （`M20_adjusted`）的 `joint_torque_log_flat.npz` 里 `robot.find_joints(".*")` 的
    `joint_names` 实测值（该资产夹爪叫 `arm_joint7/8`）。两者资产不同，不构成矛盾，
    但**本仓库没有在 `M20_Piper_own` 上实测过**，所以必须 A/B。
  - 验收：两条候选各跑 L3 的"零位移命令"，把 20 s 内的高度/倾角曲线记录下来，
    差异明确的那条写进契约文档并删掉另一条。
  - 预估：0.5 个工作日。

- [ ] **P0-3 `last_action` 与"实际下发的动作"必须一致**
  - 要做什么：确定部署侧的限幅策略后，把**限幅后**的动作喂回 `actions` 观测
    （训练语义：`env.action_manager.action` = 原始输出被 `clip_actions=100` 截断；
    部署侧若再加安全限幅，就必须把限幅后的值喂回去）。同时把"轮速目标 ±15 rad/s"
    这类部署专有限幅改成**可配置**并登记。
  - 依据（为什么）：训练侧 `DEFECT_LOG_zh.md` DEF-008；本仓库
    `m20_piper_policy_runner.hpp` 里 `last_action_mode` 的注释已经踩过一次
    （`processed` 会让机器人 3 s 内翻倒）。
  - 验收：L1 打印"喂回的 16 维"与"实际下发"逐元素相等；L3 零位移命令稳定。
  - 预估：0.5 个工作日。

- [ ] **P0-4 复位/进入 RL 时**不要**喂零命令**
  - 要做什么：进入 `RLControlState` 的那一帧就把 `ee_goal` 设成"当前 EE 位姿（root 系）"、
    `body_pose` 设成"当前实测 height/pitch/roll"，history 整窗用同一帧填满。
  - 依据（为什么）：训练侧 `TODO_zh.md` P1-3 ⑫（reset 后第一帧 `pose_command_b` 全 0）
    与 `docs/deploy_sim2sim_sim2real_zh.md` 第 5 节坑 2。本仓库现在用的是
    `UserCommand` 的硬编码默认值 `(0.1092, 0, 0.3439)`（臂基座坐标系、且是**另一个**
    资产的默认姿态），既不是"当前位姿"也不是 **root 系**。
  - 验收：L3 里从 idle 进 RL 的那一帧，日志打印的 `ee_goal`/`body_pose` 与
    `arm_controller` FK 算出的当前位姿一致（差 < 1e-3）。
  - 预估：0.5 个工作日。

- [ ] **P0-5 `body_pose.height` 的度量与取值范围**
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

- [ ] **P0-6 机械臂 PD 增益对齐到本次训练（40/8 → 300/20）**
  - 要做什么：`arm_controller.py` 的 `ARM_KP/ARM_KD`、仿真侧"首条命令前的默认保持"
    `ARM_DEFAULT_KP/KD`，统一取本次 run 的 `piper_arm` 执行器配置；
    把它做成单一常量来源，避免三处各写一份。
  - 依据（为什么）：`2026-09-20_00-50-31/params/env.yaml` 的
    `piper_arm.stiffness=300.0 / damping=20`、`piper_gripper=4000/200`。
    当前仓库三处不一致：`arm_controller.py:84` 是 `40/8`、
    `piper_arm_interface.hpp:78` 是 `300/20`、
    `mujoco_simulation_ros2.py:116` 又写了一份 `40/8`。
  - 验收：L2/L3 里臂跟踪阶跃不振荡；把三处的值打印出来逐条比对。
  - 预估：0.5 个工作日。

- [ ] **P0-7 安全接管阈值与限幅**
  - 要做什么：把训练终止阈值（倾角 > 0.8 rad、height < 0.30 m）接进
    `RLControlState::PostureUnsafeCheck`（现在是空实现），触发后切阻尼/站立；
    关节力矩/速度限幅按训练 `env.yaml`（腿 76.4 N·m / 22.4 rad/s，轮 21.6 / 79.3，
    臂 100 / 3.0，夹爪 10 / 1.0）；补通信超时与软启动。
  - 依据（为什么）：`docs/deploy_sim2sim_sim2real_zh.md` 第 9 节；
    本仓库 `PostureUnsafeCheck()` 里两段判断被注释掉了。
  - 验收：人为把机器人推倒时能自动接管且日志给出原因；力矩限幅在
    L3 的大命令注入测试里生效。
  - 预估：1 个工作日。

- [ ] **P0-8 加固"进场"：把 L3 冒烟接进合并门槛并覆盖更多起点**
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
> 零增益命令执行）—— 这两条修完，`tests/sim2sim_smoke.py --mode hold/rl` 均已 PASS。
> 遥测（P3-2）与一键冒烟（P3-1 的一半）也已可用。

- [ ] **P1-1 MuJoCo 物理与训练对齐（结构性差异清单）**
  - 要做什么：逐条核对并记录取舍：`timestep`（MJCF 0.002 vs 训练 `sim.dt=0.005`）、
    执行器延迟（训练 `DelayedPD` 随机 0~5 物理步）、摩擦（训练随机 0.35~1.5）、
    `restitution`（0~0.7）、求解迭代（4/1）、`base_link` 的显式惯性
    （本仓库 MJCF 加了 `mass=15.882` + `diaginertia`，训练侧资产是否有同样值 **未核对**）。
  - 依据（为什么）：这些量决定"仿真里站得住"能不能推出"真机站得住"。
  - 验收：`docs/sim2sim_layout_contract_zh.md` 里出一张"训练 vs MuJoCo"对照表，
    每行标"已对齐 / 有意偏离 / 未核对"。
  - 预估：1 个工作日。

- [ ] **P1-2 轮子速度伺服的行为验收**
  - 要做什么：单独给 4 个轮子发阶跃速度目标，测稳态误差与上升时间，
    确认 `kp=0 / kd=0.6` + 轮子 armature 的行为与训练一致；
    确认轮子的**符号**（`ω_des = 5·a`）。
  - 依据（为什么）：轮子是唯一的速度伺服通道，坐标/符号/标定错误首先在这里暴露
    （训练侧 `DEFECT_LOG_zh.md` 的失败模式表第 2 行）。
  - 验收：`a[12..15] = +1` 时四个轮子同向、稳态 ω ≈ 5 rad/s（±10%）。
  - 预估：0.5 个工作日。

- [ ] **P1-3 一次性"接口自检"脚本**
  - 要做什么：把 `check_policy_interface.py` 扩成"启动期自检"：ONNX 维度/名字、
    布局 JSON、24 维关节映射、16 维动作映射、默认角、增益、观测 scale/clip
    全部断言一遍，失败就 `exit(1)` 并打印"期望 vs 实际"。
  - 依据（为什么）：训练侧 `DEF-013/DEF-016` 的教训是"布局变了会**静默**失效"。
  - 验收：故意把布局 JSON 改错一个数字，脚本必须报错退出。
  - 预估：0.5 个工作日。

- [ ] **P1-4 IK 与训练侧 DLS 的一致性**
  - 要做什么：对比 `arm_controller.py` 的 DLS 解与训练侧
    `velocity/mdp/actions.py::CommandDrivenIKAction`（λ=0.01、绝对位姿、
    `gripper_base`、root 系）在若干目标位姿上的解，记录关节角差。
  - 依据（为什么）：臂虽不占动作维度，但它的位姿进入 `joint_pos` 观测与
    `ee_goal`，解不一致会持续给策略"错误的现实"。
  - 验收：随机 20 个目标上 `max |Δq| < 0.02 rad`（或在文档里写清偏差来源）。
  - 预估：1 个工作日。

- [ ] **P1-5 清理旧 checkpoint 的调试开关**
  - 要做什么：逐条判定 `WORKFLOW_zh.md` 第 6.1 节里"旧 checkpoint 期"引入的开关
    （`M20_LAST_ACTION_MODE` / `M20_IK_FEEDBACK` / `M20_FREEZE_*` /
    `M20_ARM_DEFAULT_TRAINED` / `M20_ZERO_CMD_WHEEL_BRAKE`）在新 checkpoint 下是否还需要，
    不需要的连同代码一起删。
  - 依据（为什么）：新 checkpoint 动作是 16 维、臂默认角就是 `0.5/-0.5`，
    这些开关的适用前提已经不存在。
  - 验收：删掉后 L1 + L3 仍通过；开关登记表同步更新。
  - 预估：0.5 个工作日。

---

## P2 —— sim2real 落地

- [ ] **P2-1 真机腿部标定链核对**
  - 要做什么：核对 `M20Interface`（真机 dir/offset 标定、启动期 ±360° 多圈补偿）
    与仿真 `M20SimInterface`（恒等标定）的差异清单，确认"仿真里跑通的策略
    换到真机只需要换接口"。
  - 依据（为什么）：`m20_sim_interface.hpp` 的注释已经指出真机接口会在启动期
    调整偏移，若机器人启动时漂移会污染坐标系。
  - 验收：出一张"仿真 vs 真机"接口差异表 + 上线检查单。
  - 预估：1 个工作日。

- [ ] **P2-2 机械臂真机传输**
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

---

## P3 —— 工具与文档

- [ ] **P3-1 `tests/` 一键验收**
  - 要做什么：`tests/run_all.sh` 依次跑 L0 → L1 → L3（无头 + 数值判据），
    输出 PASS/FAIL 与关键数字；CI 或人工合并前只跑这一条命令。
  - 验收：故意引入一个错误（改错关节顺序）时脚本必须 FAIL。

- [ ] **P3-2 遥测落盘**
  - 要做什么：仿真侧提供 `M20_SIM_TELEMETRY=<path>`，按 200 Hz 写 CSV：
    `t, base_pos, base_quat, rpy, omega, q[24], dq[24], tau[24], 轮接触力`，
    供 L3 判据与离线画图使用（现在只能靠 `[JVEL-SIM]` 打印肉眼判断）。
  - 验收：CSV 行数 = 时长 × 200（±1%），能直接算出高度/倾角曲线。

- [ ] **P3-3 清理过时文档**
  - 要做什么：`docs/M20_Piper_deploy_6commits_zh.md` 描述的是早期 6 个提交
    （`9095957`~`2fcab7c`），与 `main`（领先 `origin/main` 21 个提交）已经不符；
    按新规范改写成"历史归档"，或并入 `DONE_zh.md` 并在原处留指针。
  - 验收：`rg "M20_Piper_deploy_6commits" docs/ README.md` 的引用都指到现行文档。

- [ ] **P3-4 把策略文件纳入版本管理的方式定下来**
  - 要做什么：现在 `policy/*.onnx` 是直接提交进仓库的（旧 checkpoint）。
    需要定规则：新策略放哪、命名规则（含 run 与 checkpoint 版本）、
    是否连 `policy_layout.json` 一起提交、如何在部署机上换策略。
  - 验收：`README.md` 里有"换策略"的标准步骤，且 runner 会在缺文件时打印该步骤。
