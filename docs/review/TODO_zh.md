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
| 2026-09-20 | **P1-1 完成**（物理对照表定案，写入契约文档第 5 节） | `main` |
| 2026-09-20 | **P1-2 完成**（轮子速度伺服阶跃验收）；新增 `--mode wheel_step`；`arm_move` 增加末端位移判据；仿真遥测增加末端位姿与轮速指令列；**DEF-022 修键盘目标限速**，IK 精度实测 0.2 mm | `feat/actuator-accuracy-tests` |

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
- [ ] ~~P0-4（原始条目，保留供对照）~~
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
- [ ] ~~P0-6（原始条目，保留供对照）~~
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
- [ ] ~~P0-7（原始条目，保留供对照）~~
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
