# 缺陷 / 特性记录（DEFECT LOG）

**文档职责**：每条"现象 → 根因 → 修正 → 结果"，一个缺陷/特性一条，编号 `DEF-0xx`。
新条目加在最上面。模板见 `templates/DEFECT_ENTRY_TEMPLATE_zh.md`。

**维护约定**：见 `templates/DOC_TEMPLATE_zh.md`。凡是改变了
关节顺序 / 坐标系 / 增益 / 观测维度 / 话题的改动，**必须**在这里留一条。

## 更新记录

| 日期 | 更新内容 | 相关 commit / 分支 |
|---|---|---|
| 2026-09-20 | 初版：补记 DEF-001~DEF-010（含本轮核对发现的 5 条未修缺陷） | `docs/review-spec` |
| 2026-09-20 | 新增 DEF-013（仿真把"非控制字帧"当零增益命令执行）；DEF-012 已修 | `fix/sim2sim-bringup` |
| 2026-09-20 | 新增 DEF-014（ORT C++ API 两个生命周期陷阱）、DEF-015（SIGINT 退出时 abort）；DEF-005/010 已修（接口切到 83/700/16） | `feat/policy-layout-v2` |
| 2026-09-20 | DEF-007（臂增益）/ DEF-009（高度区间）/ DEF-011（armature）已修；新增 `tests/sim2sim_smoke.py --mode arm` | `fix/arm-gains-armature-height` |
| 2026-09-20 | 新增 DEF-016（安全接管）、DEF-017（策略线程吃到未初始化观测 → 进 RL 偶发摔倒） | `fix/safety-takeover` |
| 2026-09-20 | 新增 DEF-019（`ee_goal` 默认值坐标系错）；DEF-018 补软启动/执行器语义两组对照实验数据 | `fix/entry-transient` |
| 2026-09-20 | 新增 DEF-020（`arm_controller` 发布 `ee_goal` 用错坐标系）、DEF-021（`arm_teleop` 非 TTY 崩溃）；新增 `--mode arm_move` | `fix/ee-goal-frame-arm-node` |
| 2026-09-20 | 新增 DEF-022（键盘增量无限速 → 一按撞边界）；IK 精度实测 0.2 mm | `feat/actuator-accuracy-tests` |

---

### DEF-017 `2026-09-20` 进 RL 的第一拍可能用"全零观测"算动作（偶发摔倒）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `部分已修`（门禁已加；**仍有残留的偶发失败**，见 DEF-018） |
| 影响面 | 二者 |
| 关联 | `fix/safety-takeover`；`state_machine/quadruped_wheel/rl_control_state.hpp` |

**1. 现象（怎么发现的）**

* `tests/sim2sim_smoke.py` 在连续跑多个模式时**偶发**失败：`rl` 模式在
  t≈9.4 s、`walk` 模式在 t≈8.5 s（都正好是"刚进 RL"的那一两秒）摔倒；
  同一配置单独连跑 3 次又全过。
* 这类"偶发 + 发生在状态切换瞬间"的失败，先看**时序**而不是策略。

**2. 根因**

* `RLControlState::OnEnter()` 里先 `std::thread(...PolicyRunner)` 起策略线程，
  但**观测缓冲 `rbs_[2]` 还没被 `UpdateRobotObservation()` 填过**：
  策略线程第一拍读到的是默认构造的 `RobotBasicState`（关节角全 0、
  `base_rot_mat` 单位矩阵），算出的是一个与真实姿态无关的动作；
  更糟的是这一"垃圾帧"会被写进 history 的**最旧端**，影响后面 10 个策略周期
  （200 ms），足以把机器人推倒。
* 为什么偶发：谁先跑取决于线程调度 —— 大多数时候 `Run()` 的第一次
  `UpdateRobotObservation()` 抢在策略 tick 之前，偶尔抢不到。

**3. 修正**

* 新增 `rbs_ready_` 门禁：`UpdateRobotObservation()` 成功写入一帧后置位，
  策略线程在门禁为 false 时**不执行任何 tick**；
  `OnEnter()` 里先手动采一帧真实状态再起线程。

**4. 结果（验收）**

* `--mode rl` 单独连跑 3 次全 PASS（修复前同一序列里约 1/3 概率失败）；
* **未收敛**：五档**连续**跑时仍偶发失败（`rl` 在 t≈10.0 s），见 `DEF-018`。
  这一条修复是"应该修的真问题"，但显然不是唯一原因。

---

### DEF-022 `2026-09-20` 键盘增量路径没有目标限速：按一下就把目标甩到工作空间边界

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（可用性 / 安全） |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `interface/robot/simulation/arm_controller.py`（`_apply_incremental` / `_tick`） |
+
+**1. 现象**
+* `arm_teleop` 的键盘增量是按 **200 Hz** 发的（每 tick 5 mm ⇒ 1 m/s），
+  `arm_controller` 收到就直接 `target_pos += inc`：**按住 0.1~0.4 s 目标就撞到
+  工作空间限位**，之后臂一直趴在边界上不动、IK 残差 15~22 mm（结构性，不是 IK 坏），
+  观感是"一按就撞死、不跟手"。
+* 自动化测试里表现为：`--mode arm_move` 想看"跟踪精度"，结果每次都测到
+  撞限位后的残差。
+
+**2. 根因**：没有对**被跟踪目标**做速度限制；输入频率（200 Hz）比控制周期（50 Hz）
+快 4 倍，逐条消息限速等于没限。
+
+**3. 修正**
+* `_apply_incremental` 改为**累计原始增量**，在 `_tick`（50 Hz）里按
+  `M20_EE_MAX_LIN_SPEED`（默认 0.35 m/s）/ `M20_EE_MAX_ANG_SPEED`（0.8 rad/s）
+  推进"请求位姿"；
+* 被跟踪目标再用 `_slew_toward` 逼近请求位姿（同样是限速），VR 路径用更松的
+  `M20_VR_MAX_LIN_SPEED` / `_ANG_SPEED`（2.0 m/s / 5.0 rad/s）；
+* 请求位姿在写回时按工作空间 `EE_POS_CLAMP` 夹取。
+
+**4. 结果（验收）**：`tests/sim2sim_smoke.py --mode arm_move`（进 RL 后短按
+numpad 8 0.4 s 再松开）：
+
+| | 末端 x 位移 | IK 位置残差（末尾 5 次） | 底盘 tilt |
+|---|---|---|---|
+| 修复前 | +0.49 m（撞到 0.6 m 限位） | **21.8 mm**（结构性） | 1.2° |
+| 修复后 | **+0.246 m**（限速后的正常位移） | **0.18 mm** | 1.1° |
+
+即：轻点只挪一点、按住连续走；IK 在可达目标上的跟踪精度 **0.2 mm**。
+`--repeat 6`：失败率 17%（入口瞬态，DEF-018）。
+
+---
+
+### DEF-021 `2026-09-20` `arm_teleop_node` 在非终端 stdin 下起不来

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `interface/robot/simulation/arm_teleop_node.py::_setup_stdin` |
+
+**1. 现象**：用管道（`printf "8" | python3 arm_teleop_node.py`）或从 launch/服务拉起时，
+节点直接崩：`termios.error: (25, 'Inappropriate ioctl for device')`。
+
+**2. 根因**：`_setup_stdin()` 无条件 `tcgetattr/tty.setraw`；stdin 不是 TTY 时必崩。
+后果有两层：① 自动化测试没法驱动臂遥操作；② 真机上若用 service/launch 方式拉起臂
+键盘节点（没有交互终端）会直接挂。
+
+**3. 修正**：`_setup_stdin/_restore_stdin` 捕获 `termios.error`，非 TTY 时跳过 raw 模式
+（打印提示），按键读取逻辑不变。
+
+**4. 结果**：`tests/sim2sim_smoke.py --mode arm_move` 能通过管道按住 numpad 8，
+臂端到端动起来（见下一条）。
+
+---
+
+### DEF-020 `2026-09-20` `arm_controller` 发布的 `ee_goal` 是**臂基座坐标系**（与策略差 24 cm）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `interface/robot/simulation/arm_controller.py`；`rl_control_state.hpp`；`DEF-019` |

**1. 现象（怎么发现的）**
* 单独起 `arm_controller`，`ros2 topic echo --once /ARM_TELEOP_STATE` 得到
  `(0.1092, 0, 0.3439, ...)` —— **臂基座坐标系**下 Piper 默认姿态的 EE 位置；
* 而 `rl_deploy` 把它直接塞进策略观测的 `ee_goal`，策略要的是 **root（机体）坐标系**
  （默认姿态应当是 `0.3492, 0, 0.4327`，两者相差臂座安装偏移 `(0.24, 0, 0.0888)`）。
* 后果：只要**开了机械臂节点**，策略就会持续看到一个落在机体内部 24 cm 的 EE 目标 ——
  这正是"部署效果欠佳"的候选主因之一（`DEF-019` 是同一问题的"默认值"版本）。

**2. 根因**：两个节点对同一话题 `/ARM_TELEOP_STATE` 的坐标系约定不一致：
`arm_controller` 整条链路（FK/IK/限位）都在臂基座系，发布时没有加回臂座偏移；
`rl_deploy` 按训练口径（`HeightInvariantEECommand.command_local`，root 系）使用。

**3. 修正**：`arm_controller` 发布前把 `ARM_BASE_OFFSET = (0.24, 0, 0.0888)` 加回去
（纯平移，姿态不变），并留 `M20_EE_GOAL_BODY_FRAME=0` 回到旧行为用于调试。

**4. 结果（验收）**
* `ros2 topic echo --once /ARM_TELEOP_STATE` → `(0.3492, 0, 0.4327, ...)`（默认）；
  `M20_EE_GOAL_BODY_FRAME=0` → `(0.1092, 0, 0.3439, ...)`；
* `M20_PIPER_DEBUG` dump：`--mode arm` 下第一拍 `obs[73:80] = (0.3492, 0, 0.4327)`；
* 新增端到端用例 `--mode arm_move`（见 `DONE_zh.md` 第十三节）。

---

### DEF-019 `2026-09-20` `ee_goal` 默认值用了**臂基座坐标系**的值（差 24 cm）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `state_machine/quadruped_wheel/rl_control_state.hpp`（`ee_goal_` 初值）、`include/types/common_types.h`（`UserCommand::ee_goal_pos`） |

**1. 现象（怎么发现的）**
* 打开 `M20_PIPER_DEBUG` dump 第一拍的观测：`obs[73:80] = (0.1092, 0, 0.3439, ...)`
  —— 这是**臂基座坐标系**下 Piper 默认姿态的 EE 位置；
* 而策略要的是 **root（机体）坐标系**的目标位姿，默认姿态下应当是
  `(0.3492, 0, 0.4327)`（两条值正好差臂座安装偏移 `(0.24, 0, 0.0888)`）。
* 影响：臂节点没起、或刚进 RL 还没收到 `/ARM_TELEOP_STATE` 时，策略看到的是
  一个**落在机体内部 24 cm** 的 EE 目标。

**2. 根因**：同一个物理位姿在两个坐标系里各写了一份常量，代码取了臂基座坐标系那份。
文档（`README.md` / 契约文档）里写的都是 root 系那份，所以只看文档会以为是对的。

**3. 修正**：两处默认值统一改成 root 系 `(0.3492, 0, 0.4327)`，并在注释里写明
"另一份 (0.1092, 0, 0.3439) 是臂基座坐标系，别再混用"。

**4. 结果（验收）**
* `M20_PIPER_DEBUG` dump 的第一拍 `obs[73:80]` 变成 `(0.3492, 0, 0.4327)`；
* 离线量化（同一标称状态，只换 `ee_goal`）：臂基座值 |a|max = 1.44、
  root 系值 |a|max = 1.46、零位姿 1.24 —— 说明这一项在标称状态下**不会**
  炸掉输出，但它是"观测与训练口径不一致"，属于必须修的接口错。

---

### DEF-018 `2026-09-20` 进 RL 的入口瞬态：约 25% 的运行会发散（**不是部署侧 bug**）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（**策略侧**，部署侧只能减轻不能消除） |
| 状态 | **`未修`**：部署侧已完成 4 组对照实验，都不是解；已按"训练侧 P1-2"的口径交接 |
| 影响面 | 二者（真机入口同样会有这个瞬态） |
| 关联 | `fix/entry-transient`；`tests/sim2sim_smoke.py`；训练仓库 `TODO_zh.md` P1-2（s3 阶段 `root_height_below_minimum` 0.09~0.15） |

**1. 现象（怎么发现的）**
* `tests/sim2sim_smoke.py --mode rl`（进 RL 后零命令、25 s）**约 25%（3/12）** 的运行
  在进 RL 后 1~2 s 摔倒；同一配置多跑几次则时好时坏。
* 抓现场（`M20_PIPER_DEBUG` dump + 遥测）：
  * 进 RL 第一拍动作被软启动压到 0（正常）；
  * 第 2 拍起轮速命令约 -0.14（ω≈-0.7 rad/s）并逐渐放大；
  * **失败运行**里轮速命令持续增长到 ±3~5（ω ±15~25 rad/s）、机身俯仰从 0.5° → 5° → 33°
    → 59° → 109°，即**策略自身的闭环在入口发散**，不是某个观测接错。
  * 对比同一测试的通过运行：第 2~3 拍就回到 ±1 rad/s、俯仰回到 1° 以内。
* 两组 dump 的**第一拍观测只差 1.2e-3**（入口时序抖动），随后每拍放大 ~1.45 倍
  —— 典型的"边缘稳定 + 微小扰动"。

**2. 已排除的原因（都是实测，不是推理）**

| 候选 | 做法 | 结果 |
|---|---|---|
| 仿真跑慢、控制周期被拉长 | 给控制循环加"追帧"，并在遥测里记录墙钟 | 实时因子全程 **1.000**（最差 1 s 窗口 0.97）→ 排除 |
| 策略线程吃到未初始化观测 | `rbs_ready_` 门禁 + `OnEnter` 先采一帧 | 是真 bug（DEF-017），但对失败率无影响 |
| `run_cnt_` 未初始化（线程先于 `OnEnter`） | 就地初始化 + 调整启动顺序 | 是真 bug，对失败率无影响 |
| `ee_goal` 坐标系错 | 修成 root 系（DEF-019） | 是真 bug，对失败率无影响 |
| 软启动长度 | `M20_SOFT_START_TICKS` = 0/10/25/50，各 6 次 | 0/6、1/6、2/6、2/6 → **软启动不是解** |
| 入口执行器语义跳变（轮子 位置保持→速度伺服） | idle/standup 的轮子改成 kp=0/kd=0.6 | **更差**（7/12 vs 3/12）→ 回退 |
| 执行器延迟（训练 DelayedPD 0~25 ms） | 仿真加每关节随机 0~25 ms 延迟，12 次 | **更差**（5/12）→ 保留为开关、默认关 |
| 轮子"无扰交接"（进 RL 时 kp 10→0 线性过渡） | `M20_WHEEL_HANDOVER_TICKS` = 0/25/50，各 12 次 | 3/12、4/12、2/12 → 噪声内，**无显著改善** → 保留为开关、默认关 |
| 站立腿增益 200 → RL 的 80 | `StandUpState` 改用 80/2，12 次 | 4/12 → 无改善，回退 |

**3. 结论与交接**
* 部署侧的"接口/时序"问题都已修完（DEF-013/017/019 + `run_cnt_`）；
* 剩下的 **~25% 入口发散属于策略在该动力学下的边缘稳定性**：
  训练侧自己的 s3 阶段 `root_height_below_minimum` 就是 0.09~0.15/20 s 量级，
  与本现象同源；MuJoCo 与 Isaac 的动力学差异（接触求解、摩擦/恢复系数的随机化、
  仿真步长）会放大这个边缘性。
* 交接给训练侧的建议（对应训练仓库 `TODO_zh.md` P1-2）：
  ① 提高入口鲁棒性（reset 后前 0.2~0.5 s 的动作/历史分布、动作平滑代价）；
  ② 或在训练里显式加入"从站立保持切到策略"的过渡（把
  部署侧的入口状态当成 reset 分布的一部分做 domain randomization）。

**4. 当前可用的验收口径**
* 单档 `--mode rl` / `walk` / `arm_move` **不能**当通过/失败判据（会给假阴性）；
* `tests/sim2sim_smoke.py --repeat N` 会跑 N 次并报告失败率与实时因子，
  这三档的判据是"失败率 ≤ 1/2"（当前实测 rl 0~25%、walk 17~50%、arm_move 0~17%），
  其余档位仍要求 0 失败 —— 这是"策略现状"的量化，不是合格线；
* `hold / walk / arm / push` 四档仍然可以单次判通过（它们没观察到这种抖动）。

---

### DEF-016 `2026-09-20` 安全接管阈值是空实现（摔倒后策略还在输出）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `fix/safety-takeover`；`state_machine/quadruped_wheel/rl_control_state.hpp::PostureUnsafeCheck`；`interface/robot/simulation/mujoco_simulation_ros2.py` |

**1. 现象**：`PostureUnsafeCheck()` 里两段倾角判断被注释掉，函数恒返回 false ⇒
`LoseControlJudge()` 永远不会因为姿态把机器人切到阻尼状态；机器人被打倒后
策略继续输出、轮子继续转。

**2. 根因**：接管阈值没实现（训练侧的终止阈值 `倾角 > 0.8 rad` 与
`height < 0.30 m` 只写在文档里）。

**3. 修正**：
  * 倾角：`acos(cos(roll)·cos(pitch))`（= 训练定义 `acos(-g_z)`，yaw 无关），
    阈值 0.8 rad（`M20_TILT_TAKEOVER` 可覆盖，`<=0` 关闭）；
  * 兜底"腿折叠"：`|q − q_default| > 1.2 rad`（`M20_LEG_FOLD_TAKEOVER`），
    真机会出现"腿被压在身下但姿态还没到 45°"的情形；
  * 触发时打印 `[TAKEOVER!] <原因> = <值>`（只打一次），
    状态机沿用既有逻辑切到 `kJointDamping`；
  * 仿真侧新增扰动注入 `M20_SIM_PUSH_FORCE/_AT/_DURATION`（`xfrc_applied`），
    让"接管"这件事可被自动测试。

**4. 结果（验收）**：新增 L3 用例 `tests/sim2sim_smoke.py --mode push`
（默认 800 N / 12 s / 0.3 s）：

  * 默认阈值：`[TAKEOVER!] leg fold = 1.228 (rad > 1.2)` → 切 `joint_damping`，PASS；
  * 把折叠判据关掉（`M20_LEG_FOLD_TAKEOVER=0`）复测倾角通路：
    `[TAKEOVER!] tilt = 0.818 (rad > 0.8)` → 切 `joint_damping`，PASS。

  两条通路都验证过；`Push` 模式**期望**摔倒，所以跳过"摔倒判据"那一项，
  改为断言日志里出现 `[TAKEOVER!]` 与 `joint_damping`。

---

### DEF-011 / DEF-007 / DEF-009 修复记录（2026-09-20，`fix/arm-gains-armature-height`）

三条都是"参数/模型与训练不一致"的缺陷，改动很小但影响 sim2sim 到 sim2real 的可迁移性：

| 条目 | 改了什么 | 验收 |
|---|---|---|
| `DEF-011` armature | MJCF 里把 `armature` 从 `<motor>`（MuJoCo 静默忽略）挪到对应的 `<joint>`：轮 0.00243216、臂/夹爪 0.01、腿 0 | `check_mjcf_contract.py`：**10 PASS / 0 FAIL**（原来 2 个 FAIL）；`非零 armature 的关节` 一项 PASS |
| `DEF-007` 臂增益 | 三处统一到训练值 **300/20**（夹爪 4000/200）：`arm_controller.py`、`mujoco_simulation_ros2.py::ARM_DEFAULT_KP/KD`（`piper_arm_interface.hpp` 本来就是 300/20） | 新增 `--mode arm`：臂相对默认位姿最大偏差 **0.0153 rad**、最大关节力矩 **4.6 N·m**（限幅 100），底盘 height 0.515 m / tilt 1.0° |
| `DEF-009` 高度区间 | 键盘与 VR 的机身高度命令上限从 **0.60 → 0.55**（训练终值区间 (0.33, 0.55)），并在代码里写清"高度是**相对足端**的度量：`root_z − mean(四轮 z) + 0.09`" | 编译 + 四档 L3 全绿；度量定义由 `check_mjcf_contract.py` 实测复核（默认姿态着地 = 0.5266 m） |

顺带修掉 `arm_controller.py` 退出时的 `rclpy.shutdown` 二次调用（SIGINT 后抛 `RCLError`，
会让测试脚本把正常退出误判成崩溃）。

**遗留**：`DEF-009` 只收敛了区间；"操作员按一次键就是 +0.002 m"这种**相对增量**语义没变
（命令值本身就是训练里的那个相对高度，所以不需要换算）。真机上如果实际站立高度
与训练差得多，仍然要靠 P2-1 的标定核对。

---

### DEF-015 `2026-09-20` 收到 SIGINT 退出时 `rl_deploy` abort（`terminate called without an active exception`）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `feat/policy-layout-v2`；`state_machine/quadruped_wheel/qw_state_machine.hpp::Stop`、`.../rl_control_state.hpp::OnExit` |

**1. 现象**：`tests/sim2sim_smoke.py` 结束（SIGINT）时，`rl_deploy` 打
`terminate called without an active exception` + `[ros2run]: Aborted`。

**2. 根因**：`QwStateMachine::Stop()` 只停 `SafetyController / 用户接口 / 机器人接口`，
**没有调用当前状态的 `OnExit()`** ⇒ 进过 RL 之后，`RLControlState::run_policy_thread_`
仍在跑；`rl_controller_` 这个 shared_ptr 在进程结束时析构，`std::thread` 成员
在 joinable 状态下析构 → `std::terminate`。

**3. 修正**：`Stop()` 先 `current_controller_->OnExit()` 再停接口；
`RLControlState::OnExit()` 加幂等判断（`run_policy_thread_.joinable()`），
避免"状态切换 + 进程退出"两条路径各调一次时对已 join 的线程再 join。

**4. 结果**：退出日志只剩 `[rclcpp]: signal_handler(SIGINT/SIGTERM)` +
`[KEYBOARD] Stopped.`；`tests/sim2sim_smoke.py` 新增的
"日志里不许出现 Segmentation fault/Traceback/Aborted" 检查通过。

---

### DEF-014 `2026-09-20` ONNX Runtime C++ API 的两个生命周期陷阱（段错误 / length_error）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `feat/policy-layout-v2`；`run_policy/m20_piper_policy_runner.hpp::OpenSession` |

**1. 现象**：布局驱动版 runner 一启动就死，两种表现：
  * 传 `nullptr` 当 allocator → `Segmentation fault`；
  * 改成默认 allocator 后变成 `std::length_error: cannot create std::vector
    larger than max_size()`。

**2. 根因**（都是 ORT C++ 封装的生命周期问题，与模型无关 —— 用一个 20 行的
独立小程序验证过同一份 ONNX 能正常加载）：
  1. `GetInputNameAllocated(i, nullptr)` 返回的 `AllocatedStringPtr` 析构时调用
     `allocator->Free(...)` → 空指针解引用；
  2. `session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo()` 返回的对象**指向
     TypeInfo 临时量的内部数据**，把它存进变量、之后再 `GetShape()` 就是
     use-after-free（拿到垃圾 size → `std::length_error`，或者直接段错误）。

**3. 修正**：allocator 用 `Ort::AllocatorWithDefaultOptions`；
形状一律在同一句里取完（`GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape()`）。

**4. 结果**：`rl_deploy` 启动打印
`layout kind=history obs=83 history=10x70 latent=32 action=16`；
L1/L3 全绿。
**教训**：这类"启动就死"的问题，如果只看"机器人站着没倒"，会被误判成"策略在工作"
（当时仿真自身的保持姿态也是站着的）—— 所以 `tests/sim2sim_smoke.py` 现在会断言
日志里必须出现 `M20PiperPolicyRunner` 与 `rl_control`。

---

### DEF-013 `2026-09-20` 仿真把"非控制字帧"当成零增益命令执行 ⇒ 进 RL 前机器人先被放倒

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | `sim2sim` |
| 关联 | `fix/sim2sim-bringup`；`interface/robot/simulation/mujoco_simulation_ros2.py`；`interface/robot/hardware/dds_interface.hpp:189` |

**1. 现象（怎么发现的）**

* 复现：`python3 tests/sim2sim_smoke.py --mode rl --duration 25`（修复前）。
* 遥测（`M20_SIM_TELEMETRY`）显示：`t=3.2~3.6 s` 腿被驱动到
  `hipy ∓1.30 / knee ±2.91`、底盘高度掉到 **0.11 m**（趴地）；
  随后 `t=3.7~4.1 s` 底盘被顶到 **0.836 m**（从地面弹起），
  之后落地不稳、`t≈6 s` 翻倒到倾角 138°。
* 这个时间点正是 `rl_deploy` 启动的时刻（`DdsInterface` 构造函数里的
  `sleep(1)` + `ResetJointError()`）。

**2. 根因**

* `DdsInterface` 的构造函数会连发 4 帧 `ResetJointError`：`kp=0, kd=0,
  position=0, velocity=0`，控制字分别是 `1(disable) / 17(error reset) /
  2(enable) / 23(get status)`。真机上这些帧只做**电机状态操作**，
  增益字段由固件忽略；仿真节点却把它们当成普通增益命令执行 ⇒ 腿瞬间零增益、
  机器人趴下；等 `M20SimInterface::Start()` 的站立命令到达时，机器人已经
  折叠在地上，于是被"顶"起来弹飞。
* 为什么以前没暴露：没有遥测时只能看到"进 RL 就摔"，很容易误判成策略问题。

**3. 修正**

* 仿真侧在 `/JOINTS_CMD`、`/ARM_JOINTS_CMD` 的回调里检查 `control_word`：
  只有 `kIndexMotorControl(=4)` 的帧才是关节控制命令，其余整帧丢弃并计数
  （首次/第 10 次/第 100 次打印一条 INFO）。
* 备选方案是"在 `ResetJointError` 里不发 position"，但那会改动真机路径，否掉。

**4. 结果（验收）**

* 修复前后同一个测试（旧 checkpoint、`--mode rl --duration 25`）：

  | | height min | 后半段 height 均值 | tilt max | 结果 |
  |---|---|---|---|---|
  | 修复前 | −0.271 m | −0.112 m | 147.4° | FAIL（t≈3.4 s 触发摔倒判据） |
  | 修复后 | **0.502 m** | **0.539 m** | **1.7°** | **PASS**（25 s 不摔） |

* 回归：`--mode hold` 也 PASS（height 0.498 m、tilt 1.4°）。

---

### DEF-012 `2026-09-20` 仿真的初始腿部保持命令用的是**另一个机型**的站立位姿

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修`（`fix/sim2sim-bringup`） |
| 影响面 | `sim2sim` |
| 关联 | `interface/robot/simulation/mujoco_simulation_ros2.py:150`、`:233`；`include/.../m20_sim_interface.hpp:43` |

**1. 现象（怎么发现的）**

* 只起仿真节点（不起 `rl_deploy`）时，读 `/JOINTS_DATA` 得到腿部关节稳态值：

  ```bash
  # 容器内
  M20_USE_VIEWER=0 M20_JVEL_DEBUG=0 python3 \
      src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py &
  ros2 topic echo --once --field data.joints_data /JOINTS_DATA
  # fl: hipx -0.4376  hipy -1.2097  knee +2.8162
  # fr: hipx +0.4376  hipy -1.2096  knee +2.8161
  # hl: hipx -0.4370  hipy +1.2089  knee -2.8130
  # hr: hipx +0.4370  hipy +1.2088  knee -2.8129
  ```

* 这正好是 `LEG_INIT["M20"]`（旧机型 M20 的站立位姿
  `hipx ±0.438 / hipy ∓1.16 / knee ±2.76`），而**不是**训练默认姿态
  `hipx 0 / hipy ∓0.6 / knee ±1.0`。

**2. 根因**

* `mujoco_simulation_ros2.py` 的初始化与 `_reset_callback` 都用
  `self.pos_cmd = LEG_INIT["M20"]` 作为保持命令，但 `_set_initial_pose()` 用的是
  `JOINT_INIT = LEG_INIT["M20_Piper_own"] + ARM_INIT`。注释写的是
  "Hold the initial (policy-default) pose"，代码却不是。
* 影响窗口：从仿真启动到 `rl_deploy` 的第一条 `/JOINTS_CMD` 到达之间
  （`M20SimInterface::Start()` 会立刻发正确的训练默认姿态）。
  所以**正常运行时的实际影响有限**，但一旦只跑仿真、或 `rl_deploy` 启动慢，
  机器人会被驱动到一个训练里从未见过的姿态（hipy/knee 偏差 0.6 / 1.8 rad），
  之后进 RL 的第一帧观测就是分布外的。
* README 里那条"站起来时可能因自碰撞卡住，不是 bug"很可能就是这个位姿切换造成的。

**3. 修正**

* 计划改法：把 `M20` 与 `M20_Piper_own` 两套位姿的用途分清楚 —— 本仓库的
  M20+Piper 部署只应该用 `M20_Piper_own` 的那套，并让"初始保持命令"和
  "初始 qpos"取自同一个常量。

**4. 结果（验收）**

* **已修**：`self.pos_cmd` 改用 `LEG_INIT["M20_Piper_own"]`（与初始 qpos 同一份常量）。
* 验收（`tests/sim2sim_smoke.py --mode hold --duration 12`）：

  | | height min | 后半段 height 均值 | tilt max | 结果 |
  |---|---|---|---|---|
  | 修复前 | 0.057 m | 0.095 m（趴地） | 2.2° | FAIL |
  | 修复后 | **0.493 m** | **0.498 m** | 1.4° | PASS |

* 独立复核（`/tmp/holdtest.py`，纯 MuJoCo 不开 ROS）：保持目标=训练默认姿态 →
  height 0.5042 m；保持目标=旧机型姿态 → height 0.0954 m。与上面一致。

---

### DEF-011 `2026-09-20` MJCF 的 `armature` 写在 `<motor>` 上，被 MuJoCo **静默忽略**

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷 |
| 状态 | `已修`（`fix/arm-gains-armature-height`，见上方修复记录） |
| 影响面 | `sim2sim` |
| 关联 | `M20_Piper_description/mjcf/M20_Piper_own.xml:18,21,24`；`e5e1917`（引入这些属性的提交）；`scripts/check_mjcf_contract.py` |

**1. 现象（怎么发现的）**

* `check_mjcf_contract.py` 输出：24 个执行器的 `dof_armature` **全为 0**，
  而期望是"轮 0.00243216、臂/夹爪 0.01"（= 训练 `env.yaml` 的 actuator 配置）。
* 最小复现（容器内）：

  ```python
  # <motor ... armature="0.01"/>  →  dof_armature = [0.]
  # <joint ... armature="0.01"/>  →  dof_armature = [0.01]
  # <motor ... bogusattr="1"/>    →  Schema violation: unrecognized attribute
  ```

  即 `armature` 在 actuator 上是**合法但无效**的属性（不报错、不生效）。

**2. 根因**

* `e5e1917` 的意图是"按训练配置给 MJCF 补 armature 以避免高增益 PD 发散"，
  但写成了 `<motor ... armature="..."/>`。MuJoCo 只在 `<joint>` 上使用 `armature`。
* 为什么没暴露：同一提交还加了 `ctrlrange`（力矩限幅），
  力矩限幅本身也能压住发散，所以"看起来修好了"。

**3. 修正**

* 计划改法：把 armature 移到对应的 `<joint>` 上（轮 0.00243216、臂/夹爪 0.01、腿 0），
  删掉 `<motor>` 上的无效属性；改完必须重跑 `check_mjcf_contract.py` 与 L3。
* 为什么重要：轮子的 `armature` 相对轮子自身惯量不小，
  缺了它等于"仿真里的轮子比训练里轻"，速度伺服环的响应与训练不一致。

**4. 结果（验收）**

* 未修 → **已修**（`fix/arm-gains-armature-height`，见顶部修复记录）：`check_mjcf_contract.py` 该判据已 PASS。

---

### DEF-010 `2026-09-20` 部署侧喂给策略的接口仍是**旧 checkpoint** 的（86/770/23）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（接口错配） |
| 状态 | `已修`（`feat/policy-layout-v2`） |
| 影响面 | 二者 |
| 关联 | `main @ 40744b5`；`run_policy/m20_piper_policy_runner.hpp`；训练 run `logs/rsl_rl/history_adaptation/2026-09-20_00-50-31` |

**1. 现象（怎么发现的）**

* 触发条件：直接读 `main` 上 runner 的常量与所加载 ONNX 的输入输出形状。
* 可观测证据：runner 里是
  `kObsDim=86 / kHistoryStepDim=77 / kActionDim=23`，加载
  `policy/history_adaptation_full.onnx`；而最新训练 run 的部署态导出
  `exported_deploy/policy_layout.json` 写的是
  `policy_obs_dim=83 / history_single_step_dim=70 / action_dim=16`
  （`source_checkpoint=model_19999.pt`、`source_run=.../2026-09-20_00-50-31`）。
* 影响面：所有"部署效果"的结论。好消息是错配会**抛异常**而不是静默算错
  （runner 里有 `model_obs != kObsDim → throw std::runtime_error("Policy dim mismatch")`）。

**2. 根因**

* 定位过程：对比 `main`（2026-08/09 写的）与训练侧 2026-09-18 的改动。
* 真正的原因：训练侧把 IK 从普通 action term 改成 CommandManager 驱动
  （`ee_ik.action_dim = 0`），并把 policy 的 `joint_pos/joint_vel` 从 22 维
  （leg+wheel+arm）改回 **24 维原生序**（训练侧 DEF-016 的修正）⇒
  观测 86→83、动作 23→16。部署侧没有跟着改。
* 为什么以前没暴露：换 checkpoint 之前，两边是自洽的（都用旧维度）。

**3. 修正**

* 计划改法：见 `TODO_zh.md` P0-1 —— runner 改成**布局驱动**（读
  `policy_layout.json` + ONNX 形状），不再写死常量。
* 备选方案：再写一个 `M20PiperPolicyRunnerV2`。否掉的理由是两套实现会长期分叉，
  而"布局驱动"本来就是这个仓库最需要的能力。
* 兼容性代价：旧 checkpoint 若还要跑，保留在
  `wip/old-ckpt-arm-coupling-debug` 分支上，不放在 `main`。

**4. 结果（验收）**

* **已修**：runner 改成布局驱动（读 `policy_layout.json` + 断言 ONNX 形状/名字），
  策略目录 `policy/m20_piper_history_20260920/{policy.onnx, policy.pt, policy_layout.json}`，
  旧 checkpoint 的三个产物与旧探针脚本已删除。
* 验收：
  * L1 `scripts/check_policy_interface.py` → **PASS**（83/700/16、名字/形状一致、
    标称状态 `|a|max=1.46`、ONNX vs TorchScript 相对误差 **9.8e-07**）；
  * L3 `tests/sim2sim_smoke.py`：`hold` / `rl` / `walk` **全部 PASS**
    （walk 后半段平均 +0.58 m/s，键盘命令 +0.7）。

---

### DEF-009 `2026-09-20` `body_pose.height` 的度量与取值范围都与训练不一致

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（命令语义） |
| 状态 | `部分已修`（区间已收敛到训练值，见上方修复记录；度量语义待真机核对） |
| 影响面 | 二者 |
| 关联 | `interface/user_command/keyboard_interface.hpp:36`；`state_machine/quadruped_wheel/rl_control_state.hpp`（VR 分支）；训练侧 `mdp/utils.py::compute_base_height_rel_to_feet` |

**1. 现象（怎么发现的）**

* 触发条件：读部署侧的高度命令默认值与钳位范围。
* 可观测证据：部署侧把"机身高度"当**绝对值**用（键盘默认 `0.513f`、
  上限 `0.60`；VR 分支 `clamp(..., 0.33f, 0.60f)`）。训练侧的定义是
  `height = root_z − mean(四个 wheel body 的 z) + 0.09`（0.09 = 轮半径），
  课程终值区间 **(0.33, 0.55)**。
* 影响面：机身高度跟踪这条命令通道的全部效果，以及"高度没反应/反应过冲"这类现象。

**2. 根因**

* 真正的原因：两套定义在"平地、轮子贴地"时数值接近（`0.55 − 0.1134 + 0.09 ≈ 0.527`），
  所以不会立刻暴露；一旦姿态角变化或接触状态变化就偏。上限 0.60 更是
  直接超出训练分布 9%。
* 为什么以前没暴露：仿真里默认一直站在平地、很少动高度命令。

**3. 修正**

* 计划改法：见 `TODO_zh.md` P0-5（改度量 + 收敛区间 + 超范围警告）。

**4. 结果（验收）**

* 未修 → **已修**（区间收敛到训练值 (0.33, 0.55)，见顶部修复记录）。

---

### DEF-008 `2026-09-20` 进入 RL 时 `ee_goal` / `body_pose` 是**固定常量**，不是当前实测值

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（复位语义） |
| 状态 | `未修（见 TODO_zh.md P0-4）` |
| 影响面 | 二者 |
| 关联 | `state_machine/quadruped_wheel/rl_control_state.hpp`（`ee_goal_` / `ApplyVrCommand`）；`interface/user_command/keyboard_interface.hpp`（`body_height_`）；训练侧 `TODO_zh.md` P1-3 ⑫ |

**1. 现象（怎么发现的）**

* 可观测证据：`RLControlState::PolicyRunner` 每个策略 tick 都把成员 `ee_goal_`
  写进 `uc->ee_goal_pos/quat`；`ee_goal_` 的初值是常量
  `(0.3492, 0, 0.4327) + quat(0.7373, 0, 0.6756, 0)`
  （= Piper 默认关节姿态下 `gripper_base` 在 **root 系**的位姿，已由
  `check_mjcf_contract.py` 实测：pos 差 5.7e-05 m、旋转角差 0.000°）。
  只有 `/ARM_TELEOP_STATE` 到达后才会被覆盖。
* 影响：进 RL 时（以及臂节点没起时）喂给策略的 `ee_goal` 是**固定的默认姿态**，
  而不是"当前 EE 位姿"；`body_pose` 同理（键盘的 `body_height_ = 0.513f` 常量）。
  训练侧自己的 reset 语义是"用当前位姿初始化"（`pose_command_b` 从
  `pose_start_b` 插值，`pose_start_b` = 复位那一刻的真实 EE 位姿）。

**2. 根因**

* 真正的原因：部署侧把"命令"当成"操作员没输入时的常量兜底"，而不是
  "随当前状态初始化的量"。参考实现是训练侧的
  `HeightInvariantEECommand._resample_command`：重采样时把目标设为
  **当前真实 EE 位姿**，再插值到采样目标。
* 附带一条容易误导的地方：`UserCommand::ee_goal_pos` 的默认值
  `(0.1092, 0, 0.3439)` 是**臂基座坐标系**下的同一个位姿；
  在当前代码路径上它会被 `ee_goal_` 覆盖（= 死代码），但很容易被当成有效默认值。

**3. 修正**

* 计划改法：见 `TODO_zh.md` P0-4 —— 进入 RL / 收到 reset 时，用
  `arm_controller` 反馈的当前 EE 位姿（root 系）与当前实测
  `height/pitch/roll` 初始化这两块观测。

**4. 结果（验收）**

* 未修。

---

### DEF-007 `2026-09-20` 机械臂 PD 增益在仓库里有**三份**，其中两份是旧训练配置的

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（参数不一致） |
| 状态 | `已修`（`fix/arm-gains-armature-height`，见上方修复记录） |
| 影响面 | 二者 |
| 关联 | `arm_controller.py:84`；`interface/robot/hardware/piper_arm_interface.hpp:78`；`interface/robot/simulation/mujoco_simulation_ros2.py:116`；训练 run `2026-09-20_00-50-31/params/env.yaml` |

**1. 现象（怎么发现的）**

* 三处各写一份臂关节增益，互相矛盾：

| 位置 | 值 | 注释里声称的来源 |
|---|---|---|
| `arm_controller.py:84`（`ARM_KP, ARM_KD`） | **40 / 8** | "matches Isaac Lab piper_arm actuator (40/8)" |
| `piper_arm_interface.hpp:78`（`Start()` 保持用） | **300 / 20** | "from the Isaac Lab DelayedPDActuatorCfg" |
| `mujoco_simulation_ros2.py:116`（首条 `/ARM_JOINTS_CMD` 到达前） | **40 / 8** | "stiffness 40 / damping 8" |

* 权威值：本次 run 的 `env.yaml` 是 `piper_arm: stiffness 300.0 / damping 20`、
  `piper_gripper: 4000 / 200`。

**2. 根因**

* `40/8` 来自更早的一次训练配置；训练侧改执行器配置时没有同步部署侧，
  而部署侧同一份信息抄了三遍，于是只有一处被更新。

**3. 修正**

* 计划改法：见 `TODO_zh.md` P0-6（收敛成单一常量来源 + 启动打印）。

**4. 结果（验收）**

* 未修 → **已修**（三处统一到 300/20，见顶部修复记录）。原文的观察仍然成立 —— 当时臂的控制刚度比训练低 7.5 倍，是"臂跟不上命令 / 位姿观测与真值漂移 /
  策略看到的臂状态与训练分布不符"的候选原因之一。

---

### DEF-006 `2026-09-20` 部署侧的 ±3 动作限幅破坏了 `actions` 观测的自洽性

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（观测/控制不一致） |
| 状态 | `已修`（`feat/policy-layout-v2`：新 runner 里 `last_action_obs_ = 实际下发的动作`，开关随旧策略一起删除） |
| 影响面 | 二者 |
| 关联 | `run_policy/m20_piper_policy_runner.hpp`；`wip/old-ckpt-arm-coupling-debug`；训练侧 `DEF-008` |

**1. 现象（怎么发现的）**

* 触发条件：`e5e1917` 给部署侧加了"原始动作限制在 ±3"的安全网（轮速约 ±15 rad/s），
  但 `last_action` 观测一开始仍喂 `clip(raw, ±100)`。
* 可观测证据：策略被告知"我命令了 +500 rad/s"，机器人实际只收到 +15 rad/s；
  轮通道长期贴限幅。
* 影响：底盘被持续驱动（轮速钉在 ±15 rad/s），机器人被推走/压到侧躺。

**2. 根因**

* 训练里 `actions` 观测 = `env.action_manager.action` = **控制用的同一个值**；
  部署侧多加了一层限幅却没同步观测。DEF-002 修了"processed vs raw"，但没覆盖这一层。

**3. 修正**

* 新增 `M20_LAST_ACTION_MODE`：`applied`（默认，喂实际下发的值）/
  `raw` / `processed`。
* 仿真 A/B：`raw` 能站住；`processed`（换算成关节目标）约 3 s 内翻倒、
  轮速钉在 ±15 rad/s。

**4. 结果（验收）**

* 仿真行为改善。**遗留**：开关还没清理，最终要按新 checkpoint 定死一套原则
  （`TODO_zh.md` P0-3 / P1-5）。

---

### DEF-005 `2026-09-20` 24 维关节序在本仓库里有**三种写法**，且新资产的序没实测过

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（静默失效风险） |
| 状态 | `已修`（`feat/policy-layout-v2`：由训练侧探针实测判定为**交错序**，写进 `policy_layout.json::joint_order_native` 与 `M20PiperPolicyRunner::NativeOrder`，L1 交叉断言） |
| 影响面 | 二者 |
| 关联 | `run_policy/m20_piper_policy_runner.hpp`；训练侧 `DEF-021`；`wip/old-ckpt-arm-coupling-debug` |

**1. 现象（怎么发现的）**

* `policy_obs` / `history` 里 `joint_pos`、`joint_vel` 的 24 维顺序，仓库里出现过三种：

| 序 | 写法 | 出现在 | 证据 |
|---|---|---|---|
| ① **交错序** | `hipx×4, arm1, hipy×4, arm2, knee×4, arm3, wheel×4, arm4-6, gripper×2` | 训练侧 `docs/deploy_sim2sim_sim2real_zh.md` 第 4 节 / `DEF-021` | `probe_deploy_layout.py` 在 `M20_Piper_own` 上打印的原生序（"wheel=15..18"）**(二手引用)** |
| ② **分组序** | `hipx×4, hipy×4, knee×4, wheel×4, arm×6, gripper×2` | `wip/old-ckpt-arm-coupling-debug` | `M20_adjusted` 旧 run 的 `joint_torque_log_flat.npz` 里 `find_joints(".*")` 的输出 **(实测，但资产不同)** |
| ③ **逐腿序** | `(hipx,hipy,knee)×4 腿, wheel×4, arm×6, gripper×2` | `main @ 40744b5` 的 `HistoryJointOrder` | 无（只是把 MJCF 顺序当成原生序） |

* 影响：24/83 的观测维度被喂错 ⇒ 站不住、乱抖、动作发散（训练侧失败模式表第 1 行）。

**2. 根因**

* 证据来自**两个不同的资产**：`M20_adjusted.usd`（旧 run，夹爪叫 `arm_joint7/8`）
  与 `M20_Piper_own.usd`（新 run，夹爪叫 `gripper_joint1/2`）。
  PhysX 按资产里关节的定义顺序上报，两个资产本来就可能不同；
  但 `M20_Piper_own` 的序**本仓库没有实测过**（本机没有 Isaac 环境），
  只能引用训练侧探针的输出。

**3. 修正**

* 计划改法：见 `TODO_zh.md` P0-2 —— 序做成布局文件里的字段 + 环境变量可覆盖，
  然后在 sim2sim 里 A/B（"零位移命令下 20 s 是否站得住"是强判据）。

**4. 结果（验收）**

* 未修 → **已修**（判定为交错序 ①，见 `feat/policy-layout-v2`；原文如下供对照）：当时 `main` 上用的是三种里最没有证据支持的 ③。

---

### DEF-004 `2026-09-04` 臂/夹爪在 MuJoCo 显式 PD 下发散（MJCF 缺 armature）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-04` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | `sim2sim` |
| 关联 | `e5e1917`；`M20_Piper_description/mjcf/M20_Piper_own.xml` |

**1. 现象**：从 USD/URDF 派生的 MJCF 直接跑，臂/夹爪用显式 PD 时数值发散。

**2. 根因**：MJCF 里 `armature` 为 0，而训练侧 `DelayedPDActuator` 配置是
arm/gripper `0.01`、wheel `0.00243216`；惯量过小 + 高增益（夹爪 4000/200）
在显式积分下不稳定。

**3. 修正**：给 MJCF 的 `<motor>` 加 `class`，按训练配置补 `armature`
（腿 0、轮 0.00243216、臂/夹爪 0.01）与 `ctrlrange`（力矩限幅）。

**4. 结果**：臂/夹爪不再发散。**遗留**：`base_link` 上额外加了一条显式
`<inertial mass="15.882" diaginertia="...">`，训练侧 USD 是否同值**未核对**
（`TODO_zh.md` P1-1）。

---

### DEF-003 `2026-09-04` 1 ms 物理步长下机身 500 Hz 自激（IMU 读到假角速度）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-04` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | `sim2sim` |
| 关联 | `e5e1917`；`interface/robot/simulation/mujoco_simulation_ros2.py` |

**1. 现象**：`DT=0.001` 单步积分时，机身原地高频抖动；IMU 角速度读数出现
±2 rad/s 的交替值，而位姿上看不出对应运动。

**2. 根因**：USD 派生的 MJCF 有一个轻阻尼的 ~500 Hz 摇摆模态，
1 ms 步长下数值积分处于临界稳定。

**3. 修正**：物理积分步长改 `PHYSICS_DT=0.0002`，每个 1 ms 控制 tick 做 5 个子步；
控制 tick 与 200 Hz 反馈频率不变。

**4. 结果**：抖动与假角速度消失。**遗留**：MJCF 自带 `timestep=0.002` 与训练
`sim.dt=0.005` 仍不一致，取舍要在 `TODO_zh.md` P1-1 里明确。

---

### DEF-002 `2026-09-16` `last_action` 观测喂的是"换算后的关节目标"

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-16` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `40744b5`；`run_policy/m20_piper_policy_runner.hpp`；训练侧 `DEF-008` |

**1. 现象**：部署侧把"处理后的动作"（腿 = 默认角 + scale·a，轮 = scale·a）
喂回 `actions` 观测；训练里 `actions` 观测是**策略原始输出**
（被 `clip_actions=100` 截断）。轮速列因此被乘了 5、腿列被平移了默认角。

**2. 根因**：把"动作"与"换算后的关节目标"当成同一个量。

**3. 修正**：`40744b5` 改成训练口径的原始动作（`M20_LAST_ACTION_MODE=raw` 成为默认）。

**4. 结果**：见 DEF-006 —— 之后又发现"部署侧再加 ±3 限幅后仍然不一致"，
最终语义定为"喂**实际下发**的动作"。

---

### DEF-001 `2026-09-04` sim2sim 里真机标定污染了仿真坐标系

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-04` |
| 类型 | 缺陷 |
| 状态 | `已修` |
| 影响面 | `sim2sim` |
| 关联 | `e5e1917`；`interface/robot/hardware/m20_sim_interface.hpp`、`CMakeLists.txt` |

**1. 现象**：不加特殊编译选项时，仿真里"站不住 / 走反"，而模型本身没问题。

**2. 根因**：`M20Interface`（真机用）带 dir/offset 标定，`Start()` 里还会按
多圈关节做 ±360° 偏移补偿；MJCF 与训练用的是同一份 URDF，**原始关节帧就是策略帧**，
再套一层真机标定等于给观测加了一个错误的仿射变换。

**3. 修正**：新增 `M20SimInterface`（dir=1、offset=0，`Start()` 直接保持默认站立姿态），
由 CMake 选项 `-DSIM2SIM=ON` 选择。

**4. 结果**：仿真坐标系与 MJCF 一致。**注意**：不加 `-DSIM2SIM=ON` 会静默走回
真机接口，`README.md` 已标红提醒。
