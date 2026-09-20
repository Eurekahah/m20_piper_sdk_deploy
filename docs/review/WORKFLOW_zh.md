# 开发 / 分支 / 调试 / 测试规范

**文档职责**：规定"怎么改这个仓库"。管：分支模型、提交规范、模块边界、
每个模块改动后**必须**跑的测试、debug 开关的登记规则、合并回 `main` 的门槛。
不管：具体待办（看 `TODO_zh.md`）、已做过什么（看 `DONE_zh.md`）、
每个缺陷的来龙去脉（看 `DEFECT_LOG_zh.md`）、接口契约（看
`docs/sim2sim_layout_contract_zh.md`）。

**维护约定**：见 `templates/DOC_TEMPLATE_zh.md`。

## 更新记录

| 日期 | 更新内容 | 相关 commit / 分支 |
|---|---|---|
| 2026-09-20 | 初版：分支模型 + 模块边界 + 测试分层 + debug 开关登记表 | `docs/review-spec` |

---

## 0. 唯一不变量

> **`main` 任何时候都必须处于"能编译 + 能过 L1 离线验收 + 能过 L3 sim2sim 冒烟"的状态。**

任何让它不满足的改动，只能待在功能分支上。判断标准是下面第 5 节的"合并门槛"，
不是"我觉得没问题"。

---

## 1. 分支模型

| 分支 | 用途 | 生命周期 |
|---|---|---|
| `main` | 随时可编译、可跑 sim2sim 的主线 | 永久；只接受过了合并门槛的合并 |
| `feat/<topic>` | 新功能 / 新模块（例：`feat/policy-layout-v2`） | 合并后删除 |
| `fix/<topic>` | 修缺陷（例：`fix/arm-pd-gain`） | 合并后删除 |
| `docs/<topic>` | 只改文档 | 合并后删除 |
| `wip/<topic>` | **实验/调试现场**：可能编译不过、可能半成品 | **不合并**，长期保留供 `git show` 查证 |

命名用小写 + 连字符，`<topic>` 要能看出在改什么（不接受 `feat/test1`）。

历史约定：仓库早期直接在 `main` 上连做 21 个提交（`origin/main..main`），
`docs/M20_Piper_deploy_6commits_zh.md` 记录的是其中最早 6 个。从本规范起，
新工作一律走功能分支。

### 已存在的 `wip` 分支

| 分支 | 内容 | 什么时候还能用 |
|---|---|---|
| `wip/old-ckpt-arm-coupling-debug` | 上一轮针对**旧 checkpoint**（`policy_obs 86 / history 770 / action 23`，`M20_adjusted` 资产）的臂→轮耦合诊断：一批 `M20_*` 调试开关、`last_action` 三种模式、body-frame `ee_goal`、README 重写 | 当需要复现"旧 checkpoint 为什么压不住"时；**不要**把它合回 `main` |

---

## 2. 提交规范

沿用仓库既有的 conventional-commit 前缀，一行主题用英文，正文可中文：

```
<type>(<scope>): <一句话主题>

<为什么这么改 / 关键实测数字 / 关联 DEF-0xx>
```

* `type` ∈ `feat` / `fix` / `docs` / `refactor` / `test` / `chore`。
* `scope` 建议用模块名：`rl` / `sim` / `arm` / `vr` / `teleop` / `safety` / `docs`。
* **一个提交只做一件事**，并且这次提交必须自带"怎么验证它"的信息
  （写进提交正文或对应的 `DEF-0xx`）。
* 涉及接口契约（关节序、坐标系、增益、话题、观测维度）的提交，
  必须在正文里写清"改前 → 改后"，否则 review 无法判断。

---

## 3. 模块边界（改哪个模块 → 跑哪些测试）

| 编号 | 模块 | 主要文件 | 改动后必须跑 |
|---|---|---|---|
| M1 | **策略接口层**（观测组装 / 动作映射 / history 窗口 / ONNX 会话） | `run_policy/m20_piper_policy_runner.hpp`、`run_policy/policy_runner_base.hpp` | L1 + L3 |
| M2 | **状态机与安全**（idle/standup/rl/liedown、接管阈值、急停） | `state_machine/quadruped_wheel/*`、`include/utils/safe_controller.hpp` | L0 + L3 |
| M3 | **腿部接口 / 标定**（`/JOINTS_CMD`/`/JOINTS_DATA`、dir/offset） | `interface/robot/hardware/{m20_interface,m20_sim_interface,dds_interface}.hpp` | L3 |
| M4 | **MuJoCo 仿真**（物理步长、执行器、传感器、`/reset_sim`） | `interface/robot/simulation/mujoco_simulation_ros2.py`、`M20_Piper_description/mjcf/*.xml` | L3 + L3'（参数对照） |
| M5 | **机械臂 IK / 遥操作**（DLS IK、限位、增益、`/ARM_*` 话题） | `interface/robot/simulation/arm_controller.py`、`arm_teleop_node.py`、`hardware/piper_arm_interface.hpp` | L2 + L3 |
| M6 | **仿真/真机传输适配**（`arm_real_adapter`、agx 桥接） | `interface/robot/simulation/arm_real_adapter.py` | L2（话题回环）+ 真机 L4 |
| M7 | **VR 遥操作** | `interface/robot/simulation/vr_teleop_*.py` | L2 + L3（连上 VR 后走一遍） |
| M8 | **文档 / 工具脚本** | `docs/**`、`src/M20_sdk_deploy/scripts/**` | L0（不破坏编译）+ 脚本自跑一遍 |

---

## 4. 测试分层

| 层 | 名字 | 跑什么 | 耗时 | 何时跑 |
|---|---|---|---|---|
| **L0** | 编译 | `colcon build --packages-up-to m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON` | ~20 s | 任何代码改动 |
| **L1** | 离线策略验收 | `python3 src/M20_sdk_deploy/scripts/check_policy_interface.py`（ONNX ↔ TorchScript 数值对照、维度/名字断言、标称观测下的动作幅值） | ~5 s | 改 M1 / 换策略时 |
| **L2** | 单节点话题回环 | 只起被测节点 + `ros2 topic echo`，人为发命令看反馈（不需要整个 sim2sim） | ~30 s | 改 M5 / M6 / M7 |
| **L3** | 端到端 sim2sim | `rl_deploy` + `mujoco_simulation_ros2.py` 无头跑 N 秒，按第 5 节判据打分 | ~1 min | 改 M1~M5 后**必跑** |
| **L3'** | 参数对照 | 仿真侧参数 vs 训练侧 `params/env.yaml`（执行器增益、armature、限位、默认角、观测 scale/clip）逐条表格化比对 | ~5 min（人工） | 改 M4 后 |
| **L4** | 真机 | 仅在真机上做；必须有急停/吊装/阻尼兜底 | — | 合并到 `main` 之后、且 L3 全绿 |

### L3 的判据（数值验收）

统一用同一套定义，避免"看起来能站"：

* **躯干高度**：`root_z − mean(四个 wheel body 的 z) + 0.09`（0.09 = 轮半径；
  与训练 `mdp/utils.py::compute_base_height_rel_to_feet` 同定义）。
* **倾角**：`acos(-g_z)`（`g` = 机体坐标系下的重力投影；直立 0，侧躺 90°）。
* **摔倒判据**：倾角 > 0.8 rad **或** 躯干高度 < 0.30 m
  （= 训练终止阈值，真机上同时作为**接管阈值**）。
* 起步验收：`base_velocity = 0`、`body_pose = 当前实测`、`ee_goal = 当前 EE 位姿` 时，
  20 s 内不触发摔倒判据，且躯干高度稳态误差 < 0.03 m。

> 这些判据已有可执行入口（2026-09-20 起）：

```bash
# 一键（L0 编译 + L1 + L3' + 六档 L3）：
bash tests/run_all.sh                 # 完整
SKIP_BUILD=1 MODES="hold rl" bash tests/run_all.sh   # 只跑子集

# 手动逐档：
# 容器内，仓库根目录
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 tests/sim2sim_smoke.py --mode hold --duration 12   # ① 裸模型站立
python3 tests/sim2sim_smoke.py --mode rl   --duration 25   # ② 站立 → 进 RL（零命令）
python3 tests/sim2sim_smoke.py --mode walk --duration 25   # ③ 进 RL 后按 w 前进（判速度）
python3 tests/sim2sim_smoke.py --mode arm  --duration 25   # ④ 额外起 arm_controller（判臂位姿/力矩）
python3 tests/sim2sim_smoke.py --mode arm_move --duration 25  # ⑤ 按住 numpad 动臂（判臂链路）
python3 tests/sim2sim_smoke.py --mode push --duration 25   # ⑥ 800 N 侧推（判安全接管触发）
python3 tests/sim2sim_smoke.py --mode rl --duration 20 --repeat 6  # 入口失败率（判 <= 1/3）
# PASS/FAIL + 高度/倾角数字；遥测 CSV 落在 /tmp/m20_sim2sim.telemetry.csv

# ⚠️ 2026-09-20 起：**连续跑五档约有 1/3 概率在 `rl` 档偶发摔倒（DEF-018，未定位）**。
#    在那之前：单档跑可以当验收，`tests/run_all.sh` 还不能当合并门槛（TODO P0-9）。
```

别只靠肉眼看 viewer：`DEF-012`/`DEF-013` 都是"看着像策略不行、其实是进场流程错"。

**并行注意**：仿真的控制循环是**墙钟驱动**的（`if time.time() - last >= 1ms`），
同机并发跑两个实例会把控制周期拉长，结果不可复现（实测同一配置一次 PASS 一次 FAIL）。
`sim2sim_smoke.py` 会在开跑前检查残留进程并直接拒绝。

---

## 5. 合并门槛（合回 `main` 前逐条打勾）

1. `git log --oneline main..HEAD` 里每个提交都只做一件事，且主题能看懂。
2. L0 通过（编译无 error）。
3. 按第 3 节表格，跑过该模块要求的测试层，**并且把数字写进 `DONE_zh.md` 或 `DEF-0xx`**。
4. `TODO_zh.md` 里对应条目已经迁移/勾选；新引入的缺陷/特性在 `DEFECT_LOG_zh.md` 有记录。
5. 新增的环境变量、话题、编译选项都登记进了 `README.md`（见第 6 节）。
6. 不把调试开关、临时打印、被注释掉的实验代码留在默认路径上
   （要么删掉，要么用默认关闭的开关包起来并登记）。
7. 用 `git merge --no-ff` 合并（保留分支形状，方便 `git log --graph` 回看），
   合并提交信息里写清"这次合入的验收是什么"。

---

## 6. debug 规范

### 6.1 开关登记表（新增开关必须在这里加一行）

| 开关 | 作用 | 默认 | 归属 |
|---|---|---|---|
| `M20_USE_VIEWER` | 是否开 MuJoCo 窗口 | `1` | 仿真 |
| `M20_SIM_TELEMETRY` / `M20_SIM_TELEMETRY_PERIOD` | 遥测 CSV 路径 / 采样周期（tick） | 关 / `5`（=200 Hz） | 仿真 |
| `M20_JVEL_DEBUG` / `M20_JVEL_PERIOD` | 打印腿/轮关节速度（1 Hz） | `1` / `50` tick | 仿真+部署 |
| `M20_PIPER_DEBUG` | 把前 300 个 policy tick 的 obs/action dump 到 `policy_debug.txt` | 关 | 策略 |
| `<del>M20_LAST_ACTION_MODE / M20_IK_FEEDBACK / M20_FREEZE_* / M20_ARM_DEFAULT_TRAINED / M20_ZERO_CMD_WHEEL_BRAKE</del>` | 旧 checkpoint 期的诊断开关（16 维动作、无 `ee_ik`）——**已随旧策略一起删除** | — | 策略（历史） |
| `M20_EE_GOAL_BODY_FRAME` | `ee_goal` 用机体坐标系（`1`）还是臂基座坐标系（`0`） | `1` | 机械臂 |
| `M20_POLICY_DIR` | 策略目录（含 `policy.onnx` + `policy_layout.json`） | `policy/m20_piper_history_20260920` | 策略 |
| `M20_POLICY_ONNX` / `M20_POLICY_LAYOUT` | 单独覆盖 onnx / layout 路径 | 取自 `M20_POLICY_DIR` | 策略 |
| `M20_ACTION_CLIP` | 动作安全限幅（训练 `clip_actions` = 100） | `100` | 策略 |
| `M20_TILT_TAKEOVER` | 安全接管的倾角阈值 [rad]（训练终止阈值；`<=0` 关闭） | `0.8` | 安全 |
| `M20_LEG_FOLD_TAKEOVER` | 安全接管的"腿折叠"阈值 [rad]（`|q-q_default|`；`<=0` 关闭） | `1.2` | 安全 |
| `M20_SIM_PUSH_FORCE` / `_AT` / `_DURATION` | 仿真侧扰动注入（`xfrc_applied` 作用在 `base_link`） | `0`（关）/ `12 s` / `0.3 s` | 仿真 |
| `M20_ARM_TRAINED_DEFAULT` | 仿真里臂初始位姿用训练资产的默认角 | 关 | 仿真 |
| `M20_ARM_ROT_LOCAL_FRAME` / `M20_ARM_ROLL_SIGN` | 遥操作旋转合成方式（A/B 用） | 与训练一致 | 机械臂 |
| `M20_EE_MAX_LIN_SPEED` / `M20_EE_MAX_ANG_SPEED` | 键盘积分路径的 EE 限速 | `0.35 m/s` / `0.8 rad/s` | 机械臂 |
| `M20_VR_MAX_LIN_SPEED` / `M20_VR_MAX_ANG_SPEED` | VR 绝对偏移路径的 **目标** 限速 | `2.0 m/s` / `5.0 rad/s` | 机械臂 |
| `M20_ARM_DEBUG` / `M20_ARM_DEBUG_PERIOD` | 打印 IK 残差（目标 vs FK），用于 P1-4 精度验收 | 关 / `50` tick | 机械臂 |
| `M20_SIM_WHEEL_STEP_RAD_S` / `_AT` | 仿真侧轮子速度阶跃（P1-2 验收） | `0`（关）/ `5 s` | 仿真 |

> 上面这些开关多数是为**旧 checkpoint** 的诊断引入的（在
> `wip/old-ckpt-arm-coupling-debug` 上）。换到新 checkpoint 后要逐条判定
> "还需要吗"，不需要的连同代码一起删（`TODO_zh.md` P1）。

### 6.2 临时调试代码

* 一次性诊断不许直接改默认路径。要么**加开关 + 登记**，要么写在
  `wip/<topic>` 分支上。
* 打印行统一带前缀 `[<模块>]`（`[JVEL]` / `[JVEL-SIM]` / `[arm_controller]` ...），
  方便从多终端日志里 grep。
* 高频打印必须限频（≥ 1 s 一次）或默认关闭；`/JOINTS_CMD` 回调级别禁止打印。
* 诊断结束把 **现象 → 根因 → 修正 → 数字** 写进 `DEFECT_LOG_zh.md`，
  代码里的"TEMP DEBUG"整段删掉。

### 6.3 排查顺序（当"策略跑不住"时）

按这个顺序排除，每步只动一个变量：

1. **接口是否对齐** —— 关节顺序/坐标系/增益/scale 与
   `docs/sim2sim_layout_contract_zh.md` 逐条核对（先别怀疑策略）。
2. **开环动作** —— 绕过策略发固定关节目标，确认 PD/轮速伺服/接触正常。
3. **零动作** —— 策略输出全 0 ⇒ 关节目标 = 默认角，应保持站立。
4. **接策略但命令全零位移** —— `base_velocity=0`、`body_pose=当前实测`、
   `ee_goal=当前 EE 位姿`、history 用第一帧填满；应当接近站立。
5. **逐个放开** —— 先小 EE 步进，再小底盘速度，最后放到训练区间。

任一步不通过就停在那一步，把现象记进 `DEFECT_LOG_zh.md`，不要跳步。

---

## 7. 环境备忘

```bash
# 运行环境：docker 容器 m20_piper_ros（镜像 m20-piper-deploy:latest）
docker start m20_piper_ros
docker exec -it m20_piper_ros bash

# 容器内：宿主机仓库挂在 /root/m20_piper_ws
cd /root/m20_piper_ws
source /opt/ros/humble/setup.bash

# 编译（sim2sim）
colcon build --packages-up-to m20_sdk_deploy \
  --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON
```

```bash
# sim2sim：3~4 个终端，每个都要先 export ROS_DOMAIN_ID=1 + source install/setup.bash
ros2 run m20_sdk_deploy rl_deploy                                   # T1 状态机 + 策略
python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py   # T2 仿真
python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py           # T3 机械臂 IK（可选）
python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py          # T4 机械臂键盘（可选）

# 无头跑（CI / 冒烟）：M20_USE_VIEWER=0，必要时 MUJOCO_GL=egl 或 osmesa
```

交互（T1 键盘）：`z` 站立 → 站立完成后 `c` 进 RL；`wasd/qe` 底盘速度；
`h/j` 高度、`b/n` 俯仰、`[`/`]` 横滚；`r` 阻尼、`x` 趴下。
