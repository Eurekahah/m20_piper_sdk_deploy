# 代码走查：缺漏与不优雅之处（2026-10-09）

**范围**：`src/M20_sdk_deploy/` 全部第一方代码（C++ ~2.6k 行 + Python ~2.5k 行 + CMake），
排除 `include/utils/json.hpp`（第三方）与 `third_party/`。`tests/`、`scripts/` 一并看了。

**方法**：静态阅读 + 引用计数（grep 判定"定义了但没人用"）+ 今晚的实测
（`tests/command_sweep.py`、`tests/sim2sim_smoke.py`、L1/L3' 验收）。每条都给 `文件:行`。
**除下面 §0.3 提到的两处小改外没有动代码** —— 这份文档是给你决策用的。

**一句话结论**：部署链路（状态机 → 接口 → 策略 runner）结构清楚、文档和验收比一般项目强；
主要问题三类：① **安全层有一半没接线**；② **同一份常量抄了 6~8 遍**（历史上已因此踩过
DEF-007/029/030）；③ **一批死代码/死分支**没清。

---

## 0. 摘要（按优先级）

| # | 类别 | 位置 | 问题 | 影响 | 建议 |
|---|---|---|---|---|---|
| 1 | **缺漏（安全）** | `safe_controller.hpp:212`/`:223` | `IsBatteryNormal()`、`IsUserCommandNormal()` **定义了但从未被调用** ⇒ 低电量、手柄/键盘心跳超时**实际没生效** | 真机断连/低电量不会触发保护 | 在 `Run()` 里接上 + 补假反馈用例 |
| 2 | **缺漏（安全）** | `safe_controller.hpp:101,134,157` | `imu_update_overtime`/`joint_update_error` 只被赋 0、**永不置 1**；关节"数据不更新"检测被注释掉 | 传感器/反馈卡死检测形同虚设 | 实现或删除，别留假保护 |
| 3 | **缺漏（语义）** | `common_types.h:91` vs `safe_controller.hpp:371-389` | `safe_control_mode` 注释是"0 normal,1 stand,2 sit,3 damping"，实际当**错误码**写 2/3 | 语义重载，必被误读 | 拆字段或改注释 |
| 4 | **缺漏（单位）** | `safe_controller.hpp:366` + `:235` | `current_time_` 是**秒**，心跳判据却写 `> 1000.0` | 接上后超时=1000 s（≈永不触发） | 统一到 ms 或秒 |
| 5 | **缺漏（测试）** | `tests/run_all.sh:19` | 新用例 `command_sweep.py` 未纳入；且测不到 **vy/wz/姿态组合** | 覆盖盲区（见 §7） | 挂进 `run_all.sh` |
| 6 | **不优雅（重复）** | 8 处默认站姿 + 6 处默认 EE 位姿 | 见 §3.1 | 改一处漏三处 | 抽单一来源 |
| 7 | **不优雅（UB 味）** | `motion_state_feedback.hpp:36`、`state_base.h:44` | 构造期 `memset(this,...)`；且每个 State 构造都 memset 那个 static 实例 | 非平凡对象 memset；会清掉全局状态 | 用初始化列表 / reset 接口 |
| 8 | **不优雅（并发）** | `common_types.h:74` `UserCommand` | 键盘线程 / VR 回调 / 策略线程 / 安全线程同读写，只有部分字段有 mutex | 理论会撕裂 | 至少写明字段所有权 |
| 9 | **不优雅（构建）** | `CMakeLists.txt:14`、`interface/CMakeLists.txt:6` | `add_definitions(-w)` **关掉全部警告** | 真 bug 被吞 | 至少 `-Wall -Wextra` |
| 10 | **死代码** | 见 §4（约 700 行） | 旧机型 runner、`car_move`、`keyboard_interface_sim.hpp`、`backward.hpp` 分支… | 阅读成本 + 误用风险 | 分批删 |

### 0.3 本次已做的两处小改（其余未动）

1. `rl_control_state.hpp:249-258`：策略目录找不到时，错误信息现在打印**解析后的绝对路径 + cwd**
   （起因见 §1.4，我今晚被这个坑掉 25 分钟测试）。
2. 文档里 `M20_POLICY_DIR` 的示例改成绝对路径：
   `COMMANDS_SIM2SIM_zh.md` §7、`policy/README.md` 第 5 步、`WORKFLOW_zh.md` 开关表、
   `DEFECT_LOG_zh.md` DEF-035、`deploy_code_delta_2026-10-09_zh.md` §3。

---

## 1. 功能性缺漏（安全 / 真机相关）

### 1.1 安全层：做了但没接线

`SafeController::Run()`（`safe_controller.hpp:355-405`）实际只调 4 个检查：
`IsDriverStatusNormal` / `IsJointDataNormal` / `IsMotorTempertureNormal` / `IsImuDataNormal`。

| 定义了却从没被调用（全仓库只有定义那一处） | 行 | 本该保护什么 |
|---|---|---|
| `IsBatteryNormal()` | `:212` | 低电量（`battery_data[0] < 10`） |
| `IsUserCommandNormal()` | `:223` | 手柄/键盘心跳超时（断连） |

同文件还有 4 个"永远为 0"的错误位（只被 `PrintRobotErrorState` 读）：
`imu_update_overtime`(`:101`)、`joint_update_error`(`:134`)、
`battery_heat_warn`/`cpu_heat_warn`/`cpu_freq_warn`(`:288-296`)。
另外 `joint_data_same_cnt_`(`:153`) 每周期都在算，但**判定被注释掉**(`:157-161`)。

> 结论：这套安全层**看起来覆盖 6 类故障，实际只覆盖 4 类**。上真机前建议逐条定"实现 or 删除"。

### 1.2 单位混淆会把保护变成失效

`Run()` 里 `current_time_ = GetTimestampMs() / 1000;`（**秒**），而
`IsUserCommandNormal()` 写的是 `current_time_ - cmd_check_time_ > 1000.0`。
按秒算 = 1000 s ≈ 永不超时。今天因为没被调用而无害，一接上就变成"看起来有、实际没有"。

### 1.3 `safe_control_mode` 语义重载

`common_types.h:91` 注释：`0 normal, 1 stand, 2 sit, 3 joint damping`；
实际写入（`:371/377/383/389`）：驱动/温度/IMU 故障写 2，关节数据故障写 3；
读取方（`idle_state:177`、`standup_state:125`、`rl_control_state:433`、`liedown_state:137`）只判 `!=0`。
"1 stand" 从未使用。建议拆字段或至少把注释改成"非 0 = 安全故障"。

### 1.4 `M20_POLICY_DIR` 相对路径口径不一致（今晚实测踩到）

代码默认策略目录按 `__FILE__` 解析（`rl_control_state.hpp:245-250`），
而 `M20_POLICY_DIR` 覆盖值按**进程 cwd** 解析。今晚我用
`M20_POLICY_DIR=policy/m20_piper_rough_20261002`（按仓库根写）跑命令扫描，
rl_deploy 直接 `policy not found` 退出，机器人退化到"待机保持"并慢慢塌，
看起来像"这个模型站不住、不跟命令"，白跑 3 遍。

已做 §0.3 的两处缓解；**根治建议**：env 覆盖也统一到同一基准（仓库根或绝对路径），
或只接受绝对路径。

### 1.5 其他要确认的点

* `M20Interface::SetJointCommand()`（`m20_interface.hpp:193+`）注释里写了"超过硬件限幅会被
  固件饱和" ⇒ **软件侧不夹力矩**，全靠硬件兜。风险可接受，但值得在文档里点明。
* `M20Interface::ResetPositionOffset()` 的 ±360° 偏移判定只看 hipy/knee **"是否变化过"**，
  不看变化幅度；真机启动时若被碰一下，可能判错偏移。建议加"变化后落点是否合理"的复核。

### 1.6 【真 bug】`liedown_state.hpp` 的关节限位夹取是空操作

`liedown_state.hpp:48`（hipy）与 `:61`（knee）调用
`LimitNumber(theta, lo, hi);` **但没接返回值** ⇒ 夹取完全没生效。
`standup_state.hpp:48,61` 里一模一样的两个函数是 **`theta = LimitNumber(...)`**（接了）
—— 典型的"复制粘贴掉了一截"。

实测后果：趴下目标 `goal_joint_pos_ = (…, GetHipYPosByHeight(0.03), GetKneePosByHeight(0.03), 0)`，
其中 knee = π − acos((l1²+l2²−h²)/(2·l1·l2)) = **3.02 rad**，而膝关节上限是
**2.758 rad**（`control_parameters.h:44-45`）⇒ 趴下时膝关节目标**在限位外**，
靠机械限位顶住（不是致命，但姿态与设计意图不符，且对限位是硬顶）。

修法（两字符）：把 `liedown_state.hpp:48,61` 改成 `theta = LimitNumber(...)`。
建议顺手把这两个函数抽成一处（见 §4 的重复项），避免再有第三份抄歪。

---

## 2. 测试与验收缺漏

| # | 缺什么 | 证据 | 建议 |
|---|---|---|---|
| 2.1 | `command_sweep.py` 没进 `run_all.sh` | `run_all.sh:19` 的 `MODES` | 加一档（短冒烟 + 单独长跑） |
| 2.2 | 之前没有 vy/wz/姿态组合用例 | L3 原先只有 `rl`(零命令)/`walk`(+vx)/`arm_move` | 今晚已补 22 阶段的 `command_sweep.py` |
| 2.3 | **没有单元测试**（C++ 0 个、Python 0 个 pytest） | `tests/` 只有 3 个端到端脚本 | 先给纯函数加：`GetCubicSplinePos`、`RpyToRm`、`NormalizeAngle`、`fk_pose/mat_to_quat` |
| 2.4 | 没有 CI / pre-commit / lint 配置 | 无 `.github/`、无 `pyproject`/`.flake8` | 至少 L1+脚本自跑 |
| 2.5 | L1 只验默认策略目录 | `check_policy_interface.py:29` | 改成扫 `policy/*/` 全部验（今天平地目录就因没验而缺字段） |
| 2.6 | 真机侧只有 `preflight` + L2 桥接，无可执行验收 | — | 把 `sim2real_checklist_zh.md` 的判据尽量脚本化 |
| 2.7 | 默认站姿/EE 位姿一致性只断言了 MJCF 那一份 | `check_mjcf_contract.py` | 把 C++ 那 4 份也纳入断言（§3.1） |

---

## 3. 设计 / 可维护性

### 3.1 同一份常量抄了 6~8 遍（最值得动手）

**默认站姿**（hipy ∓0.6 / knee ±1.0 / 臂 0.5,-0.5 / 夹爪 0）在 8 处：
`m20_sim_interface.hpp:43-46`、`standup_state.hpp:71-74`、`mujoco_simulation_ros2.py:155-160`、
`m20_piper_policy_runner.hpp:399-404`、`piper_arm_interface.hpp:121`、
`arm_stub_node.py:55`、`arm_controller.py:82`、`check_policy_interface.py:34-40`(+`check_mjcf_contract.py:45-53`)。

**默认 EE 位姿**（`0.3492,0,0.4327,0.7373,0,0.6756,0`）在 6 处：
`common_types.h:104`、`rl_control_state.hpp:64`、`arm_stub_node.py`、`arm_real_adapter.py:76`、
`check_policy_interface.py:47`、`check_mjcf_contract.py:87`。

**臂/夹爪增益** 4 处（已对齐，但仍是 4 份拷贝）。

> 建议：一份 `deploy_contract.json`（+ 生成的 `.h`），把默认站姿 / 默认 EE 位姿 / 增益 /
> 关节限位收成单一来源。约 1 天成本，能一次性消灭"改一处漏三处"。

### 3.2 `memset` 非平凡对象

* `motion_state_feedback.hpp:36`：构造函数里 `memset(this, 0, sizeof(...))`，
  对象含 Eigen `Vec3f` 与匿名 union ⇒ UB 边界。
* `state_base.h:44`：`std::memset(&msfb_, 0, sizeof(msfb_))`，而 `msfb_` 是 **static**
  ⇒ 每造一个 State 就清一次全局状态；`main.cpp:12` 已静态构造过它。今天靠启动顺序侥幸无事。

> 建议：删 `state_base.h:44`；`MotionStateFeedback` 用成员初始化列表清零。

### 3.3 `UserCommand` 并发访问没有约定

| 执行流 | 写 | 读 |
|---|---|---|
| 键盘线程（`keyboard_interface.hpp:307-312`） | 速度/机身/`ee_inc`/`gripper`/`ee_reset`/`target_mode`/`safe_control_mode` | — |
| VR 回调（`rl_control_state.hpp:272-300`） | 速度/机身 + `SetRemoteActive` | — |
| 策略线程（`rl_control_state.hpp:203-215`） | `ee_goal_*` | 速度/机身/ee |
| 安全线程（`safe_controller.hpp`） | `safe_control_mode` | `time_stamp` |

只有 `ee_goal_` 有 mutex。`body_height` 等字段有键盘+VR 两个写者，靠
`IsRemoteActive()`（非原子）互斥。建议：把"字段所有权表"写进 `common_types.h` 注释（成本极低），
或改 `std::atomic`/双缓冲。

### 3.4 状态机残留

`qw_state_machine.hpp:96-123` 的 switch 覆盖 5 个状态，`default` 兜 `joint_damping`；
成员 `car_move_controller_`(`:33`) 从未创建 ⇒ 删。

### 3.5 Python 侧

* `arm_controller.py:249-269`：DLS 用**数值雅可比**（每步 6 次 FK + 差分），
  50 Hz × 最多 20 迭代 ⇒ 最坏 120 FK/tick。可解析化省一个数量级 CPU
  （今晚实测"仿真+策略+IK"会把实时因子从 1.0 拖到 ~0.5，见 DEF-034）。
* `print()` 与 `get_logger()` 混用（56 处 `print`）：建议 ROS 节点统一 logger。
* `arm_teleop_node.py:185-188` 用 `raise SystemExit(0)` 从定时器回调退出（异常当控制流），
  建议改 `self._stop` 标志由 `main` 收尾。
* `vr_teleop_device.py:182-198` 三处宽 `except Exception` 只打印，会吞真实原因。

---

## 4. 死代码 / 死分支（约 700 行，可分批删）

| 位置 | 规模 | 判定 |
|---|---|---|
| `run_policy/m20_policy_runner.hpp` + `rl_control_state.hpp:44` `m20_policy_` | ~265 行 | 只在 `rl_control_state.hpp` 被**声明**，从未构造 |
| `interface/user_command/keyboard_interface_sim.hpp` | 283 行 | 全仓库 0 引用；且与在用的键盘接口**类名相同**（误 include 会静默改行为） |
| `qw_state_machine.hpp:33` `car_move_controller_` | 1 行 | 只有声明 |
| `main.cpp:3-9` `#ifdef USE_SIMULATION` + `backward.hpp` | 7 行 | CMake 定义的是 `SIM2SIM`；`USE_SIMULATION` 全局仅此一处，且 `backward.hpp` **仓库里不存在**（一旦定义就编译不过） |
| `safe_controller.hpp:212,223` | ~25 行 | 见 §1.1（接线或删） |
| `basic_function.hpp:61 Sign()` / `custom_types.h:44 GetAbsPath()` | 12 行 | 0 引用 |
| `motion_state_feedback.hpp:50-63`（gait / error setters） | 14 行 | 5 个函数全 0 引用 |
| `control_parameters.h` 的 `body_len_x_/y_`、`hip_len_`、`wheel_link_len_`、`pre_height_`、`stand_height_`、`common_policy_*`、`lie_down_duration_` | ~12 成员 | 0 引用（`thigh_len_/shank_len_/wheel_vel_limit_/liedown_duration_` 在用） |
| `standup_state.hpp:39-63`、`:24-25`、`:42/55 default_pos`、`:72-74` 冗余赋值 | ~35 行 | 这两个函数在 standup **从未调用**（只有 liedown 用），且与 `liedown_state.hpp:39-63` 完全重复；`default_pos` 算了没用 |
| `robot_interface.h:46 start_flag_`、`:111 GetJointCommand()` | 4 行 | 0 引用 |
| `dds_interface.hpp:247 RefreshRobotData()` 空实现 | 2 行 | 空函数但被 `state_machine_base.h:123` 每 5 ms 调用，像是"刷新数据"其实什么都不做 |

---

## 5. CMake / 构建

| 位置 | 问题 | 建议 |
|---|---|---|
| `CMakeLists.txt:14`、`interface/CMakeLists.txt:6` | `add_definitions(-w)` 全局关警告 | 改 `-Wall -Wextra` 逐条消；要关就按 target 关 |
| `CMakeLists.txt:60-76` | `if(BUILD_PLATFORM STREQUAL "arm")` 与 `else()` 两分支**完全相同** | 合并 |
| `CMakeLists.txt:52` | `file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")` | 显式列源文件 |
| `CMakeLists.txt:44,58` | `onnxruntime//` 双斜杠；`CMAKE_INSTALL_RPATH` 混了相对路径 `../lib/onnxruntime/` | 清理 |
| `interface/CMakeLists.txt:17,20` | `interface` 静态库里混进 gamepad SDK 源码；`-lpthread` 等旧式链接 | 用 `Threads::Threads`；手柄单独 target |
| `interface/CMakeLists.txt:10-12,21-29` | 一批注释掉的 add_library/add_executable | 删 |
| 全局 | 没有 `install(DIRECTORY policy ...)`，策略路径依赖 `__FILE__`（源码路径） | 装到 `share/` 并按可执行文件位置解析，否则换机器/移动目录就跑不起来 |

---

## 6. 命名与一致性（小但多）

* 拼写：`temperture`（应 temperature；`safe_controller.hpp` 多处、`dds_interface.hpp:243`）、
  `turnning_vel_scale`（turning；`common_types.h`）、`rbs_/rsb_`（`rl_control_state.hpp:32`）。
* 两套并行枚举：`RobotMotionState`(`custom_types.h:14`) 与 `StateName`(`:22`) 数值完全相同，
  `target_mode` 用前者、状态机用后者，靠 `uint8_t(...)` 互转 ⇒ 合并或注明同值。
* `UserCommand` 里 `ee_inc[6]`（每 tick 增量）与 `body_*`（绝对值）混放，只靠注释区分 ⇒ 分组加前缀。
* `basic_function.hpp` 有两个同名 `LimitNumber`（2 参对称夹 / 3 参区间夹）⇒ 建议改名 `ClampSymmetric`/`ClampRange`。
* 键盘速度上限 `0.7/0.5/0.7`（`keyboard_interface.hpp:29-31`）远低于训练命令区间（`vx ±5`）
  ⇒ 注释里说明这两个数字的来源（安全考虑？历史值？）。

---

## 7. 今晚实测顺带发现的两个行为问题

### 7.1 `wz < 0`（顺时针）基本不跟

单命令阶跃（每条命令 8 s、取后 6 s 稳态，`/tmp/step_test.py`）：

| 命令 | 实测 | 跟踪度 |
|---|---|---|
| `vx +0.5` | +0.520 | 104% |
| `vx -0.5` | −0.352 | 70% |
| `vy +0.5` | +0.583 | 117% |
| `vy -0.5` | −0.437 | 87% |
| `wz +0.5` | +0.378 | 76% |
| **`wz -0.5`** | **−0.027** | **5%** |

隔离单发（只有这一个阶段、10 s）复现同样结果 ⇒ 不是顺序/残留问题，是**方向性不对称**。
可疑方向：左右侧轮的符号/增益、或训练侧 `yaw_rate` 符号约定。
建议在训练仓库做一次左右对称性探针（用同一套 MJCF 跑 `wz = ±0.3/±0.7`）。

### 7.2 侧移/转向的"跟随"要看窗口长度

4 s 阶段（稳态窗 2.4 s）里 `vy 0.2` 测出 ≈0，但同一条命令在 8 s 阶跃里测出 0.58
⇒ **侧移响应有 2~4 s 建立时间**。验收判据必须固定窗口，否则会把"慢"误判成"不跟"。

---

## 8. 建议的执行顺序（低风险 → 高收益）

1. **删死代码**（§4）：零行为变化；`keyboard_interface_sim.hpp` 优先（同名类，误用会静默改行为）。
2. **接安全层 + 统一时间单位**（§1.1/§1.2/§1.3）：先用"假驱动状态/假电量"在仿真里做用例。
3. **`-w` → `-Wall -Wextra`**（§5）：清一遍噪音会暴露不少隐患。
4. **单一常量来源**（§3.1）：消灭"改一处漏三处"。
5. **策略目录口径统一 + `install(DIRECTORY policy)`**（§1.4/§5）。
6. **补单元测试 + 把 `command_sweep` 挂进 `run_all.sh`**（§2）。
7. **（训练侧）查 `wz<0` 不对称**（§7.1）。
