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
| 2026-09-20 | 新增 DEF-023（无臂部署口径）+ `arm_stub_node.py` + `--mode no_arm` | `feat/no-arm-stub` |
| 2026-09-22 | 新增 DEF-024（真机启动自检无限等待 → 卡死在 `joint data update is not finished`）+ DEF-025（CMake 缓存 SIM2SIM） | `fix/real-startup-selfcheck` |
| 2026-09-22 | 新增 DEF-026（进 RL 时抢 `/ARM_JOINTS_CMD`，把 IK 控制的臂拽一下）；新增『只调臂』（不开策略）配置与用例 | `feat/arm-only` |
| 2026-09-29 | 新增 DEF-028（`arm_teleop_node` Ctrl+C 关不掉）；新增两份命令速查（sim2sim / sim2real） | `fix/arm-teleop-exit` |
| 2026-10-09 | 新增 DEF-029（臂 PD 阻尼 20→8、臂速度上限 3 rad/s）、DEF-030（夹爪 PD 4000/200→286/5）；DEF-031（默认策略切到多地形 run）；DEF-032（核对项：IK clamp / EE 复位锚定 部署侧已满足） | `sync/2026-10-09-sim-delta` |
| 2026-10-09 | 容器实测（`tests/run_all.sh`）后新增 DEF-033（`arm` 档判据与 DEF-018 不自洽，未改）；各档实测数字见 `docs/deploy_code_delta_2026-10-09_zh.md` §7 | 同上 |
| 2026-10-09 | 新增 DEF-034（宿主机电池降频 ⇒ 实时因子 0.67 ⇒ 进 RL 乱动/摔倒）；仿真新增 `[SIM-RT]` 实时因子自检与 `M20_SIM_RENDER_INTERVAL` 开关 | `sync/2026-10-09-sim-delta` |
| 2026-10-09 | 新增 DEF-035（默认策略切到平地 `cloud_cap12_20k`；`--mode push` 推力 800→1500 N） | 同上 |
| 2026-10-09 | 夜测后新增 DEF-036（vy/wz 方向性不对称，未修）、DEF-037（`liedown` 限位夹取空操作，未修）、DEF-038（`M20_POLICY_DIR` 相对路径口径，已缓解） | `sync/2026-10-09-sim-delta` |
| 2026-10-12 | 新增 DEF-039（`ee_goal` 四元数用了与训练**相反的符号分支** ⇒ 零速度命令下 0.045 m/s 持续前向漂移；改为训练/链式 FK 分支 + 逐帧连续，已修并端到端实测）、DEF-040（`no_arm`/`arm_mixed` 断言读的是 append 的 `policy_debug.txt` 的**第一块** ⇒ 读到历史 run，未修） | 工作区（建议 `fix/ee-goal-quat-sign`） |
| 2026-10-12 | 新增 DEF-044（VR 臂跟随：默认站姿手腕奇异 + 笛卡尔绝对偏移 ⇒ 局部 IK 解不出来还把"顶在限位上的解"下发，臂自己甩到 joint1=2.168，关节 20 rad/s / 力矩 139 N·m；改为**柱坐标位置 + 手腕关节偏移**的映射 + IK 收敛检查 + 零空间姿态正则，并新增回归用例 `tests/vr_arm_chain_check.py`） | `fix/vr-arm-ik-reachability` |
| 2026-10-13 | 开窗口（MuJoCo viewer）对着 12 个具名末端位姿指令做可视化验收（`scripts/arm_pose_demo.py` 新增）：跟踪 5 mm / 0.8°，关节 ≤0.06 rad/s、力矩 ≤4.6 N·m，机器人 RL 站立不接管；顺带修掉**半径下限把标定锚点自己钳住**（floor 0.12 → min(0.05, r_anchor)） | `fix/vr-arm-ik-reachability` |
| 2026-10-13 | 新增 DEF-045（残留 `arm_stub_node.py` 与 arm_controller **同时发** `/ARM_JOINTS_DATA` + `/ARM_TELEOP_STATE` ⇒ IK 反馈与策略 `ee_goal` 观测被混入常量默认位姿；表现"目标读数不变、臂不听使唤"） | 同上 |
| 2026-10-13 | 新增 DEF-046（VR 复位只把机身锚点置失效 ⇒ 复位前的偏移被固化成新零点、机身永远回不到默认；按操作员语义改为"机身/臂/速度回默认 + 交还控制权，再按 B 重新标定"，新增回归 `tests/vr_reset_semantics_check.py` 与 A/B 复现） | 工作区（建议 `fix/vr-reset-defaults`） |
| 2026-10-13 | 新增 DEF-057（VR 节点把地址打印成 `https://localhost:8443`：IP 探测依赖外网默认路由，真机内网没有 ⇒ 改为**离线枚举所有网卡地址**（无线优先）+ `M20_VR_IP` 指定；顺带修 `xlevr/config.py` 的 `config.yaml` 相对路径 ⇒ 警告消失且改 yaml 生效） | `sync/2026-10-13-vr-ip` |
| 2026-10-13 | **修编号冲突**：10-10 那批（VR 限速 / 手感旋钮 / 双发布者 / 复位与 IK 健壮性 / 离线依赖）与 10-12 真机条目撞了 039、040 两个号 ⇒ 前者重编号为 **DEF-052~DEF-056**（标题里标了"原 DEF-0xx"），并同步更新 DONE / TODO / DEPLOY_REAL_VR / `arm_pose_demo.py` 里的引用；10-12/13 的条目（039/040/044/045/046）**保持原号不动**；**DEF-047 留给 10-13 的 `tests/arm_ik_unreachable_check.py`（IK 够不到目标时的行为回归），该条目尚未在本文登记** | `sync/2026-10-13-defnumber-fix` |
| 2026-10-13 | 新增 **DEF-058**（头显页面依赖公网 CDN 拉 A-Frame ⇒ 内网里页面"卡住"、只剩中间一张卡片、点哪都没反应；**不是上位机性能问题**）：`index.html` 改本地 `aframe.min.js` + 缺失时页顶红色横幅；新增 `scripts/fetch_aframe.sh`（6 镜像 + 校验 + `--check`）；`sync_to_robot.sh` 与 VR 启动横幅都提示引擎是否就位；HTTPS 服务改 `ThreadingHTTPServer` | `sync/2026-10-13-vr-ip` |
| 2026-10-13 | **DEF-058 闭环**（操作员实测：放好 `aframe.min.js` 后头显页面正常、可遥操）；新增 **`docs/OFFLINE_ASSETS_zh.md`（离线资源清单：要下载什么）**——把 A-Frame、上位机 `PyYAML`/`websockets` wheel、**不用下的** scipy/numpy/pynput/torch、放哪/怎么自检/缺了的症状/换机器后重做哪些集中一处，并在 `DEPLOY_REAL_VR_zh.md` §1.5/§1.6、`COMMANDS_SIM2REAL_zh.md` §0.3、`docs/README_zh.md` 索引里加了指针 | `sync/2026-10-13-vr-ip` |
| 2026-10-13 | 新增 **DEF-059**（真机"左前侧摆关节过温 100 ℃ → 安全接管 → 趴下 → 10 s 后恢复"）：**仿真侧没有电机热模型**（`mujoco_simulation_ros2.py` 把 `motion_temp/driver_temp` 写死 40/45 ℃ ⇒ 温度那条接管路径在 sim2sim 不可达）；等价可观测的**关节力矩**实测为**长期静载**：站立状态机 `fl_hipx +12.0 N·m`、RL 站立零命令 `fl_hipx +15.1 N·m`（四个侧摆里最大、且是 `fr_hipx` 的 2.3 倍），是 M20 腿型"侧摆轴在机体侧 / 轮子接地点在 y≈0.23"的固有静负荷。新增离线探针 `tests/joint_torque_report.py` | `investigate/left-hipx-temp-sim2sim` |
| 2026-10-13 | **DEF-059 反问修正**（用户指出"这个量级的力矩不会导致过热吧、而且没到限幅、为什么右边没事"）：补做**执行器延迟压力测试**（0 / 5 / 25 ms：`fl_hipx` std 0.139→0.115→0.111 N·m、高频带能量不变 ⇒ 仿真里"延迟 ⇒ 抖动 ⇒ 发热"不成立），并在条目里写明"**静载只是必要条件，选不出具体关节**"（同腿 `knee 21.4 / hipy 16.3` 同量级、`76.4 N·m` 是训练 `effort_limit` 不是热额定、仓库无电机规格）+ **四刀**真机取数清单；新增 **P1-8** | `investigate/left-hipx-temp-sim2sim` |
| 2026-10-13 | **DEF-059 真机取数工具**：新增 `scripts/record_joint_data.py`（M20 上 200 Hz 录 `/JOINTS_DATA` 16 关节 `pos/vel/tau/motion_temp/driver_temp` + `/IMU_DATA`，每 5 s 打印最热 3 关节，退出给汇总并标出 `>100 ℃` 区段）与 `scripts/joint_log_report.py`（离线四刀：与仿真参考值对照 / 两个温度分开看 + 过温时序 / 谁最热 vs 谁力矩最大 + 左右镜像 / `--spectrum`）；命令写进 `COMMANDS_SIM2REAL_zh.md` **§7**（含 `scp` 取回与 `ros2 bag` 兜底）。两者已在仿真 `/JOINTS_DATA` 上自测通过 | `investigate/left-hipx-temp-sim2sim` |

---

### DEF-059 `2026-10-13` 站立/RL 站立时**前腿侧摆关节长期顶 12~15 N·m**；仿真无热模型 ⇒ 真机"左前侧摆过温 → 100 ℃ 安全接管 → 趴下 → 10 s 后恢复"在 sim2sim **无法出现**（只能看力矩）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-13` |
| 类型 | `缺陷`（真机现象；仿真侧缺热模型 = 可观测性缺口） |
| 状态 | `未修（见 TODO_zh.md P1-7）` |
| 影响面 | `二者`（`fl_hipx` 的静载在 sim2sim 与真机同源；真机才有温度） |
| 关联 | `investigate/left-hipx-temp-sim2sim`；`mujoco_simulation_ros2.py:836-837/874-875`（写死温度）、`include/utils/safe_controller.hpp:190/202/381`（`>100 ℃` ⇒ `safe_control_mode=2`）、`state_machine/quadruped_wheel/{standup,rl_control,joint_damping}_state.hpp`、`state_machine/parameters/m20_control_parameters.cpp`（腿 `swing_leg_kp_ = 200`、`hip_len_ = 0.104`）、新增 `tests/joint_torque_report.py`（仿真遥测）、`scripts/record_joint_data.py`（**真机录** `/JOINTS_DATA`）、`scripts/joint_log_report.py`（离线四刀）、`COMMANDS_SIM2REAL_zh.md` §7（命令）；相关 **DEF-036**（vy/wz 方向不对称，同属"策略/结构左右不对称"家族） |

**1. 现象（操作员 2026-10-13 报告的真机现象 + 本轮仿真复现）**

* **真机**：进入**站立状态机**或 **rl_control** 后过一段时间，**左前侧摆关节（`fl_hipx`）**
  温度升到 **100 ℃** ⇒ 触发安全保护 ⇒ 机器人**趴下**；约 **10 s** 后温度掉回阈值以下 ⇒
  **又能恢复**。
* **复现命令（仿真侧，看等价量"关节力矩"）**：
  ```
  M20_SIM_TELEMETRY=/root/m20_piper_ws/tmp/temp_probe/stand.telemetry.csv \
    python3 tests/sim2sim_smoke.py --mode stand --duration 60
  M20_SIM_TELEMETRY=.../rl.telemetry.csv  python3 tests/sim2sim_smoke.py --mode rl  --duration 60
  M20_SIM_TELEMETRY=.../hold.telemetry.csv python3 tests/sim2sim_smoke.py --mode hold --duration 40
  python3 tests/joint_torque_report.py tmp/temp_probe/rl.telemetry.csv --t0 25
  ```
* **实测（200 Hz 遥测；MJCF 序；单位 N·m。`stand`/`hold` 取 t≥15 s 窗，
  `rl` 取 t≥25 s 窗 —— 进 RL 后的收敛要 ~30 s，窗口取早了会把入口瞬态算进来，
  例如 `rl` 用 t≥15 s 窗会得到 fl 15.24 / hr −12.44）**：

  | 状态（用例） | 腿 kp/kd | `fl_hipx` | `fr_hipx` | `hl_hipx` | `hr_hipx` | 轮接触力 fl/fr/hl/hr | 站高 |
  |---|---|---|---|---|---|---|---|
  | 站立状态机（`--mode stand`，`z` 后停在 standup 保持） | 200 / 4 | **+12.0**（std 0.002） | −11.9 | +8.0 | −7.9 | 106.5 / 105.9 / 86.2 / 85.6 N | 0.514 m |
  | 仿真自带保持（`--mode hold`，无状态机，目标=训练默认姿态） | 80 / 2 | **+18.6**（std 3.0） | −18.5 | +8.7 | −8.6 | 114.1 / 113.6 / 78.5 / 77.9 N | 0.479 m |
  | RL 站立（`--mode rl`，零速度命令） | 80 / 2 | **+15.1**（std 0.09） | −6.6 | +4.6 | −13.6 | 127.0 / 80.5 / 70.2 / 106.5 N | 0.510 m |

* 时间演化（`--ts`，三次独立 run 结论一致）：进 RL 后 `fl_hipx` 先冲到 **+20.2 N·m**
  （t≈8 s，入口瞬态），随后单调收敛，**t≈30 s 起钉在 +15.0 N·m**；150 s 的长用例里
  **t≥70 s 之后完全不变**（fl +15.03 / fr −6.44 / hl +4.60 / hr −13.68，站高 0.5097 m），
  即这是一条**稳态**而非漂移。
* 全腿热负荷（`mean(τ²)`，与电机铜损同量纲）排序：
  `fl_knee 460.7 > fl_hipy 266.5 > hr_knee 240.7 >` **`fl_hipx 227.2`** `> hr_hipx 184.0`
  `≫ fr_hipx 43.0 > hl_hipx 21.0` ⇒ **左前侧摆是四个侧摆里最大的那个，且是右前的 5.3 倍**。
* 频谱：四个侧摆的高频段（>5 Hz）都 **< 0.001 N·m** ⇒ 仿真里**没有抖动/颤振**，
  热负荷全部来自"**直流静载**"，不是高频来回。
* **不是"抖动"，也不是"顶到限幅"**：`|τ| > 20 N·m` 的采样 0/24774，离
  `ctrlrange` 的 ±76.4 N·m 很远 ⇒ 如果真机过热，**不能**用"仿真里力矩饱和/抖振"解释。
* **执行器延迟压力测试**（2026-10-13 补做，用来验证"真机控制环抖动 ⇒ 额外发热"这条假设）：
  把 `M20_SIM_ACTUATOR_DELAY_TICKS` 从默认 0 开到 **5**（≈训练随机化上限）与 **25**
  （1 ms 控制 tick），三次 `--mode rl` 用例的 `fl_hipx` 几乎不动：
  std **0.139 → 0.115 → 0.111 N·m**，5~600 Hz 带内能量依旧 **< 0.001 N·m**，站高/倾角不变
  ⇒ **在本模型里，"延迟造成极限环抖动 ⇒ 额外发热"这条路不成立**，也就是说
  **仿真没有能力复现或排除真机上的这类现象**（正结果：仿真侧"静载"结论干净；
  负结果：真机若真在抖，来源只能是仿真没有的东西——编码器量化、齿隙/背隙、电流环、
  静摩擦/死区、驱动器开关损耗）。

**2. 根因**

* **仿真侧没有温度模型（可观测性缺口）**：`mujoco_simulation_ros2.py:836-837`（腿）与
  `:874-875`（臂）把 `motion_temp = 40.0`、`driver_temp = 45.0` 写死 ⇒
  `SafeController::IsMotorTempertureNormal()`（`safe_controller.hpp:190`）的
  `max_temp > 100` **永远不成立** ⇒ 真机那条"过温 → 接管"路径在 sim2sim 里**不可达**。
  （真机链路已核对：`:202` 置 `safe_control_mode=2` ⇒ `standup_state.hpp` /
  `rl_control_state.hpp` 的 `LoseControlJudge()` ⇒ `JointDampingState`（只发 kd=2/1、
  无位置目标 = 卸力）⇒ **趴下**；温度掉回后 `Safe control recovered: safe_control_mode -> 0`
  （`:396-399`）⇒ **又能恢复** —— 与操作员描述的现象链完全一致。）**故本现象在
  sim2sim 里既不会出现、也无法验证。**
* **力矩为什么长期这么大（机理，可独立复算）**：`fl_hipx` 关节轴在**机体侧**
  （MJCF `fl_hipx` body `y = +0.0685`），而轮子接地点在 `y = +0.0685 + 0.0984 + 0.0597
  ≈ 0.2266` ⇒ **力臂 ≈ 0.158 m**。平地站立时前腿承担 ~106 N（前/后 ≈ 55/45）
  ⇒ 侧摆关节必须顶住 `106 × 0.158 ≈ 16.7 N·m`，扣掉腿自重那部分（≈2 N·m）后与实测的
  12~15 N·m 同量级。**这是 M20 腿型 + 站立姿态的固有静负荷**：与 kp 无关，
  kp 只决定"让步多少角度"——站立状态机 kp=200 时停在 `q = −0.060 rad`（200×0.060 = 12.0），
  RL kp=80 时停在 `q_des − q = 0.188 rad`（80×0.188 = 15.1）。
* **关节角速度 ≈ 0 ⇒ 没有机械功率，只有铜损**：`P ≈ I²R ≈ (τ/k_t)²·R ∝ τ²`，
  电机电流长期为"顶力矩/转矩常数"⇒ 温度单调升到某个平衡值；只要平衡值 > 100 ℃，
  真机就会"顶一会儿 → 过温 → 趴下 → 冷却 → 再站起来"循环。
* ⚠️ **但这**不构成**"为什么偏偏是左前侧摆"的解释 —— 本轮**没有**闭合这一问**：
  * 同一时刻前腿上有三个关节的力矩**在同一量级**（RL 站立稳态：`fl_knee −21.4` /
    `fl_hipy −16.3` / `fl_hipx +15.2`，后腿 `hr_knee 15.7` / `hr_hipx −13.6`）；
    按 `mean(τ²)` 排，`fl_hipx` 只是**第 4 名**（在 `fl_knee`、`fl_hipy`、`hr_knee` 之后）。
  * `76.4 N·m` 是**训练配置的 `effort_limit`（峰值量级的上限）**，**不是电机的连续/热额定**；
    `τ = 15 N·m ≈ 20% 峰值` 落在"可能正好压着连续额定"的区间，但**仓库里没有 M20 电机的
    额定力矩/电流、`k_t`、热阻**，所以**无法从这些数据算"该多少摄氏度"**。
  * 直立站桩对这类按间歇工况选型的关节电机是**最坏工况**（无休息、无反电动势、
    气流最差）⇒ 长时间"顶力矩"本身可以把温度顶到平衡值；但"哪个关节先跨过 100 ℃"
    由**电机个体差异 / 散热路径 / 安装位置（左前上方是机械臂与计算单元）/ 该路接插件**决定，
    这些都在**仿真之外**。
  * **站立状态机阶段左右是基本对称的**（`fl_hipx +12.0` / `fr_hipx −11.9`）⇒
    若操作员看到的是"刚站起来还没进 RL"就左前先热，仿真给不出这个"左"，
    更指向真机该腿自身（标定 `dir/offset`、装配/背隙、线束、驱动器、电机个体）。
  * 还有一个**数据口径**上的坑：仿真遥测里的 `torque` 是**控制器算出的 PD 指令**
    （`input_tq`），而真机 `/JOINTS_DATA` 的 `torque` 是**电机实测/估计**值；
    "两边数字一样"**不等于**"两边电流一样"（多了摩擦、纹波、相电阻差异就会分叉）。
  * 要闭合这一问，只能拿真机数据一刀一刀切（四刀，工具已就位：
    `scripts/record_joint_data.py` 在真机上录 → `scp` 取回 →
    `scripts/joint_log_report.py` 出四刀报告，命令见
    `COMMANDS_SIM2REAL_zh.md` **§7**；待办 `TODO_zh.md` P1-7 / P1-8）：
    ① 16 个关节静止时的 `torque` 与仿真表逐个对（差多少 ⇒ 是负载/标定问题，还是相等 ⇒ 不是）；
    ② `motion_temp`（电机）与 `driver_temp`（驱动器）**分开看**（代码判的是 `motion_temp`；
    若是 `driver_temp` 先到 100，那是功率级/压降/开关损耗，不是绕组铜损）；
    ③ 站立 10 分钟记**全部 16 个**关节的温度曲线（邻居也在 80~95 ℃ 缓升 ⇒ "整条前腿都贴着上限，
    谁的边际最差谁先跳"；只有它自己冲上去 ⇒ 该关节自身问题）；
    ④ 用 `tests/joint_torque_report.py` 的口径分析一份真机 200~1000 Hz 的 `torque` 记录
    （看 std / 高频带能量 ⇒ 有没有仿真没有的纹波/极限环）。
* **"左前"这个方位在 RL 站立里是稳定的**：策略在零速度命令下自己收敛到**不对称站姿**
  （反推命令：`fl_hipx` 目标 ≈ +0.19 rad、`fr_hipx` ≈ −0.05 rad；动作值 +1.5 / −0.4），
  轮接触力也长期偏成 127/80/70/107 N（对角吃重）。两次 45~60 s 与一次 150 s 用例都复现
  同一模式（注意：策略是确定性的、仿真初始状态也几乎对称，所以这**不能**当成独立采样，
  "重的那一侧是不是永远是左前"仍需在真机/带扰动的情况下确认）。
* 在**站立状态机**里左右是基本对称的（12.0 / −11.9）⇒ 若操作员是在"刚站起来还没进 RL"
  的阶段看到左前先热，那更可能是**真机自身的左右差异**（标定/装配/线的走向/摩擦）叠加，
  而不是仿真里的这条不对称。

**3. 修正**

* 本轮**没有改任何控制/姿态逻辑**（真机现象、且改站姿会直接把策略推到分布外）：
  新增离线探针 `tests/joint_torque_report.py`（τ 统计 / 热负荷排序 / 频谱 / 饱和检查 /
  1 Hz 时间序列），登记本条目。
* 为"拿真机数据切四刀"补了两个脚本（都已在仿真 `/JOINTS_DATA` 上自测通过，
  含人造过温数据的过温事件检测）：
  `scripts/record_joint_data.py`（真机 200 Hz 录 16 关节 `pos/vel/tau/motion_temp/driver_temp`
  + `/IMU_DATA`，终端每 5 s 打印"最热的 3 个关节"，退出时给汇总并标出 `>100 ℃` 的区段）
  与 `scripts/joint_log_report.py`（离线：仿真参考值对照 / 两个温度分开看 + 过温时序 /
  谁最热 vs 谁力矩最大 + 左右镜像对比 / `--spectrum` 高频带能量）；
  使用命令见 `COMMANDS_SIM2REAL_zh.md` §7。
* 备选方案（都**没做**，需用户/训练侧决定）：
  a) 给仿真加**一阶热模型**（`dT/dt = (T_amb − T)/τ_th + k·τ²`，按 `tau`/`dq` 算铜损），
     让"过温 ⇒ 自动接管 ⇒ 倒 ⇒ 冷却恢复"这条安全路径在 sim2sim 里可回归——
     参数只能取假设值，但足以验证**逻辑**；
  b) 真机核对：`ros2 topic echo /JOINTS_DATA` 看 16 个关节的 `torque` 与 `motion_temp`
     长期曲线，确认 `fl_hipx` 是"纯静载"还是"静载 + 抖动/额外摩擦"（若是后者，仿真
     测不到，需要查电流环/减速箱/装配）；
  c) 若确认是静载：从**姿态**入手降载（站高、髋外展角、前后配重/加配重），
     但这是训练侧改动（会改策略的分布），必须两边同步。

**4. 结果（验收）**

* 实测数据：见 §1 的对照表与时间序列（`stand` 47 s、`hold` 42 s、`rl` 64 s、`rl_b` 49 s、
  `rl_long` 154 s 共 5 次用例，`rl` 档全部 PASS）。
* 回归：`tests/sim2sim_smoke.py --mode stand|hold|rl` 本身判据（高度/倾角）全过
  （`rl` 档 0.502~0.514 m、倾角 ≤0.9°），说明**这个静载没有把机器人顶坏**，
  它是"正常但很费电/很热"的工况。
* 遗留问题：① 仿真无法验证过温保护（无热模型）；② 真机 `fl_hipx` 是"静载"还是
  "静载+抖动"未确认；③ "左前"这一侧在真机上的可重复性未确认（策略确定性 ⇒ 仿真的
  复现不能当独立样本）；④ 原始遥测 CSV 体积大（单次 10~33 MB），只留在
  `tmp/temp_probe/`（`tmp/` 已 gitignore），未入库。

---

### DEF-046 `2026-10-13` VR 复位只把机身锚点置为"失效" ⇒ 复位前的偏移被固化成新零点，机身永远回不到默认

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-13` |
| 类型 | `缺陷`（复位语义 / 外部源与本地源的控制权交接） |
| 状态 | `已修`（`rl_control_state.hpp` + `user_command_interface.h` + `keyboard_interface.hpp` + `vr_teleop_device.py`；新增回归 `tests/vr_reset_semantics_check.py`，含旧实现的 A/B 复现） |
| 影响面 | `二者`（sim2sim 与真机走同一条 VR 链路） |
| 关联 | DEF-042（复位锚定）、DEF-044（臂侧的同类问题：臂专门定义了 `ARM_TELEOP_FLAG_RESET_DEFAULT=2`"锚到默认末端位姿"）、`COMMANDS_SIM2SIM_zh.md` §5、`docs/DEPLOY_REAL_VR_zh.md` §5.1、`tests/vr_reset_semantics_check.py` |

**1. 现象（操作员）**：头显里按**左 X/Y（reset）**之后，**机身姿态没有回默认** ——
复位那一刻的高度/俯仰/横滚偏置原样留着（观感是"复位反而把它锁住了"）；而且复位后
VR 仍保持激活（2026-10-10 起的旧默认），只能反向推手柄把它推回去。

**2. 根因（两层，均可离线复现）**

* **锚点被"归档"**：`rl_control_state.hpp` 的 `/VR_TELEOP` 回调在 reset 边沿**只**把
  `vr_anchor_valid_ = false`；紧接着 `ApplyVrCommand()` 的重新锚定分支取的是**当前
  指令值**（`vr_body_anchor_[i] = uc->body_height/…`，本意是"按 B 时不要跳变"）⇒
  `target = 旧 anchor + 旧 off + 0 = 复位前的指令值`，**偏移被固化成新零点**。
  设备侧其实已经把 13 维指令与机身高度积分量清零了（`vr_teleop_device.py::_on_reset`），
  是策略侧把它锁住。
* **没有"回默认"的路径**：设备侧 `RESET_KEEPS_ACTIVE` 当时默认 `1`（保持激活）⇒ 复位后
  VR 仍持有控制权，键盘被 `IsRemoteActive()` 挡住，机身字段没有任何源去写默认值。

**3. 修正**（按操作员 2026-10-13 定义的语义：复位 = 机身姿态回默认 + 臂末端位姿回默认
 + 速度归 0 + **交还控制权**；再按 B 把当前左右手柄位姿重新采成校准原点）

* `rl_control_state.hpp`：reset 边沿**直接把机身锚点写成默认 `(0.513, 0, 0)`**
  （不再交给"锚到当前指令值"分支）、清零 `vr_vel_`/`vr_body_off_`、把共享 `UserCommand`
  的速度/机身写回默认，并打印 `[VR] RESET -> 机身姿态回默认 (0.513 / 0 / 0)…`；
* `user_command_interface.h` + `keyboard_interface.hpp`：新增
  `RequestLocalReset()/ConsumeLocalResetRequest()`，键盘线程消费后把自己的
  `body_height_/body_pitch_/body_roll_` 归位到默认并清空"按住"的键 —— 否则它会在 5 ms 后
  用复位前的旧值把默认指令覆盖掉；
* `vr_teleop_device.py`：`RESET_KEEPS_ACTIVE` 默认 `1` → **`0`**（复位即 `_started=False`，
  `/VR_TELEOP` 持续发全 0 + `active=0`，等价"持续给 0 指令"）；
  `M20_VR_RESET_KEEPS_ACTIVE=1` 可回旧语义；
* 臂侧沿用既有 `ARM_TELEOP_FLAG_RESET_DEFAULT=2`（`arm_teleop` → `arm_controller`
  重置到默认末端位姿）+ `/reset_sim`，本次无需改动。

**4. 结果与验证**（`tests/vr_reset_semantics_check.py`：设备级 + 端到端，
含 `default`/`keepactive` 两个场景，并对 DEF-018 入口瞬态自动重跑）

| 时点 | 机身俯仰 | 机身高度 | \|轮速指令\| | 臂末端目标 x | 机身锚点 |
|---|---|---|---|---|---|
| 基线（进 RL、VR 未接管） | +0.5° | 0.506 m | 0.24 rad/s（零速残余） | 0.3492 m | — |
| 复位前（VR 命令 +0.12 rad / +30 mm / vx 0.3 / 臂 +50 mm） | **+6.8°** | 0.516 m | 3.71 rad/s | 0.3990 m | 0.513/0/0 |
| 复位后 | **+0.7°** | 0.514 m | **0.165** rad/s | **0.3492 m**（= 默认位姿） | 0.513/0/0 |
| 再按 B 之后 | +0.7° | 0.512 m | — | 0.3790 m（重新跟手） | 0.513/0/0 |

* 日志：`rl_deploy` 打印 `[VR] RESET -> 机身姿态回默认 (0.513 / 0 / 0)` 与
  `[RESET] 外部源（VR）交还控制权：机身姿态 -> 默认 (0.513, 0, 0)、速度 -> 0`；
  `arm_teleop` 打印 `[reset] /reset_sim 完成；EE 目标已重置到默认位姿`。
* **A/B 复现（旧实现原样：reset 边沿只 `vr_anchor_valid_ = false`；`keepactive` 场景）**：
  复位后俯仰仍偏 **+6.3°**、锚点被固化成 **`0.543 / 0.12 / 0`** ⇒ 用例 6 条判据 FAIL。
  这正是操作员报的现象。
* 用例里固化的两条口径（都是策略侧特性，不是本次改动的锅）：① **机身高度命令跟踪很慢**
  （命令 +30 mm，8 s 只走到 +10 mm 且还在爬）⇒ 高度只判"方向 + 量级下限"，严格判据放在
  俯仰通道与日志上；② **零速有残余轮速指令**（实测 0.2~1.4 rad/s）⇒ "速度归 0"只能跟
  基线比，不能卡绝对值。

---

### DEF-045 `2026-10-13` 残留的 `arm_stub_node` 与 arm_controller **同时**发 `/ARM_JOINTS_DATA` / `/ARM_TELEOP_STATE` ⇒ IK 反馈与策略观测被混入常量

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-13` |
| 类型 | `缺陷`（运行环境 / 话题发布者不唯一） |
| 状态 | `已定位`（清掉残留进程即恢复；**尚未**做成上机前的自动检查） |
| 影响面 | `sim2sim`（上真机同样适用：多起一个桥接/假臂节点就会踩到） |
| 关联 | `arm_stub_node.py`（发 `/ARM_JOINTS_DATA` + `/ARM_TELEOP_STATE`）、`arm_real_adapter.py`、`arm_controller.py`、`scripts/check_vr_arm_chain.py`（能列出发布者数量）；同类问题见 DEF-041（2026-10-10，`/VR_TELEOP` 上两个发布者） |

**1. 现象**：开窗口做 DEF-044 的位姿演示时，臂**在动**、但演示脚本读到的
`/ARM_TELEOP_STATE`（目标）**恒定是默认位姿** `(0.3492, 0, 0.4327, −0.7373, 0,
−0.6756, 0)`，与"目标应随指令变化"矛盾；`arm_controller` 侧的 `[arm_ik]` 打印里
目标却是在正常变化的（残差 0.1~1.6 mm）。同一时间 `rejects` 计数在启动阶段涨到 166
就不动了。

**2. 根因**：`ps` 里发现一个 **11 小时前**（`etimes` ≈ 3.6e4 s）遗留的
`arm_stub_node.py`（早先 `--mode arm_mixed` / `no_arm` 用例起的），它按设计在发
*常量* 的臂反馈与常量 ee_goal：

```
4269  36388  /usr/bin/python3 .../arm_stub_node.py
```

于是这些话题上有**两个发布者**，两路数据交替到达：

| 话题 | 发布者 | 后果 |
|---|---|---|
| `/ARM_JOINTS_DATA` | 仿真（真实关节）+ **残留 stub（常量默认位姿）** | `arm_controller` 的 IK **种子/反馈** `cur_joints` 被掺入常量 ⇒ 单拍内目标-反馈关系自相矛盾，安全网拒发（166 拍） |
| `/ARM_TELEOP_STATE` | `arm_controller`（真实目标）+ **残留 stub（常量）** | 策略的 `ee_goal` 观测被掺入常量 ⇒ 观测与真机不一致；人肉读话题时看到"目标不变" |
| `/ARM_JOINTS_CMD` | 只有 `arm_controller` ✓ | 命令本身是干净的（所以臂还在动，掩盖了问题） |

**3. 修正**：`kill` 掉残留进程后立刻恢复：`/ARM_TELEOP_STATE` 随指令变化、
`rejects` 不再增长、演示的"目标 vs 实际"逐拍对得上（5 mm / 0.8°）。

**4. 结果与待办**：

* 本次是**环境问题**，代码无需改；但它会伪装成"臂不听使唤 / 观测不对"，且
  **上机时同样会发生**（多起一个假臂节点或 `arm_real_adapter` 就踩到）。
* 现在可用的自检：`python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py`
  会打印每个话题的**发布者数量**并按"某个话题有 2 个以上发布者"直接告警。
* **待办（建议做成流程）**：跑任何 sim2sim/真机用例**之前**先做一次残留清理
  （`tests/sim2sim_smoke.py` 里已有可复用的 `leftover_processes()` /
  `force_cleanup()`），或把"话题只有一个发布者"做成 `run_all.sh` 的前置断言。
  见 `TODO_zh.md` 的 P2-8。

---

### DEF-044 `2026-10-12` VR 臂跟随在默认站姿上解不出目标，却把"顶在限位上的解"当命令发出去 ⇒ 臂自己甩（20 rad/s / 139 N·m）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-12` |
| 类型 | `缺陷`（VR→EE 映射 + IK 下发判据） |
| 状态 | `已修`（sim2sim 端到端实测 + 新增回归用例） |
| 影响面 | `二者` |
| 关联 | 分支 `fix/vr-arm-ik-reachability`；`arm_controller.py`（`vr_absolute_target`、`ik_step` 姿态正则、收敛检查）、`tests/vr_arm_chain_check.py`（新增）、`tests/run_all.sh`、`src/M20_sdk_deploy/COMMANDS_SIM2SIM_zh.md` §5、`docs/DEPLOY_REAL_VR_zh.md`；算法层面与训练侧 VR 设备（`vr_extented.py` 的笛卡尔偏移语义）不同，见下 |

**1. 现象**：键盘控制机械臂正常，**VR 控制却很怪**——臂会自己转过去、甩到一边、
不跟手；操作员报的是"腿臂同时操作时臂动作不对"。实测（sim2sim，腿在走 + VR 手画圆，
实时因子 1.000）：臂端被顶到 **joint1 = 2.168 rad（正好是它的上限，124°）**、
joint4 = 1.745、joint5 = ±1.22，关节速度峰值 **20.8 rad/s**、指令力矩峰值
**139.5 N·m**（真机驱动器只有 3 rad/s / 100 N·m）。

**2. 根因（三层，均可离线复现）**

* **默认站姿正好是手腕奇异位形**：`(0, 0.5, -0.5, 0, 0, 0)` 里 joint5 = joint6 = 0
  ⇒ joint4 与 joint6 同轴 ⇒ Jacobian 最小奇异值 **3.6e-18**（条件数 1.8e12）；
  joint5 到 0.3 rad 才升到 3.8e-3。最弱的方向恰好是"侧向平移 + 绕 z 的姿态"。
* **VR 手部动作必然激励这些方向，而它们的解在关节空间里很远**。用约束优化求"离当前
  姿态最近"的严格解（`tmp/dbg/ik_minq.py`）：键盘的 +x 平移只要 **0.30 rad** 关节重排，
  而侧移 +y 5 cm 要 **2.67 rad**（+y 2 cm 也要 2.40 rad）、工具 yaw 10° 要 **2.47 rad**。
  局部 DLS 单拍最多走 `20 × 0.05 = 1.0 rad`，而且在奇异方向上不推进——离线 trace 显示
  误差 20 次迭代**稳稳停在整 5 cm 不动**，步长全花在零空间（joint4/6 等量反号相消）。
* **解不出来照样下发 + 硬夹限位**：`ik_step()` 迭代 20 次就返回最后一组（已被
  `np.clip` 钳到关节限位上的）解，`_tick_impl()` 无条件发布 ⇒ 臂被驱到"顶限位"的
  姿态；仿真里没有关节速度限制（真机 3 rad/s）、力矩夹在 ±100 N·m，于是看到 20 rad/s、
  139 N·m 的甩动。

**3. 为什么键盘没事**：操作员只按 ±x/±z（well-conditioned，7~8 次迭代收敛，
朝向误差 0.5°，joint4/5/6 全程不动）。**键盘一旦按旋转键（numpad `+`）同样会坏**
（实测 yaw 键：姿态残差 176°、六个关节全部顶限位）⇒ 不是"VR 传输坏了"，
而是"侧移 + 旋转这类请求在这个位形上根本不可达"。
**腿是无关项**：腿站着不动时失败完全一样（残差 114.5 mm / 65.4°，顶限位同前）。

**4. 修正**（`arm_controller.py`）

* **映射（新默认 `M20_VR_ARM_MAP=cylindrical`）**：
  * 位置改用**柱坐标**：前后 → 半径（沿臂当前朝向伸缩）、左右 → **基座摆动 Δθ**
    （臂自然摆过去）、上下 → z；
  * 姿态改用**手腕关节偏移**：(droll, dpitch, dyaw) 当作 (joint6, joint5, joint4)
    的增量，目标姿态 = `Rz(Δθ) · FK(anchor + 手腕偏移)`——三个旋转分量永远落在手腕
    自己的自由度上（永远可达、代价极小），工具姿态跟着臂的摆动走，不需要手腕去补偿
    整条臂。`M20_VR_ARM_MAP=cartesian` 可退回旧行为做 A/B（不推荐）。
* **IK 收敛检查（安全网）**：每拍算残差，超过 `M20_ARM_IK_MAX_POS_ERR`（默认 10 mm）
  或 `_ANG_ERR`（默认 3°）就**不下发**——沿用上一拍指令，并把被跟踪目标拉回实际位姿
  （否则请求会一直躺在不可达处、下一拍继续失败），限频告警。
* **零空间姿态正则**：`ik_step(..., q_nom=anchor, posture_gain)` 压住"不影响任务、
  但会让手腕沿自由方向漂到限位"的分量（没它时 ~10 s 就把 joint4 拧到 +1.745、
  joint6 到 -2.094，都是限位）。
* **关节指令限速默认 3.0 rad/s**（= 真机驱动器口径，原来默认 0 = 不限）。

**5. 结果（同一套 rig、同一段假头显动作，对照实测）**

| 指标 | 修改前 | 修改后 |
|---|---|---|
| IK 位置残差 > 10 mm 的采样占比 | **31.8%** | **0.0%** |
| IK 位置残差 p95 / max | 104 / 114 mm | **2.8 / 9.8 mm** |
| IK 姿态残差 p95 | 58.8° | **0.0°** |
| 跟手误差（请求目标 vs 实际末端）p95 | 99.9 mm | **26 mm** |
| 关节速度 max | **20.8 rad/s** | **1.9 rad/s** |
| 指令力矩 max | **139.5 N·m** | **8.1 N·m** |
| 停在关节限位的关节 | joint1/4/5 | **无** |
| 安全网拒发拍数 | —（没有安全网） | 1 |

**只加安全网、不改映射**（A/B：`M20_VR_ARM_MAP=cartesian` + 关正则/限速）：残差看着是
0，但**拒发了 965 拍**——也就是臂有一半时间"不动"（因为它满足不了请求），说明
**映射才是真正的修复**，安全网是兜底。

**6. 可视化验收（2026-10-13，MuJoCo 窗口 + 12 个具名末端位姿指令）**

按"操作员怎么看效果"的方式复核了一遍：仿真（`M20_USE_VIEWER=1`，开窗口）
→ `arm_controller` → `rl_deploy`（**必须** `z` 站立 → `c` 进 RL，否则只有仿真自己
的腿部默认保持，机器人会**缓慢摔倒**）→ `scripts/arm_pose_demo.py`（本次新增的工具，
依次下发下面 12 个位姿并打印实测）。

环境：实时因子 **1.000**（全程），base 高度 0.508~0.510 m、倾角 <1°、**0 次安全接管**。

| # | 指令（相对标定锚点） | 目标末端 (root 系) | 实际末端 | 位置误差 | 姿态误差 |
|---|---|---|---|---|---|
| 1 | 中位（标定锚点） | (0.360, 0.000, 0.428) | (0.360, 0.000, 0.423) | 5.1 mm | 0.8° |
| 2 | 前伸 8 cm | (0.429, 0.000, 0.428) | (0.429, 0.000, 0.422) | 5.9 mm | 1.2° |
| 3 | 后收 8 cm ⚠️见下 | (0.360, 0.000, 0.428) | (0.360, 0.000, 0.423) | 5.1 mm | 0.8° |
| 4 | 左移 8 cm → 基座摆动 | (0.340, +0.067, 0.428) | (0.340, +0.066, 0.423) | 5.1 mm | 0.8° |
| 5 | 右移 8 cm | (0.340, −0.067, 0.428) | (0.340, −0.066, 0.423) | 5.1 mm | 0.8° |
| 6 | 抬高 6 cm | (0.360, 0.000, 0.488) | (0.360, 0.000, 0.483) | 4.9 mm | 0.8° |
| 7 | 降低 6 cm | (0.360, 0.000, 0.368) | (0.359, 0.000, 0.363) | 5.1 mm | 0.8° |
| 8 | 手腕自转 +25° | (0.360, 0.000, 0.428) | (0.360, 0.000, 0.423) | 5.1 mm | 0.8° |
| 9 | 手腕俯仰 +25° | (0.360, 0.000, 0.428) | (0.360, 0.000, 0.423) | 4.9 mm | 0.8° |
| 10 | 手腕侧倾 +25° | (0.360, 0.000, 0.428) | (0.360, 0.000, 0.423) | 5.1 mm | 0.8° |
| 11 | 组合：前伸+左移+自转 | (0.399, 0.057, 0.448) | (0.400, 0.056, 0.442) | 5.7 mm | 1.1° |
| 12 | 回中位 | (0.360, 0.000, 0.428) | (0.360, 0.000, 0.423) | 5.1 mm | 0.8° |

（位置误差 ~5 mm / 姿态 ~0.8° 是"慢慢走过去"的稳态跟踪差，IK 自身残差 0.1~1.6 mm；
关节速度 ≤0.06 rad/s、指令力矩 ≤4.6 N·m；第 8~10 步位置不变、姿态各转 25° ✓。）

**顺带修掉的一个小缺陷（同属本映射）**：第 1 行与第 3 行的目标**完全一样**
（都是 `x = 0.360`）——因为半径下限 `M20_VR_RADIUS_MIN` 默认 0.12 m，而默认站姿的
半径只有 **0.1092 m**：锚点自己就落在下限之内 ⇒ "手停在标定点"这个最基本的输入
**回不到锚点**，往回拉 1.1 cm 以内完全没反应。现在下限取
`min(M20_VR_RADIUS_MIN, r_anchor)`（`M20_VR_RADIUS_MIN` 默认改为 0.05），锚点永远
可表示、标定点两侧手感一致。⚠️ 这个修正**已提交、但还没在窗口里复跑**：下次开窗口
时确认（中位应回到 0.3492、后收 4 cm 应到 0.3092）。

---

**7. 回归**：`tests/vr_arm_chain_check.py`（新增，`tests/run_all.sh` 里 `VRA=0` 可跳）。
它用假头显发"侧移 + 前伸 + 上下 + 手腕三轴转动"，断言：IK 残差 p95 ≤ 5 mm、
安全网拒发 ≤ 20 拍、限位停留 ≤ 0.2 s、|dq| ≤ 4 rad/s、力矩 ≤ 40 N·m、跟手误差
p95 ≤ 40 mm。**对旧行为会 FAIL**（实测：残差 100.5 mm、joint5 停限位 8.46 s、
21.3 rad/s、156.8 N·m），对新行为 PASS。键盘路径的回归：`--mode arm_move` /
`arm_mixed` / `no_arm` 全部 PASS，`check_policy_interface.py` L1 PASS。

---

### DEF-039 `2026-10-12` `ee_goal` 四元数用了与训练相反的符号分支 ⇒ 零速度命令下 ~0.045 m/s 持续前向漂移

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-12` |
| 类型 | `缺陷`（接口约定：观测的**原始分量**） |
| 状态 | `已修`（sim2sim 端到端实测；真机侧同一函数已同步，待上机复核） |
| 影响面 | `二者` |
| 关联 | 工作区（建议分支 `fix/ee-goal-quat-sign`）；`rl_control_state.hpp::ee_goal_`、`arm_stub_node.py::DEFAULT_EE_GOAL_BODY`、`arm_real_adapter.py`、`arm_udp_bridge.py`、`arm_controller.py::align_quat_sign`、`check_policy_interface.py::EE_GOAL_DEFAULT`、`tests/sim2sim_smoke.py`、`tests/arm_real_bridge_check.py`；训练仓库 `docs/deploy_sim2sim_sim2real_zh.md` §6 与 `docs/review/DEFECT_LOG_zh.md`（known_issues ⑫ 的实测值） |

**1. 现象（怎么发现的）**

* 触发：`rl_deploy` 一进 RL（键盘 `z` → `c`）就开始缓慢**单向前进**（用户报的"静止命令下的偏移"）；不摔、不抖，`[SIM-RT]` 实时因子全程 1.000。
* 实测 1（`tests/sim2sim_smoke.py --mode rl --duration 45`，平地模型 `policy/m20_piper_flat_20260930`）：Δx **+1.69 m**、Δy −0.49 m、Δyaw **−24.3°**；稳态轮速指令 fl −2.74 / fr +0.22 / hl −1.80 / hr +2.11 rad/s（左右反号 ⇒ 一边前进一边左转）。
* 实测 2（下面新写的零命令用例，同一套遥测口径，30 s × 6 次）：Δx **+1.339 m**、Δyaw **−20.8°**、vx 0.0446 m/s、wz −0.69°/s；6/6 复现，逐次数值几乎完全一致 ⇒ 是**确定性偏置**，不是随机漂移。

**2. 根因**

* `ee_goal`（观测 73..79）的四元数取的是"旋转矩阵→四元数"那一支（`mat_to_quat`，大致 w≥0）：默认站姿 = **(+0.7373, 0, +0.6756, 0)**。
* 策略训练时吃的是 PhysX `subtract_frame_transforms(root_quat, ee_quat)` 的**链式合成**结果，同一姿态 = **(−0.7373, 0, −0.6756, 0)**（训练仓库 known_issues ⑫ 的实测：`pose_command_b[0] = (0.3492, 0, 0.4327, −0.7373, …)`）。
* 本仓库用 MuJoCo 本身复算：`mju_mulQuat(conj(xquat[base_link]), xquat[gripper_base])` = **(−0.7373, 0, −0.6756, 0)**，与 PhysX 一致；只有"矩阵→四元数"给正号。也就是说 `scripts/check_mjcf_contract.py`（`EXPECT_GRIPPER_BASE_QUAT_WXYZ`）一直在断言负号，而**运行期代码喂的是正号**——内部自相矛盾。
* 反号不是"同一个旋转"意义上的无害差别：策略吃的是 4 个原始分量，整体取反 = 4 维分布外输入。标称观测下只翻这个符号，策略输出最大变化 **0.39**（轮速 −1.66 → −2.43 rad/s）⇒ 一个恒定的轮速偏置。
* 排除实验（都不是原因）：① 纯 `[MJCF + ONNX + 契约观测]` 探针（去掉 ROS/线程/软启动/实时因子）能复现同一漂移，且用部署自己记录的进 RL 姿态能对到 Δx +1.73 m/40 s、末轮速 −2.735；② 物理步长 0.2 ms 与 1 ms 结果相同（不是数值伪影）；③ 地面摩擦 0.4 / 1.0 / 1.5 只差几个百分点；④ 5 kHz 采样的机身陀螺最大 0.11~0.31 rad/s（无高频污染）。

**3. 修正**

* 新增 `align_quat_sign(q, ref)`（`arm_controller.py`）：把 q 翻到与 ref 同一符号分支；缺省 ref = `EE_DEFAULT_QUAT_TRAIN = (−0.7373, 0, −0.6756, 0)`。发布时传"上一帧发布值"⇒ **逐帧连续**（姿态过 180° 时符号会跳，只在第一帧对齐不够）。
* 常量改成训练分支：`rl_control_state.hpp` 的默认 `ee_goal_`、`arm_stub_node.py`、`arm_real_adapter.py`、`arm_udp_bridge.py`、`check_policy_interface.py::EE_GOAL_DEFAULT`（L1 自检）、`tests/sim2sim_smoke.py` 的 no_arm 期望值、`tests/arm_real_bridge_check.py`。
* 发布路径全部过 `align_quat_sign`：`arm_controller._tick` 的状态发布、`arm_real_adapter._publish_ee_state`、`arm_udp_bridge` 的 Rx 节点。四条路径的 IK/运动学逻辑不受影响（四元数只用于观测）。
* 备选与否掉：只把常量取反在"臂恰好停在默认姿态"时也对，但臂一动（过 180°）符号分支就不连续 ⇒ 否掉。

**4. 结果（验收）**

* 单变量 A/B（30 s、n=6、同一 runner；唯一差别 = 喂 +0.7373 还是 −0.7373）：

| | Δx | Δy | Δyaw | vx | wz | 有效 |
|---|---|---|---|---|---|---|
| 改动前 | **+1.339 m** | −0.342 m | **−20.8°** | 0.0446 m/s | −0.69 °/s | 6/6 |
| 改动后 | **+0.206 m** | +0.011 m | **−0.81°** | 0.0069 m/s | −0.027 °/s | 5/6 |

  ⇒ 前向漂移降到 **1/6.5**、偏航漂移降到 **1/26**（每次 run 约 30 s 仿真、实时因子 1.000）。
* 回归：`check_policy_interface.py` **L1 PASS**；`--mode no_arm` **PASS**（实测第一拍 `ee_goal = (0.3492, 0, 0.4327, −0.7373, 0, −0.6756, 0)`）；`--mode rl` **PASS**；`--mode arm` **PASS**（臂偏差 0.0207 rad / 6.6 N·m）。
* **副作用专项排除（入口瞬态 DEF-018）**：`tests/sim2sim_smoke.py --mode rl --repeat 12` 在**改动后**是 4/12（33%），在**改动前**（外部按 +0.7373 发布 `/ARM_TELEOP_STATE`）同样是 **4/12（33%）** ⇒ 今天的入口失败率是环境/当天状态（DEF-034 同源）不是本次改动引入的；本改动方向与 2026-10-09 记录的 `rl` 0/6、DEF-018 历史区间 0~33% 一致。
* 遗留：① 进 RL 的 1~4 s 入口瞬态（**DEF-018**）仍在（当天 33%，与本改动无关）。② 零速仍有 ~0.007 m/s 残留（训练侧 (0,0,0) 档 `err_vel_xy` 本来就是 0.10 m/s 量级 ⇒ 属策略侧，不是本缺陷）。③ 真机 `arm_real_adapter` 已同步，待上机复核。④ 本次用来量漂移的探针在 `.tmp_models/drift/`（gitignore），`policy_debug.txt` 已改名为 `policy_debug.txt.pre_2026-10-12.bak`（见 DEF-040）。

---

### DEF-040 `2026-10-12` `no_arm`/`arm_mixed` 断言读的是 append 的 `policy_debug.txt` 的**第一块** ⇒ 读到历史 run（未修）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-12` |
| 类型 | `缺陷`（测试口径） |
| 状态 | `未修`（一行可修，见下） |
| 影响面 | `sim2sim`（只影响测试结论） |
| 关联 | `tests/sim2sim_smoke.py::main`（`if args.mode in ("arm_mixed","no_arm")` 段）、`M20_PIPER_DEBUG` 的 dump（`m20_piper_policy_runner.hpp` 用 `std::ios::app` 打开 `policy_debug.txt`） |

**1. 现象**：改完 DEF-039 后跑 `--mode no_arm`，断言报 `ee_goal 不是默认位姿（… 0.7373 …）`，但 `arm_stub_node` 的日志明明白白打的是 `-0.7373`；`policy_debug.txt` 当时已有 **82 个** `tick 0` 块（该文件只 append，从不截断），而解析代码取的是**第一个** `tick ` 块 ⇒ 断言读的是 2026-10-09 那次历史 run 的观测（`arm_obs_last` 同理，取的是整个文件的最后一块，可能来自另一次 run）。

**2. 修法（建议）**：跑之前把 `policy_debug.txt` 截断/改名，或把 dump 写到每次 run 自己的路径（`M20_PIPER_DEBUG=<path>`），解析时只读本次 run 的那一份。

---

### DEF-029 `2026-10-09` 臂 PD 阻尼过大（20→8）+ 臂速度上限对齐到 3 rad/s

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 缺陷（增益）/ 特性（新增约束） |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | 训练侧 DEF-064 / DEF-065；`arm_controller.py`、`mujoco_simulation_ros2.py`、`piper_arm_interface.hpp`、`check_mjcf_contract.py` |

**1. 现象（训练侧实测）**：臂 `|qd|` 到 5 rad/s 时阻尼力矩 `20×5 = 100 N·m`
正好吃满限幅 ⇒ 关节长期饱和（逐档 `|tau|` 均 53.4 N·m、饱和 23.5%、原地抖）；
`|qd|` p99 恒为 **5.0**（贴 USD 的 `maxJointVelocity`），超速时间占比 33%。

**2. 根因**：① 阻尼项相对刚度太大（`20` 在 5 rad/s 就打满 100 N·m）；
② actuator cfg 里的 `velocity_limit=3.0` 在 `DelayedPDActuator` 里只参与力矩裁剪、
**不限速**，真正生效的是 USD 的 5 rad/s。

**3. 修正**：
* **臂 PD 300/20 → 300/8**（刚度不动）。训练实测：22.5 N·m、饱和 0%，
  跟踪还更好（4.14 → 2.19 cm）；只降刚度是错的（150/8 跟踪退到 2.89 cm）。
* **臂关节 1~6 速度上限按 3 rad/s**：训练侧启动事件 `align_joint_velocity_limits`
  把 PhysX DOF 上限从 USD 的 5 改成 3 ⇒ `|qd|` p99 3.00、超速 5~18%。
  部署侧在 `arm_controller.py` 记下 `ARM_VEL_LIMIT = 3.0`，**真机由
  agx_arm_ctrl / 驱动器侧限制，别放宽到 5**。
* 三处臂增益常量同步：`arm_controller.py`（`ARM_KP/KD`）、
  `mujoco_simulation_ros2.py`（`ARM_DEFAULT_KP/KD`）、
  `piper_arm_interface.hpp`（`ResetToDefault`），以及 `check_mjcf_contract.py`
  的 `EXPECT_ACTUATOR`。

**4. 结果（验收）**：`check_mjcf_contract.py` 的四种执行器增益断言改成 300/8、
286/5（仿真侧 MuJoCo 显式 PD 用同一组常量）；真机速度上限按第 §3 表设 3 rad/s。

---

### DEF-030 `2026-10-09` 夹爪 PD 4000/200 → 286/5（位置环原来长期饱和）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 缺陷（增益） |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | 训练侧 DEF-058；`arm_controller.py`、`mujoco_simulation_ros2.py`、`piper_arm_interface.hpp` |

**1. 现象（训练侧方波阶跃实测）**：夹爪 `4000/200` 下位置环**实际失效**——
饱和 88~100%、稳态误差最大 0.0314 rad（行程的 90%）、多数段到不了目标。

**2. 根因**：刚度/阻尼相对夹爪 `armature=0.01` 过大，`ctrlrange` 只有 10 N·m
⇒ 每次都给到限幅、长期饱和。

**3. 修正**：**286/5**（= 按"满行程 0.035 rad 误差刚好给到 10 N·m 限幅"标定），
阻尼取 ζ≈1.5；开合指令本来就已是 `±0.035`（不超行程），无需改。

**4. 结果（验收）**：训练实测饱和 **0%**、上升时间 40~80 ms、稳态误差 ≤0.0005、
开到底误差 0.005 → 0.0001；夹持力上限不变（仍能顶到 10 N·m）。

---

### DEF-031 `2026-10-09` 部署默认策略切到多地形 run `cloud_slowvx20k`（接口不变，仅换权重）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 特性（换策略） |
| 状态 | `已修` |
| 影响面 | 策略侧 |
| 关联 | `policy/m20_piper_rough_20261002/`、`rl_control_state.hpp`、`scripts/check_policy_interface.py`、`scripts/preflight_real_deploy.py` |

**1. 现象**：训练侧新 run `2026-10-02_00-44-42_cloud_slowvx20k`（多地形）导出
了一份新 `policy.onnx`；旧默认目录是平地 run `2026-09-20_00-50-31`。

**2. 根因**：非缺陷——训练侧把默认部署模型换成了多地形 `cloud_slowvx20k`
（见训练仓库 `docs/deploy_code_delta_2026-10-09_zh.md` §5.4）。

**3. 修正**：
* 新策略目录放在 `policy/m20_piper_rough_20261002/`（`policy.onnx` + `policy.pt` +
  `policy_layout.json`），并补上部署侧必需的 `joint_order_native` /
  `joint_order_action`（与旧目录逐项相同——接口一个字没变：obs 83 / history 10×70 /
  action 16 / 增益与缩放全同）。
* 默认目录改成新 run：`rl_control_state.hpp`、`scripts/check_policy_interface.py`、
  `scripts/preflight_real_deploy.py`。
* 旧目录 `policy/m20_piper_history_20260920/` **保留**做 A/B 与平地回滚。

**4. 结果（验收）**：把两份 `policy.onnx` 逐项对照（用本仓库
`third_party/onnxruntime` 编的最小校验器，复刻 `check_policy_interface.py` 的
形状/名字/标称输出三项）：两者输入输出名 `policy_obs(83)/history_flat(700)` →
`action(16)` 一致；标称观测下输出都有限、动作幅值上限分别 0.958 / 1.459（< 10）；
两者输出不同（确认是不同权重）。ONNX↔TorchScript 数值对照需容器内的
onnxruntime，容器未起，记为待补跑（`bash tests/run_all.sh` 的 L1）。

---

### DEF-032 `2026-10-09` 核对：IK 输出夹关节限位 / EE 复位锚定 / 碰撞盒（部署侧结论）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 核对（无需改 / 不适用） |
| 状态 | `已核对` |
| 影响面 | 臂侧 |
| 关联 | 训练侧 DEF-054 / DEF-034 / DEF-066 |

**1. IK 输出必须夹关节限位（训练侧 DEF-054 新增保护）** —— **部署侧已满足**：
`arm_controller.ik_step()` 每步返回 `np.clip(q + dq, ARM_JOINT_LIMITS)`，
限位表与 URDF/MJCF 一致；不需要额外改动。

**2. EE 命令复位锚定（训练侧 known_issues ⑫ / DEF-034）** —— **部署侧天然满足**：
策略观测里的 `ee_goal` 由 `arm_controller`（IK 目标）或 `arm_real_adapter`（FK
当前位姿）持续发布，复位键 `ee_reset` 取的也是当前 FK，不用常量 0。

**3. EE 目标碰撞盒改成机体局部系（训练侧 known_issues ⑭ / DEF-066）** ——
**部署侧不适用**：本仓库没有"EE 目标不得穿身体"的采样检查（目标来自遥操作，
不是采样器），无对应实现。

---

### DEF-033 `2026-10-09` `run_all.sh` 把 `arm` 档当"0 失败"，与 DEF-018 不自洽（**未改，待确认**）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 缺陷（测试口径） |
| 状态 | **未修**（建议改法已给，等确认） |
| 影响面 | 测试 |
| 关联 | `tests/run_all.sh`、`tests/sim2sim_smoke.py`、`DEF-018`、TODO P2-7 |

**1. 现象**：2026-10-09 在容器 `m20_piper_ros` 跑完整 `tests/run_all.sh`，
除 `arm` 外各档全绿，但 `arm` 档偶发触发倾角接管导致整轮报红。

**2. 根因**：
* `arm` 档同样会 `z`→`c` 进 RL，因此同样吃 **DEF-018** 的入口瞬态
  （策略在进 RL 后几秒偶发边缘发散）；
* 但 `tests/run_all.sh` 只对 `rl / walk / arm_move / no_arm / arm_mixed` 用 `--repeat`
  + 失败率判据，`arm` 走"单跑、要求 0 失败"的分支；`sim2sim_smoke.py` 内部的
  容忍列表是 `rl / walk / arm_move / no_arm`，也不含 `arm`。

**3. 证据（为什么不是本次改动引入）**：
* `arm` 档各跑 12 次：新多地形策略 **9/12 通过（失败 25%）**、旧平地策略
  **8/12 通过（失败 33%）** ⇒ 两者同量级；
* 失败现场遥测：臂关节力矩全程 ≤5 N·m（倒地瞬间 21.9，限幅 100），倒下由**轮子/底盘**
  发散引起（零速度命令下 `wcmd` 涨到 47.9 rad/s、`tilt` 1°→55°）⇒ 与臂增益无关。

**4. 建议改法（一行，未应用）**：`run_all.sh` 的 `if` 条件加 `arm`；
`sim2sim_smoke.py` 的 `limit` 元组加 `"arm"`，即用同样的"失败率 ≤ 1/2"判据。
见 TODO P2-7。

---

### DEF-034 `2026-10-09` 宿主机降频 ⇒ 仿真跑不到实时 ⇒ 进 RL 必"乱动/摔倒"（**环境，非代码**）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 环境/性能（不是缺陷，但现象很像缺陷） |
| 状态 | `已定位`（对策：插电 / 关窗口 / 看 `[SIM-RT]`） |
| 影响面 | 仿真 |
| 关联 | `mujoco_simulation_ros2.py`（`[SIM-RT]` 打印 + `M20_SIM_RENDER_INTERVAL`）、`state_machine_base.h` 的 1 kHz timerfd、DEF-018 |

**1. 现象**：手工按 `z`（站立）→ `c`（进 RL）后，底盘在 t≈7~9 s 开始"乱动"并翻倒，
`[TAKEOVER!] tilt/leg fold` 接管；但同样的用例用 `tests/sim2sim_smoke.py` 跑却偶尔全过。

**2. 根因（实测数据）**：宿主笔记本在**电池供电**上：

| 项 | 值 |
|---|---|
| 电源 | `BAT0 discharging 26%`（≈36 min）、`ADP0 online=0` |
| CPU 调频 | `governor=powersave`、`platform_profile=low-power`、`EPP=power` |
| 实际频率 | 全核 **0.8~1.0 GHz**（i9-12900H 上限 4.9 GHz） |
| 造成的结果 | MuJoCo 仿真实时因子从 **1.00 掉到 0.63~0.82** |

而 `rl_deploy` 的策略是**按墙钟 50 Hz** 跑的（`state_machine_base.h` 用 1 kHz timerfd，
每 4 tick 一个策略周期），设计上要求"仿真 ≈ 实时"（仿真主循环里那段注释就是在说这件事）。
仿真落后墙钟 ⇒ 同一个策略动作序列在**仿真时间**里被压缩成 ~1.5 倍速 ⇒ 训练分布外 ⇒ 乱动/翻倒。

**3. 对照实验（同一份代码、同一条命令）**：

| 条件 | 实时因子 | `--mode rl` 结果 |
|---|---|---|
| 插电时（早先） | **1.000** | 通过（0/6 失败） |
| 电池 + 降频（现在） | **0.67~0.70** | **3/3 失败**（t≈9 s 触发摔倒判据） |

**4. 对策 / 新增工具**：
* 插上电源再跑（最省事）；需要立刻用电池跑就提频：
  `sudo cpupower frequency-set -g performance` 或
  `echo performance | sudo tee /sys/firmware/acpi/platform_profile`。
* 仿真节点新增 **1 Hz 实时因子自检**：`[SIM-RT] 仿真 x.xxs / 墙钟 y.yys -> 实时因子 0.xx`，
  低于 0.95 会附 ` <== 仿真落后墙钟，策略速率会被压缩（进 RL 易乱动）`。
  **看到这条持续 <0.95，就别指望窗口里的策略表现**。
* 渲染间隔改成可调：`M20_SIM_RENDER_INTERVAL`（默认 `50`=20 fps，调大=更省 CPU）。
  实测把 50 调到 200 只能把因子从 0.62 抬到 0.69 ⇒ **主因是整机降频，不是渲染**。

---

### DEF-035 `2026-10-09` 默认策略切到平地模型 `cloud_cap12_20k` + `push` 用例推力重新标定

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 特性（换策略）+ 测试参数标定 |
| 状态 | `已修` |
| 影响面 | 策略侧 / 测试 |
| 关联 | `policy/m20_piper_flat_20260930/`、`rl_control_state.hpp`、`scripts/{check_policy_interface,preflight_real_deploy}.py`、`tests/sim2sim_smoke.py` |

**1. 现象**：训练侧 `deploy_code_delta_2026-10-09_zh.md` §5.4 指定"平地用
`2026-09-30_19-36-00_cloud_cap12_20k`"，该 run 的导出放进
`policy/m20_piper_flat_20260930/` 后，`check_policy_interface.py` 报
`[FAIL] layout 里没有 joint_order_native 字段`（导出的 layout 不含部署侧两个关节序字段）。

**2. 根因**：非缺陷 —— 导出脚本只给"接口契约"，`joint_order_native` /
`joint_order_action` 按约定由部署侧补（见 `policy/README.md` 换策略第 3 步）。

**3. 修正**：
* 给 `m20_piper_flat_20260930/policy_layout.json` 补齐两个关节序字段（与另两个目录逐项相同）。
* 默认策略目录切到它：`rl_control_state.hpp`、`check_policy_interface.py`、
  `preflight_real_deploy.py`（多地形改
  `M20_POLICY_DIR=/root/m20_piper_ws/src/M20_sdk_deploy/policy/m20_piper_rough_20261002`；
  ⚠️ 这个环境变量按进程 cwd 解析，别写 `policy/...`）。
* **`--mode push` 默认推力 800 → 1500 N**（见下）。

**4. 结果（容器实测，宿主机已插电、实时因子 1.000）**：

| 项 | 结果 |
|---|---|
| L1 | ✅ PASS（`\|a\|max` 1.093；ONNX↔TorchScript **1.36e-06**） |
| L3' `check_mjcf_contract` | ✅ 10 PASS / 0 FAIL |
| L3 `hold` | ✅ height 0.497 m、tilt 1.5° |
| L3 `rl`（repeat 6） | ✅ **0/6**（0%） |
| L3 `walk`（repeat 6） | ✅ 2/6（33%，容差 ≤50%；命令 +0.7 m/s） |
| L3 `arm` | ✅ 臂偏差 0.0207 rad、力矩 6.6 N·m |
| L3 `arm_move`（repeat 6） | ✅ **0/6**，末端 ee_x 0.3492→0.6002 m（**+0.25 m**） |
| L3 `wheel_step` / `no_arm` / `arm_only` / `arm_mixed` | ✅ 四轮 +5.00 rad/s；无臂口径一致；IK 残差 0.12 mm；观测臂项变化 1.03 rad |
| L3 `push`（800 N） | ⚠️ **没触发接管**——机器人**扛住了**（高度 0.511→0.441 m、倾角 3.9°、约 1.5 s 自恢复） |
| L3 `push`（1500 N / 2500 N） | ✅ 均触发 `[TAKEOVER!] leg fold` → `joint_damping`；1500 N repeat 3 = **3/3** |

> `push` 的判据是"**必须**被打倒并触发接管"，属于"用足够大的力验证安全通路"。
> 平地模型比 2026-09 的模型抗推得多，800 N 已打不倒 ⇒ 用例误判成 FAIL，
> 所以把默认值标定到 1500 N（并在文件头/文档里写明原因）。

**5. 顺带**：插电后实时因子回到 **1.000**（电池时 0.67），印证 DEF-034。

---

### DEF-058 `2026-10-13` 头显页面依赖公网 CDN 拉 A-Frame ⇒ 内网里页面"卡住"、只有一张卡片、点哪都没反应

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-13` |
| 类型 | 缺陷（离线部署） |
| 状态 | `已修 · 已闭环`（**2026-10-13 操作员实测：头显刷新后恢复正常，可正常遥操**） |
| 影响面 | 真机 / 遥操作 |
| 关联 | `third_party/XLeVR/web-ui/index.html`、新增 `scripts/fetch_aframe.sh`、`scripts/sync_to_robot.sh`、`vr_teleop_device.py`、`docs/DEPLOY_REAL_VR_zh.md` §1.6/§5.2/§8/§9 |

**1. 现象**：头显里能打开 `https://10.21.31.103:8443`，但**页面卡住**：
屏幕中间只有一张写着 `VR Teleoperation` 的卡片，没有 `Enter VR` / `Start Controller Tracking`
之类的按钮，**点哪儿都没反应**（操作员问"是不是 M20 主板跑不动这些程序"）。

**2. 根因**：**不是算力问题**——上位机的 HTTPS:8443、WS:8442、静态文件都正常服务
（地址能打开本身就是证据）。真正的原因在页面这一侧：

* `web-ui/index.html` 第 8 行原本是
  `<script src="https://aframe.io/releases/1.7.1/aframe.min.js"></script>`
  —— **公网 CDN**。上位机和头显都在内网、没有外网 ⇒ 这个脚本拉不下来：
  * 它在 `<head>` 里且**没有 `defer`/`async`** ⇒ **阻塞 HTML 解析**，
    浏览器一直等这个永远不来的响应，页面就"卡住"（等到超时才把后面解析出来）；
  * `web-ui/vr_app.js` 第 3 行顶层就是 `AFRAME.registerComponent('controller-updater', …)`
    ⇒ 抛 `AFRAME is not defined` ⇒ **整个文件一行注册都执行不到**
    （包括那个绿色 `Start Controller Tracking` 按钮的注入逻辑）；
  * 没有 A-Frame 时 `<a-scene>` 只是个未知标签、不会渲染，A-Frame 自带的
    `Enter VR` 按钮当然也不存在。
* 同一页面的 `interface.js` 是**另一个文件**，它照常执行：检测到"设备支持 `immersive-vr`"
  就把桌面 UI 隐藏、只显示 `#vrContent` —— 那是 `styles.css` 里
  `position:fixed; left/top:50%; z-index:9999` 的**纯静态 div**（里面只有标题文字），
  没有任何事件 ⇒ **屏幕上就只剩中间那张卡片，且点什么都没反应**。
* 顺带一个会放大"卡住的观感"的问题：HTTPS 服务用的是**单线程**
  `http.server.HTTPServer`，而浏览器会并发开好几条连接
  （`index.html` + 3 个 `.js` + `.css` + `.jpg`），一次只服务一条 ⇒ 加载被拉长。

**3. 修正**：
* `web-ui/index.html`：改用**同目录的离线副本** `aframe.min.js`（`defer`，与
  `vr_app.js`/`interface.js` 保持先后顺序），页面上再无阻塞式 CDN 引用；
  并新增兜底脚本：本地引擎缺失时 ① 立刻在页面顶部插一条**红色横幅**（写清缺什么文件、
  怎么修 —— 头显里看不到 devtools，这条是给操作员看的）；② 若头显恰好能上外网，
  从 jsdelivr → unpkg → aframe.io 依次补一份并**自动刷新一次**
  （`sessionStorage` 标记只试一次，避免刷新循环）。
* 新增 `scripts/fetch_aframe.sh`（**在有外网的机器上跑**）：6 个镜像
  （官方 release / jsdelivr / unpkg / cdnjs / bootcdn / staticfile）依次尝试，
  校验"够大 + 不是 HTML 错误页 + 内含 `AFRAME`"后落到 `web-ui/`；
  `--check` 只做自检，`--url` / `--dest` 可指定。
* `scripts/sync_to_robot.sh`：同步前打印 A-Frame 是否就位；缺了直接给出
  "头显里会怎样 + 该跑哪条命令"的警告。
* `vr_teleop_device.py`：启动横幅新增
  `头显页面: 已自带离线 A-Frame（xxxx KB）`（缺失时 ⚠️ + 修法）；
  HTTPS 服务换成 `http.server.ThreadingHTTPServer` + `daemon_threads=True`。
* 文档：`DEPLOY_REAL_VR_zh.md` 新增 **§1.6**（离线引擎这一步 + 怎么确认好了）、
  §5.2 头显侧提示、§8 排障两行、§9 真机实测项（已勾）、§10 命令速查两行。

**4. 验收**（本机静态/单测；容器与头显此刻都不可用）：

| 项 | 结果 |
|---|---|
| `index.html` 是否还有阻塞式 CDN 引用 | **无**：`<script src="aframe.min.js" defer>`，`defer` 顺序 = `aframe.min.js → vr_app.js → interface.js`；CDN 地址只出现在兜底列表里（1 处） |
| 页面内联脚本语法（`node` 逐段 `new Function()` 解析） | **2/2 OK**（2327 / 337 字符） |
| `vr_app.js` / `interface.js` 语法（`node --check`） | **OK** |
| `_display_info()` 两个分支（本机模拟：真缺文件 / 临时放一份 400 KB 假文件） | 缺 → `⚠️ 头显页面缺离线引擎 …` + `fetch_aframe.sh`；有 → `头显页面: 已自带离线 A-Frame（390 KB）` |
| `fetch_aframe.sh --check` | 缺文件 → `exit 1` + 指出缺哪个路径；假文件（585 KB、含 `AFRAME`）→ `OK … exit 0`；HTML 错误页 → 判为太小/HTML（拒绝） |
| `vr_teleop_device.py` 语法 | `python3 -m py_compile` OK |
| **头显端实机复核** | ✅ **2026-10-13 操作员实测通过**：把 `aframe.min.js` 放到 `web-ui/` 后头显刷新页面 → 顶部红条消失、按钮出现，**机械臂遥操正常**（闭环） |
| "要下载什么"的清单 | 已汇总为 `docs/OFFLINE_ASSETS_zh.md`（A-Frame + 上位机 wheel + 明确不用下的 scipy/numpy/pynput/torch + 换机器后重做哪些），并被 `DEPLOY_REAL_VR_zh.md` §1.5/§1.6、`COMMANDS_SIM2REAL_zh.md` §0.3 引用 |

---

### DEF-057 `2026-10-13` VR 节点把地址打印成 `https://localhost:8443`（真机内网没有默认路由）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-13` |
| 类型 | 缺陷（可用性） |
| 状态 | `已修` |
| 影响面 | 真机 / 遥操作 |
| 关联 | `vr_teleop_device.py::_display_info`（新增 `_local_ipv4_candidates`）、`third_party/XLeVR/xlevr/config.py`、`docs/DEPLOY_REAL_VR_zh.md` §5/§8 |

**1. 现象**：真机上起 `vr_teleop_node.py`，打印
`Open in your headset browser:  https://localhost:8443` —— 头显照这个地址当然打不开。

**2. 根因**：地址探测用的是"UDP connect 到 8.8.8.8 再看本机出口 IP"这个技巧，
**它需要默认路由**；机器人内网（WiFi/有线各自一个网段）没有到外网的路由 ⇒
`connect()` 抛 `ENETUNREACH` ⇒ `except` 里退回 `"localhost"`。
（顺带：`xlevr/config.py` 的 `load_config()` 用的是相对路径 `config.yaml`，
不在 XLeVR 目录下启动就打印 `Config file config.yaml not found, using defaults`
—— 也就是说**改那个 yaml 不生效**，和这个警告是同一个"路径按 cwd 解析"的坑。）

**3. 修正**：
* 新增 `_local_ipv4_candidates()`：**离线**枚举本机非回环 IPv4，顺序
  ① `ip -4 -o addr show` ② `hostname -I` ③ 才用 UDP 技巧；**无线网卡排前面**；
* `_display_info()` 改成打印**所有候选**（首选一行 + "本机其它地址"一行 + 用法提示），
  并支持 `M20_VR_IP=<ip>` 显式指定；一个都识别不到时打印人工排查指引（不再有误导性的 `localhost`）；
* `xlevr/config.py` 改成**先找包旁边的 `config.yaml`**（`Path(__file__).parents[1]`），
  找不到才退回 cwd/默认值 ⇒ 警告消失，而且改 yaml 真的生效。
* 服务端本来就绑 `0.0.0.0`（HTTPS + WS 都是），页面里的 `wss://<host>:8442`
  跟着 `window.location.hostname` 走 ⇒ **HTTPS 通了 WS 就通**，用正确 IP 手输也能连。

**4. 验收（本机单测；容器此刻不可用）**：

| 项 | 结果 |
|---|---|
| 解析（模拟真机 `ip` 输出：lo + end0 10.21.31.103 + wlan0 10.21.41.1 + docker0） | `wlan0 10.21.41.1` 排第一、回环被跳过、顺序正确 |
| 打印（无覆盖） | 首选 `https://10.21.41.1:8443` + `本机其它地址: 10.21.31.103 (end0)` + 提示 |
| 打印（`M20_VR_IP=10.21.31.103`） | `https://10.21.31.103:8443 （由 M20_VR_IP 指定）` |
| 打印（三个手段都拿不到地址） | 打印 `⚠️ 没能自动识别本机 IP … 用 M20_VR_IP=<你的IP>`（不再是 localhost） |

---

### DEF-056 `2026-10-10` 真机（上位机）不能联网 ⇒ VR 节点起不来（scipy 缺失）（原 DEF-043）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-10` |
| 类型 | 缺陷（离线依赖） |
| 状态 | `已修` |
| 影响面 | 真机部署 / 遥操作 |
| 关联 | `third_party/XLeVR/xlevr/_rotation_compat.py`（新增）、`vr_teleop_device.py`、`third_party/XLeVR/xlevr/inputs/vr_ws_server.py`、`requirements_teleop.txt`、`docs/DEPLOY_REAL_VR_zh.md` §1.5 |

**1. 现象**：真机上位机上起 `vr_teleop_node.py` 直接报
`ModuleNotFoundError: No module named 'scipy'`（`vr_teleop_device.py` 第 35 行的
`from scipy.spatial.transform import Rotation as R`），而上位机**不能联网**、装不了包。

**2. 根因**：VR 链路里 scipy 只被用来做**四元数/欧拉角换算**：
`vr_teleop_device.py`（`from_quat` / `from_euler("x")` / `inv()` / `*` / `as_euler("XYZ")`）
与 XLeVR 自带的 `xlevr/inputs/vr_ws_server.py`（`from_euler("xyz")` / `as_quat` /
`inv` / `*` / `as_rotvec`）。为这点功能在离线机器上装 30 MB 级的 scipy（还要和
numpy 大版本配套）不划算。

**3. 修正**：
* 新增 `third_party/XLeVR/xlevr/_rotation_compat.py`：**纯 numpy** 的
  `scipy.spatial.transform.Rotation` 最小兼容子集（上面那些方法，语义与 scipy 对齐）；
* 两个使用点都改成 `try: scipy … except ImportError: 用 _rotation_compat` ——
  **有 scipy 的机器行为完全不变**，没有的也能跑；
* `requirements_teleop.txt` 把 scipy 降级为可选；`yaml` / `websockets` 标为必需；
* `docs/DEPLOY_REAL_VR_zh.md` 新增 §1.5「上位机 Python 依赖（离线）」：下载 wheel 的
  跨平台参数（`--platform manylinux2014_aarch64 --python-version 38 --abi cp38`）、
  `pip3 install --no-index --find-links`、以及**没有 pip 时直接解压 wheel + PYTHONPATH** 的兜底。

**4. 验收**：

| 项 | 结果 |
|---|---|
| 与真 scipy 数值对照 | 2000 组随机旋转，`as_quat`/`as_matrix`/`inv`/`*`/`as_rotvec`/`from_euler(xyz,XYZ,x)`/`as_euler(XYZ)` 最大绝对误差 **≤1.0e-14**（除 `as_quat` 完全相同） |
| 屏蔽 scipy 后起 `vr_teleop_node.py` | ✅ 正常：打印 `HTTPS server on port 8443` / `VR WebSocket server on port 8442` / `VR teleop ready` / 灵敏度行 |
| 还缺什么（实测） | `yaml`、`websockets` **必需**（屏蔽任一即 ImportError）；`scipy` 已非必需 |
| 预备好的 wheel | `numpy-1.24.4` / `PyYAML-6.0.3` / `websockets-13.1`（cp38 + aarch64），在开发机 `/tmp/m20_wheels/` |

---

### DEF-055 `2026-10-10` 复位（VR 左 X/Y）后臂被旧目标拽走 + IK 节点可能因单拍异常退出（原 DEF-042）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-10` |
| 类型 | 缺陷（复位语义 / 健壮性）+ 手感 |
| 状态 | `已修` |
| 影响面 | 遥操作 / 状态机 |
| 关联 | `arm_teleop_node.py`、`arm_controller.py`、`arm_teleop_protocol.py`、`run_sim2sim.sh` |

**1. 现象（操作员）**：① 复位后机械臂会自己"飞"一下（很明显的瞬态），末端目标**没有**
回到默认位姿；② `arm_controller`（IK 那个终端）**有时候会掉**；③ 操作过程中仍有瞬态变化。

**2. 根因**（三条独立的）：
* **复位语义**：X/Y 复位后 VR 会被置为 inactive（设备的训练侧语义），`arm_teleop_node`
  于是走**键盘增量**分支 —— 那条路不看 `vr_calibrate_pulse`；同时 `/reset_sim` 是异步的，
  在复位真正落地前 `arm_controller` 还抱着**复位前的旧目标** ⇒ 它把刚被复位到默认位姿的臂
  又拽走，等再"重新锚定"时采到的是**半路位姿**。实测臂停在 `j2=0.905` 这种既非默认
  也非旧目标的位姿（与默认差 0.40 rad）。
* **节点会掉**：`_tick`/订阅回调里任何未捕获异常都会从 `rclpy.spin()` 冒出来 ⇒ 进程带
  一条 traceback 退出。最可能的是臂反馈出现 NaN/Inf 时 `np.linalg.solve` 抛 `LinAlgError`。
* **另一个"掉"的来源（很容易误判）**：`run_sim2sim.sh` 的收工清理里有
  `pkill -9 -f .../arm_controller.py` —— 按用法文档 `arm_controller` 是操作员**在另一个
  终端自己起的**，于是在 T1 按 Ctrl+C 会把那个终端一起杀掉。

**3. 修正**：
* **复位改成确定性语义**：`arm_teleop_protocol` 新增 `flag=2`（`ARM_TELEOP_FLAG_RESET_DEFAULT`）
  = "锚点取**默认关节姿态**的 FK 位姿、偏移按 0"；X/Y 复位时 `arm_teleop_node` 发一帧这种
  命令（VR 在不在都发）⇒ 目标立刻回到默认末端位姿，**不再依赖时序**。
  （`make_absolute_offset` 原来把 flag 归一化成 `0/1`，会把 `2.0` 压成 `1.0`，一并修掉。）
* **IK 节点不许死**：① 反馈非有限值直接丢弃该帧；② IK 异常兜底（沿用上一拍指令）；
  ③ 整个控制拍包 try/except（限频报错）。三层都不会让进程退出。
* `run_sim2sim.sh` 收工只收**自己启动的**进程（`pkill` 只保留仿真节点；要恢复"全清"用
  `M20_SIM2SIM_CLEANUP_ALL=1`）。
* 手感：VR 死区从"硬阈值"改成**软死区**（跨阈值不再从 0 跳到 4 mm，消除台阶感）。

**4. 验收（容器实测）**：

| 项 | 结果 |
|---|---|
| 复位后臂的稳态 | `q=[-0.00, 0.496, -0.485, 0.004, 0.003, -0.004]` ⇒ 与默认位姿最大偏差 **0.015 rad（0.9°）**（改前 0.40 rad，且位姿是"半路"的） |
| `arm_controller` 存活 | 复位前后都 alive ✓ |
| 软死区 | `_apply_deadzone` 单测：0.004→0.000、0.005→0.001、0.02→0.016（连续，无跳变） |
| 收工不再误杀 | 模拟"手起 arm_controller + T1 Ctrl+C"：arm_controller **仍在** ✓ |
| 回归 | `arm_only` / `arm` / `arm_move` 全 OK（`arm_move` 失败率 0%，IK 残差 0.18 mm），`全部通过` |

**5. 追加（操作员反馈"reset 没问题，但机械臂无法操控了"）**：复位后 VR 被置为
inactive 是**训练侧原语义**（X/Y = 停），操作员于是发现"复位后手柄不再管用"。
按操作员要求改成 **X/Y 复位后保持 VR 激活**：只把**设备的校准原点**重采一次
（偏移从 0 重新开始），不发 `_calibrate_pulse`（arm_controller 的目标由上面那条
flag=2 的"重置到默认末端位姿"负责，两条链路不互相覆盖）；
`M20_VR_RESET_KEEPS_ACTIVE=0` 可回到老语义。
三段时序实测（假 VR）：A 按 B 后 `dx=0.10` → 臂动（q₂=0.968）；X/Y → 臂回默认；
**复位后不按 B**，手再动 10 cm → 臂又跟（q₂=0.966）✓。回归仍 `全部通过`。

**6. 顺带清理**：`_apply_deadzone` 相关、`_reset_state`（只写不读的死状态）删除。

---

### DEF-054 `2026-10-10` `/VR_TELEOP` 上跑两个发布者 ⇒ 目标在两帧之间来回跳（"动作特别快/不跟手"）（原 DEF-041）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-10` |
| 类型 | 缺陷（工具/环境） |
| 状态 | `已定位`（脚本可自动检出；不是代码 bug） |
| 影响面 | 遥操作 |
| 关联 | `scripts/check_vr_arm_chain.py`（新增"发布者数量"检查）、`COMMANDS_SIM2SIM_zh.md` §5 判据表 |

**1. 现象**：机械臂"动作特别快、不像电机能执行"、"某个方向不跟手"、夹爪"瞬间开合"。

**2. 根因（实测复现）**：`/VR_TELEOP` 上有**两个发布者**时，两路命令**交替到达**
（各自 50 Hz，交错 100 Hz）。`arm_teleop_node` 每收到一帧就刷新绝对偏移，
`arm_controller` 的目标因此在两帧之间来回跳 —— 实测目标在
`x = 0.1075` 与 `0.2275`（正好差 12 cm）之间跳变 ⇒ 腕关节被反复砸到限位
（joint4 −1.64 / 限位 ±1.745）、IK 残差 20°、位置还有 4~5 cm 的耦合漂移。
**表现与操作员描述完全一致。**

**3. 修正**：自检脚本新增"每个话题的发布者数量与名字"，>1 时直接给结论；
`COMMANDS_SIM2SIM_zh.md` §5 的判据表补一行。常见原因：起了两个 `vr_teleop_node`
（或两个 `arm_teleop_node`）、换台机器/终端时旧节点没退干净。

**4. 顺带纠正一条我自己的误判**：上面这些"腕关节乱摆/IK 不收敛"是**脏数据**（双发布者）
造成的。清成**唯一发布者**后重测（只给 `dx=+0.12` 恒定）：

| 量 | 干净输入下的结果 |
|---|---|
| `ee_x` | **+0.1276 m**（命令 +0.12 ⇒ 1:1 跟手） |
| `ee_y` / `ee_z` | −0.0001 / −0.017（几乎无耦合） |
| 腕关节 Δ | q20 **−0.385 rad**（平滑，远未到限位） |
| IK 残差 | **0.08 mm / 0.000°，2 次迭代收敛** |

⇒ **臂侧（arm_controller + arm_teleop）在干净输入下是准的**；"不跟手"要先查发布者
数量与 `/VR_TELEOP` 的实际数值，而不是怀疑 IK/限位。

---

### DEF-053 `2026-10-10` VR 动臂"太快"的成因与两个可调旋钮（灵敏度可环境变量覆盖）（原 DEF-040）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-10` |
| 类型 | 特性（可调性）/ 说明 |
| 状态 | `已修`（默认值保持不变；新增旋钮） |
| 影响面 | 遥操作 |
| 关联 | `arm_controller.py:127-130`、`vr_teleop_device.py`（`SENS_ENV`）、`COMMANDS_SIM2SIM_zh.md` §5.1 |

**1. 现象（操作员）**：VR 下机械臂"动得很快"。

**2. 根因（读码 + 数字）**：
* VR 的手臂跟随是 **1:1**：`arm_pos_sensitivity = 1.0`，`command[3:6] = 手柄位移`，
  所以"手快臂就快"，臂的速度 ≈ 手的速度（直到撞上限速）；
* 上限由 `arm_controller` 的 **VR 限速**决定：`M20_VR_MAX_LIN_SPEED` 默认 **2.0 m/s**、
  `M20_VR_MAX_ANG_SPEED` 默认 **5.0 rad/s**（键盘路径只有 **0.35 / 0.8**）
  ⇒ 每个 20 ms 控制周期最多走 `2.0×0.02 = 4 cm`、转 `5.0×0.02 = 5.7°`；
* 附带：底盘 `v_x_sensitivity` 默认 **5.0**（摇杆满量程 = 5 m/s），虽然有
  `M20_VR_MAX_VX=1.0` 兜底，但**摇杆推 20% 就吃满 1 m/s**，油门很窜。

**3. 修正**：把 `vr_teleop_device.py` 的 8 个灵敏度都做成**环境变量可覆盖**
（`M20_VR_VX_SENS` / `_VY_` / `_WZ_` / `_HEIGHT_` / `_PITCH_` / `_ROLL_` /
`_ARM_POS_` / `_ARM_ROT_SENS`；**默认值 = 原值，不设变量时行为完全不变**），
并在启动时打印生效值。限速那两个本来就是环境变量，无需改代码。

**4. 验收（容器实测）**：`M20_VR_ARM_POS_SENS=0.5 M20_VR_ARM_ROT_SENS=0.4 M20_VR_VX_SENS=3`
起节点 → 打印 `arm pos 0.5 / rot 0.4`、`chassis vx 3`，进程内取值 `0.50 / 0.40 / 3.00` ✓。

**5. 给操作员的推荐值**（写在 `COMMANDS_SIM2SIM_zh.md` §5.1）：
臂限速 `0.4 m/s` / `1.0 rad/s`；臂灵敏度 `0.5` / `0.5`；底盘油门 `2.0`。

**6. 2026-10-10 收尾（操作员确认手感合适后已固化为默认值）**：
`M20_VR_MAX_LIN_SPEED=0.4`、`M20_VR_MAX_ANG_SPEED=1.0`（`arm_controller.py`）；
`arm_pos_sensitivity=0.5`、`arm_rot_sensitivity=0.5`、`v_x_sensitivity=2.0`
（`vr_teleop_device.py` 的 `Se2VRExtendedCfg`）—— 都可用环境变量回到原值。
另新增两个"真机感"旋钮（默认保持现状）：`M20_GRIPPER_SLEW`（夹爪目标限速，
默认 0 = 直接阶跃 ⇒ 仿真里 ~10 ms 到位的"瞬间开合"）与
`M20_ARM_MAX_JOINT_VEL`（关节指令限速，默认 0；设 3.0 与真机驱动器一致）。

---

### DEF-052 `2026-10-10` VR 通道缺速度限幅（前向最高 1 m/s）+ 新增臂链路自检（原 DEF-039）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-10` |
| 类型 | 特性（安全限幅）/ 工具 |
| 状态 | `已修` |
| 影响面 | 遥操作 |
| 关联 | `rl_control_state.hpp`（`ApplyVrCommand`）、`scripts/check_vr_arm_chain.py`、`COMMANDS_SIM2SIM_zh.md` §5 |

**1. 现象/风险**：VR 的速度是**摇杆连续量**，原来直接写进 `uc->forward_vel_scale`
等字段，**没有任何限幅** ⇒ 一次推杆就能给到训练分布外的速度（实测命令 1.5 m/s 时
机器人真的跑到 1.62~1.73 m/s）。

**2. 修正**：在 `ApplyVrCommand` 里按部署包线夹取，默认
**`vx ±1.0`（操作员要求）/ `vy ±0.5` / `wz ±0.7`**，可用
`M20_VR_MAX_VX` / `_VY` / `_WZ` 覆盖；启动时打印一行 `[VR] 速度限幅: ...`。

**3. 验收（容器实测，实时因子 1.000）**：VR 命令 `vx = 1.5` ⇒ 实测 **1.01 m/s**
（改前 1.62）；命令 `vx = 0.7` 不受影响（0.70→0.71）。

**4. 附带**：新增 `scripts/check_vr_arm_chain.py`（8 s 采样，逐环节判定
`/VR_TELEOP → /ARM_TELEOP → /ARM_JOINTS_CMD → /ARM_JOINTS_DATA`）。
自测：假 VR 发布者（active=1、dx 0→0.15 m）下链路全通、臂实际转动 0.35 rad；
故意不起 `arm_teleop_node` 时脚本正确指出"`/ARM_TELEOP` 没有数据"。

---

### DEF-038 `2026-10-09` `M20_POLICY_DIR` 的相对路径口径与默认目录不一致（已缓解）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 缺陷（可用性/坑） |
| 状态 | `已缓解`（根治见 TODO P4-6） |
| 影响面 | 工具/部署 |
| 关联 | `rl_control_state.hpp:245-258`、`COMMANDS_SIM2SIM_zh.md` §7、`policy/README.md`、`WORKFLOW_zh.md` 开关表 |

**1. 现象**：按仓库根写 `M20_POLICY_DIR=policy/m20_piper_rough_20261002` 起 `rl_deploy`，
进程立刻打印 `policy not found` 并退出；此时机器人不会动、只会退化到"待机保持"并缓慢下沉倾覆，
**看起来非常像"这个模型站不住/不跟命令"**（我为此白跑了 25 分钟的对多地形模型的测试）。

**2. 根因**：代码**默认**目录是按 `__FILE__` 解析的（`base/../../policy/<目录>`，等价于
`src/M20_sdk_deploy/policy/<目录>`），而 `M20_POLICY_DIR` 覆盖值是**按进程 cwd** 解析的。
两者口径不同，且错误信息只打印原始字符串，不打印解析后的绝对路径，猜不到问题在哪。

**3. 修正（本次）**：
* 错误信息改为打印 `fs::absolute(policy_dir)` **与 cwd**；
* 文档里的示例统一改成绝对路径，并在 `WORKFLOW_zh.md` 开关表里写明"按进程 cwd 解析"。

**4. 验收**：用 `M20_POLICY_DIR=<绝对路径>` 复跑，策略正常加载（日志打印 policy_dir），
命令扫描 3/3 通过；用相对路径仍会失败，但现在一眼能看出是路径问题。

---

### DEF-037 `2026-10-09` `liedown_state` 的关节限位夹取是空操作（**未修**）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 缺陷（姿态目标超限位） |
| 状态 | **未修**（一行改法已给） |
| 影响面 | 状态机（趴下） |
| 关联 | `liedown_state.hpp:48,61`、对照 `standup_state.hpp:48,61` |

**1. 现象**：走查时发现 `liedown_state.hpp:48`（hipy）与 `:61`（knee）写作
`LimitNumber(theta, lo, hi);` —— **没有接返回值**，夹取完全没生效。
`standup_state.hpp` 里一模一样的两个函数是 `theta = LimitNumber(...)`（接了）⇒ 复制粘贴掉了一截。

**2. 后果（推算）**：趴下目标 `GetKneePosByHeight(0.03)` = π − acos((l1²+l2²−h²)/(2·l1·l2))
= **3.02 rad**，而 `control_parameters.h:44-45` 的膝关节上限是 **2.758 rad**
⇒ 趴下时膝关节目标在限位**外** 0.26 rad，靠机械限位硬顶（不致命，但与设计意图不符）。

**3. 修法**：把 `liedown_state.hpp:48,61` 改成 `theta = LimitNumber(...)`；
建议顺手把这两个函数从 standup/liedown 抽成一处（现在两份重复，还有一份抄歪了）。

---

### DEF-036 `2026-10-09` 侧移/转向存在方向性不对称（**未修，需训练侧确认**）

| 项 | 内容 |
|---|---|
| 日期 | `2026-10-09` |
| 类型 | 缺陷（策略行为，疑似左右符号/增益） |
| 状态 | **未修**，待训练侧专项探针 |
| 影响面 | 策略 |
| 关联 | `tests/command_sweep.py`、`docs/review/ACCEPTANCE_2026-10-09_zh.md` §4.1、TODO P4-9 |

**1. 现象（单命令阶跃，8 s 命令 / 后 6 s 稳态）**：

| 命令 | 平地模型 | 多地形模型 |
|---|---|---|
| `wz +0.5` | +0.378（76%） | +0.440（88%） |
| `wz −0.5` | **−0.027（5%）** | −0.288（58%） |
| `vy +0.5` | +0.583（117%） | **+0.046（9%）** |
| `vy −0.5` | −0.437（87%） | −0.555（111%） |

**2. 排查过的**：隔离单发（只跑该阶段 10 s）复现同样结果 ⇒ 不是阶段顺序/上一段残留；
`vx` 正负、机身姿态两个模型都正常 ⇒ 不是整体标定问题。
两个模型"坏的方向"还不一样（平地坏在左转、多地形坏在左移）。

**3. 怀疑方向**：左右侧轮子的符号/增益不对称，或训练侧 `yaw_rate`/`base_velocity`
的符号约定与部署侧不一致。

**4. 下一步**：训练仓库用同一 MJCF 跑 `vy/wz = ±0.3/±0.7` 的四象限探针，
给出"命令 vs 实测"对照；确认后要么改训练、要么在部署侧修正。
**在这条查清之前，不能说"部署包线已验证"。**

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

### DEF-028 `2026-09-29` `arm_teleop_node` 按 Ctrl+C 关不掉（raw 模式吃掉了 SIGINT）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-29` |
| 类型 | 缺陷（可用性） |
| 状态 | `已修` |
| 影响面 | 二者 |
| 关联 | `interface/robot/simulation/arm_teleop_node.py::_setup_stdin` |

**1. 现象**：真机上按 `Ctrl+C` 关不掉臂键盘节点（只有 `ESC` 能退），只能 `kill`。

**2. 根因**：`_setup_stdin()` 用 `tty.setraw()`。raw 模式会**清掉 ISIG**，
终端不再因为 `Ctrl+C` 产生 SIGINT —— 它变成普通字节 `0x03` 被按键循环读走；
而循环里只处理 `ESC(0x1b)`，`0x03` 落到"未识别按键"被忽略。

**3. 修正**：改用 `tty.setcbreak()`（同样关行缓冲/回显，逐字节读键，但**保留 ISIG**），
并在按键循环里把 `0x03` 也当作退出（兜住非标准终端状态）；`main()` 捕获
`KeyboardInterrupt` 并保证"终端恢复 + 不重复 `rclpy.shutdown()`"。

**4. 结果（验收）**：在真 TTY 下实测——`Ctrl+C` 打印
`[arm_teleop] stopping ... / stopped (terminal restored)` 并退出；`ESC` 路径不变仍可用。

---

### DEF-026 `2026-09-22` 进 RL 时 rl_deploy 会把正在被 IK 控制的臂拽回默认位姿

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-22` |
| 类型 | 缺陷（多发布者抢话题） |
| 状态 | `已修` |
| 影响面 | `sim2real` / `sim2sim`（臂栈与策略合并跑时） |
| 关联 | `interface/robot/hardware/piper_arm_interface.hpp::Start` |

**1. 现象**：臂栈（`arm_controller` 的 IK）与策略同时跑时，**按 `c` 进 RL 的那一拍**
臂会被拽回默认位姿一下（一个控制周期后又回到 IK 目标）—— 因为 `/ARM_JOINTS_CMD`
上有两个发布者：`arm_controller`（50 Hz 的 IK 目标）和 `rl_deploy` 的
`PiperArmInterface::Start()`（进 RL 时发的一条"保持默认位姿"）。

**2. 根因**：把"默认保持"也算成部署侧的职责；当外部已经在控制臂时，这条就是抢话题。

**3. 修正**：`PiperArmInterface` 也监听 `/ARM_JOINTS_CMD`；`Start()` 时若最近 0.5 s
内有人在发（典型的：`arm_controller` 在跑），就**跳过**默认保持命令并打印
`[PiperArmInterface] /ARM_JOINTS_CMD 上已有其它发布者（如 arm_controller），跳过进 RL 时的默认位姿保持命令`。
没人发时照旧发（无臂/臂静止的场景仍需要）。

**4. 结果（验收）**

| 场景 | 行为 | 实测 |
|---|---|---|
| `--mode arm` / `arm_move`（arm_controller 在跑） | 跳过默认保持 | 日志出现上面那句；`arm_move` 仍 PASS（末端 +0.239 m、IK 残差 0.18 mm） |
| `--mode no_arm`（没人控制臂） | 照旧发默认保持 | 跳过日志出现 **0** 次；`no_arm` PASS |

---

### DEF-024 `2026-09-22` 真机启动卡在 `joint data update is not finished`（自检无限等待）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-22` |
| 类型 | 缺陷（**真机阻断**） |
| 状态 | `已修` |
| 影响面 | `sim2real` |
| 关联 | `interface/robot/hardware/m20_interface.hpp::ResetPositionOffset`（原 `DdsInterface` 里也有同样写法） |
+
+**1. 现象**：真机上按 `z` 进站立时，终端反复打印：
+
+```
+0 :0
+1 :0
+...
+15 :0
+joint data update is not finished
+```
+
+每 10 s 一次、永不结束；程序卡在启动阶段（状态机还没起来），而这段时间机器人被
+`SetJointCommand(MatXf::Zero(...))` 发了**零增益** —— 也就是机器人是软的、会塌。
+
+**2. 根因**：`ResetPositionOffset()` 的启动自检要求
+**16 个关节（含 4 个轮子）每一个都"变化过一次"**才退出，而且是 `while` **无限等待**：
+
+* 收不到 `/JOINTS_DATA`（不在 SDK 模式 / ROS domain 不一致 / 话题不通）→ 位置恒 0，永不满足；
+* 收到了但**机器人站着不动**：腿被撑住、**轮子不转** → 也永不满足（轮子几乎不可能主动变化）；
+* 它的本意只是"确认反馈是活的、并据此决定多圈编码器的 ±360° 偏移（只用 hipy/knee 两列）"，
+  要求全部 16 个关节都变化，是过强的条件。
+
+**3. 修正**（`M20Interface::ResetPositionOffset`）
+
+1. 有限等待：`M20_JOINT_UPDATE_TIMEOUT_S`（默认 **5 s**，`=0` 完全跳过），绝不卡住；
+2. 每秒打印进度：`等待关节反馈变化 n/16（未变化: ...）`；
+3. 收到 0 条 → 明确报"完全没有收到 /JOINTS_DATA"并给出三条排查建议；
+4. `hipy/knee` 没有全部变化 → **跳过** ±360° 偏移校正（用未更新的数据修正会毁掉坐标系）；
+5. 结束时打印一次 **16 个实测关节角**（真机最需要的诊断信息）。
+
+**4. 结果（验收）**：在容器里用真机接口（`-DSIM2SIM=OFF`）复现"没有关节反馈"：
+
+```
+[M20Interface] 等待关节反馈变化 0/16（未变化: 0 1 ... 15），已等 1.0s / 3s
+[M20Interface] 等待关节反馈变化 0/16（...），已等 3.0s / 3s
+[M20Interface] ⚠️ 完全没有收到 /JOINTS_DATA（收到 0 条）。
+    请确认：① 机器人已进入 SDK 模式（手柄 + 授权码）；
+              ② ROS_DOMAIN_ID 与机器人服务一致；
+              ③ `ros2 topic hz /JOINTS_DATA` 有数据。
+[M20Interface] ⚠️ hipy/knee 没有全部发生变化 ⇒ **跳过**多圈 ±360° 偏移自检（保持默认 offset 表）。
+[M20Interface] joint pos (rad) = 0.000 ...
+```
+
+不再无限等待；`SIM2SIM` 回归（`--mode hold`）仍 PASS。
+
+---
+
+### DEF-025 `2026-09-22` CMake 会缓存 `SIM2SIM`：编过仿真版之后再编真机版必须显式 `-DSIM2SIM=OFF`
+
+| 项 | 内容 |
+|---|---|
+| 日期 | `2026-09-22` |
+| 类型 | 缺陷（构建陷阱） |
+| 状态 | `已修`（文档 + 预检查） |
+| 影响面 | `sim2real` |
+| 关联 | `CMakeLists.txt`；`scripts/preflight_real_deploy.py` |
+
+**1. 现象**：先按仿真方式编过 `-DSIM2SIM=ON`，之后只写
+`--cmake-args -DBUILD_PLATFORM=x86`（或 `=arm`）再编 —— **拿到的仍是仿真接口**
+（`M20SimInterface`，恒等标定），真机上关节坐标系就错了；而且日志里同样会打
+`M20 is using Ecan Hardware Interface`，不容易看出来。
+
+**2. 根因**：`SIM2SIM` 是 CMake cache 变量，`colcon build` 不会因为你这次没传就把它
+清掉。本次就是这个坑：第一次验证 DEF-024 时用了没带 `-DSIM2SIM` 的编译，实际跑的还是
+仿真接口（所以看不到新日志）。
+
+**3. 修正**：真机构建必须显式写 `-DSIM2SIM=OFF`；`scripts/preflight_real_deploy.py`
+的 `[2] 编译产物` 会读 `build/m20_sdk_deploy/CMakeFiles/rl_deploy.dir/flags.make`，
+发现 `-DSIM2SIM` 就报**阻断项**。
+
+**4. 结果**：预检查在当前容器（SIM2SIM 构建）正确报
+`[必须] 编译选项 SIM2SIM = True ...
+真机要**不带** -DSIM2SIM 的产物`。
+
+---
+
+### DEF-023 `2026-09-20` 臂没装时，策略看到的是"臂不在默认姿态"（观测与 ee_goal 自相矛盾）

| 项 | 内容 |
|---|---|
| 日期 | `2026-09-20` |
| 类型 | 缺陷（无臂部署口径不清） |
| 状态 | `已修`（并补了明确的无臂方案） |
| 影响面 | 二者 |
| 关联 | `interface/robot/hardware/piper_arm_interface.hpp`、`state_machine/quadruped_wheel/rl_control_state.hpp`；新增 `interface/robot/simulation/arm_stub_node.py` |
+
+**1. 现象**
+* 这条策略的观测含 **24 个关节**（12 腿 + 4 轮 + 6 臂 + 2 夹爪）与 7 维 `ee_goal`，
+  所以"臂没装"不是"少几个输入"，而是"有两个输入没有来源"。
+* `PiperArmInterface` 原来把 8 个臂关节的反馈**初始化为 0**，而训练默认姿态是
+  `arm2 = +0.5 / arm3 = -0.5`：于是没有臂数据时，策略被告知
+  "arm2 差了 -0.5 rad、arm3 差了 +0.5 rad"（`joint_pos` 是不乘 scale 的原始量），
+  而 `ee_goal` 又是"默认姿态"的目标 —— **观测与目标自相矛盾**。
+
+**2. 根因**：没有定义"无臂部署"的口径。接口的"未收到数据"用 0 兜底，
+与 `ee_goal` 的默认值（默认姿态）不一致；也没有任何提示告诉操作者状态是假的。
+
+**3. 修正**（三件事，缺一不可）
+1. `PiperArmInterface` 的初值改成**训练默认姿态**（0, 0.5, -0.5, 0, 0, 0, 0, 0），
+   并加 `HasData()` 记录有没有真的收到过 `/ARM_JOINTS_DATA`；
+2. `RLControlState::OnEnter` 在 `HasData() == false` 时打印明确警告；
+3. 新增 **`arm_stub_node.py`（假臂节点）**：无臂部署时以 200 Hz 发布
+   `/ARM_JOINTS_DATA`（默认姿态、零速度/力矩、`status_word=1`），以 50 Hz 发布
+   `/ARM_TELEOP_STATE`（root 系默认位姿 `(0.3492, 0, 0.4327)` + 四元数），
+   并订阅丢弃 `/ARM_JOINTS_CMD`。可用 `M20_ARM_STUB_POSE` / `M20_ARM_STUB_EE` 改姿态。
+   仿真侧新增 `M20_SIM_NO_ARM_DATA=1` 用来模拟"臂没装"（仿真照常跑，只是不发臂反馈）。
+
+**4. 结果（验收）**：新增 L3 用例 `tests/sim2sim_smoke.py --mode no_arm`
+（仿真不发臂数据 + 假臂节点），断言**第一拍观测里的 8 个臂项 = 0**（即"臂在默认姿态"）
+且 `ee_goal` = 默认位姿：
+
+| 场景 | 第一拍 obs 臂项 | ee_goal | 结果 |
+|---|---|---|---|
+| 修复前（无臂、无假节点） | arm2/arm3 ≈ ∓0.5 | 默认位姿 | 观测与目标矛盾（未测，代码推出） |
+| 修复后 + 假臂节点 | `[0,0,0,0,0,0,0,0]` | `(0.3492,0,0.4327,...)` | PASS（`--repeat 6` 失败率 17%，仅入口瞬态） |
+| 修复后、不起假节点 | `[0,0,0,0,0,0,0,0]` + 启动警告 | 默认位姿 | PASS（接口初值兜底） |
+
+> ⚠️ 注意：这是"**假装臂在默认姿态**"，不是"把臂从观测里拿掉"。真要拿掉，
+> 必须改训练侧观测口径并重训（部署侧单方面改观测 = 换了策略，见
+> `src/M20_sdk_deploy/policy/README.md` 的硬性约束）。
+
+---
+
+### DEF-022 `2026-09-20` 键盘增量路径没有目标限速：按一下就把目标甩到工作空间边界

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
