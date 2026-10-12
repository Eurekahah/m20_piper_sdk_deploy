# 仿真新代码 → 部署代码需要跟着改的地方（2026-10-09）

**文档职责**：把训练仓库（`loco-manip-unified-rl-agent`）
`docs/deploy_code_delta_2026-10-09_zh.md`（基线 `2d49f47` → `f663ee1`，比基线多 37 个
source/scripts 提交）逐条落到**本仓库**：哪些改了、改在哪、哪些本来就已经满足、
哪些不适用。管"这次同步做了什么、结论是什么"；来龙去脉看
`docs/review/DEFECT_LOG_zh.md` 的 DEF-029~DEF-032。

**一句话结论**：**策略接口一个字都没变**（obs 83 / history 10×70 / action 16 /
增益与缩放全同）⇒ 换 `policy.onnx` 不用改部署代码；要改的是**执行器参数与命令层
语义**。下面 6 条里，本仓库改了 3 条、另 3 条本来就已经满足或不适用。

---

## 0. 逐条对照

| # | 训练侧改动（sim 新代码） | 本仓库要不要改 | 落点 |
|---|---|---|---|
| ① | 臂 PD `damping 20 → 8`（DEF-064） | **要改** | `arm_controller.py`、`mujoco_simulation_ros2.py`、`piper_arm_interface.hpp`、`check_mjcf_contract.py` |
| ② | 臂关节速度上限 = **3 rad/s**（DEF-065） | **要改（真机侧）** | `arm_controller.py` 记 `ARM_VEL_LIMIT`；真机由 agx_arm_ctrl/驱动器设 3 rad/s，别放宽到 5 |
| ③ | IK 输出必须夹到关节限位（DEF-054） | **已满足** | `arm_controller.ik_step()` 已 `np.clip(q+dq, ARM_JOINT_LIMITS)` |
| ④ | 夹爪 PD `4000/200 → 286/5`；开合 ±0.035（DEF-058 / DEF-065） | **要改** | 同 ①（开合 ±0.035 本来就是对的） |
| ⑤ | EE 命令复位锚定到当前实测位姿（known_issues ⑫ / DEF-034） | **已满足** | `ee_goal` 由 `arm_controller` / `arm_real_adapter` 持续发布真实值，`ee_reset` 取当前 FK |
| ⑥ | EE 目标碰撞盒改成机体局部系（known_issues ⑭ / DEF-066） | **不适用** | 部署侧没有"EE 目标不得穿身体"的采样检查 |
| — | 默认部署模型换成多地形 `cloud_slowvx20k` | **要改** | `policy/m20_piper_rough_20261002/` + `rl_control_state.hpp` 等默认目录 |

---

## 1. 臂 / 夹爪增益与速度上限（①②④）

权威值（训练侧 `deeprobotics.py`，2026-10-08）：

| 执行器 | 力矩上限 | 速度上限 | stiffness | damping |
|---|---|---|---|---|
| `piper_arm`（joint1~6） | 100 N·m | **3 rad/s**（旧 USD 是 5） | 300 | **8**（旧 20） |
| `piper_gripper`（gripper1/2） | 10 N·m | 1.0 rad/s | **286**（旧 4000） | **5**（旧 200） |

为什么（训练侧实测，逐条见 DEF-064 / DEF-058）：

* 臂 300/20：`|qd|` 到 5 rad/s 时 `20×5 = 100 N·m` 吃满限幅 ⇒ 长期饱和、原地抖；
  300/8 ⇒ 22.5 N·m、饱和 0%、跟踪还更好（4.14 → 2.19 cm）。
* 臂速度上限：cfg 里的 `velocity_limit=3.0` 在 `DelayedPDActuator` 里**只裁剪力矩、
  不限速**，真正生效的是 USD 的 5 ⇒ 启动事件 `align_joint_velocity_limits` 把 PhysX
  DOF 上限改成 3，实测 `|qd|` p99 5.0 → 3.00。
* 夹爪 4000/200：位置环实际失效（饱和 88~100%、稳态误差最大 0.0314 rad）；
  286/5 = 按"满行程 0.035 rad 误差刚好给到 10 N·m 限幅"标定 ⇒ 饱和 0%、
  稳态误差 ≤0.0005。

本仓库把三处写死的常量 + 一处断言同步到 300/8、286/5：
`arm_controller.py::ARM_KP/KD`、`mujoco_simulation_ros2.py::ARM_DEFAULT_KP/KD`、
`piper_arm_interface.hpp::ResetToDefault()`、`check_mjcf_contract.py::EXPECT_ACTUATOR`。

> ⚠️ **臂 3 rad/s 是本次新增的约束**：仿真里已对齐，真机**别按 5 放开**
> （joint6 本来就是 3）。真机这条由 `agx_arm_ctrl` / 驱动器配置，部署脚本只记下
> `ARM_VEL_LIMIT = 3.0` 作为约定。

## 2. 真机限幅对齐表（部署控制器按这个设）

| 部位 | 力矩上限 | 速度上限 | 备注 |
|---|---|---|---|
| 腿（hipx/hipy/knee） | 76.4 N·m | 22.4 rad/s | 本次未改 |
| 轮 | 21.6 N·m | 79.3 rad/s | 速度伺服：`tau = 0.6*(w_des - w)` |
| 臂 joint1~6 | 100 N·m | **3 rad/s**（旧 USD 是 5） | PD 300/**8**；IK 输出夹关节限位 |
| 夹爪 | 10 N·m | 1.0 rad/s | PD **286/5**；开合目标 ±0.035 |

## 3. 换策略（多地形 `cloud_slowvx20k`）

* 新目录：`policy/m20_piper_rough_20261002/`（run `2026-10-02_00-44-42_cloud_slowvx20k`，
  checkpoint `model_19999.pt`）——多地形模型。
* 平地模型：`policy/m20_piper_flat_20260930/`（run `2026-09-30_19-36-00_cloud_cap12_20k`，
  即训练侧 §5.4 指定的平地模型）；更早的平地 `policy/m20_piper_history_20260920/`
  （run `2026-09-20_00-50-31`）也保留着，三者接口逐字段相同。
* 两者接口**逐字段相同**，`policy_layout.json` 的 `joint_order_native` /
  `joint_order_action` 也逐项相同；只是权重不同。
* 默认目录：**平地用 `m20_piper_flat_20260930`**（`rl_control_state.hpp` +
  `check_policy_interface.py` / `preflight_real_deploy.py` 的默认都指向它）；
  多地形改 `M20_POLICY_DIR=/root/m20_piper_ws/src/M20_sdk_deploy/policy/m20_piper_rough_20261002`
  即可（该变量按进程 cwd 解析，别写 `policy/...`）。

## 4. 不用改的部分（只是训练侧）

扰动加强 + 课程、静止伫立惩罚、多地形任务与 SlowVx 课程、`max_noise_std`、
`joint_mirror_signed`、报告口径修正——都只影响训练，部署侧不需要复现。
工具侧 `export_deploy_policy.py` / `probe_deploy_layout.py` 自基线未改
⇒ 导出流程与 `policy_layout.json` 格式同上次上机时完全一样（本仓库新目录也已补齐
部署侧两个关节序字段）。

## 5. 上机前请记住的已知短板（引用训练侧）

1. **上坡 + 低/零速最弱**（训练侧 DEF-061）：上坡 `(0,0,0)` 的速度误差是平地的 4.4x。
   真机上"斜坡上原地站住"要格外小心。
2. 多地形模型（`cloud_slowvx20k`）的能力大致到 terrain level 4.7/9，更陡是分布外。
3. 抓取成功率没在高层训练过的策略上复验过（现成高层 checkpoint 是 2026-05 的）。

## 6. 验收清单

1. `python3 src/M20_sdk_deploy/scripts/check_policy_interface.py`（默认已指向新目录）
   —— 接口逐项对上（本次用 ONNX Runtime C++ 编了个最小校验器，在容器未起时先验了
   形状/名字/标称输出；容器起来后请补跑完整 L1）。
2. `python3 src/M20_sdk_deploy/scripts/check_mjcf_contract.py` —— 增益断言改成 300/8、
   286/5 后应 PASS。
3. MuJoCo sim2sim 按 [`sim2sim_layout_contract_zh.md`](sim2sim_layout_contract_zh.md)
   的顺序（裸模型站立 → 零动作 → 接 ONNX 零命令 → 开 IK → EE 小步进 → 加 vx）。
4. 关节/夹爪限幅按 §2 表设好，**特别是臂 3 rad/s**。
5. 复位后第一拍的 EE 目标 = 当前实测位姿（§⑤，部署侧已满足，核对即可）。

---

## 7. 容器实测记录（2026-10-09，容器 `m20_piper_ros`）

在容器里 `colcon build`（x86 / SIM2SIM）后跑完 `tests/run_all.sh`：

| 项 | 结果 |
|---|---|
| L0 编译 | ✅ `2 packages finished [23.0s]`，无 error |
| L1 策略接口 | ✅ **PASS**（新目录；`joint_order_native` 一致；标称 `\|a\|max` 0.958；ONNX↔TorchScript 相对误差 **5.96e-07**） |
| L3' `check_mjcf_contract` | ✅ **10 PASS / 0 FAIL / 3 UNKNOWN**（新增益断言 300/8、286/5 通过） |
| L2 `arm_real_bridge_check` | ✅ PASS（上行 1.49e-10、ee_goal 位置差 0.04 mm、下行 joint2 0.8 / gripper 0.04） |
| L3 `hold` | ✅ height 0.498 m、tilt 1.4° |
| L3 `wheel_step` | ✅ 四轮稳态 **+5.00 rad/s**（命令 +5） |
| L3 `no_arm` / `arm_only` / `arm_mixed` | ✅ PASS（arm_only 残差 0.10 mm、末端 +0.20 m；arm_mixed 观测臂项变化 1.01 rad） |
| L3 `rl` / `walk` / `arm_move` | ✅ 重复 6 次失败率 0~17%（< 1/3），walk vx +0.813 m/s |
| L3 `push` | ✅ 800 N 侧推 → `[TAKEOVER!] leg fold 1.283` → `joint_damping` |
| L3 `arm` | ⚠️ **偶发**：单跑 25 s 时约 25~33% 概率触发倾角接管（**不是本次改动引入**，见下） |

**`arm` 档偶发摔倒的定性**（本次重点排查）：

* 现象：`arm` 档（`z` → `c` 进 RL，臂保持默认位姿）有时在进 RL 后几秒底盘发散，
  `tilt` 越过 0.8 rad → 安全接管。摔倒时刻稳定落在 t≈13~14.5 s。
* **不是臂/增益问题**：失败现场遥测里臂关节力矩全程 **≤5 N·m**（只在倒地瞬间到 21.9，
  限幅 100）；倒下是**轮子/底盘发散**——零速度命令下 `wcmd` 从 t≈9 s 起涨到 47.9 rad/s、
  轮速 47.7 rad/s，`tilt` 1° → 55°。
* **不是换策略引入**：`arm` 档各跑 12 次，新多地形策略 **9/12 通过（失败 25%）**、
  旧平地策略 **8/12 通过（失败 33%）**——两者同量级，属训练侧 DEF-018 的
  "进 RL 入口瞬态 / 策略侧边缘稳定性"（部署侧 4 组对照实验早已排除，见
  `docs/review/DEFECT_LOG_zh.md` DEF-018）。
* **测试口径不一致（待定）**：`tests/run_all.sh` 把 `arm` 当"0 失败"档单跑，
  但 `arm` 同样会进 RL、同样吃 DEF-018；真正用失败率判的只有
  `rl / walk / arm_move / no_arm / arm_mixed`（`tests/sim2sim_smoke.py` 的容忍列表是
  `rl / walk / arm_move / no_arm`）。所以 `run_all.sh` 会以 ~25~33% 的概率因这一档整体报红。
  建议（未改，等你确认）：把 `arm` 也加进 `run_all.sh` 的 `--repeat` 分支与
  `sim2sim_smoke.py` 的容忍列表，用同样的"失败率 ≤ 1/3"判据。
