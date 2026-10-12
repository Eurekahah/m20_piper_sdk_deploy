# 下个 session 的开工 prompt（可直接整段复制）

> 说明：这是给"下一个 AI session"用的交接 prompt。写完新版本就把这一节整体替换掉。

```text
仓库：/home/eureka/code/m20_piper_sdk_deploy（M20 四轮腿 + AgileX Piper 机械臂的 SDK 部署仓库）
训练仓库：/home/eureka/code/loco-manip-unified-rl-agent（Isaac Lab 5.1 + rsl_rl，**只有代码不能跑**）
运行环境：docker 容器 m20_piper_ros（镜像 m20-piper-deploy:latest），
          宿主仓库挂在容器内 /root/m20_piper_ws；容器当前是 stopped，先 `docker start m20_piper_ros`

【范围】只做 M20 这一部分（`src/M20_sdk_deploy`）。目标是**先把 sim2sim 做好**，
        并且**方便转 sim2real**；Lite3 那几个包不要动。

【文档规范（必须遵守）】docs/review/ 只保留：
  WORKFLOW_zh.md          # 分支/提交/模块边界/测试分层/合并门槛/debug 开关登记表
  TODO_zh.md              # 唯一未完成清单（P0→P3）
  DONE_zh.md              # 已完成（带日期 + commit + 实测数字）
  DEFECT_LOG_zh.md        # 每个缺陷/特性：现象→根因→修正→结果（DEF-0xx）
  NEXT_SESSION_PROMPT.md  # 本文件
  templates/DOC_TEMPLATE_zh.md、templates/DEFECT_ENTRY_TEMPLATE_zh.md
  docs/sim2sim_layout_contract_zh.md  # 接口契约 + 逐条核对状态（改接口时必须同步更新）
  每次改动**必须**：更新 TODO/DONE 的"更新记录"（加日期）、给新缺陷/特性加一条 DEF-0xx、
  接口类改动同步更新契约文档。

【唯一不变量】main 任何时候都要能编译 + 过 L1（离线策略验收）+ 过 L3（sim2sim 端到端冒烟）。
  分支：feat/<topic> / fix/<topic> / docs/<topic> / wip/<topic>（wip 不合并）。
  合并用 `git merge --no-ff`。

【当前状态】工作区有 2026-10-09 的改动未提交（策略同步 + 换模型 + 走查文档）。
  ✅ sim2sim 已达标：`bash tests/run_all.sh` 全绿（L0 + L1 + L3' + L2 +
     **command_sweep（22 个命令阶段）** + 10 档 L3）。
  接口：83 / 700 / 16，布局驱动（读 policy_layout.json + ONNX 形状/名字断言），
     策略目录 policy/m20_piper_flat_20260930/（**平地，2026-10-09 起默认**；
     多地形用 policy/m20_piper_rough_20261002/，更早的平地
     policy/m20_piper_history_20260920/ 也在；换策略见 policy/README.md）。
  ✅ 2026-10-09 夜已按训练侧 delta 同步（臂 PD 300/8、夹爪 286/5、臂速度上限 3 rad/s），
     并把**平地 / 多地形两套模型**都用完整 harness 跑了一遍，详见
     `docs/review/ACCEPTANCE_2026-10-09_zh.md`。要点：
       * 命令组合扫描（22 阶段 × 3 遍）：两模型 **66 次命令暴露 0 摔倒**、实时因子 1.000；
       * 10 档 L3：两模型**全部通过**（rl 0/6；walk 17%/33%；arm_move 0%/33%，均在容差内）；
       * `vx` 跟踪极准（0.7→0.72、1.0→1.01、1.5→1.62/1.73）；
       * `--mode push` 默认推力由 800 N 改为 **1500 N**（800 N 已被平地模型扛住）；
       * **未解决**：`vy`/`wz` 存在**方向性不对称**（平地 `wz-` 只跟 5%、多地形 `vy+` 只跟 9%）,
         见 DEF-036 —— 这是目前最需要训练侧确认的一条。
  ⚠️ 跑之前先看仿真终端 `[SIM-RT] 实时因子`：**≈1.00 才算数**（电池供电会掉到 0.6~0.7，DEF-034）。
  ⚠️ `M20_POLICY_DIR` 按**进程 cwd** 解析，请用绝对路径（DEF-038）。
  📋 全代码走查已出（`docs/review/CODE_REVIEW_2026-10-09_zh.md`），待办落在 TODO 的
     **P4-1 ~ P4-9**（死亡代码、安全层未接线、常量抄 8 遍、`-w` 关警告…）。
  ⚠️ DEF-018（未修，**已定位为策略侧**）：rl / walk / arm_move 三档在"进 RL 后
     1~4 s"有 0~35% 概率发散。部署侧 5 组对照实验全部排除（软启动长度、
     轮子执行器语义、执行器延迟 0~25 ms、站立腿增益、轮子无扰交接），
     仿真实时因子 1.000。与训练侧 s3 阶段 0.09~0.15/20 s 的终止率同源
     ⇒ 交接训练侧（训练仓库 TODO P1-2）。

【TODO 现状】P0 全部清空；剩余只有三条**真机**项：
  P2-1 真机腿部标定链核对  P2-2 arm_real_adapter 实机联调  P2-3 时延/抖动测量。
  流程与判据见 docs/sim2real_checklist_zh.md（含"没做过"清单，别把没验过的当成验过）。

【下个 session 建议】
  1. 真机可用 ⇒ 按 docs/sim2real_checklist_zh.md 走阶段 A（只上腿）：
     第一件事是"标定核对"（摆成训练默认姿态读 /JOINTS_DATA，偏差应 < 0.05 rad）。
  2. 真机首跑务必注意 DEF-018：进 RL 的 1~2 s 别放手，先给小速度命令（vx 0.1~0.2）。
  3. 训练侧若改了策略，回来跑 `bash tests/run_all.sh`，并把 DEF-018 的失败率
     （`--repeat 12`）与历史对比（当前 rl 0~33%、walk 17~50%、arm_move 0~17%）。

【运行命令备忘】
  docker start m20_piper_ros && docker exec -it m20_piper_ros bash
  cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
  export ROS_DOMAIN_ID=1
  ros2 run m20_sdk_deploy rl_deploy                          # T1（z 站立 → c 进 RL）
  M20_USE_VIEWER=0 python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py   # T2
  python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py      # T3（臂 IK，可选）
  python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py     # T4（臂键盘，可选）
  ros2 service call /reset_sim std_srvs/srv/Empty

【踩坑备忘（累计）】
  * **关节顺序有三套**：动作序（12 腿 fl,fr,hl,hr + 4 轮）、articulation 原生序
    （观测 joint_pos/joint_vel 的 24 维）、MJCF 序（每腿 hipx/hipy/knee/wheel 连续 +
    臂 + 夹爪）。一律按关节名映射，不要按位置硬编（DEF-005）。
  * **history 里同名项与 policy_obs 不一样**：history 用**原始值**（ang_vel 不乘 0.25、
    joint_vel 不乘 0.05），且 joint_pos 24 维**含轮子不置零**；policy_obs 里乘了 scale
    且轮子列置零。
  * 训练里 `actions` 观测 = 控制用的**同一个值**；部署侧加了限幅就必须把限幅后的值喂回去
    （DEF-006）。
  * MuJoCo 的 `armature` 只认 `<joint armature=...>`，写在 `<motor>` 上是**合法但无效**的
    （DEF-011）。
  * 训练 terminate 阈值：倾角 > 0.8 rad、`root_z − mean(4 轮 z) + 0.09 < 0.30`；
    `body_pose.height` 用的就是这个"相对足端"的度量（默认姿态着地后 = 0.5266 m）。
  * `ee_goal` 是 **root 系**位姿（pos + quat **wxyz**），训练里 clip ±3；
    默认臂位姿下 `gripper_base` 在 root 系是 `(0.3492, 0, 0.4326)`。
  * 不要用 `play.py` 导出的 `<run>/exported/policy.pt`（actor-only），
    要用 `<run>/exported_deploy/policy.onnx`。
  * 训练仓库 `logs/` 每个 run 有 ~50 个 5 MB 的 model_*.pt，注意磁盘。
  * 容器里 `MUJOCO_GL=osmesa` 会因 OpenGL 绑定报错；用默认 `glfw` + `M20_USE_VIEWER=0`
    可以无头跑纯物理仿真（实测 OK）。
```
