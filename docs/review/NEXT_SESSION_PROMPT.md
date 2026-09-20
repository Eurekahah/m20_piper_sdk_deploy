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

【当前状态】main = 40744b5（领先 origin/main 21 个提交，工作区干净）。
  ⚠️ main 上跑的是**旧 checkpoint**的接口（policy_obs 86 / history 770 / action 23，
     M20_adjusted 资产）⇒ 这是"部署效果欠佳"的第一嫌疑。
  最新训练 run：训练仓库 logs/rsl_rl/history_adaptation/2026-09-20_00-50-31，
     checkpoint model_19999.pt，部署态产物在 <run>/exported_deploy/
     {policy.pt, policy.onnx, policy_layout.json}，接口是 **83 / 700 / 16**，
     资产 M20_Piper_own。**还没接进部署侧**。
  上一轮针对旧 checkpoint 的调试工作已另存到 `wip/old-ckpt-arm-coupling-debug @ 8de683c`
     （一批 M20_* 调试开关、last_action 三种模式、body-frame ee_goal）——**不要合并**。
  本轮核对发现 5 条未修缺陷（DEF-005/007/008/009/010）+
     2 条新发现（DEF-011 armature 无效、DEF-012 初始保持位姿用错机型）。

【下个 session 的第一件事】按 TODO_zh.md P0 从 P0-1 开始：
  把 M20PiperPolicyRunner 改成**布局驱动**（读 policy_layout.json + ONNX 形状），
  按 83/700/16 重写观测组装（joint_pos/joint_vel 都是 **24 维原生序**、
  joint_pos 只把 4 个轮子列置零、history 每步 70、动作 16 维、**没有 ee_ik 槽位**），
  然后按 P0-2 用 sim2sim A/B 判定 24 维原生序到底是"交错序"还是"分组序"。

【测试分层（详见 WORKFLOW_zh.md）】
  L0 编译：容器内 `source /opt/ros/humble/setup.bash && colcon build --packages-up-to
           m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON`（~20 s，已验证通过）
  L1 离线：`python3 src/M20_sdk_deploy/scripts/check_policy_interface.py`（待写）
  L3' 参数对照：`python3 src/M20_sdk_deploy/scripts/check_mjcf_contract.py`
           （已可用，当前 8 PASS / 2 FAIL / 3 UNKNOWN；两个 FAIL 是 DEF-011/DEF-012 相关）
  L3 端到端：`rl_deploy` + `mujoco_simulation_ros2.py` 无头跑 N 秒，按契约文档的
           数值判据打分（高度/倾角曲线；tests/ 一键脚本待写 = TODO P3-1/P3-2）

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
