# M20 + Piper 仓库：后续 Codex 会话必读

本文件只记录相对稳定的项目概况、开发约束和注意事项。当前状态、待办、缺陷和修复进展
以 `docs/STATUS_zh.md` 与 `docs/DEFECTS_zh.md` 为准，不要复制到本文件。

## 1. 项目概况

- 仓库：`m20_piper_sdk_deploy`
- 主要目标：为 M20 轮足机器人 + AgileX Piper 机械臂提供 ROS 2 部署、仿真和真机控制能力。
- 主要范围：MuJoCo sim2sim、RL 策略部署、状态机、腿部/轮子接口、Piper IK、
  键盘遥操作、VR 遥操作和 sim2real 工具链。
- 当前开发重点在 `src/M20_sdk_deploy`。
- 除非用户明确要求，不要修改 Lite3 相关包：
  `src/Lite3_sdk_deploy`、`src/lite3_sdk_service`、`src/lite3_transfer`。
- 训练仓库位于 `/home/eureka/code/loco-manip-unified-rl-agent`。
  该仓库用于查询训练配置、导出格式和问题背景，不代表当前环境可以直接运行训练。

## 2. 代码结构

```text
src/M20_sdk_deploy/
  interface/                 机器人、DDS、MuJoCo、机械臂和 VR 接口
  state_machine/             idle、站立、RL、趴下和阻尼状态机
  run_policy/                策略加载、观测组装和动作映射
  policy/                    一次训练对应的部署产物
  scripts/                   编译辅助、同步、预检、录制和分析工具
  M20_Piper_description/     MJCF、URDF 和 mesh
  third_party/               XLeVR、Eigen、ONNX Runtime、手柄等
```

一个策略目录通常包含：

- `policy.onnx`
- `policy.pt`
- `policy_layout.json`

`policy.onnx` 和 `policy_layout.json` 必须成对存在。

## 3. 开发前阅读

开始任务前按需要阅读：

| 文档 | 内容 |
|---|---|
| `docs/STATUS_zh.md` | 当前状态、验收结果和唯一待办清单 |
| `docs/CONTRACT_zh.md` | 策略接口、关节顺序、坐标系、增益和限幅 |
| `docs/DEPLOY_zh.md` | 编译、同步、仿真/真机部署、VR 和排障 |
| `docs/DEVELOPMENT_zh.md` | 分支、测试、合并门槛和代码质量约定 |
| `docs/DEFECTS_zh.md` | 历史缺陷的根因和实测证据 |

总索引是 `docs/README_zh.md`。

资料职责：

- 动态状态和任务只写 `docs/STATUS_zh.md`。
- 新缺陷和实测证据追加到 `docs/DEFECTS_zh.md`。
- 接口或参数变化更新 `docs/CONTRACT_zh.md`。
- 部署步骤变化更新 `docs/DEPLOY_zh.md`。
- 测试或分支规则变化更新 `docs/DEVELOPMENT_zh.md`。

不要重新创建已合并的旧文档，包括 `DONE`、`TODO`、`WORKFLOW`、日期验收报告、
`NEXT_SESSION_PROMPT`、旧检查单、旧接口契约和旧离线资源文档。

`docs/THERMAL_zh.md` 已明确删除，不要重新创建。
温度记录图保留在 `docs/img/overheat_temps.png`，历史结论查看
`docs/DEFECTS_zh.md` 的 DEF-059/DEF-060。

## 4. 运行环境

| 项 | 值 |
|---|---|
| 宿主机仓库 | `/home/eureka/code/m20_piper_sdk_deploy` |
| 容器内仓库 | `/root/m20_piper_ws` |
| 仿真容器 | `m20_piper_ros` |
| 仿真 ROS 2 | Humble |
| 真机 ROS 2 | Foxy |
| 仿真 `ROS_DOMAIN_ID` | `1` |

检查容器是否运行：

```bash
docker ps --format '{{.Names}}\t{{.Status}}'
```

如果容器未运行：

```bash
docker start m20_piper_ros
```

## 5. 快捷命令格式

仿真快捷命令必须直接给出宿主机可复制的完整形式：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=1 && exec ros2 topic hz /JOINTS_DATA'
```

必须遵守：

- 不要定义或建议用户定义 `m20()`、shell alias 或其他快捷包装函数。
- 不要使用 `docker exec` 或 `docker exec -d` 的简写。
- 统一使用 `docker exec -it m20_piper_ros bash -lc '……'`。
- 长运行节点在容器命令前加 `exec`，确保 Ctrl+C 能结束真实进程。
- 命令中的仓库目录从 `/root/m20_piper_ws` 开始。
- `M20_POLICY_DIR` 按进程 cwd 解析，配置时使用绝对路径。

## 6. 编译约定

### sim2sim

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && colcon build --packages-up-to m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON'
```

### 真机

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF'
```

注意：

- CMake 会缓存 `SIM2SIM`，切到真机时必须显式设置 `-DSIM2SIM=OFF`。
- 修改 C++ 后需要重新编译；只改 Python 节点通常不需要重编。
- 真机部署前使用 `scripts/preflight_real_deploy.py` 检查构建类型、策略和话题。

## 7. 测试原则

`main` 的最低要求：

1. 能编译。
2. L1 策略接口检查通过。
3. L3 sim2sim 冒烟通过，实时因子接近 1.0。

常用入口：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 src/M20_sdk_deploy/scripts/check_policy_interface.py'
```

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 src/M20_sdk_deploy/scripts/check_mjcf_contract.py'
```

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 tests/sim2sim_smoke.py --mode rl --duration 20'
```

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && exec bash tests/run_all.sh'
```

测试要求：

- 先看实际测试输出，再下结论。
- 仿真通过不等于真机通过。
- 文档里的历史数字不能冒充本次实测。
- 仿真结果必须结合 `[SIM-RT]` 实时因子判断。
- 多发布者和残留进程会制造伪故障；调试控制链前先检查进程和话题发布者。

## 8. 技术不变量

### 8.1 接口布局

策略输入输出和关节顺序以 `policy_layout.json`、ONNX 形状及
`docs/CONTRACT_zh.md` 为准。不要按位置猜测关节顺序。

仓库中可能同时存在以下顺序：

- 动作序
- 训练 articulation 原生序
- MJCF 顺序

必须按关节名映射。

### 8.2 History 与观测

- History 和 policy observation 对角速度、关节速度的 scale 处理不同。
- History 的 24 维关节位置包含轮子，不把轮子列置零。
- Policy observation 中轮子位置列需要置零。
- 复位或进入 RL 时，history 使用当前帧填满。

具体切片以 `docs/CONTRACT_zh.md` 为准。

### 8.3 动作回灌

训练中的动作观测就是控制使用的同一个动作。部署侧如增加动作限幅，
必须把限幅后的动作喂回策略观测。

### 8.4 坐标与末端位姿

- `ee_goal` 使用 root 系位置和四元数。
- 四元数必须保持训练链式 FK 的符号分支，不能任意换成 `w >= 0` 的等价表示。
- 高度使用相对足端定义，公式和默认值见 `docs/CONTRACT_zh.md`。

### 8.5 MuJoCo

- `armature` 必须写在 `<joint>`，写在 `<motor>` 会被 MuJoCo 静默忽略。
- `MUJOCO_GL=osmesa` 在当前环境可能引发 OpenGL 绑定错误。
- 无头仿真使用默认 `glfw` 和 `M20_USE_VIEWER=0`。

### 8.6 策略产物

- 不要使用 `play.py` 导出的 actor-only 策略。
- 使用训练导出的部署目录，例如 `exported_deploy/policy.onnx`。
- 更换策略时保留对应 layout，避免维度、关节序和输入名不匹配。

## 9. 真机安全

- 未经用户明确要求，不要直接执行真机控制命令。
- 真机调试必须有人值守，并准备硬件急停。
- 按“只上腿 → 机械臂 → VR”的顺序分阶段验证。
- 进入 RL 前确认周围安全，并保留软件 `r` 和硬件急停接管手段。
- 发现异常倾斜、持续轮速、过温或 `[TAKEOVER!]` 时立即进入 joint damping。
- 常规退出顺序：`r` 关节阻尼 → `x` 趴下 → VR → 机械臂节点 →
  `rl_deploy` → 退出 SDK 模式。
- 真机参数和历史结论必须记录到对应文档，不能只留在终端输出中。

## 10. Git 分支与提交

开始修改前先确认工作区：

```bash
git status --short --branch
git log -5 --oneline
```

默认按一个功能点、缺陷或独立文档整理创建一条新分支：

```bash
git switch -c fix/<topic>
git switch -c feat/<topic>
git switch -c docs/<topic>
```

分支命名示例：

- `fix/arm-ik-limit`
- `feat/no-arm-layout`
- `docs/consolidate-project-docs`
- `wip/<topic>` 只用于临时实验，不合并

工作要求：

- AI 完成一个完整功能点、缺陷修复或独立文档整理，并完成相应测试后，**先提醒用户
  验收确认，不要自动 commit**。
- 只有用户明确回复“可以提交”或同义确认后，才能执行 commit。
- 每个已确认的完整功能点、缺陷修复或独立文档整理提交一次 commit。
- 不要把无关修改混进同一个 commit；一个大任务可以拆成多个可独立验证的 commit。
- 已有用户未提交改动时，先判断它们是否属于当前任务。无法安全分开时先询问用户，
  不要覆盖、回滚或擅自打包进自己的提交。
- 功能代码和对应测试、文档尽量放在同一个逻辑提交中。
- 修改接口、关节顺序、坐标系、增益、话题或观测维度时，提交必须同时包含合同文档更新。

提交前检查：

```bash
git diff --check
git diff
git status --short
```

按模块运行最低测试，例如：

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 src/M20_sdk_deploy/scripts/check_policy_interface.py'
```

```bash
docker exec -it m20_piper_ros bash -lc 'cd /root/m20_piper_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 tests/sim2sim_smoke.py --mode rl --duration 20'
```

只暂存当前任务涉及的文件：

```bash
git add <file1> <file2> <directory>
git diff --cached
```

不要习惯性使用 `git add -A`，除非已经确认工作区全部改动都属于当前任务。

提交信息使用 conventional commit 前缀，主题写清楚改了什么：

```text
fix(rl): correct action observation feedback
feat(sim): add thermal fault injection
docs: consolidate project documentation
```

提交正文建议写明：

- 为什么需要这次修改。
- 改前和改后的关键行为。
- 实际执行的测试命令和结果。
- 相关缺陷编号，例如 `DEF-059`。

测试通过后再合并回目标分支：

```bash
git switch <目标分支>
git merge --no-ff <已完成分支>
```

合并同样需要用户明确确认后才能执行。

合并后重新检查：

```bash
git status --short --branch
git log --graph --oneline -8
```

约束：

- 不使用 `git reset --hard`、`git checkout -- <file>` 等会丢失用户改动的命令。
- 不修改或 amend 已经推送的提交，除非用户明确要求。
- 未经用户明确要求，不执行 `git push`。
- 未得到用户明确的提交确认，不执行 `git commit`。

## 11. 修改规范

- 不覆盖、回滚或删除用户的无关改动。
- 使用 `rg` 或 `rg --files` 查找代码和文件。
- 修改文档后检查本地 Markdown 链接。
- 修改接口、坐标、增益、话题或关节顺序时必须同步更新合同文档。
- 新增环境变量必须有默认值、用途和所属模块说明。
- 临时调试代码不进入默认控制路径。
- 提交前检查 `git diff --check` 和相关测试。
