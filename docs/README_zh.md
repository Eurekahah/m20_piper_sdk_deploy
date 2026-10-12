# 文档索引

本目录分两类文档：

## 1. `docs/review/` —— 长期维护的状态文档（按训练仓库的规范）

| 文件 | 管什么 |
|---|---|
| [`review/WORKFLOW_zh.md`](review/WORKFLOW_zh.md) | 分支模型、提交规范、模块边界、测试分层（L0/L1/L2/L3/L4）、合并门槛、debug 开关登记表 |
| [`review/TODO_zh.md`](review/TODO_zh.md) | **唯一**的未完成清单（P0→P3） |
| [`review/DONE_zh.md`](review/DONE_zh.md) | 已完成（带日期 + commit + 实测数字） |
| [`review/DEFECT_LOG_zh.md`](review/DEFECT_LOG_zh.md) | 每个缺陷/特性的现象→根因→修正→结果（`DEF-0xx`） |
| [`review/NEXT_SESSION_PROMPT.md`](review/NEXT_SESSION_PROMPT.md) | 下个 session 的开工 prompt（整段可复制） |
| [`review/ACCEPTANCE_2026-10-09_zh.md`](review/ACCEPTANCE_2026-10-09_zh.md) | **验收报告（2026-10-09 夜）**：平地 vs 多地形 —— 22 命令组合 × 3 遍、10 档 L3、单命令阶跃；含"侧移/转向方向不对称"等发现 |
| [`review/CODE_REVIEW_2026-10-09_zh.md`](review/CODE_REVIEW_2026-10-09_zh.md) | **全代码走查**：缺漏（安全层未接线、限位夹取失效、单位混淆）与不优雅（常量抄 8 遍、死代码 ~700 行、`-w` 关警告）；每条带 `文件:行` |
| [`review/templates/`](review/templates/) | 文档模板与缺陷条目模板 |

规范要点：每次改动都要更新 `TODO`/`DONE` 的"更新记录"、给新缺陷加一条 `DEF-0xx`；
**`main` 必须始终能编译 + 过 sim2sim 冒烟**。

## 2. 专题文档

| 文件 | 管什么 |
|---|---|
| [`sim2sim_layout_contract_zh.md`](sim2sim_layout_contract_zh.md) | **接口契约**：观测/动作布局、关节顺序、坐标系、增益、命令语义、MuJoCo 参数，以及每条说法的核对状态（✅/⚠️/❓/❌） |
| [`../src/M20_sdk_deploy/COMMANDS_SIM2SIM_zh.md`](../src/M20_sdk_deploy/COMMANDS_SIM2SIM_zh.md) | **sim2sim 命令速查（宿主机一行式）**：命令都带 `docker exec`，直接粘不用进容器；全仿真 / 只调腿 / 只调臂 / 腿仿真+臂独立传输 / VR / 一键验收 / 换策略 + 键盘速查 + 实时因子与残留进程排障 |
| [`../src/M20_sdk_deploy/COMMANDS_SIM2REAL_zh.md`](../src/M20_sdk_deploy/COMMANDS_SIM2REAL_zh.md) | **sim2real 命令速查**：只部署 M20 / 只调臂 / 只感知真臂 / IK+键盘合并 / DDS 不通的两种修法 / 排障 |
| [`sim2real_checklist_zh.md`](sim2real_checklist_zh.md) | **真机上线检查单**：仿真↔真机接口差异表、分阶段步骤（只上腿 → 接臂 → 测量回填）、回滚顺序、"没做过"清单 |
| [`DEPLOY_REAL_VR_zh.md`](DEPLOY_REAL_VR_zh.md) | **真机 + VR 完整部署流程**（执行版）：编译 arm 版 → scp → SDK 模式 → 只上腿 → 接真臂 → 接 VR（头显/按键分工/复位语义）→ 手感与工作空间调参 → 安全回滚 → 排障 |
| [`deploy_code_delta_2026-10-09_zh.md`](deploy_code_delta_2026-10-09_zh.md) | **仿真新代码 → 部署代码的同步说明**（2026-10-09）：臂/夹爪增益与速度上限、换到多地形策略，逐条对照"要改 / 已满足 / 不适用" |

## 3. 历史归档

| 文件 | 说明 |
|---|---|
| `M20_Piper_deploy_6commits_zh.md` | 最早 6 个提交（`9095957`~`2fcab7c`，2026-08-20~09-04）的工作总结。**已加历史归档横幅**：里面的策略文件与接口都已不在 `main` 上，现行状态看 `review/DONE_zh.md` |
