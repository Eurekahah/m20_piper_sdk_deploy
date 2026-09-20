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
| [`review/templates/`](review/templates/) | 文档模板与缺陷条目模板 |

规范要点：每次改动都要更新 `TODO`/`DONE` 的"更新记录"、给新缺陷加一条 `DEF-0xx`；
**`main` 必须始终能编译 + 过 sim2sim 冒烟**。

## 2. 专题文档

| 文件 | 管什么 |
|---|---|
| [`sim2sim_layout_contract_zh.md`](sim2sim_layout_contract_zh.md) | **接口契约**：观测/动作布局、关节顺序、坐标系、增益、命令语义、MuJoCo 参数，以及每条说法的核对状态（✅/⚠️/❓/❌） |

## 3. 历史归档

| 文件 | 说明 |
|---|---|
| `M20_Piper_deploy_6commits_zh.md` | 最早 6 个提交（`9095957`~`2fcab7c`，2026-08-20~09-04）的工作总结。**已过时**：`main` 目前领先 `origin/main` 21 个提交，现行状态看 `review/DONE_zh.md` |
