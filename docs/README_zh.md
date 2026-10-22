# M20 + Piper 文档索引

文档按“做决策、部署、开发、查历史”组织，不再按日期拆分。

| 文档 | 用途 | 什么时候看 |
|---|---|---|
| [STATUS_zh.md](STATUS_zh.md) | 当前状态、验收摘要、唯一待办清单、推荐工作顺序 | 开始任何工作前 |
| [DEPLOY_zh.md](DEPLOY_zh.md) | sim2sim/sim2real 部署、离线资源、阶段 A/B/C、VR、回滚和排障 | 实际运行或上真机 |
| [CONTRACT_zh.md](CONTRACT_zh.md) | 策略接口、history、动作、关节顺序、执行器、仿真/真机差异 | 修改策略、接口、模型或控制参数 |
| [DEVELOPMENT_zh.md](DEVELOPMENT_zh.md) | 分支、测试层、合并门槛、调试约定、代码质量缺口 | 开发、提测、合并前 |
| [DEFECTS_zh.md](DEFECTS_zh.md) | 历史缺陷的现象、根因、修复、关键实测 | 追查“为什么这样改” |

## 推荐阅读路径

### 第一次接手项目

1. [STATUS_zh.md](STATUS_zh.md)
2. [CONTRACT_zh.md](CONTRACT_zh.md)
3. [DEPLOY_zh.md](DEPLOY_zh.md)

### 跑仿真

1. [DEPLOY_zh.md](DEPLOY_zh.md) 的编译和同步部分。
2. [COMMANDS_SIM2SIM_zh.md](../src/M20_sdk_deploy/COMMANDS_SIM2SIM_zh.md)。
3. [DEVELOPMENT_zh.md](DEVELOPMENT_zh.md) 的测试层和判据。

### 上真机

1. [STATUS_zh.md](STATUS_zh.md) 确认当前阻塞。
2. [DEPLOY_zh.md](DEPLOY_zh.md) 按只上腿、机械臂、VR 分阶段执行。
3. [COMMANDS_SIM2REAL_zh.md](../src/M20_sdk_deploy/COMMANDS_SIM2REAL_zh.md) 复制现场命令。
4. 站立热风险的处理状态见 [STATUS_zh.md](STATUS_zh.md)，历史证据见
   [DEFECTS_zh.md](DEFECTS_zh.md) 的 DEF-059/DEF-060。

### 修改策略或接口

1. [CONTRACT_zh.md](CONTRACT_zh.md)。
2. [src/M20_sdk_deploy/policy/README.md](../src/M20_sdk_deploy/policy/README.md)。
3. [DEVELOPMENT_zh.md](DEVELOPMENT_zh.md) 的最低测试和合并门槛。

## 文档边界

- `STATUS_zh.md` 是唯一的当前任务清单。
- 根目录 [AGENTS.md](../AGENTS.md) 是后续 Codex session 的必读工作说明。
- 不再创建按日期命名的 DONE、TODO、验收报告或下个 session prompt。
- 新缺陷只追加到 `DEFECTS_zh.md`，并同步更新 `STATUS_zh.md` 的当前影响。
- 接口和真机参数变化必须同步 `CONTRACT_zh.md`。
- 部署步骤变化必须同步 `DEPLOY_zh.md`。
- 历史实验细节保留在 git 和 `DEFECTS_zh.md`，不再复制到多个文档。

## 历史说明

原 `docs/review/` 文档、早期六提交总结、验收报告、全代码走查和离线资源清单已在
2026-10-22 合并或删除。关键结论已经迁入上述长期文档；需要追溯原始上下文时使用 git：

```bash
git log --all -- docs/
git show 659dc86:docs/review/DONE_zh.md
git show 659dc86:docs/review/TODO_zh.md
git show 659dc86:docs/review/WORKFLOW_zh.md
```
