# 策略目录约定（policy/）

一个子目录 = **一次训练的部署产物**。换策略 = 换目录，不覆盖旧目录。

```
policy/
  README.md                       ← 本文件
  m20_piper_flat_20260930/        ← run 2026-09-30_19-36-00_cloud_cap12_20k（**当前默认，平地**）
    policy.onnx                   ← 部署实际加载（输入 policy_obs + history_flat）
    policy.pt                     ← TorchScript，同接口（离线数值对照用）
    policy_layout.json            ← 接口契约的机器可读版本（维度/名字/关节序/自检结论）
  m20_piper_rough_20261002/       ← run 2026-10-02_00-44-42_cloud_slowvx20k（多地形）
    policy.onnx / policy.pt / policy_layout.json
  m20_piper_history_20260920/     ← run 2026-09-20_00-50-31（更早的平地模型，保留做 A/B）
    policy.onnx                   ← 部署实际加载（输入 policy_obs + history_flat）
    policy.pt                     ← TorchScript，同接口（离线数值对照用）
    policy_layout.json            ← 接口契约的机器可读版本（维度/名字/关节序/自检结论）
  <下一个 run>/                   ← 直接新增目录，别改旧的
```

## 命名

`<实验名>_<日期>`，日期取训练 run 的日期（`YYYYMMDD`）。
例：`m20_piper_rough_20261002`（experiment_name=history_adaptation，
run=2026-10-02_00-44-42_cloud_slowvx20k）、`m20_piper_flat_20260930`
（run=2026-09-30_19-36-00_cloud_cap12_20k，平地）、`m20_piper_history_20260920`
（run=2026-09-20_00-50-31，更早的平地）。

**多地形 vs 平地怎么选**（训练侧 `docs/deploy_code_delta_2026-10-09_zh.md` §5.4）：
平地用 `m20_piper_flat_20260930`（当前默认），多地形用 `m20_piper_rough_20261002`；
两者接口逐字段相同。

## 怎么换

1. 在训练侧跑导出：
   `python scripts/reinforcement_learning/rsl_rl/export_deploy_policy.py --run <run> --checkpoint model_19999.pt`
   → `<run>/exported_deploy/{policy.onnx, policy.pt, policy_layout.json}`；
2. 把整个 `exported_deploy/` 复制成 `policy/<实验名>_<日期>/`；
3. 给 `policy_layout.json` 补两个部署侧字段（**必须**，L1 会断言）：
   * `joint_order_native`：24 个关节的 articulation 原生序
     （由训练侧 `probe_deploy_layout.py` 的 `[1]` 段给出）；
   * `joint_order_action`：16 个动作槽位顺序（12 腿 + 4 轮）。
   参考 `m20_piper_rough_20261002/policy_layout.json`（或
   `m20_piper_history_20260920/policy_layout.json`）的写法与注释。
4. 跑 `python3 src/M20_sdk_deploy/scripts/check_policy_interface.py policy/<新目录>`
   确认 PASS（维度/名字/关节序/标称幅值/ONNX↔TorchScript 相对误差）；
5. 用 `M20_POLICY_DIR=/root/m20_piper_ws/src/M20_sdk_deploy/policy/<新目录>` 起
   `rl_deploy` 验证（⚠️ 该变量按**进程 cwd** 解析，写 `policy/<新目录>` 会找不到；
   代码里的默认目录是按 `__FILE__` 解析的，两者口径不同，所以这里用绝对路径）；
   确认无误后再改
   `state_machine/quadruped_wheel/rl_control_state.hpp` 里的默认目录；
6. 跑 `bash tests/run_all.sh`，把数字写进 `docs/review/DONE_zh.md`。

## 硬性约束

* **不要**放 `play.py` 导出的 `<run>/exported/policy.pt`（actor-only）：
  它的输入是 `[policy_obs, latent]`，latent 没有来源；`policy_layout.json` 的
  `kind` 字段会拦住它（runner 只接受 `kind=history`）。
* `policy.onnx` 与 `policy_layout.json` **必须成对**存在，且维度/名字一致
  （启动时 runner 会断言，L1 也会断言）。
* 旧目录保留（便于 A/B 与回滚），仓库里最多留 2~3 个；更老的删掉但在
  `docs/review/DONE_zh.md` 里留一行"引用过哪个 run"。
