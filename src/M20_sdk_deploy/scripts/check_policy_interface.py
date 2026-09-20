#!/usr/bin/env python3
"""L1 离线验收：策略目录（policy.onnx + policy_layout.json）的接口与数值。

检查项（任一失败即退出码 1）：
  1. layout 的 kind / 维度自洽，且与 ONNX 的输入输出**形状与名字**一致；
  2. 24 个关节的原生序在 layout 里有记录，且与 runner 里写死的那张表一致
     （两边都改才算改对 —— 这是 DEF-005 的护栏）；
  3. 标称观测（默认姿态 + 零速度 + `ee_goal`=默认 + `body_pose`=0.513）下：
     输出有限、无 NaN、幅值在安全限幅内；
  4. 若目录里有 `policy.pt`（TorchScript），ONNX 与它做**相对**误差对照
     （判据 < 1e-5，按输出幅值归一 —— 策略输出没有归一化）。

用法::

  python3 src/M20_sdk_deploy/scripts/check_policy_interface.py \
      src/M20_sdk_deploy/policy/m20_piper_history_20260920
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort

DEFAULT_DIR = (Path(__file__).resolve().parent / ".." / "policy"
               / "m20_piper_history_20260920")

# runner 里写死的原生序（M20PiperPolicyRunner::NativeOrder），必须与布局文件一致
RUNNER_NATIVE_ORDER = [
    "fl_hipx_joint", "fr_hipx_joint", "hl_hipx_joint", "hr_hipx_joint", "arm_joint1",
    "fl_hipy_joint", "fr_hipy_joint", "hl_hipy_joint", "hr_hipy_joint", "arm_joint2",
    "fl_knee_joint", "fr_knee_joint", "hl_knee_joint", "hr_knee_joint", "arm_joint3",
    "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
    "arm_joint4", "arm_joint5", "arm_joint6", "gripper_joint1", "gripper_joint2",
]
DEFAULT_JOINTS = {  # 训练 init_state.joint_pos
    "fl_hipx_joint": 0.0, "fr_hipx_joint": 0.0, "hl_hipx_joint": 0.0, "hr_hipx_joint": 0.0,
    "fl_hipy_joint": -0.6, "fr_hipy_joint": -0.6, "hl_hipy_joint": 0.6, "hr_hipy_joint": 0.6,
    "fl_knee_joint": 1.0, "fr_knee_joint": 1.0, "hl_knee_joint": -1.0, "hr_knee_joint": -1.0,
    "fl_wheel_joint": 0.0, "fr_wheel_joint": 0.0, "hl_wheel_joint": 0.0,
    "hr_wheel_joint": 0.0,
    "arm_joint1": 0.0, "arm_joint2": 0.5, "arm_joint3": -0.5, "arm_joint4": 0.0,
    "arm_joint5": 0.0, "arm_joint6": 0.0,
    "gripper_joint1": 0.0, "gripper_joint2": 0.0,
}
EE_GOAL_DEFAULT = np.array([0.3492, 0.0, 0.4327, 0.7373, 0.0, 0.6756, 0.0], np.float32)
BODY_HEIGHT_DEFAULT = 0.513
FALLBACK_ACTION_LIMIT = 10.0      # 标称状态下动作幅值的合理上限（超了要人工看）

fails: list[str] = []


def check(ok: bool, msg: str) -> None:
    print(("  [PASS] " if ok else "  [FAIL] ") + msg)
    if not ok:
        fails.append(msg)


def build_nominal_obs(layout: dict) -> tuple[np.ndarray, np.ndarray]:
    n = len(RUNNER_NATIVE_ORDER)
    joint_pos = np.array([DEFAULT_JOINTS[j] for j in RUNNER_NATIVE_ORDER], np.float32)
    obs = np.zeros(layout["policy_obs_dim"], np.float32)
    obs[3:6] = (0.0, 0.0, -1.0)                     # projected gravity（直立）
    obs[9:9 + n] = joint_pos * 0.0                   # (q - q_default) = 0
    obs[9 + n:9 + 2 * n] = 0.0
    a0 = 9 + 2 * n
    obs[a0 + layout["action_dim"]:a0 + layout["action_dim"] + 7] = EE_GOAL_DEFAULT
    obs[a0 + layout["action_dim"] + 7:a0 + layout["action_dim"] + 10] = (
        BODY_HEIGHT_DEFAULT, 0.0, 0.0)

    step = np.zeros(layout["history_single_step_dim"], np.float32)
    step[3:6] = (0.0, 0.0, -1.0)
    hist = np.tile(step, layout["history_length"])
    return obs[None, :], hist[None, :]


def main() -> int:
    policy_dir = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_DIR.resolve()
    onnx_path = policy_dir / "policy.onnx"
    layout_path = policy_dir / "policy_layout.json"
    print(f"[L1] policy_dir = {policy_dir}")
    if not onnx_path.is_file() or not layout_path.is_file():
        print(f"[L1] FAIL: 缺少 {onnx_path.name} 或 {layout_path.name}")
        return 1
    layout = json.loads(layout_path.read_text())

    print("[1] layout 自洽")
    n = len(RUNNER_NATIVE_ORDER)
    check(layout.get("kind") == "history", f"kind = {layout.get('kind')}")
    act = layout["action_dim"]
    expect_obs = 3 + 3 + 3 + n + n + act + 7 + 3
    check(layout["policy_obs_dim"] == expect_obs,
          f"policy_obs_dim {layout['policy_obs_dim']} == 3+3+3+24+24+{act}+7+3 = {expect_obs}")
    check(layout["history_single_step_dim"] == 3 + 3 + n + n + act,
          f"history step {layout['history_single_step_dim']} == 3+3+24+24+{act}")
    check(act == 16, f"action_dim = {act}（12 腿 + 4 轮，动作里没有机械臂）")
    check(layout.get("history_order") == "oldest -> newest", "history 顺序 = 最旧→最新")

    print("[2] ONNX 形状与名字")
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ins = {i.name: list(i.shape) for i in sess.get_inputs()}
    outs = {o.name: list(o.shape) for o in sess.get_outputs()}
    check(set(ins) == {"policy_obs", "history_flat"} and set(outs) == {"action"},
          f"输入 {sorted(ins)} / 输出 {sorted(outs)}")
    check(ins["policy_obs"][1] == layout["policy_obs_dim"],
          f"policy_obs 形状 {ins['policy_obs']}")
    check(ins["history_flat"][1] ==
          layout["history_length"] * layout["history_single_step_dim"],
          f"history_flat 形状 {ins['history_flat']}")
    check(outs["action"][1] == act, f"action 形状 {outs['action']}")

    print("[3] 原生关节序（layout ↔ runner 写死的表）")
    order = layout.get("joint_order_native") or layout.get("native_joint_order")
    if order is None:
        check(False, "layout 里没有 joint_order_native 字段（runner 与文档无法交叉校验）")
    else:
        check(list(order) == RUNNER_NATIVE_ORDER,
              "layout.joint_order_native == M20PiperPolicyRunner::NativeOrder")

    print("[4] 标称观测下的输出")
    obs, hist = build_nominal_obs(layout)
    action = sess.run(["action"], {"policy_obs": obs, "history_flat": hist})[0][0]
    print("      legs[12]  : " + " ".join("%+.3f" % v for v in action[:12]))
    print("      wheels[4] : " + " ".join("%+.3f" % v for v in action[12:16]))
    check(np.all(np.isfinite(action)), "输出全为有限值（无 NaN/Inf）")
    amax = float(np.abs(action).max())
    check(amax < FALLBACK_ACTION_LIMIT,
          f"标称状态下 |a|max = {amax:.3f} < {FALLBACK_ACTION_LIMIT}")

    print("[5] 与 TorchScript 对照（若存在 policy.pt）")
    pt_path = policy_dir / "policy.pt"
    if pt_path.is_file():
        try:
            import torch
            m = torch.jit.load(str(pt_path))
            with torch.no_grad():
                ref = m(torch.from_numpy(obs), torch.from_numpy(hist)).numpy()[0]
            denom = max(1.0, float(np.abs(ref).max()))
            rel = float(np.abs(ref - action).max()) / denom
            check(rel < 1e-5, f"ONNX vs TorchScript 相对误差 {rel:.2e}（<1e-5）")
        except ImportError:
            print("  [SKIP] 没有 torch，跳过")
    else:
        print("  [SKIP] 目录里没有 policy.pt")

    print()
    if fails:
        print(f"[L1] FAIL（{len(fails)} 项）")
        return 1
    print("[L1] PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
