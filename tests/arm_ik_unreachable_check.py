#!/usr/bin/env python3
"""IK 够不到目标时的行为回归（DEF-047）。

**要保证的语义**（2026-10-13 操作员要求）：把末端目标推到手够不到的地方是**正常
操作**，此时

  1. 臂**停在原地**（沿用上一拍指令，不下发"顶在关节限位上的解"—— 这条安全网保留，
     理由见 DEF-044）；
  2. **不许把被跟踪目标拉回实际位姿**（旧实现的"归位"逻辑）：`/ARM_TELEOP_STATE`
     应当保持在**请求**位姿，手往回一带就自然恢复跟随；
  3. 恢复可达之后必须重新跟手（不允许"卡住不动"）。

判据：
  * 不可达窗口：`|目标 − 实际|` **> 100 mm**（旧实现会把它拉回到 ~0 ⇒ 这条直接抓旧行为）；
    同期 `/ARM_JOINTS_CMD` 基本不变（相邻采样 |Δq| ≤ 0.01 rad）且实际关节与进入
    不可达前偏差 ≤ 0.05 rad（臂真的停住了、没有甩）；
  * 恢复窗口：`|目标 − 实际|` ≤ 30 mm，且实际末端与不可达结束时相比动过（> 10 mm）。

不需要 rl_deploy（腿由仿真自己的默认保持撑着），臂链路与腿无关（DEF-044 的结论）。

用法（容器内，仓库根目录）::

    python3 tests/arm_ik_unreachable_check.py
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tests"))
from sim2sim_smoke import force_cleanup, killer, leftover_processes, spawn  # noqa: E402

SIM_DIR = REPO / "src/M20_sdk_deploy/interface/robot/simulation"
SIM = SIM_DIR / "mujoco_simulation_ros2.py"
ARM = SIM_DIR / "arm_controller.py"
ARM_TELEOP = SIM_DIR / "arm_teleop_node.py"

# 时间线（秒，相对假头显启动）
T_START = 0.5        # 按 B（激活 + 标定）
T_NEAR = 2.0         # 可达：dx = +50 mm
T_FAR = 5.0          # 不可达：dz = +550 mm（目标在臂座上方 0.894 m，远超臂展 ~0.63 m）
T_BACK = 9.0         # 回到可达：dx = +30 mm
T_END = 12.5
DX_NEAR = 0.050
DZ_FAR = 0.550
DX_BACK = 0.030

TOL_HOLD_CMD = 0.010     # 不可达窗口内相邻采样的 |Δq| 上限 [rad]
TOL_HOLD_MOVE = 0.050    # 不可达窗口内实际关节相对"进入前的姿态"的偏差上限 [rad]
MIN_TARGET_GAP = 0.100   # 不可达窗口内 |目标 − 实际| 的下限 [m]（旧实现会 <10 mm）
MAX_RECOVER_GAP = 0.030  # 恢复窗口内 |目标 − 实际| 的上限 [m]
MIN_ACTUAL_MOVE = 0.010  # 恢复窗口内实际末端相对不可达结束时的位移下限 [m]


def load_kinematics():
    """复用 arm_controller 的 MDH/FK，把 /ARM_JOINTS_DATA 算成 root 系末端位姿。"""
    sys.path.insert(0, str(SIM_DIR))     # arm_controller 会 import 同目录的协议模块
    spec = importlib.util.spec_from_file_location("arm_controller_kin", ARM)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class Probe:
    """假头显 + 链路记录：把 (wall, VR 帧, 目标, 实际 FK, 关节指令, 臂反馈) 落 CSV。"""

    def __init__(self, out_csv: Path, t0: float, kin):
        from vr_teleop_protocol import (VR_ACTIVE_IDX, VR_CALIBRATE_IDX,
                                        VR_TELEOP_LEN)
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
        from drdds.msg import JointsData, JointsDataCmd

        self.np = np
        self.kin = kin
        self.LEN = VR_TELEOP_LEN
        self.ACTIVE = VR_ACTIVE_IDX
        self.CALIB = VR_CALIBRATE_IDX
        self.rclpy = rclpy
        self.Msg = Float32MultiArray
        self.t0 = t0

        rclpy.init()
        self.node = Node("arm_ik_unreachable_probe")
        self.pub = self.node.create_publisher(Float32MultiArray, "/VR_TELEOP", 10)
        self.f = open(out_csv, "w", buffering=1)
        cols = (["wall", "vr_dx", "vr_dz", "active"]
                + ["st_x", "st_y", "st_z"]                 # /ARM_TELEOP_STATE（目标，root 系）
                + ["fb0", "fb1", "fb2", "fb3", "fb4", "fb5"]   # /ARM_JOINTS_DATA
                + ["cmd0", "cmd1", "cmd2", "cmd3", "cmd4", "cmd5"])  # /ARM_JOINTS_CMD
        self.f.write(",".join(cols) + "\n")
        self.last_st = [0.0] * 3
        self.last_fb = [0.0] * 6
        self.last_cmd = [0.0] * 6
        self.last_vr = [0.0, 0.0, 0.0]
        self.node.create_subscription(JointsData, "/ARM_JOINTS_DATA",
                                      self._fb_cb, 50)
        self.node.create_subscription(JointsDataCmd, "/ARM_JOINTS_CMD",
                                      self._cmd_cb, 50)
        self.node.create_subscription(Float32MultiArray, "/ARM_TELEOP_STATE",
                                      self._st_cb, 50)
        self.node.create_timer(0.02, self._tick)
        self.node.create_timer(0.02, self._log)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _fb_cb(self, msg):
        self.last_fb = [float(j.position) for j in msg.data.joints_data[:6]]

    def _cmd_cb(self, msg):
        self.last_cmd = [float(j.position) for j in msg.data.joints_data[:6]]

    def _st_cb(self, msg):
        if len(msg.data) >= 7:
            self.last_st = [float(v) for v in msg.data[:3]]

    def _frame(self, t: float):
        msg = [0.0] * self.LEN
        if t < T_START:
            return msg
        msg[self.ACTIVE] = 1.0
        if t < T_START + 0.04:
            msg[self.CALIB] = 1.0
        if t < T_NEAR:
            pass                                   # 标定后先停在原点
        elif t < T_FAR:
            msg[3] = DX_NEAR                       # 可达：前伸 5 cm
        elif t < T_BACK:
            msg[5] = DZ_FAR                        # 不可达：抬高 55 cm
        else:
            msg[3] = DX_BACK                       # 回到可达：前伸 3 cm
        return msg

    def _tick(self):
        t = time.monotonic() - self.t0
        msg = self._frame(t)
        out = self.Msg()
        out.data = msg
        self.pub.publish(out)
        self.last_vr = [msg[3], msg[5], msg[13]]

    def _log(self):
        t = time.monotonic() - self.t0
        row = ([f"{t:.4f}", f"{self.last_vr[0]:.6g}", f"{self.last_vr[1]:.6g}",
                f"{self.last_vr[2]:.6g}"]
               + [f"{v:.6g}" for v in self.last_st]
               + [f"{v:.6g}" for v in self.last_fb]
               + [f"{v:.6g}" for v in self.last_cmd])
        self.f.write(",".join(row) + "\n")

    def root_ee_from_feedback(self, q):
        """臂反馈 → root 系末端位置（与 /ARM_TELEOP_STATE 同口径）。"""
        pos, _ = self.kin.fk_pose(self.kin.PIPER_MDH,
                                  self.np.asarray(q, dtype=float))
        return pos + self.kin.ARM_BASE_OFFSET

    def close(self):
        try:
            self.f.close()
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass


def mean(v):
    v = list(v)
    return sum(v) / len(v) if v else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/m20_arm_ik_unreachable")
    args = ap.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", "1")
    kin = load_kinematics()
    out = Path(args.out)
    chain = Path(str(out) + ".chain.csv")
    for p in (chain, Path(str(out) + ".sim.log"), Path(str(out) + ".arm.log"),
              Path(str(out) + ".arm_teleop.log")):
        if p.exists():
            p.unlink()
    for _ in range(20):
        if not leftover_processes():
            break
        force_cleanup()
        time.sleep(0.5)
    if leftover_processes():
        print("[ik-unreach] FAIL: 还有上一次的 sim/rl_deploy 在跑，先清掉")
        return 1

    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "1")
    env["M20_USE_VIEWER"] = "0"
    env.setdefault("M20_JVEL_DEBUG", "0")
    env.setdefault("M20_ARM_DEBUG", "1")

    sim = arm = teleop = None
    logs = []
    probe = None
    try:
        sim, f = spawn([sys.executable, str(SIM)], env, str(out) + ".sim.log")
        logs.append(f)
        time.sleep(3.0)
        arm, f = spawn([sys.executable, str(ARM)], env, str(out) + ".arm.log")
        logs.append(f)
        time.sleep(1.5)
        teleop, f = spawn([sys.executable, str(ARM_TELEOP)], env,
                          str(out) + ".arm_teleop.log", stdin=subprocess.DEVNULL)
        logs.append(f)
        time.sleep(1.0)
        probe = Probe(chain, time.monotonic(), kin)
        print(f"[ik-unreach] 时间线 {T_END:.1f} s：可达 {DX_NEAR*1000:.0f} mm → "
              f"不可达 {DZ_FAR*1000:.0f} mm（抬高） → 回到可达 {DX_BACK*1000:.0f} mm")
        time.sleep(T_END + 1.5)
    finally:
        if probe is not None:
            probe.close()
        killer(teleop)
        killer(arm)
        killer(sim)
        force_cleanup()
        for f in logs:
            try:
                f.close()
            except Exception:
                pass

    rows = list(csv.DictReader(open(chain)))
    if len(rows) < 200:
        print(f"[ik-unreach] FAIL: 采样太少（{len(rows)}）")
        return 1
    for r in rows:
        for k in r:
            r[k] = float(r[k])

    def win(t0, t1):
        return [r for r in rows if t0 <= r["wall"] <= t1]

    def gap(rs):
        """窗口内 |目标 − 实际| 的均值 [m]（同一帧的 /ARM_TELEOP_STATE vs FK 反馈）。"""
        gs = []
        for r in rs:
            ee = probe.root_ee_from_feedback([r[f"fb{i}"] for i in range(6)])
            tgt = [r["st_x"], r["st_y"], r["st_z"]]
            gs.append(math.dist(ee, tgt))
        return mean(gs), (max(gs) if gs else float("nan"))

    near = win(T_NEAR + 1.5, T_FAR - 0.3)      # 可达稳态
    far = win(T_FAR + 1.0, T_BACK - 0.3)       # 不可达
    back = win(T_BACK + 1.5, T_END - 0.3)      # 恢复
    fails: list[str] = []
    if not (near and far and back):
        print("[ik-unreach] FAIL: 某个窗口没有采样"
              f"（near {len(near)} / far {len(far)} / back {len(back)}）")
        return 1

    g_near, g_near_max = gap(near)
    g_far, g_far_max = gap(far)
    g_back, g_back_max = gap(back)
    print(f"[ik-unreach] |目标 − 实际|：可达 {g_near*1000:.1f} mm（max {g_near_max*1000:.1f}）"
          f"；不可达 **{g_far*1000:.1f} mm**（max {g_far_max*1000:.1f}）"
          f"；恢复 {g_back*1000:.1f} mm（max {g_back_max*1000:.1f}）")

    # 不可达窗口：目标没有被"归位"到实际位姿
    if g_far < MIN_TARGET_GAP:
        fails.append(f"不可达时目标被拉回了实际位姿（|目标−实际| 只有 "
                     f"{g_far*1000:.1f} mm ≤ {MIN_TARGET_GAP*1000:.0f} mm）"
                     "⇒ 归位逻辑还在")

    # 不可达窗口：臂停住、没有甩
    dq_cmd = []
    for a, b in zip(far[:-1], far[1:]):
        dq_cmd.append(max(abs(b[f"cmd{i}"] - a[f"cmd{i}"]) for i in range(6)))
    q_enter = [far[0][f"cmd{i}"] for i in range(6)]
    q_hold_dev = max(abs(far[-1][f"cmd{i}"] - q_enter[i]) for i in range(6))
    print(f"[ik-unreach] 不可达窗口：相邻采样 max|Δq_cmd| {max(dq_cmd):.4f} rad；"
          f"整窗指令漂移 {q_hold_dev:.4f} rad")
    if max(dq_cmd) > TOL_HOLD_CMD or q_hold_dev > TOL_HOLD_MOVE:
        fails.append("不可达时臂没有停住（指令一直在动）"
                     f"：max|Δq_cmd| {max(dq_cmd):.4f} / 漂移 {q_hold_dev:.4f} rad")

    # 恢复窗口：重新跟手
    if g_back > MAX_RECOVER_GAP:
        fails.append(f"目标回到可达后没有恢复跟手：|目标−实际| {g_back*1000:.1f} mm "
                     f"> {MAX_RECOVER_GAP*1000:.0f} mm")
    ee_far_end = probe.root_ee_from_feedback([far[-1][f"fb{i}"] for i in range(6)])
    ee_back = probe.root_ee_from_feedback([back[-1][f"fb{i}"] for i in range(6)])
    moved = math.dist(ee_far_end, ee_back)
    print(f"[ik-unreach] 恢复后实际末端位移 {moved*1000:.1f} mm")
    if moved < MIN_ACTUAL_MOVE:
        fails.append(f"恢复后实际末端几乎没动（{moved*1000:.1f} mm）"
                     "⇒ 卡在不可达状态里了")

    # 安全网本身没被削弱：不可达期间确实发生过拒发
    rej = 0
    for line in Path(str(out) + ".arm.log").read_text(errors="ignore").splitlines():
        if "rejects" in line:
            with_rej = line.split("rejects")[-1].split(")")[0].strip()
            try:
                rej = max(rej, int(with_rej))
            except ValueError:
                pass
    print(f"[ik-unreach] arm_controller 累计拒发 {rej} 拍")
    if rej == 0:
        fails.append("不可达期间一次拒发都没有 ⇒ 安全网/判据可能没生效（用例本身失效）")

    print()
    if fails:
        print("[ik-unreach] FAIL")
        for f in fails:
            print("  ✗ " + f)
        return 1
    print("[ik-unreach] PASS（够不到就停住、目标不归位、回位即恢复跟随）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
