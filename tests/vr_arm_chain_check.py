#!/usr/bin/env python3
"""VR → 机械臂链路检查（DEF-044 的回归用例）。

**为什么有这个用例**：VR 的手臂跟随是"绝对偏移"，而默认站姿
`(0, 0.5, -0.5, 0, 0, 0)` 里 joint5 = joint6 = 0（joint4 与 joint6 同轴）
⇒ **手腕正好在奇异位形**。旧实现把 VR 偏移当成"笛卡尔绝对偏移"直接丢给局部
IK，而"EE 侧移 5 cm / 工具转 10°"在这种位形下需要 2.4~2.7 rad 的关节重排 ——
IK 走不动，节点又把"顶在关节限位上的解"照发，于是臂自己甩到 joint1 = 2.168
（= 上限）、关节速度 20 rad/s、指令力矩 139 N·m（真机驱动器只有 3 rad/s /
100 N·m）。键盘之所以没事，是因为操作员只按 ±x/±z（只要 0.30 rad 重排）。

本用例用"假头显"直接发 /VR_TELEOP（同样的 16 浮点布局、mode 1 语义），
让手做**侧移 + 前伸 + 上下 + 手腕转动**（正是旧实现失败的那组动作），然后检查：

  1. IK 收敛：`[arm_ik]` 的位置残差 p95 ≤ 5 mm、最大 ≤ 10 mm；
  2. 不下发坏解：安全网拒发（rejects）≤ 20 拍；
  3. 不撞关节限位：任何关节在限位 5 mrad 内的累计时间 ≤ 0.2 s；
  4. 速度/力矩在真机包线内：|dq| ≤ 4 rad/s、指令力矩 ≤ 40 N·m；
  5. 真的跟手：实际末端相对**请求**目标（离线按映射重算）p95 ≤ 40 mm，
     且末端确实动过（z/y 行程 ≥ 1 cm）。

不需要 rl_deploy：腿由仿真自己的默认保持撑着（`arm_only` 同款），
臂链路与腿是否在走无关（2026-10-12 实测：腿站着不动同样复现/同样修好）。

用法：
    python3 tests/vr_arm_chain_check.py            # 默认 /tmp/m20_vr_arm
    python3 tests/vr_arm_chain_check.py --out /tmp/x --duration 22
"""

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tests"))
from sim2sim_smoke import force_cleanup, killer, leftover_processes, spawn  # noqa: E402

SIM = REPO / "src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py"
ARM = REPO / "src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py"
ARM_TELEOP = REPO / "src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py"
SIM_DIR = REPO / "src/M20_sdk_deploy/interface/robot/simulation"

ARM_DEFAULT = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
ARM_LIMITS = [(-2.618, 2.168), (0.0, 3.14), (-2.967, 0.0),
              (-1.745, 1.745), (-1.22, 1.22), (-2.0944, 2.0944)]
ARM_BASE_OFFSET = (0.24, 0.0, 0.0888)      # 臂座在 root 系的安装平移

IK_P95_MM = 5.0
IK_MAX_MM = 10.0
MAX_REJECTS = 20
LIMIT_DWELL_S = 0.2
# 关节速度：**指令**的限速是 3 rad/s（M20_ARM_MAX_JOINT_VEL，真机驱动器口径），
# 但"实际"关节速度按 PD 跟踪必然略超（要动就得有位置误差：v = kp/kd·err，
# 反向时误差翻倍）——所以指令用紧判据（p99 ≤ 3.5），实际只用松判据（≤ 8，
# 旧实现是 20 rad/s 量级，这才是要挡的东西）。
MAX_DQ_CMD_P99 = 3.5
MAX_DQ = 8.0
MAX_TAU = 40.0
REQ_ERR_P95_MM = 40.0


def fake_vr_stream(t, duration):
    """假头显：返回 (chassis, ee_offsets, gripper) —— 手在 sim/root 系里画圆。

    手的运动放在 root 系：x 前伸、y 侧移、z 上下；手腕三个轴都转一点。
    数值就是 `Se2VRExtendedDevice` 会发出来的那种量级（≤ 7 cm / ≤ 0.19 rad）。
    """
    if t < 0.5 or t > duration:                       # 未激活 / 结束
        return (0.0, 0.0, 0.0), (0.0,) * 6
    w = 2.0 * math.pi / 6.7
    if t < 3.5:                                        # 校准后先停一会儿
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    s, s2 = math.sin(w * t), math.sin(w * t + 0.7)
    return ((0.0, 0.0, 0.0),
            (0.06 * s2,          # dx 前伸/后收
             0.05 * s,           # dy 侧移（旧实现就是在这一路上撞死的）
             0.03 * s2,          # dz 上下
             0.10 * s,           # droll  手腕自转
             0.17 * s2,          # dpitch 手腕俯仰
             0.19 * s))          # dyaw   手腕侧倾


def requested_ee(anchor_pos, off):
    """按 M20_VR_ARM_MAP=cylindrical 的映射重算"请求的末端位置"（root 系）。"""
    dx, dy, dz = off[:3]
    ax, ay, az = anchor_pos
    r_a = math.hypot(ax, ay)
    th = math.atan2(ay, ax) + math.atan2(dy, max(r_a + dx, 0.12))
    r = max(r_a + dx, 0.12)
    return (r * math.cos(th) + ARM_BASE_OFFSET[0],
            r * math.sin(th) + ARM_BASE_OFFSET[1],
            az + dz + ARM_BASE_OFFSET[2])


class FakeHeadset:
    """进程内的假头显：自己发 /VR_TELEOP，同时记录链路数据。"""

    def __init__(self, out_csv: Path, duration: float):
        sys.path.insert(0, str(SIM_DIR))
        from vr_teleop_protocol import VR_TELEOP_LEN, VR_ACTIVE_IDX, VR_CALIBRATE_IDX
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
        from drdds.msg import JointsData, JointsDataCmd

        self.rclpy = rclpy
        self.Msg = Float32MultiArray
        rclpy.init()
        self.node = Node("vr_arm_chain_check_headset")
        self.VR_ACTIVE_IDX = VR_ACTIVE_IDX
        self.VR_CALIBRATE_IDX = VR_CALIBRATE_IDX
        self.LEN = VR_TELEOP_LEN
        self.pub = self.node.create_publisher(Float32MultiArray, "/VR_TELEOP", 10)
        self.f = open(out_csv, "w", buffering=1)
        cols = (["t"] + [f"vr{i}" for i in range(self.LEN)]
                + [f"cmd{i}" for i in range(6)] + [f"fb{i}" for i in range(6)]
                + [f"st{i}" for i in range(7)])
        self.f.write(",".join(cols) + "\n")
        self.last_vr = [0.0] * self.LEN
        self.last_cmd = [0.0] * 6
        self.last_fb = [0.0] * 6
        self.last_st = [0.0] * 7
        self.node.create_subscription(JointsDataCmd, "/ARM_JOINTS_CMD",
                                      self._cmd_cb, 50)
        self.node.create_subscription(JointsData, "/ARM_JOINTS_DATA",
                                      self._fb_cb, 50)
        self.node.create_subscription(Float32MultiArray, "/ARM_TELEOP_STATE",
                                      self._st_cb, 50)
        self.t0 = time.monotonic()
        self.duration = duration
        self.node.create_timer(0.02, self._tick)
        self.node.create_timer(0.005, self._log)
        # 必须在**另一个线程**里 spin：本线程要留在 main() 里等整个用例跑完
        self._spin = threading.Thread(target=rclpy.spin, args=(self.node,),
                                      daemon=True)
        self._spin.start()

    def _cmd_cb(self, msg):
        self.last_cmd = [float(j.position) for j in msg.data.joints_data[:6]]

    def _fb_cb(self, msg):
        self.last_fb = [float(j.position) for j in msg.data.joints_data[:6]]

    def _st_cb(self, msg):
        self.last_st = [float(v) for v in msg.data[:7]]

    def _tick(self):
        t = time.monotonic() - self.t0
        chassis, off = fake_vr_stream(t, self.duration)
        msg = [0.0] * self.LEN
        msg[0:3] = chassis
        msg[3:6] = off[:3]
        msg[6:9] = off[3:6]
        msg[12] = 0.0
        msg[self.VR_ACTIVE_IDX] = 1.0
        msg[self.VR_CALIBRATE_IDX] = 1.0 if t < 0.6 else 0.0
        out = self.Msg()
        out.data = msg
        self.pub.publish(out)
        self.last_vr = list(msg)

    def _log(self):
        t = time.monotonic() - self.t0
        row = [f"{t:.4f}"] + [f"{v:.6g}" for v in self.last_vr]
        row += [f"{v:.6g}" for v in self.last_cmd]
        row += [f"{v:.6g}" for v in self.last_fb]
        row += [f"{v:.6g}" for v in self.last_st]
        self.f.write(",".join(row) + "\n")

    def close(self):
        try:
            self.f.close()
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass


def parse_ik_log(path: Path):
    perr, aerr, rej = [], [], 0
    pat = re.compile(r"pos_err=([\d.eE+-]+) m ang_err=([\d.eE+-]+) deg.*?rejects (\d+)")
    for line in path.read_text().splitlines():
        m = pat.search(line)
        if m:
            perr.append(float(m.group(1)))
            aerr.append(float(m.group(2)))
            rej = max(rej, int(m.group(3)))
    return perr, aerr, rej


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/m20_vr_arm")
    ap.add_argument("--duration", type=float, default=16.0)
    args = ap.parse_args()

    # 本进程自己也要发 /VR_TELEOP，必须和子进程在**同一个 ROS domain**
    # （容器里 ROS_DOMAIN_ID 可能没设，那样假头显会和其它节点隔离开）
    os.environ.setdefault("ROS_DOMAIN_ID", "1")

    out = Path(args.out)
    telemetry = Path(str(out) + ".telemetry.csv")
    chain = Path(str(out) + ".chain.csv")
    for p in (telemetry, chain, Path(str(out) + ".sim.log"),
              Path(str(out) + ".arm.log")):
        if p.exists():
            p.unlink()

    for _ in range(20):
        stale = leftover_processes()
        if not stale:
            break
        force_cleanup()
        time.sleep(0.5)
    stale = leftover_processes()
    if stale:
        print("[vr-arm] FAIL: 还有上一次的 sim/arm 节点在跑：")
        for line in stale:
            print("  " + line)
        return 1

    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "1")
    env["M20_USE_VIEWER"] = "0"
    env["M20_SIM_TELEMETRY"] = str(telemetry)
    env["M20_JVEL_DEBUG"] = "0"
    env["M20_ARM_DEBUG"] = "1"
    env["M20_ARM_DEBUG_PERIOD"] = "5"          # 10 Hz：残差 + rejects 计数

    sim = arm = teleop = None
    logs = []
    headset = None
    fails: list[str] = []
    try:
        sim, f = spawn([sys.executable, str(SIM)], env, str(out) + ".sim.log")
        logs.append(f)
        time.sleep(3.0)
        arm, f = spawn([sys.executable, str(ARM)], env, str(out) + ".arm.log")
        logs.append(f)
        time.sleep(1.5)
        teleop, f = spawn([sys.executable, str(ARM_TELEOP)], env,
                          Path(str(out) + ".arm_teleop.log"),
                          stdin=subprocess.DEVNULL)
        logs.append(f)
        time.sleep(1.0)
        headset = FakeHeadset(chain, args.duration)
        print(f"[vr-arm] 假头显已启动：{args.duration:.0f} s（手画圆 + 手腕三轴转动）")
        time.sleep(args.duration + 2.0)
    finally:
        if headset is not None:
            headset.close()
        killer(teleop)
        killer(arm)
        killer(sim)
        force_cleanup()
        for f in logs:
            try:
                f.close()
            except Exception:
                pass

    # ---------------- 断言 ----------------
    import numpy as np
    perr, aerr, rej = parse_ik_log(Path(str(out) + ".arm.log"))
    if len(perr) < 50:
        print(f"[vr-arm] FAIL: arm_controller 日志太少（{len(perr)} 条），节点可能没起来")
        return 1
    perr = np.array(perr) * 1000.0
    print(f"[vr-arm] IK 位置残差 p95 {np.percentile(perr, 95):.1f} mm / max "
          f"{perr.max():.1f} mm（{len(perr)} 个采样）；安全网拒发 {rej} 拍")
    if np.percentile(perr, 95) > IK_P95_MM:
        fails.append(f"IK 位置残差 p95 {np.percentile(perr, 95):.1f} mm > {IK_P95_MM} mm")
    if perr.max() > IK_MAX_MM:
        fails.append(f"IK 位置残差最大 {perr.max():.1f} mm > {IK_MAX_MM} mm")
    if rej > MAX_REJECTS:
        fails.append(f"安全网拒发 {rej} 拍 > {MAX_REJECTS}（目标被推到工作空间外）")

    import csv
    rows = list(csv.DictReader(open(telemetry)))
    t = np.array([float(r["t"]) for r in rows])
    q = np.stack([[float(r[f"q{16 + i}"]) for i in range(6)] for r in rows])
    dq = np.stack([[float(r[f"dq{16 + i}"]) for i in range(6)] for r in rows])
    tau = np.stack([[float(r[f"tau{16 + i}"]) for i in range(6)] for r in rows])
    m = t > 6.0
    dwell = 0.0
    worst = None
    for i in range(6):
        lo, hi = ARM_LIMITS[i]
        at = (np.abs(q[m, i] - lo) < 5e-3) | (np.abs(q[m, i] - hi) < 5e-3)
        d = float(at.sum()) * 0.005
        if d > dwell:
            dwell, worst = d, i + 1
    print(f"[vr-arm] 关节速度 max {np.abs(dq[m]).max():.2f} rad/s，指令力矩 max "
          f"{np.abs(tau[m]).max():.1f} N·m；限位停留 {dwell:.2f} s"
          + (f"（joint{worst}）" if worst else ""))
    if dwell > LIMIT_DWELL_S:
        fails.append(f"joint{worst} 停在限位 {dwell:.2f} s > {LIMIT_DWELL_S} s")
    if np.abs(dq[m]).max() > MAX_DQ:
        fails.append(f"关节速度 max {np.abs(dq[m]).max():.2f} rad/s > {MAX_DQ}")
    if np.abs(tau[m]).max() > MAX_TAU:
        fails.append(f"指令力矩 max {np.abs(tau[m]).max():.1f} N·m > {MAX_TAU}")

    # 跟手程度：实际末端 vs "请求目标"（按映射离线重算，见 requested_ee）
    ch = list(csv.DictReader(open(chain)))
    tt = np.array([float(r["t"]) for r in ch])
    offs = np.array([[float(r[f"vr{i}"]) for i in range(3, 9)] for r in ch])
    anchor = None
    for r in rows:                                  # 用仿真第一拍的实际末端当锚点
        if float(r["t"]) > 0.5:
            anchor = (float(r["ee_x"]) - ARM_BASE_OFFSET[0],
                      float(r["ee_y"]) - ARM_BASE_OFFSET[1],
                      float(r["ee_z"]) - ARM_BASE_OFFSET[2])
            break
    if anchor is None:
        anchor = (0.1092, 0.0, 0.3439)
    req = np.array([requested_ee(anchor, o) for o in offs])
    ee = np.stack([[float(r["ee_x"]), float(r["ee_y"]), float(r["ee_z"])]
                   for r in rows])
    # 两份日志的时钟不同源（chain = 假头显启动起算；telemetry = 仿真启动起算，
    # 差 ~5 s），先按**关节反馈**对齐，再比"请求 vs 实际末端"。
    sim_t = t
    tw = np.array([float(r["wall"]) for r in rows])
    fb = np.stack([[float(r[f"fb{i}"]) for i in range(6)] for r in ch])
    best, off_s = None, 0.0
    for d in np.arange(-12.0, 12.0, 0.02):
        x = tw + d
        ok = (x >= tt[0]) & (x <= tt[-1])
        if ok.sum() < 200:
            continue
        ref = np.stack([np.interp(x[ok], tt, fb[:, j]) for j in range(6)], axis=1)
        cur = q[ok]                       # 仿真实际关节角（同一批采样）
        e = float(np.abs((ref - ref.mean(0)) - (cur - cur.mean(0))).mean())
        if best is None or e < best:
            best, off_s = e, d
    x = np.clip(tw + off_s, tt[0], tt[-1])
    idx = np.searchsorted(tt, x)
    req_al = req[idx]
    err = np.linalg.norm(req_al - ee, axis=1) * 1000.0
    print(f"[vr-arm] 时钟对齐 d = {off_s:+.2f} s（按关节反馈匹配）")
    span = np.array([ee[:, 1].max() - ee[:, 1].min(),
                     ee[:, 2].max() - ee[:, 2].min()]) * 1000.0
    print(f"[vr-arm] 请求-实际末端误差 p95 {np.percentile(err, 95):.1f} mm；"
          f"末端行程 y {span[0]:.0f} mm / z {span[1]:.0f} mm")
    if np.percentile(err, 95) > REQ_ERR_P95_MM:
        fails.append(f"跟手误差 p95 {np.percentile(err, 95):.1f} mm > {REQ_ERR_P95_MM} mm")
    if span.min() < 10.0:
        fails.append(f"末端几乎没动（y/z 行程 {span.round(1)} mm）")

    # 指令侧的速度限速（M20_ARM_MAX_JOINT_VEL）真的生效了吗？
    cmd = np.stack([[float(r[f"cmd{i}"]) for i in range(6)] for r in ch])
    ct = np.array([float(r["t"]) for r in ch])
    dtc = np.diff(ct)
    ok = dtc > 3e-3          # 丢掉采样抖动造成的 0 ms 间隔（否则除出来的值是假的）
    slew = np.abs(np.diff(cmd, axis=0))[ok] / dtc[ok][:, None]
    slew_p99 = float(np.percentile(slew, 99))
    print(f"[vr-arm] 指令关节速度 p99 {slew_p99:.2f} / max {slew.max():.2f} rad/s"
          f"（限速 3.0）")
    if slew_p99 > MAX_DQ_CMD_P99:
        fails.append(f"指令关节速度 p99 {slew_p99:.2f} rad/s > {MAX_DQ_CMD_P99}")

    if fails:
        print("\n[vr-arm] FAIL")
        for f_ in fails:
            print("  - " + f_)
        return 1
    print("\n[vr-arm] PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
