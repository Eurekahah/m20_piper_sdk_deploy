#!/usr/bin/env python3
"""L3 端到端 sim2sim 冒烟测试（在容器 m20_piper_ros 里跑）。

做四件事：
  1. 起 MuJoCo 仿真节点（无头，开遥测落盘）；
  2. 起 rl_deploy，用管道喂键盘命令走进状态机 idle → standup → RL（可选再给速度命令）；
  3. 采集遥测 CSV，按 `docs/sim2sim_layout_contract_zh.md` 的判据算"高度/倾角"曲线；
  4. 打印 PASS/FAIL 与关键数字，FAIL 时退出码 1。

判据（与训练 terminate 一致）：
  * 躯干高度 height = root_z − mean(四个 wheel body 的 z) + 0.09
  * 倾角 tilt = acos(-g_z)（g = 机体系重力投影；直立 0，侧躺 90°）
  * 摔倒：tilt > 0.8 rad 或 height < 0.30 m

模式：
  --mode hold    只起仿真（不起 rl_deploy）：验证"裸模型+默认保持"能不能站住
  --mode stand   起 rl_deploy 并走到 standup（不进 RL）
  --mode rl      走到 RL，命令全零（默认；对应契约文档的"零位移命令"步骤）
  --mode walk    走到 RL 后按住 w 前进（vx = +0.7 m/s，键盘上限）

用法（容器内）::

  cd /root/m20_piper_ws
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 tests/sim2sim_smoke.py --mode rl --duration 25

注意：本脚本会 `pkill -f mujoco_simulation_ros2.py` / `pkill -f rl_deploy`
清理自己起的进程（只杀它启动的那两个 PID 及其子进程）。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SIM = REPO / "src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py"
WHEEL_RADIUS = 0.09
TILT_LIMIT = 0.8          # rad
HEIGHT_LIMIT = 0.30       # m


def spawn(cmd, env, log_path, stdin=subprocess.DEVNULL):
    log = open(log_path, "w")
    return subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                            stdin=stdin, start_new_session=True, bufsize=0), log


def killer(proc):
    """杀整个进程组（rl_deploy 会自己开线程，但没有子进程）。"""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def quat_to_R(w, x, y, z):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def analyse(csv_path: Path):
    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        return None
    out = {"t": [], "height": [], "tilt": []}
    for r in rows:
        t = float(r["t"])
        base_z = float(r["base_z"])
        wheel_z = [float(r[k]) for k in ("wheel_z_fl", "wheel_z_fr", "wheel_z_hl", "wheel_z_hr")]
        height = base_z - sum(wheel_z) / 4.0 + WHEEL_RADIUS
        R = quat_to_R(*(float(r[k]) for k in ("base_qw", "base_qx", "base_qy", "base_qz")))
        tilt = math.acos(max(-1.0, min(1.0, R[2][2])))
        out["t"].append(t)
        out["height"].append(height)
        out["tilt"].append(tilt)
    out["n"] = len(rows)
    out["t_end"] = out["t"][-1]
    n = len(out["t"])
    tail = slice(int(n * 0.5), n)          # 后半段（进入 RL 之后）
    out["height_tail_mean"] = sum(out["height"][tail]) / len(out["height"][tail])
    out["height_min"] = min(out["height"])
    out["tilt_max_tail"] = max(out["tilt"][tail])
    out["tilt_max"] = max(out["tilt"])
    out["fell_at"] = next((out["t"][i] for i in range(n)
                           if out["tilt"][i] > TILT_LIMIT or out["height"][i] < HEIGHT_LIMIT), None)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["hold", "stand", "rl", "walk"], default="rl")
    ap.add_argument("--duration", type=float, default=25.0, help="总时长（秒）")
    ap.add_argument("--out", default="/tmp/m20_sim2sim", help="日志与遥测的输出前缀")
    ap.add_argument("--viewer", action="store_true", help="开 MuJoCo 窗口（默认无头）")
    args = ap.parse_args()

    out_prefix = Path(args.out)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    telemetry = out_prefix.with_suffix(".telemetry.csv")
    sim_log = out_prefix.with_suffix(".sim.log")
    deploy_log = out_prefix.with_suffix(".deploy.log")
    if telemetry.exists():
        telemetry.unlink()

    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "1")
    env["M20_USE_VIEWER"] = "1" if args.viewer else "0"
    env["M20_SIM_TELEMETRY"] = str(telemetry)
    env["M20_JVEL_DEBUG"] = "0"

    sim = deploy = None
    sim_f = dep_f = None
    try:
        print(f"[smoke] mode={args.mode} duration={args.duration}s out={out_prefix}*")
        sim, sim_f = spawn([sys.executable, str(SIM)], env, sim_log)
        time.sleep(2.0)                      # 等仿真节点起来并把姿态稳住

        if args.mode == "hold":
            # 只观察仿真自己的默认保持（不起 rl_deploy）
            t_start = time.time()
            while time.time() - t_start < args.duration:
                time.sleep(0.2)
        else:
            # rl_deploy 用管道接键盘（KeyboardInterface 是非阻塞读 stdin），
            # 按键脚本：z 站立 → 4 s 后 c 进 RL（stand_duration = 2 s，留余量）
            deploy, dep_f = spawn(["ros2", "run", "m20_sdk_deploy", "rl_deploy"],
                                  env, deploy_log, stdin=subprocess.PIPE)
            time.sleep(2.0)

            def press(key: bytes):
                try:
                    deploy.stdin.write(key)
                    deploy.stdin.flush()
                except (BrokenPipeError, ValueError):
                    pass

            t_start = time.time()
            press(b"z")
            print("[smoke] sent 'z' (stand up)")
            if args.mode != "stand":
                time.sleep(4.0)                  # stand_duration = 2 s，留余量
                press(b"c")
                print("[smoke] sent 'c' (RL control)")

            while time.time() - t_start < args.duration:
                if args.mode == "walk":
                    press(b"w")                  # 键盘是"按住"语义，需持续重复
                time.sleep(0.2)
    finally:
        killer(deploy)
        killer(sim)
        for f in (sim_f, dep_f):
            try:
                if f:
                    f.close()
            except Exception:
                pass

    info = analyse(telemetry)
    if info is None:
        print(f"[smoke] FAIL: no telemetry rows in {telemetry}")
        return 1

    print(f"\n[smoke] telemetry: {info['n']} rows, t_end = {info['t_end']:.2f} s")
    print(f"  height  : min {info['height_min']:.3f} m, 后半段均值 {info['height_tail_mean']:.3f} m")
    print(f"  tilt    : max {math.degrees(info['tilt_max']):.1f}°, "
          f"后半段 max {math.degrees(info['tilt_max_tail']):.1f}°")

    fails = []
    if args.mode != "hold":
        need_rows = int(max(args.duration - 4.0, 5.0) * 100)
        if info["n"] < need_rows:
            fails.append(f"遥测行数太少（{info['n']} < {need_rows}）：仿真或 rl_deploy 提前挂了")
    if info["t_end"] < args.duration * 0.8:
        fails.append(f"遥测只录到 {info['t_end']:.1f}s（期望 ~{args.duration}s）")
    if info["fell_at"] is not None:
        fails.append(f"触发摔倒判据 @ t={info['fell_at']:.2f}s "
                     f"(tilt>{math.degrees(TILT_LIMIT):.0f}° 或 height<{HEIGHT_LIMIT} m)")

    if fails:
        print("\n[smoke] FAIL")
        for f in fails:
            print("  - " + f)
        print(f"  logs: {sim_log} / {deploy_log}")
        return 1
    print("\n[smoke] PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
