#!/usr/bin/env python3
"""L3 命令组合扫描：RL control 状态下把"部署会用到的一整套命令"逐个下发，
逐段判定底盘是否稳、速度/姿态命令是否被跟随。

为什么单独做一个用例
--------------------
`sim2sim_smoke.py` 每档只测**一件事**（`rl` = 零命令、`walk` = 按住 w、`arm_move`
= 按 numpad 8），回答不了"部署时这一整套命令范围内到底行不行"。本用例把
速度幅值（vx 0.2→1.5）、侧移、转向、组合（边走边转 / 边走边侧移）、机身姿态
（高度/俯仰/横滚）串起来跑一遍，逐段给数字。

为什么用 `/VR_TELEOP` 而不是键盘
--------------------------------
部署键盘的速度键是**阶跃满量程**的（`w`=0.7 m/s、`a/d`=0.5、`q/e`=0.7 rad/s，
见 `keyboard_interface.hpp`），没法下发 0.2 / 1.0 / 1.5 这种幅值；而
`/VR_TELEOP`（`vr_teleop_protocol.py`，16 float）是**连续量**通道，策略侧同样走
`ApplyVrCommand`（`rl_control_state.hpp`）⇒ 这才是能扫"命令包线"的口子。
键盘仍负责模式键（`z` 站立 / `c` 进 RL / `r` 阻尼）。

判据（与训练 terminate 一致）
-----------------------------
* 躯干高度 `height = root_z − mean(四个 wheel body 的 z) + 0.09`
* 倾角 `tilt = acos(-g_z)`；摔倒：`tilt > 0.8 rad` 或 `height < 0.30 m`
* 实时因子（sim 时间/墙钟）必须 ≥ 0.95，否则结果不算数（见 DEF-034）

用法（容器内，仓库根目录，先 source ROS + install）：:

    python3 tests/command_sweep.py                  # 1 遍
    python3 tests/command_sweep.py --repeat 3       # 3 遍
    M20_POLICY_DIR=policy/m20_piper_rough_20261002 python3 tests/command_sweep.py

退出码：0 = 全部阶段没有摔倒且实时因子达标；1 = 有阶段摔倒 / 数据异常。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py"
RL_BIN = REPO / "install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy"

WHEEL_RADIUS = 0.09
TILT_LIMIT = 0.8          # rad（训练终止阈值）
HEIGHT_LIMIT = 0.30       # m
BASE_HEIGHT = 0.513       # 键盘默认机身高度命令 = VR 锚点

VR_LEN = 16
VR_CHASSIS, VR_EE_POS, VR_EE_EULER, VR_BODY = 0, 3, 6, 9
VR_GRIPPER, VR_ACTIVE, VR_CALIBRATE, VR_RESET = 12, 13, 14, 15

# 命令阶段：名字 / 时长 s / vx / vy / wz / dheight / dpitch / droll
# 速度幅值参考训练命令区间（vx ±5、vy ±1、wz ±1）与部署现实范围；
# 机身姿态偏移落在训练 `body_pose` 区间（height 0.33~0.55、pitch ±0.35、roll ±0.25）内。
PHASES: list[tuple[str, float, float, float, float, float, float, float]] = [
    ("idle",        4.0,  0.00,  0.00,  0.00,  0.000,  0.000,  0.000),
    ("vx +0.2",     4.0,  0.20,  0.00,  0.00,  0.000,  0.000,  0.000),
    ("vx +0.4",     4.0,  0.40,  0.00,  0.00,  0.000,  0.000,  0.000),
    ("vx +0.7",     5.0,  0.70,  0.00,  0.00,  0.000,  0.000,  0.000),
    ("vx +1.0",     5.0,  1.00,  0.00,  0.00,  0.000,  0.000,  0.000),
    ("vx +1.5",     5.0,  1.50,  0.00,  0.00,  0.000,  0.000,  0.000),
    ("vx -0.3",     4.0, -0.30,  0.00,  0.00,  0.000,  0.000,  0.000),
    ("vy +0.2",     4.0,  0.00,  0.20,  0.00,  0.000,  0.000,  0.000),
    ("vy -0.2",     4.0,  0.00, -0.20,  0.00,  0.000,  0.000,  0.000),
    ("wz +0.3",     4.0,  0.00,  0.00,  0.30,  0.000,  0.000,  0.000),
    ("wz -0.3",     4.0,  0.00,  0.00, -0.30,  0.000,  0.000,  0.000),
    ("wz +0.7",     4.0,  0.00,  0.00,  0.70,  0.000,  0.000,  0.000),
    ("vx+wz",       5.0,  0.50,  0.00,  0.30,  0.000,  0.000,  0.000),
    ("vx+vy",       5.0,  0.50,  0.20,  0.00,  0.000,  0.000,  0.000),
    ("vx+wz big",   5.0,  0.70,  0.00,  0.50,  0.000,  0.000,  0.000),
    ("height 0.55", 4.0,  0.00,  0.00,  0.00, +0.037,  0.000,  0.000),
    ("height 0.35", 4.0,  0.00,  0.00,  0.00, -0.163,  0.000,  0.000),
    ("pitch +0.2",  4.0,  0.00,  0.00,  0.00,  0.000, +0.200,  0.000),
    ("pitch -0.2",  4.0,  0.00,  0.00,  0.00,  0.000, -0.200,  0.000),
    ("roll +0.15",  4.0,  0.00,  0.00,  0.00,  0.000,  0.000, +0.150),
    ("roll -0.15",  4.0,  0.00,  0.00,  0.00,  0.000,  0.000, -0.150),
    ("settle",      3.0,  0.00,  0.00,  0.00,  0.000,  0.000,  0.000),
]


# ---------------------------------------------------------------------------
# 进程管理（只按"本仓库路径"匹配，避免误伤别人的进程）
# ---------------------------------------------------------------------------
OUR_MARKERS = ("M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py",
               "lib/m20_sdk_deploy/rl_deploy")


def _cmdline(pid: int) -> str:
    try:
        return open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode()
    except OSError:
        return ""


def leftover() -> list[str]:
    out = []
    me = os.getpid()
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me:
            continue
        cmd = _cmdline(int(d))
        if cmd and any(m in cmd for m in OUR_MARKERS):
            out.append(f"{d}: {cmd.strip()[:120]}")
    return out


def force_cleanup() -> list[str]:
    killed = []
    for line in leftover():
        pid = int(line.split(":", 1)[0])
        try:
            os.kill(pid, signal.SIGKILL)
            killed.append(line)
        except (ProcessLookupError, PermissionError):
            pass
    return killed


def killer(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            return
    try:
        proc.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def spawn(cmd: list[str], env: dict, log: Path,
          stdin=None) -> tuple[subprocess.Popen, object]:
    f = open(log, "wb")
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env,
                         stdin=stdin, start_new_session=True)
    return p, f


# ---------------------------------------------------------------------------
# 遥测
# ---------------------------------------------------------------------------
def last_sim_t(csv_path: Path) -> float | None:
    """读 CSV 最后一行的时间（阶段边界用，避免依赖墙钟对齐）。"""
    try:
        with open(csv_path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - 4096))
            lines = f.read().decode(errors="ignore").strip().splitlines()
        for line in reversed(lines):
            parts = line.split(",")
            if len(parts) > 3 and parts[0][0].isdigit():
                return float(parts[0])
    except (OSError, ValueError, IndexError):
        return None
    return None


def load_rows(csv_path: Path) -> list[dict]:
    with open(csv_path, newline="") as f:
        return list(csv.DictReader(f))


STEADY_FRAC = 0.4      # 阶段窗口里前 40% 当瞬态丢掉，只用后 60% 量"稳态跟随"


def analyse(rows: list[dict], marks: list[tuple[str, float, float]]) -> dict:
    t = [float(r["t"]) for r in rows]
    h, tilt, vyaw = [], [], []
    for r in rows:
        z = float(r["base_z"]) - sum(float(r[f"wheel_z_{k}"])
                                     for k in ("fl", "fr", "hl", "hr")) / 4.0 + WHEEL_RADIUS
        h.append(z)
        tilt.append(math.acos(max(-1.0, min(1.0,
                    math.cos(float(r["roll"])) * math.cos(float(r["pitch"]))))))
        vyaw.append(float(r["yaw"]))
    wall = float(rows[-1]["wall"]) if "wall" in rows[0] else None
    rtf = t[-1] / wall if wall else None

    def idx_window(a: float, b: float) -> list[int]:
        return [i for i in range(len(t)) if a <= t[i] <= b]

    phases = []
    for name, a, b in marks:
        ids = idx_window(a, b)
        if len(ids) < 20:
            phases.append({"name": name, "n": len(ids), "empty": True})
            continue
        i1 = ids[-1]
        i0 = ids[int(len(ids) * STEADY_FRAC)]     # 稳态段起点
        dt = max(t[i1] - t[i0], 1e-6)
        # ⚠️ 命令是**机体系**的，而 base_x/base_y 是**世界系**。机器人转过向之后
        # 直接用世界系位移会让 vx/vy 测错（甚至变号）。这里先算世界系速度，
        # 再用窗口平均 yaw 投影回机体（小角度近似，窗口内 yaw 变化 <0.5 rad 时足够准）。
        vx_w = (float(rows[i1]["base_x"]) - float(rows[i0]["base_x"])) / dt
        vy_w = (float(rows[i1]["base_y"]) - float(rows[i0]["base_y"])) / dt
        yaw_mid = sum(float(rows[i]["yaw"]) for i in ids) / len(ids)
        cy, sy = math.cos(yaw_mid), math.sin(yaw_mid)
        vx = cy * vx_w + sy * vy_w
        vy = -sy * vx_w + cy * vy_w
        yaw0 = math.degrees(float(rows[i0]["yaw"]))
        # 转向：用 IMU 的 omega_z（瞬时角速度，机体系）取稳态均值。
        # 早先用"差分 yaw / dt"量，短窗口 + 起转瞬态会让数字严重偏小（实测同一段
        # 命令 0.7 rad/s 能测出 0.59、-0.3 rad/s 只测出 -0.07），换 omega_z 更可靠。
        w_ids = ids[int(len(ids) * STEADY_FRAC):] or ids
        wz = sum(float(rows[i]["omega_z"]) for i in w_ids) / len(w_ids)
        fell = next((t[i] for i in ids if tilt[i] > TILT_LIMIT or h[i] < HEIGHT_LIMIT), None)
        phases.append({
            "name": name, "n": len(ids), "empty": False,
            "h_min": min(h[i] for i in ids), "h_mean": sum(h[i] for i in ids) / len(ids),
            "tilt_max": max(tilt[i] for i in ids),
            "vx": vx, "vy": vy, "wz": wz, "fell_at": fell,
            "yaw0_deg": yaw0,
            "err_x": None, "err_y": None, "err_z": None,
            "ee_dx": None,
        })

    # 全局
    g_fell = next((t[i] for i in range(len(t)) if tilt[i] > TILT_LIMIT or h[i] < HEIGHT_LIMIT), None)
    return {"phases": phases, "rtf": rtf, "t_end": t[-1],
            "h_min": min(h), "tilt_max": max(tilt), "fell_at": g_fell, "n": len(rows)}


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def run_once(args) -> int:
    out_prefix = Path(args.out)
    telemetry = out_prefix.with_suffix(".telemetry.csv")
    sim_log = out_prefix.with_suffix(".sim.log")
    dep_log = out_prefix.with_suffix(".deploy.log")
    for p in (telemetry, sim_log, dep_log):
        p.unlink(missing_ok=True)

    for _ in range(20):
        if not leftover():
            break
        for line in force_cleanup():
            print("  [startup-cleanup] 强杀残留: " + line)
        time.sleep(0.5)
    if leftover():
        print("[sweep] FAIL: 仍有本仓库的 sim/rl_deploy 在跑，先清掉再来：")
        for line in leftover():
            print("  " + line)
        return 1

    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "1")
    env["M20_USE_VIEWER"] = "1" if args.viewer else "0"
    env["M20_SIM_TELEMETRY"] = str(telemetry)
    env.setdefault("M20_JVEL_DEBUG", "0")
    # ⚠️ 本进程自己也要在同一个 domain 上（rclpy.init 读的是 os.environ）。
    # 之前忘了这一步：发布节点跑在 domain 0、子进程在 domain 1 ⇒ 命令送不到，
    # 表现为"所有阶段实测速度都≈0"，很容易误判成策略不跟命令。
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    sim = deploy = None
    sim_f = dep_f = None
    marks: list[tuple[str, float, float]] = []
    pub = None
    try:
        sim, sim_f = spawn([sys.executable, str(SIM)], env, sim_log)
        time.sleep(2.5)
        deploy, dep_f = spawn([str(RL_BIN)], env, dep_log, stdin=subprocess.PIPE)
        time.sleep(2.0)

        def press(key: bytes) -> None:
            try:
                deploy.stdin.write(key)
                deploy.stdin.flush()
            except (BrokenPipeError, ValueError, AttributeError):
                pass

        press(b"z")
        time.sleep(4.0)                 # stand_duration = 2 s，留余量

        # ---- 起 VR 命令通道（策略侧 ApplyVrCommand 直接吃这三个速度/三个姿态偏移）----
        import rclpy
        from std_msgs.msg import Float32MultiArray
        rclpy.init()
        node = rclpy.create_node("command_sweep_pub")
        pub = node.create_publisher(Float32MultiArray, "/VR_TELEOP", 10)
        target = {"vx": 0.0, "vy": 0.0, "wz": 0.0,
                  "dh": 0.0, "dp": 0.0, "dr": 0.0, "stop": False}

        def publisher_loop() -> None:
            while not target["stop"] and rclpy.ok():
                msg = Float32MultiArray()
                data = [0.0] * VR_LEN
                data[VR_CHASSIS:VR_CHASSIS + 3] = [target["vx"], target["vy"], target["wz"]]
                data[VR_BODY:VR_BODY + 3] = [target["dh"], target["dp"], target["dr"]]
                data[VR_ACTIVE] = 1.0
                # 必须全是 float：std_msgs 的 data 字段对 int 会直接 assert 失败，
                # 而那个 assert 在后台线程里抛出来只会静默杀掉发布线程（命令全丢，
                # 表现为"机器人完全不动"，很难查）——所以在这里显式转换。
                msg.data = [float(x) for x in data]
                pub.publish(msg)
                time.sleep(0.02)        # 50 Hz

        th = threading.Thread(target=publisher_loop, daemon=True)
        th.start()
        time.sleep(1.0)                 # 让策略先锚定（Anchor = 0.513）

        press(b"c")
        time.sleep(2.5)                 # 进 RL，等它稳一下

        phases = PHASES
        if args.only:
            want = [s.strip() for s in args.only.split(",") if s.strip()]
            phases = [p for p in PHASES if any(w in p[0] for w in want)]
            if not phases:
                print(f"[sweep] --only {args.only!r} 没匹配到阶段")
                return 1
        print(f"[sweep] policy={os.environ.get('M20_POLICY_DIR', '<默认>')} "
              f"domain={os.environ['ROS_DOMAIN_ID']} viewer={args.viewer} "
              f"共 {len(phases)} 个阶段")
        for name, dur, vx, vy, wz, dh, dp, dr in phases:
            # 先把命令归零过渡一下：阶段之间直接换命令会让"稳态跟随"混进上一段的
            # 动量/瞬态，测出来的速度既不是上一段也不是这一段的。过渡段不采样。
            if args.settle > 0:
                target.update(vx=0.0, vy=0.0, wz=0.0, dh=0.0, dp=0.0, dr=0.0)
                time.sleep(args.settle)
            target.update(vx=vx, vy=vy, wz=wz, dh=dh, dp=dp, dr=dr)
            dur_s = dur * args.dur_scale
            t0 = last_sim_t(telemetry) or 0.0
            time.sleep(dur_s)
            t1 = last_sim_t(telemetry) or (t0 + dur_s)
            marks.append((name, t0, t1))
            print(f"  [{len(marks):2d}/{len(phases)}] {name:<12} "
                  f"vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} dh={dh:+.3f} "
                  f"dp={dp:+.2f} dr={dr:+.2f}  (sim {t0:.1f}~{t1:.1f} s)")

        press(b"r")                     # 阻尼收尾
        time.sleep(1.5)
    finally:
        try:
            if pub is not None:
                pass
        except Exception:
            pass
        killer(deploy)
        killer(sim)
        for f in (sim_f, dep_f):
            try:
                if f:
                    f.close()
            except Exception:
                pass
        for _ in range(100):
            procs = [p for p in (sim, deploy) if p is not None]
            if all(p.poll() is not None for p in procs):
                break
            time.sleep(0.1)
        for line in force_cleanup():
            print("  [cleanup] 强杀残留: " + line)
        time.sleep(0.5)

    if not telemetry.exists():
        print("[sweep] FAIL: 没有遥测文件")
        return 1
    rows = load_rows(telemetry)
    if len(rows) < 500:
        print(f"[sweep] FAIL: 遥测行数太少（{len(rows)}）")
        return 1
    info = analyse(rows, marks)

    print(f"\n[sweep] 遥测 {info['n']} 行，t_end {info['t_end']:.1f} s，"
          f"实时因子 {info['rtf']:.3f}" if info["rtf"] else "")
    print(f"{'阶段':<12}{'vx':>7}{'vx_meas':>9}{'vy':>7}{'vy_meas':>9}"
          f"{'wz':>7}{'wz_meas':>9}{'h_min':>8}{'h_mean':>8}{'tilt_max':>9}"
          f"{'yaw0°':>8}  判定")
    cmd = {p[0]: p for p in PHASES}

    fails: list[str] = []
    for ph in info["phases"]:
        if ph.get("empty"):
            print(f"{ph['name']:<12}  —— 数据不足（{ph['n']} 行）")
            continue
        c = cmd[ph["name"]]
        ok = ph["fell_at"] is None
        if not ok:
            fails.append(f"{ph['name']} 摔倒 @ t={ph['fell_at']:.2f}s")
        print(f"{ph['name']:<12}{c[2]:>7.2f}{ph['vx']:>9.2f}{c[3]:>7.2f}{ph['vy']:>9.2f}"
              f"{c[4]:>7.2f}{ph['wz']:>9.2f}{ph['h_min']:>8.3f}{ph['h_mean']:>8.3f}"
              f"{math.degrees(ph['tilt_max']):>8.1f}°"
              f"{ph.get('yaw0_deg', 0.0):>8.0f}  "
              f"{'OK' if ok else 'FELL'}")

    if info["rtf"] is not None and info["rtf"] < 0.95:
        fails.append(f"实时因子 {info['rtf']:.3f} < 0.95（仿真落后墙钟，结果不算数；见 DEF-034）")
    if info["t_end"] < sum(p[1] for p in phases) * 0.5:
        fails.append(f"仿真只跑到 {info['t_end']:.1f}s（阶段都没跑完）")

    print()
    if fails:
        print(f"[sweep] FAIL（{len(fails)} 项）：")
        for f in fails:
            print("  - " + f)
        return 1
    print("[sweep] PASS：所有阶段都没摔倒，实时因子达标")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeat", type=int, default=1, help="重复跑 N 遍并汇总失败率")
    ap.add_argument("--out", default="/tmp/m20_sweep", help="日志/遥测输出前缀")
    ap.add_argument("--viewer", action="store_true", help="开 MuJoCo 窗口（默认无头）")
    ap.add_argument("--only", default="",
                    help="只跑名字里含这些子串的阶段（逗号分隔，调试用）")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="阶段之间的归零过渡时长（s，0 = 不要过渡）")
    ap.add_argument("--dur-scale", type=float, default=1.0,
                    help="所有阶段时长 × 该系数（拉长看稳态/慢响应用；调试用）")
    args = ap.parse_args()

    if args.repeat > 1:
        fails = 0
        for i in range(args.repeat):
            print(f"--- repeat {i + 1}/{args.repeat} ---")
            r = subprocess.run([sys.executable, __file__] +
                               [a for a in sys.argv[1:]
                                if a != "--repeat" and a != str(args.repeat)],
                               env=dict(os.environ))
            if r.returncode != 0:
                fails += 1
        print(f"\n[sweep] 命令组合扫描重复 {args.repeat} 次：失败 {fails} 次"
              f"（失败率 {fails / args.repeat:.0%}）")
        return 1 if fails else 0

    if not RL_BIN.is_file():
        print(f"[sweep] FAIL: 没找到 {RL_BIN}（先 colcon build）")
        return 1
    return run_once(args)


if __name__ == "__main__":
    sys.exit(main())
