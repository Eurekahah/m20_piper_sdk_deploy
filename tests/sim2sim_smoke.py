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
  --mode arm     额外起 arm_controller（IK），验证机械臂保持在默认位姿且不扰动底盘
  --mode push    进 RL 后由仿真施加一次侧向力，验证安全接管会触发并切到 joint_damping
                 （力/时刻/时长用 M20_SIM_PUSH_FORCE / _AT / _DURATION，默认 800 N / 12 s / 0.3 s）

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
ARM = REPO / "src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py"
CANDIDATE_MODES = ("rl", "walk", "arm")
WHEEL_RADIUS = 0.09
TILT_LIMIT = 0.8          # rad
HEIGHT_LIMIT = 0.30       # m
ARM_DEFAULT = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0]   # arm1..6 + gripper1/2
ARM_TAU_LIMIT = 100.0     # N·m（训练 effort_limit）


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
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass


def leftover_processes() -> list[str]:
    """上一次跑剩下的 sim/rl_deploy。仿真的控制循环是**墙钟驱动**的：
    同机并发跑两个实例会让 1 ms 控制周期被拉长，结果不可复现（实测出现过
    同一配置一次 PASS 一次 FAIL），所以这里直接拒绝开跑。"""
    out = subprocess.run(["pgrep", "-af", "mujoco_simulation_ros2.py|rl_deploy"],
                         capture_output=True, text=True)
    bad = []
    for line in out.stdout.splitlines():
        if not line.strip():
            continue
        # 排除"匹配到自己"的几种情况：pgrep 自身、包着本脚本的 shell、本脚本
        if ("pgrep" in line or "sim2sim_smoke.py" in line
                or "bash -lc" in line or "sh -c" in line):
            continue
        bad.append(line)
    return bad


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
    # 行走判据用 base_x 的"后半段平均速度"（t 是仿真时间）
    j = int(n * 0.6)
    out["vx_tail"] = (float(rows[-1]["base_x"]) - float(rows[j]["base_x"])) / \
                     max(out["t"][-1] - out["t"][j], 1e-6)
    # 机械臂：稳态偏差（相对默认角）与最大力矩；tau 列同样是 MJCF 序（16..23）
    arm_dev = 0.0
    arm_tau = 0.0
    for r in rows[j:]:
        for k, name_i in enumerate(range(16, 24)):
            arm_dev = max(arm_dev, abs(float(r[f"q{name_i}"]) - ARM_DEFAULT[k]))
            arm_tau = max(arm_tau, abs(float(r[f"tau{name_i}"])))
    out["arm_dev"] = arm_dev
    out["arm_tau_max"] = arm_tau
    out["fell_at"] = next((out["t"][i] for i in range(n)
                           if out["tilt"][i] > TILT_LIMIT or out["height"][i] < HEIGHT_LIMIT), None)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["hold", "stand", "rl", "walk", "arm", "push"],
                    default="rl")
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

    stale = leftover_processes()
    if stale:
        print("[smoke] FAIL: 上一次的进程还在跑，先清掉再来（否则控制周期会被拉长、结果不可复现）:")
        for line in stale:
            print("  " + line)
        return 1

    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "1")
    env["M20_USE_VIEWER"] = "1" if args.viewer else "0"
    env["M20_SIM_TELEMETRY"] = str(telemetry)
    env.setdefault("M20_JVEL_DEBUG", "0")
    if args.mode == "push":
        env.setdefault("M20_SIM_PUSH_FORCE", "800")
        env.setdefault("M20_SIM_PUSH_AT", "12")
        env.setdefault("M20_SIM_PUSH_DURATION", "0.3")

    sim = deploy = arm = None
    sim_f = dep_f = arm_f = None
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
            if args.mode == "arm":
                # IK 节点：它会把 /ARM_JOINTS_CMD 与 /ARM_TELEOP_STATE 接起来，
                # 也就是策略观测里 ee_goal 的来源（P0-4 的路径）
                arm_log = out_prefix.with_suffix(".arm.log")
                arm, arm_f = spawn([sys.executable, str(ARM)], env, arm_log)
                time.sleep(1.0)
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
        killer(arm)
        killer(sim)
        for f in (sim_f, dep_f, arm_f):
            try:
                if f:
                    f.close()
            except Exception:
                pass
        # 等进程真的退干净，避免下一个用例被上一个的残留影响
        for _ in range(50):
            procs = [p for p in (sim, deploy, arm) if p is not None]
            if all(p.poll() is not None for p in procs):
                break
            time.sleep(0.1)
        # 再看一眼有没有残留（zombie 的 poll() 也可能已经返回 None/0）
        for _ in range(150):
            if not leftover_processes():
                break
            time.sleep(0.1)
        else:
            print("[smoke] 警告：仍有 sim/rl_deploy 进程残留，下一个用例可能受影响：")
            for line in leftover_processes():
                print("  " + line)
        time.sleep(1.0)

    info = analyse(telemetry)
    if info is None:
        print(f"[smoke] FAIL: no telemetry rows in {telemetry}")
        return 1

    print(f"\n[smoke] telemetry: {info['n']} rows, t_end = {info['t_end']:.2f} s")
    print(f"  height  : min {info['height_min']:.3f} m, 后半段均值 {info['height_tail_mean']:.3f} m")
    print(f"  tilt    : max {math.degrees(info['tilt_max']):.1f}°, "
          f"后半段 max {math.degrees(info['tilt_max_tail']):.1f}°")
    if args.mode == "walk":
        print(f"  vx(后半段): {info['vx_tail']:+.3f} m/s  （键盘命令 +0.7 m/s）")
    if args.mode == "arm":
        print(f"  arm     : 相对默认角最大偏差 {info['arm_dev']:.4f} rad, "
              f"最大关节力矩 {info['arm_tau_max']:.1f} N·m（限幅 {ARM_TAU_LIMIT}）")

    fails = []
    if args.mode != "hold":
        need_rows = int(max(args.duration - 4.0, 5.0) * 100)
        if info["n"] < need_rows:
            fails.append(f"遥测行数太少（{info['n']} < {need_rows}）：仿真或 rl_deploy 提前挂了")
        # rl_deploy 真的活着并进过 RL 吗？日志里必须有这两样东西。
        dep = deploy_log.read_text(errors="ignore") if deploy_log.exists() else ""
        if "Segmentation fault" in dep or "Traceback" in dep or "Aborted" in dep:
            fails.append("rl_deploy 崩了（日志里有 Segmentation fault/Traceback/Aborted）")
        if "M20PiperPolicyRunner" not in dep:
            fails.append("rl_deploy 日志里没有 M20PiperPolicyRunner：策略 runner 没起来")
        if "rl_control" not in dep:
            fails.append("rl_deploy 没有进入 rl_control 状态：策略从没被执行过")
    if info["t_end"] < args.duration * 0.8:
        fails.append(f"遥测只录到 {info['t_end']:.1f}s（期望 ~{args.duration}s）")
    if info["fell_at"] is not None and args.mode != "push":
        fails.append(f"触发摔倒判据 @ t={info['fell_at']:.2f}s "
                     f"(tilt>{math.degrees(TILT_LIMIT):.0f}° 或 height<{HEIGHT_LIMIT} m)")
    if args.mode == "walk":
        # 走起来才算过：命令 0.7 m/s，实测应在合理区间，方向为 +x
        if not (0.3 <= info["vx_tail"] <= 1.2):
            fails.append(f"walk 模式的平均前进速度 {info['vx_tail']:+.3f} m/s 不在 "
                         f"[0.3, 1.2]（键盘命令 +0.7 m/s）")
    if args.mode == "arm":
        # IK 节点把臂保持在默认位姿（默认目标就是默认关节角），力矩不超限
        if info["arm_dev"] > 0.1:
            fails.append(f"arm 模式的臂偏离默认位姿 {info['arm_dev']:.4f} rad > 0.1")
        if info["arm_tau_max"] >= ARM_TAU_LIMIT:
            fails.append(f"arm 模式的臂关节力矩 {info['arm_tau_max']:.1f} N·m 触到限幅")
    if args.mode == "push":
        # 这里**期望**摔（被打倒），要验的是"安全接管触发并切进 joint_damping"
        dep_txt = deploy_log.read_text(errors="ignore") if deploy_log.exists() else ""
        if "[TAKEOVER!]" not in dep_txt:
            fails.append("push 模式：没有看到 [TAKEOVER!]（倾角/折叠阈值没触发安全接管）")
        if "joint_damping" not in dep_txt:
            fails.append("push 模式：状态机没有切到 joint_damping")
        else:
            line = next((l.strip() for l in dep_txt.splitlines()
                         if l.startswith("[TAKEOVER!]")), "(no event)")
            print("  安全接管: " + line)

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
