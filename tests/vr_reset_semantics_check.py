#!/usr/bin/env python3
"""VR 复位语义回归（DEF-046）。

**要保证的语义**（2026-10-13 操作员定义）：头显里按**左 X/Y（reset）**之后

  1. 机身姿态回默认（高度 0.513 / 俯仰 0 / 横滚 0，与键盘 `l` 同一口径）；
  2. 机械臂末端位姿回默认；
  3. 速度归 0；
  4. VR **交还控制权**（`/VR_TELEOP` 持续发全 0 + `active=0`，等价"持续给 0 指令"）。

再按**右 B** → 把那一刻的左右手柄位姿重新采成校准原点（干净的新零点）并重新接管。

**为什么会有这个用例**：旧实现里 `rl_control_state.hpp` 在 reset 边沿只把
`vr_anchor_valid_` 置 false，`ApplyVrCommand()` 的"重新锚定"分支取的是**当前指令值**
`uc->body_height`（本意是让按 B 不跳变）⇒ 复位前的机身偏移被固化成新零点，
机身永远回不到默认（现象：复位后机身保持在被抬/被压/倾斜的姿态）。

用例分两段：
  A. **设备级**（秒级、不需要 ROS 图）：直接构造 `Se2VRExtendedDevice`，
     按 B → 复位 → 再按 B，检查 `advance()` 的 active/指令/校准触发。
  B. **端到端**：sim + arm_controller + arm_teleop + rl_deploy（键盘管道喂 `z`/`c`）
     + 假头显按时间线发 `/VR_TELEOP`，断言：
       * 复位前：机身高度/俯仰 ≈ 基线 + 偏移、轮速命令非 0、臂末端目标被推出去；
       * 复位后：高度/俯仰回基线、轮速命令 ≈ 0、臂末端目标 = 默认位姿；
       * 再按 B：臂重新跟手、机身不跳；`rl_deploy` 日志里有新的复位打印。

用法（容器内，仓库根目录）::

    python3 tests/vr_reset_semantics_check.py            # 全跑（~40 s）
    python3 tests/vr_reset_semantics_check.py --unit-only
"""

from __future__ import annotations

import argparse
import csv
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
from sim2sim_smoke import (  # noqa: E402
    force_cleanup,
    killer,
    leftover_processes,
    quat_to_R,
    spawn,
    WHEEL_RADIUS,
)

SIM_DIR = REPO / "src/M20_sdk_deploy/interface/robot/simulation"
SIM = SIM_DIR / "mujoco_simulation_ros2.py"
ARM = SIM_DIR / "arm_controller.py"
ARM_TELEOP = SIM_DIR / "arm_teleop_node.py"
RL_DEPLOY_BIN = REPO / "install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy"

# ---- 时间线（相对"进 RL"那一刻的墙钟，秒）---------------------------------
T_B = 5.0            # 第一次按 B（激活 + 标定）
T_RESET = 13.0       # 按左 X/Y（复位）
T_B2 = 19.0          # 再按 B（重新标定并接管）
T_END = 24.0         # 假头显结束

# ---- 复位前施加的偏移（= 假头显直接发到 /VR_TELEOP 的量）------------------
BODY_H_OFF = 0.030   # 机身高度 [m]   -> [9]
BODY_P_OFF = 0.120   # 机身俯仰 [rad] -> [10]
VR_VX = 0.30         # 底盘前向速度 [m/s] -> [0]
ARM_DX = 0.050       # 臂 EE 沿半径前伸 [m] -> [3]
ARM_DZ = 0.030       # 臂 EE 抬升 [m]         -> [5]
ARM_DX2 = 0.030      # 第二次按 B 之后的臂偏移

# ---- 判据 -----------------------------------------------------------------
EE_DEFAULT_ROOT = (0.3492, 0.0, 0.4327)   # 默认关节姿态下 gripper_base 在 root 系
TOL_H = 0.008        # 机身高度容差 [m]
TOL_P = 0.020        # 机身俯仰容差 [rad]
TOL_EE = 0.006       # 臂末端目标容差 [m]


# ===========================================================================
# A. 设备级：复位 = 失活 + 全 0；再按 B = 重新标定并接管
# ===========================================================================
def check_device_semantics() -> list[str]:
    """直接构造设备对象，按 B / 复位 / 再按 B，检查 advance() 的对外语义。"""
    import importlib.util

    sys.path.insert(0, str(SIM_DIR))
    spec = importlib.util.spec_from_file_location(
        "vr_teleop_device_under_test", SIM_DIR / "vr_teleop_device.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod      # dataclass 装饰器要能在 sys.modules 里找到本模块
    spec.loader.exec_module(mod)

    fails: list[str] = []
    if mod.RESET_KEEPS_ACTIVE:
        fails.append("RESET_KEEPS_ACTIVE 默认不是 0（复位应交还控制权）")

    try:
        dev = mod.Se2VRExtendedDevice(mod.Se2VRExtendedCfg())
    except Exception as exc:      # 端口被占等（容器里一般不会）
        print(f"[vr-reset] 设备级检查跳过（构造失败：{type(exc).__name__}: {exc}）")
        return fails

    # ① 按 B：激活 + 两手都要求重标定 + 校准脉冲
    dev._on_button_press("b", "right")
    cmd13, ev = dev.advance()
    if not ev["active"]:
        fails.append("按 B 之后 active 不是 1")
    if not ev["calibrate"]:
        fails.append("按 B 之后没有 calibrate 脉冲")
    if not (dev._calibration_triggered["left"] and dev._calibration_triggered["right"]):
        fails.append("按 B 之后没有要求左右手都重标定")
    if any(abs(v) > 1e-12 for v in cmd13):
        fails.append(f"按 B 之后的指令不是全 0：{list(cmd13)}")

    # ② 让设备先积累一点"偏移"，再复位 —— 复位必须把它清掉
    dev._command[:] = 1.0
    dev._body_height_offset = 0.2
    dev._on_reset("fail")
    cmd13, ev = dev.advance()
    if ev["active"]:
        fails.append("复位之后 active 不是 0（没有交还控制权）")
    if not ev["reset_fail"]:
        fails.append("复位之后没有 reset 事件")
    if any(abs(v) > 1e-12 for v in cmd13):
        fails.append(f"复位之后仍在发非 0 指令：{list(cmd13)}")
    if dev._body_height_offset != 0.0:
        fails.append("复位之后机身高度积分量没清零")

    # ③ 复位后持续发 0（= 持续给 0 指令）
    for _ in range(3):
        cmd13, ev = dev.advance()
        if ev["active"] or any(abs(v) > 1e-12 for v in cmd13):
            fails.append("复位后的后续帧不是 (active=0, 全 0)")
            break

    # ④ 再按 B：重新激活 + 两手重新标定
    dev._on_button_press("b", "right")
    cmd13, ev = dev.advance()
    if not ev["active"]:
        fails.append("复位后再按 B 没有重新激活")
    if not (dev._calibration_triggered["left"] and dev._calibration_triggered["right"]):
        fails.append("复位后再按 B 没有把左右手位姿重新采成校准原点")

    dev._started = False
    if mod.RESET_KEEPS_ACTIVE:
        fails.append("RESET_KEEPS_ACTIVE 不是 0")
    print("[vr-reset] 设备级检查：%s" % ("PASS" if not fails else "FAIL"))
    return fails


# ===========================================================================
# B. 端到端：假头显 + sim + arm_controller + arm_teleop + rl_deploy
# ===========================================================================
class FakeHeadset:
    """按时间线发 /VR_TELEOP，同时把 (墙钟, 仿真时间, VR 帧, 臂链路) 记成 CSV。"""

    def __init__(self, out_csv: Path, t0: float, keep_active: bool = False):
        from vr_teleop_protocol import (VR_ACTIVE_IDX, VR_CALIBRATE_IDX,
                                        VR_RESET_IDX, VR_TELEOP_LEN)
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
        from drdds.msg import JointsData, JointsDataCmd

        self.VR_ACTIVE_IDX = VR_ACTIVE_IDX
        self.VR_CALIBRATE_IDX = VR_CALIBRATE_IDX
        self.VR_RESET_IDX = VR_RESET_IDX
        self.LEN = VR_TELEOP_LEN

        self.rclpy = rclpy
        self.Msg = Float32MultiArray
        self.t0 = t0
        self.keep_active = keep_active
        rclpy.init()
        self.node = Node("vr_reset_semantics_headset")
        self.pub = self.node.create_publisher(Float32MultiArray, "/VR_TELEOP", 10)
        self.f = open(out_csv, "w", buffering=1)
        cols = (["wall", "sim_t"] + [f"vr{i}" for i in range(self.LEN)]
                + ["at_mode", "at_dx", "at_dy", "at_dz"]
                + [f"cmd{i}" for i in range(6)]
                + ["st_x", "st_y", "st_z", "st_qw", "st_qx", "st_qy", "st_qz"]
                + [f"fb{i}" for i in range(7)])
        self.f.write(",".join(cols) + "\n")
        self.last_vr = [0.0] * self.LEN
        self.last_at = [0.0] * 4
        self.last_cmd = [0.0] * 6
        self.last_st = [0.0] * 7
        self.last_fb = [0.0] * 7
        self.sim_t = -1.0
        self.node.create_subscription(JointsData, "/ARM_JOINTS_DATA",
                                      self._fb_cb, 50)
        self.node.create_subscription(JointsDataCmd, "/ARM_JOINTS_CMD",
                                      self._cmd_cb, 50)
        self.node.create_subscription(Float32MultiArray, "/ARM_TELEOP_STATE",
                                      self._st_cb, 50)
        self.node.create_subscription(Float32MultiArray, "/ARM_TELEOP",
                                      self._at_cb, 50)
        self.node.create_timer(0.02, self._tick)     # 50 Hz：发 VR 帧
        self.node.create_timer(0.02, self._log)      # 50 Hz：落 CSV
        self._spin = threading.Thread(target=rclpy.spin, args=(self.node,),
                                      daemon=True)
        self._spin.start()

    # ---- 订阅回调 ----
    def _fb_cb(self, msg):
        self.sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_fb = [float(j.position) for j in msg.data.joints_data[:7]]

    def _cmd_cb(self, msg):
        self.last_cmd = [float(j.position) for j in msg.data.joints_data[:6]]

    def _st_cb(self, msg):
        self.last_st = [float(v) for v in msg.data[:7]]

    def _at_cb(self, msg):
        if len(msg.data) >= 4:
            self.last_at = [float(v) for v in msg.data[:4]]

    # ---- 时间线 ----
    def _frame(self, t: float) -> list[float]:
        msg = [0.0] * self.LEN
        if T_B <= t < T_RESET:
            msg[0] = VR_VX
            msg[3] = ARM_DX
            msg[5] = ARM_DZ
            msg[9] = BODY_H_OFF
            msg[10] = BODY_P_OFF
            msg[self.VR_ACTIVE_IDX] = 1.0
            if t < T_B + 0.03:                       # B 的 calibrate 脉冲（一拍）
                msg[self.VR_CALIBRATE_IDX] = 1.0
        elif T_RESET <= t < T_B2:
            # 复位帧：全 0 + reset=1（一拍），之后持续全 0。
            # active：默认语义交还控制权（0）；M20_VR_RESET_KEEPS_ACTIVE=1 时保持 1。
            if t < T_RESET + 0.03:
                msg[self.VR_RESET_IDX] = 1.0
            if self.keep_active:
                msg[self.VR_ACTIVE_IDX] = 1.0
        elif t >= T_B2:
            msg[3] = ARM_DX2
            msg[self.VR_ACTIVE_IDX] = 1.0
            if t < T_B2 + 0.03:
                msg[self.VR_CALIBRATE_IDX] = 1.0
        return msg

    def _tick(self):
        t = time.monotonic() - self.t0
        msg = self._frame(t)
        out = self.Msg()
        out.data = msg
        self.pub.publish(out)
        self.last_vr = msg

    def _log(self):
        t = time.monotonic() - self.t0
        row = ([f"{t:.4f}", f"{self.sim_t:.4f}"]
               + [f"{v:.6g}" for v in self.last_vr]
               + [f"{v:.6g}" for v in self.last_at]
               + [f"{v:.6g}" for v in self.last_cmd]
               + [f"{v:.6g}" for v in self.last_st]
               + [f"{v:.6g}" for v in self.last_fb])
        self.f.write(",".join(row) + "\n")

    def close(self):
        try:
            self.f.close()
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass


def analyse_telemetry(csv_path: Path):
    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        return None
    out = []
    for r in rows:
        base_z = float(r["base_z"])
        wheel_z = [float(r[k]) for k in
                   ("wheel_z_fl", "wheel_z_fr", "wheel_z_hl", "wheel_z_hr")]
        R = quat_to_R(*(float(r[k]) for k in
                        ("base_qw", "base_qx", "base_qy", "base_qz")))
        pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
        wcmd = [float(r[k]) for k in
                ("wcmd_fl", "wcmd_fr", "wcmd_hl", "wcmd_hr")]
        out.append({"t": float(r["t"]),
                    "height": base_z - sum(wheel_z) / 4.0 + WHEEL_RADIUS,
                    "pitch": pitch,
                    "wcmd": sum(wcmd) / 4.0})
    return out


def mean(seq):
    seq = list(seq)
    return sum(seq) / len(seq) if seq else float("nan")


def in_range(rows, key, lo, hi):
    return [r for r in rows if lo <= r[key] <= hi]


def run_e2e(scenario: str, out: Path, duration: float) -> tuple[list[str], bool]:
    """跑一遍端到端。

    scenario = `default`    ：复位交还控制权（RESET_KEEPS_ACTIVE=0，新默认）
    scenario = `keepactive` ：复位保持激活（=1，操作员 2026-10-10 的旧语义）——
                              **机身锚点被"归档"的那个 bug 就在这个分支**：
                              修复前复位后机身俯仰会停在被偏置的角度。

    返回 `(fails, entry_ok)`：`entry_ok=False` 表示这一跑踩到进 RL 的入口瞬态
    （DEF-018，机器人没站住）⇒ 判据不成立，应当重跑而不是判红。
    """
    fails: list[str] = []
    keep_active = scenario == "keepactive"
    print(f"\n[vr-reset] ==== 场景 {scenario} (M20_VR_RESET_KEEPS_ACTIVE="
          f"{'1' if keep_active else '0（默认）'}) ====")
    if keep_active:
        os.environ["M20_VR_RESET_KEEPS_ACTIVE"] = "1"    # 子进程继承
    else:
        os.environ.pop("M20_VR_RESET_KEEPS_ACTIVE", None)
    os.environ.setdefault("ROS_DOMAIN_ID", "1")
    telemetry = Path(str(out) + ".telemetry.csv")
    chain = Path(str(out) + ".chain.csv")
    for p in (telemetry, chain, Path(str(out) + ".sim.log"),
              Path(str(out) + ".arm.log"), Path(str(out) + ".deploy.log"),
              Path(str(out) + ".arm_teleop.log")):
        if p.exists():
            p.unlink()

    for _ in range(20):
        if not leftover_processes():
            break
        force_cleanup()
        time.sleep(0.5)
    if leftover_processes():
        print("[vr-reset] FAIL: 还有上一次的 sim/rl_deploy 在跑，先清掉")
        return ["还有上一次的 sim/rl_deploy 在跑（清理失败）"], True

    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "1")
    env["M20_USE_VIEWER"] = "0"
    env["M20_SIM_TELEMETRY"] = str(telemetry)
    env.setdefault("M20_JVEL_DEBUG", "0")
    env.setdefault("M20_ARM_DEBUG", "1")

    sim = arm = teleop = deploy = None
    headset = None
    logs = []
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

        deploy_cmd = ([str(RL_DEPLOY_BIN)] if RL_DEPLOY_BIN.is_file()
                      else ["ros2", "run", "m20_sdk_deploy", "rl_deploy"])
        deploy, dep_f = spawn(deploy_cmd, env, str(out) + ".deploy.log",
                              stdin=subprocess.PIPE)
        logs.append(dep_f)
        time.sleep(2.0)

        def press(key: bytes):
            try:
                deploy.stdin.write(key)
                deploy.stdin.flush()
            except (BrokenPipeError, ValueError):
                pass

        press(b"z")
        print("[vr-reset] sent 'z' (stand up)")
        time.sleep(4.0)
        press(b"c")
        print("[vr-reset] sent 'c' (RL control)")
        t0 = time.monotonic()                     # = 时间线原点
        # 立刻开始记录 sim_t（基线要用）；keep_active 决定复位后 active 是否仍为 1
        headset = FakeHeadset(chain, t0, keep_active=keep_active)
        time.sleep(duration + 2.0)
    finally:
        if headset is not None:
            headset.close()
        killer(deploy)
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
    with open(chain) as fh:
        rows = list(csv.DictReader(fh))
    if len(rows) < 100 or rows[0]["sim_t"] in ("", "-1.0000"):
        print(f"[vr-reset] FAIL: 假头显没采到数据（{len(rows)} 行）")
        return ["假头显没采到数据（sim 时间戳为空）"], True
    for r in rows:
        for k in r:
            r[k] = float(r[k])
    ws = [r for r in rows if r["sim_t"] > 0]
    if len(ws) < 100:
        print(f"[vr-reset] FAIL: /ARM_JOINTS_DATA 的仿真时间戳太少（{len(ws)}）")
        return ["/ARM_JOINTS_DATA 的仿真时间戳太少（臂反馈没来？）"], True

    def sim_at_wall(w):
        """用假头显自己记的 (wall, sim_t) 把墙钟换成仿真时间。"""
        prev = ws[0]
        for r in ws:
            if r["wall"] >= w:
                if r["wall"] == prev["wall"]:
                    return r["sim_t"]
                f = (w - prev["wall"]) / (r["wall"] - prev["wall"])
                return prev["sim_t"] + f * (r["sim_t"] - prev["sim_t"])
            prev = r
        return ws[-1]["sim_t"]

    tel = analyse_telemetry(telemetry)
    if not tel:
        print("[vr-reset] FAIL: 没有遥测数据")
        return ["没有遥测数据（仿真没起来？）"], True

    def tel_window(w0, w1):
        a, b = sim_at_wall(w0), sim_at_wall(w1)
        return in_range(tel, "t", a, b)

    def chain_window(w0, w1):
        return in_range(rows, "wall", w0, w1)

    # 基线：进 RL 之后、第一次按 B 之前（VR 不在场）= "默认姿态"的实测值。
    # ⚠️ 两个策略侧特性决定了判据口径（2026-10-13 实测）：
    #   * **机身高度命令跟踪很慢**：命令 +30 mm，8 s 只走到 +10 mm 且还在爬 ⇒
    #     高度只判"方向 + 量级下限"，严格判据放在俯仰通道与 rl_deploy 的日志上；
    #   * **零速有残余轮速指令**（命令 0 时实测 |wcmd| ≈ 1.4 rad/s）⇒
    #     "速度归 0"只能跟基线比。
    base_tel = tel_window(T_B - 1.5, T_B - 0.2)
    base = {"height": mean(r["height"] for r in base_tel),
            "pitch": mean(r["pitch"] for r in base_tel),
            "wcmd": mean(abs(r["wcmd"]) for r in base_tel)}
    print(f"[vr-reset] 基线（VR 未接管）：height {base['height']:.3f} m、"
          f"pitch {math.degrees(base['pitch']):+.1f}°、"
          f"|轮速指令| {base['wcmd']:.2f} rad/s（零速残余）")
    # 进 RL 的入口瞬态（DEF-018）会让机器人直接摔倒；这时"回默认"的判据不成立，
    # 应当重跑而不是判红（与 tests/run_all.sh 对 rl/walk 档的"按失败率判"同一口径）。
    if base["height"] < 0.30:
        print(f"[vr-reset] 基线不在站立状态（height {base['height']:.3f} m < 0.30）"
              "⇒ 进 RL 入口瞬态（DEF-018），本次不计")
        return [], False

    # ---- 复位前：VR 的机身/速度/臂偏移都应该生效 ----
    pre_tel = tel_window(T_B + 3.0, T_RESET - 0.5)
    pre_chain = chain_window(T_B + 3.0, T_RESET - 0.5)
    pre = {"height": mean(r["height"] for r in pre_tel),
           "pitch": mean(r["pitch"] for r in pre_tel),
           "wcmd": mean(abs(r["wcmd"]) for r in pre_tel),
           "ee_x": mean(r["st_x"] for r in pre_chain)}
    d_h, d_p = pre["height"] - base["height"], pre["pitch"] - base["pitch"]
    print(f"[vr-reset] 复位前：height {pre['height']:.3f} m（{d_h:+.3f}）"
          f"、pitch {math.degrees(pre['pitch']):+.1f}°（{math.degrees(d_p):+.1f}°）"
          f"、|轮速指令| {pre['wcmd']:.2f} rad/s"
          f"、臂 ee_x {pre['ee_x']:.4f} m（默认 {EE_DEFAULT_ROOT[0]:.4f}）")
    if abs(d_p - BODY_P_OFF) > TOL_P:
        fails.append(f"复位前的机身俯仰偏移没生效：实测 {d_p:+.4f} rad，"
                     f"期望 {BODY_P_OFF:+.3f}")
    if not (0.003 <= d_h <= BODY_H_OFF + 0.02):
        fails.append(f"复位前的机身高度偏移没生效：实测 {d_h:+.4f} m"
                     f"（期望正向、量级 ~{BODY_H_OFF:.3f}）")
    if pre["wcmd"] < max(0.5, 2.0 * base["wcmd"]):
        fails.append(f"复位前的速度指令没进去：|轮速指令| {pre['wcmd']:.3f} rad/s"
                     f"（基线 {base['wcmd']:.3f}）")
    if abs(pre["ee_x"] - (EE_DEFAULT_ROOT[0] + ARM_DX)) > 0.02:
        fails.append(f"复位前的臂目标不在预期位置：ee_x {pre['ee_x']:.4f} m，"
                     f"期望 ≈ {EE_DEFAULT_ROOT[0] + ARM_DX:.4f}")

    # ---- 复位后：机身/速度回默认、臂回默认位姿 ----
    post_tel = tel_window(T_RESET + 4.0, T_B2 - 1.0)
    post_chain = chain_window(T_RESET + 4.0, T_B2 - 1.0)
    post = {"height": mean(r["height"] for r in post_tel),
            "pitch": mean(r["pitch"] for r in post_tel),
            "wcmd": mean(abs(r["wcmd"]) for r in post_tel),
            "ee": [mean(r[k] for r in post_chain)
                   for k in ("st_x", "st_y", "st_z")]}
    print(f"[vr-reset] 复位后：height {post['height']:.3f} m（基线 "
          f"{post['height'] - base['height']:+.3f}）、"
          f"pitch {math.degrees(post['pitch']):+.1f}°（基线 "
          f"{math.degrees(post['pitch'] - base['pitch']):+.1f}°）、"
          f"|轮速指令| {post['wcmd']:.3f} rad/s、臂目标 "
          f"{[round(v, 4) for v in post['ee']]}")
    if abs(post["pitch"] - base["pitch"]) > TOL_P:
        fails.append("复位后机身俯仰没回默认：仍偏 "
                     f"{math.degrees(post['pitch'] - base['pitch']):+.1f}°")
    if abs(post["height"] - base["height"]) > 2 * TOL_H:
        fails.append(f"复位后机身高度没回默认：{post['height']:.4f} m vs 基线 "
                     f"{base['height']:.4f} m（差 {post['height'] - base['height']:+.4f}）")
    if post["wcmd"] > max(0.5, 1.5 * base["wcmd"]):
        fails.append(f"复位后速度没归 0：|轮速指令| {post['wcmd']:.3f} rad/s"
                     f"（基线 {base['wcmd']:.3f}）")
    for i, nm in enumerate("xyz"):
        if abs(post["ee"][i] - EE_DEFAULT_ROOT[i]) > TOL_EE:
            fails.append(f"复位后臂末端目标 {nm} 不是默认值：{post['ee'][i]:.4f} vs "
                         f"{EE_DEFAULT_ROOT[i]:.4f}")

    # ---- 再按 B：臂重新跟手、机身不跳 ----
    tail_tel = tel_window(T_B2 + 2.5, duration - 0.5)
    tail_chain = chain_window(T_B2 + 2.5, duration - 0.5)
    tail = {"height": mean(r["height"] for r in tail_tel),
            "pitch": mean(r["pitch"] for r in tail_tel),
            "ee_x": mean(r["st_x"] for r in tail_chain)}
    print(f"[vr-reset] 再按 B 之后：height {tail['height']:.3f} m、"
          f"pitch {math.degrees(tail['pitch']):+.1f}°、臂 ee_x {tail['ee_x']:.4f} m")
    if abs(tail["ee_x"] - (EE_DEFAULT_ROOT[0] + ARM_DX2)) > 0.02:
        fails.append(f"再按 B 之后臂没有重新跟手：ee_x {tail['ee_x']:.4f} m，"
                     f"期望 ≈ {EE_DEFAULT_ROOT[0] + ARM_DX2:.4f}")
    if abs(tail["pitch"] - base["pitch"]) > TOL_P:
        fails.append("再按 B 之后机身俯仰跳了："
                     f"{math.degrees(tail['pitch'] - base['pitch']):+.1f}°")
    if abs(tail["height"] - base["height"]) > 2 * TOL_H:
        fails.append(f"再按 B 之后机身高度跳了：{tail['height']:.4f} m vs 基线 "
                     f"{base['height']:.4f} m")
    if not (0.30 < tail["height"] < 0.70):
        fails.append(f"再按 B 之后机器人不在站立状态：height {tail['height']:.3f}")

    # ---- 日志断言 ----
    deploy_log = Path(str(out) + ".deploy.log").read_text(errors="ignore")
    if "[VR] RESET -> 机身姿态回默认" not in deploy_log:
        fails.append("rl_deploy 日志里没有 `[VR] RESET -> 机身姿态回默认`"
                     "（新复位分支没走到）")
    if "[RESET] 外部源（VR）交还控制权" not in deploy_log:
        fails.append("rl_deploy 日志里没有键盘归位打印 `[RESET] 外部源（VR）交还控制权`")
    # 关键回归：**任何一次重锚都不能把偏移固化成新零点**。修复前在"保持激活"模式下
    # 复位会打一条 `[VR] body anchor = 0.543 / 0.12 / 0`，这里直接把它挡掉。
    anchors = re.findall(r"\[VR\] body anchor = ([-\d.]+) / ([-\d.]+) / ([-\d.]+)",
                         deploy_log)
    for a_h, a_p, a_r in anchors:
        if abs(float(a_h) - 0.513) > 0.005 or abs(float(a_p)) > 0.005 \
                or abs(float(a_r)) > 0.005:
            fails.append(f"机身锚点被固化成非默认值：anchor = {a_h} / {a_p} / {a_r}"
                         "（复位把偏移当成了新零点）")
    print(f"[vr-reset] 机身锚点记录：{anchors if anchors else '（无）'}")
    arm_teleop_log = Path(str(out) + ".arm_teleop.log").read_text(errors="ignore")
    if "/reset_sim 完成" not in arm_teleop_log:
        fails.append("arm_teleop 侧没有走 /reset_sim（臂/腿没复位）")

    print("[vr-reset] 场景 %s：%s" % (scenario, "PASS" if not fails else "FAIL"))
    for f in fails:
        print("  ✗ " + f)
    return fails, True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/m20_vr_reset")
    ap.add_argument("--unit-only", action="store_true")
    ap.add_argument("--scenario", choices=("default", "keepactive", "both"),
                    default="default",
                    help="default=复位交还控制权（新默认）；keepactive=复位保持激活"
                         "（旧语义，机身锚点那个 bug 就在这个分支）；both=都跑")
    ap.add_argument("--duration", type=float, default=T_END,
                    help="假头显时长（默认到 T_END）")
    ap.add_argument("--attempts", type=int, default=3,
                    help="进 RL 入口瞬态（DEF-018）摔倒时最多重跑几次")
    args = ap.parse_args()

    fails = check_device_semantics()
    if args.unit_only:
        for f in fails:
            print("  ✗ " + f)
        return 1 if fails else 0

    scenarios = (["default", "keepactive"] if args.scenario == "both"
                 else [args.scenario])
    for sc in scenarios:
        out = Path(args.out + ("" if sc == "default" else "_" + sc))
        invalid = 0
        ok = False
        for attempt in range(args.attempts):
            fs, entry_ok = run_e2e(sc, out, args.duration)
            if not entry_ok:
                invalid += 1
                print(f"[vr-reset] 场景 {sc} 第 {attempt + 1}/{args.attempts} 次："
                      "入口瞬态（DEF-018）摔倒，本次结果不计，重跑")
                continue
            fails += fs
            ok = not fs
            break
        if not ok and invalid >= args.attempts:
            fails.append(f"场景 {sc}：{args.attempts} 次都踩在进 RL 的入口瞬态上"
                         "（DEF-018），无法判定 —— 过一会儿重跑")

    print()
    if fails:
        print("[vr-reset] FAIL（共 %d 条）" % len(fails))
        return 1
    print("[vr-reset] PASS（复位：机身/臂/速度回默认 + 交还控制权；再按 B 重新标定并接管）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
