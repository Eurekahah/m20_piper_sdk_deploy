#!/usr/bin/env python3
"""录 `/JOINTS_DATA`（16 个腿/轮关节的 position / velocity / torque / **两个温度**）
到 CSV —— 用来核对真机上"某个关节长期顶力矩 ⇒ 过温"这类问题（DEF-059 / TODO P1-8）。

为什么需要它：仿真侧**没有电机热模型**（`mujoco_simulation_ros2.py` 把
`motion_temp/driver_temp` 写死 40/45 ℃），所以真机的温度现象只能真机录。
本脚本同时录 `torque` 与 `motion_temp`（电机）/ `driver_temp`（驱动器）**两个**温度，
因为 `SafeController::IsMotorTempertureNormal()` 判的是 `motion_temp`，两者混淆会判错病因。

用法（**在 M20 上、与跑 `rl_deploy` 同一个终端环境里**）::

    cd ~/M20_sdk_deploy
    source /opt/ros/foxy/setup.bash
    source /opt/robot/scripts/setup_ros2.sh
    python3 src/M20_sdk_deploy/scripts/record_joint_data.py --duration 600

* `--duration 0` = 一直录到 Ctrl+C；
* 落盘：`--out`（默认 `~/joint_logs/joints_<时间戳>.csv`）+ 同名 `.meta.json`
  （记录关节名与话题，离线分析用）；
* 每 `--print-every` 秒在终端打印"最热的 3 个关节 + 力矩最大的关节"，
  方便边跑边看有没有在升温；
* 退出时打印每个关节的 τ 均值/极值与温度极值。

离线分析（把 CSV 拷回开发机后）::

    python3 src/M20_sdk_deploy/scripts/joint_log_report.py <csv>

注意：订阅用 **best-effort**（`qos_profile_sensor_data`）—— 这样对 reliable 与
best-effort 的发布者都收得到（反过来 reliable 订阅者遇到 best-effort 发布者会收不到，
见场景 5 的同类坑）。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    from drdds.msg import JointsData
except Exception as exc:  # pragma: no cover - 真机上环境没 source 对时会走到这里
    sys.stderr.write(
        "[err] 不能 import drdds.msg：{}\n"
        "      请在**与跑 rl_deploy 相同的环境**里运行：\n"
        "        source /opt/ros/foxy/setup.bash\n"
        "        source /opt/robot/scripts/setup_ros2.sh\n"
        "      （若是自己的 workspace）再 source install/setup.bash\n".format(exc))
    raise SystemExit(2)

# 机器人 /JOINTS_DATA 的关节顺序 = MJCF 序（每腿 hipx/hipy/knee/wheel 连续）。
# 消息里的 `name` 只有 4 个 char，真机可能给有意义的名字；为空时用这张表兜底。
DEFAULT_NAMES = [f"{leg}_{j}" for leg in ("fl", "fr", "hl", "hr")
                 for j in ("hipx", "hipy", "knee", "wheel")]
DOF = 16


def _decode_name(raw) -> str:
    try:
        s = bytes(raw).split(b"\x00")[0].decode("ascii", "replace").strip()
    except Exception:
        s = ""
    return s


class JointRecorder(Node):
    def __init__(self, out_path: str, topic: str, with_imu: bool):
        super().__init__("m20_joint_recorder")
        self.out_path = out_path
        self.names: list = [""] * DOF
        self.n = 0
        self.t_first = None
        self.t_last = None
        self.wall0 = time.monotonic()
        # 每关节累计统计（不依赖 numpy，真机 Python 3.8 也能跑）
        self.tau_sum = [0.0] * DOF
        self.tau_min = [float("inf")] * DOF
        self.tau_max = [float("-inf")] * DOF
        self.tau_sum2 = [0.0] * DOF
        self.mt_max = [float("-inf")] * DOF
        self.mt_min = [float("inf")] * DOF
        self.dt_max = [float("-inf")] * DOF
        self.last_mt = [float("nan")] * DOF
        self.last_dt = [float("nan")] * DOF
        self.over_100 = {i: [] for i in range(DOF)}   # 关节 -> [(起, 止), ...]
        self._over_start = {}
        self.imu = None

        cols = (["t", "wall"] + [f"pos{i}" for i in range(DOF)]
                + [f"vel{i}" for i in range(DOF)] + [f"tau{i}" for i in range(DOF)]
                + [f"mtemp{i}" for i in range(DOF)] + [f"dtemp{i}" for i in range(DOF)]
                + [f"sw{i}" for i in range(DOF)]
                + (["imu_roll", "imu_pitch", "imu_yaw"] if with_imu else []))
        self.cols = cols
        self.f = open(out_path, "w", buffering=1)
        self.w = csv.writer(self.f)
        self.w.writerow(cols)

        self.create_subscription(JointsData, topic, self._cb, qos_profile_sensor_data)
        if with_imu:
            from drdds.msg import ImuData
            self.create_subscription(ImuData, "/IMU_DATA", self._imu_cb,
                                     qos_profile_sensor_data)
        self.get_logger().info(f"订阅 {topic}（best-effort），落盘 {out_path}")

    def _imu_cb(self, msg):
        self.imu = (float(msg.data.roll), float(msg.data.pitch), float(msg.data.yaw))

    def _stamp(self, msg):
        try:
            s = msg.header.stamp
            t = float(s.sec) + float(s.nanosec) * 1e-9
            return t if t > 0 else float("nan")
        except Exception:
            return float("nan")

    def _cb(self, msg):
        joints = msg.data.joints_data
        if len(joints) < DOF:
            return
        ts = self._stamp(msg)
        wall = time.monotonic() - self.wall0
        t = ts if ts == ts else wall      # 真机的 stamp 可能是 0 ⇒ 退回墙钟
        if self.t_first is None:
            self.t_first = t
        self.t_last = t
        row = [t, wall]
        pos = [0.0] * DOF
        vel = [0.0] * DOF
        tau = [0.0] * DOF
        mt = [0.0] * DOF
        dt = [0.0] * DOF
        sw = [0] * DOF
        for i in range(DOF):
            j = joints[i]
            if not self.n:
                self.names[i] = _decode_name(j.name) or DEFAULT_NAMES[i]
            pos[i] = float(j.position)
            vel[i] = float(j.velocity)
            tau[i] = float(j.torque)
            mt[i] = float(j.motion_temp)
            dt[i] = float(j.driver_temp)
            sw[i] = int(j.status_word)
            self.tau_sum[i] += tau[i]
            self.tau_sum2[i] += tau[i] * tau[i]
            self.tau_min[i] = min(self.tau_min[i], tau[i])
            self.tau_max[i] = max(self.tau_max[i], tau[i])
            self.mt_max[i] = max(self.mt_max[i], mt[i])
            self.mt_min[i] = min(self.mt_min[i], mt[i])
            self.dt_max[i] = max(self.dt_max[i], dt[i])
            self.last_mt[i] = mt[i]
            self.last_dt[i] = dt[i]
            # 过 100 ℃ 的区段（诊断"保护 → 趴下 → 恢复"的时序）
            if mt[i] > 100.0 and i not in self._over_start:
                self._over_start[i] = wall
            elif mt[i] <= 100.0 and i in self._over_start:
                self.over_100[i].append((self._over_start.pop(i), wall))
        row += pos + vel + tau + mt + dt + sw
        if self.imu is not None and "imu_roll" in self.cols:
            row += list(self.imu)
        elif "imu_roll" in self.cols:
            row += [float("nan")] * 3
        self.w.writerow(row)
        self.n += 1

    # ---- 打印 ----
    def live_line(self) -> str:
        hot = sorted(range(DOF), key=lambda i: -self.last_mt[i])[:3]
        hot_s = " ".join(f"{self.names[i]} {self.last_mt[i]:.1f}(drv {self.last_dt[i]:.1f})"
                         for i in hot)
        hz = self.n / max(time.monotonic() - self.wall0, 1e-9)
        return (f"[{time.monotonic() - self.wall0:7.1f}s] {hz:6.1f} Hz  n={self.n:<7d}"
                f" 最热: {hot_s}")

    def summary(self) -> str:
        out = []
        n = max(self.n, 1)
        out.append("\n== 每个关节（整段）==")
        out.append(f"{'joint':>10} {'tau均值':>9} {'tau极值':>17} "
                   f"{'m_temp 首/末/峰':>22} {'d_temp峰值':>10}")
        for i in range(DOF):
            mean = self.tau_sum[i] / n
            out.append(f"{self.names[i]:>10} {mean:9.3f} "
                       f"{self.tau_min[i]:9.2f}/{self.tau_max[i]:9.2f} "
                       f"{'':>4}{self.mt_min[i]:7.1f}/{self.last_mt[i]:6.1f}/{self.mt_max[i]:6.1f} "
                       f"{self.dt_max[i]:10.1f}")
        for i in range(DOF):
            segs = list(self.over_100[i])
            if i in self._over_start:
                segs.append((self._over_start[i], float("nan")))
            if segs:
                txt = ", ".join(f"{a:.1f}s→{'未恢复' if b != b else '%.1fs' % b}"
                                for a, b in segs)
                out.append(f"  ⚠️ {self.names[i]} motion_temp > 100 ℃ 的区段: {txt}")
        hottest = max(range(DOF), key=lambda i: self.mt_max[i])
        o = self.over_100[hottest]
        out.append(f"  最热关节: {self.names[hottest]} 峰 {self.mt_max[hottest]:.1f} ℃"
                   f"（driver 峰 {self.dt_max[hottest]:.1f} ℃）")
        if o:
            a, b = o[0]
            out.append(f"    第一次 > 100 ℃: {a:.1f}s，持续 "
                       f"{'未恢复' if b != b else '%.1f s' % (b - a)}")
        return "\n".join(out)

    def close(self):
        self.f.flush()
        self.f.close()
        meta = {
            "topic": "/JOINTS_DATA",
            "joint_names": self.names,
            "joint_order": "MJCF 序（每腿 hipx/hipy/knee/wheel 连续）",
            "fields": "pos/vel/tau/mtemp(电机)/dtemp(驱动器)/sw，索引即本表下标",
            "rows": self.n, "seconds": round(time.monotonic() - self.wall0, 2),
            "started": datetime.now().isoformat(timespec="seconds"),
        }
        with open(self.out_path.replace(".csv", ".meta.json"), "w") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)


def main() -> int:
    ap = argparse.ArgumentParser(description="录 /JOINTS_DATA 到 CSV（含两个温度）")
    ap.add_argument("--out", default="", help="输出 CSV（默认 ~/joint_logs/joints_<时间戳>.csv）")
    ap.add_argument("--topic", default="/JOINTS_DATA")
    ap.add_argument("--duration", type=float, default=600.0,
                    help="录多少秒；0 = 一直录到 Ctrl+C（默认 600）")
    ap.add_argument("--print-every", type=float, default=5.0, help="终端概览间隔（秒）")
    ap.add_argument("--no-imu", action="store_true", help="不录 /IMU_DATA（默认录，用于看跌倒/倾角）")
    args = ap.parse_args()

    out = args.out or os.path.join(
        os.path.expanduser("~"), "joint_logs",
        f"joints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    rclpy.init()
    node = JointRecorder(out, args.topic, not args.no_imu)
    t0 = time.monotonic()
    next_print = t0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_print:
                print(node.live_line(), flush=True)
                next_print = time.monotonic() + args.print_every
            if args.duration and time.monotonic() - t0 >= args.duration:
                break
    except KeyboardInterrupt:
        print("\n[rec] Ctrl+C，收尾…", flush=True)
    finally:
        print(node.summary(), flush=True)
        print(f"[rec] {node.n} 行 -> {out}"
              f"（+ {os.path.basename(out).replace('.csv', '.meta.json')}）", flush=True)
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    if node.n == 0:
        print("[rec] ⚠️ 一帧都没收到：先确认 `ros2 topic hz /JOINTS_DATA` 有数据、"
              "domain/RMW 与发布者一致", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
