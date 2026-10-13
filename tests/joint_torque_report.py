#!/usr/bin/env python3
"""离线分析 `M20_SIM_TELEMETRY` CSV 里的**腿关节力矩**（定位"某个关节长期顶力矩"这类
发热/饱和问题）。

为什么需要它：仿真侧**没有电机热模型**（`mujoco_simulation_ros2.py` 里
`motion_temp/driver_temp` 是写死的 40/45 ℃），所以真机上的"过温 → 安全接管 → 趴下"
在仿真里永远不会出现。能观察的等价量是**关节力矩**：关节角速度≈0 时，
电机电流 ≈ τ / k_t，铜损 ∝ τ² ⇒ `mean(τ²)` 就是"谁最热"的一阶代理量。

用法（容器内，仓库根目录）::

    M20_SIM_TELEMETRY=/tmp/x.csv python3 tests/sim2sim_smoke.py --mode rl --duration 60
    python3 tests/joint_torque_report.py /tmp/x.csv --t0 25       # 只看稳态
    python3 tests/joint_torque_report.py /tmp/x.csv --ts          # 打 1 Hz 时间序列

列口径（别用错）：CSV 的 `q/dq/tau` 24 列是 **MJCF 序**，每腿 `hipx/hipy/knee/wheel`
连续（`fl fr hl hr`），后面接 `arm_joint1..6 + gripper_joint1/2`。
**不是**策略观测的 articulation 原生序，也不是动作序（见
`docs/sim2sim_layout_contract_zh.md` §3）。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

LEGS = ["fl", "fr", "hl", "hr"]
JOINTS = ["hipx", "hipy", "knee", "wheel"]
# 侧摆关节 = hipx，MJCF 序 0/4/8/12（左前 = 0）
HIPX_COLS = [0, 4, 8, 12]
LEG_TORQUE_LIMIT = 76.4      # N·m，MJCF leg_motor ctrlrange
WHEEL_RADIUS = 0.09          # m，与 sim2sim_smoke.py 的 height 口径一致


def load(path: Path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    q = np.array([[float(r[f"q{i}"]) for i in range(24)] for r in rows])
    dq = np.array([[float(r[f"dq{i}"]) for i in range(24)] for r in rows])
    tau = np.array([[float(r[f"tau{i}"]) for i in range(24)] for r in rows])
    wf = np.array([[float(r[k]) for k in
                    ("wheel_f_fl", "wheel_f_fr", "wheel_f_hl", "wheel_f_hr")] for r in rows])
    h = np.array([float(r["base_z"])
                  - 0.25 * sum(float(r[k]) for k in
                               ("wheel_z_fl", "wheel_z_fr", "wheel_z_hl", "wheel_z_hr"))
                  + WHEEL_RADIUS for r in rows])
    return t, q, dq, tau, wf, h


def band_rms(x: np.ndarray, fs: float, lo: float, hi: float) -> float:
    """x 在 [lo, hi) Hz 的幅值 RMS（去均值 + Hann 窗）。"""
    x = x - x.mean()
    n = len(x)
    if n < 16:
        return float("nan")
    w = np.hanning(n)
    spec = np.fft.rfft(x * w)
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    scale = 2.0 / w.sum()
    m = (freqs >= lo) & (freqs < hi)
    return float(np.sqrt(np.sum((np.abs(spec[m]) * scale) ** 2) / 2.0))


def dump_timeseries(t, q, tau, wf, h):
    step = max(int(round(1.0 / np.median(np.diff(t)))), 1)
    print("t[s]  tau_hipx[fl fr hl hr]      q_hipx[fl fr hl hr]      "
          "wheelF[fl fr hl hr]      height")
    for i in range(0, len(t), step):
        print(f"{t[i]:6.2f}  " + " ".join(f"{tau[i, c]:+7.2f}" for c in HIPX_COLS)
              + "  " + " ".join(f"{q[i, c]:+.4f}" for c in HIPX_COLS)
              + "  " + " ".join(f"{v:6.1f}" for v in wf[i])
              + f"  {h[i]:.4f}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path, help="M20_SIM_TELEMETRY 写出的遥测 CSV")
    ap.add_argument("--t0", type=float, default=15.0, help="稳态窗口起点（s，默认 15）")
    ap.add_argument("--t1", type=float, default=1e9, help="稳态窗口终点（s，默认到末尾）")
    ap.add_argument("--ts", action="store_true", help="只打 1 Hz 时间序列（不打印统计表）")
    args = ap.parse_args()

    t, q, dq, tau, wf, h = load(args.csv)
    sel = (t >= args.t0) & (t <= args.t1)
    if sel.sum() < 16:
        print(f"[err] 窗口内样本太少（{sel.sum()} 行），检查 --t0/--t1")
        return 1
    if args.ts:
        dump_timeseries(t, q, tau, wf, h)
        return 0

    fs = 1.0 / float(np.median(np.diff(t)))
    print(f"{args.csv}: {len(t)} 行, t {t[0]:.2f}~{t[-1]:.2f} s, fs {fs:.0f} Hz, "
          f"稳态窗 {t[sel][0]:.2f}~{t[sel][-1]:.2f} s（{sel.sum()} 样本）")
    print(f"高度(稳态) 均值 {h[sel].mean():.3f} m / 最低 {h[sel].min():.3f} m")
    print(f"轮接触力 均值 [fl fr hl hr] {wf[sel].mean(axis=0).round(1)} N"
          f"  ⇒ 左/右 {wf[sel][:, 0::2].mean():.1f}/{wf[sel][:, 1::2].mean():.1f} N,"
          f" 前/后 {wf[sel][:, :2].mean():.1f}/{wf[sel][:, 2:].mean():.1f} N")

    print("\n== 腿关节力矩（稳态窗；heat 列 = mean(τ²)，与电机铜损同量纲）==")
    print(f"{'joint':>9} {'mean':>9} {'std':>8} {'min':>9} {'max':>9} "
          f"{'|max|':>8} {'rms':>8} {'heat':>10}")
    for leg in range(4):
        for j in range(4):
            x = tau[sel, leg * 4 + j]
            print(f"{LEGS[leg] + '_' + JOINTS[j]:>9} {x.mean():9.3f} {x.std():8.3f} "
                  f"{x.min():9.3f} {x.max():9.3f} {np.abs(x).max():8.3f} "
                  f"{math.sqrt((x ** 2).mean()):8.3f} {(x ** 2).mean():10.4f}")

    heat = [(f"{LEGS[c // 4]}_{JOINTS[c % 4]}", (tau[sel, c] ** 2).mean())
            for c in range(16)]
    heat.sort(key=lambda kv: -kv[1])
    print("\n== 热负荷排序（前 5）==")
    for name, val in heat[:5]:
        print(f"  {name:>9}: {val:9.2f}   (τ_rms {math.sqrt(val):6.2f} N·m)")

    print("\n== 四个侧摆(hipx)关节对比 ==")
    for leg, c in enumerate(HIPX_COLS):
        x = tau[sel, c]
        print(f"{LEGS[leg] + '_hipx':>9}: q {q[sel, c].mean():+.4f} rad | "
              f"τ 均值 {x.mean():+.3f} std {x.std():.3f} |max| {np.abs(x).max():.3f} "
              f"rms {math.sqrt((x ** 2).mean()):.3f} heat {(x ** 2).mean():.2f}")

    print("\n== hipx 频谱（看有没有高频抖动，会额外发热）==")
    for leg, c in enumerate(HIPX_COLS):
        x = tau[sel, c]
        bands = "  ".join(f"{lo}-{hi}Hz {band_rms(x, fs, lo, hi):.4f}"
                          for lo, hi in ((0, 5), (5, 50), (50, 200), (200, 600)))
        print(f"{LEGS[leg] + '_hipx':>9}: {bands}")

    print(f"\n== 饱和检查（限幅 ±{LEG_TORQUE_LIMIT} N·m）==")
    for leg, c in enumerate(HIPX_COLS):
        x = np.abs(tau[sel, c])
        print(f"{LEGS[leg] + '_hipx':>9}: >0.9·限幅 {(x > 0.9 * LEG_TORQUE_LIMIT).sum()}"
              f" 样本, >20 N·m {(x > 20).sum()} 样本 / {len(x)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
