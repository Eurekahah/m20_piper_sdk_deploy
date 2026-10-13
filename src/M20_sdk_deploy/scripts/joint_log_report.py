#!/usr/bin/env python3
"""离线分析 `record_joint_data.py` 录下来的真机 `/JOINTS_DATA` CSV —— 回答
"为什么偏偏是左前侧摆过温"（DEF-059 / TODO P1-8）。

四刀（对应 DEF-059 §2 末的清单）：
  ① 每个关节的 position / torque 与**仿真参考值**并列（差多少 ⇒ 负载/标定问题，还是相等）；
  ② `motion_temp`（电机）与 `driver_temp`（驱动器）**分开看**，并列出 >100 ℃ 的区段时序
     （量化"保护 → 趴下 → 恢复"）；
  ③ 谁最热 vs 谁力矩最大（若两者不是同一个关节 ⇒ 不是纯铜损）；
  ④ `--spectrum`：扭矩的 std 与高频带能量（仿真里是 <0.001 N·m，真机若有 ⇒ 纹波/极限环）。

用法（开发机或容器里，需要 numpy）::

    python3 src/M20_sdk_deploy/scripts/joint_log_report.py joints_xxx.csv
    python3 src/M20_sdk_deploy/scripts/joint_log_report.py joints_xxx.csv --ref rl
    python3 src/M20_sdk_deploy/scripts/joint_log_report.py joints_xxx.csv --t0 30 --spectrum

`--ref` 选对照表：`rl`（RL 站立零命令，默认）/ `standup`（站立状态机）/ `none`。
参考值来自 2026-10-13 的 sim2sim 实测（`DEFECT_LOG_zh.md` DEF-059：
`--mode stand` 47 s / `--mode rl` 154 s 的稳态窗）。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

LEGS = ("fl", "fr", "hl", "hr")
JOINTS = ("hipx", "hipy", "knee", "wheel")
NAMES = [f"{leg}_{j}" for leg in LEGS for j in JOINTS]
DOF = 16

# ---- sim2sim 实测参考值（稳态；单位 rad / N·m）------------------------------
# standup：站立状态机（z 之后保持，腿 kp=200）—— 2026-10-13 `--mode stand` 47 s
REF_STANDUP = {
    "q": [-0.0600, -0.6062, 1.0476, None, 0.0595, -0.6060, 1.0473, None,
          -0.0401, 0.5999, -1.0381, None, 0.0397, 0.5999, -1.0377, None],
    "tau": [12.0, 1.27, -9.50, 0.006, -11.9, 1.23, -9.44, 0.005,
            8.02, -0.02, 7.60, -0.006, -7.94, -0.01, 7.52, -0.005],
}
# rl：RL 站立、零速度命令（腿 kp=80）—— 2026-10-13 `--mode rl` 154 s 的 t>100 s
REF_RL = {
    "q": [-0.0036, -0.2873, 1.0670, None, 0.0318, -0.5144, 1.1336, None,
          -0.0216, 0.2643, -0.9916, None, 0.0872, 0.3163, -0.9614, None],
    "tau": [15.03, -16.34, -21.48, -0.68, -6.44, -4.47, -10.02, -0.22,
            4.60, 9.00, 10.38, 0.38, -13.68, 11.28, 15.48, 0.52],
}
REF = {"standup": REF_STANDUP, "rl": REF_RL, "none": None}

OVER_TEMP = 100.0


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"[err] {path} 没有数据行")
    meta_path = path[:-4] + ".meta.json" if path.endswith(".csv") else path + ".meta.json"
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
    names = meta.get("joint_names") or NAMES
    out = {"names": names, "meta": meta}
    for key, tpl in (("wall", "wall"), ("pos", "pos{}"), ("vel", "vel{}"),
                     ("tau", "tau{}"), ("mtemp", "mtemp{}"), ("dtemp", "dtemp{}")):
        if key == "wall":
            out[key] = np.array([float(r["wall"]) for r in rows])
        else:
            out[key] = np.array([[float(r[tpl.format(i)]) for i in range(DOF)]
                                 for r in rows])
    for k in ("imu_roll", "imu_pitch", "imu_yaw"):
        if k in rows[0]:
            out[k] = np.array([float(r[k]) for r in rows])
    out["n"] = len(rows)
    return out


def band_rms(x, fs, lo, hi):
    x = x - x.mean()
    n = len(x)
    if n < 32:
        return float("nan")
    w = np.hanning(n)
    spec = np.fft.rfft(x * w)
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    m = (freqs >= lo) & (freqs < hi)
    return float(np.sqrt(np.sum((np.abs(spec[m]) * 2.0 / w.sum()) ** 2) / 2.0))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--t0", type=float, default=0.0, help="分析窗口起点（s，默认 0）")
    ap.add_argument("--t1", type=float, default=1e9, help="分析窗口终点（s）")
    ap.add_argument("--ref", choices=("rl", "standup", "none"), default="rl",
                    help="与哪套仿真参考值对照（默认 rl）")
    ap.add_argument("--spectrum", action="store_true", help="打印扭矩高频带能量")
    args = ap.parse_args()

    d = load(args.csv)
    w = d["wall"]
    sel = (w >= args.t0) & (w <= args.t1)
    if sel.sum() < 32:
        print(f"[err] 窗口内样本太少（{sel.sum()}）")
        return 1
    names = d["names"]
    fs = 1.0 / float(np.median(np.diff(w)))
    dur = float(w[sel][-1] - w[sel][0])
    print(f"{args.csv}: {d['n']} 行，{dur:.1f} s（窗口 {w[sel][0]:.1f}~{w[sel][-1]:.1f} s，"
          f"{sel.sum()} 样本，{fs:.0f} Hz）")
    if "imu_roll" in d:
        print(f"IMU: roll ±{np.nanmax(np.abs(d['imu_roll'][sel])):.1f}°, "
              f"pitch ±{np.nanmax(np.abs(d['imu_pitch'][sel])):.1f}° "
              f"(倾角大 ⇒ 中途趴下/被抬动了)")

    # ---- ① 逐关节：τ / 温度 / 位置 ----
    mt, dt = d["mtemp"][sel], d["dtemp"][sel]
    ref = REF[args.ref]
    print(f"\n== ① 逐关节（窗口内均值；对照 `--ref {args.ref}`）==")
    hdr = f"{'joint':>9} {'tau均值':>9} {'tau_std':>8} {'tau极值':>16}"
    if ref:
        hdr += f" {'仿真tau':>8} {'差':>8}"
    hdr += f" {'q均值':>8}{'仿真q':>8} {'m_temp 首/末/峰':>21} {'d_temp 首/末/峰':>21}"
    print(hdr)
    for i in range(DOF):
        tau = d["tau"][sel, i]
        q = d["pos"][sel, i]
        line = (f"{names[i]:>9} {tau.mean():9.3f} {tau.std():8.3f} "
                f"{tau.min():7.2f}/{tau.max():7.2f}")
        if ref:
            r = ref["tau"][i]
            line += f" {r:8.2f} {tau.mean() - r:+8.2f}"
        line += f" {q.mean():8.3f}"
        line += f"{ref['q'][i]:8.3f}" if (ref and ref["q"][i] is not None) else f"{'—':>8}"
        line += (f" {mt[0, i]:6.1f}/{mt[-1, i]:6.1f}/{mt[:, i].max():6.1f}"
                 f" {dt[0, i]:6.1f}/{dt[-1, i]:6.1f}/{dt[:, i].max():6.1f}")
        print(line)

    # ---- ② >100 ℃ 事件时序（两个温度分开看）----
    print(f"\n== ② 过温事件（阈值 {OVER_TEMP:.0f} ℃）==")
    found = False
    for i in range(DOF):
        for field, arr, label in (("motor", mt, "motion_temp"),
                                  ("driver", dt, "driver_temp")):
            over = arr[:, i] > OVER_TEMP
            if not over.any():
                continue
            found = True
            idx = np.flatnonzero(over)
            print(f"  ⚠️ {names[i]} 的 {label} > {OVER_TEMP:.0f} ℃: "
                  f"首次 {w[sel][idx[0]]:.1f}s，持续 {len(idx) / fs:.1f} s，"
                  f"峰值 {arr[:, i].max():.1f} ℃")
    if not found:
        print(f"  窗口内没有任何关节的 m/d_temp 超过 {OVER_TEMP:.0f} ℃"
              f"（若无过温，说明这次没复现；可看下面的斜率/温差）")
    # 升温最快的关节（用首末温差近似）
    rise = [(names[i], mt[-1, i] - mt[0, i]) for i in range(DOF)]
    rise.sort(key=lambda kv: -kv[1])
    print("  升温最多（末-首，℃）: " + ", ".join(f"{n} {v:+.1f}" for n, v in rise[:4]))

    # ---- ③ 谁最热 vs 谁力矩最大 ----
    print("\n== ③ 谁最热 vs 谁力矩最大 ==")
    hot = max(range(DOF), key=lambda i: mt[:, i].max())
    load_i = max(range(DOF), key=lambda i: (d["tau"][sel, i] ** 2).mean())
    heat = sorted(((names[i], float((d["tau"][sel, i] ** 2).mean())) for i in range(DOF)),
                  key=lambda kv: -kv[1])[:5]
    print(f"  最热关节: {names[hot]}（motion {mt[:, hot].max():.1f} ℃ / "
          f"driver {dt[:, hot].max():.1f} ℃）")
    print(f"  力矩最大(τ²)关节: {names[load_i]}")
    print("  热负荷前 5（τ²）: " + ", ".join(f"{n} {v:.0f}" for n, v in heat))
    if hot != load_i:
        print("  ⇒ 两者不是同一个关节：说明热的那一个并不是力矩最大的那一个，"
              "纯铜损（τ ∝ 电流）解释不了它 ⇒ 查 driver_temp / 电流纹波 / 该电机额定")
    # 左右镜像对比（同名前缀 fl/fr、hl/hr）
    print("\n  左右镜像对比（同位置左右两侧的 τ 均值，N·m）:")
    for j in JOINTS:
        for fl_i, fr_i in ((LEGS.index("fl"), LEGS.index("fr")),
                           (LEGS.index("hl"), LEGS.index("hr"))):
            a, b = fl_i * 4 + JOINTS.index(j), fr_i * 4 + JOINTS.index(j)
            ma = d["tau"][sel, a].mean()
            mb = d["tau"][sel, b].mean()
            if abs(ma) + abs(mb) < 2.0:
                continue
            ratio = abs(ma) / max(abs(mb), 1e-9)
            flag = "  <== 明显不对称" if (ratio > 1.5 or ratio < 1 / 1.5) else ""
            print(f"    {names[a]:>9} {ma:+7.2f} | {names[b]:>9} {mb:+7.2f}"
                  f"   (|左|/|右| = {ratio:.2f}){flag}")

    # ---- ④ 频谱 ----
    if args.spectrum:
        print("\n== ④ 扭矩频谱（仿真参考：>5 Hz 都 <0.001 N·m；真机若有 ⇒ 纹波/极限环）==")
        for i in range(DOF):
            x = d["tau"][sel, i]
            bands = "  ".join(f"{lo}-{hi}Hz {band_rms(x, fs, lo, hi):.3f}"
                              for lo, hi in ((0, 5), (5, 50), (50, 200), (200, 500)))
            print(f"  {names[i]:>9}: std {x.std():7.3f}  {bands}")

    print("\n判读提示（对应 DEF-059 §2 末的四刀）：")
    print("  * 温差/升温在**同一关节**、其它关节也一起升 ⇒ 整条腿都贴着热上限，谁的边际最差谁先跳")
    print("  * 只有那个关节热、τ 也不比镜像大  ⇒ 该关节自身（电机/驱动器/线束/散热位置）")
    print("  * τ 明显比镜像大（>1.5×）        ⇒ 负载/标定（dir/offset、装配、背隙）")
    print("  * driver_temp 先到 / 比 motor_temp 高很多 ⇒ 功率级（压降/开关损耗/接插件），不是绕组")
    print("  * >5 Hz 有明显能量               ⇒ 纹波/极限环（仿真里没有，见 DEF-059 的延迟实验）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
