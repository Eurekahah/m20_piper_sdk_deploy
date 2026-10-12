#!/usr/bin/env python3
"""VR → 机械臂 链路自检：8 秒采样，直接告诉你"卡在哪一环"。

链路（每一环都是一个话题）：

    vr_teleop_node ──/VR_TELEOP──▶ arm_teleop_node ──/ARM_TELEOP──▶ arm_controller
                          │                                              │
                          └──────────▶ rl_deploy（腿/机身）              └──/ARM_JOINTS_CMD──▶ 执行器
                          ▲                                             ▲
                          │                          /ARM_JOINTS_DATA（反馈，200 Hz）

用法（容器内，仓库根目录，先 source ROS + install）::

    python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py            # 采 8 s
    python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py --seconds 15
    python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py --watch    # 逐轴监视

`--watch` 会每秒打一行 `/VR_TELEOP` 的 EE 六轴 + 各轴"最近 1 秒变化量"，
用来回答"推手柄向前，到底是哪个 EE 轴在变"（Ctrl+C 退出）：

* 某个轴在变、但机械臂不动 ⇒ 问题在**臂侧**（IK/工作空间/限位）；
* 你推的是"前"，变的却是别的轴 ⇒ **映射/手柄朝向**问题（见 §"世界系 vs 机体系"）；
* 六个轴全不动 ⇒ 手柄没被跟踪（或动得小于 4 mm 死区）。

采样期间**请按提示动一下右手柄**（VR 里右手柄管机械臂，左手柄管机身/底盘）。

退出码：0 = 链路看起来没断；1 = 有一环没数据（会打印是哪一环）。
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from drdds.msg import JointsData, JointsDataCmd

# /VR_TELEOP 的字段含义（见 interface/robot/simulation/vr_teleop_protocol.py）
VR_FIELDS = [
    (0, "chassis vx"), (1, "chassis vy"), (2, "chassis wz"),
    (3, "EE dx"), (4, "EE dy"), (5, "EE dz"),
    (6, "EE droll"), (7, "EE dpitch"), (8, "EE dyaw"),
    (9, "body dheight"), (10, "body dpitch"), (11, "body droll"),
    (12, "gripper"), (13, "active"), (14, "calibrate"), (15, "reset"),
]


class ChainChecker(Node):
    def __init__(self) -> None:
        super().__init__("vr_arm_chain_check")
        self.count = {t: 0 for t in
                      ("/VR_TELEOP", "/ARM_TELEOP", "/ARM_JOINTS_CMD",
                       "/ARM_JOINTS_DATA", "/ARM_TELEOP_STATE")}
        self.last_vr: list[float] | None = None
        self.last_arm_teleop: list[float] | None = None
        self.max_ee = 0.0
        self.max_chassis = 0.0
        self.arm_data_min = None
        self.arm_data_max = None
        # 注意：不能叫 self.publishers —— rclpy 的 Node 已经有这个属性（只读）
        self.pub_names: dict[str, list[str]] = {}

        self.create_subscription(Float32MultiArray, "/VR_TELEOP",
                                 self._vr_cb, 10)
        self.create_subscription(Float32MultiArray, "/ARM_TELEOP",
                                 self._at_cb, 10)
        self.create_subscription(Float32MultiArray, "/ARM_TELEOP_STATE",
                                 self._state_cb, 10)
        self.create_subscription(JointsDataCmd, "/ARM_JOINTS_CMD",
                                 self._cmd_cb, 10)
        self.create_subscription(JointsData, "/ARM_JOINTS_DATA",
                                 self._data_cb, 10)

    def _vr_cb(self, msg) -> None:
        self.count["/VR_TELEOP"] += 1
        d = [float(x) for x in msg.data]
        self.last_vr = d
        if len(d) >= 9:
            self.max_ee = max(self.max_ee, max(abs(x) for x in d[3:9]))
            self.max_chassis = max(self.max_chassis, max(abs(x) for x in d[0:3]))

    def _at_cb(self, msg) -> None:
        self.count["/ARM_TELEOP"] += 1
        self.last_arm_teleop = [float(x) for x in msg.data]

    def _state_cb(self, msg) -> None:
        self.count["/ARM_TELEOP_STATE"] += 1

    def _cmd_cb(self, _msg) -> None:
        self.count["/ARM_JOINTS_CMD"] += 1

    def _data_cb(self, msg) -> None:
        self.count["/ARM_JOINTS_DATA"] += 1
        try:
            q = [float(msg.data.joints_data[i].position) for i in range(6)]
        except Exception:
            return
        self.arm_data_min = q if self.arm_data_min is None else \
            [min(a, b) for a, b in zip(self.arm_data_min, q)]
        self.arm_data_max = q if self.arm_data_max is None else \
            [max(a, b) for a, b in zip(self.arm_data_max, q)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0, help="采样时长（s）")
    ap.add_argument("--watch", action="store_true",
                    help="逐轴监视模式：每秒打印 EE 六轴与各轴的 1 s 变化量（Ctrl+C 退出）")
    args = ap.parse_args()

    rclpy.init()
    node = ChainChecker()

    if args.watch:
        print("[vr-chain] 逐轴监视：请**只动右手柄的一个方向**（先前后、再左右、再上下），"
              "看下面哪一列在变。Ctrl+C 退出。", flush=True)
        print("%6s | %-30s | %-30s | %s"
              % ("t(s)", "VR 偏移 [dx dy dz dr dp dy]", "最近 1 s 变化量（同顺序）",
                 "ARM_TELEOP [mode dx dy dz]"), flush=True)
        prev: list[float] | None = None
        t0 = time.time()
        last = 0.0
        try:
            while True:
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.time() - t0
                if now - last < 1.0:
                    continue
                last = now
                d = node.last_vr
                if d is None:
                    print("%6.1f | 没收到 /VR_TELEOP" % now, flush=True)
                    continue
                ee = [float(x) for x in d[3:9]]
                chg = [0.0] * 6 if prev is None else [a - b for a, b in zip(ee, prev)]
                prev = ee
                at = node.last_arm_teleop or []
                at_s = ("[%.0f %+.4f %+.4f %+.4f]" % (at[0], at[1], at[2], at[3])
                        if len(at) >= 4 else "(无)")
                print("%6.1f | %-30s | %-30s | %s"
                      % (now,
                         " ".join("%+.4f" % v for v in ee),
                         " ".join("%+.4f" % v for v in chg),
                         at_s), flush=True)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
        return 0

    print(f"[vr-chain] 采样 {args.seconds:.0f} s —— 现在**动一下右手柄**（右手柄管机械臂），"
          f"顺便按一下右 B 试试激活/重标定 ...", flush=True)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)

    n = node.count
    sec = args.seconds
    print("\n=== 各话题流量 ===")
    for t, c in n.items():
        print("  %-20s %5d 条  (≈%.1f Hz)" % (t, c, c / sec))

    # 发布者数量：**这是最容易踩的坑** —— 同时跑两个 vr_teleop_node（或两个
    # arm_teleop_node）时，两路命令会交替到达，目标在两帧之间来回跳，
    # 表现就是"动作特别快/不跟手/夹爪乱拍"（2026-10-10 实测复现过）。
    print("\n=== 各话题的发布者（重复的要杀掉）===")
    for t in n:
        try:
            infos = node.get_publishers_info_by_topic(t)
        except Exception:
            infos = []
        names = ["%s/%s" % (i.node_namespace.rstrip("/"), i.node_name) for i in infos]
        node.pub_names[t] = names
        flag = "  ← ⚠️ 有 %d 个发布者！" % len(names) if len(names) > 1 else ""
        print("  %-20s %d 个: %s%s" % (t, len(names), ", ".join(names) or "(无)", flag))

    print("\n=== /VR_TELEOP 最后一帧 ===")
    d = node.last_vr
    if d is None:
        print("  （没收到）")
    else:
        for i, name in VR_FIELDS:
            if i < len(d):
                print("  [%2d] %-16s = %+.4f" % (i, name, d[i]))
        print("  —— 采样期 |EE 偏移| 最大 = %.4f（m/rad）; |chassis| 最大 = %.3f"
              % (node.max_ee, node.max_chassis))

    print("\n=== /ARM_TELEOP 最后一帧（mode 1 = VR 绝对偏移）===")
    print("  ", node.last_arm_teleop if node.last_arm_teleop else "（没收到）")
    if node.arm_data_min is not None:
        print("=== /ARM_JOINTS_DATA 关节角变化范围（6 轴）===")
        print("  min", [round(x, 3) for x in node.arm_data_min])
        print("  max", [round(x, 3) for x in node.arm_data_max])
        span = max(b - a for a, b in zip(node.arm_data_min, node.arm_data_max))
        print("  → 采样期最大变化 %.4f rad %s"
              % (span, "(臂真的在动)" if span > 0.01 else "(臂基本没动)"))

    # ---------------- 判定 ----------------
    problems: list[str] = []
    for t, names in node.pub_names.items():
        if len(names) > 1:
            problems.append(
                "%s 上有 %d 个发布者（%s）⇒ 命令会**交替到达**，目标在两帧之间来回跳；"
                "表现就是『动作特别快 / 不跟手 / 夹爪乱拍』。把多余的节点杀掉"
                "（常见原因：起了两个 vr_teleop_node 或两个 arm_teleop_node）"
                % (t, len(names), ", ".join(names)))
    if n["/VR_TELEOP"] == 0:
        problems.append("VR 节点没在发 /VR_TELEOP：`vr_teleop_node.py` 没起、崩了，或 "
                        "ROS_DOMAIN_ID 与其它终端不一致（所有终端都要是 1）")
    else:
        if d and d[13] < 0.5:
            problems.append("active=0 ⇒ 还没激活：在**右手柄**上按一次 **B**"
                            "（按下后 VR 节点会打印 `[VR] Button B -> START + calibrate`）")
        if node.max_ee == 0.0:
            problems.append("采样期 EE 偏移一直是 0 ⇒ 右手柄的位姿没变化："
                            "① 确认你动的是**右手柄**（左手柄管机身、不是臂）；"
                            "② 手柄要在头显视野内（Quest 出视野会丢跟踪）；"
                            "③ 按 B 之后再动 ≥1 cm（有 4 mm 死区）")
    if n["/ARM_TELEOP"] == 0 and n["/VR_TELEOP"] > 0:
        problems.append("arm_teleop_node 没在发 /ARM_TELEOP：没起、崩了，"
                        "或者**没有 TTY**（它的键盘要 setcbreak，必须用 `docker exec -it` 起）")
    if n["/ARM_JOINTS_CMD"] == 0 and n["/ARM_TELEOP"] > 0:
        problems.append("arm_controller 没在发 /ARM_JOINTS_CMD：没起或崩了")
    if n["/ARM_JOINTS_DATA"] == 0:
        problems.append("收不到 /ARM_JOINTS_DATA（臂反馈）：仿真/真臂侧没在发，"
                        "arm_controller 拿不到当前关节角就没法做 IK")

    print()
    if problems:
        print("[vr-chain] 发现 %d 个问题：" % len(problems))
        for p in problems:
            print("  - " + p)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print("[vr-chain] 链路各环都有数据。如果臂还是不动，看上面 `臂基本没动` 那行："
          "若 /ARM_TELEOP 的偏移在变但臂不动 ⇒ 目标不可达/被限位夹住（换近一点的目标试）。")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
