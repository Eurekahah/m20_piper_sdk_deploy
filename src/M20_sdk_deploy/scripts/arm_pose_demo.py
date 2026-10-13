#!/usr/bin/env python3
"""末端位姿指令演示：一串具名位姿依次下发给 arm_controller（看着调 / 验收用）。

它直接发 /ARM_TELEOP（mode 1 = 绝对偏移，锚在当前实际位姿）—— 与 arm_teleop
在 VR 激活时发的消息**同一种**，所以走的是同一条 arm_controller 链路
（DEF-044 的柱坐标映射 + IK 收敛检查 + 关节指令限速）。每一步保持 `--hold`
秒，并在每步结束时打印"目标 / 实际 / 位置误差 / 姿态误差 / max|dq| / max|tau|"。

**怎么跑**（容器内，按顺序；详见 `COMMANDS_SIM2SIM_zh.md` §5.3）：

  1) 仿真（要看画面就开窗口）：
       M20_USE_VIEWER=1 python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
  2) 臂 IK：
       M20_ARM_DEBUG=1 python3 src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py
  3) 腿：**必须**起 rl_deploy 并按 z（站立）→ c（进 RL）——只靠仿真自己的腿部默认
     保持，机器人会缓慢摔倒（2026-10-13 实测）：
       ros2 run m20_sdk_deploy rl_deploy          # 然后按 z、再按 c
  4) 本脚本（默认循环）：
       python3 src/M20_sdk_deploy/scripts/arm_pose_demo.py --hold 2.5

⚠️ **不要同时起 arm_teleop_node / arm_stub_node / arm_real_adapter**：它们和
arm_controller 会在 /ARM_TELEOP（或 /ARM_JOINTS_DATA、/ARM_TELEOP_STATE）上
形成**两个发布者**，两路目标/反馈交替到达 —— 表现是"目标读数是常量、臂不听使唤"，
见 DEF-041 / DEF-045。跑之前可以用
`python3 src/M20_sdk_deploy/scripts/check_vr_arm_chain.py` 看发布者数量。
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
SIM_DIR = REPO / "src/M20_sdk_deploy/interface/robot/simulation"
sys.path.insert(0, str(SIM_DIR))

from arm_controller import ARM_BASE_OFFSET, PIPER_MDH, fk_pose, mat_to_quat  # noqa: E402
from arm_teleop_protocol import make_absolute_offset  # noqa: E402

# (名字, (dx, dy, dz, droll, dpitch, dyaw))：偏移量，单位 m / rad
STEPS = [
    ("中位（标定锚点）", (0.00, 0.00, 0.00, 0.00, 0.00, 0.00)),
    ("前伸 8 cm", (0.08, 0.00, 0.00, 0.00, 0.00, 0.00)),
    ("后收 4 cm", (-0.04, 0.00, 0.00, 0.00, 0.00, 0.00)),
    ("左移 8 cm → 基座摆动", (0.00, 0.08, 0.00, 0.00, 0.00, 0.00)),
    ("右移 8 cm", (0.00, -0.08, 0.00, 0.00, 0.00, 0.00)),
    ("抬高 6 cm", (0.00, 0.00, 0.06, 0.00, 0.00, 0.00)),
    ("降低 6 cm", (0.00, 0.00, -0.06, 0.00, 0.00, 0.00)),
    ("手腕自转 +25°", (0.00, 0.00, 0.00, 0.44, 0.00, 0.00)),
    ("手腕俯仰 +25°", (0.00, 0.00, 0.00, 0.00, 0.44, 0.00)),
    ("手腕侧倾 +25°", (0.00, 0.00, 0.00, 0.00, 0.00, 0.44)),
    ("组合：前伸+左移+自转", (0.06, 0.06, 0.02, 0.35, 0.20, 0.00)),
    ("回中位", (0.00, 0.00, 0.00, 0.00, 0.00, 0.00)),
]


class PoseDemo:
    def __init__(self, hold: float, repeat: int, via_vr: bool):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
        from drdds.msg import JointsData

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("pose_demo")
        self.Msg = Float32MultiArray
        self.topic = "/VR_TELEOP" if via_vr else "/ARM_TELEOP"
        self.via_vr = via_vr
        self.pub = self.node.create_publisher(Float32MultiArray, self.topic, 10)
        self.fb = None
        self.target = None
        self.node.create_subscription(JointsData, "/ARM_JOINTS_DATA", self._fb_cb, 50)
        self.node.create_subscription(Float32MultiArray, "/ARM_TELEOP_STATE",
                                      self._st_cb, 50)
        self.off = list(STEPS[0][1])
        self.node.create_timer(0.02, self._publish)
        self.hold = hold
        self.repeat = repeat

    def _fb_cb(self, msg):
        self.fb = msg

    def _st_cb(self, msg):
        self.target = np.array([float(v) for v in msg.data[:7]])

    def _publish(self):
        if self.via_vr:
            # 16 浮点布局：active + 标定脉冲；由 arm_teleop 转成 mode 1 转发
            data = [0.0] * 16
            data[3:6] = self.off[:3]
            data[6:9] = self.off[3:6]
            data[12] = 0.0
            data[13] = 1.0
            data[14] = 1.0 if getattr(self, "_calib", True) else 0.0
        else:
            data = make_absolute_offset(*self.off, -1.0, 0.0)
        msg = self.Msg()
        msg.data = [float(v) for v in data]
        self.pub.publish(msg)

    def _measure(self):
        """实际末端（root 系）+ 关节速度/力矩，取自 /ARM_JOINTS_DATA。"""
        if self.fb is None or self.target is None:
            return None
        j = self.fb.data.joints_data
        q = np.array([float(x.position) for x in j[:6]])
        dq = np.array([float(x.velocity) for x in j[:6]])
        tau = np.array([float(x.torque) for x in j[:6]])
        p, R = fk_pose(PIPER_MDH, q)
        p = p + np.array(ARM_BASE_OFFSET)
        qa = mat_to_quat(R)
        qt = self.target[3:7]
        qt = qt / max(float(np.linalg.norm(qt)), 1e-12)
        ang = math.degrees(2.0 * math.acos(min(1.0, abs(float(np.dot(qa, qt))))))
        return dict(p=p, err_mm=float(np.linalg.norm(p - self.target[:3])) * 1000.0,
                    ang_deg=ang, dq=float(np.abs(dq).max()),
                    tau=float(np.abs(tau).max()))

    def run(self):
        rounds = (itertools.count(1) if self.repeat <= 0
                  else range(1, self.repeat + 1))
        total = "∞" if self.repeat <= 0 else str(self.repeat)
        for _ in range(200):                       # 等反馈/目标话题起来
            if self.fb is not None and self.target is not None:
                break
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.fb is None or self.target is None:
            print(f"[pose_demo] 话题没起来（fb={self.fb is not None} "
                  f"st={self.target is not None}）：先起 sim + arm_controller")
            return
        print(f"[pose_demo] 反馈就绪，往 {self.topic} 下发 {len(STEPS)} 个位姿指令"
              f"（每步 {self.hold:.1f}s × {total} 轮）\n", flush=True)
        t0 = time.monotonic()
        for rnd in rounds:
            for i, (name, off) in enumerate(STEPS, 1):
                self.off = list(off)
                t_start = time.monotonic()
                while time.monotonic() - t_start < self.hold:
                    self.rclpy.spin_once(self.node, timeout_sec=0.05)
                m = self._measure()
                if m is None:
                    continue
                print("[pose_demo] 轮 %s/%s 步 %2d/%d %-20s 目标(%s) 实际(%s) "
                      "位置误差 %5.1f mm 姿态误差 %4.1f° | max|dq| %4.2f rad/s "
                      "max|tau| %5.1f N·m"
                      % (rnd, total, i, len(STEPS), name,
                         "%.3f %.3f %.3f" % tuple(self.target[:3]),
                         "%.3f %.3f %.3f" % tuple(m["p"]),
                         m["err_mm"], m["ang_deg"], m["dq"], m["tau"]),
                      flush=True)
        print(f"\n[pose_demo] {len(STEPS) * self.repeat} 个步骤跑完"
              f"（{time.monotonic() - t0:.1f}s）。仿真与 arm_controller 仍在跑，"
              f"窗口里可以继续看；停就 Ctrl+C。", flush=True)
        while True:                                # 停在最后一步，别停发消息
            self.rclpy.spin_once(self.node, timeout_sec=0.1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", type=float, default=2.5, help="每步保持秒数")
    ap.add_argument("--repeat", type=int, default=0,
                    help="整串重复几遍；0 = 一直循环（给操作员看着调）")
    ap.add_argument("--via-vr", action="store_true",
                    help="发 /VR_TELEOP（需要 arm_teleop 在跑），默认直接发 /ARM_TELEOP")
    args = ap.parse_args()
    os.environ.setdefault("ROS_DOMAIN_ID", "1")
    demo = PoseDemo(args.hold, args.repeat, args.via_vr)
    try:
        demo.run()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
