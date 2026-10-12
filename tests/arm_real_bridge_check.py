#!/usr/bin/env python3
"""L2 真机臂桥接链路验收（不需要真臂，也不需要 Jetson）。

把"agx_arm_ctrl 会提供的东西"用假数据喂进去，检查 arm_real_adapter 的两个方向：

  上行（感知）: /feedback/joint_states  →  /ARM_JOINTS_DATA + /ARM_TELEOP_STATE
  下行（控制）: /ARM_JOINTS_CMD         →  /control/joint_states

断言：
  1. 喂入 joint1..6 = 默认姿态 + gripper=0.07 后，
     /ARM_JOINTS_DATA 的 8 个 position = [0, 0.5, -0.5, 0, 0, 0, +0.035, -0.035]；
  2. /ARM_TELEOP_STATE = 用 FK 算出的 root 系末端位姿，且与
     "默认姿态下 (0.3492,0,0.4327) + 四元数 (-0.7373,0,-0.6756,0)" 相差 < 2 mm / 0.5°
     （四元数取训练/链式 FK 的符号分支，DEF-039）；
  3. 发一条 /ARM_JOINTS_CMD（arm2 挪到 0.8、gripper1/2 = ±0.02），
     /control/joint_states 收到 joint2 ≈ 0.8、gripper ≈ 0.04。

用法（容器或任何有 ROS 环境的机器）::

  python3 tests/arm_real_bridge_check.py

退出码 0 = 全通过。
"""

from __future__ import annotations

import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from drdds.msg import (JointsData, JointsDataCmd, JointData, JointDataCmd,
                       JointsDataValue, JointsDataCmdValue, MetaType)

REPO = Path(__file__).resolve().parent.parent
ADAPTER = REPO / "src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py"

DEFAULT_Q = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
EXPECT_EE_POS = [0.3492, 0.0, 0.4327]
EXPECT_EE_QUAT = [-0.7373, 0.0, -0.6756, 0.0]   # 训练/链式 FK 的符号分支（DEF-039）

fails: list[str] = []


def check(ok: bool, msg: str) -> None:
    print(("  [PASS] " if ok else "  [FAIL] ") + msg)
    if not ok:
        fails.append(msg)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("arm_real_bridge_check")
        self.fb = self.create_publisher(JointState, "/feedback/joint_states", 10)
        self.cmd = self.create_publisher(JointsDataCmd, "/ARM_JOINTS_CMD", 10)
        self.data = None
        self.ee = None
        self.agx_ctrl = None
        self.create_subscription(JointsData, "/ARM_JOINTS_DATA", self._on_data, 10)
        self.create_subscription(Float32MultiArray, "/ARM_TELEOP_STATE", self._on_ee, 10)
        self.create_subscription(JointState, "/control/joint_states", self._on_ctrl, 10)

    def _on_data(self, m: JointsData) -> None:
        self.data = [float(m.data.joints_data[i].position) for i in range(8)]

    def _on_ee(self, m: Float32MultiArray) -> None:
        self.ee = [float(v) for v in m.data]

    def _on_ctrl(self, m: JointState) -> None:
        self.agx_ctrl = (list(m.name), list(m.position))

    def publish_feedback(self, q, gripper_width) -> None:
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = [f"joint{i + 1}" for i in range(6)] + ["gripper"]
        m.position = [float(v) for v in q] + [float(gripper_width)]
        self.fb.publish(m)

    def publish_cmd(self, q8) -> None:
        m = JointsDataCmd()
        m.data = JointsDataCmdValue()
        m.data.joints_data = [JointDataCmd() for _ in range(16)]
        for i in range(8):
            m.data.joints_data[i].position = float(q8[i])
            m.data.joints_data[i].kp = 0.0
            m.data.joints_data[i].kd = 0.0
            m.data.joints_data[i].control_word = 4
        self.cmd.publish(m)


def quat_angle_deg(qa, qb) -> float:
    d = abs(sum(a * b for a, b in zip(qa, qb)))
    d = max(-1.0, min(1.0, d))
    return math.degrees(2.0 * math.acos(d))


def main() -> int:
    print(f"[bridge] adapter = {ADAPTER}")
    proc = subprocess.Popen([sys.executable, str(ADAPTER)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    rclpy.init()
    node = Probe()
    try:
        time.sleep(1.0)                       # 等 adapter 起来
        print("[1] 上行：/feedback/joint_states → /ARM_JOINTS_DATA + /ARM_TELEOP_STATE")
        t0 = time.time()
        while time.time() - t0 < 5.0 and (node.data is None or node.ee is None):
            node.publish_feedback(DEFAULT_Q, 0.07)     # 默认姿态 + 夹爪开 0.07
            rclpy.spin_once(node, timeout_sec=0.01)
        check(node.data is not None, "/ARM_JOINTS_DATA 有数据")
        if node.data:
            expect = DEFAULT_Q + [0.035, -0.035]
            worst = max(abs(a - b) for a, b in zip(node.data, expect))
            check(worst < 1e-6,
                  f"8 个关节 = {['%.4f' % v for v in node.data]}（期望 {expect}，最大差 {worst:.2e}）")
        check(node.ee is not None, "/ARM_TELEOP_STATE 有数据")
        if node.ee:
            dp = max(abs(a - b) for a, b in zip(node.ee[:3], EXPECT_EE_POS))
            da = quat_angle_deg(node.ee[3:], EXPECT_EE_QUAT)
            check(dp < 0.002 and da < 0.5,
                  f"ee_goal = {['%.4f' % v for v in node.ee]}（位置差 {dp * 1000:.2f} mm、"
                  f"姿态差 {da:.3f}°）")

        print("[2] 下行：/ARM_JOINTS_CMD → /control/joint_states")
        q8 = DEFAULT_Q + [0.02, -0.02]
        q8[1] = 0.8
        t0 = time.time()
        while time.time() - t0 < 5.0 and node.agx_ctrl is None:
            node.publish_cmd(q8)
            rclpy.spin_once(node, timeout_sec=0.01)
        check(node.agx_ctrl is not None, "/control/joint_states 有数据")
        if node.agx_ctrl:
            names, pos = node.agx_ctrl
            idx = {n: i for i, n in enumerate(names)}
            ok = (abs(pos[idx["joint2"]] - 0.8) < 1e-6
                  and abs(pos[idx["gripper"]] - 0.04) < 1e-6)
            check(ok, f"joint2 = {pos[idx['joint2']]:.4f}（期望 0.8）、"
                      f"gripper = {pos[idx['gripper']]:.4f}（期望 0.04 = 0.02-(-0.02)）")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print()
    if fails:
        print(f"[bridge] FAIL（{len(fails)} 项）")
        return 1
    print("[bridge] PASS：真机臂桥接的两个方向都通")
    return 0


if __name__ == "__main__":
    sys.exit(main())
