#!/usr/bin/env python3
"""无臂部署用的"假臂"节点（arm_stub_node）。

**什么时候用**：机械臂还没装 / 没上电 / 没接 CAN，但要用同一套策略跑 M20。

**为什么需要它**：这条策略的观测是 24 关节的 `joint_pos` / `joint_vel`
（12 腿 + 4 轮 + 6 臂 + 2 夹爪，见 `docs/sim2sim_layout_contract_zh.md`），
再加上 7 维 `ee_goal`。臂没有反馈时：

* `PiperArmInterface` 会把 8 个臂关节当成"默认姿态"（`DEF-023`），
  于是策略认为"臂在默认位置"；
* 但 `ee_goal` 如果没有 `/ARM_TELEOP_STATE`，用的是默认值，
  **两者必须一致**，否则策略会去追一个和自己状态矛盾的目标。

本节点就是让这两件事一致：

* 以 200 Hz 发布 `/ARM_JOINTS_DATA`：8 个关节 = 训练默认姿态
  （`arm_joint1..6 = 0, 0.5, -0.5, 0, 0, 0`，夹爪 `0, 0`），速度/力矩为 0，
  `status_word = 1`（正常）、`control_word = kIndexMotorControl(4)`；
* 以 50 Hz 发布 `/ARM_TELEOP_STATE`：该默认姿态下 `gripper_base` 在
  **root 系**的位姿 `(0.3492, 0, 0.4326)` + 四元数 `(-0.7373, 0, -0.6756, 0)`
  （由 `scripts/check_mjcf_contract.py` 实测；符号必须与训练/链式 FK 一致，
  见 DEF-039 —— 这是策略观测的原始分量，不能写成 +0.7373）；
* 订阅 `/ARM_JOINTS_CMD` 并丢弃（可选打印），这样即使有人误起
  `arm_controller`，也不会因为"没人接收"而困惑。

**注意**：这是"假装臂在默认姿态"，不是"把臂从观测里去掉"。如果你希望策略
真正感知不到臂，那是**训练侧**的改动（改观测口径并重训），部署侧不能单方面改
（改了观测就等于换了策略，见 `policy/README.md` 的硬性约束）。

环境变量：
  M20_ARM_STUB_POSE="0,0.5,-0.5,0,0,0,0,0"   假反馈的 8 关节角（rad）
  M20_ARM_STUB_RATE=200                       /ARM_JOINTS_DATA 的频率 [Hz]
  M20_ARM_STUB_DEBUG=1                        打印收到的 /ARM_JOINTS_CMD（1 Hz）
  M20_ARM_STUB_EE=""                          覆盖 ee_goal（"x,y,z,qw,qx,qy,qz"，root 系）
  M20_ARM_STUB_FOLLOW=1                       打开"跟随命令"（模拟 CAN 驱动器）：
                                              /ARM_JOINTS_CMD 的位置目标被当成真的会执行，
                                              反馈按 M20_ARM_STUB_MAX_VEL 的速度上限一阶跟随。
                                              用途：**没有真臂**也想调 arm_controller /
                                              arm_teleop 这条链路（sim2sim 的 mixed 配置）。
  M20_ARM_STUB_MAX_VEL=3.0                    跟随速度上限 [rad/s]（真机 piper 的限速）
"""

from __future__ import annotations

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from drdds.msg import JointsData, JointsDataCmd, JointData
from std_msgs.msg import Float32MultiArray

# 与训练 init_state / MJCF 默认姿态一致（arm_joint1..6 + gripper_joint1/2）
DEFAULT_POSE = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

# 默认姿态下 gripper_base 在 root（机体）系里的位姿 —— 与
# `PiperArmPolicyRunner` 的 ee_goal 默认值、`arm_controller` 的 body-frame 输出一致
# （实测值见 scripts/check_mjcf_contract.py：pos=(0.3492, 0, 0.4326)）
DEFAULT_EE_GOAL_BODY = np.array(
    [0.3492, 0.0, 0.4327, -0.7373, 0.0, -0.6756, 0.0], dtype=np.float64)

K_INDEX_MOTOR_CONTROL = 4


def parse_pose(s: str) -> np.ndarray:
    vals = [float(v) for v in s.replace(" ", "").split(",") if v != ""]
    if len(vals) != 8:
        raise ValueError("M20_ARM_STUB_POSE 需要 8 个数（arm1..6 + gripper1/2）")
    return np.array(vals, dtype=np.float64)


class ArmStubNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_stub")
        pose_env = os.environ.get("M20_ARM_STUB_POSE", "")
        self.pose = parse_pose(pose_env) if pose_env else DEFAULT_POSE
        ee_env = os.environ.get("M20_ARM_STUB_EE", "")
        if ee_env:
            vals = [float(v) for v in ee_env.replace(" ", "").split(",") if v != ""]
            if len(vals) != 7:
                raise ValueError("M20_ARM_STUB_EE 需要 7 个数（pos3 + quat wxyz4）")
            self.ee_goal = np.array(vals, dtype=np.float64)
        else:
            self.ee_goal = DEFAULT_EE_GOAL_BODY.copy()

        rate = float(os.environ.get("M20_ARM_STUB_RATE", "200"))
        self.rate = max(rate, 1.0)
        self.debug = os.environ.get("M20_ARM_STUB_DEBUG", "0") != "0"
        # 跟随模式 = 模拟 CAN 驱动器；关闭时是"静态假臂"（臂没装，只保持默认姿态）
        self.follow = os.environ.get("M20_ARM_STUB_FOLLOW", "0") != "0"
        self.max_vel = float(os.environ.get("M20_ARM_STUB_MAX_VEL", "3.0"))
        self.target = self.pose.copy()
        self.cmd_count = 0
        self.last_cmd_time = self.get_clock().now()

        self.data_pub = self.create_publisher(JointsData, "/ARM_JOINTS_DATA", 10)
        self.state_pub = self.create_publisher(Float32MultiArray, "/ARM_TELEOP_STATE", 10)
        self.cmd_sub = self.create_subscription(
            JointsDataCmd, "/ARM_JOINTS_CMD", self._cmd_cb, 10)

        self.create_timer(1.0 / self.rate, self._publish_data)
        self.create_timer(0.02, self._publish_state)          # 50 Hz，与 arm_controller 一致

        print("[arm_stub] 假臂节点启动（臂没装时用）")
        print("[arm_stub] /ARM_JOINTS_DATA  = "
              + " ".join("%+.3f" % v for v in self.pose) + " @ %.0f Hz" % rate)
        print("[arm_stub] /ARM_TELEOP_STATE = "
              + " ".join("%+.4f" % v for v in self.ee_goal) + " (root 系)")
        if self.debug:
            print("[arm_stub] 会以 1 Hz 打印收到的 /ARM_JOINTS_CMD（丢弃）")
        if self.follow:
            print("[arm_stub] 跟随模式：按 /ARM_JOINTS_CMD 的位置目标运动，速度上限 "
                  "%.2f rad/s（模拟 CAN 驱动器）" % self.max_vel)

    # ------------------------------------------------------------------
    def _publish_data(self) -> None:
        if self.follow:
            # 一阶跟随（速度限幅），把命令位置变成"真实反馈"
            dt = 1.0 / self.rate
            step = self.max_vel * dt
            self.pose = self.pose + np.clip(self.target - self.pose, -step, step)
        msg = JointsData()
        msg.data.joints_data = [JointData() for _ in range(16)]
        for i in range(8):
            j = msg.data.joints_data[i]
            j.name = [32, 32, 32, 32]
            j.data_id = 0
            j.status_word = 1                      # 正常
            j.position = float(self.pose[i])
            j.velocity = 0.0
            j.torque = 0.0
            j.motion_temp = 40.0
            j.driver_temp = 45.0
        self.data_pub.publish(msg)

    def _publish_state(self) -> None:
        msg = Float32MultiArray()
        msg.data = [float(v) for v in self.ee_goal]
        self.state_pub.publish(msg)

    def _cmd_cb(self, msg: JointsDataCmd) -> None:
        # 静态模式：丢弃（无臂部署时没有执行器）；跟随模式：当成位置目标执行。
        self.cmd_count += 1
        if self.follow:
            for i in range(8):
                self.target[i] = float(msg.data.joints_data[i].position)
        if not self.debug:
            return
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds > 1e9:
            self.last_cmd_time = now
            pos = [round(float(msg.data.joints_data[i].position), 3) for i in range(8)]
            print(f"[arm_stub] 收到 /ARM_JOINTS_CMD（累计 {self.cmd_count} 条，丢弃）: {pos}")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ArmStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("[arm_stub] 退出")
    return 0


if __name__ == "__main__":
    sys.exit(main())
