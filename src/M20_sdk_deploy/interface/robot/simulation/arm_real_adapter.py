#!/usr/bin/env python3
"""
Real-arm transport adapter: deployment arm_controller <-> agx_arm_ctrl.

The deployment pipeline keeps the DLS-IK arm_controller as the only high-level
source of arm targets for both sim and real hardware. On the real robot this
adapter sits between it and the external AgileX ROS2 driver:

    arm_controller.py  --/ARM_JOINTS_CMD-->  arm_real_adapter.py
                                              |--/control/joint_states--> agx_arm_ctrl
                                              <--/feedback/joint_states--
                                              |--/ARM_JOINTS_DATA--> arm_controller/rl_deploy

Run agx_arm_ctrl with `fast_mode:=true` so /control/joint_states drives the
arm with the direct, un-interpolated move_js path suitable for 50 Hz IK
teleop, and the gripper is handled by the same topic:

    ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
      can_port:=can0 arm_type:=piper effector_type:=agx_gripper fast_mode:=true

Then start this adapter in the same ROS_DOMAIN_ID:

    python3 src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py

Mapping conventions (kept identical to the sim URDF):
  arm_joint1..6  -> joint1..6          (raw rad)
  gripper_joint1/2 -> gripper width w  (m), w = q6 - q7
  feedback width w -> gripper_joint1 = +w/2, gripper_joint2 = -w/2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from drdds.msg import JointsData, JointsDataCmd, JointData, JointsDataValue
from drdds.msg import MetaType


ARM_CMD_TOPIC = '/ARM_JOINTS_CMD'
ARM_DATA_TOPIC = '/ARM_JOINTS_DATA'
AGX_CONTROL_TOPIC = '/control/joint_states'
AGX_FEEDBACK_TOPIC = '/feedback/joint_states'

ARM_JOINT_COUNT = 6
TOTAL_JOINT_COUNT = 8  # 6 arm + 2 sim gripper joints
GRIPPER_EFFORT = 1.0


class ArmRealAdapter(Node):
    def __init__(self):
        super().__init__('arm_real_adapter')

        self._arm_cmd_sub = self.create_subscription(
            JointsDataCmd, ARM_CMD_TOPIC, self._arm_cmd_cb, 10)
        self._agx_ctrl_pub = self.create_publisher(
            JointState, AGX_CONTROL_TOPIC, 10)

        self._agx_fb_sub = self.create_subscription(
            JointState, AGX_FEEDBACK_TOPIC, self._agx_fb_cb, 10)
        self._arm_data_pub = self.create_publisher(
            JointsData, ARM_DATA_TOPIC, 10)

        self._last_feedback = None
        self._feedback_warned = False

        # republish the latest agx feedback at 200 Hz (drdds side)
        self.create_timer(0.005, self._publish_feedback)

        self.get_logger().info(
            "[arm_real_adapter] started: /ARM_JOINTS_CMD -> "
            f"{AGX_CONTROL_TOPIC}, {AGX_FEEDBACK_TOPIC} -> /ARM_JOINTS_DATA")

    # ------------------------------------------------------------------
    def _arm_cmd_cb(self, msg):
        """drdds 8-DOF command (arm_joint1..6 + gripper1/2) -> agx joint_states."""
        if len(msg.data.joints_data) < TOTAL_JOINT_COUNT:
            return

        arm_pos = [
            msg.data.joints_data[i].position for i in range(ARM_JOINT_COUNT)]
        gr1 = msg.data.joints_data[6].position
        gr2 = msg.data.joints_data[7].position
        width = float(max(0.0, gr1 - gr2))

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = [f"joint{i + 1}" for i in range(ARM_JOINT_COUNT)] + ["gripper"]
        cmd.position = arm_pos + [width]
        cmd.velocity = [0.0] * (ARM_JOINT_COUNT + 1)
        cmd.effort = [0.0] * ARM_JOINT_COUNT + [GRIPPER_EFFORT]
        self._agx_ctrl_pub.publish(cmd)

    # ------------------------------------------------------------------
    def _agx_fb_cb(self, msg):
        self._last_feedback = msg

    def _publish_feedback(self):
        if self._last_feedback is None:
            if not self._feedback_warned:
                self.get_logger().warn(
                    f"[arm_real_adapter] no data on {AGX_FEEDBACK_TOPIC} yet")
                self._feedback_warned = True
            return

        name_to_idx = {n: i for i, n in enumerate(self._last_feedback.name)}
        positions = self._last_feedback.position

        q = [0.0] * TOTAL_JOINT_COUNT
        for i in range(ARM_JOINT_COUNT):
            name = f"joint{i + 1}"
            if name in name_to_idx:
                q[i] = float(positions[name_to_idx[name]])

        if "gripper" in name_to_idx:
            w = float(positions[name_to_idx["gripper"]])
            q[6] = 0.5 * w
            q[7] = -0.5 * w

        out = JointsData()
        out.header = MetaType()
        out.header.frame_id = 0
        stamp = Time()
        now = self.get_clock().now().nanoseconds
        stamp.sec = int(now // 1_000_000_000)
        stamp.nanosec = int(now % 1_000_000_000)
        out.header.stamp = stamp
        out.data = JointsDataValue()
        out.data.joints_data = [JointData() for _ in range(16)]
        for i in range(TOTAL_JOINT_COUNT):
            joint = out.data.joints_data[i]
            joint.name = [32, 32, 32, 32]
            joint.data_id = 0
            joint.status_word = 1
            joint.position = float(q[i])
            joint.velocity = 0.0
            joint.torque = 0.0
            joint.motion_temp = 40.0
            joint.driver_temp = 45.0
        self._arm_data_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArmRealAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
