#!/usr/bin/env python3
"""
Standalone VR teleop ROS2 node.

Runs the XLeVR HTTPS/WebSocket services and publishes the current VR command
snapshot on /VR_TELEOP (Float32MultiArray, layout in vr_teleop_protocol.py).

Run in its own terminal:

    python3 src/M20_sdk_deploy/interface/robot/simulation/vr_teleop_node.py

Then open the printed https://<ip>:8443 page in the headset browser.

This node only publishes device-level commands; routing to the arm controller
and to rl_deploy (legs/body) is handled by the integration nodes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from vr_teleop_device import Se2VRExtendedCfg, Se2VRExtendedDevice
from vr_teleop_protocol import make_msg


class VRTeleopNode(Node):
    def __init__(self):
        super().__init__('vr_teleop')
        self.device = Se2VRExtendedDevice(Se2VRExtendedCfg())

        self.vr_pub = self.create_publisher(
            Float32MultiArray, '/VR_TELEOP', 10)
        self.create_timer(0.02, self._tick)  # 50 Hz

    def _tick(self):
        cmd13, events = self.device.advance()
        msg = Float32MultiArray()
        msg.data = make_msg(
            cmd13,
            active=events["active"],
            calibrate=events["calibrate"],
            reset=events["reset_fail"] or events["reset_success"],
        )
        self.vr_pub.publish(msg)

        if events["calibrate"]:
            self.get_logger().info("[VR] calibrate event -> B pressed")
        if events["reset_fail"] or events["reset_success"]:
            self.get_logger().info("[VR] reset event -> X/Y pressed")


def main(args=None):
    rclpy.init(args=args)
    node = VRTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
