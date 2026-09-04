#!/usr/bin/env python3
"""
Standalone arm teleop hub (keyboard + VR).

Owns the /ARM_TELEOP publisher so arm/gripper commands are independent of the
rl_deploy leg state machine. Run it in its own terminal whenever the arm is
driven by keyboard:

    python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py

When /VR_TELEOP reports an active VR session (operator pressed B), the hub
switches to mode 1 (absolute-offset VR commands) and the keyboard arm keys are
ignored. This keeps a single publisher on /ARM_TELEOP while both input sources
are wired.

Keyboard bindings intentionally match the existing deploy keyboard layout
(numpad EE + G/L); mode keys (R/Z/C/X) stay with rl_deploy.

  Numpad 8/2 : EE x +/-
  Numpad 4/6 : EE y +/-
  Numpad 7/9 : EE z +/-
  Numpad 1/3 : EE roll +/-
  Numpad 0/. : EE pitch +/-
  Numpad +/- : EE yaw +/-
  G          : gripper open/close toggle
  L          : reset arm target to current pose + open gripper
  ESC        : stop this node
"""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from arm_teleop_protocol import (
    make_incremental,
    make_absolute_offset,
)
from vr_teleop_protocol import (
    VR_ACTIVE_IDX,
    VR_CALIBRATE_IDX,
    VR_EE_EULER_START,
    VR_EE_POS_START,
    VR_GRIPPER_IDX,
    VR_RESET_IDX,
)

EE_POS_STEP = 0.005   # m per 5 ms repeat (same as deploy keyboard)
EE_ORN_STEP = 0.02    # rad per 5 ms repeat
KEY_TIMEOUT_MS = 500.0
TICK_PERIOD_S = 0.005

# numpad char -> (index into [dx,dy,dz,droll,dpitch,dyaw], sign)
EE_KEY_MAP = {
    '8': (0, +1.0), '2': (0, -1.0),
    '4': (1, +1.0), '6': (1, -1.0),
    '7': (2, +1.0), '9': (2, -1.0),
    '1': (3, +1.0), '3': (3, -1.0),
    '0': (4, +1.0), '.': (4, -1.0),
    '+': (5, +1.0), '-': (5, -1.0),
}


def _now_ms():
    return time.monotonic() * 1000.0


class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.arm_teleop_pub = self.create_publisher(
            Float32MultiArray, '/ARM_TELEOP', 10)

        self.held_keys = {}
        self.last_seen_ms = {}
        self.ee_inc = [0.0] * 6
        self.gripper_cmd = -1.0
        self.gripper_closed = False
        self.ee_reset_pending = 0.0

        # latest VR arm snapshot (mode 1 absolute-offset)
        self.vr_active = False
        self.vr_ee_off = [0.0] * 6
        self.vr_gripper = -1.0
        self.vr_calibrate_pulse = False
        self.vr_calib_prev = False
        self.vr_reset_prev = False

        self._setup_stdin()
        self.print_help()

        self.create_timer(TICK_PERIOD_S, self._tick)
        self.vr_teleop_sub = self.create_subscription(
            Float32MultiArray, '/VR_TELEOP', self._vr_teleop_cb, 10)

    def _setup_stdin(self):
        self._old_attr = termios.tcgetattr(sys.stdin.fileno())
        tty.setraw(sys.stdin.fileno())

    def _restore_stdin(self):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old_attr)

    def print_help(self):
        print("\n" + "=" * 52)
        print("  ARM TELEOP (standalone, arm/gripper only)")
        print("=" * 52)
        print("  EE position:  Num8/Num2 x, Num4/Num6 y, Num7/Num9 z")
        print("  EE rotation:  Num1/Num3 roll, Num0/Num. pitch, Num+/Num- yaw")
        print("  G gripper toggle, L reset arm target + open gripper")
        print("  ESC to stop this node")
        print("  (NumLock ON; mode keys R/Z/C/X belong to rl_deploy)")
        print("  (VR active after pressing B overrides the keyboard arm keys)")
        print("=" * 52 + "\n")

    def _vr_teleop_cb(self, msg):
        if len(msg.data) < 16:
            return
        self.vr_active = msg.data[VR_ACTIVE_IDX] > 0.5
        for i in range(6):
            if i < 3:
                self.vr_ee_off[i] = msg.data[VR_EE_POS_START + i]
            else:
                self.vr_ee_off[i] = msg.data[VR_EE_EULER_START + (i - 3)]
        self.vr_gripper = msg.data[VR_GRIPPER_IDX]

        calib = msg.data[VR_CALIBRATE_IDX] > 0.5
        if calib and not self.vr_calib_prev:
            self.vr_calibrate_pulse = True
        self.vr_calib_prev = calib

        reset = msg.data[VR_RESET_IDX] > 0.5
        if reset and not self.vr_reset_prev:
            # sim reset will restore the arm; tell arm_controller to re-anchor
            # to the fresh pose on the next absolute-offset command
            self.vr_calibrate_pulse = True
        self.vr_reset_prev = reset

    def _read_keys(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            if not ch:
                break
            k = ch.lower()
            now = _now_ms()

            if k == '\x1b':  # ESC
                self.get_logger().info("ESC pressed, stopping")
                self._restore_stdin()
                raise SystemExit(0)
            elif k == 'g':
                self.gripper_closed = not self.gripper_closed
                self.gripper_cmd = 1.0 if self.gripper_closed else 0.0
                self.get_logger().info(
                    "[GRIPPER] " + ("CLOSE" if self.gripper_closed else "OPEN"))
            elif k == 'l':
                self.ee_reset_pending = 1.0
                self.gripper_closed = False
                self.gripper_cmd = 0.0
                self.get_logger().info("[RESET] arm target reset to current pose")
            elif k in EE_KEY_MAP:
                self.held_keys[k] = True
                self.last_seen_ms[k] = now

    def _drop_stale_keys(self):
        now = _now_ms()
        stale = [k for k in self.held_keys
                 if now - self.last_seen_ms.get(k, now) > KEY_TIMEOUT_MS]
        for k in stale:
            del self.held_keys[k]
            self.last_seen_ms.pop(k, None)

    def _compute_ee_inc(self):
        for i in range(6):
            self.ee_inc[i] = 0.0
        for k in self.held_keys:
            idx, sign = EE_KEY_MAP[k]
            step = EE_POS_STEP if idx < 3 else EE_ORN_STEP
            self.ee_inc[idx] += sign * step

    def _tick(self):
        self._read_keys()

        if self.vr_active:
            data = make_absolute_offset(
                self.vr_ee_off[0], self.vr_ee_off[1], self.vr_ee_off[2],
                self.vr_ee_off[3], self.vr_ee_off[4], self.vr_ee_off[5],
                self.vr_gripper,
                1.0 if self.vr_calibrate_pulse else 0.0,
            )
            self.vr_calibrate_pulse = False
        else:
            self._drop_stale_keys()
            self._compute_ee_inc()
            data = make_incremental(
                self.ee_inc[0], self.ee_inc[1], self.ee_inc[2],
                self.ee_inc[3], self.ee_inc[4], self.ee_inc[5],
                self.gripper_cmd, self.ee_reset_pending,
            )
            if self.ee_reset_pending > 0.5:
                self.ee_reset_pending = 0.0

        msg = Float32MultiArray()
        msg.data = [float(v) for v in data]
        self.arm_teleop_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node._restore_stdin()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
