"""
Layout for the /VR_TELEOP Float32MultiArray published by the VR node.

The message is a snapshot of the current VR controller state plus one-shot
event flags, published at ~50 Hz:

  [0:3]    chassis velocity (vx, vy, wz)
  [3:6]    arm EE offset from calibration origin (dx, dy, dz)
  [6:9]    arm EE orientation offset (droll, dpitch, dyaw)
  [9:12]   body pose offset (dheight, dpitch, droll)
  [12]     gripper: 0 open / 1 closed
  [13]     active: 1 once the operator pressed B (start + calibrate)
  [14]     calibrate: 1 on the press edge of B
  [15]     reset: 1 on the press edge of X/Y (fail/success both reset)

Semantics follow the training-side VR device (offsets relative to the
controller calibration origin), but consumers keep incremental/integrated
interfaces: a calibrate event tells arm/body anchors to re-capture the robot
state, after which the offsets are applied on top of the fresh anchor.
"""

VR_TELEOP_LEN = 16

VR_CHASSIS_START = 0
VR_EE_POS_START = 3
VR_EE_EULER_START = 6
VR_BODY_START = 9
VR_GRIPPER_IDX = 12
VR_ACTIVE_IDX = 13
VR_CALIBRATE_IDX = 14
VR_RESET_IDX = 15


def make_msg(cmd13, active=False, calibrate=False, reset=False):
    msg = [0.0] * VR_TELEOP_LEN
    msg[VR_CHASSIS_START:VR_CHASSIS_START + 3] = cmd13[0:3]
    msg[VR_EE_POS_START:VR_EE_POS_START + 3] = cmd13[3:6]
    msg[VR_EE_EULER_START:VR_EE_EULER_START + 3] = cmd13[6:9]
    msg[VR_BODY_START:VR_BODY_START + 3] = cmd13[9:12]
    msg[VR_GRIPPER_IDX] = float(cmd13[12])
    msg[VR_ACTIVE_IDX] = 1.0 if active else 0.0
    msg[VR_CALIBRATE_IDX] = 1.0 if calibrate else 0.0
    msg[VR_RESET_IDX] = 1.0 if reset else 0.0
    return msg
