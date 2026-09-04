#!/usr/bin/env python3
"""
 * @file arm_controller.py
 * @brief AgileX Piper arm controller for sim2sim deploy
 * @author M20+Piper deploy
 * @date 2026-08-20
 *
 * The arm is not controlled by the RL policy; it tracks an absolute
 * end-effector pose solved by DLS (damped least squares) Jacobian IK, mirroring
 * the Isaac Lab training (DifferentialIKController, ik_method="dls",
 * lambda=0.01, body=gripper_base, joint_names=arm_joint[1-6]).
 *
 * pyAgxArm fusion: the Piper MDH parameters are loaded from
 * pyAgxArm.utiles.mdh_kinematics when the SDK is installed; a copy of the
 * table + FK is embedded as a fallback so sim2sim works without the SDK.
 *
 * Topics:
 *   subscribe /ARM_TELEOP      std_msgs/Float32MultiArray
 *                              new 9-float protocol (arm_teleop_protocol.py)
 *                              mode 0: incremental keyboard command
 *                              mode 1: absolute-offset VR command
 *   subscribe /ARM_JOINTS_DATA drdds/msg/JointsData   (8 current joints)
 *   publish   /ARM_JOINTS_CMD  drdds/msg/JointsDataCmd (8 target joints)
 *   publish   /ARM_TELEOP_STATE std_msgs/Float32MultiArray
 *                              [x,y,z, qw,qx,qy,qz] absolute EE target (obs)
 *
 * For sim2real the same node can later write to the real Piper over CAN with
 * pyAgxArm (AgxArmFactory.create_arm(ArmModel.PIPER, ...)) instead of
 * publishing /ARM_JOINTS_CMD.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from drdds.msg import JointsData, JointsDataCmd

from arm_teleop_protocol import (
    ARM_TELEOP_LEN,
    ARM_TELEOP_MODE_ABSOLUTE,
    ARM_TELEOP_EULER_START,
    ARM_TELEOP_FLAG_IDX,
    ARM_TELEOP_GRIPPER_IDX,
    ARM_TELEOP_MODE_IDX,
    ARM_TELEOP_POS_START,
)


# ----------------------------------------------------------------------------
# Piper kinematics (pyAgxArm fusion)
# ----------------------------------------------------------------------------
try:
    from pyAgxArm.utiles.mdh_kinematics import get_mdh
    _MDH = get_mdh("piper")
    print("[arm_controller] using pyAgxArm MDH table")
except Exception:
    _MDH = None

# Modified-DH (d, a, alpha, theta_offset) for Piper, from pyAgxArm
# pyAgxArm/api/constants.py ROBOT_MDH_PRESET["piper"].
PIPER_MDH = _MDH if _MDH is not None else (
    (0.123, 0.0, 0.0, 0.0),
    (0.0, 0.0, -1.5707963267948966, -3.0058060377846343),
    (0.0, 0.28503, 0.0, -1.793849405199772),
    (0.25075, -0.02198, 1.5707963267948966, 0.0),
    (0.0, 0.0, -1.5707963267948966, 0.0),
    (0.091, 0.0, 1.5707963267948966, 0.0),
)

ARM_JOINT_LIMITS = np.array([
    (-2.618, 2.168),   # arm_joint1
    (0.0, 3.14),       # arm_joint2
    (-2.967, 0.0),     # arm_joint3
    (-1.745, 1.745),   # arm_joint4
    (-1.22, 1.22),     # arm_joint5
    (-2.0944, 2.0944), # arm_joint6
])
DEFAULT_ARM_JOINTS = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0])

# gains from the Isaac Lab DelayedPDActuatorCfg
ARM_KP, ARM_KD = 40.0, 8.0   # matches Isaac Lab piper_arm actuator (stiffness 40 / damping 8)
GRIPPER_KP, GRIPPER_KD = 4000.0, 200.0
GRIPPER_OPEN = np.array([0.035, -0.035])
GRIPPER_CLOSED = np.array([0.0, 0.0])

EE_POS_CLAMP = ((-0.6, 0.6), (-0.6, 0.6), (0.02, 0.95))
IK_LAMBDA = 0.01
MAX_DQ_PER_STEP = 0.05
IK_ITERATIONS = 20
IK_ERROR_TOL = 1e-3


# ----------------------------------------------------------------------------
# quaternion helpers (wxyz convention, matching Isaac Lab)
# ----------------------------------------------------------------------------
def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_multiply(q1, q2):
    """q1 * q2 (both wxyz)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        j = (i + 1) % 3
        k = (j + 1) % 3
        s = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k])) * 2
        q = np.zeros(4)
        q[i + 1] = 0.25 * s
        q[j + 1] = (R[j, i] + R[i, j]) / s
        q[k + 1] = (R[k, i] + R[i, k]) / s
        q[0] = (R[k, j] - R[j, k]) / s
        x, y, z = q[1], q[2], q[3]
        w = q[0]
    return np.array([w, x, y, z])


# ----------------------------------------------------------------------------
# MDH FK (same convention as pyAgxArm.utiles.mdh_kinematics)
# ----------------------------------------------------------------------------
def _link_mdh(d, a, alpha, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st, 0.0, a],
        [ca * st, ca * ct, -sa, -sa * d],
        [sa * st, sa * ct, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def fk_mdh(mdh, q):
    """Return the 4x4 homogeneous transform for the Piper (arm-base frame)."""
    T = np.eye(4)
    for (d, a, alpha, theta_off), qi in zip(mdh, q):
        T = T @ _link_mdh(d, a, alpha, qi + theta_off)
    return T


def fk_pose(mdh, q):
    T = fk_mdh(mdh, q)
    return T[:3, 3].copy(), T[:3, :3].copy()


def pose_error(target_pos, target_R, cur_pos, cur_R):
    pos_err = target_pos - cur_pos
    R_err = target_R @ cur_R.T
    cos_a = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cos_a)
    if angle < 1e-9:
        orn_err = np.zeros(3)
    else:
        axis = np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ]) / (2.0 * math.sin(angle))
        orn_err = axis * angle
    return np.concatenate([pos_err, orn_err])


def ik_step(mdh, q, target_pos, target_R):
    """One DLS iteration; returns (new_q, error_norm)."""
    cur_pos, cur_R = fk_pose(mdh, q)
    err = pose_error(target_pos, target_R, cur_pos, cur_R)
    err_norm = float(np.linalg.norm(err))
    if err_norm < IK_ERROR_TOL:
        return q, err_norm

    # numerical Jacobian (6x6)
    J = np.zeros((6, 6))
    eps = 1e-6
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        p2, R2 = fk_pose(mdh, dq)
        J[:3, i] = (p2 - cur_pos) / eps
        R_delta = R2 @ cur_R.T
        cos_a = np.clip((np.trace(R_delta) - 1.0) / 2.0, -1.0, 1.0)
        ang = math.acos(cos_a)
        if ang < 1e-9:
            axis = np.zeros(3)
        else:
            axis = np.array([
                R_delta[2, 1] - R_delta[1, 2],
                R_delta[0, 2] - R_delta[2, 0],
                R_delta[1, 0] - R_delta[0, 1],
            ]) / (2.0 * math.sin(ang))
        J[3:, i] = axis * ang / eps

    dq = J.T @ np.linalg.solve(J @ J.T + IK_LAMBDA ** 2 * np.eye(6), err)
    dq_norm = float(np.linalg.norm(dq))
    if dq_norm > MAX_DQ_PER_STEP:
        dq *= MAX_DQ_PER_STEP / dq_norm

    lo = ARM_JOINT_LIMITS[:, 0]
    hi = ARM_JOINT_LIMITS[:, 1]
    return np.clip(q + dq, lo, hi), err_norm


# ----------------------------------------------------------------------------
# ROS node
# ----------------------------------------------------------------------------
class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.cur_joints = DEFAULT_ARM_JOINTS.copy()
        self.cur_gripper = GRIPPER_CLOSED.copy()

        # absolute EE target, initialised at the default arm pose
        self.target_pos, self.target_R = fk_pose(PIPER_MDH, DEFAULT_ARM_JOINTS)
        self.target_quat = mat_to_quat(self.target_R)
        self.gripper_target = GRIPPER_CLOSED.copy()

        self.arm_data_sub = self.create_subscription(
            JointsData, '/ARM_JOINTS_DATA', self._data_cb, 10)
        self.teleop_sub = self.create_subscription(
            Float32MultiArray, '/ARM_TELEOP', self._teleop_cb, 10)
        self.arm_cmd_pub = self.create_publisher(
            JointsDataCmd, '/ARM_JOINTS_CMD', 10)
        self.state_pub = self.create_publisher(
            Float32MultiArray, '/ARM_TELEOP_STATE', 10)

        self.timer = self.create_timer(0.02, self._tick)  # 50 Hz

        # Anchor state used by mode 1 (absolute-offset / VR):
        # target = anchor + controller offset, with the anchor captured from
        # the actual robot pose on ee_recalibrate.
        self.anchor_pos, self.anchor_R = fk_pose(PIPER_MDH, DEFAULT_ARM_JOINTS)
        self.anchor_quat = mat_to_quat(self.anchor_R)
        self.abs_anchor_initialized = False

        self.get_logger().info(
            "[arm_controller] started (MDH FK + DLS IK, 50 Hz)")

    def _data_cb(self, msg):
        if len(msg.data.joints_data) >= 8:
            q = np.array([msg.data.joints_data[i].position for i in range(8)])
            self.cur_joints = q[:6]
            self.cur_gripper = q[6:]

    def _teleop_cb(self, msg):
        if len(msg.data) < ARM_TELEOP_LEN:
            # backwards-compatible 8-float incremental message (no mode field)
            if len(msg.data) < 8:
                return
            data = msg.data
            self._apply_incremental(
                data[0], data[1], data[2],
                data[3], data[4], data[5],
                data[6], data[7],
            )
            return

        mode = msg.data[ARM_TELEOP_MODE_IDX]
        dx, dy, dz = msg.data[ARM_TELEOP_POS_START:ARM_TELEOP_POS_START + 3]
        droll, dpitch, dyaw = msg.data[
            ARM_TELEOP_EULER_START:ARM_TELEOP_EULER_START + 3]
        gripper_cmd = msg.data[ARM_TELEOP_GRIPPER_IDX]
        flag = msg.data[ARM_TELEOP_FLAG_IDX]

        if mode > 0.5:
            self._apply_absolute_offset(dx, dy, dz, droll, dpitch, dyaw,
                                        gripper_cmd, flag)
        else:
            self._apply_incremental(dx, dy, dz, droll, dpitch, dyaw,
                                    gripper_cmd, flag)

    def _apply_incremental(self, dx, dy, dz, droll, dpitch, dyaw,
                           gripper_cmd, ee_reset):
        """Legacy keyboard semantics: integrate per-tick deltas."""
        if ee_reset > 0.5:
            pos, R = fk_pose(PIPER_MDH, self.cur_joints)
            self.target_pos = pos
            self.target_R = R
            self.target_quat = mat_to_quat(R)
            self.abs_anchor_initialized = False
        else:
            self.target_pos += np.array([dx, dy, dz])
            dq = quat_from_euler(droll, dpitch, dyaw)
            self.target_quat = quat_multiply(dq, self.target_quat)
            self.target_R = quat_to_mat(self.target_quat)

        self._clamp_and_gripper(gripper_cmd)

    def _apply_absolute_offset(self, dx, dy, dz, droll, dpitch, dyaw,
                               gripper_cmd, recalibrate):
        """
        VR semantics: target = anchor + controller offset.

        The anchor is captured from the actual robot pose on the first absolute
        message or whenever the operator presses recalibrate (B). Between
        recalibrations a stationary controller therefore keeps the target
        still, and hand drift is not integrated into the target.
        """
        if recalibrate > 0.5 or not self.abs_anchor_initialized:
            pos, R = fk_pose(PIPER_MDH, self.cur_joints)
            self.anchor_pos = pos
            self.anchor_R = R
            self.anchor_quat = mat_to_quat(R)
            self.abs_anchor_initialized = True

        self.target_pos = self.anchor_pos + np.array([dx, dy, dz])
        dq = quat_from_euler(droll, dpitch, dyaw)
        self.target_quat = quat_multiply(dq, self.anchor_quat)
        self.target_R = quat_to_mat(self.target_quat)

        self._clamp_and_gripper(gripper_cmd)

    def _clamp_and_gripper(self, gripper_cmd):
        self.target_pos = np.clip(self.target_pos,
                                  [c[0] for c in EE_POS_CLAMP],
                                  [c[1] for c in EE_POS_CLAMP])
        if gripper_cmd >= 0.0:
            self.gripper_target = (
                GRIPPER_OPEN if gripper_cmd >= 0.5 else GRIPPER_CLOSED)

    def _tick(self):
        # solve IK from the current (feedback) joint angles
        q = self.cur_joints.copy()
        err = 1e9
        for _ in range(IK_ITERATIONS):
            q, err = ik_step(PIPER_MDH, q, self.target_pos, self.target_R)
            if err < IK_ERROR_TOL:
                break

        cmd = JointsDataCmd()
        for i in range(6):
            cmd.data.joints_data[i].position = float(q[i])
            cmd.data.joints_data[i].velocity = 0.0
            cmd.data.joints_data[i].torque = 0.0
            cmd.data.joints_data[i].kp = ARM_KP
            cmd.data.joints_data[i].kd = ARM_KD
            cmd.data.joints_data[i].control_word = 4  # kIndexMotorControl
        for i in range(2):
            cmd.data.joints_data[6 + i].position = float(self.gripper_target[i])
            cmd.data.joints_data[6 + i].velocity = 0.0
            cmd.data.joints_data[6 + i].torque = 0.0
            cmd.data.joints_data[6 + i].kp = GRIPPER_KP
            cmd.data.joints_data[6 + i].kd = GRIPPER_KD
            cmd.data.joints_data[6 + i].control_word = 4
        self.arm_cmd_pub.publish(cmd)

        state = Float32MultiArray()
        state.data = [float(v) for v in
                      [self.target_pos[0], self.target_pos[1], self.target_pos[2],
                       self.target_quat[0], self.target_quat[1],
                       self.target_quat[2], self.target_quat[3]]]
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
