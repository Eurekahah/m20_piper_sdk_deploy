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
import os

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
# 增益取自当前训练的 actuator 配置（2026-10-08，B2-④/⑤）：
#   piper_arm     stiffness 300 / damping 8    （DEF-064：原 300/20）
#   piper_gripper stiffness 286 / damping 5    （DEF-058：原 4000/200）
# 为什么改（仿真逐档实测）：
#   * 臂 300/20：|qd| 到 5 rad/s 时阻尼力矩 20*5 = 100 N·m 吃满限幅 ⇒ 长期饱和
#     （逐档 |tau| 均 53.4 N·m、饱和 23.5%、原地抖）。300/8 ⇒ 22.5 N·m、饱和 0%、
#     跟踪还更好（4.14 -> 2.19 cm）。只降刚度是错的（150/8 跟踪退到 2.89 cm）。
#   * 夹爪 4000/200：位置环实际失效——饱和 88~100%、稳态误差最大 0.0314 rad。
#     286/5 = 按"满行程 0.035 rad 误差刚好给到 10 N·m 限幅"标定 ⇒ 饱和 0%、
#     上升时间 40~80 ms、稳态误差 <=0.0005（DEF-058）。
# 与 `hardware/piper_arm_interface.hpp` 的 ResetToDefault()、以及仿真侧
# `mujoco_simulation_ros2.py` 的 ARM_DEFAULT_KP/KD 必须一致。
ARM_KP, ARM_KD = 300.0, 8.0
GRIPPER_KP, GRIPPER_KD = 286.0, 5.0
GRIPPER_OPEN = np.array([0.035, -0.035])
GRIPPER_CLOSED = np.array([0.0, 0.0])

# 臂关节速度上限（DEF-065，2026-10-08 新增约束）：真机 Piper 关节 1~6 的速度上限
# 按 3 rad/s 设（joint6 本来就是 3）。仿真里启动事件 `align_joint_velocity_limits`
# 已把 PhysX 的 DOF max velocity 从 USD 的 5 改成 3，实测 |qd| p99 5.0 -> 3.00、
# 超速时间占比 33% -> 5~18%。真机上这条由 agx_arm_ctrl / 驱动器侧限制（本节点发的
# 是位置 PD，速度字段为 0），部署时不要放宽到 5。
ARM_VEL_LIMIT = 3.0

# 臂座相对机体（base_link）的安装偏移：M20_Piper_own.xml 里
# `arm_base_link pos = "0.24 0 0.0888"`（纯平移，两个坐标系轴向一致）。
ARM_BASE_OFFSET = np.array([0.24, 0.0, 0.0888], dtype=np.float64)

# 本节点整条链路（FK/IK/限位/目标）都在**臂基座坐标系**，
# 但策略观测里的 `ee_goal` 要的是 **root（机体）坐标系**的目标位姿
# （训练 `HeightInvariantEECommand.command_local`）。所以对外发布
# `/ARM_TELEOP_STATE` 时必须把臂座偏移加回去（DEF-020）。
#   M20_EE_GOAL_BODY_FRAME=1 -> 发布机体坐标系（与训练一致，默认）
#   M20_EE_GOAL_BODY_FRAME=0 -> 发布臂基座坐标系（旧行为，仅调试）
EE_GOAL_IN_BODY_FRAME = os.environ.get("M20_EE_GOAL_BODY_FRAME", "1") != "0"

# 被跟踪目标的限速（DEF-022）：键盘的增量是按 200 Hz 累加的（每 tick 5 mm），
# 一次按键按住 0.1 s 就能把目标推到工作空间边界 —— 观感是"一按就撞死、
# 之后不动了"。这里限制**被跟踪目标**的移动速度，输入怎么给都不会瞬移。
#   M20_EE_MAX_LIN_SPEED  0.35 m/s（键盘积分路径）
#   M20_EE_MAX_ANG_SPEED  0.8 rad/s
# VR 的绝对偏移路径另有更松的限速（M20_VR_MAX_LIN_SPEED / _ANG_SPEED）。
EE_MAX_LIN_SPEED = float(os.environ.get("M20_EE_MAX_LIN_SPEED", "0.35"))
EE_MAX_ANG_SPEED = float(os.environ.get("M20_EE_MAX_ANG_SPEED", "0.8"))
# VR 的绝对偏移路径：默认值 = 2026-10-10 操作员实测后确认的"手感合适"的一组
# （原来分别是 2.0 m/s / 5.0 rad/s，操作员反馈"太快"，实测每拍要走 4 cm）。
# 注意这是**被跟踪目标**的限速，不限制关节；关节侧由真机驱动器的 3 rad/s 兜底。
VR_MAX_LIN_SPEED = float(os.environ.get("M20_VR_MAX_LIN_SPEED", "0.4"))
VR_MAX_ANG_SPEED = float(os.environ.get("M20_VR_MAX_ANG_SPEED", "1.0"))
# 夹爪目标限速 [stroke/s]：0 = 不限速（直接阶跃到 ±0.035，仿真里 ~10 ms 就到底，
# 看着"瞬间开合"）。真机夹爪走完一个行程大约 0.3~0.5 s ⇒ 建议 0.15~0.2。
GRIPPER_SLEW = float(os.environ.get("M20_GRIPPER_SLEW", "0.0"))
# 关节指令限速 [rad/s]：0 = 不限速（现状，与训练侧一致）。
# 真机 Piper 驱动器会把关节速度限制在 ~3 rad/s（DEF-065），所以**设 3.0 更接近真机**：
# IK 在奇异位形附近会想一步跳 1~2 rad，限速后会被摊成可见的轨迹。
# ⚠️ 训练侧 DEF-054 的结论是"限制 **EE 目标** 的单步变化会让跟踪变差"；这里限制的是
# **关节指令**（更靠执行器一层），与驱动器限速同义，不会把目标本身改坏。
ARM_MAX_JOINT_VEL = float(os.environ.get("M20_ARM_MAX_JOINT_VEL", "0.0"))

EE_POS_CLAMP = ((-0.6, 0.6), (-0.6, 0.6), (0.02, 0.95))
IK_LAMBDA = 0.01
MAX_DQ_PER_STEP = 0.05
IK_ITERATIONS = 20
IK_ERROR_TOL = 1e-3


# ----------------------------------------------------------------------------
# quaternion helpers (wxyz convention, matching Isaac Lab)
# ----------------------------------------------------------------------------
# 训练侧（IsaacLab / PhysX）观测里的 `ee_goal` 四元数是**链式 FK 合成**的结果
# （`subtract_frame_transforms(root_quat, ee_quat)`），它的符号分支由父链连续决定；
# 而 `mat_to_quat(R)`（旋转矩阵→四元数）只会给出其中一支（大致 w>=0）。
# 两者是"同一个旋转、不同符号"，但**策略吃的是 4 个原始分量**——
# 默认站姿下训练值是 (-0.7373, 0, -0.6756, 0)，本文件下面的 mat_to_quat 给
# (+0.7373, 0, +0.6756, 0)。差一个符号 = 4 维输入全反 = 分布外输入，
# 实测在零速度命令下会把底盘推成 ~0.04 m/s 的持续前向漂移（DEF-039）。
# 所以：凡是把位姿发给策略观测的路径，都要用 align_quat_sign() 把符号拉回
# "训练那一支"并保持连续（不能只在第一帧对齐，姿态过 180° 时符号会跳）。
EE_DEFAULT_QUAT_TRAIN = np.array([-0.7373, 0.0, -0.6756, 0.0])   # 训练实测值


def align_quat_sign(q, ref=None):
    """把 q 翻到与 ref（缺省 = 训练默认分支）同一支上。

    q 与 -q 表示同一个旋转；调用方式：ref 传"上一帧发布出去的四元数"，
    就能保持发布序列的符号连续（等价于链式 FK 的分支）。
    """
    q = np.asarray(q, dtype=np.float64)
    ref = EE_DEFAULT_QUAT_TRAIN if ref is None else np.asarray(ref, dtype=np.float64)
    return -q if float(np.dot(q, ref)) < 0.0 else q


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


# ---------------------------------------------------------------------------
# IK 残差打印（P1-4 的验收手段）：每隔 M20_ARM_DEBUG_PERIOD 个 tick 打印一次
# "目标位姿 vs FK(当前关节角)" 的位置/姿态残差。
#   M20_ARM_DEBUG=1             打开（默认关）
#   M20_ARM_DEBUG_PERIOD=50     每 50 个 tick（=1 s，节点 50 Hz）打印一次
# 判据：稳态下位置残差应在 IK_ERROR_TOL(1e-3 m) 量级；被限位钳住时会变大，
# 这时要看日志里的 "reached limit" 提示，而不是当成 IK 坏了。
# ---------------------------------------------------------------------------
ARM_DEBUG = os.environ.get("M20_ARM_DEBUG", "0") != "0"
ARM_DEBUG_PERIOD = int(os.environ.get("M20_ARM_DEBUG_PERIOD", "50"))


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
        self.gripper_goal = GRIPPER_CLOSED.copy()
        # 上一拍真正下发的关节指令（M20_ARM_MAX_JOINT_VEL 限速用）
        self.q_cmd = DEFAULT_ARM_JOINTS.copy()

        # absolute EE target, initialised at the default arm pose
        self.target_pos, self.target_R = fk_pose(PIPER_MDH, DEFAULT_ARM_JOINTS)
        self.target_quat = mat_to_quat(self.target_R)
        # 发布给策略观测的 ee_goal 四元数：初始支 = 训练默认分支（见 align_quat_sign）
        self._last_pub_quat = EE_DEFAULT_QUAT_TRAIN.copy()
        # 请求位姿（键盘累加的目标）；被跟踪目标 target_* 以限速逼近它（DEF-022）
        self._req_pos = self.target_pos.copy()
        self._req_quat = self.target_quat.copy()
        self._last_input_mode = 0.0
        self._inc_pos_accum = np.zeros(3)
        self._inc_euler_accum = np.zeros(3)
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
        self.debug_cnt = 0
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
        self._last_input_mode = float(mode)   # 0=键盘增量 1=VR 绝对偏移（决定限速档）
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
            self._req_pos = pos.copy()
            self._req_quat = self.target_quat.copy()
            self.abs_anchor_initialized = False
        else:
            # 先把原始增量**累计**起来，真正的限速在 _tick 里按 50 Hz 做
            # （键盘/VR 是以 200 Hz 发增量的，每条消息各自限速等于没限 —— DEF-022）。
            self._inc_pos_accum = self._inc_pos_accum + np.array([dx, dy, dz],
                                                                dtype=np.float64)
            self._inc_euler_accum = self._inc_euler_accum + np.array(
                [droll, dpitch, dyaw], dtype=np.float64)

        self._clamp_and_gripper(gripper_cmd)

    def _apply_absolute_offset(self, dx, dy, dz, droll, dpitch, dyaw,
                               gripper_cmd, recalibrate):
        """
        VR semantics: target = anchor + controller offset.

        The anchor is captured from the actual robot pose on the first absolute
        message or whenever the operator presses recalibrate (B). Between
        recalibrations a stationary controller therefore keeps the target
        still, and hand drift is not integrated into the target.

        `recalibrate >= 1.5`（ARM_TELEOP_FLAG_RESET_DEFAULT）是**重置语义**：
        锚点直接取"默认关节姿态"对应的 FK 位姿、偏移按 0 处理 ⇒ 目标就是默认
        末端位姿。为什么不"锚到当前位姿"：复位后 arm_controller 手里还有旧目标，
        在它被换掉之前就会把臂从默认位姿拽走，这时采到的"当前位姿"是半路的
        （2026-10-10 实测：臂停在 j2=0.9 这种既非默认也非旧目标的位姿）。
        """
        if recalibrate >= 1.5:
            pos, R = fk_pose(PIPER_MDH, DEFAULT_ARM_JOINTS)
            self.anchor_pos = pos
            self.anchor_R = R
            self.anchor_quat = mat_to_quat(R)
            self.abs_anchor_initialized = True
            dx = dy = dz = droll = dpitch = dyaw = 0.0
        elif recalibrate > 0.5 or not self.abs_anchor_initialized:
            pos, R = fk_pose(PIPER_MDH, self.cur_joints)
            self.anchor_pos = pos
            self.anchor_R = R
            self.anchor_quat = mat_to_quat(R)
            self.abs_anchor_initialized = True

        self._req_pos = self.anchor_pos + np.array([dx, dy, dz])
        dq = quat_from_euler(droll, dpitch, dyaw)
        self._req_quat = quat_multiply(dq, self.anchor_quat)

        self._clamp_and_gripper(gripper_cmd)

    def _slew_toward(self, desired_pos, desired_R, max_lin, max_ang, dt):
        """把被跟踪目标往期望位姿走，每 tick 的位移/转角不超过限速。"""
        d = np.asarray(desired_pos, dtype=np.float64) - self.target_pos
        dist = float(np.linalg.norm(d))
        step = max_lin * dt
        if dist > step and dist > 1e-9:
            self.target_pos = self.target_pos + d * (step / dist)
        else:
            self.target_pos = np.asarray(desired_pos, dtype=np.float64).copy()

        # 姿态：用 quaternion 的夹角限制每 tick 的最大旋转
        q_des = mat_to_quat(desired_R)
        q_cur = self.target_quat
        q_err = quat_multiply(q_des, np.array([q_cur[0], -q_cur[1], -q_cur[2], -q_cur[3]]))
        ang = 2.0 * math.acos(float(np.clip(abs(q_err[0]), -1.0, 1.0)))
        max_ang_step = max_ang * dt
        if ang > max_ang_step and ang > 1e-9:
            t = max_ang_step / ang
            q_new = np.array([1.0, 0.0, 0.0, 0.0]) * (1.0 - t) + q_err * t
            q_new /= np.linalg.norm(q_new)
            self.target_quat = quat_multiply(q_new, q_cur)
        else:
            self.target_quat = q_des
        self.target_quat /= np.linalg.norm(self.target_quat)
        self.target_R = quat_to_mat(self.target_quat)

    def _clamp_and_gripper(self, gripper_cmd):
        self.target_pos = np.clip(self.target_pos,
                                  [c[0] for c in EE_POS_CLAMP],
                                  [c[1] for c in EE_POS_CLAMP])
        if gripper_cmd >= 0.0:
            # 只更新"目标"；真正的关节指令在 _tick 里按 M20_GRIPPER_SLEW 逼近，
            # 不然仿真里 10 ms 就开到顶/关到底，看着不像真夹爪。
            self.gripper_goal = (
                GRIPPER_OPEN if gripper_cmd >= 0.5 else GRIPPER_CLOSED)
            if GRIPPER_SLEW <= 0.0:
                self.gripper_target = self.gripper_goal.copy()

    def _tick(self):
        # solve IK from the current (feedback) joint angles
        # 先把"被跟踪目标"往请求位姿限速逼近（DEF-022）：键盘的增量是 200 Hz 累加的，
        # 不限速的话按一下就把目标甩到工作空间边界；VR 的绝对偏移路径用更松的限速。
        dt = 1.0 / 50.0
        # 键盘增量路径：把累计的原始增量按限速写进"请求位姿"
        if getattr(self, "_last_input_mode", 0.0) <= 0.5:
            step = EE_MAX_LIN_SPEED * dt
            n = float(np.linalg.norm(self._inc_pos_accum))
            if n > 1e-12:
                inc = self._inc_pos_accum * (min(1.0, step / n))
                self._req_pos = self._req_pos + inc
            ang_step = EE_MAX_ANG_SPEED * dt
            a = float(np.linalg.norm(self._inc_euler_accum))
            if a > 1e-12:
                e = self._inc_euler_accum * (min(1.0, ang_step / a))
                self._req_quat = quat_multiply(
                    quat_from_euler(e[0], e[1], e[2]), self._req_quat)
            self._inc_pos_accum = np.zeros(3)
            self._inc_euler_accum = np.zeros(3)
            self._req_pos = np.clip(self._req_pos,
                                    [c[0] for c in EE_POS_CLAMP],
                                    [c[1] for c in EE_POS_CLAMP])
        if getattr(self, "_last_input_mode", 0.0) > 0.5:
            self._slew_toward(self._req_pos, quat_to_mat(self._req_quat),
                              VR_MAX_LIN_SPEED, VR_MAX_ANG_SPEED, dt)
        else:
            self._slew_toward(self._req_pos, quat_to_mat(self._req_quat),
                              EE_MAX_LIN_SPEED, EE_MAX_ANG_SPEED, dt)
        q = self.cur_joints.copy()
        err = 1e9
        ik_iters = 0
        for _ in range(IK_ITERATIONS):
            ik_iters += 1
            q, err = ik_step(PIPER_MDH, q, self.target_pos, self.target_R)
            if err < IK_ERROR_TOL:
                break

        # IK 残差（目标 vs 用解出的关节角做 FK），用于 P1-4 的精度验收：
        # 位置残差用米、姿态残差用度。被工作空间/关节限位钳住时残差会变大，
        # 那种情况日志里同时会有 "reached limit" 提示（不是 IK 坏了）。
        if ARM_DEBUG and (self.debug_cnt % max(ARM_DEBUG_PERIOD, 1) == 0):
            fk_pos, fk_R = fk_pose(PIPER_MDH, q)
            pos_err = float(np.linalg.norm(fk_pos - self.target_pos))
            R_delta = fk_R @ self.target_R.T
            cos_a = float(np.clip((np.trace(R_delta) - 1.0) / 2.0, -1.0, 1.0))
            ang_err = float(np.degrees(math.acos(cos_a)))
            print("[arm_ik] target=(%.4f %.4f %.4f) pos_err=%.5f m ang_err=%.3f deg "
                  "(ik_iters %d)" % (self.target_pos[0], self.target_pos[1],
                                     self.target_pos[2], pos_err, ang_err, ik_iters))
        self.debug_cnt += 1

        # 夹爪目标限速（可选）：按 M20_GRIPPER_SLEW 逼近目标。
        # 不做的话位置目标直接阶跃 ±0.035，仿真里 kp=286/kd=5 + armature=0.01
        # 的带宽约 170 rad/s ⇒ 10~20 ms 就到底，肉眼看就是"瞬间开合"（真夹爪要 0.3~0.5 s）。
        if GRIPPER_SLEW > 0.0:
            step = GRIPPER_SLEW * dt
            d = self.gripper_goal - self.gripper_target
            self.gripper_target = self.gripper_target + np.clip(d, -step, step)

        # 关节指令限速（可选，M20_ARM_MAX_JOINT_VEL>0 时生效）：
        # IK 在奇异位形附近会给出单拍 1~2 rad 的跳变，真机驱动器会在 3 rad/s 处截住，
        # 仿真里不限速就会"啪"一下过去（看着不像电机）。这里按同样的口径摊开。
        if ARM_MAX_JOINT_VEL > 0.0:
            lim = ARM_MAX_JOINT_VEL * dt
            q = self.q_cmd + np.clip(q - self.q_cmd, -lim, lim)
        self.q_cmd = np.asarray(q, dtype=np.float64).copy()

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

        # 策略观测用的 ee_goal：默认按**机体坐标系**发布（把臂座偏移加回去），
        # 否则策略会拿到一个落在机体内部的 EE 目标（差 24 cm，DEF-020）。
        goal_pos = (self.target_pos + ARM_BASE_OFFSET
                    if EE_GOAL_IN_BODY_FRAME else self.target_pos)
        # 符号对齐到训练（链式 FK）那一支，并保持逐帧连续（DEF-039）
        q_pub = align_quat_sign(self.target_quat, self._last_pub_quat)
        self._last_pub_quat = q_pub
        state = Float32MultiArray()
        state.data = [float(v) for v in
                      [goal_pos[0], goal_pos[1], goal_pos[2],
                       q_pub[0], q_pub[1], q_pub[2], q_pub[3]]]
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT 时 rclpy 的信号处理可能已经把 context shutdown 过了，
        # 再调一次会抛 RCLError（退出码变非 0，测试脚本会误判成崩溃）。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
