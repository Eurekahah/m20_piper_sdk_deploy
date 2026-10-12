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

# ---------------------------------------------------------------------------
# VR 绝对偏移的**映射方式**（2026-10-12，DEF-044）
#
# 为什么不能直接用"笛卡尔绝对偏移"（= 旧行为、训练侧语义）：
# 默认站姿 `(0, 0.5, -0.5, 0, 0, 0)` 里 joint5 = joint6 = 0，joint4 与 joint6 同轴
# ⇒ 手腕正好落在**奇异位形**（Jacobian 最小奇异值 ~4e-18，条件数 1.8e12）。
# 这个位形下"EE 侧移 5 cm 而工具姿态不变"或"工具转角 10°"在关节空间里**没有近邻解**：
# 用约束优化求离当前姿态最近的严格解，侧移 5 cm 需要 2.67 rad、侧移 2 cm 也要
# 2.40 rad、工具 yaw 10° 需要 2.47 rad 的关节重排（作为对照：键盘的 +x 平移只要
# 0.30 rad）。局部 DLS 单拍最多走 20×0.05 = 1.0 rad，于是要么卡住、要么跳到远处
# 分支（实测臂端被顶到限位：joint1 = 2.168、joint4 = 1.745、joint5 = ±1.22），
# 关节速度 ~20 rad/s、指令力矩 ~139 N·m（真机驱动器 3 rad/s / 100 N·m）。
#
#   实测（2026-10-12，sim2sim，腿在走 + VR 手画圆）：
#     映射                         IK 残差 med/p95/max      顶限位
#     笛卡尔（旧，训练侧语义）      59.2 / 125.5 / 128.7 mm   有
#     柱坐标 + 手腕关节偏移（本实现） 0.4 /   1.6 /   2.0 mm   无
#
# 新映射（默认 `cylindrical`）做两件事：
#   1) 位置：把手部平移从"笛卡尔绝对偏移"改成**柱坐标**
#        前/后  -> 半径 r（沿臂当前朝向伸缩）
#        左/右  -> 基座 yaw Δθ（臂自然摆过去，工具跟着摆 —— 这才是真机的自然动作）
#        上/下  -> z
#   2) 姿态：把 VR 的三个旋转偏移当**手腕关节偏移**下发（j4/j5/j6），
#      于是它们永远可达、永远便宜（工具自转就是 joint6 一个自由度）。
#      顺带把"臂摆过 Δθ"体现在目标姿态上（Rz(Δθ)·R_anchor），所以工具跟着臂走、
#      不要求手腕去补偿整条臂的摆动。
#
# `M20_VR_ARM_MAP=cartesian` 可以退回旧行为做 A/B 或回滚（不推荐，原因见上）。
VR_ARM_MAP = os.environ.get("M20_VR_ARM_MAP", "cylindrical").strip().lower()
VR_SWING_MAX = float(os.environ.get("M20_VR_SWING_MAX_DEG", "60")) * math.pi / 180.0
VR_RADIUS_MIN = float(os.environ.get("M20_VR_RADIUS_MIN", "0.12"))

# 夹爪目标限速 [stroke/s]：0 = 不限速（直接阶跃到 ±0.035，仿真里 ~10 ms 就到底，
# 看着"瞬间开合"）。真机夹爪走完一个行程大约 0.3~0.5 s ⇒ 建议 0.15~0.2。
GRIPPER_SLEW = float(os.environ.get("M20_GRIPPER_SLEW", "0.0"))
# 关节指令限速 [rad/s]：0 = 不限速。默认 **3.0 = 真机 Piper 驱动器的口径**（DEF-065）：
# IK 在奇异位形附近会想一步跳 1~2 rad，限速后会被摊成可见的轨迹，而不是"啪"一下。
# 这也让 sim2sim 的关节速度不会超过真机能跟上的范围（实测失败时曾到 20 rad/s）。
# ⚠️ 训练侧 DEF-054 的结论是"限制 **EE 目标** 的单步变化会让跟踪变差"；这里限制的是
# **关节指令**（更靠执行器一层），与驱动器限速同义，不会把目标本身改坏。
ARM_MAX_JOINT_VEL = float(os.environ.get("M20_ARM_MAX_JOINT_VEL", "3.0"))

# IK 收敛检查（DEF-044）：解不出来就**不下发**（沿用上一拍指令 + 把被跟踪目标拉回
# 实际位姿），而不是把"顶在限位上的解"当命令发出去 —— 那正是"臂乱甩"的来源。
#   M20_ARM_IK_MAX_POS_ERR  位置残差上限 [m]（默认 10 mm）
#   M20_ARM_IK_MAX_ANG_ERR  姿态残差上限 [deg]（默认 3°）
# 实测（新旧映射对照）：新映射下 10 Hz 采样的残差 p95 = 1.6 mm，所以这两条只在
# 真的把目标推出工作空间时才触发。
IK_REJECT_POS_ERR = float(os.environ.get("M20_ARM_IK_MAX_POS_ERR", "0.010"))
IK_REJECT_ANG_ERR = float(os.environ.get("M20_ARM_IK_MAX_ANG_ERR", "3.0")) * math.pi / 180.0

# IK 的姿态正则（零空间项）权重：只压"不影响任务但会让手腕漂到限位"的分量，
# 参考位姿 = 当前锚点的关节角。0 = 关掉（旧行为，会漂）。见 ik_step 的说明。
IK_POSTURE_GAIN = float(os.environ.get("M20_ARM_IK_POSTURE_GAIN", "0.3"))

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


def rot_z(a):
    """3x3 rotation about z (used to swing the arm in the cylindrical map)."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def vr_absolute_target(anchor_pos, anchor_R, anchor_joints, offsets):
    """把 VR 的 (dx,dy,dz,droll,dpitch,dyaw) 变成目标位姿（DEF-044）。

    `cylindrical`（默认）：
      * 位置：以臂座 z 轴为轴的柱坐标
          dx -> 半径增量（沿当前朝向伸缩）
          dy -> 基座摆动 Δθ = atan2(dy, r)（夹在 ±M20_VR_SWING_MAX_DEG）
          dz -> 高度
      * 姿态：目标 = Rz(Δθ) · FK(anchor_joints + 手腕偏移)，其中手腕偏移把
        (droll, dpitch, dyaw) 当作 (joint6, joint5, joint4) 的增量
        —— 即"手自转 -> 工具自转、手俯仰 -> 手腕俯仰、手侧倾 -> 前臂滚转"。
        这样三个旋转分量永远落在手腕自己的自由度上（永远可达、代价很小），
        而工具的姿态跟着臂的摆动走，不需要手腕去补偿整条臂。

    `cartesian`（旧行为，训练侧语义，用 M20_VR_ARM_MAP=cartesian 打开）：
      位置 = anchor + (dx,dy,dz)；姿态 = R(offset) · anchor_R。
      注意这个映射在默认站姿（手腕奇异位形）上会要求 2.4~2.7 rad 的关节重排。
    """
    dx, dy, dz, droll, dpitch, dyaw = [float(v) for v in offsets]
    ax, ay, az = float(anchor_pos[0]), float(anchor_pos[1]), float(anchor_pos[2])

    if VR_ARM_MAP == "cartesian":
        pos = np.array([ax + dx, ay + dy, az + dz], dtype=np.float64)
        dR = quat_to_mat(quat_from_euler(droll, dpitch, dyaw))
        return pos, dR @ anchor_R

    r_a = math.hypot(ax, ay)
    th_a = math.atan2(ay, ax)
    r = max(r_a + dx, VR_RADIUS_MIN)
    dth = max(-VR_SWING_MAX, min(VR_SWING_MAX, math.atan2(dy, r)))
    th = th_a + dth
    pos = np.array([r * math.cos(th), r * math.sin(th), az + dz], dtype=np.float64)

    q_w = np.asarray(anchor_joints, dtype=np.float64).copy()
    q_w[3] += dyaw        # 前臂滚转
    q_w[4] += dpitch      # 手腕俯仰
    q_w[5] += droll       # 工具自转
    q_w = np.clip(q_w, ARM_JOINT_LIMITS[:, 0], ARM_JOINT_LIMITS[:, 1])
    _, R_w = fk_pose(PIPER_MDH, q_w)
    return pos, rot_z(dth) @ R_w


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


def ik_step(mdh, q, target_pos, target_R, q_nom=None, posture_gain=0.0):
    """One DLS iteration; returns (new_q, error_norm).

    `q_nom` + `posture_gain` 是可选的**姿态正则**（DEF-044）：把"零空间里不影响
    任务、但会让手腕慢慢拧到限位"的分量拉回 q_nom。默认姿态下手腕接近奇异
    （joint4 与 joint6 同轴），这个自由方向在数值上收益极小、却会随每拍累积，
    实测（新映射）臂在 ~10 s 内把 joint4 拧到 +1.745、joint6 拧到 -2.094（都是限位）。
    给一点正则就能钉住（权重要小，只压自由方向，不干扰正常跟踪）。
    """
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

    Jy = np.linalg.solve(J @ J.T + IK_LAMBDA ** 2 * np.eye(6), J)
    dq = J.T @ np.linalg.solve(J @ J.T + IK_LAMBDA ** 2 * np.eye(6), err)
    if q_nom is not None and posture_gain > 0.0:
        # 零空间投影：(I - J⁺J)(q_nom - q)，J⁺ = Jᵀ(JJᵀ+λ²I)⁻¹
        dq = dq + posture_gain * (np.eye(6) - J.T @ Jy) @ (np.asarray(q_nom) - q)
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
        self._bad_fb_warned = False
        self._ik_fail_warned = False
        self._tick_fail_warned = False
        self.ik_rejects = 0          # IK 未收敛（没下发）的拍数，DEF-044

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
        # 锚点对应的**关节角**：柱坐标映射要用它当手腕偏移的基准（DEF-044）
        self.anchor_joints = DEFAULT_ARM_JOINTS.copy()
        self.abs_anchor_initialized = False

        self.get_logger().info(
            "[arm_controller] started (MDH FK + DLS IK, 50 Hz)")

    def _data_cb(self, msg):
        if len(msg.data.joints_data) >= 8:
            q = np.array([msg.data.joints_data[i].position for i in range(8)])
            # 只接受**有限**的反馈：臂侧（或驱动器）在异常/复位瞬间可能给 NaN/Inf，
            # 一旦进到 IK 里会让 np.linalg.solve 抛 LinAlgError ⇒ spin() 退出 ⇒
            # 整个节点"掉了"（表现为终端里一条 traceback 就没了）。这里直接丢弃坏帧。
            if not np.all(np.isfinite(q)):
                if not self._bad_fb_warned:
                    self._bad_fb_warned = True
                    self.get_logger().warn(
                        "[arm_controller] 收到非有限的关节反馈（NaN/Inf），已丢弃该帧；"
                        "后续再出现不会重复提示")
                return
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
            self.anchor_joints = DEFAULT_ARM_JOINTS.copy()
            self.abs_anchor_initialized = True
            dx = dy = dz = droll = dpitch = dyaw = 0.0
        elif recalibrate > 0.5 or not self.abs_anchor_initialized:
            pos, R = fk_pose(PIPER_MDH, self.cur_joints)
            self.anchor_pos = pos
            self.anchor_R = R
            self.anchor_quat = mat_to_quat(R)
            self.anchor_joints = np.asarray(self.cur_joints, dtype=np.float64).copy()
            self.abs_anchor_initialized = True

        # 目标位姿由映射算出来（DEF-044：默认柱坐标 + 手腕关节偏移，
        # 见 vr_absolute_target 的说明；M20_VR_ARM_MAP=cartesian 可退回旧行为）
        req_pos, req_R = vr_absolute_target(
            self.anchor_pos, self.anchor_R, self.anchor_joints,
            (dx, dy, dz, droll, dpitch, dyaw))
        self._req_pos = np.clip(req_pos,
                                [c[0] for c in EE_POS_CLAMP],
                                [c[1] for c in EE_POS_CLAMP])
        self._req_quat = mat_to_quat(req_R)

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
        # 最后一道防线：单拍里任何未预料的异常都不许把节点带走。
        # 在 rclpy 里，定时器回调抛异常会直接从 spin() 冒出来 ⇒ 进程带着一条 traceback
        # 退出（操作员看到的就是"IK 那个终端掉了"）。这里吞掉并限频报错。
        try:
            self._tick_impl()
        except Exception as exc:   # noqa: BLE001
            if not self._tick_fail_warned:
                self._tick_fail_warned = True
                self.get_logger().error(
                    "[arm_controller] 控制拍异常（%s: %s），已跳过本拍且**不会退出**；"
                    "后续再出现不会重复提示" % (type(exc).__name__, exc))

    def _tick_impl(self):
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
        try:
            # VR 路径用锚点姿态做零空间正则，钉住"自由方向"上的手腕漂移；
            # 键盘路径保持原样（它的运动本来就在良态方向，不需要）。
            pos_q, pos_gain = ((self.anchor_joints, IK_POSTURE_GAIN)
                               if getattr(self, "_last_input_mode", 0.0) > 0.5
                               else (None, 0.0))
            for _ in range(IK_ITERATIONS):
                ik_iters += 1
                q, err = ik_step(PIPER_MDH, q, self.target_pos, self.target_R,
                                 q_nom=pos_q, posture_gain=pos_gain)
                if err < IK_ERROR_TOL:
                    break
        except Exception as exc:   # noqa: BLE001
            # 兜底：IK 内部任何异常都**不能**把节点带走（回调里抛异常会让 rclpy.spin
            # 直接退出，终端"啪"一下没了 —— 真机部署里这是最糟的失败模式）。
            # 这一拍沿用上一拍的好指令保持住，并限频提示。
            if not self._ik_fail_warned:
                self._ik_fail_warned = True
                self.get_logger().error(
                    "[arm_controller] IK 异常（%s: %s），本拍沿用上一拍指令保持；"
                    "后续再出现不会重复提示" % (type(exc).__name__, exc))
            q = self.q_cmd.copy()

        # IK 残差（目标 vs 用解出的关节角做 FK）：位置用米、姿态用度。
        # DEF-044：残差**每拍都算**，不只是调试时打印 —— 它是"这个解到底能不能下发"
        # 的判据（下面的收敛检查要用）。
        fk_pos, fk_R = fk_pose(PIPER_MDH, q)
        pos_err = float(np.linalg.norm(fk_pos - self.target_pos))
        R_delta = fk_R @ self.target_R.T
        cos_a = float(np.clip((np.trace(R_delta) - 1.0) / 2.0, -1.0, 1.0))
        ang_err = float(math.acos(cos_a))

        # 收敛检查（DEF-044）：没收敛就**不下发**。
        # 旧行为是把 `ik_step()` 最后一组（已经被 np.clip 钳到关节限位上的）关节角直接
        # 当命令发出去 —— 实测 arm 被顶到 joint1 = 2.168（= 上限）、joint4 = 1.745、
        # joint5 = ±1.22，关节速度 20 rad/s、指令力矩 139 N·m，操作员看到的就是
        # "臂自己乱甩/转过去了"。现在：沿用上一拍指令，并把**被跟踪目标拉回实际位姿**
        # （否则请求会一直躺在不可达的地方，下一拍继续失败），限频告警。
        if pos_err > IK_REJECT_POS_ERR or ang_err > IK_REJECT_ANG_ERR:
            self.ik_rejects += 1
            if (not ARM_DEBUG) and self.ik_rejects in (1, 50, 500):
                self.get_logger().warn(
                    "[arm_controller] IK 未收敛（残差 %.1f mm / %.1f deg > 阈值 %.0f mm / %.0f deg）"
                    "→ 本拍沿用上一拍指令；累计 %d 拍。目标已拉回实际位姿，"
                    "请把目标移回工作空间内（M20_ARM_IK_MAX_POS_ERR/_ANG_ERR 可调）"
                    % (pos_err * 1000.0, math.degrees(ang_err),
                       IK_REJECT_POS_ERR * 1000.0, math.degrees(IK_REJECT_ANG_ERR),
                       self.ik_rejects))
            q = self.q_cmd.copy()
            hold_pos, hold_R = fk_pose(PIPER_MDH, q)
            self.target_pos = hold_pos
            self.target_R = hold_R
            self.target_quat = mat_to_quat(hold_R)
            fk_pos, fk_R = hold_pos, hold_R
            pos_err = float(np.linalg.norm(fk_pos - self.target_pos))
            cos_a = float(np.clip((np.trace(fk_R @ self.target_R.T) - 1.0) / 2.0, -1.0, 1.0))
            ang_err = float(math.acos(cos_a))

        if ARM_DEBUG and (self.debug_cnt % max(ARM_DEBUG_PERIOD, 1) == 0):
            print("[arm_ik] target=(%.4f %.4f %.4f) pos_err=%.5f m ang_err=%.3f deg "
                  "(ik_iters %d, rejects %d)"
                  % (self.target_pos[0], self.target_pos[1], self.target_pos[2],
                     pos_err, math.degrees(ang_err), ik_iters, self.ik_rejects))
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
