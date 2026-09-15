"""
 * @file mujoco_simulation_ros2.py
 * @brief simulation in mujoco for M20 + AgileX Piper arm (sim2sim deploy)
 *
 *  The M20 legs (16 DOF) are driven by the RL policy through the official
 *  /JOINTS_CMD topic, the Piper arm (6 joints + 2 gripper) is driven by the
 *  separate arm_controller node through /ARM_JOINTS_CMD.
 *
 *  Topics:
 *    subscribe /JOINTS_CMD     drdds/msg/JointsDataCmd   (16 M20 leg joints)
 *    subscribe /ARM_JOINTS_CMD drdds/msg/JointsDataCmd   (8 arm/gripper joints)
 *    publish   /JOINTS_DATA    drdds/msg/JointsData      (16 M20 leg joints)
 *    publish   /ARM_JOINTS_DATA drdds/msg/JointsData     (8 arm/gripper joints)
 *    publish   /IMU_DATA       drdds/msg/ImuData
"""

import os
import time
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_srvs.srv import Empty
from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd


MODEL_NAME = "M20_Piper"
# Get the directory of the current Python file
CURRENT_DIR = Path(__file__).resolve().parent

# Define the XML path relative to the Python file
XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "M20_Piper_description" / "mjcf" / "M20_Piper_own.xml"

# Convert to absolute path as string
XML_PATH = str(XML_PATH.resolve())
USE_VIEWER = os.environ.get("M20_USE_VIEWER", "1") == "1"
# 1 Hz dump of the 16 leg/wheel joint velocities (sim ground truth). The wheel
# joints are velocity-controlled in RL (kp 0 / kd 0.6), so their qvel is the
# first place a sign/scale/limit mismatch shows up. Set M20_JVEL_DEBUG=0 to mute.
JVEL_DEBUG = os.environ.get("M20_JVEL_DEBUG", "1") != "0"
JVEL_PERIOD = 1000           # control ticks (1 ms) -> 1 s
DT = 0.001
RENDER_INTERVAL = 50
# The USD-derived M20_Piper MJCF has a lightly damped 500 Hz rocking mode
# that is marginally unstable at a 1 ms integrator step (the base gyro sees
# +-2 rad/s alternating values that never appear in the pose). Integrate the
# physics at 0.2 ms in batches that still advance 1 ms per control tick, so
# topic rates and policy timing stay unchanged.
PHYSICS_DT = 0.0002
SUBSTEPS = int(round(DT / PHYSICS_DT))
assert abs(PHYSICS_DT * SUBSTEPS - DT) < 1e-12

# ----------------------------------------------------------------------------
# DOF layout
# ----------------------------------------------------------------------------
LEG_DOF = 16                 # M20 legs (12 hip/leg + 4 wheels)
ARM_DOF = 8                  # Piper arm (arm_joint1..6) + gripper (2)
TOTAL_DOF = LEG_DOF + ARM_DOF
WHEEL_RADIUS = 0.09          # used to auto-place the robot on the ground

LEG_CMD_TOPIC = "/JOINTS_CMD"
LEG_DATA_TOPIC = "/JOINTS_DATA"
ARM_CMD_TOPIC = "/ARM_JOINTS_CMD"
ARM_DATA_TOPIC = "/ARM_JOINTS_DATA"
IMU_TOPIC = "/IMU_DATA"

# ----------------------------------------------------------------------------
# Calibration parameters for the 16 M20 leg joints in sim2sim.
# The M20_Piper MJCF is generated from the same URDF Isaac Lab trains on, so
# the raw MJCF frame IS the policy frame: use identity calibration (dir=1,
# offset=0) together with the M20SimInterface in rl_deploy (SIM2SIM build).
# ----------------------------------------------------------------------------
JOINT_DIR = np.ones(LEG_DOF, dtype=np.float32)
POS_OFFSET_DEG = np.zeros(LEG_DOF, dtype=np.float32)
POS_OFFSET_RAD = POS_OFFSET_DEG / 180.0 * np.pi

# The arm/gripper joints are raw rad (dir=1, offset=0) on both sides.
ARM_DIR = np.ones(ARM_DOF, dtype=np.float32)
ARM_OFFSET_RAD = np.zeros(ARM_DOF, dtype=np.float32)

# Initial pose: M20 legs start at the Isaac Lab default standing pose
# (hipy +/-0.6, knee -/+1.0 -> base height ~0.53 with wheels on the ground),
# the Piper arm starts at its default (arm2=0.5, arm3=-0.5, gripper closed).
LEG_INIT = {
    "M20": np.array([-0.438, -1.16, 2.76, 0,
                     0.438, -1.16, 2.76, 0,
                     -0.438, 1.16, -2.76, 0,
                     0.438, 1.16, -2.76, 0], dtype=np.float32),
    "M20_Piper_own": np.array([0.0, -0.6, 1.0, 0,
                                0.0, -0.6, 1.0, 0,
                                0.0, 0.6, -1.0, 0,
                                0.0, 0.6, -1.0, 0], dtype=np.float32)
}
ARM_INIT = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
JOINT_INIT = np.concatenate([LEG_INIT["M20_Piper_own"], ARM_INIT]).astype(np.float32)

# Default arm hold (used until the first /ARM_JOINTS_CMD message arrives, so the
# arm does not flop around during idle/standup). Gains match the Isaac Lab
# actuator config (piper_arm: stiffness 40 / damping 8 / armature 0.01,
# piper_gripper: stiffness 4000 / damping 200).
ARM_DEFAULT_KP = np.array([40.0] * 6 + [4000.0, 4000.0], dtype=np.float32)
ARM_DEFAULT_KD = np.array([8.0] * 6 + [200.0, 200.0], dtype=np.float32)


class MuJoCoSimulationNode(Node):
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str = XML_PATH):

        super().__init__('mujoco_simulation')

        # 加载 MJCF
        if not os.path.isfile(xml_path):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.model.opt.timestep = PHYSICS_DT
        self.data = mujoco.MjData(self.model)

        # 机器人自由度列表
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..23
        self.dof_num = len(self.actuator_ids)
        assert self.dof_num == TOTAL_DOF, f"Expected {TOTAL_DOF} DOF for M20_Piper, got {self.dof_num}"

        # 初始化站立姿态
        self._set_initial_pose()
        # 缓存 (legs)
        # Hold the initial (policy-default) pose from the very first step, so the
        # robot does not collapse while rl_deploy is still starting up (the real
        # robot is also held before the SDK takes over).
        LEG_HOLD_KP = np.tile(np.array([80., 80., 80., 10.], dtype=np.float32), 4)
        LEG_HOLD_KD = np.tile(np.array([2., 2., 2., 0.6], dtype=np.float32), 4)
        self.kp_cmd = LEG_HOLD_KP.reshape(-1, 1)
        self.kd_cmd = LEG_HOLD_KD.reshape(-1, 1)
        self.pos_cmd = LEG_INIT["M20"].reshape(-1, 1)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)

        # 缓存 (arm/gripper)
        self.arm_kp_cmd = np.zeros((ARM_DOF, 1), np.float32)
        self.arm_kd_cmd = np.zeros_like(self.arm_kp_cmd)
        self.arm_pos_cmd = np.zeros_like(self.arm_kp_cmd)
        self.arm_vel_cmd = np.zeros_like(self.arm_kp_cmd)
        self.arm_tau_ff = np.zeros_like(self.arm_kp_cmd)
        self.arm_cmd_valid = False   # until first /ARM_JOINTS_CMD message
        self.step_count_ = 0

        self.input_tq = np.zeros((TOTAL_DOF, 1), np.float32)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0

        self.get_logger().info(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        # ROS Publishers
        self.imu_pub = self.create_publisher(ImuData, IMU_TOPIC, 200)
        self.joints_pub = self.create_publisher(JointsData, LEG_DATA_TOPIC, 200)
        self.arm_joints_pub = self.create_publisher(JointsData, ARM_DATA_TOPIC, 200)

        # ROS Subscribers
        self.cmd_sub = self.create_subscription(
            JointsDataCmd,
            LEG_CMD_TOPIC,
            self._cmd_callback,
            50
        )
        self.arm_cmd_sub = self.create_subscription(
            JointsDataCmd,
            ARM_CMD_TOPIC,
            self._arm_cmd_callback,
            50
        )

        # 重置服务：把仿真恢复到初始位姿并清空所有命令缓冲。
        # 调用示例: ros2 service call /reset_sim std_srvs/srv/Empty
        self.reset_srv = self.create_service(Empty, 'reset_sim', self._reset_callback)

        # 可视化
        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    # ------------------------------------------------------------------------
    def _wheel_geom_ids(self):
        ids = []
        for i in range(self.model.ngeom):
            if self.model.geom(i).name.endswith("_wheel_collision"):
                ids.append(i)
        return ids

    def _set_initial_pose(self):
        """Set the joint angles and place the robot so the wheels touch ground."""
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + TOTAL_DOF] = JOINT_INIT
        qpos0[:3] = np.array([0, 0, 0.0])
        qpos0[3:7] = np.array([1, 0, 0, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

        # shift the base so the lowest wheel center sits at WHEEL_RADIUS
        wheel_ids = self._wheel_geom_ids()
        lowest_z = float(np.min(self.data.geom_xpos[wheel_ids, 2]))
        self.data.qpos[2] += WHEEL_RADIUS - lowest_z
        mujoco.mj_forward(self.model, self.data)

    def _reset_callback(self, request, response):
        """Reset the simulation to the initial standing pose and clear commands."""
        mujoco.mj_resetData(self.model, self.data)
        self._set_initial_pose()
        self.data.qvel[:] = 0.0

        # legs: 回到默认站立保持命令
        LEG_HOLD_KP = np.tile(np.array([80., 80., 80., 10.], dtype=np.float32), 4)
        LEG_HOLD_KD = np.tile(np.array([2., 2., 2., 0.6], dtype=np.float32), 4)
        self.kp_cmd = LEG_HOLD_KP.reshape(-1, 1)
        self.kd_cmd = LEG_HOLD_KD.reshape(-1, 1)
        self.pos_cmd = LEG_INIT["M20"].reshape(-1, 1)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)

        # arm/gripper: 回到默认位姿保持，等待新的 /ARM_JOINTS_CMD
        self.arm_kp_cmd = np.zeros((ARM_DOF, 1), np.float32)
        self.arm_kd_cmd = np.zeros_like(self.arm_kp_cmd)
        self.arm_pos_cmd = np.zeros_like(self.arm_kp_cmd)
        self.arm_vel_cmd = np.zeros_like(self.arm_kp_cmd)
        self.arm_tau_ff = np.zeros_like(self.arm_kp_cmd)
        self.arm_cmd_valid = False

        if self.viewer is not None:
            self.viewer.sync()

        self.get_logger().info("[reset_sim] simulation reset to initial pose")
        return response

    # ------------------------------------------------------------------------
    def _cmd_callback(self, msg: JointsDataCmd):
        """Leg joint commands from rl_deploy (published in robot frame)."""
        if len(msg.data.joints_data) != LEG_DOF:
            self.get_logger().warn("Received JointsDataCmd with incorrect number of leg joints")
            return
        pub_pos = np.zeros(LEG_DOF, dtype=np.float32)
        pub_vel = np.zeros(LEG_DOF, dtype=np.float32)
        for i in range(LEG_DOF):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = joint_cmd.kp
            self.kd_cmd[i] = joint_cmd.kd
            pub_pos[i] = joint_cmd.position
            pub_vel[i] = joint_cmd.velocity
            self.tau_ff[i] = joint_cmd.torque  # tau_ff no processing

        # Convert: raw = published * dir + offset_rad
        self.pos_cmd.flat = pub_pos * JOINT_DIR + POS_OFFSET_RAD
        self.vel_cmd.flat = pub_vel * JOINT_DIR

    def _arm_cmd_callback(self, msg: JointsDataCmd):
        """Arm/gripper joint commands from arm_controller (raw rad)."""
        if len(msg.data.joints_data) < ARM_DOF:
            self.get_logger().warn("Received JointsDataCmd with fewer than 8 arm joints")
            return

        pub_pos = np.zeros(ARM_DOF, dtype=np.float32)
        pub_vel = np.zeros(ARM_DOF, dtype=np.float32)
        for i in range(ARM_DOF):
            joint_cmd = msg.data.joints_data[i]
            self.arm_kp_cmd[i] = joint_cmd.kp
            self.arm_kd_cmd[i] = joint_cmd.kd
            pub_pos[i] = joint_cmd.position
            pub_vel[i] = joint_cmd.velocity
            self.arm_tau_ff[i] = joint_cmd.torque

        self.arm_pos_cmd.flat = pub_pos * ARM_DIR + ARM_OFFSET_RAD
        self.arm_vel_cmd.flat = pub_vel * ARM_DIR
        self.arm_cmd_valid = True

    def start(self):
        # 主模拟循环
        step = 0
        last_time = time.time()
        while rclpy.ok():
            if time.time() - last_time >= DT:
                last_time = time.time()
                step += 1
                self.step_count_ += 1
                # 每个 1ms 控制 tick 内做 5 个 0.2ms 物理子步
                for _ in range(SUBSTEPS):
                    self._apply_joint_torque()
                    mujoco.mj_step(self.model, self.data)

                self.timestamp = step * DT

                # 采样 & 发送观测 (every 5 control ticks for 200 Hz)
                if step % 5 == 0:
                    self._publish_robot_state(step)

                # 关节速度打印 (1 Hz)
                if JVEL_DEBUG and step % JVEL_PERIOD == 0:
                    self._print_leg_wheel_velocity()

                # 可视化
                if self.viewer and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()

            # Handle ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.0)

    def _apply_joint_torque(self):
        # 当前关节状态
        q = self.data.qpos[7:7 + TOTAL_DOF].reshape(-1, 1)
        dq = self.data.qvel[6:6 + TOTAL_DOF].reshape(-1, 1)

        # legs
        self.input_tq[:LEG_DOF] = (
                self.kp_cmd * (self.pos_cmd - q[:LEG_DOF]) +
                self.kd_cmd * (self.vel_cmd - dq[:LEG_DOF]) +
                self.tau_ff
        )

        # arm/gripper: use the default hold until the first command arrives
        arm_q = q[LEG_DOF:]
        arm_dq = dq[LEG_DOF:]
        if self.arm_cmd_valid:
            kp = self.arm_kp_cmd
            kd = self.arm_kd_cmd
            pos = self.arm_pos_cmd
            vel = self.arm_vel_cmd
            tau = self.arm_tau_ff
        else:
            kp = ARM_DEFAULT_KP.reshape(-1, 1)
            kd = ARM_DEFAULT_KD.reshape(-1, 1)
            pos = ARM_INIT.reshape(-1, 1)
            vel = np.zeros_like(arm_q)
            tau = np.zeros_like(arm_q)
        self.input_tq[LEG_DOF:] = kp * (pos - arm_q) + kd * (vel - arm_dq) + tau

        # 写入 control 缓冲区
        self.data.ctrl[:] = self.input_tq.flatten()

    # --------------------------------------------------------
    def _print_leg_wheel_velocity(self):
        """1 Hz dump of the 16 leg/wheel joint velocities (sim ground truth).

        Wheel joints are the only ones in velocity mode (kp 0 / kd 0.6, target
        from the policy), so their qvel is what feeds the policy as
        joint_vel[12:16] and is the first place a frame/sign/limit error shows
        up. The value published on /JOINTS_DATA is identical here because
        JOINT_DIR is 1 in sim2sim.
        """
        q = self.data.qpos[7:7 + LEG_DOF]
        dq = self.data.qvel[6:6 + LEG_DOF]
        leg_names = ("fl", "fr", "hl", "hr")

        meas = " ".join(
            "%s[hx %+.2f hy %+.2f kn %+.2f wh %+.2f]"
            % (leg_names[leg], dq[leg * 4], dq[leg * 4 + 1],
               dq[leg * 4 + 2], dq[leg * 4 + 3])
            for leg in range(4)
        )
        print("[JVEL-SIM] t=%6.3f s meas(rad/s) %s" % (self.timestamp, meas))

        cmd = " ".join(
            "%s %+.2f (kp %.1f kd %.2f)"
            % (leg_names[leg], self.vel_cmd[leg * 4 + 3, 0],
               self.kp_cmd[leg * 4 + 3, 0], self.kd_cmd[leg * 4 + 3, 0])
            for leg in range(4)
        )
        wheel_q = " ".join("%s %+.1f" % (leg_names[leg], q[leg * 4 + 3]) for leg in range(4))
        print("[JVEL-SIM] wheel cmd(rad/s) %s | wheel q(rad) %s" % (cmd, wheel_q))

    # --------------------------------------------------------
    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q

        # roll (X-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # pitch (Y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)  # 防止数值漂移导致 |t2|>1
        pitch = np.arcsin(t2)

        # yaw (Z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    # --------------------------------------------------------
    def _publish_robot_state(self, step: int):
        # ----- IMU -----
        q_world = self.data.sensordata[:4]  # quaternion (w, x, y, z) in MuJoCo convention
        rpy_rad = self.quaternion_to_euler(q_world)  # returns [roll, pitch, yaw] in radians
        # Convert to degrees
        rpy_deg = [angle * (180.0 / 3.141592653589793) for angle in rpy_rad]

        body_acc = self.data.sensordata[4:7]
        angvel_b = self.data.sensordata[7:10]  # body frame

        imu_msg = ImuData()
        imu_msg.header = MetaType()
        imu_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_deg[0])
        imu_msg.data.pitch = float(rpy_deg[1])
        imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(angvel_b[0])
        imu_msg.data.omega_y = float(angvel_b[1])
        imu_msg.data.omega_z = float(angvel_b[2])
        imu_msg.data.acc_x = float(body_acc[0])
        imu_msg.data.acc_y = float(body_acc[1])
        imu_msg.data.acc_z = float(body_acc[2])
        self.imu_pub.publish(imu_msg)

        # ----- legs -----
        q = self.data.qpos[7:7 + LEG_DOF]
        dq = self.data.qvel[6:6 + LEG_DOF]
        tau = self.input_tq[:LEG_DOF].flatten()

        # Convert raw to published: published = (raw - offset_rad) * dir
        pub_pos = (q - POS_OFFSET_RAD) * JOINT_DIR
        pub_vel = dq * JOINT_DIR
        pub_tau = tau * JOINT_DIR  # Torque also needs direction flip

        legs_msg = JointsData()
        legs_msg.header = MetaType()
        legs_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        legs_msg.header.stamp = stamp
        legs_msg.data = JointsDataValue()
        legs_msg.data.joints_data = [JointData() for _ in range(LEG_DOF)]
        for i in range(LEG_DOF):
            joint = legs_msg.data.joints_data[i]
            joint.name = [32, 32, 32, 32]  # Dummy name (four spaces)
            joint.data_id = 0  # Dummy
            joint.status_word = 1  # Normal
            joint.position = float(pub_pos[i])
            joint.torque = float(pub_tau[i])
            joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0  # Dummy normal temp
            joint.driver_temp = 45.0  # Dummy normal temp
        self.joints_pub.publish(legs_msg)

        # ----- arm / gripper (raw) -----
        arm_q = self.data.qpos[7 + LEG_DOF:7 + TOTAL_DOF]
        arm_dq = self.data.qvel[6 + LEG_DOF:6 + TOTAL_DOF]
        arm_tau = self.input_tq[LEG_DOF:].flatten()

        pub_arm_pos = (arm_q - ARM_OFFSET_RAD) * ARM_DIR
        pub_arm_vel = arm_dq * ARM_DIR
        pub_arm_tau = arm_tau * ARM_DIR

        arm_msg = JointsData()
        arm_msg.header = MetaType()
        arm_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        arm_msg.header.stamp = stamp
        arm_msg.data = JointsDataValue()
        # JointsDataValue is a fixed 16-element array; fill the first ARM_DOF entries
        arm_msg.data.joints_data = [JointData() for _ in range(16)]
        for i in range(ARM_DOF):
            joint = arm_msg.data.joints_data[i]
            joint.name = [32, 32, 32, 32]  # Dummy name (four spaces)
            joint.data_id = 0  # Dummy
            joint.status_word = 1  # Normal
            joint.position = float(pub_arm_pos[i])
            joint.torque = float(pub_arm_tau[i])
            joint.velocity = float(pub_arm_vel[i])
            joint.motion_temp = 40.0  # Dummy normal temp
            joint.driver_temp = 45.0  # Dummy normal temp
        self.arm_joints_pub.publish(arm_msg)


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    rclpy.init()
    sim_node = MuJoCoSimulationNode()
    sim_node.start()
    sim_node.destroy_node()
    rclpy.shutdown()
