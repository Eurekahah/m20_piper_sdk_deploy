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

# ---------------------------------------------------------------------------
# 遥测落盘（L3 端到端验收用）
#   M20_SIM_TELEMETRY=<path>         写到这个 CSV；默认关
#   M20_SIM_TELEMETRY_PERIOD=<ticks> 采样周期（默认 5 tick = 200 Hz）
# 列：t, base xyz, base quat(wxyz), rpy, omega_b, q[24], dq[24], tau[24],
#     wheel_contact_force[4]
# ⚠️ q/dq/tau 的 24 列是 **MuJoCo(MJCF) 顺序**：每腿 hipx/hipy/knee/wheel 连续
#    （轮子在 3/7/11/15），然后 arm_joint1..6 与 gripper_joint1/2。
#    这**不是**策略观测用的 articulation 原生序，也不是动作序 ——
#    离线分析时别用错（很容易把 arm_joint1 的 0.5 当成"轮子位置"）。
# 用途：tests/sim2sim_smoke.py 用它算"高度/倾角"曲线与摔倒判据
# （判据与训练一致：倾角 > 0.8 rad 或 height < 0.30 m，height 用
#  root_z − mean(四轮 z) + 0.09 的定义）。
# ---------------------------------------------------------------------------
TELEMETRY_PATH = os.environ.get("M20_SIM_TELEMETRY", "")
TELEMETRY_PERIOD = int(os.environ.get("M20_SIM_TELEMETRY_PERIOD", "5"))

# ---------------------------------------------------------------------------
# 扰动注入（用来验证 rl_deploy 的安全接管确实会触发，见 P0-7 / DEF-016）
#   M20_SIM_PUSH_FORCE=<N>     作用在 base_link 上的 +x 方向的力（默认 0 = 关）
#   M20_SIM_PUSH_AT=<s>        从第几秒开始推（默认 12）
#   M20_SIM_PUSH_DURATION=<s>  持续多久（默认 0.3）
# ---------------------------------------------------------------------------
PUSH_FORCE = float(os.environ.get("M20_SIM_PUSH_FORCE", "0"))
PUSH_AT = float(os.environ.get("M20_SIM_PUSH_AT", "12"))
PUSH_DURATION = float(os.environ.get("M20_SIM_PUSH_DURATION", "0.3"))

# ---------------------------------------------------------------------------
# 执行器延迟（训练侧 DelayedPDActuator：每个执行器随机 0~5 个**物理步**，
# 训练 sim.dt=5 ms ⇒ 0~25 ms）。部署/仿真侧原来完全没有延迟，
# 等于把策略放在比训练更"锐"的执行器上，入口瞬态会更容易发散（DEF-018 候选）。
#   M20_SIM_ACTUATOR_DELAY_TICKS=<N>  每关节在 [0, N] 个**控制 tick**(1 ms) 里随机
#                                     取一个固定延迟；0 = 关闭（默认）
# 延迟按固定种子采样，保证同一配置可复现。
# ---------------------------------------------------------------------------
ACTUATOR_DELAY_MAX = int(os.environ.get("M20_SIM_ACTUATOR_DELAY_TICKS", "0"))

# ---------------------------------------------------------------------------
# 轮子速度伺服阶跃（P1-2）：从 M20_SIM_WHEEL_STEP_AT 秒起，四个轮子收到固定的
# 速度目标 M20_SIM_WHEEL_STEP_RAD_S（直接覆盖 /JOINTS_CMD 里的轮子通道），
# 用来量测速度环的稳态误差/上升时间，验证 kp=0 / kd=0.6 + 轮子 armature 的链路。
# 默认关闭（值为 0）。
# ---------------------------------------------------------------------------
WHEEL_STEP = float(os.environ.get("M20_SIM_WHEEL_STEP_RAD_S", "0"))
WHEEL_STEP_AT = float(os.environ.get("M20_SIM_WHEEL_STEP_AT", "5"))

DT = 0.001
# 单次迭代最多补多少个 1 ms 控制 tick（防止落后时雪崩；正常应为 1~2）
MAX_CATCHUP_TICKS = int(os.environ.get("M20_SIM_MAX_CATCHUP", "20"))
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
# arm does not flop around during idle/standup). Gains must equal the training
# actuator config (DEF-007): piper_arm stiffness 300 / damping 20,
# piper_gripper 4000 / 200 —— 与 arm_controller.py 的 ARM_KP/KD、
# piper_arm_interface.hpp 的 Start() 保持值三处一致。
ARM_DEFAULT_KP = np.array([300.0] * 6 + [4000.0, 4000.0], dtype=np.float32)
ARM_DEFAULT_KD = np.array([20.0] * 6 + [200.0, 200.0], dtype=np.float32)


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
        # 保持目标必须与初始 qpos（JOINT_INIT = M20_Piper_own 的训练默认姿态）
        # 用同一份常量。用 LEG_INIT["M20"]（旧机型站立位姿）会把腿驱动到一个
        # 训练里从未见过的姿态，底盘直接坐到地上（DEF-012，实测 height 0.095 m）。
        self.pos_cmd = LEG_INIT["M20_Piper_own"].reshape(-1, 1)
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
        self.ignored_cmd_frames_ = 0   # 非 kIndexMotorControl 的控制帧计数（DEF-013）
        self.push_logged_ = False      # 扰动注入只打一次日志
        self.max_lag_ = 0.0            # 控制循环相对墙钟的最大落后（秒）
        # 执行器延迟（DEF-018 / P1-1）
        self.delay_max_ = max(ACTUATOR_DELAY_MAX, 0)
        if self.delay_max_ > 0:
            rng = np.random.default_rng(0)
            self.leg_delay_ = rng.integers(0, self.delay_max_ + 1, size=LEG_DOF)
            self.arm_delay_ = rng.integers(0, self.delay_max_ + 1, size=ARM_DOF)
        else:
            self.leg_delay_ = np.zeros(LEG_DOF, dtype=int)
            self.arm_delay_ = np.zeros(ARM_DOF, dtype=int)
        self.cmd_hist_ = []            # 最近 (delay_max_+1) 个控制 tick 的命令帧
        self.cmd_push_pending_ = False  # 每个控制 tick 只压一帧（5 个物理子步共用）

        self.base_body_id_ = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                               "base_link")
        self.ee_body_id_ = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                             "gripper_base")

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

        # 遥测（L3 验收）
        self.telemetry_file = None
        self.wheel_geom_id_list = self._wheel_geom_ids()
        self.wheel_body_id_list = [i for i in range(self.model.nbody)
                                   if self.model.body(i).name.endswith("_wheel")]
        if TELEMETRY_PATH:
            self.telemetry_file = open(TELEMETRY_PATH, "w", buffering=1)
            cols = (["t", "wall", "base_x", "base_y", "base_z",
                     "base_qw", "base_qx", "base_qy", "base_qz",
                     "roll", "pitch", "yaw", "omega_x", "omega_y", "omega_z"]
                    + [f"q{i}" for i in range(TOTAL_DOF)]
                    + [f"dq{i}" for i in range(TOTAL_DOF)]
                    + [f"tau{i}" for i in range(TOTAL_DOF)]
                    + ["wheel_z_fl", "wheel_z_fr", "wheel_z_hl", "wheel_z_hr"]
                    + ["wheel_f_fl", "wheel_f_fr", "wheel_f_hl", "wheel_f_hr"]
                    # 末端位姿（gripper_base 相对 base_link，root 系）：
                    # 用来验收 IK 的跟踪精度（策略观测 ee_goal 的口径）
                    + ["ee_x", "ee_y", "ee_z", "ee_qw", "ee_qx", "ee_qy", "ee_qz"]
                    + ["wcmd_fl", "wcmd_fr", "wcmd_hl", "wcmd_hr"])
            self.telemetry_file.write(",".join(cols) + "\n")
            self.get_logger().info(
                f"[telemetry] -> {TELEMETRY_PATH} @ {1000 // max(TELEMETRY_PERIOD, 1)} Hz")

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
        self.pos_cmd = LEG_INIT["M20_Piper_own"].reshape(-1, 1)
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
    # drdds 控制字：只有 kIndexMotorControl(=4) 的帧才是"关节控制命令"。
    # `DdsInterface` 的构造函数会发 4 帧 kp=0/kd=0/pos=0 的
    # control_word ∈ {1(disable), 17(error reset), 2(enable), 23(get status)}
    # 帧（真机上这些帧只做电机状态操作，增益字段被固件忽略）。
    # 仿真侧如果不看控制字，就会把这 4 帧当成"零增益命令"执行 ⇒ 腿瞬间失去
    # 支撑、机器人趴下再被后续命令弹起来（记得 DEF-013）。
    CONTROL_WORD_MOTOR = 4

    def _is_motor_control(self, msg: JointsDataCmd, n: int) -> bool:
        """True = 这一帧是关节控制命令；非控制字（错误复位/使能…）直接丢弃。"""
        for i in range(n):
            if msg.data.joints_data[i].control_word != self.CONTROL_WORD_MOTOR:
                self.ignored_cmd_frames_ += 1
                if self.ignored_cmd_frames_ in (1, 10, 100):
                    self.get_logger().info(
                        f"ignoring non-motor-control JointsDataCmd "
                        f"(control_word={msg.data.joints_data[i].control_word}, "
                        f"seen {self.ignored_cmd_frames_} times)")
                return False
        return True

    def _cmd_callback(self, msg: JointsDataCmd):
        """Leg joint commands from rl_deploy (published in robot frame)."""
        if len(msg.data.joints_data) != LEG_DOF:
            self.get_logger().warn("Received JointsDataCmd with incorrect number of leg joints")
            return
        if not self._is_motor_control(msg, LEG_DOF):
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
        if not self._is_motor_control(msg, ARM_DOF):
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
        wall_t0 = time.monotonic()
        while rclpy.ok():
            now = time.time()
            if now - last_time >= DT:
                # **追帧**：策略（rl_deploy）是按墙钟 50 Hz 跑的，如果仿真落后
                # 墙钟，等价于把策略的控制周期按仿真时间拉长 —— 既改变稳定性，
                # 也让"连续跑多档"时结果不可复现（DEF-018）。
                # 这里按欠账补若干控制 tick，并限制单次补帧上限，避免雪崩。
                due = int((now - last_time) / DT)
                due = max(1, min(due, MAX_CATCHUP_TICKS))
                last_time += due * DT
                if last_time < now - 0.5:          # 落后太多就重新对齐
                    last_time = now
                self.max_lag_ = max(self.max_lag_, now - last_time)
                for _ in range(due):
                    step += 1
                    self.step_count_ += 1
                    self.cmd_push_pending_ = True   # 本 tick 压一帧命令（执行器延迟用）
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

                    # 遥测落盘（默认 200 Hz）
                    if self.telemetry_file is not None and step % TELEMETRY_PERIOD == 0:
                        self._write_telemetry(wall_t0)

                    # 可视化
                    if self.viewer and step % RENDER_INTERVAL == 0:
                        self.viewer.sync()

            # Handle ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.0)

    def _apply_joint_torque(self):
        # ---- 执行器延迟：把"本 tick 收到的命令"压入历史，实际用的是 N 个 tick 之前的那帧
        frame = np.zeros((TOTAL_DOF, 5), dtype=np.float64)
        frame[:LEG_DOF, 0] = self.kp_cmd.flatten()
        frame[:LEG_DOF, 1] = self.pos_cmd.flatten()
        frame[:LEG_DOF, 2] = self.kd_cmd.flatten()
        frame[:LEG_DOF, 3] = self.vel_cmd.flatten()
        frame[:LEG_DOF, 4] = self.tau_ff.flatten()
        frame[LEG_DOF:, 0] = self.arm_kp_cmd.flatten()
        frame[LEG_DOF:, 1] = self.arm_pos_cmd.flatten()
        frame[LEG_DOF:, 2] = self.arm_kd_cmd.flatten()
        frame[LEG_DOF:, 3] = self.arm_vel_cmd.flatten()
        frame[LEG_DOF:, 4] = self.arm_tau_ff.flatten()
        if self.cmd_push_pending_:
            # 一个控制 tick 只压一帧（本函数 5 个物理子步里会被调用 5 次）
            self.cmd_hist_.append(frame)
            if len(self.cmd_hist_) > self.delay_max_ + 1:
                self.cmd_hist_.pop(0)
            self.cmd_push_pending_ = False
        if self.delay_max_ > 0 and len(self.cmd_hist_) > self.delay_max_:
            # 每个关节按自己的延迟取历史帧（用 vstack 后按行挑，省一个循环）
            hist = np.stack(self.cmd_hist_)              # (T, 24, 5)
            T = hist.shape[0]
            idx_leg = np.clip(T - 1 - self.leg_delay_, 0, T - 1)
            idx_arm = np.clip(T - 1 - self.arm_delay_, 0, T - 1)
            delayed = hist[np.concatenate([idx_leg, idx_arm]),
                           np.arange(TOTAL_DOF)]
        else:
            delayed = frame

        # 扰动注入（默认关）：用来验证安全接管
        if PUSH_FORCE != 0.0:
            active = PUSH_AT <= self.timestamp < PUSH_AT + PUSH_DURATION
            if active and not self.push_logged_:
                self.get_logger().warn(
                    f"[push] applying {PUSH_FORCE} N on base_link at t={self.timestamp:.2f}s")
                self.push_logged_ = True
            self.data.xfrc_applied[self.base_body_id_, 0] = PUSH_FORCE if active else 0.0
        # 当前关节状态
        q = self.data.qpos[7:7 + TOTAL_DOF].reshape(-1, 1)
        dq = self.data.qvel[6:6 + TOTAL_DOF].reshape(-1, 1)

        # 轮子速度阶跃（P1-2，默认关）：直接覆盖四个轮子的执行器语义为
        # kp=0 / kd=0.6 / 速度目标 = WHEEL_STEP，用来验收速度伺服链
        if WHEEL_STEP != 0.0 and self.timestamp >= WHEEL_STEP_AT:
            for leg in range(4):
                idx = leg * 4 + 3
                delayed[idx, 0] = 0.0          # kp
                delayed[idx, 2] = 0.6          # kd
                delayed[idx, 3] = WHEEL_STEP   # 速度目标
                delayed[idx, 4] = 0.0          # tau_ff
        # legs
        self.input_tq[:LEG_DOF] = (
                delayed[:LEG_DOF, 0:1] * (delayed[:LEG_DOF, 1:2] - q[:LEG_DOF]) +
                delayed[:LEG_DOF, 2:3] * (delayed[:LEG_DOF, 3:4] - dq[:LEG_DOF]) +
                delayed[:LEG_DOF, 4:5]
        )

        # arm/gripper: use the default hold until the first command arrives
        arm_q = q[LEG_DOF:]
        arm_dq = dq[LEG_DOF:]
        if self.arm_cmd_valid:
            kp = delayed[LEG_DOF:, 0:1]
            pos = delayed[LEG_DOF:, 1:2]
            kd = delayed[LEG_DOF:, 2:3]
            vel = delayed[LEG_DOF:, 3:4]
            tau = delayed[LEG_DOF:, 4:5]
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
    def _wheel_contact_forces(self):
        """四个轮子的接触法向力（N），按 fl/fr/hl/hr 的 MJCF 顺序。"""
        forces = [0.0, 0.0, 0.0, 0.0]
        if not self.wheel_geom_id_list:
            return forces
        geom_to_wheel = {g: k for k, g in enumerate(self.wheel_geom_id_list)}
        f6 = np.zeros(6, dtype=np.float64)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            k = geom_to_wheel.get(c.geom1, geom_to_wheel.get(c.geom2, None))
            if k is None:
                continue
            mujoco.mj_contactForce(self.model, self.data, i, f6)
            forces[k] += abs(float(f6[0]))
        return forces

    def _write_telemetry(self, wall_t0):
        """一行遥测。列定义见文件头的 M20_SIM_TELEMETRY 注释。
        `wall` = 距仿真节点启动的墙钟秒数（与 `t`=仿真时间对照，用来看实时因子）。"""
        q_world = self.data.sensordata[:4]
        rpy = self.quaternion_to_euler(q_world)
        omega_b = self.data.sensordata[7:10]
        q = self.data.qpos[7:7 + TOTAL_DOF]
        dq = self.data.qvel[6:6 + TOTAL_DOF]
        tau = self.input_tq.flatten()
        # 末端位姿（相对 base_link；root 系）
        if self.ee_body_id_ >= 0 and self.base_body_id_ >= 0:
            d_pos = self.data.xpos[self.ee_body_id_] - self.data.xpos[self.base_body_id_]
            qb = self.data.xquat[self.base_body_id_]
            qe = self.data.xquat[self.ee_body_id_]
            qb_inv = np.array([qb[0], -qb[1], -qb[2], -qb[3]], dtype=np.float64)
            q_rel = np.zeros(4, dtype=np.float64)
            mujoco.mju_mulQuat(q_rel, qb_inv, qe)
        else:
            d_pos = np.zeros(3)
            q_rel = np.array([1.0, 0, 0, 0])
        # 轮子速度指令（rad/s）：RL 的 vel_cmd（kp=0 时就是速度目标）
        wcmd = [float(self.vel_cmd[leg * 4 + 3, 0]) for leg in range(4)]

        row = ([self.timestamp, time.monotonic() - wall_t0,
                self.data.qpos[0], self.data.qpos[1], self.data.qpos[2],
                q_world[0], q_world[1], q_world[2], q_world[3],
                rpy[0], rpy[1], rpy[2],
                omega_b[0], omega_b[1], omega_b[2]]
               + list(q) + list(dq) + list(tau)
               + list(self.data.xpos[self.wheel_body_id_list, 2])
               + self._wheel_contact_forces()
               + list(d_pos) + list(q_rel)
               + wcmd)
        self.telemetry_file.write(",".join("%.6g" % v for v in row) + "\n")

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
