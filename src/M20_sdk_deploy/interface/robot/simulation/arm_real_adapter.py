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

新增（2026-09-22）：把臂的**当前末端位姿**用 FK 算出来，发布到 `/ARM_TELEOP_STATE`
（root/机体坐标系，pos + quat wxyz）。原因：策略观测里有 7 维 `ee_goal`，
如果臂动了但没人更新它，策略就会一直以为"臂还在默认位姿"，观测与真机不一致
（分布外）。两种情况都需要它：

  A) 用我们的 arm_controller(IK)+arm_teleop 控制：arm_controller 自己也发
     `/ARM_TELEOP_STATE`，本节点的这份会被它覆盖（两者一致，无害）；
  B) 保留 agx 自带的键盘控制（不使用 arm_controller）：**必须**由本节点发
     `/ARM_TELEOP_STATE`，否则策略看到的 ee_goal 是错的。

开关：M20_ADAPTER_PUBLISH_EE=0 关闭（默认开）。

**在哪台机器上跑**：本节点发布/订阅的都是 drdds 消息，所以放在**已经装好 drdds 的
M20 电脑**上最省事（Jetson 上就只需要 agx_arm_ctrl）。agx 的
`/control/joint_states`、`/feedback/joint_states` 是 sensor_msgs，跨机器走 DDS 即可。
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from builtin_interfaces.msg import Time
from drdds.msg import JointsData, JointsDataCmd, JointData, JointsDataValue
from drdds.msg import MetaType


ARM_CMD_TOPIC = '/ARM_JOINTS_CMD'
ARM_DATA_TOPIC = '/ARM_JOINTS_DATA'
# agx 侧的话题名做成可配：不同版本的 agx_arm_ros / 不同 launch 可能带命名空间，
# 在 Jetson 上用 `ros2 topic list | grep -i joint` 看实际名字，然后用环境变量指过来。
AGX_CONTROL_TOPIC = os.environ.get('M20_ADAPTER_AGX_CONTROL_TOPIC', '/control/joint_states')
AGX_FEEDBACK_TOPIC = os.environ.get('M20_ADAPTER_AGX_FEEDBACK_TOPIC', '/feedback/joint_states')
CHECK_SECONDS = float(os.environ.get('M20_ADAPTER_CHECK_SECONDS', '5'))

ARM_JOINT_COUNT = 6
TOTAL_JOINT_COUNT = 8  # 6 arm + 2 sim gripper joints
GRIPPER_EFFORT = 1.0
EE_STATE_TOPIC = '/ARM_TELEOP_STATE'

# 默认姿态下 gripper_base 在 root 系的位置（实测，见 check_mjcf_contract.py）
DEFAULT_EE_GOAL_BODY = [0.3492, 0.0, 0.4327, 0.7373, 0.0, 0.6756, 0.0]

try:  # 同目录的 arm_controller 里有 MDH 表与 FK，优先复用（避免两份定义漂移）
    from arm_controller import PIPER_MDH, ARM_BASE_OFFSET, fk_pose, mat_to_quat  # noqa: E402
    _FK_OK = True
except Exception as _e:  # pragma: no cover - 只在这两个文件不在一起时发生
    _FK_OK = False
    _FK_ERR = _e


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
        self._ee_q = None
        self._ee_log_cnt = 0
        self.cmd_count = 0

        # 是否用 FK 发布 /ARM_TELEOP_STATE（策略观测的 ee_goal）
        self._publish_ee = os.environ.get('M20_ADAPTER_PUBLISH_EE', '1') != '0'
        self._ee_pub = None
        if self._publish_ee:
            if _FK_OK:
                self._ee_pub = self.create_publisher(Float32MultiArray, EE_STATE_TOPIC, 10)
                self.create_timer(0.02, self._publish_ee_state)   # 50 Hz
            else:
                self.get_logger().warn(
                    f'[arm_real_adapter] 无法导入 FK（{_FK_ERR}）⇒ 不发布 {EE_STATE_TOPIC}；'
                    '策略会一直用默认 ee_goal（臂动起来后观测会不一致）')

        # republish the latest agx feedback at 200 Hz (drdds side)
        self.create_timer(0.005, self._publish_feedback)

        self.get_logger().info(
            "[arm_real_adapter] started: /ARM_JOINTS_CMD -> "
            f"{AGX_CONTROL_TOPIC}, {AGX_FEEDBACK_TOPIC} -> /ARM_JOINTS_DATA")

    # ------------------------------------------------------------------
    def _arm_cmd_cb(self, msg):
        """drdds 8-DOF command (arm_joint1..6 + gripper1/2) -> agx joint_states."""
        self.cmd_count += 1
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

    def _publish_ee_state(self):
        """用 FK 把当前臂关节角换算成 root 系的末端位姿，作为策略观测的 ee_goal。"""
        if self._last_feedback is None or self._ee_q is None:
            return
        pos, R = fk_pose(PIPER_MDH, self._ee_q)
        pos_body = pos + ARM_BASE_OFFSET          # 臂基座系 -> 机体(root)系
        quat = mat_to_quat(R)                     # wxyz
        msg = Float32MultiArray()
        msg.data = [float(pos_body[0]), float(pos_body[1]), float(pos_body[2]),
                    float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
        self._ee_pub.publish(msg)

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

        self._ee_q = [q[i] for i in range(ARM_JOINT_COUNT)]
        self._ee_log_cnt += 1
        if self._ee_log_cnt == 200:      # ~1 s 后打一次，便于确认链路
            pos, _ = fk_pose(PIPER_MDH, self._ee_q)
            pb = pos + ARM_BASE_OFFSET
            self.get_logger().info(
                '[arm_real_adapter] 臂反馈已接通：q=%s，末端(root系)=%.3f %.3f %.3f'
                % (['%.3f' % v for v in self._ee_q], pb[0], pb[1], pb[2]))

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


def check_mode() -> int:
    """自检：等几秒看能不能收到 agx 反馈 / 有没有人发 /ARM_JOINTS_CMD，然后退出。

    用法: python3 arm_real_adapter.py --check
    """
    rclpy.init()
    node = ArmRealAdapter()
    node.get_logger().info(
        f'[check] 监听 {CHECK_SECONDS:.0f} s：agx 反馈 = {AGX_FEEDBACK_TOPIC}，'
        f'agx 命令 = {AGX_CONTROL_TOPIC}')
    t_end = time.time() + CHECK_SECONDS
    fb_before = 0
    while time.time() < t_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    ok_fb = node._last_feedback is not None
    ok_cmd = node.cmd_count > 0
    print()
    print(f'[check] {AGX_FEEDBACK_TOPIC:<28} : '
          + ('收到数据 ✓' if ok_fb else '**没有数据 ✗**（话题名不对 / 跨机器发现没通 / 机器人没上电）'))
    print(f'[check] {ARM_CMD_TOPIC:<28} : '
          + (f'收到 {node.cmd_count} 条 ✓（有人在控制臂）' if ok_cmd
             else '没有收到（没人发 /ARM_JOINTS_CMD —— 只桥接感知时正常）'))
    if not ok_fb:
        print()
        print('[check] 排查顺序：')
        print('  1) 在 **Jetson** 上：`ros2 topic list | grep -i joint` 看实际话题名；')
        print('     若与上面不同，用 M20_ADAPTER_AGX_FEEDBACK_TOPIC=<实际名> 指过来。')
        print('  2) 两边 `echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION $ROS_LOCALHOST_ONLY` 必须一致')
        print('     （ROS_LOCALHOST_ONLY 不能是 1）。')
        print('  3) 在 M20 上 `ros2 node list` 能否看到 Jetson 的节点：看不到 = DDS 发现没通')
        print('     （组播被挡时配静态对端：CYCLONEDDS_URI 的 <Peers>，或 Fast DDS 的 profiles）。')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok_fb else 1


def main(args=None):
    if '--check' in (args or sys.argv):
        return check_mode()
    rclpy.init(args=args)
    node = ArmRealAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
