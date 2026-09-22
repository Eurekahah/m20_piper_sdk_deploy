#!/usr/bin/env python3
"""跨机器臂数据兜底通道（UDP），不依赖 ROS 2 的跨机器 DDS 发现。

**什么时候用**：Jetson（跑 agx_arm_ros）和 M20 电脑之间 `ping` 通，但
`ros2 node list` 互相看不到对方节点（组播被交换机挡、多网卡选错接口、
跨发行版 RMW 不兼容……）。这时用本脚本把臂的 8 个关节值直接走 UDP 送过去。

拓扑：

    [Jetson 容器]                                   [M20]
      agx_arm_ctrl                                   rl_deploy / arm_controller
        /feedback/joint_states (sensor_msgs)             ↑ /ARM_JOINTS_DATA (drdds)
              ↓                                          ↑ /ARM_TELEOP_STATE
      arm_udp_bridge.py --tx  ──── UDP 200 Hz ───→  arm_udp_bridge.py --rx

用法：
    # Jetson（容器里，和 agx 同一个 ROS 环境；只需要 sensor_msgs）
    python3 arm_udp_bridge.py --tx --peer 10.21.31.103 --port 45800

    # M20（需要 drdds + 本仓库；会发布 /ARM_JOINTS_DATA 与 /ARM_TELEOP_STATE）
    python3 arm_udp_bridge.py --rx --port 45800

报文格式（小端，共 4 + 4 + 6*4 + 4 = 36 字节，另有 4 字节魔数）：
    magic 'M20A' | seq(uint32) | q1..q6(float32, rad) | gripper(float32, m)

特性：丢包只丢一帧（无重传）；收发都打印 1 Hz 统计，便于判断是"没收到"还是"没发出"。
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

MAGIC = b"M20A"
FMT = "!II6ff"            # magic 作为两个 uint32 拼进去会乱，下面手工处理
PACK = struct.Struct("<4sI6ff")

ARM_JOINT_COUNT = 6
TOTAL_JOINT_COUNT = 8

# 默认姿态下 gripper_base 在 root 系的位置（与 arm_real_adapter / runner 一致）
DEFAULT_EE_GOAL_BODY = [0.3492, 0.0, 0.4327, 0.7373, 0.0, 0.6756, 0.0]


# ---------------------------------------------------------------------------
# 发送端（Jetson）
# ---------------------------------------------------------------------------
def run_tx(peer: str, port: int, topic: str) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state = {"q": [0.0] * 6, "grip": 0.0, "rx": 0, "tx": 0, "last": 0.0,
             "have": False}

    class Tx(Node):
        def __init__(self) -> None:
            super().__init__("arm_udp_bridge_tx")
            self.create_subscription(JointState, topic, self._cb, 10)
            self.create_timer(0.005, self._tick)          # 200 Hz

        def _cb(self, msg: JointState) -> None:
            idx = {n: i for i, n in enumerate(msg.name)}
            for i in range(ARM_JOINT_COUNT):
                n = f"joint{i + 1}"
                if n in idx:
                    state["q"][i] = float(msg.position[idx[n]])
            if "gripper" in idx:
                state["grip"] = float(msg.position[idx["gripper"]])
            state["rx"] += 1
            state["have"] = True

        def _tick(self) -> None:
            seq = state["tx"]
            pkt = PACK.pack(MAGIC, seq, *state["q"], state["grip"])
            sock.sendto(pkt, (peer, port))
            state["tx"] += 1

    rclpy.init()
    node = Tx()
    print(f"[udp-tx] {topic} → udp://{peer}:{port} @200 Hz（Ctrl-C 退出）")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - state["last"] >= 1.0:
                state["last"] = now
                if state["have"]:
                    print(f"[udp-tx] 已发 {state['tx']} 帧/秒，收到 {state['rx']} 条 "
                          f"{topic}；q={['%.3f' % v for v in state['q']]} grip={state['grip']:.3f}")
                else:
                    print(f"[udp-tx] 还没收到 {topic} 的任何数据（agx 没起？话题名不同？）")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sock.close()
    return 0


# ---------------------------------------------------------------------------
# 接收端（M20）
# ---------------------------------------------------------------------------
def run_rx(port: int, publish_ee: bool) -> int:
    import rclpy
    from rclpy.node import Node
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Float32MultiArray
    from drdds.msg import JointsData, JointData, JointsDataValue, MetaType

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.002)

    state = {"q": [0.0] * 6, "grip": 0.0, "n": 0, "last": 0.0, "seq": None,
             "lost": 0, "have": False}

    fk = None
    if publish_ee:
        try:                      # 复用 arm_controller 里的 MDH/FK（同目录）
            from arm_controller import PIPER_MDH, ARM_BASE_OFFSET, fk_pose, mat_to_quat
            fk = (PIPER_MDH, ARM_BASE_OFFSET, fk_pose, mat_to_quat)
        except Exception as e:    # pragma: no cover
            print(f"[udp-rx] 无法导入 FK（{e}）⇒ 不发布 /ARM_TELEOP_STATE")

    class Rx(Node):
        def __init__(self) -> None:
            super().__init__("arm_udp_bridge_rx")
            self.data_pub = self.create_publisher(JointsData, "/ARM_JOINTS_DATA", 10)
            self.ee_pub = self.create_publisher(Float32MultiArray, "/ARM_TELEOP_STATE", 10)
            self.create_timer(0.005, self._publish)        # 200 Hz
            self.create_timer(0.02, self._publish_ee)      # 50 Hz

        def _publish(self) -> None:
            if not state["have"]:
                return
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
            q8 = list(state["q"]) + [0.5 * state["grip"], -0.5 * state["grip"]]
            for i in range(TOTAL_JOINT_COUNT):
                j = out.data.joints_data[i]
                j.name = [32, 32, 32, 32]
                j.data_id = 0
                j.status_word = 1
                j.position = float(q8[i])
                j.velocity = 0.0
                j.torque = 0.0
                j.motion_temp = 40.0
                j.driver_temp = 45.0
            self.data_pub.publish(out)

        def _publish_ee(self) -> None:
            if not state["have"]:
                return
            if fk is None:
                goal = DEFAULT_EE_GOAL_BODY
            else:
                mdh, offset, fk_pose, mat_to_quat = fk
                pos, R = fk_pose(mdh, state["q"])
                pb = pos + offset
                quat = mat_to_quat(R)
                goal = [pb[0], pb[1], pb[2], quat[0], quat[1], quat[2], quat[3]]
            msg = Float32MultiArray()
            msg.data = [float(v) for v in goal]
            self.ee_pub.publish(msg)

    rclpy.init()
    node = Rx()
    print(f"[udp-rx] 监听 udp://0.0.0.0:{port}，收到后发布 /ARM_JOINTS_DATA"
          + (" 与 /ARM_TELEOP_STATE" if fk else "") + "（Ctrl-C 退出）")
    try:
        while rclpy.ok():
            try:
                pkt, addr = sock.recvfrom(1024)
            except socket.timeout:
                pkt, addr = None, None
            if pkt:
                if len(pkt) >= PACK.size and pkt[:4] == MAGIC:
                    _, seq, *vals = PACK.unpack(pkt[:PACK.size])
                    state["q"] = list(vals[:6])
                    state["grip"] = float(vals[6])
                    if state["seq"] is not None and seq - state["seq"] > 1:
                        state["lost"] += seq - state["seq"] - 1
                    state["seq"] = seq
                    state["n"] += 1
                    state["have"] = True
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.time()
            if now - state["last"] >= 1.0:
                state["last"] = now
                if state["have"]:
                    print(f"[udp-rx] 收到 {state['n']} 帧/秒（丢 {state['lost']}），"
                          f"q={['%.3f' % v for v in state['q']]} grip={state['grip']:.3f}")
                else:
                    print("[udp-rx] 还没有收到任何 UDP 包"
                          "（对端没起 --tx / IP 端口不对 / 防火墙）")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sock.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--tx", action="store_true", help="发送端（跑在 Jetson 容器里）")
    g.add_argument("--rx", action="store_true", help="接收端（跑在 M20 上）")
    ap.add_argument("--peer", default="10.21.31.103", help="--tx 的目标 IP（M20）")
    ap.add_argument("--port", type=int, default=45800)
    ap.add_argument("--topic", default="/feedback/joint_states", help="--tx 订阅的话题")
    ap.add_argument("--no-ee", action="store_true",
                    help="--rx 不发布 /ARM_TELEOP_STATE（只用关节数据时）")
    a = ap.parse_args()
    if a.tx:
        return run_tx(a.peer, a.port, a.topic)
    return run_rx(a.port, not a.no_ee)


if __name__ == "__main__":
    sys.exit(main())
