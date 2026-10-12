"""
Shared layout for the internal /ARM_TELEOP Float32MultiArray protocol.

The arm teleop pipeline (keyboard / VR -> arm_controller) is internal to this
deployment repo, so we intentionally keep it as a documented Float32MultiArray
layout instead of changing the shared drdds message package. All producers and
consumers must import these constants so the index mapping lives in one place.

New message length: 9 floats
  [0] mode
        0 = INCREMENTAL (keyboard style):
              [1:4]   EE position increment  (dx, dy, dz)
              [4:7]   EE orientation increment (droll, dpitch, dyaw)
              [7]     gripper: >=0 set (0 open / 1 closed), -1 no change
              [8]     ee_reset: 1 -> re-anchor target to current robot pose
        1 = ABSOLUTE_OFFSET (VR style):
              [1:4]   EE position offset  (x, y, z) relative to the anchor
              [4:7]   EE orientation offset (roll, pitch, yaw) relative to
                      the anchor, applied as a quaternion multiply
              [7]     gripper: same semantics as mode 0
              [8]     ee_recalibrate: 1 -> re-anchor to current robot pose
                      and keep applying the current offsets
              2 -> **重置到默认末端位姿**（anchor = 默认关节角对应的 FK 位姿，
                   偏移按 0 处理）。用于"复位仿真 / 左 X/Y 复位"：不依赖
                   "复位完成后再采当前位姿"的时序，臂会干净地回默认姿态、
                   且不会因为旧目标还在而被拽走（2026-10-10 实测的瞬态根因）。

For backwards compatibility arm_controller still accepts the old 8-float
message (no mode field), which is interpreted as mode 0.
"""

ARM_TELEOP_MODE_INCREMENTAL = 0.0
ARM_TELEOP_MODE_ABSOLUTE = 1.0

ARM_TELEOP_LEN = 9
ARM_TELEOP_MODE_IDX = 0
# indices 1:4 pos, 4:7 euler, 7 gripper, 8 flag
ARM_TELEOP_POS_START = 1
ARM_TELEOP_EULER_START = 4
ARM_TELEOP_GRIPPER_IDX = 7
ARM_TELEOP_FLAG_IDX = 8

# flag 的取值（只有 mode 1 用；mode 0 里 [8] 是 0/1 布尔）
ARM_TELEOP_FLAG_NONE = 0.0
ARM_TELEOP_FLAG_REANCHOR = 1.0        # 锚到当前位姿
ARM_TELEOP_FLAG_RESET_DEFAULT = 2.0   # 锚到"默认末端位姿"（= 复位语义）


def make_incremental(dx=0.0, dy=0.0, dz=0.0,
                     droll=0.0, dpitch=0.0, dyaw=0.0,
                     gripper=-1.0, ee_reset=0.0):
    """Build a mode-0 (incremental) /ARM_TELEOP command list."""
    msg = [ARM_TELEOP_MODE_INCREMENTAL] * ARM_TELEOP_LEN
    msg[ARM_TELEOP_POS_START:ARM_TELEOP_POS_START + 3] = [dx, dy, dz]
    msg[ARM_TELEOP_EULER_START:ARM_TELEOP_EULER_START + 3] = [droll, dpitch, dyaw]
    msg[ARM_TELEOP_GRIPPER_IDX] = gripper
    msg[ARM_TELEOP_FLAG_IDX] = 1.0 if ee_reset else 0.0
    return msg


def make_absolute_offset(dx=0.0, dy=0.0, dz=0.0,
                         droll=0.0, dpitch=0.0, dyaw=0.0,
                         gripper=-1.0, ee_recalibrate=0.0):
    """Build a mode-1 (absolute offset) /ARM_TELEOP command list."""
    msg = [ARM_TELEOP_MODE_ABSOLUTE] * ARM_TELEOP_LEN
    msg[ARM_TELEOP_POS_START:ARM_TELEOP_POS_START + 3] = [dx, dy, dz]
    msg[ARM_TELEOP_EULER_START:ARM_TELEOP_EULER_START + 3] = [droll, dpitch, dyaw]
    msg[ARM_TELEOP_GRIPPER_IDX] = gripper
    # ⚠️ 原样透传（不再 `1.0 if flag else 0.0`）：flag 不只 0/1，
    # ARM_TELEOP_FLAG_RESET_DEFAULT=2.0 会被那个写法压成 1.0，语义就丢了。
    msg[ARM_TELEOP_FLAG_IDX] = float(ee_recalibrate)
    return msg
