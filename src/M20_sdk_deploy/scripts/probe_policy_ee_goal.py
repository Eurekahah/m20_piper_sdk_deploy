#!/usr/bin/env python3
"""Sweep the ee_goal block (obs[76:83]) of the deployed ONNX policy.

The deployment feeds obs[76:83] = (pos 3 + quat wxyz 4) of the absolute EE
target. When nothing publishes /ARM_TELEOP_STATE the deploy keeps a hardcoded
default. This script checks how sensitive the actor output is to that block, so
we can tell whether an out-of-distribution ee_goal is what makes the wheels
saturate.

Training data reference (rl_training .../deeprobotics_m20): the EE command is
sampled in a height-invariant frame at z = 0.6, spherical radius 0.3..0.52,
pitch -45..36 deg, yaw +-72 deg, orientation roll/pitch +-22.5 deg, yaw +-180.
"""

import math
import sys

import numpy as np
import onnxruntime as ort

POLICY = sys.argv[1] if len(sys.argv) > 1 else \
    "src/M20_sdk_deploy/policy/history_adaptation_full.onnx"

OBS_DIM = 86
HIST_STEPS, HIST_STEP_DIM = 10, 77
LEG, WHEEL, ARM, GRIP = 12, 4, 6, 2
ROBOT_JOINTS = LEG + WHEEL + ARM + GRIP


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy], dtype=np.float32)


def build(ee_goal, body_height=0.513, joint_pos_rel=None, joint_vel=None):
    obs = np.zeros(OBS_DIM, dtype=np.float32)
    obs[3:6] = (0.0, 0.0, -1.0)
    if joint_pos_rel is not None:
        obs[9:31] = joint_pos_rel
    if joint_vel is not None:
        obs[31:53] = joint_vel
    obs[76:83] = ee_goal
    obs[83:86] = (body_height, 0.0, 0.0)

    step = np.zeros(HIST_STEP_DIM, dtype=np.float32)
    step[3:6] = (0.0, 0.0, -1.0)
    if joint_pos_rel is not None:
        step[6:6 + ROBOT_JOINTS] = np.concatenate(
            [joint_pos_rel[:LEG + WHEEL], np.zeros(GRIP), joint_pos_rel[LEG + WHEEL:]])
    if joint_vel is not None:
        step[30:54] = np.concatenate(
            [joint_vel[:LEG + WHEEL], np.zeros(GRIP), joint_vel[LEG + WHEEL:]])
    return obs[None, :], np.tile(step, HIST_STEPS)[None, :]


def main():
    sess = ort.InferenceSession(POLICY, providers=["CPUExecutionProvider"])
    print("policy:", POLICY)
    print("%-52s %-34s %s" % ("ee_goal (pos | quat wxyz)", "legs[0:12]", "wheels[12:16]"))

    nominal = np.zeros(22, dtype=np.float32)
    candidates = [
        ("deploy default (hardcoded)", np.array([0.1092, 0.0, 0.3439, 0.7373, 0.0, 0.6756, 0.0], dtype=np.float32)),
        ("zero pos + identity quat", np.array([0.0, 0.0, 0.0, 1, 0, 0, 0], dtype=np.float32)),
        ("training-like: 0.5,0,0.6 ident", np.concatenate([[0.5, 0.0, 0.6], quat_from_rpy(0, 0, 0)])),
        ("training-like: 0.6,0,0.6 ident", np.concatenate([[0.6, 0.0, 0.6], quat_from_rpy(0, 0, 0)])),
        ("training-like: 0.5,0,0.45 ident", np.concatenate([[0.5, 0.0, 0.45], quat_from_rpy(0, 0, 0)])),
        ("training-like: 0.5,0,0.6 yaw90", np.concatenate([[0.5, 0.0, 0.6], quat_from_rpy(0, 0, math.pi / 2)])),
        ("deploy default, but z=0.6", np.array([0.1092, 0.0, 0.6, 0.7373, 0.0, 0.6756, 0.0], dtype=np.float32)),
    ]
    for tag, ee in candidates:
        obs, hist = build(ee)
        a = sess.run(["actions"], {"obs": obs, "obs_history": hist})[0][0]
        print("%-52s %-34s %s" % (tag,
                                  " ".join("%+.2f" % v for v in a[:12]),
                                  " ".join("%+.2f" % v for v in a[12:16])))

    print("\nlegs[12] and wheels[4] for a range of EE z (x=0.5, identity quat):")
    for z in (0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.9):
        ee = np.concatenate([[0.5, 0.0, z], quat_from_rpy(0, 0, 0)])
        obs, hist = build(ee)
        a = sess.run(["actions"], {"obs": obs, "obs_history": hist})[0][0]
        print("  z=%.1f  wheels %s" % (z, " ".join("%+.2f" % v for v in a[12:16])))


if __name__ == "__main__":
    main()
