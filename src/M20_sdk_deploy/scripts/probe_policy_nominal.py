#!/usr/bin/env python3
"""Offline probe of the deployed history-adaptation ONNX policy.

Feeds the policy the same observation layout the deployment builds
(M20PiperPolicyRunner) but with hand-specified contents, so we can tell whether
the network itself is sane or whether the deployment's obs assembly is off.

obs [1, 86]:
  [0:3]   base angular velocity * 0.25
  [3:6]   projected gravity
  [6:9]   base velocity command (vx, vy, wz)
  [9:31]  joint pos rel default, policy order (12 legs, 4 wheels(0), 6 arm)
  [31:53] joint velocity * 0.05, policy order
  [53:76] last action (23: 12 leg pos, 4 wheel vel, 7 ee_ik)
  [76:83] ee goal (pos 3 + quat wxyz 4)
  [83:86] body pose command (height, pitch, roll)

obs_history [1, 770] = 10 x
  [0:3]   base angular velocity (raw)
  [3:6]   projected gravity
  [6:30]  joint pos rel default, articulation order (24 joints)
  [30:54] joint velocity (raw), articulation order (24 joints)
  [54:77] last action (23)

Usage:
  python3 probe_policy_nominal.py [policy.onnx] [wheel_vel]
"""

import sys

import numpy as np
import onnxruntime as ort

POLICY = sys.argv[1] if len(sys.argv) > 1 else \
    "src/M20_sdk_deploy/policy/history_adaptation_full.onnx"
WHEEL_VEL = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

OBS_DIM = 86
HIST_STEPS, HIST_STEP_DIM = 10, 77
LEG, WHEEL, ARM, GRIP = 12, 4, 6, 2
POLICY_JOINTS = LEG + WHEEL + ARM          # 22, policy obs order
ROBOT_JOINTS = POLICY_JOINTS + GRIP        # 24, articulation (history) order


def build(wheel_vel: float, hist_wheel_slots: str = "policy",
          ee_goal_xyz=(0.1092, 0.0, 0.3439),
          ee_goal_quat=(0.7373, 0.0, 0.6756, 0.0),
          body_height=0.513):
    obs = np.zeros(OBS_DIM, dtype=np.float32)
    obs[3:6] = (0.0, 0.0, -1.0)                 # projected gravity
    obs[76:79] = ee_goal_xyz
    obs[79:83] = ee_goal_quat
    obs[83:86] = (body_height, 0.0, 0.0)

    step = np.zeros(HIST_STEP_DIM, dtype=np.float32)
    step[3:6] = (0.0, 0.0, -1.0)
    if hist_wheel_slots == "policy":            # wheels at 12..15 (legs, wheels, arm)
        step[6 + LEG:6 + LEG + WHEEL] = wheel_vel
        step[6 + ROBOT_JOINTS + LEG:6 + ROBOT_JOINTS + LEG + WHEEL] = wheel_vel
    elif hist_wheel_slots == "mjcf":            # wheels interleaved per leg (3,7,11,15)
        for leg in range(4):
            step[6 + leg * 4 + 3] = wheel_vel
            step[6 + ROBOT_JOINTS + leg * 4 + 3] = wheel_vel
    hist = np.tile(step, HIST_STEPS)
    return obs[None, :], hist[None, :]


def run(sess, obs, hist):
    return sess.run(["actions"], {"obs": obs, "obs_history": hist})[0][0]


def show(tag, a):
    legs = " ".join("%+.2f" % v for v in a[0:12])
    wheels = " ".join("%+.3f" % v for v in a[12:16])
    ik = " ".join("%+.2f" % v for v in a[16:23])
    print("%-44s legs[%s] wheels[%s] ik[%s]" % (tag, legs, wheels, ik))


def main():
    sess = ort.InferenceSession(POLICY, providers=["CPUExecutionProvider"])
    print("policy :", POLICY)
    print("inputs :", [(i.name, i.shape) for i in sess.get_inputs()])
    print("outputs:", [(o.name, o.shape) for o in sess.get_outputs()])
    print("wheel action scale 5.0 rad/s, deploy clip +-3 raw (= +-15 rad/s)\n")

    obs, hist = build(0.0)
    show("nominal, history wheels = 0", run(sess, obs, hist))

    obs, hist = build(WHEEL_VEL, hist_wheel_slots="policy")
    show("history wheels %+.1f @ 12..15" % WHEEL_VEL, run(sess, obs, hist))

    obs, hist = build(WHEEL_VEL, hist_wheel_slots="mjcf")
    show("history wheels %+.1f @ 3,7,11,15" % WHEEL_VEL, run(sess, obs, hist))

    for w in (1.0, 5.0, 15.0, -5.0, -15.0):
        obs, hist = build(0.0)
        obs[0, 31 + LEG:31 + LEG + WHEEL] = w * 0.05
        show("policy obs wheel vel %+.0f rad/s (x0.05)" % w, run(sess, obs, hist))


if __name__ == "__main__":
    main()
