"""
VR teleop device (no Isaac Lab dependency).

This is a standalone port of the training-side ``Se2VRExtended`` logic:
it runs the XLeVR HTTPS + WebSocket server and converts Quest-class WebXR
controller packets into a 13-D command vector with the same semantics:

    [vx, vy, wz, ee_dx, ee_dy, ee_dz, ee_droll, ee_dpitch, ee_dyaw,
     dheight, dpitch, droll, gripper]

Unlike the training script, no ``absolute_commands`` flag is introduced in
the deployment. The arm/body integrations keep their existing incremental
interface; ``advance()`` additionally reports calibrate/reset events so
consumers can re-anchor when the operator presses B / X / Y.

The XLeVR sources (server + web-ui + certs) live under
``third_party/XLeVR`` and retain their upstream LICENSE.
"""

from __future__ import annotations

import asyncio
import http.server
import os
import socket
import ssl
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

XLEVR_DIR = Path(__file__).resolve().parents[3] / "third_party" / "XLeVR"
sys.path.insert(0, str(XLEVR_DIR))

from xlevr.config import XLeVRConfig
from xlevr.inputs.vr_ws_server import VRWebSocketServer

_IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def _xyzw_to_wxyz(q):
    """Convert [x,y,z,w] -> [w,x,y,z]."""
    return np.array([q[3], q[0], q[1], q[2]], dtype=np.float64)


# 90-degree rotation around X: VR wrist frame -> sim frame (training-tuned)
_R_ALIGN = R.from_euler("x", 90, degrees=True)


@dataclass
class Se2VRExtendedCfg:
    """Sensitivity/axis-map configuration (initial values copied verbatim)."""
    v_x_sensitivity: float = 5.0
    v_y_sensitivity: float = 1.0
    omega_z_sensitivity: float = 1.0
    height_sensitivity: float = 0.5
    pitch_sensitivity: float = 0.35
    roll_sensitivity: float = 0.25
    arm_pos_sensitivity: float = 1.0
    arm_rot_sensitivity: float = 1.0
    arm_rot_axis_map: tuple = (
        (1, +1.0),
        (0, -1.0),
        (2, +1.0),
    )
    arm_pos_axis_map: tuple = (
        (1, +1.0),
        (0, -1.0),
        (2, +1.0),
    )
    pos_deadzone: float = 0.004
    rot_deadzone: float = 0.006


class Se2VRExtendedDevice:
    """VR controller without Isaac Lab, exposing advance()/events."""

    def __init__(self, cfg: Optional[Se2VRExtendedCfg] = None):
        self._cfg = cfg or Se2VRExtendedCfg()

        self.v_x_sensitivity = self._cfg.v_x_sensitivity
        self.v_y_sensitivity = self._cfg.v_y_sensitivity
        self.omega_z_sensitivity = self._cfg.omega_z_sensitivity
        self.height_sensitivity = self._cfg.height_sensitivity
        self.pitch_sensitivity = self._cfg.pitch_sensitivity
        self.roll_sensitivity = self._cfg.roll_sensitivity
        self.arm_pos_sensitivity = self._cfg.arm_pos_sensitivity
        self.arm_rot_sensitivity = self._cfg.arm_rot_sensitivity
        self.pos_deadzone = self._cfg.pos_deadzone
        self.rot_deadzone = self._cfg.rot_deadzone
        self.arm_rot_axis_map = self._cfg.arm_rot_axis_map
        self.arm_pos_axis_map = self._cfg.arm_pos_axis_map

        self._started = False
        self._reset_state = False
        self._calibrate_pulse = False
        self._reset_event = None

        self._vr_origin = {
            "left": {"pos": None, "rot": None},
            "right": {"pos": None, "rot": None},
        }
        self._calibration_triggered = {"left": False, "right": False}
        self._latest_vr_pose = {"left": None, "right": None}
        self._last_buttons = {
            "right_a": False, "right_b": False,
            "left_x": False, "left_y": False,
        }

        self._command = np.zeros(13, dtype=np.float64)
        self._body_height_offset = 0.0
        self._dt = 0.0
        self._last_packet_time = time.time()
        self._prev_arm_pos = None
        self._prev_arm_rot = None
        self._prev_body_rot = None

        self._command_queue = asyncio.Queue()
        cfg_xlevr = XLeVRConfig()
        cfg_xlevr.enable_vr = True
        cfg_xlevr.enable_https = True
        cfg_xlevr.certfile = str(XLEVR_DIR / "cert.pem")
        cfg_xlevr.keyfile = str(XLEVR_DIR / "key.pem")

        self._vr_server = VRWebSocketServer(
            command_queue=self._command_queue,
            config=cfg_xlevr,
            print_only=False,
        )
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_vr_services, daemon=True)
        self._thread.start()
        time.sleep(1.0)
        self._display_info()

    # ------------------------------------------------------------------
    def reset(self):
        self._command[:] = 0.0
        self._body_height_offset = 0.0
        self._vr_origin = {
            "left": {"pos": None, "rot": None},
            "right": {"pos": None, "rot": None},
        }
        self._calibration_triggered = {"left": False, "right": False}
        self._prev_arm_pos = None
        self._prev_arm_rot = None
        self._prev_body_rot = None
        self._last_packet_time = time.time()

    def advance(self):
        """Poll the VR queue; return (cmd13, events)."""
        self._poll_queue()
        events = {
            "active": self._started,
            "calibrate": self._calibrate_pulse,
            "reset_fail": self._reset_event == "fail",
            "reset_success": self._reset_event == "success",
        }
        self._calibrate_pulse = False
        self._reset_event = None
        return self._command.copy(), events

    # ------------------------------------------------------------------
    def _run_vr_services(self):
        asyncio.set_event_loop(self._loop)
        try:
            handler = _SimpleFileHandler
            handler.web_root = str(XLEVR_DIR / "web-ui")
            httpd = http.server.HTTPServer(
                ("0.0.0.0", 8443), handler)
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(str(XLEVR_DIR / "cert.pem"),
                                str(XLEVR_DIR / "key.pem"))
            httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
            threading.Thread(target=httpd.serve_forever, daemon=True).start()
            print("HTTPS server on port 8443")
        except Exception as exc:
            print(f"[vr_teleop] HTTPS server failed: {exc}")

        try:
            self._loop.run_until_complete(self._vr_server.start())
            print("VR WebSocket server on port 8442")
            self._loop.run_forever()
        except Exception as exc:
            print(f"[vr_teleop] VR loop failed: {exc}")

    def _display_info(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            ip = "localhost"
        print("\n" + "=" * 50)
        print("VR teleop ready")
        print(f"  Open in your headset browser:  https://{ip}:8443")
        print("=" * 50 + "\n")

    # ------------------------------------------------------------------
    # queue -> command conversion (ported from vr_extented.py)
    # ------------------------------------------------------------------
    def _poll_queue(self):
        while not self._command_queue.empty():
            try:
                goal = self._command_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

            if goal.arm == "headset":
                continue

            now = time.time()
            self._dt = min(max(now - self._last_packet_time, 0.0), 0.1)
            self._last_packet_time = now

            if goal.metadata and "buttons" in goal.metadata:
                self._check_buttons(goal.metadata["buttons"],
                                    goal.metadata.get("hand", goal.arm))

            if not self._started or goal.target_position is None:
                continue

            sim_pos, r_sim = self._vr_to_sim_pose(goal)
            hand = goal.arm

            if self._calibration_triggered.get(hand, False):
                self._vr_origin[hand]["pos"] = sim_pos.copy()
                self._vr_origin[hand]["rot"] = r_sim
                self._calibration_triggered[hand] = False

            if self._vr_origin[hand]["pos"] is None:
                continue

            delta_pos = sim_pos - self._vr_origin[hand]["pos"]
            r_diff_local = self._vr_origin[hand]["rot"].inv() * r_sim

            trigger = 0.0
            if goal.metadata and "trigger" in goal.metadata:
                trigger = float(goal.metadata["trigger"])

            if hand == "right":
                self._update_arm(delta_pos, r_diff_local, trigger)
                self._update_yaw_from_thumbstick(goal)
            elif hand == "left":
                self._update_body(delta_pos, r_diff_local, trigger)
                self._update_chassis_from_thumbstick(goal)

    def _vr_to_sim_pose(self, goal):
        raw = goal.target_position
        sim_pos = np.array([raw[0], -raw[2], raw[1]], dtype=np.float64)
        raw_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        if goal.metadata and "quaternion" in goal.metadata:
            q = goal.metadata["quaternion"]
            raw_q = np.array([q.get("x", 0.), q.get("y", 0.),
                              q.get("z", 0.), q.get("w", 1.)])
        r_vr = R.from_quat(raw_q)
        r_sim = _R_ALIGN * r_vr * _R_ALIGN.inv()
        return sim_pos, r_sim

    @staticmethod
    def _apply_deadzone(v, threshold):
        return 0.0 if abs(v) < threshold else float(v)

    def _update_arm(self, delta_pos, r_diff_local, trigger):
        for i, (src, sgn) in enumerate(self.arm_pos_axis_map):
            v = self._apply_deadzone(sgn * delta_pos[src], self.pos_deadzone)
            self._command[3 + i] = v * self.arm_pos_sensitivity

        euler_local = r_diff_local.as_euler("XYZ", degrees=False)
        for i, (src, sgn) in enumerate(self.arm_rot_axis_map):
            v = self._apply_deadzone(sgn * euler_local[src], self.rot_deadzone)
            self._command[6 + i] = v * self.arm_rot_sensitivity

        # 1 = closed, 0 = open (deployment UserCommand convention)
        self._command[12] = 1.0 if trigger > 0.5 else 0.0

    def _update_yaw_from_thumbstick(self, goal):
        if not (goal.metadata and "thumbstick" in goal.metadata):
            return
        stick = goal.metadata["thumbstick"]
        stick_x = -float(stick.get("x", 0.0))
        self._command[2] = stick_x * self.omega_z_sensitivity

    def _update_body(self, delta_pos, r_diff, trigger):
        if trigger > 0.5:
            self._body_height_offset += self.height_sensitivity * self._dt
            self._body_height_offset = float(
                np.clip(self._body_height_offset, -0.5, 0.5))
        self._command[9] = (
            self._apply_deadzone(delta_pos[2], self.pos_deadzone)
            * self.height_sensitivity
            + self._body_height_offset
        )
        euler = r_diff.as_euler("XYZ", degrees=False)
        self._command[10] = (
            -self._apply_deadzone(euler[0], self.rot_deadzone)
            * self.pitch_sensitivity
        )
        self._command[11] = (
            self._apply_deadzone(euler[1], self.rot_deadzone)
            * self.roll_sensitivity
        )

    def _update_chassis_from_thumbstick(self, goal):
        if not (goal.metadata and "thumbstick" in goal.metadata):
            return
        stick = goal.metadata["thumbstick"]
        stick_x = float(stick.get("x", 0.0))
        stick_y = float(stick.get("y", 0.0))
        self._command[0] = -stick_y * self.v_x_sensitivity
        self._command[1] = -stick_x * self.v_y_sensitivity

    def _check_buttons(self, buttons_dict, hand):
        is_right = "right" in str(hand).lower()
        is_left = "left" in str(hand).lower()
        for key, pressed in buttons_dict.items():
            k = str(key).lower()
            if is_right:
                if k == "a":
                    self._edge_button("right_a", bool(pressed))
                elif k == "b":
                    self._edge_button("right_b", bool(pressed))
            elif is_left:
                if k == "a":
                    self._edge_button("left_x", bool(pressed))
                elif k == "b":
                    self._edge_button("left_y", bool(pressed))

    def _edge_button(self, uid, pressed):
        was = self._last_buttons.get(uid, False)
        if pressed and not was:
            parts = uid.split("_")
            self._on_button_press(parts[1], parts[0])
        self._last_buttons[uid] = pressed

    def _on_button_press(self, btn, hand):
        if btn == "b":
            print("[VR] Button B -> START + calibrate")
            self._started = True
            self._reset_state = False
            self._command[:] = 0.0
            self._body_height_offset = 0.0
            self._calibration_triggered["left"] = True
            self._calibration_triggered["right"] = True
            self._calibrate_pulse = True
        elif btn == "x":
            print("[VR] Button X -> RESET (fail)")
            self._started = False
            self._reset_state = True
            self.reset()
            self._reset_event = "fail"
        elif btn == "y":
            print("[VR] Button Y -> RESET (success)")
            self._started = False
            self._reset_state = True
            self.reset()
            self._reset_event = "success"


class _SimpleFileHandler(http.server.SimpleHTTPRequestHandler):
    web_root = ""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=self.web_root, **kwargs)

    def log_message(self, fmt, *args):
        pass
