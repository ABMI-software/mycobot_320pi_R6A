#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PyQt5 real-time validation dashboard — DREAM vs encoders.

Dashboard 1 (implemented): live camera feed with two overlays per keypoint —
a big green circle reprojected from the ENCODER angles (known ground truth,
via forward_kinematics + solvePnP against the DREAM 2D detections, camera
intrinsics only, no pre-calibrated extrinsic) and a small magenta circle at
the raw DREAM 2D detection. The closer the small circle sits to the center of
the big one, the better DREAM's keypoint detection.

Dashboards 2 (pilotage) and 3 (courbes 6 joints) are placeholders: comparing
DREAM-estimated joint angles against the encoders requires solving for both
the joint angles AND the camera pose from a single monocular view, which is
mathematically ill-posed (verified: near-zero reprojection error is reachable
with >60 deg wrong angles — see training/dream/dream_angle_solver.py and the
already-documented ambiguity in training/dream/estimate_angles_from_keypoints.py).
Parked until an anchoring strategy (per-session pose calibration or multi-cam
triangulation) is chosen.

Usage:
    ros2 run mycobot_gateway dream_validation_dashboard
    ros2 run mycobot_gateway dream_validation_dashboard --ros-args -p sim:=true
"""

import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow,
    QTabWidget, QVBoxLayout, QWidget,
)

PKG_DIR = Path(__file__).resolve().parent
ASSETS_DIR = PKG_DIR / 'assets'
LOGO_PATH = ASSETS_DIR / 'abmi_engineering_logo.jpeg'

# training/dream sits two levels up from mycobot_gateway/mycobot_gateway/
_REPO_ROOT = PKG_DIR.parent.parent
_TRAINING_DREAM = _REPO_ROOT / 'training' / 'dream'
_CALIBRATION_DIR = _REPO_ROOT / 'training' / 'calibration'
if str(_TRAINING_DREAM) not in sys.path:
    sys.path.insert(0, str(_TRAINING_DREAM))

from mycobot_fk import forward_kinematics, KEYPOINT_NAMES, GAZEBO_INTRINSICS  # noqa: E402

# /joint_states name order — matches joint_sync.py / dream_inference_node.py
JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

KP_LABELS = [n.replace('mycobot320_', '') for n in KEYPOINT_NAMES]

BIG_COLOR_BGR = (0, 200, 0)      # encoder reconstruction — green
SMALL_COLOR_BGR = (230, 0, 200)  # DREAM detection — magenta


def load_arducam_intrinsics():
    """Real Arducam K + distortion, from the existing charuco calibration."""
    meta_path = _CALIBRATION_DIR / 'cam_3.meta.json'
    meta = json.loads(meta_path.read_text())
    r = meta['results']
    K = np.array([[r['fx'], 0.0, r['cx']],
                  [0.0, r['fy'], r['cy']],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array(r['dist_coeffs'], dtype=np.float64)
    return K, dist


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    encoding = msg.encoding.lower()
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).copy()

    if encoding == 'rgb8':
        return arr.reshape((h, w, 3))[:, :, ::-1].copy()
    if encoding == 'bgr8':
        return arr.reshape((h, w, 3))
    if encoding == 'rgba8':
        return arr.reshape((h, w, 4))[:, :, 2::-1].copy()
    if encoding == 'bgra8':
        return arr.reshape((h, w, 4))[:, :, :3].copy()
    arr = arr.reshape((h, w, -1))
    return arr[:, :, :3].copy() if arr.shape[2] >= 3 else np.repeat(arr, 3, axis=2)


class DashboardNode(Node):
    """Owns all ROS2 I/O. No Qt objects touched here except via plain data."""

    def __init__(self):
        super().__init__('dream_validation_dashboard')

        self.declare_parameter('sim', False)
        self.declare_parameter('camera_topic', '')
        sim = self.get_parameter('sim').value
        camera_topic_param = self.get_parameter('camera_topic').value

        if sim:
            self.camera_topic = camera_topic_param or '/synth_camera/image'
            self.camera_K = GAZEBO_INTRINSICS.copy()
            self.camera_dist = None
        else:
            self.camera_topic = camera_topic_param or '/camera/image_raw'
            self.camera_K, self.camera_dist = load_arducam_intrinsics()

        self.get_logger().info(
            f'📷 camera_topic={self.camera_topic}  fx={self.camera_K[0,0]:.1f} '
            f'fy={self.camera_K[1,1]:.1f} cx={self.camera_K[0,2]:.1f} cy={self.camera_K[1,2]:.1f}'
        )

        # ── Live state ──
        self.latest_bgr = None
        self.latest_joint_q = None          # (6,) rad
        self.latest_kp_2d = None            # (7,2) px
        self.latest_kp_valid = None         # (7,) bool
        self.latest_status = ''
        self.frame_count = 0
        self.kp_count = 0
        self._fps_window = []
        self._kp_recv_times = []

        # ── Manual pilot state (mirrors simple_gui.py) ──
        self.current_angles_deg = [0.0] * 6
        self.current_coords = [0.0] * 6
        self.last_feedback = None

        # ── Subscriptions ──
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(Image, self.camera_topic, self._image_cb, img_qos)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/dream/keypoints', self._kp_cb, 10)
        self.create_subscription(String, '/dream/status', self._status_cb, 10)

        # ── Manual pilot pub/sub ──
        self.cmd_pub = self.create_publisher(String, '/to_robot', 10)
        self.create_subscription(String, '/from_robot', self._feedback_cb, 10)

    # ── Callbacks ──────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        self.latest_bgr = image_msg_to_bgr(msg)
        self.frame_count += 1
        now = time.time()
        self._fps_window.append(now)
        cutoff = now - 2.0
        self._fps_window = [t for t in self._fps_window if t > cutoff]

    def _joint_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.latest_joint_q = np.array(
                [name_to_pos[j] for j in JOINT_NAMES], dtype=np.float64)
        except KeyError:
            pass

    def _kp_cb(self, msg: Float64MultiArray):
        flat = msg.data
        n = len(flat) // 3
        kp = np.zeros((n, 2), dtype=np.float64)
        valid = np.zeros(n, dtype=bool)
        for i in range(n):
            kp[i] = [flat[3 * i], flat[3 * i + 1]]
            valid[i] = flat[3 * i + 2] > 0.5
        self.latest_kp_2d = kp
        self.latest_kp_valid = valid
        self.kp_count += 1
        now = time.time()
        self._kp_recv_times.append(now)
        self._kp_recv_times = [t for t in self._kp_recv_times if t > now - 5.0]

    def _status_cb(self, msg: String):
        self.latest_status = msg.data

    def _feedback_cb(self, msg: String):
        self.last_feedback = msg.data
        if msg.data.startswith('angles:'):
            try:
                self.current_angles_deg = eval(msg.data.replace('angles:', ''))
            except Exception:
                pass
        if msg.data.startswith('coords:'):
            try:
                self.current_coords = eval(msg.data.replace('coords:', ''))
            except Exception:
                pass

    # ── Manual pilot commands (same wire protocol as simple_gui.py) ──
    def send_angles(self, angles, speed=50):
        self._publish_cmd({'action': 'send_angles', 'angles': angles, 'speed': speed})

    def send_coords(self, coords, speed=50, mode=0):
        self._publish_cmd({'action': 'send_coords', 'coords': coords, 'speed': speed, 'mode': mode})

    def get_angles(self):
        self._publish_cmd({'action': 'get_angles'})

    def get_coords(self):
        self._publish_cmd({'action': 'get_coords'})

    def _publish_cmd(self, cmd: dict):
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

    # ── Derived quantities ─────────────────────────────────────
    def fps(self) -> float:
        if len(self._fps_window) < 2:
            return 0.0
        span = self._fps_window[-1] - self._fps_window[0]
        return (len(self._fps_window) - 1) / span if span > 0 else 0.0

    def dream_rate_hz(self) -> float:
        if len(self._kp_recv_times) < 2:
            return 0.0
        span = self._kp_recv_times[-1] - self._kp_recv_times[0]
        return (len(self._kp_recv_times) - 1) / span if span > 0 else 0.0

    def compute_overlay(self):
        """Big circles (encoder FK reprojected) + small circles (raw DREAM) + per-kp error.

        Returns None if we don't have enough live data to compute anything yet.
        """
        if self.latest_joint_q is None or self.latest_kp_2d is None:
            return None
        valid = self.latest_kp_valid
        if valid is None or valid.sum() < 4:
            return None

        positions, _ = forward_kinematics(self.latest_joint_q)
        pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
        idx = np.where(valid)[0]

        ok, rvec, tvec = cv2.solvePnP(
            pts_3d[idx], self.latest_kp_2d[idx], self.camera_K, self.camera_dist,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not ok:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(
            pts_3d[idx], self.latest_kp_2d[idx], self.camera_K, self.camera_dist, rvec, tvec)

        big_px, _ = cv2.projectPoints(pts_3d, rvec, tvec, self.camera_K, self.camera_dist)
        big_px = big_px.reshape(-1, 2)

        err_px = np.full(len(KEYPOINT_NAMES), np.nan)
        err_px[idx] = np.linalg.norm(big_px[idx] - self.latest_kp_2d[idx], axis=1)
        rms_px = float(np.sqrt(np.nanmean(err_px[idx] ** 2)))

        return {
            'big_px': big_px, 'small_px': self.latest_kp_2d, 'valid': valid,
            'err_px': err_px, 'rms_px': rms_px, 'n_valid': int(valid.sum()),
        }


class Tab1LiveValidation(QWidget):
    """Camera feed + big(encoder)/small(DREAM) circle overlay + per-joint error panel."""

    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        layout = QHBoxLayout(self)

        self.image_label = QLabel('En attente d\'image caméra…')
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setStyleSheet('background-color: #111; color: #999;')
        layout.addWidget(self.image_label, stretch=3)

        side = QVBoxLayout()
        layout.addLayout(side, stretch=1)

        self.status_label = QLabel('Statut : —')
        self.status_label.setFont(QFont('Sans', 10, QFont.Bold))
        side.addWidget(self.status_label)

        self.fps_label = QLabel('Caméra : — FPS')
        side.addWidget(self.fps_label)
        self.dream_rate_label = QLabel('DREAM : — Hz')
        side.addWidget(self.dream_rate_label)
        self.rms_label = QLabel('Erreur RMS reprojection : — px')
        side.addWidget(self.rms_label)

        legend = QLabel(
            '<span style="color:#00c800;">●</span> grand cercle = encodeurs (FK)<br>'
            '<span style="color:#e600c8;">●</span> petit cercle = DREAM (détection 2D)'
        )
        side.addWidget(legend)

        side.addWidget(self._hline())

        self.kp_grid = QGridLayout()
        side.addLayout(self.kp_grid)
        header_kp = QLabel('Keypoint')
        header_kp.setFont(QFont('Sans', 9, QFont.Bold))
        header_err = QLabel('Erreur (px)')
        header_err.setFont(QFont('Sans', 9, QFont.Bold))
        self.kp_grid.addWidget(header_kp, 0, 0)
        self.kp_grid.addWidget(header_err, 0, 1)
        self.err_value_labels = []
        for i, label in enumerate(KP_LABELS):
            self.kp_grid.addWidget(QLabel(label), i + 1, 0)
            val = QLabel('—')
            self.kp_grid.addWidget(val, i + 1, 1)
            self.err_value_labels.append(val)

        side.addStretch()

    @staticmethod
    def _hline():
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def refresh(self):
        n = self.node
        self.fps_label.setText(f'Caméra : {n.fps():.1f} FPS')
        self.dream_rate_label.setText(f'DREAM : {n.dream_rate_hz():.1f} Hz')
        self.status_label.setText(f'Statut : {n.latest_status or "—"}')

        if n.latest_bgr is None:
            return
        frame = n.latest_bgr.copy()
        overlay = n.compute_overlay()

        if overlay is not None:
            self.rms_label.setText(f'Erreur RMS reprojection : {overlay["rms_px"]:.2f} px '
                                    f'({overlay["n_valid"]}/7 kp)')
            for i in range(len(KEYPOINT_NAMES)):
                bx, by = overlay['big_px'][i]
                cv2.circle(frame, (int(round(bx)), int(round(by))), 12, BIG_COLOR_BGR, 2)
                if overlay['valid'][i]:
                    sx, sy = overlay['small_px'][i]
                    cv2.circle(frame, (int(round(sx)), int(round(sy))), 4, SMALL_COLOR_BGR, -1)
                    self.err_value_labels[i].setText(f'{overlay["err_px"][i]:.1f}')
                else:
                    self.err_value_labels[i].setText('non détecté')
        else:
            self.rms_label.setText('Erreur RMS reprojection : — px (données insuffisantes)')
            for lbl in self.err_value_labels:
                lbl.setText('—')

        self._display_frame(frame)

    def _display_frame(self, frame_bgr: np.ndarray):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            self.image_label.width(), self.image_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image_label.setPixmap(pix)


class PlaceholderTab(QWidget):
    def __init__(self, text: str):
        super().__init__()
        layout = QVBoxLayout(self)
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont('Sans', 12))
        label.setStyleSheet('color: #777;')
        layout.addWidget(label)


class DashboardWindow(QMainWindow):
    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('ABMI — DREAM Validation Dashboard')
        self.resize(1100, 650)

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.addWidget(self._build_header())

        self.tabs = QTabWidget()
        self.tab1 = Tab1LiveValidation(node)
        self.tabs.addTab(self.tab1, '1 — Validation temps réel')
        self.tabs.addTab(
            PlaceholderTab('Dashboard 2 — Pilotage & KPI\n(à venir)'),
            '2 — Pilotage')
        self.tabs.addTab(
            PlaceholderTab('Dashboard 3 — Courbes 6 joints\n(à venir — nécessite un ancrage de pose caméra)'),
            '3 — Courbes joints')
        outer.addWidget(self.tabs)

        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)  # ~30 Hz UI/ROS pump

    def _build_header(self):
        header = QFrame()
        h = QHBoxLayout(header)
        logo = QLabel()
        if LOGO_PATH.exists():
            pix = QPixmap(str(LOGO_PATH)).scaledToHeight(48, Qt.SmoothTransformation)
            logo.setPixmap(pix)
        h.addWidget(logo, alignment=Qt.AlignLeft)

        title = QLabel('DREAM — Validation IA vs Encodeurs')
        title.setFont(QFont('Sans', 14, QFont.Bold))
        h.addWidget(title)
        h.addStretch()
        return header

    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.tab1.refresh()

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    app = QApplication(sys.argv)
    window = DashboardWindow(node)
    window.show()

    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
