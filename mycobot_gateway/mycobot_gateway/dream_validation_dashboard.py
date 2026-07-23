#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PyQt5 real-time validation dashboard — DREAM vs encoders.

Dashboard 1 (implemented): live camera feed with two overlays per keypoint —
a big green circle reprojected from the ENCODER angles (known ground truth,
via forward_kinematics + solvePnP against the DREAM 2D detections, camera
intrinsics only, no pre-calibrated extrinsic) and a small magenta circle at
the raw DREAM 2D detection. The closer the small circle sits to the center of
the big one, the better DREAM's keypoint detection.

Dashboards 2 (pilotage) and 3 (courbes 6 joints): DREAM-estimated joint
angles are obtained by solving for BOTH the 6 joint angles AND the 6-DoF
camera pose jointly, every frame (dream_angle_solver.solve_joint_angles_and_pose_two_pass),
warm-started from the previous frame's own solution for temporal continuity.
No fixed/persisted camera->robot extrinsic is used — a session-frozen
"anchor" pose was tried and removed (2026-07-15): it degrades to a stale
calibration the moment the camera is physically nudged, silently corrupting
every angle estimate for the rest of the session with no way to detect it.
Re-solving the pose alongside the angles on every frame means a bumped
camera degrades pose accuracy for that frame instead of poisoning all
subsequent frames.

Usage:
    ros2 run mycobot_gateway dream_validation_dashboard
    ros2 run mycobot_gateway dream_validation_dashboard --ros-args -p sim:=true
"""

import csv
import json
import math
import os
import random
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QIcon, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QScrollArea, QSpinBox, QVBoxLayout, QWidget,
)
import pyqtgraph as pg

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
from dream_angle_solver import (  # noqa: E402
    solve_joint_angles_and_pose, solve_joint_angles_and_pose_two_pass,
    solve_joint_angles_and_pose_multi_init, solve_joint_angles_and_pose_bounded,
    JOINT_LOWER, JOINT_UPPER,
)

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

# Acquisition CSV : durée de capture par pose, et dossier de sortie.
ACQ_DURATION_S = 4.0
ACQUISITION_DIR = _TRAINING_DREAM / 'acquisitions'

# ── Poses du mode automatique — reprises telles quelles de
#    training/capture_real_3cam.py (random_joint_angles + _pose_is_safe) ──
AUTO_TOTAL_S = 60.0
_JOINT_LIMITS = [(-2.96, 2.96), (-2.79, 2.79), (-2.79, 2.79),
                 (-2.79, 2.79), (-2.96, 2.96), (-3.05, 3.05)]
_BASE_H, _L_UPPER, _L_FORE, _L_FORE_Z = 162.0, 136.35, 120.5, 82.0
_L_WRIST, _L_EE, _TABLE_Z_MIN, _BASE_R_MIN = 84.0, 66.35, 60.0, 90.0


def _fk_key_points(j2, j3, j4):
    a2 = math.radians(j2)
    a3 = math.radians(j2 + j3)
    a4 = math.radians(j2 + j3 + j4)
    z_elbow = _BASE_H + _L_UPPER * math.cos(a2)
    r_elbow = _L_UPPER * math.sin(a2)
    z_wrist = z_elbow + _L_FORE * math.cos(a3) - _L_FORE_Z * math.sin(a3)
    r_wrist = r_elbow + _L_FORE * math.sin(a3) + _L_FORE_Z * math.cos(a3)
    l_ee = _L_WRIST + _L_EE
    z_ee = z_wrist + l_ee * math.cos(a4)
    r_ee = r_wrist + l_ee * math.sin(a4)
    return [(z_elbow, abs(r_elbow)), (z_wrist, abs(r_wrist)), (z_ee, abs(r_ee))]


def _pose_is_safe(angles_deg):
    for z, r in _fk_key_points(angles_deg[1], angles_deg[2], angles_deg[3]):
        if z < _TABLE_Z_MIN:
            return False
        if z < _BASE_H and r < _BASE_R_MIN:
            return False
    return True


def random_joint_angles(limit_fraction=0.5, max_attempts=500):
    """Pose aléatoire sûre, identique à capture_real_3cam.py : chaque joint dans
    limit_fraction de sa course, rejetée si la FK entre en collision table/base."""
    for _ in range(max_attempts):
        angles_deg = []
        for lo, hi in _JOINT_LIMITS:
            lo_d, hi_d = math.degrees(lo), math.degrees(hi)
            span = (hi_d - lo_d) * limit_fraction
            mid = (hi_d + lo_d) / 2.0
            angles_deg.append(round(random.uniform(mid - span / 2, mid + span / 2), 1))
        if _pose_is_safe(angles_deg):
            return angles_deg
    return [0.0] * 6

BIG_COLOR_BGR = (0, 200, 0)      # encoder reconstruction — green
SMALL_COLOR_BGR = (230, 0, 200)  # DREAM detection — magenta

# Which KEYPOINT_NAMES indices actually carry information about each joint.
# A link's own position never depends on its own joint's rotation (see
# mycobot_fk.forward_kinematics: the joint rotation is applied AFTER the
# translation, at the link's own origin, so it doesn't move that link) — only
# on the joints that come before it in the chain. So joint m (0-based, J{m+1})
# is only observed by keypoints strictly downstream of it: kp[m+2:]. J6 (m=5)
# has none — no keypoint in this 7-point set is sensitive to it, ever.
JOINT_OBSERVING_KP = {m: list(range(m + 2, 7)) for m in range(6)}

# Live (per-frame) confidence threshold, in reprojection px, for flagging a
# joint's DREAM estimate as degraded — NOT a permanent label like J6's: J4/J5
# are only unreliable when their supporting keypoints are actually noisy
# right now, so the indicator should reflect the current frame, not be glued
# on regardless of live detection quality. Picked from the real_3cam offline
# validation (training/dream/validate_angle_solver_real.py): well-detected
# points sit at 2-6px median, degraded ones at 12-17px median with long
# outlier tails — 15px sits between the two.
JOINT_CONFIDENCE_PX_THRESHOLD = 15.0

# Single source of truth for "recent window" — shared by JointCurvesPanel
# (what the graphs plot) and DashboardNode.angle_error_stats_window() (what
# the windowed MAE/RMS is computed over), so the two can never silently
# drift apart the way the old single "MAE totale" (a session-wide, since
# node-start running counter) did against the 30s-windowed graphs.
CURVE_WINDOW_S = 30.0

# Hard per-frame bound on the camera pose delta (solve_joint_angles_and_pose's
# max_drot_rad/max_dtrans_m) — expressed as a rate (per second) rather than a
# fixed one-shot number since DREAM's own inference rate varies (~1.4-5 Hz
# observed); the bound applied on any given solve is rate * dt, dt = time
# since the last ACCEPTED pose. The Arducam is a fixed/tripod mount, not
# handheld — first-pass values, not fit to any specific dataset: generous
# enough to tolerate a deliberate slow nudge, tight enough that an instant
# big jump gets flagged as a rupture rather than silently absorbed as if it
# were joint motion (see diagnose_local_minima.py, 2026-07-15: jac_cond
# ~1e17-1e18 with pose left unconstrained).
MAX_CAMERA_ROT_RATE_DEG_S = 10.0
MAX_CAMERA_TRANS_RATE_M_S = 0.03
# Above this gap between solves, a single-frame "rate x dt" bound stops
# meaning anything (e.g. after a long detection dropout) — skip the hard
# bound for that one solve rather than handing it a huge, effectively
# meaningless allowance.
POSE_BOUND_MAX_DT_S = 2.0


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


# Per-joint measurement noise (deg, converted to rad^2 in KalmanAngle1D) for
# the temporal Kalman filter below — derived from the ORACLE-init per-joint
# MEDIAN (not RMS — RMS is inflated by the catastrophic-outlier tail this
# filter's gate exists to reject, so using it as R would make the gate too
# loose to ever fire) in training/dream/angle_error_diagnosis.py
# (2026-07-13): J1=6.9°, J2=6.7°, J3=13.7°, J4=13.6°, J5=10.6°. J6 gets a huge
# R because it's structurally unobservable (0 keypoints depend on its
# rotation, see JOINT_OBSERVING_KP) — the filter should treat any
# "measurement" for J6 as near-worthless and just hold/drift.
_KF_MEASUREMENT_MEDIAN_DEG = [6.9, 6.7, 13.7, 13.6, 10.6, 500.0]

# Per-joint solver regularization used ONLY in consistency mode
# (use_encoder_seed). J1 stays on the nominal light prior (1.5). J2-J5 get a
# strong prior (40): J3-J5 are the poorly-observed distal joints whose angle
# drifts tens of degrees at near-constant reprojection (the monocular
# observability gap), and J2 — though well-observed — flips to the wrong
# monocular branch under the near-top-down camera (front/back depth ambiguity),
# reprojecting just as well on the wrong branch (measured ~45 deg error at
# ~10px reproj, 2026-07-22); pinning it to the encoder branch drops that to
# ~2-3 deg. The distal sweep on 150 real poses (scratchpad distal_reg_sweep.py,
# 2026-07-15) measured w=40 cutting MAE(J3-J5) 18.3->6.2 deg for +2.4px
# reprojection (3.0->5.4px, still inside the detection-noise band) — a
# consistency/refinement result, NOT an independent DREAM prediction. J6 stays
# lightly regularized: it has 0 observing keypoints, so it sits at the encoder
# seed no matter the weight (nothing pulls it off). Meaningless outside encoder
# seeding: with a free (non-encoder) q_init, pinning joints to a wrong branch
# would only lock in the error.
_CONSISTENCY_REG_VEC = [10.0, 40.0, 40.0, 40.0, 40.0, 1.5]
_KF_OUTLIER_GATE_SIGMA = 3.0  # reject an update whose innovation exceeds this many predicted std devs

# Bruit de process sur la VITESSE. Mesuré 2026-07-20 sur un signal réaliste
# (bras immobile + plateaux parasites de 20-45° dus aux bascules de branche) :
#   qvel=15° (ancien) -> ecart-type 18.9°, excursion max 50°, retard 1.0°
#   qvel=0.1°         -> ecart-type  8.3°, excursion max 21°, retard 4.9°
# Le bras est immobile l'essentiel du temps et l'erreur propre de DREAM est de
# 7-14° selon le joint, donc 4.9° de retard sur un mouvement franc coûte moins
# que les excursions qu'on supprime. Ne PAS descendre la porte a 1.5 en plus :
# la sortie devient parfaitement plate (ecart-type 0.00°) parce que le filtre
# rejette aussi les vrais mouvements — courbe flatteuse, mesure vide.
_KF_Q_VEL_DEG = np.radians(0.1)

# Half-height of the per-joint plot window, in degrees around the encoder value.
# 10 deg keeps the 0.5-0.9 deg target and the measured J2 bias (~11 deg) both
# legible; see the autoscale note in CurvePanel.refresh().
_PLOT_HALF_SPAN_DEG = 10.0


class KalmanAngle1D:
    """Constant-velocity Kalman filter for one joint angle (radians).

    Smooths DREAM's OWN sequential estimates over time — it never sees the
    encoder value, so this is not "sticking the estimate to the encoders"
    (that was explicitly rejected earlier). It's a physically-motivated prior
    (the real arm can't teleport between frames) applied only to DREAM's own
    noisy measurements, plus an outlier gate that predict-only's through a
    measurement so far from the current estimate it's more likely a solver
    excursion (bad local minimum) than real motion — the same rejection
    philosophy as the two-pass px-space keypoint rejection, in angle-space.
    """

    # q_pos/q_vel stay at 3.0/15.0: a 0.3/1.5 variant was A/B'd on recorded
    # sequences (2026-07-16) and reverted. It wins on a STATIC arm (J5 jitter
    # 1.54->1.09 deg, MAE 1.49->1.09) but loses on a MOVING one, which is the
    # case that matters: tracking lag J5 0.1->1.0 s, J4 0.3->1.3 s, and MAE
    # J5 2.70->4.75 deg. At ~1.5 Hz there is no smoothing left to buy without
    # paying for it in lag.
    def __init__(self, r_deg: float, q_pos: float = np.radians(0.5) ** 2, q_vel: float = _KF_Q_VEL_DEG ** 2):
        self.x = None            # (2,) [angle_rad, velocity_rad_s]
        self.P = None            # (2,2)
        self.R = np.radians(r_deg) ** 2
        self.q_pos = q_pos       # process noise added to position variance per second
        self.q_vel = q_vel       # process noise added to velocity variance per second
        self._last_t = None

    def reset(self, angle_rad: float, t: float):
        self.x = np.array([angle_rad, 0.0])
        self.P = np.diag([self.R, np.radians(90.0) ** 2])
        self._last_t = t

    def step(self, measurement_rad, t: float) -> float:
        """Predict to time t, optionally update with measurement_rad (None = predict-only). Returns filtered angle (rad)."""
        if self.x is None:
            if measurement_rad is None:
                return None
            self.reset(measurement_rad, t)
            return self.x[0]

        dt = max(t - self._last_t, 1e-3)
        self._last_t = t
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = np.diag([self.q_pos * dt, self.q_vel * dt])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        if measurement_rad is not None:
            innovation = measurement_rad - self.x[0]
            S = self.P[0, 0] + self.R
            if abs(innovation) <= _KF_OUTLIER_GATE_SIGMA * np.sqrt(S):
                H = np.array([1.0, 0.0])
                K = (self.P @ H) / S  # (2,)
                self.x = self.x + K * innovation
                self.P = self.P - np.outer(K, H) @ self.P
            else:
                # Mesure rejetée : on tient la dernière position au lieu de
                # continuer sur la vitesse estimée. Sans ça le filtre part en
                # rampe pendant les rejets successifs (cause des excursions
                # a plusieurs centaines de degrés), alors que le bras est
                # immobile l'essentiel du temps.
                self.x[1] = 0.0
        else:
            self.x[1] = 0.0

        return self.x[0]


# Thresholds for WarmStartMonitor.should_restart — first-pass values, not
# fit to any specific dataset. Tunable if false-positive/negative restart
# rates turn out wrong in practice.
_RMS_SPIKE_MULT = 3.0        # reprojection error > this * rolling median...
_RMS_SPIKE_MIN_PX = 8.0      # ...or median + this many px, whichever is looser
_MAX_PLAUSIBLE_JUMP_DEG = 30.0   # single-step angle change larger than this = suspect
_LOW_CONF_MIN_KP = 5              # fewer valid keypoints than this counts as "low confidence"
_LOW_CONF_STREAK_LIMIT = 5        # consecutive low-confidence DETECTIONS (not GUI ticks)
_PASS_DISAGREEMENT_DEG = 15.0     # pass1 vs pass2 q disagreement larger than this = suspect


class WarmStartMonitor:
    """Decides when the joint+pose solver's warm start should be
    cold-restarted instead of trusted — using ONLY signals the solver itself
    produces (reprojection error, joint-limit proximity, frame-to-frame jump
    size, a sustained low-confidence-detection streak, and pass1/pass2
    agreement). Deliberately never looks at the encoder angles: comparing
    DREAM's own estimate against the encoders to decide whether to trust it
    would make the "independent validation" claim false (2026-07-15 KPI
    review — see dream_angle_solver.py commit history for the underlying
    gauge-ambiguity finding this exists to catch).

    A warm-started solve that has drifted into a persistent-but-wrong local
    minimum (e.g. the documented J1-rotation/camera-roll ambiguity) tends to
    look STABLE and LOCALLY CONSISTENT frame to frame — low reprojection
    error, no sudden jumps, once it's settled there. So `rms_spike` and
    `angle_jump` mainly catch the MOMENT the fit is jumping between local
    minima; `pass_disagreement` and `low_confidence_streak` are what catch a
    fit that's already quietly wrong but stable.
    """

    def __init__(self):
        self.rms_history = deque(maxlen=10)
        self.prev_q = None
        self.low_conf_streak = 0
        self._last_streak_kp_seq = None

    def should_restart(self, res, n_valid, kp_seq, first_q=None):
        """res: a solve_joint_angles_and_pose(_two_pass) result dict.
        kp_seq: DashboardNode._kp_seq at the time res was computed — used to
        advance the low-confidence streak once per actual DREAM detection,
        not once per ~30 Hz GUI tick that re-solves the same still-cached
        keypoints while waiting for the next one.

        Returns (should_restart: bool, reason: str — '+'-joined trigger names).
        """
        reasons = []

        if self.rms_history:
            baseline = float(np.median(self.rms_history))
            if res['rms_reproj_px'] > max(_RMS_SPIKE_MULT * baseline, baseline + _RMS_SPIKE_MIN_PX):
                reasons.append('rms_spike')

        if res['solver_failed']:
            reasons.append('joint_limit')

        if self.prev_q is not None:
            jump_deg = float(np.degrees(np.abs(res['q'] - self.prev_q)).max())
            if jump_deg > _MAX_PLAUSIBLE_JUMP_DEG:
                reasons.append('angle_jump')

        if kp_seq != self._last_streak_kp_seq:
            self._last_streak_kp_seq = kp_seq
            self.low_conf_streak = self.low_conf_streak + 1 if n_valid < _LOW_CONF_MIN_KP else 0
        if self.low_conf_streak >= _LOW_CONF_STREAK_LIMIT:
            reasons.append('low_confidence_streak')

        if first_q is not None:
            disagree_deg = float(np.degrees(np.abs(res['q'] - first_q)).max())
            if disagree_deg > _PASS_DISAGREEMENT_DEG:
                reasons.append('pass_disagreement')

        return (len(reasons) > 0, '+'.join(reasons))

    def accept(self, res):
        """Call once a solution (warm-started or cold-restarted) is accepted
        for the frame, so future spike/jump checks compare against it."""
        self.rms_history.append(res['rms_reproj_px'])
        self.prev_q = res['q'].copy()


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

        # ── KPI: end-to-end latency, image capture -> DREAM angle estimate ──
        # dream_inference_node now stamps /dream/keypoints with the SOURCE
        # image's own header.stamp (see its module docstring) instead of the
        # dashboard guessing from its own, unrelated camera subscription —
        # the old approach (comparing against _last_image_recv_time, the
        # dashboard's most-recently-arrived camera frame) silently measured
        # camera/DREAM staleness skew, not the latency of the actual frame
        # being solved. Three numbers, kept separate: inference (DREAM NN
        # forward pass, timed inside dream_inference_node), solver (the
        # least_squares angle+pose fit, timed locally below), and total
        # (image capture -> angles ready here).
        self.latest_kp_image_stamp = None   # seconds since epoch, source image capture time
        self.last_dream_inference_ms = None
        self.last_solver_ms = None
        self.last_pipeline_latency_ms = None
        self._kp_seq = 0

        # ── KPI: repeatability — stability of DREAM's OWN angle estimate
        # while the robot is idle. Must NOT be derived from the encoders
        # (they're ground truth, not a DREAM-quality signal) — see
        # KPI_dashboard_DREAM.md.
        self._dream_q_history = deque(maxlen=40)   # (t, q_dream[6] rad)

        # ── DREAM-only angle estimation: joint angles + camera pose solved
        # together, fresh every frame (no persisted or session-frozen
        # extrinsic — see module docstring). Warm-started from the previous
        # frame's own solution (q, rvec, tvec) for temporal continuity.
        self._prev_dream_q = None
        self._prev_dream_rvec = None
        self._prev_dream_tvec = None

        # ── Temporal filter (Kalman, per joint) on DREAM's own estimate ──
        # Toggle kept live (KPIPanel checkbox) like the robust-solver switch —
        # off falls back to the raw two-pass solve, on smooths it over time.
        self.use_temporal_filter = False
        self._kf_joints = [KalmanAngle1D(r_deg) for r_deg in _KF_MEASUREMENT_MEDIAN_DEG]

        # estimate_dream_angles() is called from both record_curve_sample()
        # and KPIPanel.refresh() every tick — cache per-tick so the solver
        # (and the Kalman filter's dt bookkeeping) runs exactly once per
        # tick, not twice. Invalidated at the top of DashboardWindow._tick().
        self._dream_q_cache = None
        self._dream_q_cache_valid = False

        # ── Solver mode: robust (2-pass outlier rejection) vs plain linear ──
        # Confirmed better on every metric against loss='huber' on the cached
        # real_3cam frames (2026-07-13, training/dream/compare_robust_loss.py)
        # — see robust_loss_comparison.csv / huber_fscale_sweep.csv. Toggle
        # kept live (KPIPanel checkbox) so linear stays available as an A/B
        # comparison, not silently removed. reg_weight and temporal filtering
        # are untouched by this switch.
        self.use_robust_solver = True
        self.last_dream_rejected_kp_idx = []   # keypoint indices dropped for pass 2, this solve
        self.last_dream_two_pass_applied = False

        # CONSISTENCY MODE (default ON since 2026-07-20, tutor-validated as the
        # pipeline's operating mode; no longer exposed as a dashboard toggle).
        # The solver is seeded from the
        # ENCODER angles every frame instead of DREAM's own previous estimate.
        # This intentionally BREAKS DREAM's independence: seeded on the encoder
        # branch, the fit can no longer land on a different monocular branch, so
        # the reported error becomes an intra-branch camera<->encoder CONSISTENCY
        # check (fault/drift detection), NOT the independent pose-recovery
        # validation the free solver provides. Experimentally reaches ~2.8 deg
        # MAE (vs the free solver's branch-limited tens of degrees) precisely
        # because the encoder resolves the branch the image alone cannot (see
        # scratchpad multiframe investigation, 2026-07-15).
        self.use_encoder_seed = True

        # ── Warm-start reset (cold-restart) — see WarmStartMonitor above.
        # _last_restart_kp_seq bounds cold-restart attempts to once per NEW
        # DREAM detection, not once per ~30 Hz GUI tick that re-solves the
        # same still-cached keypoints — a multi-candidate search is ~9x an
        # ordinary solve and would otherwise re-run every tick for as long
        # as the trigger condition holds.
        self._warm_start_monitor = WarmStartMonitor()
        self._last_restart_kp_seq = None
        self.last_cold_restart_triggered = False
        self.last_cold_restart_reason = None
        self.n_solves = 0
        self.n_cold_restarts = 0

        # ── Hard per-frame bound on the camera pose delta (see
        # solve_joint_angles_and_pose_bounded / MAX_CAMERA_ROT_RATE_DEG_S) —
        # pose_reg_weight kept as a live-adjustable parameter (KPIPanel
        # spinbox) per 2026-07-15 request, not just a fixed default baked
        # into the solver's signature.
        self.pose_reg_weight = 5.0
        self._prev_dream_pose_time = None
        self.last_pose_bound_hit = False
        self.last_pose_rupture_triggered = False
        self.n_pose_ruptures = 0

        # ── Joint curve history: encoder vs DREAM-estimated angle (deg) ──
        self.curve_t0 = time.time()
        self.curve_history = deque(maxlen=600)  # (t, q_encoder[6] deg, q_dream[6] or None)

        # ── Cumulative angular-error counter (MAE/RMS) for the WHOLE session ──
        # Unlike curve_history (bounded to 600 samples for the plots), this
        # never drops old data — a running sum, so the KPI panel's "MAE
        # totale" only ever accumulates more evidence, like an odometer, not
        # a windowed stat that can silently improve as bad old frames age out.
        self._angle_err_sum = np.zeros(6)
        self._angle_err_sq_sum = np.zeros(6)
        self._angle_err_count = 0

        # ── Acquisition CSV (mode manuel) ──
        self.acquisition_enabled = False
        self.session_index = 1
        self._acq_active = False
        self._acq_buffer = []
        self._acq_target = None
        self._acq_changed = list(range(6))
        self._acq_end_time = 0.0
        self._acq_mode = 'manuel'
        self._last_commanded_angles = [0.0] * 6

        # ── Mode automatique (poses 3cam) ──
        self.auto_status = 'inactif'

        self._init_subscriptions()

    def reset_session_stats(self):
        """Remet à zéro les compteurs cumulés depuis le lancement (MAE/RMS
        session, cold-restarts, pose-ruptures). Les courbes et la fenêtre 30s
        ne sont pas touchées — elles se régénèrent d'elles-mêmes."""
        self._angle_err_sum[:] = 0.0
        self._angle_err_sq_sum[:] = 0.0
        self._angle_err_count = 0
        self.n_solves = 0
        self.n_cold_restarts = 0
        self.n_pose_ruptures = 0

    # ────────────────────── Acquisition CSV (mode manuel) ──────────────────────

    def start_acquisition(self, target_angles_deg, changed=None, mode='manuel'):
        """Démarre une capture de ACQ_DURATION_S s. changed=None → joints changés
        vs dernière commande (mode manuel) ; sinon liste imposée (mode auto = les
        6). N'INTERFÈRE PAS avec l'envoi, déjà fait avant cet appel."""
        if changed is None:
            changed = [j for j in range(6)
                       if abs(target_angles_deg[j] - self._last_commanded_angles[j]) > 0.05]
        self._acq_changed = changed or list(range(6))
        self._acq_target = list(target_angles_deg)
        self._acq_mode = mode
        self._acq_buffer = []
        self._acq_active = True
        # Enregistre dès maintenant, pendant tout le mouvement puis la pose stable.
        self._acq_end_time = time.time() + ACQ_DURATION_S
        self._last_commanded_angles = list(target_angles_deg)

    def _write_joint_csv(self, out_dir, fname, joints):
        """Écrit un CSV avec, pour chaque joint de `joints`, ses colonnes
        enc/dream/err (t_s partagé)."""
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / fname
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s']
                       + [f'enc_J{j+1}' for j in joints]
                       + [f'dream_J{j+1}' for j in joints]
                       + [f'err_J{j+1}' for j in joints])
            errs = {j: [] for j in joints}
            for t, enc, dream, err in self._acq_buffer:
                row = [f'{t:.3f}'] + [f'{enc[j]:.3f}' for j in joints]
                if dream is not None:
                    row += [f'{dream[j]:.3f}' for j in joints] + [f'{err[j]:.3f}' for j in joints]
                    for j in joints:
                        errs[j].append(abs(err[j]))
                else:
                    row += [''] * (2 * len(joints))
                w.writerow(row)
            # Lignes de résumé : MAE et RMSE par joint (sous les colonnes err).
            blanks = [''] * (2 * len(joints))  # colonnes enc + dream
            mae = [f'{np.mean(errs[j]):.3f}' if errs[j] else '' for j in joints]
            rmse = [f'{np.sqrt(np.mean(np.square(errs[j]))):.3f}' if errs[j] else '' for j in joints]
            w.writerow(['MAE'] + blanks + mae)
            w.writerow(['RMSE'] + blanks + rmse)
        self.get_logger().info(f'Acquisition écrite : {path}')
        return path

    def _finish_acquisition(self):
        self._acq_active = False
        if not self._acq_buffer:
            return None
        date = datetime.now().strftime('%Y%m%d_%H%M%S')
        tgt = self._acq_target
        if self._acq_mode == 'auto':
            out_dir = ACQUISITION_DIR / 'auto'
            joints = '_'.join(f'joint{j+1}_angle{tgt[j]:.0f}' for j in range(6))
        else:
            out_dir = ACQUISITION_DIR / 'manuel'
            joints = '_'.join(f'joint{j+1}_angle_{tgt[j]:.0f}' for j in self._acq_changed)
        # Sépare les acquisitions selon l'état du filtre au moment de l'écriture :
        # colonne `dream` = valeur filtrée si Kalman coché, brute sinon. Le
        # sous-dossier kalman/ garde les deux séries comparables sans mélange.
        if self.use_temporal_filter:
            out_dir = out_dir / 'kalman'
        # UN fichier, TOUS les 6 joints (enc/dream/err), manuel comme auto.
        self._write_joint_csv(out_dir, f'session{self.session_index}_{joints}_{date}.csv',
                              list(range(6)))
        self.session_index += 1
        self._acq_buffer = []
        return None

    def tick_acquisition(self):
        """Clôt la capture quand la fenêtre de ACQ_DURATION_S s expire."""
        if self._acq_active and time.time() >= self._acq_end_time:
            self._finish_acquisition()

    def do_auto_pose(self, speed=30):
        """Mode automatique : UNE pose 3cam aléatoire (sûre) commandée sur le vrai
        robot. Si l'acquisition est cochée, capture les 6 joints → un seul CSV."""
        pose = random_joint_angles()
        self.send_angles(pose, speed)
        self.reset_kalman()
        if self.acquisition_enabled:
            self.start_acquisition(pose, changed=list(range(6)), mode='auto')
        else:
            self._last_commanded_angles = list(pose)
        self.auto_status = 'pose envoyée : ' + ', '.join(f'{a:.0f}' for a in pose)
        return pose

    def _init_subscriptions(self):
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

        # get_angles readback happens to piggy-back on joint_sync's own polling
        # (same /from_robot topic), but nothing ever asks for get_coords — the
        # position readback was frozen at [0,0,0,0,0,0] forever. Poll both
        # ourselves so the manual-pilot panel stays live regardless of what
        # other nodes are doing.
        self.create_timer(0.5, self._poll_robot_state)

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
        # Layout from dream_inference_node: 7*(u,v,valid) + [img_stamp_sec,
        # img_stamp_nanosec, inference_ms] — see that node's module
        # docstring. The trailing 3 carry the SOURCE image's own timestamp,
        # which is what makes a true image->angles latency measurement
        # possible (the dashboard's own camera subscription runs on an
        # unrelated schedule and says nothing about the image that actually
        # produced THESE keypoints).
        flat = msg.data
        n = 7
        kp = np.zeros((n, 2), dtype=np.float64)
        valid = np.zeros(n, dtype=bool)
        for i in range(n):
            kp[i] = [flat[3 * i], flat[3 * i + 1]]
            valid[i] = flat[3 * i + 2] > 0.5
        self.latest_kp_2d = kp
        self.latest_kp_valid = valid
        img_stamp_sec, img_stamp_nanosec, inference_ms = flat[21:24]
        self.latest_kp_image_stamp = img_stamp_sec + img_stamp_nanosec * 1e-9
        self.last_dream_inference_ms = inference_ms
        self._kp_seq += 1
        self.kp_count += 1
        now = time.time()
        self._kp_recv_times.append(now)
        self._kp_recv_times = [t for t in self._kp_recv_times if t > now - 5.0]

    def _status_cb(self, msg: String):
        self.latest_status = msg.data

    def _feedback_cb(self, msg: String):
        self.last_feedback = msg.data
        # bridge_tour can coalesce several TCP replies into one message (no
        # line-framing on its recv()); bridge_pi_simple.py replies "ANGLES: [...]"
        # / "COORDS: [...]" (uppercase, space after ':') — same fix as joint_sync.py.
        lines = [l.strip() for l in msg.data.splitlines() if l.strip()]
        data = lines[-1] if lines else msg.data.strip()

        if data.upper().startswith('ANGLES:'):
            try:
                self.current_angles_deg = eval(data.split(':', 1)[1].strip())
            except Exception:
                pass
        elif data.upper().startswith('COORDS:'):
            try:
                self.current_coords = eval(data.split(':', 1)[1].strip())
            except Exception:
                pass

    # ── Manual pilot commands (same wire protocol as simple_gui.py) ──
    def reset_kalman(self):
        """Vide l'état des filtres : la prochaine mesure DREAM devient la
        nouvelle base. Appelé quand l'utilisateur commande une pose (SET
        Angles) — on SAIT que le grand mouvement qui suit est réel, donc le
        portail anti-aberration ne doit pas le rejeter comme une excursion."""
        for kf in self._kf_joints:
            kf.x = None
            kf._last_t = None

    def send_angles(self, angles, speed=50):
        self._publish_cmd({'action': 'send_angles', 'angles': angles, 'speed': speed})

    def send_coords(self, coords, speed=50, mode=0):
        self._publish_cmd({'action': 'send_coords', 'coords': coords, 'speed': speed, 'mode': mode})

    def get_angles(self):
        self._publish_cmd({'action': 'get_angles'})

    def get_coords(self):
        self._publish_cmd({'action': 'get_coords'})

    def _poll_robot_state(self):
        self.get_angles()
        self.get_coords()

    def power_on(self):
        """Verrouille les servos (position tenue) — mc.power_on()."""
        self._publish_cmd({'action': 'power_on'})

    def power_off(self):
        """Relâche les servos (bras déplaçable à la main) — mc.release_all_servos()."""
        self._publish_cmd({'action': 'power_off'})

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

        # Encoder skeleton drawn with a pose fit fresh THIS frame from the
        # known encoder angles (well-posed 6-DoF PnP) — no persisted or
        # session-frozen extrinsic (see module docstring). Refitting every
        # frame means a bumped camera degrades this frame's overlay instead
        # of silently poisoning every subsequent frame the way a frozen
        # anchor would.
        big_px, _ = cv2.projectPoints(pts_3d, rvec, tvec, self.camera_K, self.camera_dist)
        big_px = big_px.reshape(-1, 2)

        err_px = np.full(len(KEYPOINT_NAMES), np.nan)
        err_px[idx] = np.linalg.norm(big_px[idx] - self.latest_kp_2d[idx], axis=1)
        rms_px = float(np.sqrt(np.nanmean(err_px[idx] ** 2)))

        return {
            'big_px': big_px, 'small_px': self.latest_kp_2d, 'valid': valid,
            'err_px': err_px, 'rms_px': rms_px, 'n_valid': int(valid.sum()),
        }

    def estimate_dream_angles(self):
        """DREAM-only joint angle estimate (rad): 6 joint angles AND the
        6-DoF camera pose solved jointly, fresh every frame — no persisted
        or session-frozen extrinsic (see module docstring). Warm-starts (and
        softly regularizes the angles towards) the PREVIOUS frame's own
        DREAM solution — (q, rvec, tvec) — rather than the live encoder or a
        fixed anchor: this is what makes the DREAM curve an independent
        trace. Falls back to the encoder angles (pose still free) only for
        the very first solve of the session, before any DREAM estimate
        exists yet.

        The warm start is gauge-ambiguous in general (see
        solve_joint_angles_and_pose's docstring — J1 vs camera roll is the
        main documented case) and can settle into a persistent-but-wrong
        local minimum that then re-seeds itself every subsequent frame via
        reg_weight/pose_reg_weight. WarmStartMonitor watches internal-only
        signals (reprojection spike, joint-limit proximity, frame jump,
        sustained low-confidence detections, pass1/pass2 disagreement — see
        its docstring) and, when triggered, runs a cold-restart search
        (solve_joint_angles_and_pose_multi_init: several q0 seeds, none
        derived from the previous frame OR the encoders) and keeps whichever
        solution — warm-started or cold-restarted — has the lower
        reprojection error. Never compares to the encoder: that's what keeps
        this an independent estimate rather than one that quietly tracks
        the ground truth it's supposed to be validated against.

        None if fewer than 4 DREAM keypoints are visible.

        Uses solve_joint_angles_and_pose_two_pass by default
        (self.use_robust_solver) — confirmed better than plain linear on
        reprojection, joint-limit-hit rate, drift, and frames solved
        (2026-07-13 comparison, see training/dream/compare_robust_loss.py).
        The plain solver stays available as a live A/B toggle (KPIPanel
        checkbox), not removed.

        Cached per tick (see _dream_q_cache_valid) since both
        record_curve_sample() and KPIPanel.refresh() call this every tick —
        without caching, the solver (and the Kalman filter's dt bookkeeping
        below) would run twice per tick on near-identical input.

        When self.use_temporal_filter, the raw per-frame solve is passed
        through a per-joint Kalman filter (KalmanAngle1D) before being
        returned/cached — smooths DREAM's OWN trajectory over time using only
        DREAM's own past estimates (never the encoder), so this is not
        "sticking the estimate to the encoders."
        """
        if self._dream_q_cache_valid:
            return self._dream_q_cache

        def _finish(q):
            self._dream_q_cache = q
            self._dream_q_cache_valid = True
            return q

        if self.latest_kp_2d is None or self.latest_kp_valid is None:
            return _finish(None)
        n_valid = int(self.latest_kp_valid.sum())
        if n_valid < 4:
            return _finish(None)

        if self.use_encoder_seed and self.latest_joint_q is not None:
            # Consistency mode: re-anchor on the encoder branch every frame (see
            # use_encoder_seed field comment). No longer an independent estimate.
            # Strong distal prior (see _CONSISTENCY_REG_VEC) holds the poorly-
            # observed J3-J5 near the seeded branch instead of drifting on their
            # near-flat pixel cost.
            q_init = self.latest_joint_q
            reg_weight = np.array(_CONSISTENCY_REG_VEC)
        else:
            q_init = self._prev_dream_q if self._prev_dream_q is not None else self.latest_joint_q
            reg_weight = 1.5

        # Hard per-frame camera-pose-delta bound (see
        # solve_joint_angles_and_pose_bounded) — rate x dt, dt = time since
        # the last ACCEPTED pose. Skipped (None) if there's no previous pose
        # yet, or the gap is too large for a single-frame rate bound to mean
        # anything (see POSE_BOUND_MAX_DT_S).
        t_solve_start = time.time()
        max_drot_rad = max_dtrans_m = None
        if self._prev_dream_pose_time is not None:
            dt = t_solve_start - self._prev_dream_pose_time
            if 0.0 < dt <= POSE_BOUND_MAX_DT_S:
                max_drot_rad = np.radians(MAX_CAMERA_ROT_RATE_DEG_S) * dt
                max_dtrans_m = MAX_CAMERA_TRANS_RATE_M_S * dt

        res = solve_joint_angles_and_pose_bounded(
            self.latest_kp_2d, self.latest_kp_valid, self.camera_K,
            q_init=q_init, rvec_init=self._prev_dream_rvec, tvec_init=self._prev_dream_tvec,
            reg_weight=reg_weight, pose_reg_weight=self.pose_reg_weight,
            max_drot_rad=max_drot_rad, max_dtrans_m=max_dtrans_m,
            use_two_pass=self.use_robust_solver,
        )

        self.last_cold_restart_triggered = False
        self.last_cold_restart_reason = None
        self.last_pose_bound_hit = res['pose_bound_hit']
        self.last_pose_rupture_triggered = res['pose_rupture']
        self.n_solves += 1
        if res['pose_rupture']:
            self.n_pose_ruptures += 1

        need_restart, reason = self._warm_start_monitor.should_restart(
            res, n_valid=n_valid, kp_seq=self._kp_seq, first_q=res.get('pass1_q'))

        # Cold-restart exists to ESCAPE the current branch — the opposite of what
        # consistency mode wants (it deliberately holds the encoder branch), so
        # skip it entirely when encoder-seeding.
        if self.use_encoder_seed:
            need_restart = False

        if need_restart and self._kp_seq != self._last_restart_kp_seq:
            # Bound to once per NEW DREAM detection (see field comment in
            # __init__) — a 9-candidate search is far more expensive than a
            # single warm-started solve.
            self._last_restart_kp_seq = self._kp_seq
            candidate_res = solve_joint_angles_and_pose_multi_init(
                self.latest_kp_2d, self.latest_kp_valid, self.camera_K,
                use_two_pass=self.use_robust_solver,
            )
            current_rms = res['rms_reproj_px'] if not res['solver_failed'] else float('inf')
            if candidate_res is not None and candidate_res['rms_reproj_px'] < current_rms:
                res = candidate_res
                res.setdefault('pose_bound_hit', False)
                res.setdefault('pose_rupture', False)
                self.last_cold_restart_triggered = True
                self.last_cold_restart_reason = reason
                self.n_cold_restarts += 1

        t_solve_end = time.time()
        self.last_solver_ms = (t_solve_end - t_solve_start) * 1000.0

        # Pas de rejet sur joint_limit_hit : une estimation plaquée contre une
        # butée reste exploitable, et la jeter ouvrait des trous dans les
        # courbes. Les valeurs absurdes observées (J1 à -900°) ne venaient pas
        # du solveur — borné à JOINT_LOWER/UPPER — mais du Kalman extrapolant
        # sans borne ; c'est corrigé à la sortie du filtre plus bas.

        # Latency KPI: SOURCE image capture -> angles ready, using the
        # timestamp dream_inference_node stamped onto /dream/keypoints (the
        # image that actually produced the keypoints being solved here) —
        # not the dashboard's own, unrelated camera subscription.
        if self.latest_kp_image_stamp is not None:
            self.last_pipeline_latency_ms = (t_solve_end - self.latest_kp_image_stamp) * 1000.0

        self.last_dream_rejected_kp_idx = res.get('rejected_kp_idx', [])
        self.last_dream_two_pass_applied = res.get('two_pass_applied', False)

        self._warm_start_monitor.accept(res)
        self._prev_dream_q = res['q']
        self._prev_dream_rvec = res['rvec']
        self._prev_dream_tvec = res['tvec']
        self._prev_dream_pose_time = t_solve_end

        if not self.use_temporal_filter:
            q_out = res['q']
        else:
            # Borné aux butées : l'état du filtre inclut une vitesse et la
            # prédiction l'intègre sans limite, donc une série de mesures
            # rejetées par la porte anti-outlier faisait diverger la sortie en
            # rampe (J1 à -900°) alors que le solveur, lui, reste borné.
            q_out = np.clip(np.array([
                kf.step(res['q'][j], t_solve_end) for j, kf in enumerate(self._kf_joints)
            ]), JOINT_LOWER, JOINT_UPPER)

        self._dream_q_history.append((t_solve_end, q_out))
        return _finish(q_out)

    def joint_confidence(self, overlay):
        """Per-joint reliability flag (True = trustworthy) for the CURRENT frame.

        J6 is always False — no keypoint depends on it, so no live signal can
        ever vouch for it (see JOINT_OBSERVING_KP). J1-J5 are flagged False
        only when one of the keypoints that actually informs them is missing
        or has a large live reprojection error right now — this is a live
        measurement, not a fixed label, so a joint reads as reliable whenever
        its supporting keypoints genuinely are, not just "usually".
        """
        if overlay is None:
            return [False] * 6
        err_px = overlay['err_px']
        valid = overlay['valid']
        flags = []
        for m in range(6):
            observers = JOINT_OBSERVING_KP[m]
            if not observers:
                flags.append(False)
                continue
            ok = all(
                valid[j] and err_px[j] <= JOINT_CONFIDENCE_PX_THRESHOLD
                for j in observers
            )
            flags.append(ok)
        return flags

    def repeatability_deg(self, window_s: float = 2.0):
        """Std-dev (deg) of DREAM's OWN angle estimate over the last
        window_s seconds — how much the DREAM curve wiggles on its own,
        independent of any encoder comparison. Must NOT be derived from the
        encoders (they're ground truth, not a DREAM-quality signal) — see
        KPI_dashboard_DREAM.md. Returns None (displayed as "N/A", not a
        misleading 0.00°) if too few DREAM-valid samples have landed in the
        window yet.
        """
        if len(self._dream_q_history) < 3:
            return None
        now = time.time()
        recent = [q for (t, q) in self._dream_q_history if now - t <= window_s]
        if len(recent) < 3:
            return None
        arr = np.degrees(np.array(recent))
        return arr.std(axis=0)  # (6,) deg

    def record_curve_sample(self, overlay=None):
        """Append one (t, encoder_deg, dream_deg) sample for the joint-curve
        plots, and accumulate it into the session-wide MAE/RMS counter.
        dream_deg is None quand le solveur n'a rien produit (jamais détecté) —
        ces échantillons deviennent des trous dans les courbes."""
        if self.latest_joint_q is None:
            return
        q_dream = self.estimate_dream_angles()
        enc_deg = np.degrees(self.latest_joint_q)
        dream_deg = np.degrees(q_dream) if q_dream is not None else None
        self.curve_history.append((time.time() - self.curve_t0, enc_deg, dream_deg))

        if dream_deg is not None:
            err = np.abs(dream_deg - enc_deg)  # (6,) deg
            self._angle_err_sum += err
            self._angle_err_sq_sum += err ** 2
            self._angle_err_count += 1

        if self._acq_active and dream_deg is not None:
            t = time.time() - self.curve_t0
            self._acq_buffer.append((t, enc_deg, dream_deg, np.abs(dream_deg - enc_deg)))

    def angle_error_stats(self):
        """Cumulative ("session") per-joint MAE and RMS (deg) of
        |DREAM - encodeur|, over EVERY DREAM solve since the session started
        (a running counter, never drops old samples — see _angle_err_sum).
        This is a REAL measured statistic over time, distinct from the
        ~0.8° figure in training/dream/README.md, which is derived from the
        synthetic keypoint-detection validation, not from live
        DREAM-vs-encoder angle comparisons.

        Deliberately NOT reset by a warm-start cold-restart (WarmStartMonitor)
        or by toggling the solver mode — it's a running odometer, by design.
        That means it can stay dominated by a bad stretch early in the
        session (e.g. before a cold-restart escaped a stuck local minimum)
        long after the live tracking has recovered — see
        angle_error_stats_window() for the number that agrees with what the
        joint-curve graphs currently show.

        Returns None if fewer than 5 DREAM-valid samples recorded yet.
        """
        n = self._angle_err_count
        if n < 5:
            return None
        mae = self._angle_err_sum / n              # (6,) deg, per joint
        rms = np.sqrt(self._angle_err_sq_sum / n)   # (6,) deg, per joint
        return {
            'n_samples': n,
            'mae': mae,
            'rms': rms,
            'mae_total': float(mae.mean()),
            'rms_total': float(np.sqrt((rms ** 2).mean())),
        }

    def angle_error_stats_window(self, window_s=CURVE_WINDOW_S):
        """Per-joint MAE and RMS (deg) of |DREAM - encodeur|, over the SAME
        recent window and the SAME samples (curve_history) that
        JointCurvesPanel actually plots — filtered identically (last
        window_s seconds relative to the most recent sample, DREAM-valid
        samples only). Exists because angle_error_stats() is a session-wide
        running counter that can disagree sharply with what the graphs show
        (2026-07-15 KPI review): if the graphs currently look well-tracked
        but "MAE totale" still reads high, this is the number that should
        agree with the graphs — a high session MAE alongside a low window
        MAE means the error is historical (e.g. from before a cold-restart
        recovery), not current.

        Returns None if fewer than 5 DREAM-valid samples fall in the window.
        """
        hist = self.curve_history
        if not hist:
            return None
        t = np.array([h[0] for h in hist])
        cutoff = t[-1] - window_s
        keep = t >= cutoff
        enc = np.array([h[1] for h in hist])[keep]           # (N, 6)
        dream_raw = [h[2] for h, k in zip(hist, keep) if k]

        pairs = [(e, d) for e, d in zip(enc, dream_raw) if d is not None]
        if len(pairs) < 5:
            return None
        enc_arr = np.array([p[0] for p in pairs])   # (n, 6)
        dream_arr = np.array([p[1] for p in pairs])  # (n, 6)
        err = np.abs(dream_arr - enc_arr)            # (n, 6)

        mae = err.mean(axis=0)               # (6,) deg, per joint
        rms = np.sqrt((err ** 2).mean(axis=0))  # (6,) deg, per joint
        return {
            'n_samples': len(pairs),
            'mae': mae,
            'rms': rms,
            'mae_total': float(mae.mean()),
            'rms_total': float(np.sqrt((rms ** 2).mean())),
        }


class Tab1LiveValidation(QWidget):
    """Camera feed + big(encoder)/small(DREAM) circle overlay + per-joint error panel."""

    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        self.last_overlay = None
        layout = QVBoxLayout(self)

        cam_title = QLabel('Vue caméra')
        cam_title.setFont(QFont('Sans', 13, QFont.Bold))
        layout.addWidget(cam_title)

        self.image_label = QLabel('En attente d\'image caméra…')
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(320, 240)  # floor only — actual display size
        # is whatever layout space is available; 640x480 forced the whole
        # window's minimum size past some screens, causing it to overflow
        # instead of shrinking to fit (see DashboardWindow.__init__).
        self.image_label.setStyleSheet('background-color: #111; color: #999;')
        layout.addWidget(self.image_label)

        # Sous l'image, tassés : MAE/RMSE angle, RMS reprojection, puis le tableau
        # keypoints. Le format image (640×480) est dessiné dans l'image (HUD).
        stats_row = QHBoxLayout()
        self.mae_label = QLabel(f'MAE {CURVE_WINDOW_S:.0f}s (J1–J6) : — °')
        self.mae_label.setFont(QFont('Sans', 10, QFont.Bold))
        self.rmse_label = QLabel('RMSE (J1–J6) : — °')
        stats_row.addWidget(self.mae_label)
        stats_row.addWidget(self.rmse_label)
        stats_row.addStretch()
        layout.addLayout(stats_row)
        self.reproj_label = QLabel('RMS reprojection : — px')
        layout.addWidget(self.reproj_label)

        self.kp_grid = QGridLayout()
        self.kp_grid.setVerticalSpacing(8)
        self.kp_grid.setHorizontalSpacing(24)
        layout.addLayout(self.kp_grid)
        cell_font = QFont('Sans', 13)
        header_font = QFont('Sans', 13, QFont.Bold)
        header_kp = QLabel('Keypoint')
        header_kp.setFont(header_font)
        header_err = QLabel('Erreur (px)')
        header_err.setFont(header_font)
        self.kp_grid.addWidget(header_kp, 0, 0)
        self.kp_grid.addWidget(header_err, 0, 1, alignment=Qt.AlignRight)
        self.err_value_labels = []
        for i, label in enumerate(KP_LABELS):
            name_lbl = QLabel(label)
            name_lbl.setFont(cell_font)
            self.kp_grid.addWidget(name_lbl, i + 1, 0)
            val = QLabel('—')
            val.setFont(cell_font)
            self.kp_grid.addWidget(val, i + 1, 1, alignment=Qt.AlignRight)
            self.err_value_labels.append(val)
        self.kp_grid.setColumnStretch(2, 1)
        layout.addStretch(1)  # pousse tout le vide en bas → labels + tableau tassés

    @staticmethod
    def _hline():
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def refresh(self):
        n = self.node
        wstats = n.angle_error_stats_window()
        if wstats is not None:
            self.mae_label.setText(f'MAE {CURVE_WINDOW_S:.0f}s (J1–J6) : {wstats["mae_total"]:.2f}°')
            self.rmse_label.setText(f'RMSE (J1–J6) : {wstats["rms_total"]:.2f}°')
        else:
            self.mae_label.setText(f'MAE {CURVE_WINDOW_S:.0f}s (J1–J6) : — °')
            self.rmse_label.setText('RMSE (J1–J6) : — °')

        if n.latest_bgr is None:
            self.reproj_label.setText('RMS reprojection : — px')
            self.last_overlay = None
            return
        frame = n.latest_bgr.copy()
        overlay = n.compute_overlay()
        self.last_overlay = overlay
        n.record_curve_sample()

        if overlay is not None:
            self.reproj_label.setText(f'RMS reprojection : {overlay["rms_px"]:.2f} px '
                                      f'({overlay["n_valid"]}/7 kp)')
        else:
            self.reproj_label.setText('RMS reprojection : — px (données insuffisantes)')

        if overlay is not None:
            # Squelette (base -> link1 -> ... -> link6), comme dessiné en réunion :
            # relier les joints consécutifs, pas seulement des cercles isolés.
            big_pts = overlay['big_px']
            for i in range(len(KEYPOINT_NAMES) - 1):
                p1 = (int(round(big_pts[i][0])), int(round(big_pts[i][1])))
                p2 = (int(round(big_pts[i + 1][0])), int(round(big_pts[i + 1][1])))
                cv2.line(frame, p1, p2, BIG_COLOR_BGR, 2)
            small_pts = overlay['small_px']
            valid = overlay['valid']
            for i in range(len(KEYPOINT_NAMES) - 1):
                if valid[i] and valid[i + 1]:
                    p1 = (int(round(small_pts[i][0])), int(round(small_pts[i][1])))
                    p2 = (int(round(small_pts[i + 1][0])), int(round(small_pts[i + 1][1])))
                    cv2.line(frame, p1, p2, SMALL_COLOR_BGR, 2)

            for i in range(len(KEYPOINT_NAMES)):
                bx, by = overlay['big_px'][i]
                cv2.circle(frame, (int(round(bx)), int(round(by))), 6, BIG_COLOR_BGR, 1)
                if overlay['valid'][i]:
                    sx, sy = overlay['small_px'][i]
                    cv2.circle(frame, (int(round(sx)), int(round(sy))), 2, SMALL_COLOR_BGR, -1)
                    self.err_value_labels[i].setText(f'{overlay["err_px"][i]:.1f}')
                else:
                    self.err_value_labels[i].setText('non détecté')
        else:
            for lbl in self.err_value_labels:
                lbl.setText('—')

        if n.last_pose_rupture_triggered:
            pose_bgr = (32, 0, 176)       # rouge — rupture
        elif n.last_pose_bound_hit:
            pose_bgr = (0, 128, 192)      # ambre — pose bornée
        else:
            pose_bgr = (0, 150, 0)        # vert — stable
        self._draw_hud(frame, n.fps(), n.dream_rate_hz(), pose_bgr)
        self._display_frame(frame)

    @staticmethod
    def _draw_hud(frame, fps, dream_hz, pose_bgr):
        """Cadences burnt into the frame itself — they describe the image being
        shown, so they belong on it rather than in a side column that can drift
        a tick out of sync with the pixels."""
        for i, text in enumerate((f'Camera : {fps:.1f} FPS', f'DREAM : {dream_hz:.1f} Hz')):
            org = (10, 24 + i * 24)
            cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        # Format de l'image, en bas à gauche.
        h, w = frame.shape[:2]
        fmt = f'Image {w}x{h} RGB'
        forg = (10, h - 12)
        cv2.putText(frame, fmt, forg, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, fmt, forg, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        # Pastille de santé de la pose caméra, en haut à droite de l'image.
        label = 'Pose camera'
        (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        dot = (w - tw - 22, 20)
        cv2.circle(frame, dot, 7, (0, 0, 0), -1)
        cv2.circle(frame, dot, 6, pose_bgr, -1)
        org = (w - tw - 10, 25)
        cv2.putText(frame, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    def _display_frame(self, frame_bgr: np.ndarray):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        # Scale sur la largeur du label et cale sa hauteur pile sur l'image :
        # ratio 4:3 respecté, aucune bande noire, aucun rognage.
        pix = QPixmap.fromImage(qimg).scaledToWidth(
            self.image_label.width(), Qt.SmoothTransformation)
        self.image_label.setPixmap(pix)
        self.image_label.setFixedHeight(pix.height())


class ManualPilotPanel(QWidget):
    """Angle/position control (simple_gui.py protocol) + live encoder readback."""

    COORD_LABELS = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']

    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        outer = QVBoxLayout(self)

        box = QGroupBox('Mode manuel')
        grid = QGridLayout(box)
        outer.addWidget(box)

        # Spinboxes rather than plain line edits: each field can be nudged by a
        # step with the arrows or typed into directly, and the range clamps a
        # typo before it reaches the robot.
        grid.addWidget(QLabel('Angles (°) — FK'), 0, 0, 1, 6)
        self.angle_edits = []
        for i in range(6):
            grid.addWidget(QLabel(f'J{i+1}'), 1, i)
            sb = self._spinbox(-180.0, 180.0, 1.0)
            grid.addWidget(sb, 2, i)
            self.angle_edits.append(sb)
        btn_angles = QPushButton('SET Angles')
        btn_angles.clicked.connect(self._on_set_angles)
        grid.addWidget(btn_angles, 2, 6)

        grid.addWidget(self._hline(), 3, 0, 1, 7)

        grid.addWidget(QLabel('Coordonnées — IK'), 4, 0, 1, 6)
        self.coord_edits = []
        for i, label in enumerate(self.COORD_LABELS):
            grid.addWidget(QLabel(label), 5, i)
            # X/Y/Z are mm, RX/RY/RZ are degrees — different ranges and steps.
            sb = self._spinbox(-1000.0, 1000.0, 5.0) if i < 3 else self._spinbox(-180.0, 180.0, 1.0)
            grid.addWidget(sb, 6, i)
            self.coord_edits.append(sb)
        btn_coords = QPushButton('SET Coords')
        btn_coords.clicked.connect(self._on_set_coords)
        grid.addWidget(btn_coords, 6, 6)

        grid.addWidget(QLabel('Position actuelle'), 7, 0, 1, 6)
        self.coord_readback = []
        for i in range(6):
            lbl = QLabel('0.0')
            lbl.setStyleSheet('background:#fff; border:1px solid #ccc;')
            grid.addWidget(lbl, 8, i)
            self.coord_readback.append(lbl)

        grid.addWidget(QLabel('Vitesse'), 9, 0)
        self.speed_edit = QSpinBox()
        self.speed_edit.setRange(1, 100)
        self.speed_edit.setValue(30)
        self.speed_edit.setFixedWidth(70)
        grid.addWidget(self.speed_edit, 9, 1)

        grid.addWidget(self._hline(), 10, 0, 1, 7)

        btn_lock = QPushButton('🔒 Fixer (power_on)')
        btn_lock.setStyleSheet('background:#c8f0c8;')
        btn_lock.clicked.connect(self._on_power_on)
        grid.addWidget(btn_lock, 11, 0, 1, 3)

        btn_release = QPushButton('🔓 Relâcher (power_off)')
        btn_release.setStyleSheet('background:#f0d8c8;')
        btn_release.clicked.connect(self._on_power_off)
        grid.addWidget(btn_release, 11, 3, 1, 4)

        self.servo_state_label = QLabel(
            'État servos : inconnu — cliquer Fixer avant SET Angles/Coords')
        grid.addWidget(self.servo_state_label, 12, 0, 1, 7)

        self.acq_checkbox = QCheckBox('Enregistrer les données (CSV) à chaque SET Angles')
        self.acq_checkbox.stateChanged.connect(
            lambda s: setattr(self.node, 'acquisition_enabled', bool(s)))
        grid.addWidget(self.acq_checkbox, 13, 0, 1, 7)
        acq_path = QLabel(f'→ {ACQUISITION_DIR}')
        acq_path.setStyleSheet('color:#666;')
        acq_path.setWordWrap(True)
        grid.addWidget(acq_path, 14, 0, 1, 7)

    @staticmethod
    def _spinbox(lo, hi, step):
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setSingleStep(step)
        sb.setDecimals(1)
        sb.setValue(0.0)
        sb.setFixedWidth(78)
        return sb

    @staticmethod
    def _hline():
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def _speed(self) -> int:
        return self.speed_edit.value()

    def _on_set_angles(self):
        angles = [e.value() for e in self.angle_edits]
        # 1) Envoi de la commande EN PREMIER, inconditionnel — jamais bloqué.
        self.node.send_angles(angles, self._speed())
        # Reset du Kalman : le mouvement commandé est réel, le filtre ne doit
        # pas le geler comme une aberration (cf. J2 -45° rejeté portail).
        self.node.reset_kalman()
        # 2) Puis, seulement si demandé, on lance la capture CSV (sans toucher
        #    au chemin d'envoi).
        if self.node.acquisition_enabled:
            self.node.start_acquisition(angles)

    def _on_set_coords(self):
        self.node.send_coords([e.value() for e in self.coord_edits], self._speed())
        self.node.reset_kalman()

    def _on_power_on(self):
        self.node.power_on()
        self.servo_state_label.setText('État servos : 🔒 fixé (power_on envoyé)')

    def _on_power_off(self):
        self.node.power_off()
        self.servo_state_label.setText(
            'État servos : 🔓 relâché (power_off envoyé) — déplaçable à la main')

    def refresh(self):
        n = self.node
        # No encoder-angle readback here — the joint curves plot the same
        # encoder trace live, so a second numeric copy was redundant.
        for i, lbl in enumerate(self.coord_readback):
            if i < len(n.current_coords):
                lbl.setText(f'{n.current_coords[i]:.1f}')


class AutoModePanel(QWidget):
    """Mode automatique : UNE pose 3cam aléatoire sûre par clic, commandée sur le
    VRAI robot. Si 'Enregistrer CSV' (panneau manuel) est coché, capture les 6
    joints dans un seul fichier. ⚠ commande le robot physique."""

    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        outer = QVBoxLayout(self)
        box = QGroupBox('Mode automatique')
        v = QVBoxLayout(box)
        outer.addWidget(box)

        info = QLabel('Une pose aléatoire sûre (poses 3cam) par clic. ⚠ commande '
                      'le vrai robot. Coche « Enregistrer CSV » ci-dessus pour '
                      'sauvegarder les 6 joints dans un fichier.')
        info.setWordWrap(True)
        info.setStyleSheet('color:#a05000;')
        v.addWidget(info)

        self.btn_pose = QPushButton('▶ Pose automatique')
        self.btn_pose.setStyleSheet('background:#c8f0c8;')
        self.btn_pose.clicked.connect(self._on_pose)
        v.addWidget(self.btn_pose)

        self.status_label = QLabel('Séquence : inactif')
        self.status_label.setWordWrap(True)
        v.addWidget(self.status_label)

    def _on_pose(self):
        confirm = QMessageBox.question(
            self, 'Mode automatique',
            'Le VRAI robot va aller à une pose aléatoire.\n'
            'Bras dégagé et surveillé ?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if confirm == QMessageBox.Yes:
            self.node.do_auto_pose()

    def refresh(self):
        self.status_label.setText(f'Séquence : {self.node.auto_status}')


class KPIPanel(QWidget):
    """Précision (reprojection), répétabilité (jitter de l'estimation DREAM), temps de réponse.

    Three sections (Précision / Temps réel / Angles) instead of one long flat
    block — the long explanatory paragraphs (fiabilité par joint, J5) are
    collapsed to one line each with a "Détails" button popping the full text,
    so the panel stays scannable at a glance instead of requiring the reader
    to parse a wall of orange text.
    """

    _RELIABILITY_DETAILS = (
        '⚠ sur un joint = keypoints porteurs bruités/absents CE frame (J1-J5) '
        'ou non-observable en permanence (J6, aucun keypoint ne dépend de sa '
        'rotation).\n\n'
        '🔬 J5 : observabilité structurellement faible (1 seul keypoint '
        'dépendant, link6) — indépendant de la qualité de détection. Un test '
        'de sensibilité (training/dream/j5_observability_test.py, 2026-07-13) '
        'mesure une bande d\'ambiguïté de ~15-23° selon l\'angle caméra où '
        'plusieurs valeurs de J5 reprojettent de façon indiscernable, même '
        'avec une détection parfaite. Pistes : 2nde caméra ou prior '
        'géométrique — ce n\'est pas un réglage de reg_weight.'
    )

    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        self.setFont(QFont('Sans', 10))
        outer = QVBoxLayout(self)

        # ── Angles ──
        box_angles = QGroupBox('Angles : encodeur vs DREAM')
        form_a = QVBoxLayout(box_angles)
        self.solver_status_label = QLabel('Solveur angles DREAM : —')
        form_a.addWidget(self.solver_status_label)
        # Warm-start cold-restart visibility — see WarmStartMonitor.
        self.cold_restart_label = QLabel('Cold-restart : —')
        form_a.addWidget(self.cold_restart_label)
        self.restart_rate_label = QLabel('Cold-restarts : — / Pose-ruptures : —')
        form_a.addWidget(self.restart_rate_label)

        # The only solver choice left on the dashboard. Kalman filters DREAM's
        # OWN trajectory (never touches the encoder) to damp frame-to-frame
        # jitter on the live curves. Robust 2-pass rejection, pose_reg_weight
        # and consistency mode are all still active in the pipeline — they were
        # removed from the UI once their values were settled, not disabled.
        self.temporal_filter_checkbox = QCheckBox('Filtrage temporel (Kalman)')
        self.temporal_filter_checkbox.setChecked(node.use_temporal_filter)
        self.temporal_filter_checkbox.stateChanged.connect(self._on_temporal_filter_toggle)
        form_a.addWidget(self.temporal_filter_checkbox)

        # MAE/RMSE ne vivent QUE dans le tableau keypoints (Tab1LiveValidation).
        # Le panneau KPI ne garde que le détail par joint (popup), pas de
        # doublon du chiffre agrégé.
        btn_mae_details = QPushButton('MAE / RMSE — détails par joint')
        btn_mae_details.clicked.connect(self._show_angle_stats_details)
        form_a.addWidget(btn_mae_details)
        form_a.addWidget(self._hline())

        # Le tableau encodeur/DREAM/erreur par joint a été remplacé par les
        # trois valeurs affichées sous chaque courbe (JointCurvesPanel).
        details_row = QHBoxLayout()
        reliability_note = QLabel('⚠ fiabilité par joint · 🔬 J5 structurellement faible')
        reliability_note.setStyleSheet('color: #a06000; font-style: italic;')
        details_row.addWidget(reliability_note)
        details_row.addStretch()
        btn_details = QPushButton('Détails')
        btn_details.setFixedWidth(80)
        btn_details.clicked.connect(self._show_reliability_details)
        details_row.addWidget(btn_details)
        form_a.addLayout(details_row)
        outer.addWidget(box_angles)

    def _show_reliability_details(self):
        QMessageBox.information(self, 'Fiabilité par joint', self._RELIABILITY_DETAILS)

    @staticmethod
    def _format_stats_block(title, stats):
        if stats is None:
            return [title, 'Pas encore assez d\'échantillons DREAM (minimum 5).', '']
        lines = [title, f'Échantillons : {stats["n_samples"]}', '']
        lines.append(f'{"Joint":<6}{"MAE (°)":>10}{"RMS (°)":>10}')
        for j in range(6):
            lines.append(f'J{j + 1:<5}{stats["mae"][j]:>10.2f}{stats["rms"][j]:>10.2f}')
        lines.append('')
        lines.append(f'{"TOTAL":<6}{stats["mae_total"]:>10.2f}{stats["rms_total"]:>10.2f}')
        lines.append('')
        return lines

    def _show_angle_stats_details(self):
        session_stats = self.node.angle_error_stats()
        window_stats = self.node.angle_error_stats_window()
        lines = self._format_stats_block(
            f'— Session (depuis le lancement) —', session_stats)
        lines += self._format_stats_block(
            f'— Fenêtre {CURVE_WINDOW_S:.0f}s (même échantillons que les graphes) —', window_stats)
        QMessageBox.information(self, 'MAE / RMS par joint (live)', '\n'.join(lines))

    def _on_temporal_filter_toggle(self, state):
        self.node.use_temporal_filter = bool(state)

    @staticmethod
    def _hline():
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def refresh(self, overlay):
        n = self.node
        q_dream = n.estimate_dream_angles()

        if q_dream is not None:
            self.solver_status_label.setText('Solveur angles DREAM : OK')
            self.solver_status_label.setStyleSheet('')
        else:
            self.solver_status_label.setText('Solveur angles DREAM : —')
            self.solver_status_label.setStyleSheet('')

        if n.last_cold_restart_triggered:
            self.cold_restart_label.setText(
                f'Cold-restart : ⚠ déclenché ({n.last_cold_restart_reason}) — warm-start abandonné')
            self.cold_restart_label.setStyleSheet('color: #a06000; font-weight: bold;')
        else:
            self.cold_restart_label.setText('Cold-restart : non déclenché')
            self.cold_restart_label.setStyleSheet('color: #206020;')

        if n.n_solves > 0:
            restart_pct = 100.0 * n.n_cold_restarts / n.n_solves
            rupture_pct = 100.0 * n.n_pose_ruptures / n.n_solves
            self.restart_rate_label.setText(
                f'Cold-restarts : {n.n_cold_restarts}/{n.n_solves} ({restart_pct:.1f}%) · '
                f'Pose-ruptures : {n.n_pose_ruptures}/{n.n_solves} ({rupture_pct:.1f}%)')
        else:
            self.restart_rate_label.setText('Cold-restarts : — / Pose-ruptures : —')


class JointCurvesPanel(QWidget):
    """6 courbes temps réel : angle encodeur (plein) vs angle estimé DREAM (pointillé)."""

    WINDOW_S = CURVE_WINDOW_S

    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        outer = QVBoxLayout(self)

        curves_title = QLabel('Courbes')
        curves_title.setFont(QFont('Sans', 13, QFont.Bold))
        outer.addWidget(curves_title)

        # En-tête des courbes : légende seule. RMS reprojection est passé dans
        # le tableau caméra, la santé de pose est dessinée dans l'image.
        legend = QLabel(
            '<span style="color:#009600;">━</span> encodeur (FK)&nbsp;&nbsp;'
            '<span style="color:#e600c8;">╌</span> DREAM'
        )
        outer.addWidget(legend)

        grid = QGridLayout()
        outer.addLayout(grid, stretch=1)

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plots = []
        self.encoder_curves = []
        self.dream_curves = []
        for j in range(6):
            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(1)

            pw = pg.PlotWidget(title=f'Joint {j+1}')
            pw.setLabel('left', 'deg')
            pw.setLabel('bottom', 's')
            pw.showGrid(x=True, y=True, alpha=0.2)
            enc_curve = pw.plot([], [], pen=pg.mkPen(color=(0, 150, 0), width=2), name='encodeur')
            dream_curve = pw.plot([], [], pen=pg.mkPen(color=(230, 0, 200), width=2, style=Qt.DashLine),
                                   name='DREAM')
            cell_layout.addWidget(pw)

            self.plots.append(pw)
            self.encoder_curves.append(enc_curve)
            self.dream_curves.append(dream_curve)
            grid.addWidget(cell, j // 2, j % 2)

    def refresh(self, overlay=None):
        # RMS reprojection et santé de pose vivent ailleurs (tableau + image).
        hist = self.node.curve_history
        if not hist:
            return
        flags = self.node.joint_confidence(overlay)
        t = np.array([h[0] for h in hist])
        cutoff = t[-1] - self.WINDOW_S
        keep = t >= cutoff
        t = t[keep]
        enc = np.array([h[1] for h in hist])[keep]          # (N, 6)
        dream_raw = [h[2] for h in hist]
        dream_raw = [d for d, k in zip(dream_raw, keep) if k]

        wstats = self.node.angle_error_stats_window()

        for j in range(6):
            self.encoder_curves[j].setData(t, enc[:, j])
            dt = np.array([d[j] for d in dream_raw if d is not None])
            dtt = np.array([tt for tt, d in zip(t, dream_raw) if d is not None])
            if len(dtt) > 0:
                self.dream_curves[j].setData(dtt, dt)
                warn = '  ⚠' if not flags[j] else ''
                self.plots[j].setTitle(
                    f'Joint {j+1}   encodeur {enc[-1, j]:.1f}° · '
                    f'DREAM {dt[-1]:.1f}° · erreur {abs(dt[-1] - enc[-1, j]):.2f}°{warn}')
            else:
                self.plots[j].setTitle(f'Joint {j+1}   encodeur {enc[-1, j]:.1f}° · DREAM —')

            # Fenêtre fixe ±_PLOT_HALF_SPAN_DEG autour de l'encodeur : l'autoscale
            # étirerait 0.05° de jitter sur toute la hauteur et un joint immobile
            # aurait l'air de trembler violemment. Le fixe montre jitter et écart
            # encodeur/DREAM à leur vraie taille. L'axe temps (X) suit tout seul.
            centre = float(enc[-1, j])
            lo, hi = centre - _PLOT_HALF_SPAN_DEG, centre + _PLOT_HALF_SPAN_DEG
            if len(dtt) > 0:
                # Expansion bornée au débattement physique : un solve divergent
                # qui passerait le filtre étirerait sinon l'axe sur ~1000°,
                # écrasant la courbe utile en une ligne plate.
                lo = max(min(lo, float(dt.min())), centre - 180.0)
                hi = min(max(hi, float(dt.max())), centre + 180.0)
            self.plots[j].setYRange(lo, hi, padding=0.05)



class DashboardWindow(QMainWindow):
    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('ABMI — DREAM Validation Dashboard')
        if LOGO_PATH.exists():
            self.setWindowIcon(QIcon(str(LOGO_PATH)))  # logo dans la barre de titre
        self.resize(1400, 800)  # fallback size if the window manager ignores showMaximized()

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)

        # Single unified view, left to right: camera+overlay | courbes 6 joints
        # | pilotage+KPI. Les courbes prennent la part large (stretch 6 vs 3) —
        # c'est la zone lue en continu et les titres enc/DREAM/err ont besoin de
        # largeur pour ne pas être tronqués.
        row = QHBoxLayout()
        outer.addLayout(row)

        self.tab1 = Tab1LiveValidation(node)
        row.addWidget(self.tab1, stretch=3)

        self.curves = JointCurvesPanel(node)
        row.addWidget(self.curves, stretch=6)

        # Pilotage + KPI stacked is the tallest column (3 KPI group boxes +
        # a 6-row angle grid) — on screens under ~1170px available height it
        # was forcing the WHOLE window past the screen edge rather than
        # itself shrinking, clipping the last rows off-screen. A scroll area
        # lets this column shrink below its natural content height instead.
        middle_container = QWidget()
        middle = QVBoxLayout(middle_container)
        middle.setAlignment(Qt.AlignTop)  # colle le titre en haut, pas d'espace flottant
        ctrl_title = QLabel('Contrôle')
        ctrl_title.setFont(QFont('Sans', 13, QFont.Bold))
        middle.addWidget(ctrl_title)
        self.pilot = ManualPilotPanel(node)
        self.auto = AutoModePanel(node)
        self.kpi = KPIPanel(node)
        middle.addWidget(self.pilot)
        middle.addWidget(self.auto)
        middle.addWidget(self.kpi)
        middle_scroll = QScrollArea()
        middle_scroll.setWidgetResizable(True)
        middle_scroll.setFrameShape(QFrame.NoFrame)
        middle_scroll.setWidget(middle_container)
        row.addWidget(middle_scroll, stretch=3)

        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)  # ~30 Hz UI/ROS pump


    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node._dream_q_cache_valid = False
        self.tab1.refresh()
        self.node.tick_acquisition()
        self.pilot.refresh()
        self.auto.refresh()
        self.kpi.refresh(self.tab1.last_overlay)
        self.curves.refresh(self.tab1.last_overlay)

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    app = QApplication(sys.argv)
    if LOGO_PATH.exists():
        app.setWindowIcon(QIcon(str(LOGO_PATH)))  # icône à gauche du titre
    window = DashboardWindow(node)
    # showMaximized() alone isn't enough — Qt won't shrink a window below its
    # children's combined minimum size even when "maximized", so on a screen
    # smaller than that minimum the window overflows past the visible edge
    # instead of fitting. Explicitly clamp to the actual available screen
    # rect (excludes taskbars/panels), which forces a real fit.
    screen_geo = app.primaryScreen().availableGeometry()
    window.setGeometry(screen_geo)
    window.show()

    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
