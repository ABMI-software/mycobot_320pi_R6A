#!/usr/bin/env python3
"""Hand teleoperation for the MyCobot 320 Pi.

Adapted from the ArmR5A `arm_test.py` (hand-teleop team script). The only
MyCobot-specific parts are:
- 6-DoF joint mapping (the R5A has 5 arm joints)
- URDF joint names (joint2_to_joint1 … joint6output_to_joint6)
- Joint limits matching the URDF (and the official elephantrobotics limits)
- Default topic /mycobot_controller/joint_trajectory
- SAFE_RANGE tightened to the MyCobot reach envelope

The control strategy is unchanged: the Wilor hand pose gives XYZ, which is
linearly mapped to joint angles (no IK). The Z axis is inverted by default
so that "hand up ⇒ arm up".

Usage (requires the hand-teleop conda env):

    conda activate hand-teleop
    python mycobot_teleop.py \\
        --ros --use-rosbridge \\
        --ros-topic /mycobot_controller/joint_trajectory \\
        --time-from-start 0.8 \\
        --z-gain 1.6 --x-gain 1.2 --y-gain 1.2
"""
from __future__ import annotations

import os
os.environ.setdefault("DISABLE_CV2_GUI", "1")
os.environ.setdefault("HEADLESS", "1")
os.environ.setdefault("MPLBACKEND", "Agg")

import argparse
import importlib
import threading
import time
from pathlib import Path
from typing import Tuple, Dict, List, Optional

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    import matplotlib
    matplotlib.use(os.environ["MPLBACKEND"])
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False
    plt = None

from hand_teleop.gripper_pose.gripper_pose import GripperPose
from hand_teleop.hand_pose.factory import ModelName
from hand_teleop.tracking.tracker import HandTracker


# --------------------------- MyCobot configuration --------------------------- #

# Wilor XYZ envelope (metres) — aligned with the R5A defaults since those are
# battle-tested for typical webcam framing. Tightening this for the MyCobot's
# physical reach (~280 mm) causes clipping as soon as the user's hand drifts
# off-centre; use --x-gain / --y-gain / --z-gain if you want less dynamic range.
SAFE_RANGE = {
    "x": (0.13, 0.36),
    "y": (-0.23, 0.23),
    "z": (0.008, 0.25),
    "g": (2, 90),
}

# URDF joint limits (radians) converted to degrees, matching the official
# elephantrobotics spec and the joint limits set in the Gazebo URDF.
JOINT_LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    "joint2_to_joint1":      (-168.0, 168.0),  # J1  (base yaw)
    "joint3_to_joint2":      (-134.6, 134.6),  # J2  (shoulder)
    "joint4_to_joint3":      (-145.0, 145.0),  # J3  (elbow)
    "joint5_to_joint4":      (-145.0, 145.0),  # J4  (wrist pitch 1)
    "joint6_to_joint5":      (-168.0, 168.0),  # J5  (wrist yaw)
    "joint6output_to_joint6": (-180.0, 180.0),  # J6  (end-effector roll)
}

DEFAULT_JOINT_NAMES = list(JOINT_LIMITS_DEG.keys())
DEFAULT_TOPIC = "/mycobot_controller/joint_trajectory"


# ------------------------------- helpers ------------------------------------ #

def clamp(v: float, lo: float, hi: float) -> float:
    return float(min(max(v, lo), hi))


def map_range(v: float, src: Tuple[float, float], dst: Tuple[float, float]) -> float:
    s0, s1 = src
    d0, d1 = dst
    if s1 == s0:
        return d0
    t = (v - s0) / (s1 - s0)
    t = clamp(t, 0.0, 1.0)
    return d0 + t * (d1 - d0)


def maybe_reverse(rng: Tuple[float, float], invert: bool) -> Tuple[float, float]:
    lo, hi = rng
    return (hi, lo) if invert else (lo, hi)


def get_xyz_from_pose(pose: GripperPose) -> np.ndarray:
    if hasattr(pose, "pos"):
        return np.asarray(pose.pos, dtype=float)
    return np.asarray(getattr(pose, "position"), dtype=float)


def get_rot_from_pose(pose: GripperPose) -> np.ndarray:
    if hasattr(pose, "rot"):
        return np.asarray(pose.rot, dtype=float)
    return np.asarray(getattr(pose, "rotation"), dtype=float)


# How many degrees of joint motion we produce per metre of hand motion at
# gain=1.0. Tuned down twice (300 → 200 → 150) to keep the commanded
# trajectory inside the JTC's achievable envelope — at gain 1.2, 15 cm of
# hand motion now produces 27° of commanded travel, which the controller
# tracks cleanly even during aggressive reach moves.
BASE_SCALE_DEG_PER_M = 150.0

# EMA smoothing — dampens Wilor/Kalman jitter.
DEFAULT_COMMAND_EMA_ALPHA = 0.20

# Max joint-angle change per frame, in degrees. At 30 Hz that's 90 °/s —
# about half of the URDF velocity limit (180 °/s) so the JTC always has
# headroom to catch up instead of saturating on transients.
DEFAULT_MAX_DELTA_DEG_PER_FRAME = 3.0


def xyz_to_joints_deg(
    rel_xyz: np.ndarray,
    *,
    rpy_deg: Tuple[float, float, float] | None = None,
    invert_x: bool = True,   # hand forward  → elbow extends (EE goes forward)
    invert_y: bool = False,  # hand right    → base rotates right
    invert_z: bool = False,  # hand up       → shoulder tilts back (EE goes up)
    invert_roll: bool = False,
    invert_pitch: bool = False,
    x_gain: float = 1.0,
    y_gain: float = 1.0,
    z_gain: float = 1.0,
    roll_gain: float = 0.4,   # hand twist → J6 end-effector roll (Wilor roll is noisy — keep gain low)
    pitch_gain: float = 0.4,  # hand pitch → split across J4 + J5 for gripper orientation
    joint_limits: Dict[str, Tuple[float, float]] = JOINT_LIMITS_DEG,
    joint_names: List[str] = DEFAULT_JOINT_NAMES,
) -> np.ndarray:
    """Map relative hand XYZ (m) to joint angles (deg) in delta form.

    The mapping is intentionally *relative*: with rel_xyz = (0, 0, 0) all
    joints are exactly 0°, so after Wilor captures its initial_pose on
    first detection the robot stays at home. The joints only move as the
    operator's hand drifts away from that reference.

    Axis routing (operator's point of view):
      Y delta (lateral)   → J1 base yaw         hand right → base right
      Z delta (vertical)  → J2 shoulder pitch   hand up → EE up
      X delta (depth)     → J3 elbow            hand forward → arm extends
      Z delta (vertical)  → J5 wrist pitch      half-coupled to shoulder
      J4, J6 stay at 0° (not driven yet)

    Gains are in dimensionless multipliers on top of BASE_SCALE_DEG_PER_M
    (≈600 deg/m). Defaults land J2/J3 around 45–90° for a 15 cm hand move,
    which feels natural. Flip signs with --no-invert-{x,y,z} if your
    robot orientation mirrors the operator.
    """
    dx, dy, dz = rel_xyz.tolist()
    if invert_x:
        dx = -dx
    if invert_y:
        dy = -dy
    if invert_z:
        dz = -dz

    scale = BASE_SCALE_DEG_PER_M
    j1 = dy * scale * y_gain           # base yaw   ← lateral
    j2 = dz * scale * z_gain           # shoulder   ← vertical
    j3 = dx * scale * x_gain           # elbow      ← depth

    # Hand orientation → wrist joints (J4 + J5 split the pitch so their
    # combined rotation points the EE in the direction the palm points,
    # J6 carries the roll / twist).
    # rpy_deg is a relative (roll, pitch, yaw) tuple in degrees; Wilor
    # produces ±60° at most for a natural palm rotation.
    if rpy_deg is not None:
        roll, pitch, _ = rpy_deg
        if invert_roll:
            roll = -roll
        if invert_pitch:
            pitch = -pitch
        j4 = pitch * pitch_gain * 0.5  # half of pitch on wrist1
        j5 = pitch * pitch_gain * 0.5  # other half on wrist2 — EE tracks palm
        j6 = roll * roll_gain
    else:
        j4 = 0.0
        j5 = 0.0
        j6 = 0.0

    joints = np.array([j1, j2, j3, j4, j5, j6], dtype=float)
    lims = {k: joint_limits[k] for k in joint_names}
    for i, name in enumerate(joint_names):
        lo, hi = lims[name]
        joints[i] = clamp(joints[i], lo, hi)
    return joints


def try_open_cap(idx_or_path, width: int, height: int):
    backends = [cv2.CAP_V4L2, cv2.CAP_ANY] if isinstance(idx_or_path, int) else [cv2.CAP_ANY]
    for be in backends:
        cap = cv2.VideoCapture(idx_or_path, be)
        if not cap.isOpened():
            cap.release()
            continue
        if isinstance(idx_or_path, int):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, _ = cap.read()
        if ok:
            return cap, ("V4L2" if be == cv2.CAP_V4L2 else "ANY")
        cap.release()
    return None, None


def pick_camera(preferred: int, width: int = 640, height: int = 480) -> int:
    tried = []
    for idx in [preferred] + [i for i in range(6) if i != preferred]:
        tried.append(idx)
        cap, _ = try_open_cap(idx, width, height)
        if cap is not None:
            cap.release()
            return idx
    raise RuntimeError("No usable camera. Tried: " + ", ".join(f"/dev/video{i}" for i in tried))


# ---------------------- ROS 2 publisher (direct rclpy) ----------------------- #

class _LazyRos2:
    loaded = False
    rclpy = None
    Node = None
    JointTrajectory = None
    JointTrajectoryPoint = None
    Duration = None

    @classmethod
    def load(cls):
        if cls.loaded:
            return
        cls.rclpy = importlib.import_module("rclpy")
        cls.Node = getattr(importlib.import_module("rclpy.node"), "Node")
        traj = importlib.import_module("trajectory_msgs.msg")
        cls.JointTrajectory = traj.JointTrajectory
        cls.JointTrajectoryPoint = traj.JointTrajectoryPoint
        cls.Duration = importlib.import_module("builtin_interfaces.msg").Duration
        cls.loaded = True


class RosArmPublisher:
    def __init__(self, topic: str, time_from_start_s: float, joint_names: List[str]) -> None:
        _LazyRos2.load()
        self.rclpy = _LazyRos2.rclpy
        self.JT = _LazyRos2.JointTrajectory
        self.JTP = _LazyRos2.JointTrajectoryPoint
        self.Duration = _LazyRos2.Duration
        self.node = _LazyRos2.Node("mycobot_teleop_publisher")
        self.pub = self.node.create_publisher(self.JT, topic, 10)
        sec = int(time_from_start_s)
        nsec = int((time_from_start_s - sec) * 1e9)
        self.tfs = self.Duration(sec=sec, nanosec=nsec)
        self.joint_names = joint_names

    def send_deg(self, joint_deg: np.ndarray) -> None:
        msg = self.JT()
        msg.joint_names = self.joint_names
        pt = self.JTP()
        pt.positions = np.radians(joint_deg).astype(float).tolist()
        pt.time_from_start = self.tfs
        msg.points = [pt]
        self.pub.publish(msg)

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:
            pass


# ---------------------- ROS 2 publisher (via rosbridge) ---------------------- #

class RosBridgeArmPublisher:
    """Publishes JointTrajectory through rosbridge (WebSocket) — matches the R5A flow.

    Also publishes /teleop/hand_xyz (Vector3Stamped) for the monitoring dashboard
    and subscribes to /teleop/gains (Float64MultiArray [x_gain, y_gain, z_gain,
    time_from_start_s]) so gains can be tuned live without restarting the script.
    """

    def __init__(self, host: str, port: int, topic: str, time_from_start_s: float,
                 joint_names: List[str]) -> None:
        try:
            import roslibpy
        except ImportError as e:
            raise SystemExit("Missing dependency: roslibpy (pip install roslibpy)") from e
        self._roslibpy = roslibpy
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        if not self.ros.is_connected:
            raise SystemExit(f"Could not connect to rosbridge ws://{host}:{port}")
        self.topic = roslibpy.Topic(self.ros, topic, "trajectory_msgs/JointTrajectory")
        self.topic.advertise()
        self.joint_names = joint_names

        # Monitoring: publish hand XYZ for the dashboard
        self.hand_xyz_topic = roslibpy.Topic(
            self.ros, "/teleop/hand_xyz", "geometry_msgs/Vector3Stamped"
        )
        self.hand_xyz_topic.advertise()

        # Gripper position command (Gazebo). Pro adaptive gripper has one
        # driving joint; we send [rad] where 0 = open, -0.7 = closed (matches
        # the URDF limits). See set_gripper_normalized() below.
        self.gripper_topic = roslibpy.Topic(
            self.ros, "/gripper_position_controller/commands",
            "std_msgs/Float64MultiArray",
        )
        self.gripper_topic.advertise()

        # Live gain tuning: subscribe to /teleop/gains
        self._gain_callback = None
        self._tfs_callback = None
        self._recalibrate_callback = None
        self.gain_topic = roslibpy.Topic(
            self.ros, "/teleop/gains", "std_msgs/Float64MultiArray"
        )
        self.gain_topic.subscribe(self._on_gain_message)

        # Recalibration trigger (Dashboard → teleop)
        self.recal_topic = roslibpy.Topic(
            self.ros, "/teleop/recalibrate", "std_msgs/Empty"
        )
        self.recal_topic.subscribe(self._on_recalibrate_message)
        sec = int(time_from_start_s)
        nsec = int((time_from_start_s - sec) * 1e9)
        self.tfs = {"sec": sec, "nanosec": nsec}

    def send_deg(self, joint_deg: np.ndarray) -> None:
        msg = {
            "joint_names": self.joint_names,
            "points": [{
                "positions": np.radians(joint_deg).astype(float).tolist(),
                "velocities": [],
                "accelerations": [],
                "effort": [],
                "time_from_start": self.tfs,
            }],
        }
        self.topic.publish(msg)

    def send_hand_xyz(self, xyz: np.ndarray) -> None:
        msg = {
            "header": {"frame_id": "camera", "stamp": {"sec": 0, "nanosec": 0}},
            "vector": {"x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2])},
        }
        self.hand_xyz_topic.publish(msg)

    def send_gripper_normalized(self, openness: float) -> None:
        """Publish gripper command. openness ∈ [0, 1]: 0 = closed, 1 = open.

        The controller drives four joints explicitly (see controller.yaml):
            [servo_left, servo_right, tip_left, tip_right]
        so we build a symmetric 4-element target that keeps the fingers
        mirrored AND the fingertips parallel to the base across the sweep:

            open  (o=1):  all four at  0 rad
            close (o=0):  [-0.7, +0.7, +0.7, -0.7]
        """
        o = max(0.0, min(1.0, float(openness)))
        servo = -0.7 * (1.0 - o)          # gripper_controller (left servo)
        data = [servo, -servo, -servo, servo]
        self.gripper_topic.publish({"data": data})

    def set_tfs(self, time_from_start_s: float) -> None:
        sec = int(time_from_start_s)
        nsec = int((time_from_start_s - sec) * 1e9)
        self.tfs = {"sec": sec, "nanosec": nsec}

    def set_gain_callback(self, cb) -> None:
        """cb(x_gain: float, y_gain: float, z_gain: float, tfs: float) -> None"""
        self._gain_callback = cb

    def set_recalibrate_callback(self, cb) -> None:
        """cb() -> None, called when /teleop/recalibrate receives a message."""
        self._recalibrate_callback = cb

    def _on_recalibrate_message(self, _msg: dict) -> None:
        if self._recalibrate_callback is not None:
            try:
                self._recalibrate_callback()
            except Exception as e:
                print(f"[WARN] recalibrate failed: {e}")

    def _on_gain_message(self, msg: dict) -> None:
        if self._gain_callback is None:
            return
        data = msg.get("data", [])
        if len(data) < 3:
            return
        x_gain = float(data[0])
        y_gain = float(data[1])
        z_gain = float(data[2])
        tfs = float(data[3]) if len(data) >= 4 else None
        try:
            self._gain_callback(x_gain, y_gain, z_gain, tfs)
            if tfs is not None:
                self.set_tfs(tfs)
        except Exception as e:
            print(f"[WARN] gain update failed: {e}")

    def shutdown(self) -> None:
        try:
            self.topic.unadvertise()
            self.hand_xyz_topic.unadvertise()
            self.gripper_topic.unadvertise()
            self.gain_topic.unsubscribe()
            self.recal_topic.unsubscribe()
            self.ros.terminate()
        except Exception:
            pass


# ------------------------------- main --------------------------------------- #

def main(
    *,
    quiet: bool = False,
    fps: int = 30,
    model: ModelName = "wilor",
    camera: str = "auto",
    cam_idx: int = 0,
    width: int = 640,
    height: int = 440,
    hand: str = "right",
    run_seconds: Optional[float] = None,
    ros2_enable: bool = True,
    use_rosbridge: bool = False,
    rosbridge_host: str = "localhost",
    rosbridge_port: int = 9090,
    ros2_topic: str = DEFAULT_TOPIC,
    joint_names: List[str] = DEFAULT_JOINT_NAMES,
    time_from_start: float = 0.25,
    x_gain: float = 1.2,
    y_gain: float = 1.2,
    z_gain: float = 1.6,
    invert_x: bool = False,
    invert_y: bool = False,
    invert_z: bool = False,
    video: Optional[str] = None,
) -> None:

    # -------- open camera / video --------
    cap = None
    if video:
        if not Path(video).exists():
            print(f"[ERROR] --video not found: {video}")
            return
        cap, be = try_open_cap(video, width, height)
        if cap is None:
            print(f"[ERROR] Failed to open video: {video}")
            return
        src_desc = Path(video).name
    elif camera == "astra":
        from orbbec_capture import open_orbbec
        try:
            cap = open_orbbec(auto_spawn=True)
            be = "oni_grabber+shm"
        except Exception as e:
            print(f"[ERROR] Failed to open Astra: {e}")
            return
        src_desc = "Orbbec Astra S (shared-memory)"
    else:
        try:
            chosen_idx = pick_camera(cam_idx, width, height)
        except RuntimeError as e:
            print(f"[ERROR] {e}")
            return
        cap, be = try_open_cap(chosen_idx, width, height)
        if cap is None:
            print(f"[ERROR] Camera /dev/video{chosen_idx} opened for test but failed later.")
            return
        src_desc = f"/dev/video{chosen_idx}"
    if not quiet:
        print(f"[INFO] Input: {src_desc} (backend={be})")

    # -------- build hand tracker (no pinocchio) --------
    # Zero base_pose so the tracker returns pure relative displacement —
    # that way rel=0 (hand at calibration reference) maps to all joints 0°.
    follower_pose = GripperPose.zero()

    tracker = HandTracker(
        cam_idx=0,
        hand=hand,
        model=model,
        urdf_path=None,
        safe_range=SAFE_RANGE,
        use_scroll=False,
        kf_dt=1 / max(1, fps),
    )
    try:
        tracker.cap.release()
    except Exception:
        pass
    tracker.cap = cap

    # -- Protect the capture loop against Wilor exceptions --
    # The upstream HandTracker._capture_loop has no try/except around
    # pose_computer.compute_relative_pose(), so a single malformed output
    # (NaN keypoints, shape mismatch, etc.) would silently kill the daemon
    # thread and freeze detection for the rest of the session. We wrap
    # compute_relative_pose so exceptions just produce a None detection
    # (same as "no hand visible" — the KF simply doesn't update that frame).
    _orig_crp = tracker.pose_computer.compute_relative_pose
    _crp_fail_count = [0]

    def _safe_compute_relative_pose(frame, focal_length, cam_t):
        try:
            return _orig_crp(frame, focal_length, cam_t)
        except Exception as e:
            _crp_fail_count[0] += 1
            if _crp_fail_count[0] <= 3 or _crp_fail_count[0] % 50 == 0:
                print(f"[WARN] Wilor compute_relative_pose failed "
                      f"(#{_crp_fail_count[0]}): {type(e).__name__}: {e}",
                      flush=True)
            return None
    tracker.pose_computer.compute_relative_pose = _safe_compute_relative_pose

    try:
        tracker._resume()
        if not quiet:
            print("[INFO] Tracker auto-resumed (press SPACE to pause, 'p' to toggle)")
    except Exception as e:
        print(f"[WARN] Could not auto-resume tracker: {e}")

    if not quiet:
        print(f"[INFO] Publishing joints: {joint_names}")
        print(f"[INFO] ROS topic: {ros2_topic}")
        print(f"[INFO] Mode: {'rosbridge' if use_rosbridge else 'direct rclpy'}")
        print(f"[INFO] Gains: x={x_gain} y={y_gain} z={z_gain}  "
              f"Invert: X={invert_x} Y={invert_y} Z={invert_z}")

    # -------- ROS publisher --------
    ros_pub = None
    if ros2_enable:
        try:
            if use_rosbridge:
                ros_pub = RosBridgeArmPublisher(
                    rosbridge_host, rosbridge_port, ros2_topic, time_from_start, joint_names
                )
            else:
                _LazyRos2.load()
                _LazyRos2.rclpy.init()
                ros_pub = RosArmPublisher(ros2_topic, time_from_start, joint_names)
        except Exception as e:
            print(f"[WARN] ROS init failed: {e}. Continuing without ROS.")
            ros_pub = None

    target_dt = 1.0 / fps
    ema_fps = None
    t_start = time.perf_counter()

    # Mutable gain container — can be updated from the dashboard via rosbridge.
    gains = {"x": x_gain, "y": y_gain, "z": z_gain}

    # EMA state for the published joint command. Zero-init so the first
    # publish matches whatever the first raw mapping produces (Wilor rel=0
    # → joints=0 → EMA stays at 0).
    q_smoothed = np.zeros(6)
    ema_alpha = DEFAULT_COMMAND_EMA_ALPHA

    def _on_gain_update(gx, gy, gz, tfs):
        gains["x"], gains["y"], gains["z"] = gx, gy, gz
        if not quiet:
            print(f"[GAINS] x={gx:.2f} y={gy:.2f} z={gz:.2f} tfs={tfs}", flush=True)

    def _recalibrate():
        """Reset the tracker's initial pose, using the next detection as new zero."""
        try:
            tracker._pause()
            time.sleep(0.05)
            tracker._resume()  # this calls pose_computer.reset() internally
            if not quiet:
                print("[RECAL] Tracker initial_pose cleared — keep palm in view", flush=True)
        except Exception as e:
            print(f"[WARN] recalibrate failed: {e}")

    if ros_pub is not None and use_rosbridge:
        try:
            ros_pub.set_gain_callback(_on_gain_update)
            ros_pub.set_recalibrate_callback(_recalibrate)
        except AttributeError:
            pass  # direct rclpy publisher doesn't support live tuning

    # Background watchdog: if the Astra frames stop flowing (oni_grabber dies
    # or its tick file goes stale), restart the grabber so the operator
    # doesn't have to rerun the whole teleop.
    if camera == "astra":
        from orbbec_capture import _oni_grabber_alive, _tick_is_fresh, _spawn_oni_grabber
        _stop_watchdog = threading.Event()

        def _astra_watchdog():
            while not _stop_watchdog.is_set():
                time.sleep(1.0)
                if not _oni_grabber_alive() or not _tick_is_fresh(max_age_s=2.0):
                    print("[WATCHDOG] Astra frames stalled — respawning oni_grabber",
                          flush=True)
                    try:
                        _spawn_oni_grabber()
                    except Exception as e:
                        print(f"[WATCHDOG] respawn failed: {e}", flush=True)
        threading.Thread(target=_astra_watchdog, daemon=True).start()
    else:
        _stop_watchdog = None

    try:
        while tracker.cap.isOpened():
            if run_seconds is not None and (time.perf_counter() - t_start) >= run_seconds:
                break

            t0 = time.perf_counter()
            try:
                pose = tracker.read_hand_state(follower_pose)
            except RuntimeError as e:
                if not quiet:
                    print(f"[ERROR] {e}")
                break

            xyz = get_xyz_from_pose(pose)
            # Extract hand roll/pitch/yaw from the relative rotation matrix
            # for wrist control (J4 pitch, J6 roll).
            try:
                rot = get_rot_from_pose(pose)
                yaw_deg, pitch_deg, roll_deg = R.from_matrix(rot).as_euler(
                    "ZYX", degrees=True
                )
                rpy = (float(roll_deg), float(pitch_deg), float(yaw_deg))
            except Exception:
                rpy = None

            q_raw = xyz_to_joints_deg(
                xyz,
                rpy_deg=rpy,
                invert_x=invert_x, invert_y=invert_y, invert_z=invert_z,
                x_gain=gains["x"], y_gain=gains["y"], z_gain=gains["z"],
                joint_limits=JOINT_LIMITS_DEG,
                joint_names=joint_names,
            )

            # EMA smoothing + slew rate limiter. EMA settles genuine moves
            # in ~4 frames (~65 ms at 60 Hz); the slew limiter caps each
            # frame's delta at DEFAULT_MAX_DELTA_DEG_PER_FRAME so a single
            # Wilor reacquisition spike can't push more than 8°/frame
            # downstream. Together they keep the JTC inside its smooth
            # tracking regime.
            ema_target = (1.0 - ema_alpha) * q_smoothed + ema_alpha * q_raw
            delta = ema_target - q_smoothed
            max_d = DEFAULT_MAX_DELTA_DEG_PER_FRAME
            delta = np.clip(delta, -max_d, max_d)
            q_smoothed[:] = q_smoothed + delta
            q_deg = q_smoothed.copy()

            if ros_pub is not None:
                ros_pub.send_deg(q_deg)
                if use_rosbridge:
                    # Mirror hand XYZ + gripper openness for the dashboard
                    try:
                        ros_pub.send_hand_xyz(xyz)
                    except AttributeError:
                        pass
                    # Map Wilor open_degree (SAFE_RANGE["g"] = 2..90) → [0, 1]
                    try:
                        g_lo, g_hi = SAFE_RANGE["g"]
                        opn = float(pose.open_degree) if hasattr(pose, "open_degree") else g_lo
                        normalized = (opn - g_lo) / max(1e-6, g_hi - g_lo)
                        ros_pub.send_gripper_normalized(normalized)
                    except AttributeError:
                        pass

            dt = time.perf_counter() - t0
            inst = 1.0 / dt if dt > 0 else fps
            ema_fps = inst if ema_fps is None else 0.9 * ema_fps + 0.1 * inst
            if not quiet:
                pairs = ", ".join(f"{n.split('_')[0]}:{v:+.1f}" for n, v in zip(joint_names, q_deg))
                print(f"XYZ {np.round(xyz, 3)} | {pairs} | FPS {ema_fps:.1f}")

            remain = target_dt - (time.perf_counter() - t0)
            if remain > 0:
                time.sleep(remain)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            tracker.cap.release()
        except Exception:
            pass
        if ros_pub is not None:
            try:
                ros_pub.shutdown()
                if not use_rosbridge:
                    _LazyRos2.rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="MyCobot 320 Pi hand teleoperation")
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--model", type=str, default="wilor")
    p.add_argument("--camera", type=str, default="auto", choices=["auto", "astra"],
                   help="'auto' = UVC webcam via cv2; 'astra' = Orbbec Astra via OpenNI2.")
    p.add_argument("--cam-idx", type=int, default=0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=440)
    p.add_argument("--hand", type=str, default="right", choices=["left", "right"])
    p.add_argument("--run-seconds", type=float, default=None)
    p.add_argument("--video", type=str, default="")

    # ROS
    p.add_argument("--ros", dest="ros2_enable", action="store_true")
    p.add_argument("--no-ros", dest="ros2_enable", action="store_false")
    p.set_defaults(ros2_enable=True)
    p.add_argument("--use-rosbridge", action="store_true",
                   help="Publish via rosbridge WebSocket instead of direct rclpy.")
    p.add_argument("--rosbridge-host", default="localhost")
    p.add_argument("--rosbridge-port", type=int, default=9090)
    p.add_argument("--ros-topic", default=DEFAULT_TOPIC)
    p.add_argument("--joints", default=",".join(DEFAULT_JOINT_NAMES),
                   help="Comma-separated joint names in controller order")
    p.add_argument("--time-from-start", type=float, default=0.25)

    # Axis gains & inversions
    p.add_argument("--x-gain", type=float, default=1.2)
    p.add_argument("--y-gain", type=float, default=1.2)
    p.add_argument("--z-gain", type=float, default=1.6)
    p.add_argument("--invert-x", dest="invert_x", action="store_true")
    p.add_argument("--no-invert-x", dest="invert_x", action="store_false")
    p.set_defaults(invert_x=True)
    p.add_argument("--invert-y", dest="invert_y", action="store_true")
    p.add_argument("--no-invert-y", dest="invert_y", action="store_false")
    p.set_defaults(invert_y=False)
    p.add_argument("--invert-z", dest="invert_z", action="store_true")
    p.add_argument("--no-invert-z", dest="invert_z", action="store_false")
    p.set_defaults(invert_z=False)

    args = p.parse_args()
    joint_names = [s.strip() for s in args.joints.split(",") if s.strip()]
    if len(joint_names) != 6:
        print(f"[WARN] --joints must list 6 names; got {len(joint_names)}. "
              f"Falling back to default.")
        joint_names = DEFAULT_JOINT_NAMES

    main(
        quiet=args.quiet, fps=args.fps, model=args.model,
        camera=args.camera,
        cam_idx=args.cam_idx, width=args.width, height=args.height,
        hand=args.hand, run_seconds=args.run_seconds,
        ros2_enable=args.ros2_enable,
        use_rosbridge=args.use_rosbridge,
        rosbridge_host=args.rosbridge_host,
        rosbridge_port=args.rosbridge_port,
        ros2_topic=args.ros_topic,
        joint_names=joint_names,
        time_from_start=args.time_from_start,
        x_gain=args.x_gain, y_gain=args.y_gain, z_gain=args.z_gain,
        invert_x=args.invert_x, invert_y=args.invert_y, invert_z=args.invert_z,
        video=(args.video or None),
    )
