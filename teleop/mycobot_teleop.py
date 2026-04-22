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


def xyz_to_joints_deg(
    xyz: np.ndarray,
    *,
    invert_x: bool = False,
    invert_y: bool = False,
    invert_z: bool = True,
    x_gain: float = 1.0,
    y_gain: float = 1.0,
    z_gain: float = 1.0,
    joint_limits: Dict[str, Tuple[float, float]] = JOINT_LIMITS_DEG,
    joint_names: List[str] = DEFAULT_JOINT_NAMES,
) -> np.ndarray:
    """Map hand XYZ to 6 MyCobot joint angles (deg). No IK — direct linear map.

    Axis mapping (same logic as R5A, extended to 6 joints):
      Y (hand lateral) → J1 base yaw
      X (hand forward) → J2 shoulder
      Z (hand height)  → J3 elbow
      J4 stays neutral (could be mapped to wrist-pitch later)
      Z (hand height)  → J5 wrist yaw (complementary to elbow)
      J6 stays neutral (end-effector roll, controlled later from hand roll)

    Per-axis gains (x_gain/y_gain/z_gain) scale the effective workspace so
    that smaller hand motion produces larger joint motion, matching the
    R5A flags --x-gain/--y-gain/--z-gain.
    """
    x, y, z = xyz.tolist()

    # Apply gains by shrinking the "active" input band around the midpoint.
    def _scaled(rng: Tuple[float, float], gain: float) -> Tuple[float, float]:
        lo, hi = rng
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo) / max(1e-6, gain)
        return (mid - half, mid + half)

    src_x = _scaled(SAFE_RANGE["x"], x_gain)
    src_y = _scaled(SAFE_RANGE["y"], y_gain)
    src_z = _scaled(SAFE_RANGE["z"], z_gain)

    lims = {k: joint_limits[k] for k in joint_names}
    dst_j1 = maybe_reverse(lims[joint_names[0]], invert_y)
    dst_j2 = maybe_reverse(lims[joint_names[1]], invert_x)
    dst_j3 = maybe_reverse(lims[joint_names[2]], invert_z)
    dst_j5 = maybe_reverse(lims[joint_names[4]], invert_z)

    j1 = map_range(y, src_y, dst_j1)
    j2 = map_range(x, src_x, dst_j2)
    j3 = map_range(z, src_z, dst_j3)
    j4 = 0.0
    j5 = map_range(z, src_z, dst_j5)
    j6 = 0.0

    joints = np.array([j1, j2, j3, j4, j5, j6], dtype=float)
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
    """Publishes JointTrajectory through rosbridge (WebSocket) — matches the R5A flow."""

    def __init__(self, host: str, port: int, topic: str, time_from_start_s: float,
                 joint_names: List[str]) -> None:
        try:
            import roslibpy
        except ImportError as e:
            raise SystemExit("Missing dependency: roslibpy (pip install roslibpy)") from e
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        if not self.ros.is_connected:
            raise SystemExit(f"Could not connect to rosbridge ws://{host}:{port}")
        self.topic = roslibpy.Topic(self.ros, topic, "trajectory_msgs/JointTrajectory")
        self.topic.advertise()
        self.joint_names = joint_names
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

    def shutdown(self) -> None:
        try:
            self.topic.unadvertise()
            self.ros.terminate()
        except Exception:
            pass


# ------------------------------- main --------------------------------------- #

def main(
    *,
    quiet: bool = False,
    fps: int = 60,
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
    time_from_start: float = 0.8,
    x_gain: float = 1.2,
    y_gain: float = 1.2,
    z_gain: float = 1.6,
    invert_x: bool = False,
    invert_y: bool = False,
    invert_z: bool = True,
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
    follower_pos = np.array([0.18, 0.0, 0.1])
    follower_rot = R.from_euler("ZYX", [0, 45, -90], degrees=True).as_matrix()
    follower_pose = GripperPose(follower_pos, follower_rot, open_degree=5)

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

    # HandTracker defaults to tracking_paused=True, waiting for a SPACE/p
    # keypress. We're running automated, so resume immediately.
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
            q_deg = xyz_to_joints_deg(
                xyz,
                invert_x=invert_x, invert_y=invert_y, invert_z=invert_z,
                x_gain=x_gain, y_gain=y_gain, z_gain=z_gain,
                joint_limits=JOINT_LIMITS_DEG,
                joint_names=joint_names,
            )

            if ros_pub is not None:
                ros_pub.send_deg(q_deg)

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
    p.add_argument("--fps", type=int, default=60)
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
    p.add_argument("--time-from-start", type=float, default=0.8)

    # Axis gains & inversions
    p.add_argument("--x-gain", type=float, default=1.2)
    p.add_argument("--y-gain", type=float, default=1.2)
    p.add_argument("--z-gain", type=float, default=1.6)
    p.add_argument("--invert-x", dest="invert_x", action="store_true")
    p.add_argument("--no-invert-x", dest="invert_x", action="store_false")
    p.set_defaults(invert_x=False)
    p.add_argument("--invert-y", dest="invert_y", action="store_true")
    p.add_argument("--no-invert-y", dest="invert_y", action="store_false")
    p.set_defaults(invert_y=False)
    p.add_argument("--invert-z", dest="invert_z", action="store_true")
    p.add_argument("--no-invert-z", dest="invert_z", action="store_false")
    p.set_defaults(invert_z=True)

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
