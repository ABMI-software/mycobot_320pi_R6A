#!/usr/bin/env python3
"""Forward kinematics for MyCobot 320 Pi — computes 3D keypoint positions.

Computes the world-frame position of each link origin (= keypoint)
from a set of joint angles, using the DH-like chain extracted from
the URDF (mycobot_pro_320_pi.urdf).

URDF joint chain (all revolute about local z):
  world  → base       (fixed, identity)
  base   → link1      joint2_to_joint1:  xyz=(0, 0, 0.162)   rpy=(0, 0, 0)
  link1  → link2      joint3_to_joint2:  xyz=(0, 0, 0)        rpy=(0, -π/2, π/2)
  link2  → link3      joint4_to_joint3:  xyz=(0.13635, 0, 0)  rpy=(0, 0, 0)
  link3  → link4      joint5_to_joint4:  xyz=(0.1205, 0, 0.082)  rpy=(0, 0, π/2)
  link4  → link5      joint6_to_joint5:  xyz=(0, -0.084, 0)   rpy=(π/2, 0, 0)
  link5  → link6      joint6output:      xyz=(0, 0.06635, 0)  rpy=(-π/2, 0, 0)
"""

import math
import numpy as np


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0],
                     [0, c, -s, 0],
                     [0, s, c, 0],
                     [0, 0, 0, 1]], dtype=np.float64)


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0],
                     [0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [0, 0, 0, 1]], dtype=np.float64)


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=np.float64)


def _T(x, y, z):
    m = np.eye(4, dtype=np.float64)
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


def _rpy(roll, pitch, yaw):
    """URDF rpy convention: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)


# ---------- URDF-derived joint parameters ----------
# Each entry: (translation xyz, rpy, axis)   axis is always 'z' for revolute
_JOINT_PARAMS = [
    # joint2_to_joint1: base → link1
    {"xyz": (0.0, 0.0, 0.162), "rpy": (0.0, 0.0, 0.0)},
    # joint3_to_joint2: link1 → link2
    {"xyz": (0.0, 0.0, 0.0), "rpy": (0.0, -math.pi / 2, math.pi / 2)},
    # joint4_to_joint3: link2 → link3
    {"xyz": (0.13635, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0)},
    # joint5_to_joint4: link3 → link4
    {"xyz": (0.1205, 0.0, 0.082), "rpy": (0.0, 0.0, math.pi / 2)},
    # joint6_to_joint5: link4 → link5
    {"xyz": (0.0, -0.084, 0.0), "rpy": (math.pi / 2, 0.0, 0.0)},
    # joint6output_to_joint6: link5 → link6
    {"xyz": (0.0, 0.06635, 0.0), "rpy": (-math.pi / 2, 0.0, 0.0)},
]

# Keypoint names matching the manipulator config
KEYPOINT_NAMES = [
    "mycobot320_base",
    "mycobot320_link1",
    "mycobot320_link2",
    "mycobot320_link3",
    "mycobot320_link4",
    "mycobot320_link5",
    "mycobot320_link6",
]


def forward_kinematics(joint_angles_rad):
    """Compute world-frame 3D positions of all 7 keypoints (base + 6 links).

    Parameters
    ----------
    joint_angles_rad : array-like of shape (6,)
        Joint angles in radians [j1, j2, j3, j4, j5, j6].

    Returns
    -------
    positions : dict
        Mapping keypoint_name → (x, y, z) in world frame.
    transforms : list of np.ndarray
        List of 7 homogeneous 4×4 transforms (world → link_i).
    """
    q = np.asarray(joint_angles_rad, dtype=np.float64)
    assert q.shape == (6,), f"Expected 6 joint angles, got {q.shape}"

    # T_world_base is identity (base sits at world origin)
    T = np.eye(4, dtype=np.float64)
    positions = {KEYPOINT_NAMES[0]: tuple(T[:3, 3].tolist())}
    transforms = [T.copy()]

    for i, jp in enumerate(_JOINT_PARAMS):
        x, y, z = jp["xyz"]
        r, p, ya = jp["rpy"]
        # Fixed transform from URDF
        T_fixed = _T(x, y, z) @ _rpy(r, p, ya)
        # Revolute joint rotation about z
        T_joint = _Rz(q[i])
        T = T @ T_fixed @ T_joint
        positions[KEYPOINT_NAMES[i + 1]] = tuple(T[:3, 3].tolist())
        transforms.append(T.copy())

    return positions, transforms


def keypoints_in_camera_frame(joint_angles_rad, T_world_camera):
    """Compute keypoint 3D positions in camera frame.

    Parameters
    ----------
    joint_angles_rad : array-like of shape (6,)
    T_world_camera : np.ndarray of shape (4, 4)
        Homogeneous transform: world → camera (camera pose in world).

    Returns
    -------
    positions_wrt_cam : list of [x, y, z]
        Each keypoint position expressed in the camera coordinate frame.
    """
    positions_world, _ = forward_kinematics(joint_angles_rad)
    T_cam_world = np.linalg.inv(T_world_camera)

    positions_wrt_cam = []
    for kp_name in KEYPOINT_NAMES:
        p_world = np.array([*positions_world[kp_name], 1.0])
        p_cam = T_cam_world @ p_world
        positions_wrt_cam.append(p_cam[:3].tolist())

    return positions_wrt_cam


def project_keypoints(positions_wrt_cam, camera_K):
    """Project 3D keypoints in camera frame to 2D pixel coordinates.

    Parameters
    ----------
    positions_wrt_cam : list of [x, y, z]
    camera_K : np.ndarray of shape (3, 3)

    Returns
    -------
    projections : list of [u, v]  (pixel coordinates, float)
    """
    projections = []
    for p in positions_wrt_cam:
        p3 = np.array(p, dtype=np.float64)
        if p3[2] <= 0:
            # Behind camera — mark as invalid
            projections.append([float('nan'), float('nan')])
            continue
        px = camera_K @ p3
        u = px[0] / px[2]
        v = px[1] / px[2]
        projections.append([float(u), float(v)])
    return projections


# --------------- camera extrinsics from Gazebo URDF ---------------

def _camera_transform_from_urdf(xyz, rpy):
    """Build the 4×4 world→camera_link transform from URDF joint origin.

    Note: Gazebo camera convention has z forward, x right, y down.
    The URDF joint places the camera_link, then Gazebo applies the
    standard camera optical frame rotation internally.
    We return T_world→optical = T_world→link @ T_link→optical
    where T_link→optical rotates to (x-right, y-down, z-forward).
    """
    x, y, z_ = xyz
    r, p, ya = rpy
    T_world_link = _T(x, y, z_) @ _rpy(r, p, ya)

    # Gazebo camera sensor looks along +x of the link frame.
    # Optical frame convention (OpenCV): z=forward, x=right, y=down.
    # In the link frame:
    #   optical x-axis = -link_y  (right = -left)
    #   optical y-axis = -link_z  (down  = -up)
    #   optical z-axis = +link_x  (forward)
    # Rotation columns = [opt_x_in_link | opt_y_in_link | opt_z_in_link]
    T_link_optical = np.array([
        [ 0,  0,  1, 0],
        [-1,  0,  0, 0],
        [ 0, -1,  0, 0],
        [ 0,  0,  0, 1],
    ], dtype=np.float64)

    return T_world_link @ T_link_optical


# Camera poses from the Gazebo URDF (mycobot_pro_320_pi_gazebo.urdf)
GAZEBO_CAMERAS = {
    "front": {
        "xyz": (0.8, 0.0, 0.4),
        "rpy": (0.0, 0.3, 3.1416),
    },
    "right": {
        "xyz": (0.0, 0.8, 0.4),
        "rpy": (0.0, 0.3, -1.5708),
    },
    "left": {
        "xyz": (0.0, -0.8, 0.4),
        "rpy": (0.0, 0.3, 1.5708),
    },
    "top": {
        "xyz": (0.0, 0.0, 1.2),
        "rpy": (0.0, 1.5708, 0.0),
    },
}

# Gazebo camera intrinsics (all cameras identical):
# horizontal_fov=1.047 rad, image 640×480
# fx = fy = (width/2) / tan(hfov/2)
_HFOV = 1.047
_W, _H = 640, 480
_FX = (_W / 2.0) / math.tan(_HFOV / 2.0)
_FY = _FX  # square pixels
_CX = _W / 2.0
_CY = _H / 2.0

GAZEBO_INTRINSICS = np.array([
    [_FX, 0.0, _CX],
    [0.0, _FY, _CY],
    [0.0, 0.0, 1.0],
], dtype=np.float64)


def get_camera_transform(camera_name):
    """Return T_world→optical for a named Gazebo camera."""
    cam = GAZEBO_CAMERAS[camera_name]
    return _camera_transform_from_urdf(cam["xyz"], cam["rpy"])


if __name__ == "__main__":
    # Quick sanity check: print keypoint positions at home pose (all zeros)
    print("=== FK at home pose (all joints = 0) ===")
    positions, _ = forward_kinematics([0.0] * 6)
    for name, pos in positions.items():
        print(f"  {name}: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

    # Check projection for front camera
    print("\n=== Projections from front camera ===")
    T_cam = get_camera_transform("front")
    kp_cam = keypoints_in_camera_frame([0.0] * 6, T_cam)
    projs = project_keypoints(kp_cam, GAZEBO_INTRINSICS)
    for name, p_cam, proj in zip(KEYPOINT_NAMES, kp_cam, projs):
        print(f"  {name}: 3D_cam=({p_cam[0]:.4f}, {p_cam[1]:.4f}, {p_cam[2]:.4f})  "
              f"→ px=({proj[0]:.1f}, {proj[1]:.1f})")
