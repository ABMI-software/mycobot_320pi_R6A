"""Gripper trajectories drawn in the Gazebo window — VALIDATION ONLY (protocol, step 10).

- True path of the gripper tip (/joint_states, sim_sorting_grasp.tool_tip), one
  colour per object being sorted (from /pickplace/status: "▶ <object> : ..."
  opens the object, "(essai N)" closes it), grey in between.
- DREAM path of the current object only (erased at the next one): the same tip
  placed through T_DREAM instead of the true camera pose, fused over the views
  with >= 5 keypoints (tri_dream_dashboard.DreamTips), then a running median of
  the last DREAM_SMOOTH points (a one-image jump is dropped, a lasting DREAM
  error stays), magenta.

Drawn as LINE_STRIP markers through the GUI MarkerManager (`gz service
/marker`; the gz Python bindings are not installed).
"""

from collections import deque
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .dream_fk_compare import PREFIXES
from .joint_history import JointHistory
from .tri_dream_dashboard import COLOURS, DreamTips, encoder_tip, pose_matrix
from .vision.sim_multicam_geometry import load_cameras

DREAM_ID = 1000
MIN_STEP_M = 0.003
MAX_POINTS = 1500
DREAM_SMOOTH = 5
MAX_JOINT_GAP_S = 0.2
# A DREAM tip farther than this from the base is a wild PnP, not a pose the arm
# can reach (~0.4 m with the gripper): physical bound, no ground truth used.
REACH_BOUND_M = 1.0


def marker_request(marker_id, colour, points):
    r, g, b = colour
    rgba = f'r: {r} g: {g} b: {b} a: 1'
    pts = ' '.join(f'point {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }}' for x, y, z in points)
    return (f'ns: "tri" id: {marker_id} action: ADD_MODIFY type: LINE_STRIP '
            f'material {{ ambient {{ {rgba} }} diffuse {{ {rgba} }} emissive {{ {rgba} }} }} {pts}')


def thin(points, n=MAX_POINTS):
    return points if len(points) <= n else points[::int(np.ceil(len(points) / n))]


class Trajectoires(Node):
    def __init__(self):
        super().__init__('tri_trajectoires_gazebo')
        desc = get_package_share_directory('mycobot_description')
        cameras = load_cameras(
            os.path.join(desc, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf'),
            {'camera_layout': 'dream50k'})
        self.T_gt = {cam: np.linalg.inv(cameras[cam].world_from_optical) for cam in PREFIXES}
        self.joints = JointHistory()
        self.current = 'transit'
        self.segments = [('transit', [])]
        self.dream_path = []
        self.dream_raw = deque(maxlen=DREAM_SMOOTH)
        self.tips = DreamTips(self.T_gt)
        self.dirty = set()
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.create_subscription(String, '/pickplace/status', self.on_status, 50)
        for cam in PREFIXES:
            self.create_subscription(Float64MultiArray, f'{PREFIXES[cam]}/keypoints',
                                     lambda m, c=cam: self.tips.keypoints(c, m), 10)
            self.create_subscription(PoseStamped, f'{PREFIXES[cam]}/pose',
                                     lambda m, c=cam: self.on_pose(c, m), 10)
        self.create_timer(1.0, self.publish)
        self.get_logger().info('trajectoires vraie (une couleur par objet) et DREAM (magenta) '
                               '-> /marker')

    def on_status(self, msg):
        text = msg.data.strip()
        if text.startswith('▶ '):
            name = text[2:].split(' ')[0]
            self.current = name if name in COLOURS else 'transit'
        elif '(essai' in text:
            self.current = 'transit'
        else:
            return
        if self.segments[-1][0] != self.current:
            last = self.segments[-1][1][-1:]
            self.segments.append((self.current, list(last)))
            if self.current != 'transit':
                self.dream_path, self.dream_raw = [], deque(maxlen=DREAM_SMOOTH)

    def on_joints(self, msg):
        self.joints.add(msg)
        if not self.joints.qs:
            return
        tip = encoder_tip(self.joints.qs[-1])
        points = self.segments[-1][1]
        if not points or np.linalg.norm(tip - points[-1]) >= MIN_STEP_M:
            points.append(tip)
            self.dirty.add(len(self.segments) - 1)

    def on_pose(self, cam, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q, gap = self.joints.at(t)
        if gap > MAX_JOINT_GAP_S:
            return
        _, est = self.tips.pose(cam, t, pose_matrix(msg), q)
        if est is None:
            return
        fused, _ = self.tips.fused(t)
        self.dream_raw.append(fused)
        point = np.median(self.dream_raw, axis=0)
        if not self.dream_path or np.linalg.norm(point - self.dream_path[-1]) >= MIN_STEP_M:
            self.dream_path.append(point)
            self.dirty.add(DREAM_ID)

    def publish(self):
        for key in sorted(self.dirty):
            if key == DREAM_ID:
                colour, points = COLOURS['dream'], self.dream_path
            else:
                name, points = self.segments[key]
                colour = COLOURS[name]
            if len(points) < 2:
                continue
            subprocess.Popen(['gz', 'service', '-s', '/marker', '--reqtype', 'gz.msgs.Marker',
                              '--reptype', 'gz.msgs.Empty', '--timeout', '500',
                              '--req', marker_request(key, colour, thin(points))],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.dirty.clear()


def main():
    rclpy.init()
    node = Trajectoires()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
