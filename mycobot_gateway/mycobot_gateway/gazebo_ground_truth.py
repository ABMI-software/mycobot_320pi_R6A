"""Gazebo ground truth of the sorting pieces — VALIDATION ONLY.

Publishes where Gazebo says each object and bin is, in the robot base frame.
No perception or planning node may subscribe to /validation/gt/* (protocol
docs/PROTOCOLE_YOLO_GAZEBO.md, invariant I4; guarded by tests/test_gt_isolation.py).

    gz topic /world/<w>/pose/info -> /validation/gt/objects
                                     (vision_msgs/Detection3DArray, frame base)

Read with `gz topic -e`, not ros_gz_bridge: the Pose_V -> TFMessage bridge
drops the model names (child_frame_id empty, measured 29/09), and the gz
Python bindings are not installed.
"""

import os
import re
import subprocess

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .vision import tri_scene


def _fields(block, keys, default=0.0):
    """gz prints protobuf text and omits every field equal to zero."""
    values = dict(re.findall(r'\b([a-z]+): ([-\d.e+]+)', block or ''))
    return [float(values.get(k, default)) for k in keys]


def _section(text, name):
    m = re.search(name + r' \{([^{}]*)\}', text)
    return m.group(1) if m else ''


def parse_pose_info(text, names):
    """(sim stamp seconds, {model: 4x4 pose in world}) from one `gz topic -e` Pose_V."""
    stamp = _section(text.split('pose {')[0], 'stamp')
    sec, nsec = _fields(stamp, ('sec', 'nsec'))
    poses = {}
    for block in text.split('pose {')[1:]:
        name = re.search(r'name: "([^"]+)"', block)
        if not name or name.group(1) not in names:
            continue
        T = np.eye(4)
        T[:3, 3] = _fields(_section(block, 'position'), 'xyz')
        orientation = _section(block, 'orientation')
        T[:3, :3] = Rotation.from_quat(_fields(orientation, 'xyzw') if orientation
                                       else [0.0, 0.0, 0.0, 1.0]).as_matrix()
        poses[name.group(1)] = T
    return sec + nsec * 1e-9, poses


def read_pose_info(world, names, timeout=10.0):
    out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{world}/pose/info'],
                         capture_output=True, text=True, timeout=timeout).stdout
    return parse_pose_info(out, names)


def _matrix(translation, rotation):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
    T[:3, 3] = [translation.x, translation.y, translation.z]
    return T


class GazeboGroundTruth(Node):
    def __init__(self):
        super().__init__('gazebo_ground_truth')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('world_name', 'tri_yolo')
        self.declare_parameter('rate', 2.0)
        self.world = str(self.get_parameter('world_name').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        models = os.path.join(get_package_share_directory('mycobot_description'), 'models')
        self.footprints = tri_scene.load_footprints(models)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(Detection3DArray, '/validation/gt/objects', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.publish)
        self.get_logger().info('Gazebo ground truth -> /validation/gt/objects (VALIDATION ONLY)')

    def publish(self):
        try:
            stamp, poses = read_pose_info(self.world, self.footprints)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'no /world/{self.world}/pose/info', throttle_duration_sec=5.0)
            return
        if not poses:
            return
        try:
            t = self.tf.lookup_transform(self.base_frame, self.world_frame, Time()).transform
        except Exception as exc:  # noqa: BLE001 — TF not up yet at launch
            self.get_logger().warn(f'no {self.world_frame}->{self.base_frame} TF yet: {exc}',
                                   throttle_duration_sec=5.0)
            return
        base_from_world = _matrix(t.translation, t.rotation)
        out = Detection3DArray()
        out.header.stamp.sec = int(stamp)
        out.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
        out.header.frame_id = self.base_frame
        for name, world_pose in sorted(poses.items()):
            pose = base_from_world @ world_pose
            footprint = self.footprints[name]
            quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
            det = Detection3D()
            det.header = out.header
            det.id = name
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = 1.0
            # Model origin: centre of the base, on the table.
            p = hyp.pose.pose
            p.position.x, p.position.y, p.position.z = pose[:3, 3]
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
            det.results.append(hyp)
            centre = pose[:3, 3] + pose[:3, :3] @ [0.0, 0.0, footprint.height / 2]
            b = det.bbox.center
            b.position.x, b.position.y, b.position.z = centre
            b.orientation = p.orientation
            det.bbox.size.x, det.bbox.size.y = 2 * footprint.half
            det.bbox.size.z = footprint.height
            out.detections.append(det)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GazeboGroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
