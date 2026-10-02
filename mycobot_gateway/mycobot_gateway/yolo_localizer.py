"""YOLO 2D detection -> 3D position in the robot base frame (perception only).

The XY whose projected 3D box (class dimensions from the model files: prior
knowledge of the pieces, not Gazebo poses) has its 2D box centre on the
detected one, at half the class height (`tri_scene.locate_from_box`). Camera
pose and intrinsics are those of the DREAM 50K layout, read from the URDF — no
calibration. The plain ray through the box centre missed the object by up to
7 mm on oblique views (01/10, yolo26 v6c, 10 scenes).

Every camera is localized on its own, then the cameras are fused per class: the
median of the views seen within `max_age_s` of the newest, a view farther than
`tri_scene.FUSION_GATE` from that median left out. Objects AND bins: the sorter
receives nothing else (protocol, step 8) and pairs them by class
(`tri_scene.sorting_pairs`).

    /yolo/<camera>/detections -> /yolo/<camera>/objects_3d (vision_msgs/Detection3DArray, frame base)
                              -> /yolo/objects_3d          (fused: one detection per class,
                                                            id = the cameras kept, comma-separated,
                                                            score = their share of the cameras)
"""

import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .vision import tri_scene
from .vision.sim_multicam_geometry import load_cameras


class YoloLocalizer(Node):
    def __init__(self):
        super().__init__('yolo_localizer')
        self.declare_parameter('cameras', ['synth_camera_top'])
        self.declare_parameter('camera_layout', 'dream50k')
        # Four cameras at 1.4-2 Hz each in headless real time (29/09): 2 s keeps
        # one frame of every camera without fusing a scene that has changed.
        self.declare_parameter('max_age_s', 2.0)
        desc = get_package_share_directory('mycobot_description')
        self.cameras = load_cameras(
            os.path.join(desc, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf'),
            {'camera_layout': str(self.get_parameter('camera_layout').value)})
        self.footprints = tri_scene.load_footprints(os.path.join(desc, 'models'))
        self.max_age = float(self.get_parameter('max_age_s').value)
        self.latest = {}
        self.pub = self.create_publisher(Detection3DArray, '/yolo/objects_3d', 10)
        self.pub_cam = {}
        for cam in self.get_parameter('cameras').value:
            self.pub_cam[cam] = self.create_publisher(Detection3DArray, f'/yolo/{cam}/objects_3d', 10)
            self.create_subscription(Detection2DArray, f'/yolo/{cam}/detections',
                                     lambda msg, c=cam: self.on_detections(c, msg), 10)

    def detection(self, header, ident, hypothesis, centre, footprint):
        d3 = Detection3D()
        d3.header = header
        d3.id = ident
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = hypothesis
        p = hyp.pose.pose
        p.position.x, p.position.y, p.position.z = (float(v) for v in centre)
        p.orientation.w = 1.0
        d3.results.append(hyp)
        d3.bbox.center = p
        d3.bbox.size.x, d3.bbox.size.y = (float(v) for v in 2 * footprint.half)
        d3.bbox.size.z = footprint.height
        return d3

    def on_detections(self, cam, msg):
        camera = self.cameras[cam]
        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base'
        seen = []
        for det in msg.detections:
            hypothesis = det.results[0].hypothesis
            footprint = self.footprints[hypothesis.class_id]
            uv = (det.bbox.center.position.x, det.bbox.center.position.y)
            try:
                centre = tri_scene.locate_from_box(camera, uv, footprint)
            except ValueError:
                continue
            seen.append((hypothesis.class_id, centre, hypothesis.score))
            out.detections.append(self.detection(out.header, cam, hypothesis, centre, footprint))
        self.pub_cam[cam].publish(out)
        self.latest[cam] = (Time.from_msg(msg.header.stamp), tri_scene.best_per_class(seen))
        self.publish_fused(out.header)

    def publish_fused(self, header):
        newest = max(t for t, _ in self.latest.values())
        recent = {cam: seen for cam, (t, seen) in self.latest.items()
                  if (newest - t).nanoseconds <= self.max_age * 1e9}
        out = Detection3DArray()
        out.header = header
        for name, (centre, kept) in sorted(tri_scene.fuse_cameras(recent).items()):
            hypothesis = ObjectHypothesisWithPose().hypothesis
            hypothesis.class_id = name
            hypothesis.score = len(kept) / len(self.pub_cam)
            out.detections.append(self.detection(header, ','.join(kept), hypothesis, centre,
                                                 self.footprints[name]))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = YoloLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
