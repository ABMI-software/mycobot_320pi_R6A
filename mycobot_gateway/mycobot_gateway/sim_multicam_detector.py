"""Four-camera image-only localization for the real_table Gazebo demo."""

import json
import os
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .vision.sim_multicam_geometry import load_cameras, red_candidates, fit_cube


class SimMulticamDetector(Node):
    def __init__(self):
        super().__init__('sim_multicam_detector')
        self.declare_parameter('min_views', 2)
        self.declare_parameter('max_reprojection_px', 3.0)
        self.declare_parameter('bin_xy', [.22, .10])
        self.declare_parameter('camera_layout', 'legacy')
        self.bin_xy = np.asarray(self.get_parameter('bin_xy').value, dtype=float)
        if self.bin_xy.shape != (2,) or not np.isfinite(self.bin_xy).all():
            raise ValueError('bin_xy must contain two finite coordinates')
        self.min_views = int(self.get_parameter('min_views').value)
        if not 2 <= self.min_views <= 4:
            raise ValueError('min_views must be between 2 and 4')
        self.max_error = float(self.get_parameter('max_reprojection_px').value)
        if not 0 < self.max_error <= 10:
            raise ValueError('max_reprojection_px must be in (0, 10]')
        urdf = os.path.join(get_package_share_directory('mycobot_description'),
                            'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf')
        self.cameras = load_cameras(
            urdf, {'camera_layout': str(self.get_parameter('camera_layout').value)})
        self.images = {}
        self.info_ready = set()
        self.last_processed = None
        self.bridge = CvBridge()
        self.pub_pose = self.create_publisher(PoseStamped, '/vision/red_cube/pose', 10)
        self.pub_status = self.create_publisher(String, '/vision/multicam/status', 10)
        self.pub_debug = self.create_publisher(Image, '/vision/multicam/debug_image', 2)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_calibration = self.create_publisher(String, '/vision/multicam/calibration', qos)
        self.create_subscription(PointStamped, '/real_table/bin_position', self.on_bin, qos)
        for name in self.cameras:
            self.create_subscription(Image, f'/{name}/image',
                                     lambda msg, n=name: self.on_image(n, msg),
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'/{name}/camera_info',
                                     lambda msg, n=name: self.on_info(n, msg),
                                     qos_profile_sensor_data)
        self.create_timer(.25, self.tick)
        self.get_logger().info('Four-camera extrinsics: Gazebo URDF mounts → robot base; '
                               'waiting for images and CameraInfo from all four cameras')

    def on_image(self, name, msg):
        self.images[name] = (msg, time.monotonic())

    def on_bin(self, msg):
        xy = np.array([msg.point.x, msg.point.y])
        if msg.header.frame_id != 'world' or not np.isfinite(xy).all():
            return
        self.bin_xy = xy
        self.images.clear()
        self.last_processed = None

    def on_info(self, name, msg):
        camera = self.cameras[name]
        K = np.array(msg.k).reshape(3, 3)
        if (msg.width != camera.width or msg.height != camera.height
                or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0):
            self.get_logger().error(f'{name}: invalid CameraInfo')
            self.info_ready.discard(name)
            return
        if name in self.info_ready and np.array_equal(camera.K, K):
            return
        camera.K = K
        self.info_ready.add(name)
        if len(self.info_ready) == 4:
            self.pub_calibration.publish(String(data=json.dumps({
                name: {'source': 'gazebo_urdf', 'K': c.K.tolist(),
                       'T_base_optical': c.world_from_optical.tolist()}
                for name, c in self.cameras.items()})))

    def tick(self):
        now = time.monotonic()
        if len(self.info_ready) != 4 or len(self.images) != 4:
            self.pub_status.publish(String(data='WAITING_FOR_FOUR_CAMERAS'))
            return
        active = {n: msg for n, (msg, arrival) in self.images.items()
                  if now - arrival < 1.5}
        if len(active) < self.min_views:
            self.pub_status.publish(String(data='STALE_IMAGES'))
            return
        stamps = {n: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                  for n, m in active.items()}
        newest = max(stamps.values())
        active = {n: m for n, m in active.items() if newest - stamps[n] <= .15}
        signature = tuple((n, stamps[n]) for n in active)
        if signature == self.last_processed:
            return
        self.last_processed = signature
        if len(active) < self.min_views:
            self.pub_status.publish(String(data='UNSYNCHRONIZED_IMAGES'))
            return
        detections, panels = {}, []
        for name, msg in active.items():
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
            detections[name], _ = red_candidates(image, self.cameras[name], self.bin_xy)
            for box in detections[name]:
                x, y, xx, yy = np.rint(box).astype(int)
                cv2.rectangle(image, (x, y), (xx, yy), (0, 255, 0), 2)
            cv2.putText(image, name, (12, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        .6, (255, 255, 255), 2)
            panels.append(image)
        result = fit_cube(self.cameras, detections, self.min_views, self.max_error, self.bin_xy)
        status = {'state': 'NO_CONSENSUS', 'candidates': {n: len(d) for n, d in detections.items()}}
        if result is not None:
            xyz, views, error = result
            pose = PoseStamped()
            # Keep the observation timestamp: stale input must never look fresh.
            pose.header.stamp = min((active[n].header.stamp for n in views),
                                    key=lambda s: s.sec + s.nanosec * 1e-9)
            pose.header.frame_id = 'world'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
            pose.pose.orientation.w = 1.
            self.pub_pose.publish(pose)
            status.update(state='OK', xyz_m=xyz.tolist(), views=views, reprojection_px=error)
        self.pub_status.publish(String(data=json.dumps(status)))
        if self.pub_debug.get_subscription_count():
            while len(panels) < 4:
                panels.append(np.zeros_like(panels[0]))
            mosaic = np.vstack([np.hstack(panels[:2]), np.hstack(panels[2:4])])
            debug = self.bridge.cv2_to_imgmsg(mosaic, 'bgr8')
            debug.header.stamp = active[next(iter(active))].header.stamp
            self.pub_debug.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = SimMulticamDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
