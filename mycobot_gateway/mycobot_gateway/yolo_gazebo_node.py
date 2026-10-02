"""yolo26 on Gazebo camera streams: 2D detections of pieces and bins.

Same model, classes and service as the real bench (scripts/yolo26_service.py
under .venv, through yolo26_dashboard.ServiceYOLO26). Perception only: never
reads Gazebo poses (protocol invariant I4). Not connected to motion planning.

    /<camera>/image -> /yolo/<camera>/detections      (vision_msgs/Detection2DArray,
                                                        header of the source image)
                    -> /yolo/<camera>/image_annotated
"""

import csv
from pathlib import Path
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

_SCRIPTS = Path(__file__).resolve().parents[2] / 'scripts'
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import tri_couleur as tc  # noqa: E402
import yolo26_dashboard as y  # noqa: E402

CSV_FIELDS = ('camera_id', 'timestamp', 'frame_id', 'class_id', 'class_name', 'confidence',
              'x_min', 'y_min', 'x_max', 'y_max', 'center_u', 'center_v')


class YoloGazeboNode(Node):
    def __init__(self):
        super().__init__('yolo_gazebo_node')
        self.declare_parameter('cameras', ['synth_camera_top'])
        self.declare_parameter('csv_path', '')
        self.cameras = list(self.get_parameter('cameras').value)
        self.bridge = CvBridge()
        self.service = y.ServiceYOLO26()
        self.get_logger().info(f'yolo26 {self.service.poids}, threshold {self.service.seuil}, '
                               f'cameras {self.cameras}')
        # One image in flight per camera: the next result the service returns
        # for that camera is this image, so it carries this header.
        self.pending = {}
        self.last_instant = {}
        self.pub_det, self.pub_img = {}, {}
        for cam in self.cameras:
            self.pub_det[cam] = self.create_publisher(Detection2DArray, f'/yolo/{cam}/detections', 10)
            self.pub_img[cam] = self.create_publisher(Image, f'/yolo/{cam}/image_annotated', 2)
            self.create_subscription(Image, f'/{cam}/image',
                                     lambda msg, c=cam: self.on_image(c, msg),
                                     qos_profile_sensor_data)
        path = str(self.get_parameter('csv_path').value)
        self.csv = None
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = open(path, 'w', newline='')
            self.csv = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
            self.csv.writeheader()
        self.create_timer(0.02, self.poll)

    def on_image(self, cam, msg):
        if cam in self.pending:
            return
        self.pending[cam] = msg.header
        self.service.soumet(cam, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def poll(self):
        if self.service.arrete:
            self.get_logger().error('yolo26_service.py stopped')
            rclpy.try_shutdown()
            return
        for cam in self.cameras:
            result = self.service.dernier(cam)
            if result is None or cam not in self.pending:
                continue
            image, detections, instant = result
            if self.last_instant.get(cam) == instant:
                continue
            self.last_instant[cam] = instant
            self.publish(cam, self.pending.pop(cam), image, detections)

    def publish(self, cam, header, image, detections):
        out = Detection2DArray()
        out.header = header
        stamp = f'{header.stamp.sec}.{header.stamp.nanosec:09d}'
        annotated = image.copy()
        for d in detections:
            x0, y0, x1, y1 = (float(v) for v in d['boite'])
            det = Detection2D()
            det.header = header
            det.id = str(tc.CLASSES_PIECES.index(d['classe']))
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d['classe']
            hyp.hypothesis.score = float(d['conf'])
            det.results.append(hyp)
            det.bbox.center.position.x = (x0 + x1) / 2
            det.bbox.center.position.y = (y0 + y1) / 2
            det.bbox.size_x = x1 - x0
            det.bbox.size_y = y1 - y0
            out.detections.append(det)
            if self.csv:
                self.csv.writerow({
                    'camera_id': cam, 'timestamp': stamp, 'frame_id': header.frame_id,
                    'class_id': det.id, 'class_name': d['classe'],
                    'confidence': round(float(d['conf']), 4),
                    'x_min': x0, 'y_min': y0, 'x_max': x1, 'y_max': y1,
                    'center_u': (x0 + x1) / 2, 'center_v': (y0 + y1) / 2})
        # Same boxes, labels and piece count as the real bench dashboard.
        y.trace_detections(annotated, detections, cam)
        if self.csv:
            self.csv_file.flush()
        self.pub_det[cam].publish(out)
        msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        msg.header = header
        self.pub_img[cam].publish(msg)

    def destroy_node(self):
        self.service.ferme()
        if self.csv:
            self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloGazeboNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
