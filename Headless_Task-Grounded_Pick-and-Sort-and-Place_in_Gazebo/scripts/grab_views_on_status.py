"""Save the latest frame of each of the 4 tri_yolo cameras when the sort reaches
grasp, lift and drop (triggered by /pickplace/status), then a labelled 4x3 sheet."""
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

CAMERAS = ('synth_camera', 'synth_camera_right', 'synth_camera_left', 'synth_camera_top')
MOMENTS = {'serrage': 'grasp', 'tenu a': 'held', 'doigts ecartes': 'drop'}


class Grabber(Node):
    def __init__(self, out):
        super().__init__('grab_views_on_status')
        self.out, self.bridge, self.latest, self.shots = out, CvBridge(), {}, {}
        for cam in CAMERAS:
            self.create_subscription(Image, f'/{cam}/image',
                                     lambda m, c=cam: self.latest.__setitem__(c, m),
                                     qos_profile_sensor_data)
        self.create_subscription(String, '/pickplace/status', self.on_status, 10)

    def on_status(self, msg):
        for key, moment in MOMENTS.items():
            if key in msg.data and moment not in self.shots and len(self.latest) == len(CAMERAS):
                self.shots[moment] = {c: self.bridge.imgmsg_to_cv2(m, 'bgr8')
                                      for c, m in self.latest.items()}
                for c, img in self.shots[moment].items():
                    cv2.imwrite(str(self.out / f'{moment}_{c}.png'), img)
                self.get_logger().info(f'saved {moment}')

    def sheet(self):
        rows = []
        for moment in MOMENTS.values():
            if moment not in self.shots:
                continue
            tiles = []
            for c in CAMERAS:
                img = cv2.resize(self.shots[moment][c], (480, 360))
                cv2.putText(img, f'{moment} / {c}', (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)
                tiles.append(img)
            rows.append(np.hstack(tiles))
        if rows:
            cv2.imwrite(str(self.out / 'views_sheet.png'), np.vstack(rows))


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Grabber(out)
    try:
        while rclpy.ok() and len(node.shots) < len(MOMENTS):
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.sheet()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
