import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class OpencvCamera(Node):
    def __init__(self):
        super().__init__('opencv_camera')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Try to find the camera automatically
        self.cap = self.find_camera()
        
        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info("Success: Camera initialized and streaming.")
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            self.get_logger().error("CRITICAL: No valid camera found on indices 0-5. Check USB cable!")

    def find_camera(self):
        for index in range(6):
            self.get_logger().info(f"Testing camera index {index}...")
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    self.get_logger().info(f"Camera found at index {index}")
                    return cap
                cap.release()
        return None

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Dropped frame...")

def main(args=None):
    rclpy.init(args=args)
    node = OpencvCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()