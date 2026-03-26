import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped

class ImageConverter(Node):
    def __init__(self):
        super().__init__("detect_marker")
        self.br = TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        # Compatibility fix for OpenCV ArUco
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            self.aruo_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.aruo_params = cv2.aruco.DetectorParameters_create()

        self.dist_coeffs = np.zeros((4,1)) 
        self.camera_matrix = None

        self.image_sub = self.create_subscription(
            Image,
            "camera/image_raw", 
            self.callback,
            10
        )
        self.get_logger().info("Marker Detector Node Started - ArUco initialized")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        if self.camera_matrix is None:
            size = gray.shape
            focal_length = size[1]
            center = (size[1] / 2, size[0] / 2)
            self.camera_matrix = np.array([
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1]
            ], dtype=np.float32)

        # Detection logic
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruo_params)

        if ids is not None:
            # Marker size is 0.05 meters (5cm)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                rvec = rvecs[i]
                tvec = tvecs[i]
                
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                
                # TF broadcast
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera_link"
                t.child_frame_id = "basic_shapes"
                
                # Using flatten to ensure we get a 1D array
                t_flat = tvec[0].flatten()
                t.transform.translation.x = float(t_flat[0])
                t.transform.translation.y = float(t_flat[1])
                t.transform.translation.z = float(t_flat[2])
                
                r_flat = rvec[0].flatten()
                quat = tf_transformations.quaternion_from_euler(r_flat[0], r_flat[1], r_flat[2])
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.br.sendTransform(t)

        cv2.imshow("Detection Feed", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()