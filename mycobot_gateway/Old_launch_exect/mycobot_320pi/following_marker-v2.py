import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from visualization_msgs.msg import Marker
from pymycobot.mycobot import MyCobot

class MarkerFollower(Node):
    def __init__(self):
        super().__init__("following_marker")

        # 1. Setup TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. Setup myCobot Connection
        # Ensure this is the correct port for the Pi-based 320
        try:
            self.mc = MyCobot('/dev/ttyAMA0', 115200)
            self.get_logger().info("Connected to myCobot hardware.")
        except Exception as e:
            self.get_logger().error(f"Hardware Connection Failed: {e}")

        self.pub_marker = self.create_publisher(Marker, "visualization_marker", 10)

        # 3. Control Loop - NO WHILE LOOP INSIDE HERE
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # Look up the transform (latest available)
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("joint1", "basic_shapes", now)
            
            # --- VISUALIZATION (Optional, for RViz) ---
            marker_msg = Marker()
            marker_msg.header.frame_id = "joint1"
            marker_msg.header.stamp = self.get_clock().now().to_msg()
            marker_msg.type = Marker.CUBE
            marker_msg.pose.position.x = trans.transform.translation.x
            marker_msg.pose.position.y = trans.transform.translation.y
            marker_msg.pose.position.z = trans.transform.translation.z
            marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = 0.04, 0.04, 0.04
            marker_msg.color.a, marker_msg.color.g = 1.0, 1.0
            self.pub_marker.publish(marker_msg)

            # --- MOVEMENT LOGIC ---
            # Conversion: Meters to Millimeters
            x = trans.transform.translation.x * 1000
            y = trans.transform.translation.y * 1000
            z = trans.transform.translation.z * 1000

            # Orientation: Facing down (Adjust these angles based on your flange orientation)
            # Standard "looking down" for myCobot: rx=180, ry=0, rz=0
            coords = [x, y, z, 180.0, 0.0, 0.0]

            self.get_logger().info(f"Commanding Move: {coords}")
            
            # Send command: speed 40, mode 1 (linear movement)
            self.mc.send_coords(coords, 40, 1)

        except Exception as e:
            # This is expected if the marker isn't currently visible
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()