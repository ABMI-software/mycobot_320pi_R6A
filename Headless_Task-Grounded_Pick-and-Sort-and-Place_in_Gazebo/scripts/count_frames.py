"""Count images per camera over N simulated seconds; header stamps only."""
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
cams = ('synth_camera_top', 'synth_camera_right', 'synth_camera_left', 'synth_camera')
span = float(sys.argv[1]); rel = sys.argv[2]
rclpy.init()
n = Node('count_frames', parameter_overrides=[Parameter('use_sim_time', value=True)])
qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.RELIABLE if rel == 'reliable' else ReliabilityPolicy.BEST_EFFORT)
st = {c: set() for c in cams}
for c in cams:
    n.create_subscription(Image, f'/{c}/image', lambda m, c=c: st[c].add(m.header.stamp.sec * 10 + m.header.stamp.nanosec // 100_000_000), qos)
while rclpy.ok() and n.get_clock().now().nanoseconds == 0:
    rclpy.spin_once(n, timeout_sec=0.5)
t0 = n.get_clock().now().nanoseconds / 1e9
while rclpy.ok() and n.get_clock().now().nanoseconds / 1e9 - t0 < span:
    rclpy.spin_once(n, timeout_sec=0.5)
for c in cams:
    s = sorted(st[c]); print(f'{rel:11s} {c:20s} {len(s):3d} images over {span:.0f} sim s (expected ~{span*10:.0f})')
