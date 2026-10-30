import sys
from collections import defaultdict
import numpy as np
import rclpy.serialization, rosbag2_py
from sensor_msgs.msg import Image
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
stamps = defaultdict(list)
while r.has_next():
    topic, data, t = r.read_next()
    if topic.endswith('/image'):
        m = rclpy.serialization.deserialize_message(data, Image)
        stamps[topic].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
for topic, s in sorted(stamps.items()):
    s = np.array(sorted(s)); d = np.diff(s)
    span = s[-1] - s[0]
    print(f"{topic:28s} n={len(s):4d} span={span:5.1f}s expected~{span*10+1:4.0f}  "
          f"interval median={np.median(d):.2f}s max={d.max():.2f}s  gaps>0.15s: {np.sum(d>0.15)}  "
          f"distinct stamps={len(set(np.round(s,3)))}")
