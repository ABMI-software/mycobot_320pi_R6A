#!/usr/bin/env python3
"""One-off: decode one real camera frame both ways and save both, so the
color order can be checked by looking at the image rather than reasoning
about it from the format string alone."""
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = "/workspace/datasets/bags/1789171382_864718428"
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

msg = None
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == "/synth_camera/image/compressed":
        msg = deserialize_message(data, get_message(type_map[topic]))
        break

print("format field:", repr(msg.format))
arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
img_as_decoded = cv2.imdecode(arr, cv2.IMREAD_COLOR)  # cv2's own BGR convention
print("decoded shape:", img_as_decoded.shape, "dtype:", img_as_decoded.dtype)
img_swapped = cv2.cvtColor(img_as_decoded, cv2.COLOR_BGR2RGB)

# Use PIL, not cv2.imwrite, to save both candidates: PIL.Image.fromarray
# writes the array's channels out AS-IS with no implicit BGR assumption,
# so whichever PNG looks visually correct tells us, unambiguously, which
# in-memory array is genuinely RGB-ordered -- no reasoning about ROS/cv2
# conventions required, just looking at the result.
from PIL import Image
Image.fromarray(img_as_decoded).save("/workspace/rlds_extraction_out/sample_as_decoded.png")
Image.fromarray(img_swapped).save("/workspace/rlds_extraction_out/sample_swapped.png")
print("wrote sample_as_decoded.png (raw cv2.imdecode order) and sample_swapped.png (cvtColor'd)")
