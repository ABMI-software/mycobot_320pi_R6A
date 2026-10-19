"""Vision/ground-truth separation and stale-measurement checks, no ROS graph."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mycobot_gateway'))
try:
    from mycobot_gateway import sim_sorting_grasp as grasp
except ModuleNotFoundError as exc:
    if exc.name not in ('rclpy', 'builtin_interfaces', 'controller_manager_msgs'):
        raise
    grasp = None


@unittest.skipIf(grasp is None, 'source /opt/ros/jazzy/setup.bash for ROS message types')
class VisionGraspTests(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(grasp.SimSortingGrasp)
        self.node.pose_source = 'vision'
        self.node.vision_pose = None
        self.node.vision_received = 0.
        self.node.status = Mock()
        self.node.object_poses = Mock(side_effect=AssertionError('ground-truth leak'))
        self.now = 100.
        self.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(self.now * 1e9)))

    def observation(self, stamp=None, frame='world', x=.22):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(
                sec=int(self.now if stamp is None else stamp), nanosec=0)),
            pose=SimpleNamespace(position=SimpleNamespace(x=x, y=-.08, z=.02)))

    def poll(self, observation):
        self.now += 1
        if observation:
            self.node._vision_cb(observation())

    def locate(self, observation):
        with patch.object(grasp.rclpy, 'ok', return_value=True), \
                patch.object(grasp.rclpy, 'spin_once',
                             side_effect=lambda *a, **k: self.poll(observation)), \
                patch.object(grasp.time, 'monotonic', side_effect=lambda: self.now):
            return self.node.target_poses()

    def test_stable_vision_never_reads_gazebo_target_pose(self):
        result = self.locate(self.observation)
        np.testing.assert_allclose(result['red_cube'], [.22, -.08, .02])
        self.node.object_poses.assert_not_called()

    def test_missing_or_stale_vision_never_falls_back_to_ground_truth(self):
        for observation in (None, lambda: self.observation(stamp=1)):
            with self.assertRaisesRegex(RuntimeError, 'visuelle recente'):
                self.locate(observation)
        self.node.object_poses.assert_not_called()

    def test_wrong_reference_frame_is_rejected(self):
        with self.assertRaises(RuntimeError):
            self.locate(lambda: self.observation(frame='camera_optical'))

    def test_unstable_position_is_rejected(self):
        with self.assertRaises(RuntimeError):
            self.locate(lambda: self.observation(x=.22 + .02 * (int(self.now) % 2)))


if __name__ == '__main__':
    unittest.main()
