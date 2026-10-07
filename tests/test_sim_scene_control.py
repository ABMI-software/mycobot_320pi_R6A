"""Prevent concurrent scene resets and picks; no Gazebo instance required."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mycobot_gateway'))
try:
    from mycobot_gateway.sim_scene_control import SimSceneControl
except ModuleNotFoundError as exc:
    if exc.name not in ('rclpy', 'geometry_msgs', 'std_srvs'):
        raise
    SimSceneControl = None


@unittest.skipIf(SimSceneControl is None, 'ROS message types required')
class SceneControlTests(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(SimSceneControl)
        self.node.busy = False
        self.node.can_pick = True
        self.node.pending = None
        self.node.publish_state = Mock()
        self.node.status = Mock()

    def request(self, action):
        return self.node.request_action(action, SimpleNamespace())

    def test_double_click_and_pick_during_randomization_are_rejected(self):
        self.assertTrue(self.request('randomize').success)
        self.assertFalse(self.request('randomize').success)
        self.assertFalse(self.request('pick').success)
        self.assertEqual(self.node.pending, 'randomize')

    def test_randomization_during_pick_is_rejected(self):
        self.assertTrue(self.request('pick').success)
        self.assertFalse(self.request('randomize').success)
        self.assertEqual(self.node.pending, 'pick')

    def test_failed_reset_disables_pick_and_allows_another_reset(self):
        self.request('randomize')
        self.node.randomize = Mock(side_effect=RuntimeError('Gazebo unavailable'))
        self.node.process_pending()
        self.assertFalse(self.node.busy)
        self.assertFalse(self.node.can_pick)
        self.assertFalse(self.request('pick').success)
        self.assertTrue(self.request('randomize').success)


if __name__ == '__main__':
    unittest.main()
