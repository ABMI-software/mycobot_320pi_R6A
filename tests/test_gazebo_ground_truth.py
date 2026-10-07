"""Parsing of `gz topic -e /world/<w>/pose/info`, which omits zero fields."""

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mycobot_gateway'))
from mycobot_gateway.gazebo_ground_truth import parse_pose_info  # noqa: E402
from mycobot_gateway.vision import tri_scene as ts  # noqa: E402

# Recorded from `ros2 launch mycobot_gateway tri_yolo.launch.py seed:=1`, 29/09/2026.
SAMPLE = ROOT / 'tests/data/pose_info_tri_yolo_seed1.txt'
PIECES = ts.OBJECTS + ts.BINS


class PoseInfoParsingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stamp, cls.poses = parse_pose_info(SAMPLE.read_text(), PIECES)

    def test_all_eight_pieces_and_only_them(self):
        self.assertEqual(set(self.poses), set(PIECES))

    def test_sim_stamp_from_header(self):
        self.assertAlmostEqual(self.stamp, 14.983, places=6)

    def test_positions_with_omitted_zero_fields(self):
        np.testing.assert_allclose(self.poses['cube_rouge'][:3, 3], [0.171505, 0.131495, 0], atol=1e-6)
        # Static bins at z = 0: gz prints no z line at all.
        self.assertEqual(self.poses['bac_bleu'][2, 3], 0.0)

    def test_orientations_are_rotations(self):
        for T in self.poses.values():
            np.testing.assert_allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-9)
            np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-6)

    def test_missing_orientation_is_identity(self):
        _, poses = parse_pose_info('pose {\n  name: "bac_rouge"\n  position {\n    x: 0.2\n  }\n}\n',
                                   PIECES)
        np.testing.assert_array_equal(poses['bac_rouge'][:3, :3], np.eye(3))
        np.testing.assert_array_equal(poses['bac_rouge'][:3, 3], [0.2, 0, 0])


if __name__ == '__main__':
    unittest.main()
