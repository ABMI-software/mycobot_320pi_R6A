"""DREAM vs FK comparator: joint interpolation, pose error, and the camera geometry."""

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mycobot_gateway'))
from mycobot_gateway import dream_fk_compare as c  # noqa: E402
from mycobot_gateway.joint_history import interpolate  # noqa: E402
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras  # noqa: E402
import mycobot_fk  # noqa: E402  (path added by dream_fk_compare)

URDF = ROOT / 'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf'
FK_NAMES = {'synth_camera': 'front', 'synth_camera_right': 'right',
            'synth_camera_left': 'left', 'synth_camera_top': 'top'}


class CompareTests(unittest.TestCase):
    def test_interpolates_between_samples(self):
        q, gap = interpolate([0.0, 1.0], [np.zeros(6), np.ones(6)], 0.25)
        np.testing.assert_allclose(q, 0.25)
        self.assertAlmostEqual(gap, 0.25)

    def test_outside_history_reports_gap(self):
        _, gap = interpolate([0.0, 1.0], [np.zeros(6), np.ones(6)], 1.5)
        self.assertAlmostEqual(gap, 0.5)

    def test_pose_error(self):
        T = np.eye(4)
        est = np.eye(4)
        est[:3, 3] = [0.003, 0.0, -0.004]
        d, angle = c.pose_error(T, est)
        self.assertAlmostEqual(np.linalg.norm(d), 0.005)
        self.assertAlmostEqual(angle, 0.0)

    def test_urdf_cameras_project_like_the_dream_labels(self):
        """FK projected with the dream50k URDF cameras = mycobot_fk (DREAM label geometry)."""
        cameras = load_cameras(URDF, {'camera_layout': 'dream50k'})
        q = np.radians([20, -30, 40, -20, -30, 10])
        positions, _ = mycobot_fk.forward_kinematics(q)
        world = np.array([positions[k] for k in mycobot_fk.KEYPOINT_NAMES])
        for cam, fk_name in FK_NAMES.items():
            ours = cameras[cam].project(world)
            labels = mycobot_fk.project_keypoints(
                mycobot_fk.keypoints_in_camera_frame(q, mycobot_fk.get_camera_transform(fk_name)),
                mycobot_fk.GAZEBO_INTRINSICS)
            np.testing.assert_allclose(ours, labels, atol=0.05, err_msg=cam)


class FlangeEstimateTests(unittest.TestCase):
    def setUp(self):
        from mycobot_gateway.tri_dream_dashboard import flange_estimate
        self.flange_estimate = flange_estimate
        self.q = np.radians([20, 30, -60, 0, -30, 0])
        self.T_gt = np.linalg.inv(load_cameras(URDF, {'camera_layout': 'dream50k'})
                                  ['synth_camera'].world_from_optical)

    def test_exact_pose_puts_flange_on_truth(self):
        true, est = self.flange_estimate(self.T_gt, self.T_gt, self.q)
        np.testing.assert_allclose(est, true, atol=1e-12)

    def test_camera_frame_offset_moves_estimate(self):
        shifted = self.T_gt.copy()
        shifted[:3, 3] += [0.0, 0.0, 0.01]
        true, est = self.flange_estimate(self.T_gt, shifted, self.q)
        self.assertAlmostEqual(np.linalg.norm(est - true), 0.01)


if __name__ == '__main__':
    unittest.main()
