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

    def test_tip_is_the_sorter_tip(self):
        from mycobot_gateway.sim_sorting_grasp import tool_tip
        from mycobot_gateway.tri_dream_dashboard import encoder_tip
        np.testing.assert_allclose(encoder_tip(self.q), tool_tip(np.degrees(self.q)))

    def test_camera_frame_offset_moves_estimate(self):
        shifted = self.T_gt.copy()
        shifted[:3, 3] += [0.0, 0.0, 0.01]
        true, est = self.flange_estimate(self.T_gt, shifted, self.q)
        self.assertAlmostEqual(np.linalg.norm(est - true), 0.01)


class DreamTipsTests(unittest.TestCase):
    def setUp(self):
        from mycobot_gateway.tri_dream_dashboard import DreamTips
        cameras = load_cameras(URDF, {'camera_layout': 'dream50k'})
        self.T_gt = {c: np.linalg.inv(cameras[c].world_from_optical) for c in FK_NAMES}
        self.tips = DreamTips(self.T_gt)
        self.q = np.radians([20, 30, -60, 0, -30, 0])

    def keypoints(self, cam, t, n):
        msg = type('M', (), {})()
        sec, nsec = int(t), round((t - int(t)) * 1e9)
        msg.data = [0.0, 0.0, 1.0] * n + [0.0, 0.0, 0.0] * (7 - n) + [sec, nsec, 100.0]
        self.tips.keypoints(cam, msg)

    def test_four_keypoint_view_is_not_fused(self):
        self.keypoints('synth_camera', 1.5, 4)
        _, est = self.tips.pose('synth_camera', 1.5, self.T_gt['synth_camera'], self.q)
        self.assertIsNone(est)
        self.assertEqual(self.tips.fused(1.5)[1], 0)

    def test_fused_is_median_of_usable_views(self):
        for cam, dz in (('synth_camera', 0.0), ('synth_camera_left', 0.0),
                        ('synth_camera_top', 0.3)):
            self.keypoints(cam, 2.0, 7)
            T = self.T_gt[cam].copy()
            T[:3, 3] += [0, 0, dz]
            self.tips.pose(cam, 2.0, T, self.q)
        true, _ = self.tips.pose('synth_camera_right', 2.0, self.T_gt['synth_camera_right'], self.q)
        est, views = self.tips.fused(2.0)
        self.assertEqual(views, 3)
        np.testing.assert_allclose(est, true, atol=1e-9)  # two exact views out of three


if __name__ == '__main__':
    unittest.main()
