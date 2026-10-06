"""Image geometry regressions; runnable without starting ROS or Gazebo."""

from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mycobot_gateway'))
from mycobot_gateway.vision.sim_multicam_geometry import (  # noqa: E402
    BIN_XY, box_vertices, fit_cube, load_cameras, projected_box,
    red_candidates, sample_cube_xy, sample_scene_xy,
)


class MulticamGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cameras = load_cameras(
            ROOT / 'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf')

    def test_optical_axes_from_world_mounts(self):
        top = self.cameras['synth_camera_top']
        # Top camera: image-right is world -Y, image-down is world -X.
        center, forward, left = top.project([[0, 0, 0], [.1, 0, 0], [0, .1, 0]])
        np.testing.assert_allclose(center, [320, 240], atol=.01)
        self.assertLess(forward[1], center[1])
        self.assertLess(left[0], center[0])
        for camera in self.cameras.values():
            T = camera.world_from_optical
            np.testing.assert_allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(T[:3, :3]), 1.)

    def test_pixel_quantization_and_one_occluded_camera(self):
        rng = np.random.default_rng(47)
        for _ in range(12):
            xy = sample_cube_xy(rng)
            detections = {n: [np.rint(projected_box(c, xy))]
                          for n, c in self.cameras.items()}
            detections['synth_camera_right'][0] += [15, -20, 20, -15]
            result = fit_cube(self.cameras, detections)
            self.assertIsNotNone(result)
            xyz, views, _ = result
            self.assertNotIn('synth_camera_right', views)
            self.assertGreaterEqual(len(views), 3)
            self.assertLess(np.linalg.norm(xyz[:2] - xy), .002)

    def test_single_view_cannot_command_a_grasp(self):
        name = 'synth_camera_top'
        detections = {name: [projected_box(self.cameras[name], [.22, -.08])]}
        self.assertIsNone(fit_cube(self.cameras, detections))

    def test_red_bin_is_not_the_cube(self):
        detections = {}
        xy = [.22, -.08]
        for name, camera in self.cameras.items():
            image = np.zeros((camera.height, camera.width, 3), dtype=np.uint8)
            # Rasterize both red silhouettes into an actual RGB image.
            for center, size in (([*BIN_XY, .015], [.10, .10, .03]),
                                 ([*xy, .02], [.04, .04, .04])):
                uv = camera.project(box_vertices(center, size))
                hull = cv2.convexHull(np.rint(uv).astype(np.int32))
                cv2.fillConvexPoly(image, hull, (25, 25, 240))
            detections[name], _ = red_candidates(image, camera)
            self.assertLessEqual(len(detections[name]), 1)
        result = fit_cube(self.cameras, detections)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(result[0][:2] - xy), .003)

    def test_random_positions_are_reproducible_clear_and_reachable(self):
        a = np.random.default_rng(9)
        b = np.random.default_rng(9)
        for _ in range(1000):
            xy = sample_cube_xy(a)
            np.testing.assert_array_equal(xy, sample_cube_xy(b))
            self.assertLessEqual(np.linalg.norm(xy), .285)
            self.assertGreaterEqual(np.linalg.norm(xy), .19)
            self.assertGreaterEqual(np.linalg.norm(xy - BIN_XY), .135)
            self.assertTrue(.14 <= xy[0] <= .285 and -.145 <= xy[1] <= .165)

    def test_both_objects_move_and_remain_separated_on_board(self):
        rng = np.random.default_rng(34)
        bins = []
        for _ in range(1000):
            cube, bin_xy = sample_scene_xy(rng)
            bins.append(bin_xy)
            self.assertGreaterEqual(np.linalg.norm(cube - bin_xy), .16)
            self.assertLessEqual(np.linalg.norm(cube), .285)
            self.assertLessEqual(np.linalg.norm(bin_xy), .28)
            self.assertGreaterEqual(np.linalg.norm(bin_xy), .20)
            # Entire bin stays inside the measured board, with margin.
            self.assertTrue(np.all(bin_xy - .05 > [-.05, -.2075]))
            self.assertTrue(np.all(bin_xy + .05 < [.572, .2415]))
        self.assertGreater(np.ptp(bins, axis=0).min(), .08)

    def test_relocated_bin_does_not_hide_cube_at_previous_bin_position(self):
        xy = BIN_XY.copy()
        bin_xy = np.array([.22, -.10])
        detections = {}
        for name, camera in self.cameras.items():
            image = np.zeros((camera.height, camera.width, 3), dtype=np.uint8)
            for center, size in (([*bin_xy, .015], [.10, .10, .03]),
                                 ([*xy, .02], [.04, .04, .04])):
                uv = camera.project(box_vertices(center, size))
                cv2.fillConvexPoly(image, cv2.convexHull(np.rint(uv).astype(np.int32)),
                                  (25, 25, 240))
            detections[name], _ = red_candidates(image, camera, bin_xy)
            self.assertLessEqual(len(detections[name]), 1)
        result = fit_cube(self.cameras, detections, bin_xy=bin_xy)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(result[0][:2] - xy), .003)


if __name__ == '__main__':
    unittest.main()
