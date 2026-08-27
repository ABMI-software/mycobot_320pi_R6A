import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import live_aruco_geometry as geometry  # noqa: E402


K = np.array(
    [[800.0, 0.0, 320.0], [0.0, 810.0, 240.0], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
DIST_ZERO = np.zeros(5, dtype=np.float64)
MARKERS = {
    19: np.array([0.14, 0.205, 0.0]),
    23: np.array([0.14, -0.18, 0.0]),
    25: np.array([0.49, 0.205, 0.0]),
    26: np.array([0.48, -0.18, 0.0]),
}


def synthetic_camera():
    # Caméra à z=0.8 m regardant le plan monde z=0.
    R_cam_world = np.diag([1.0, -1.0, -1.0])
    camera_world = np.array([0.30, 0.00, 0.80])
    tvec = (-R_cam_world @ camera_world).reshape(3, 1)
    rvec, _ = cv2.Rodrigues(R_cam_world)
    T_cam_world = np.eye(4)
    T_cam_world[:3, :3] = R_cam_world
    T_cam_world[:3, 3] = tvec.reshape(3)
    return rvec, tvec, T_cam_world


def projected_marker_centres(dist=DIST_ZERO):
    rvec, tvec, _ = synthetic_camera()
    ids = sorted(MARKERS)
    points = np.asarray([MARKERS[marker_id] for marker_id in ids])
    image, _ = cv2.projectPoints(points, rvec, tvec, K, dist)
    return {
        marker_id: tuple(uv)
        for marker_id, uv in zip(ids, image.reshape(-1, 2))
    }


class LoadingTests(unittest.TestCase):
    def test_real_calibration_bundle_loads_required_tags(self):
        bundle = geometry.load_geometry()
        self.assertEqual(bundle.camera.K.shape, (3, 3))
        self.assertIn(bundle.camera.dist.size, (4, 5, 8, 12, 14))
        self.assertEqual(bundle.camera.resolution, (640, 480))
        self.assertEqual(bundle.camera.dictionary_name, "DICT_4X4_1000")
        self.assertTrue(set(geometry.DEFAULT_MARKER_IDS) <= set(bundle.workspace.markers))


@unittest.skipUnless(hasattr(cv2, "aruco"), "OpenCV aruco absent")
class DetectionTests(unittest.TestCase):
    def test_synthetic_aruco_centres_and_id_filter(self):
        canvas = np.full((520, 720), 255, dtype=np.uint8)
        placements = {
            19: (40, 50),
            23: (560, 50),
            25: (40, 350),
            26: (560, 350),
            42: (300, 200),  # détecté mais volontairement filtré
        }
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        for marker_id, (x, y) in placements.items():
            marker = np.zeros((120, 120), dtype=np.uint8)
            if hasattr(cv2.aruco, "generateImageMarker"):
                marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 120)
            else:
                cv2.aruco.drawMarker(dictionary, marker_id, 120, marker, 1)
            canvas[y : y + 120, x : x + 120] = marker

        centres = geometry.detect_marker_centres(
            cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR), detect_scale=1.4
        )
        self.assertEqual(set(centres), set(geometry.DEFAULT_MARKER_IDS))
        for marker_id in geometry.DEFAULT_MARKER_IDS:
            x, y = placements[marker_id]
            np.testing.assert_allclose(centres[marker_id], (x + 59.5, y + 59.5), atol=1.0)


class PoseTests(unittest.TestCase):
    def test_ippe_recovers_physical_pose_and_both_transforms(self):
        estimate = geometry.estimate_live_extrinsic(
            projected_marker_centres(), MARKERS, K, DIST_ZERO
        )
        _, _, expected = synthetic_camera()
        np.testing.assert_allclose(estimate.T_cam_world, expected, atol=1e-6)
        np.testing.assert_allclose(
            estimate.T_world_cam @ estimate.T_cam_world, np.eye(4), atol=1e-9
        )
        self.assertLess(estimate.rms_reprojection_px, 1e-5)
        self.assertAlmostEqual(estimate.camera_position_world_m[2], 0.8, places=6)
        self.assertTrue(np.all(estimate.depths_camera_m > 0.0))

    def test_four_tags_are_required_by_default(self):
        detections = projected_marker_centres()
        del detections[26]
        with self.assertRaisesRegex(geometry.PoseEstimationError, "IDs absents"):
            geometry.estimate_live_extrinsic(detections, MARKERS, K, DIST_ZERO)

    def test_rms_threshold_rejects_inconsistent_centres(self):
        detections = projected_marker_centres()
        u, v = detections[26]
        detections[26] = (u + 18.0, v - 12.0)
        with self.assertRaisesRegex(geometry.PoseEstimationError, "RMS reprojection"):
            geometry.estimate_live_extrinsic(
                detections, MARKERS, K, DIST_ZERO, max_rms_px=1.0
            )

    def test_implausible_camera_height_is_rejected(self):
        with self.assertRaisesRegex(geometry.PoseEstimationError, "aucune solution IPPE physique"):
            geometry.estimate_live_extrinsic(
                projected_marker_centres(),
                MARKERS,
                K,
                DIST_ZERO,
                camera_z_range_m=(1.0, 2.0),
            )


class ProjectionTests(unittest.TestCase):
    def test_distorted_pixel_is_undistorted_then_intersected_with_plane(self):
        dist = np.array([0.08, -0.03, 0.001, -0.002, 0.01], dtype=np.float64)
        rvec, tvec, T_cam_world = synthetic_camera()
        expected = np.array([0.37, 0.09, 0.0335])
        pixel, _ = cv2.projectPoints(expected.reshape(1, 3), rvec, tvec, K, dist)
        recovered = geometry.deproject_pixel_to_base(
            pixel.reshape(2), K, np.linalg.inv(T_cam_world), expected[2], dist
        )
        np.testing.assert_allclose(recovered, expected, atol=1e-8)

    def test_already_undistorted_pixel_path(self):
        rvec, tvec, T_cam_world = synthetic_camera()
        expected = np.array([0.22, -0.12, 0.05])
        pixel, _ = cv2.projectPoints(expected.reshape(1, 3), rvec, tvec, K, None)
        recovered = geometry.deproject_pixel_to_base(
            pixel.reshape(2),
            K,
            np.linalg.inv(T_cam_world),
            expected[2],
            DIST_ZERO,
            pixel_is_undistorted=True,
        )
        np.testing.assert_allclose(recovered, expected, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
