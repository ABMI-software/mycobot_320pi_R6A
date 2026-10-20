"""Offline geometry, rejection gates, and preservation of production calibration."""
import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import calibrate_svpro_readonly as calibration


K = np.array([[700., 0., 320.], [0., 710., 240.], [0., 0., 1.]])
DIST = np.zeros(5)
MARKERS = {19: np.array([.10, .20, 0.]), 23: np.array([.10, -.17, 0.]),
           25: np.array([.52, .20, 0.]), 26: np.array([.52, -.17, 0.])}


def fixture():
    rotation = cv2.Rodrigues(np.array([.15, -.2, .08]))[0] @ np.diag([1., -1., -1.])
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = -rotation @ np.array([.3, .0, .85])
    detections = {}
    objects = {}
    for i, center in MARKERS.items():
        corners = np.tile(center, (4, 1))
        corners[:, :2] += np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]]) * .025
        objects[i] = corners
        detections[i] = calibration.project(corners, T, K, DIST)
    return T, objects, detections


class GeometryTests(unittest.TestCase):
    def test_projective_center_corrects_perspective_mean_bias(self):
        T, objects, _ = fixture()
        distortion = np.array([.12, -.05, .004, -.003, .01])
        quad = calibration.project(objects[19], T, K, distortion)
        expected = calibration.project(MARKERS[19].reshape(1, 3), T, K, distortion)[0]
        np.testing.assert_allclose(calibration.projective_center(quad, K, distortion), expected, atol=1e-4)
        self.assertGreater(np.linalg.norm(quad.mean(axis=0) - expected), .01)

    def test_known_oblique_camera_and_holdouts_recovered(self):
        T, _, detections = fixture()
        result, reasons = calibration.fit([detections, detections], MARKERS, K, DIST)
        self.assertEqual(reasons, [])
        np.testing.assert_allclose(result['T_cam_world'], T, atol=1e-6)
        self.assertLess(result['rms_reproj_px'], 1e-5)
        self.assertLess(max(x['plane_error_mm'] for x in result['validation_leave_one_out']), .001)
        self.assertEqual(set(result['centre_interframe_rms_px']), set(calibration.IDS))

    def test_physical_rejection(self):
        T = np.eye(4)
        T[2, 3] = 1.
        reasons = calibration.physical_reasons(T, np.asarray(list(MARKERS.values())))
        self.assertIn('camera is not above the marker plane', reasons)
        T[2, 3] = -1.
        self.assertIn('some marker points are behind the camera',
                      calibration.physical_reasons(T, np.asarray(list(MARKERS.values()))))
        T[0, 0] = np.nan
        self.assertEqual(calibration.physical_reasons(T, np.asarray(list(MARKERS.values()))),
                         ['camera matrix is not finite 4x4'])

    def test_real_intrinsics_scaled_to_capture(self):
        full, full_dist, native = calibration.intrinsics((800, 600))
        small, small_dist, _ = calibration.intrinsics((640, 480))
        self.assertEqual(native, (800, 600))
        np.testing.assert_allclose(small[:2], full[:2] * .8)
        np.testing.assert_array_equal(small_dist, full_dist)


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.reference = self.root / 'markers.yaml'
        self.reference.write_text(yaml.safe_dump({'units': 'mm', 'frame_id': 'base_link',
            'markers': {i: (p * 1000).tolist() for i, p in MARKERS.items()}}))
        self.images = [self.root / f'{i}.png' for i in range(2)]
        for path in self.images:
            cv2.imwrite(str(path), np.zeros((480, 640, 3), dtype=np.uint8))
        self.out = self.root / 'candidate.yaml'
        self.argv = ['--images', str(self.root / '*.png'), '--markers', str(self.reference), '--out', str(self.out)]

    def run_main(self, detections, fitted=None):
        with contextlib.redirect_stdout(io.StringIO()), \
             patch.object(calibration, 'intrinsics', return_value=(K, DIST, (800, 600))), \
             patch.object(calibration, 'detect', side_effect=detections):
            if fitted is None:
                return calibration.main(self.argv)
            with patch.object(calibration, 'fit', return_value=fitted):
                return calibration.main(self.argv)

    def test_missing_tag_in_one_frame_blocks_fit_without_dropping_frame(self):
        _, _, detection = fixture()
        incomplete = dict(detection)
        del incomplete[26]
        with patch.object(calibration, 'fit') as fit:
            code = self.run_main([(detection, []), (incomplete, [26])])
        fit.assert_not_called()
        self.assertEqual(code, 2)
        result = yaml.safe_load(self.out.read_text())
        self.assertEqual(result['validation']['status'], 'failed')
        self.assertEqual(result['marker_detection_counts'][26], 1)
        self.assertEqual(result['capture']['frame_count'], 2)
        self.assertNotIn('T_cam_world', result)

    def test_good_geometry_does_not_claim_reference_confirmed(self):
        _, _, detection = fixture()
        self.assertEqual(self.run_main([(detection, []), (detection, [])]), 0)
        result = yaml.safe_load(self.out.read_text())
        self.assertEqual(result['validation']['status'], 'passed')
        self.assertFalse(result['reference_base_confirmed'])
        self.assertFalse(result['validation']['loo_is_independent_ground_truth'])
        self.assertEqual(result['validation']['thresholds'], {'rms_max_px': 1.5, 'loo_max_mm': 5.0})

    def test_both_quality_gates_are_enforced(self):
        _, _, detection = fixture()
        fit, _ = calibration.fit([detection], MARKERS, K, DIST)
        fit['rms_reproj_px'] = 1.6
        fit['validation_leave_one_out'][0]['plane_error_mm'] = 5.1
        self.assertEqual(self.run_main([(detection, []), (detection, [])], (fit, [])), 2)
        reasons = yaml.safe_load(self.out.read_text())['validation']['reasons']
        self.assertEqual(len(reasons), 2)

    def test_production_directory_and_input_reference_are_protected(self):
        before = self.reference.read_bytes()
        for target in (calibration.CALIB / 'svpro_extrinsic_servo.yaml', self.reference, self.images[0]):
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                calibration.main(self.argv[:-1] + [str(target)])
        self.assertEqual(self.reference.read_bytes(), before)

    def test_explicit_mm_units_are_converted(self):
        read = calibration.load_markers(self.reference)
        for i in MARKERS:
            np.testing.assert_allclose(read[i], MARKERS[i])


if __name__ == '__main__':
    unittest.main()
