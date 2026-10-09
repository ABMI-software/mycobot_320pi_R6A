"""Guard: camera_layout:=dream50k must reproduce the cameras of the 50K DREAM dataset.

The DREAM labels were computed with training/dream/mycobot_fk.GAZEBO_CAMERAS and
rendered by mycobot_pro_320_pi_gazebo_nogripper.urdf. Any drift here silently
shows DREAM viewpoints it was never trained on.
"""

import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mycobot_gateway'))
sys.path.insert(0, str(ROOT / 'training' / 'dream'))
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras  # noqa: E402
import mycobot_fk  # noqa: E402

URDF_DIR = ROOT / 'mycobot_description/urdf/320_pi'
GRIPPER_URDF = URDF_DIR / 'mycobot_pro_320_pi_gazebo.urdf'
DREAM_URDF = URDF_DIR / 'mycobot_pro_320_pi_gazebo_nogripper.urdf'
FK_NAMES = {'synth_camera': 'front', 'synth_camera_right': 'right',
            'synth_camera_left': 'left', 'synth_camera_top': 'top'}
# _camera_settings.json of dream_data/synthetic_50k_ndds (not tracked by git).
FX_50K = 493.7924924535111
NDDS_SETTINGS = ROOT / 'training/dream/dream_data/synthetic_50k_ndds/_camera_settings.json'


def sensors(urdf, mappings):
    root = ET.fromstring(xacro.process_file(str(urdf), mappings=mappings).toxml())
    return {s.get('name'): s for s in root.iter('sensor') if s.get('name') in FK_NAMES}


class Dream50kCameraTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cameras = load_cameras(GRIPPER_URDF, {'camera_layout': 'dream50k'})

    def test_poses_match_dream_labels(self):
        for name, fk_name in FK_NAMES.items():
            np.testing.assert_allclose(
                self.cameras[name].world_from_optical,
                mycobot_fk.get_camera_transform(fk_name), atol=1e-9, err_msg=name)

    def test_poses_match_dream_render(self):
        rendered = load_cameras(DREAM_URDF)
        for name in FK_NAMES:
            np.testing.assert_allclose(self.cameras[name].world_from_optical,
                                       rendered[name].world_from_optical, atol=1e-12)
            np.testing.assert_allclose(self.cameras[name].K, rendered[name].K, atol=1e-12)

    def test_intrinsics_match_dataset(self):
        K = np.array([[FX_50K, 0, 320], [0, FX_50K, 240], [0, 0, 1]])
        for camera in self.cameras.values():
            self.assertEqual((camera.width, camera.height), (640, 480))
            np.testing.assert_allclose(camera.K, K, atol=1e-6)
            np.testing.assert_allclose(camera.K, mycobot_fk.GAZEBO_INTRINSICS, atol=1e-9)

    @unittest.skipUnless(NDDS_SETTINGS.exists(), '50K NDDS dataset not on this machine')
    def test_intrinsics_match_ndds_file(self):
        fx = json.loads(NDDS_SETTINGS.read_text())['camera_settings'][0]['intrinsic_settings']['fx']
        self.assertAlmostEqual(fx, FX_50K, places=9)

    def test_sensor_settings_match_dream_render(self):
        ours = sensors(GRIPPER_URDF, {'camera_layout': 'dream50k'})
        theirs = sensors(DREAM_URDF, {})
        for name in FK_NAMES:
            for path in ('update_rate', 'camera/horizontal_fov', 'camera/image/width',
                         'camera/image/height', 'camera/image/format',
                         'camera/clip/near', 'camera/clip/far'):
                self.assertEqual(ours[name].findtext(path).strip(),
                                 theirs[name].findtext(path).strip(), f'{name} {path}')
            self.assertIsNone(ours[name].find('camera/noise'))
            self.assertIsNone(ours[name].find('camera/distortion'))

    def test_legacy_layout_differs_only_in_poses(self):
        legacy = load_cameras(GRIPPER_URDF, {'camera_layout': 'legacy'})
        for name in FK_NAMES:
            np.testing.assert_allclose(legacy[name].K, self.cameras[name].K)
            np.testing.assert_allclose(legacy[name].world_from_optical[:3, :3],
                                       self.cameras[name].world_from_optical[:3, :3])

    def test_unknown_layout_is_rejected(self):
        with self.assertRaises(Exception):
            load_cameras(GRIPPER_URDF, {'camera_layout': 'dream'})


if __name__ == '__main__':
    unittest.main()
