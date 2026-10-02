"""Matching and Gazebo marker text of the YOLO vs ground-truth overlay."""

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mycobot_gateway'))
from mycobot_gateway import yolo_gt_overlay as o  # noqa: E402

GT = {'cube_rouge': (np.array([0.17, 0.13, 0.0]), 0.025),
      'bac_jaune': (np.array([0.20, -0.10, 0.0]), 0.04),
      'pave_jaune': (np.array([0.12, 0.05, 0.0]), 0.02)}


class MatchTests(unittest.TestCase):
    def test_nearest_within_gate_with_error(self):
        yolo = [('cube_rouge', 0.9, np.array([0.174, 0.133, 0.0125])),
                ('bac_vert', 0.4, np.array([0.201, -0.102, 0.02]))]
        matches, extra = o.match(GT, yolo)
        self.assertAlmostEqual(matches['cube_rouge'][3], 0.005, places=6)
        self.assertEqual(o.verdict('cube_rouge', matches['cube_rouge']), 'ok')
        self.assertEqual(o.verdict('bac_jaune', matches['bac_jaune']), 'wrong_class')
        self.assertEqual(o.verdict('pave_jaune', matches['pave_jaune']), 'missed')
        self.assertEqual(extra, [])

    def test_far_estimate_is_extra_not_a_match(self):
        matches, extra = o.match(GT, [('cube_rouge', 0.9, np.array([0.30, 0.30, 0.0]))])
        self.assertIsNone(matches['cube_rouge'])
        self.assertEqual(len(extra), 1)

    def test_one_estimate_matches_one_piece(self):
        both = {'a': (np.array([0.0, 0.0, 0.0]), 0.02), 'b': (np.array([0.02, 0.0, 0.0]), 0.02)}
        matches, _ = o.match(both, [('a', 0.8, np.array([0.005, 0.0, 0.0]))])
        self.assertIsNotNone(matches['a'])
        self.assertIsNone(matches['b'])


class LabelTests(unittest.TestCase):
    def test_labels(self):
        self.assertEqual(o.label('cube_rouge', ('cube_rouge', 0.91, None, 0.0042)), '4.2 mm')
        self.assertEqual(o.label('bac_jaune', ('bac_vert', 0.4, None, 0.002)), '2.0 mm GT bac_jaune')
        self.assertEqual(o.label('pave_jaune', None), 'pave_jaune NON DETECTE')

    def test_summary(self):
        matches, extra = o.match(GT, [('cube_rouge', 0.9, np.array([0.174, 0.133, 0.0125]))])
        self.assertEqual(o.summary_line(GT, matches, extra),
                         'vs GT : 1/3 classe juste | XY med 5.0 mm max 5.0 mm | 0 en trop')


class DrawTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from mycobot_gateway.vision.sim_multicam_geometry import load_cameras
        cls.cam = load_cameras(ROOT / 'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf',
                               {'camera_layout': 'dream50k'})['synth_camera_top']

    def test_view_is_a_4_3_crop_inside_the_image_around_every_piece(self):
        x0, y0, scale = o.view_window(self.cam, GT)
        w, h = o.VIEW_W / scale, o.VIEW_H / scale
        self.assertAlmostEqual(w / h, 4 / 3)
        self.assertGreaterEqual(x0, 0)
        self.assertGreaterEqual(y0, 0)
        self.assertLessEqual(x0 + w, self.cam.width + 1e-9)
        self.assertLessEqual(y0 + h, self.cam.height + 1e-9)
        self.assertGreater(scale, 1.0)
        for xyz, height in GT.values():
            u, v = o.to_view(*self.cam.project(xyz + [0, 0, height / 2])[0], (x0, y0, scale))
            self.assertTrue(0 <= u < o.VIEW_W and 0 <= v < o.VIEW_H)

    def test_draws_at_the_projected_centre(self):
        window = o.view_window(self.cam, GT)
        image = o.crop(np.zeros((self.cam.height, self.cam.width, 3), np.uint8), window)
        self.assertEqual(image.shape, (o.VIEW_H, o.VIEW_W, 3))
        matches, extra = o.match(GT, [('cube_rouge', 0.9, np.array([0.174, 0.133, 0.0125]))])
        o.draw_errors(image, self.cam, GT, matches, extra, window)
        u, v = o.to_view(*self.cam.project(GT['cube_rouge'][0] + [0, 0, 0.0125])[0], window)
        self.assertTrue(image[int(v) - 3:int(v) + 4, int(u) - 3:int(u) + 4].any())


class FourCameraTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from mycobot_gateway.vision.sim_multicam_geometry import load_cameras
        cls.cams = load_cameras(ROOT / 'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf',
                                {'camera_layout': 'dream50k'})

    def test_every_camera_sees_the_pieces_it_projects_inside(self):
        far = dict(GT, far_away=(np.array([3.0, 3.0, 0.0]), 0.02))
        for name in o.CAMERAS:
            seen = o.visible(self.cams[name], far)
            self.assertNotIn('far_away', seen, name)
            for xyz, h in seen.values():
                u, v = self.cams[name].project(xyz + [0, 0, h / 2])[0]
                self.assertTrue(0 <= u < 640 and 0 <= v < 480)
        self.assertEqual(set(o.visible(self.cams['synth_camera_top'], GT)), set(GT))

    def test_no_visible_piece_shows_the_whole_image(self):
        x0, y0, scale = o.view_window(self.cams['synth_camera'], {})
        self.assertEqual((x0, y0), (0.0, 0.0))
        self.assertAlmostEqual(scale, o.VIEW_W / 640)

    def test_mosaic_is_two_by_two_with_placeholders(self):
        view = np.full((o.VIEW_H, o.VIEW_W, 3), 200, np.uint8)
        m = o.mosaic({'synth_camera_top': view}, list(o.CAMERAS))
        self.assertEqual(m.shape, (o.VIEW_H, o.VIEW_W, 3))
        self.assertTrue((m[o.TILE_H + 50:, o.TILE_W + 50:] == 200).all())
        self.assertFalse((m[50:o.TILE_H, 50:o.TILE_W] == 200).any())


if __name__ == '__main__':
    unittest.main()
