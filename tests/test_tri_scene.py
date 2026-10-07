"""Sorting-scene randomization: every draw satisfies every placement constraint."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mycobot_gateway'))
from mycobot_gateway.vision import tri_scene as ts  # noqa: E402
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras  # noqa: E402

MODELS = ROOT / 'mycobot_description/models'
WORLD = ROOT / 'mycobot_description/worlds/real_table.sdf'
URDF = ROOT / 'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf'
DRAWS = 300


class TriSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.footprints = ts.load_footprints(MODELS)
        cls.board = ts.load_board(WORLD, MODELS)
        cls.top = load_cameras(URDF, {'camera_layout': 'dream50k'})['synth_camera_top']

    def draw(self, seed):
        return ts.sample_tri_scene(np.random.default_rng(seed), self.footprints, self.board, self.top)

    def test_footprints_come_from_model_files(self):
        np.testing.assert_allclose(self.footprints['cube_rouge'].half, [.02, .02])
        np.testing.assert_allclose(self.footprints['pave_jaune'].half, [.025, .015])
        np.testing.assert_allclose(self.footprints['bac_bleu'].half, [.0525, .0525])
        self.assertAlmostEqual(self.footprints['cylindre_vert'].height, .05)

    def test_work_band_lies_between_the_marker_pairs(self):
        near, far = self.board.work_x()
        self.assertGreater(near, self.board.markers['aruco_23'][0][0])
        self.assertLess(far, self.board.markers['aruco_26'][0][0])

    def test_every_draw_is_valid(self):
        rng = np.random.default_rng(12)
        near, far = self.board.work_x()
        for _ in range(DRAWS):
            layout = ts.sample_tri_scene(rng, self.footprints, self.board, self.top)
            self.assertEqual(set(layout), set(ts.OBJECTS + ts.BINS))
            self.assertEqual(ts.placement_errors(layout, self.footprints, self.board, self.top), [])
            for name, xy in layout.items():
                half = self.footprints[name].half
                self.assertGreaterEqual(xy[0] - half[0], near)
                self.assertLessEqual(xy[0] + half[0], far)

    def test_seed_replays_the_scene(self):
        a, b = self.draw(5), self.draw(5)
        for name in a:
            np.testing.assert_array_equal(a[name], b[name])
        self.assertFalse(all(np.allclose(a[n], self.draw(6)[n]) for n in a))

    def test_overlapping_items_are_rejected(self):
        layout = self.draw(3)
        layout['cube_rouge'] = layout['bac_rouge'].copy()
        self.assertTrue(any('closer than' in e for e in
                            ts.placement_errors(layout, self.footprints, self.board, self.top)))

    def test_world_keeps_board_and_markers_and_places_the_eight_pieces(self):
        layout = self.draw(4)
        world = ET.fromstring(ts.build_world(WORLD, layout, 'tri_yolo')).find('world')
        self.assertEqual(world.get('name'), 'tri_yolo')
        models = {m.get('name') for m in world.findall('model')}
        self.assertIn('table', models)
        self.assertFalse(models & set(ts.REMOVED_MODELS))
        included = {i.findtext('name'): np.array(i.findtext('pose').split(), float)
                    for i in world.findall('include')}
        for marker in ('aruco_19', 'aruco_23', 'aruco_25', 'aruco_26'):
            self.assertIn(marker, included)
        for name, xy in layout.items():
            np.testing.assert_allclose(included[name][:2], xy, atol=1e-6)
            np.testing.assert_allclose(included[name][2:], 0)

    def test_box_centre_locates_every_piece_on_every_camera(self):
        cameras = load_cameras(URDF, {'camera_layout': 'dream50k'})
        worst = 0.0
        for seed in range(20):
            for name, xy in self.draw(seed).items():
                for camera in cameras.values():
                    if not ts.in_view(camera, xy, self.footprints[name]):
                        continue
                    uv = ts.box_centre(camera, xy, self.footprints[name])
                    found = ts.locate_from_box(camera, uv, self.footprints[name])
                    worst = max(worst, float(np.hypot(*(found[:2] - xy))))
        self.assertLess(worst, 1e-4)

    def test_fusion_takes_the_median_and_drops_a_stray_view(self):
        per_camera = {'top': {'cube_rouge': np.array([0.200, 0.100, 0.02])},
                      'front': {'cube_rouge': np.array([0.202, 0.101, 0.02])},
                      'left': {'cube_rouge': np.array([0.199, 0.099, 0.02])},
                      'right': {'cube_rouge': np.array([0.260, 0.100, 0.02])}}
        centre, kept = ts.fuse_cameras(per_camera)['cube_rouge']
        self.assertEqual(kept, ['front', 'left', 'top'])
        np.testing.assert_allclose(centre, [0.200, 0.100, 0.02])

    def test_best_detection_per_class_wins_duplicates(self):
        best = ts.best_per_class([('bac_vert', [0.3, 0.0, 0.015], 0.4),
                                  ('bac_vert', [0.35, 0.0, 0.015], 0.9)])
        np.testing.assert_allclose(best['bac_vert'], [0.35, 0.0, 0.015])

    def test_pieces_are_paired_with_the_bin_of_their_class(self):
        fused = {name: (np.array([i, 0.0, 0.0]), ['top'])
                 for i, name in enumerate(ts.OBJECTS + ts.BINS)}
        del fused['bac_bleu']
        pairs = ts.sorting_pairs(fused)
        self.assertEqual(set(pairs), {'cube_rouge', 'pave_jaune', 'cylindre_vert'})
        for piece, (_, bin_xyz) in pairs.items():
            np.testing.assert_allclose(bin_xyz, fused[ts.PAIRS[piece]][0])


if __name__ == '__main__':
    unittest.main()
