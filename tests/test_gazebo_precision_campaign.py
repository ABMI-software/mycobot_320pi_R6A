"""Checks for scientific metrics and campaign completeness, without Gazebo."""
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('campaign', ROOT / 'scripts/gazebo_precision_campaign.py')
campaign = importlib.util.module_from_spec(spec)
spec.loader.exec_module(campaign)


class CampaignMetricsTests(unittest.TestCase):
    def model(self, arm_mass=3.0, gripper_mass=.34):
        root = ET.Element('robot')
        for names, mass in ((['base'] + ['link' + str(i) for i in range(1, 7)], arm_mass),
                            (['gripper_base'] + ['gripper_' + str(i) for i in range(6)], gripper_mass)):
            for name in names:
                link = ET.SubElement(root, 'link', name=name)
                inertial = ET.SubElement(link, 'inertial')
                ET.SubElement(inertial, 'mass', value=str(mass / len(names)))
        return ET.tostring(root, encoding='unicode')

    def test_mass_audit_counts_fixed_base_and_gripper_once(self):
        actual = campaign.model_masses(self.model())
        campaign.check_requested_masses(actual, 3., .34)
        self.assertAlmostEqual(actual['arm_and_gripper_kg'], 3.34)
        self.assertEqual(len(actual['links_kg']['arm']), 7)

    def test_wrong_loaded_mass_refuses_motion(self):
        actual = campaign.model_masses(self.model(1.58))
        with self.assertRaisesRegex(ValueError, 'motion refused'):
            campaign.check_requested_masses(actual, 3., .34)

    def test_incomplete_model_does_not_pass_mass_audit(self):
        with self.assertRaises(ValueError):
            campaign.model_masses('<robot/>')

    def test_nonfinite_or_negative_mass_is_rejected(self):
        for value in (-1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                campaign.model_masses(self.model(value))

    def test_sample_standard_deviation_not_population(self):
        points = [[0, 0, 0], [0, 0, 0], [3, 0, 0]]
        self.assertAlmostEqual(campaign.metrics(points)['rp_mm'], 4 / 3 + math.sqrt(3))

    def test_repeatability_is_translation_invariant(self):
        points = np.array([[0, 0, 0], [1, 2, 3], [4, -2, 1]])
        self.assertAlmostEqual(campaign.metrics(points)['rp_mm'],
                               campaign.metrics(points + [100, -20, 30])['rp_mm'])

    def test_mean_error_is_not_norm_of_mean_bias(self):
        points = campaign.TARGET + [[-1, 0, 0], [1, 0, 0]]
        m = campaign.metrics(points)
        self.assertEqual(m['mean_error_mm'], 1.)
        self.assertEqual(m['bias_xyz_mm'], [0., 0., 0.])

    def test_bbox_is_not_pairwise_distance(self):
        centers = [[0, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
        groups = {'A/haut': [[0, 0, 0]] * 2}
        for direction, point in zip(('devant', 'derriere', 'gauche', 'droite'), centers):
            groups['G/' + direction] = [point, point]
        m = campaign.summarize(groups)
        self.assertAlmostEqual(m['G_inter']['bbox_diagonal_mm'], math.sqrt(3))
        self.assertAlmostEqual(m['G_inter']['max_pairwise_mm'], math.sqrt(2))

    def test_invalid_observations_rejected(self):
        for points in ([], [[0, 0, 0]], [[0, 0, 0], [float('nan'), 0, 0]]):
            with self.assertRaises(ValueError):
                campaign.metrics(points)

    def test_partial_campaign_cannot_be_reported_as_complete(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)
            (p / 'samples.jsonl').write_text('')
            (p / 'plan.json').write_text('{}')
            (p / 'reference.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'Incomplete'):
                campaign.report(p)


if __name__ == '__main__':
    unittest.main()
