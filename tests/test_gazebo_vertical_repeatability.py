"""Verify the physical tool orientation and interpolated unidirectional path."""
import sys
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from gazebo_vertical_repeatability import make_vertical_path, TOOL_VERTICAL, TARGET_VERTICAL
from gazebo_precision_campaign import metrics
from diff_ik import fk_pose


def test_interpolated_tool_stays_down_on_vertical_line():
    q = np.array(make_vertical_path())
    times = np.linspace(0, 3, len(q))
    velocities = np.gradient(q, times, axis=0)
    velocities[[0, -1]] = 0
    curve = CubicHermiteSpline(times, q, velocities)
    previous_z = 59.99
    for t in np.linspace(0, 3, 401):
        point, rotation = fk_pose(curve(t))
        tip = point + rotation @ TOOL_VERTICAL
        assert np.linalg.norm(tip[:2] - TARGET_VERTICAL[:2]) < .01
        assert np.linalg.norm(rotation[:, 2] - [0, 0, -1]) < 1e-5
        assert tip[2] >= previous_z - 1e-6
        previous_z = tip[2]
        assert np.max(np.abs(curve(t, 1))) < 30
    assert abs(previous_z - 100) < .001


def test_error_uses_new_target():
    result = metrics([TARGET_VERTICAL, TARGET_VERTICAL], TARGET_VERTICAL)
    assert result['mean_error_mm'] == 0
    assert result['rp_mm'] == 0
