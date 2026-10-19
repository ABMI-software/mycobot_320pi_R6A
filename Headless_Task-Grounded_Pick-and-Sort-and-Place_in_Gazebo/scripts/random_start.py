"""Random arm start pose for the random-start recordings and the random-start evaluation.

The sampler of the 14/10 recordings (seeds 21-50) was lost with the machine, and its poses fit
no simple rule: all six joints vary, the per-joint spreads are not uniform, the tip is neither
in a box nor pointing down. So new poses are drawn from those recordings themselves: one of the
120 recorded start poses (config/random_start_poses_s021_050.csv) at random, plus Gaussian
jitter of JITTER x that joint's spread. That keeps the per-joint distributions and the
correlations between joints. A pose is kept only if the tip is at least MIN_TIP_Z above the
board, every joint MIN_MARGIN_DEG from its limit (the smallest values over the 120), and each
joint inside the range the 120 span: the lost sampler had hard bounds there (J5 exactly +-89.9,
J2 at most 89.7, J1 at least -69.9). A rejected draw is jittered again around the same recorded
pose, so the low starts (16 % below 20 cm) keep their share instead of being drawn out.
"""
from pathlib import Path

import numpy as np

from mycobot_gateway.sim_sorting_grasp import limit_margin, tool_tip

REFERENCE = Path(__file__).resolve().parent.parent / 'config' / 'random_start_poses_s021_050.csv'
JITTER = 0.25
MIN_TIP_Z = 0.13
MIN_MARGIN_DEG = 20.0


def reference_poses(path=REFERENCE):
    return np.loadtxt(path, delimiter=',', comments='#', skiprows=4, usecols=range(3, 9))


_REF = None


def sample(rng, attempts=200, jitter=JITTER):
    global _REF
    if _REF is None:
        _REF = reference_poses()
    scale = jitter * _REF.std(axis=0)
    lo, hi = _REF.min(axis=0), _REF.max(axis=0)
    base = _REF[rng.integers(len(_REF))]
    for _ in range(attempts):
        q = base + rng.normal(0.0, scale)
        if (np.all((q >= lo) & (q <= hi)) and tool_tip(q)[2] >= MIN_TIP_Z
                and limit_margin(q) >= MIN_MARGIN_DEG):
            return q
    return base.copy()
