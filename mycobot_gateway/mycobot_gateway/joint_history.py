"""Arm joint angles over time, read back at any stamp (an image's, invariant I5)."""

from bisect import bisect_left
from collections import deque

import numpy as np

JOINTS = ('joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6')


def interpolate(times, values, t):
    """Joint vector at t, linear between the bracketing samples; (q, gap to nearest sample)."""
    i = bisect_left(times, t)
    if i == 0 or i == len(times):
        j = min(i, len(times) - 1)
        return values[j], abs(times[j] - t)
    t0, t1 = times[i - 1], times[i]
    w = (t - t0) / (t1 - t0)
    return (1 - w) * values[i - 1] + w * values[i], min(t - t0, t1 - t)


class JointHistory:
    def __init__(self, span_s=10.0):
        self.span = span_s
        self.times, self.qs = deque(), deque()

    def add(self, msg):
        """Keep a /joint_states message (radians, [j1..j6]); partial or stale ones are skipped."""
        positions = dict(zip(msg.name, msg.position))
        if not all(j in positions for j in JOINTS):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.times and t <= self.times[-1]:
            return
        self.times.append(t)
        self.qs.append(np.array([positions[j] for j in JOINTS]))
        while self.times[-1] - self.times[0] > self.span:
            self.times.popleft()
            self.qs.popleft()

    def at(self, t):
        """(q, gap in s to the nearest sample), or (None, inf) before any sample."""
        if not self.times:
            return None, float('inf')
        return interpolate(list(self.times), list(self.qs), t)
