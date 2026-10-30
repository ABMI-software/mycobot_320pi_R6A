"""Which training frames does LeRobot draw first, and how many were in the aux warm-start sample?

Rebuilds EpisodeAwareSampler from the dataset's episode boundaries (no model is loaded) and compares
its first frames with the index saved by the driver's auxfeatures mode.

    sampler_order.py <dataset root> <aux_features.npz or aux_warm_start_index.npz>
"""
import inspect, sys
import numpy as np
import torch
from lerobot.datasets import EpisodeAwareSampler
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig

root, fitted_npz = sys.argv[1], sys.argv[2]
meta = LeRobotDatasetMetadata('local/mycobot_tri_sort', root=root)
ep = meta.episodes
cfg = SmolVLAConfig()
print('SmolVLAConfig drop_n_last_frames:', getattr(cfg, 'drop_n_last_frames', 'absent'),
      '| sampler args:', list(inspect.signature(EpisodeAwareSampler.__init__).parameters)[1:])
fitted = np.load(fitted_npz)['index']
def order(seed, n):
    s = EpisodeAwareSampler(ep['dataset_from_index'], ep['dataset_to_index'], episode_indices_to_use=None,
                            drop_n_last_frames=getattr(cfg, 'drop_n_last_frames', 0), shuffle=True, seed=seed)
    it = iter(s)
    return np.array([next(it) for _ in range(n)])
o = order(1000, 1200)
print('seed 1000, first 1200 sampler frames == warm-start frames in the same order:', bool(np.array_equal(o, fitted)),
      '| as a set:', len(set(o.tolist()) & set(fitted.tolist())), 'of 1200 shared')
nxt = order(1000, 2400)[1200:]
print('frames 1201-2400 (updates 41-80) in the warm-start sample:', len(set(nxt.tolist()) & set(fitted.tolist())))
c = order(1001, 60)
print('seed 1001 (calibration), first 60 frames in the warm-start sample:', len(set(c.tolist()) & set(fitted.tolist())))
for k in (1, 40, 41):
    b = o[(k-1)*30:k*30] if k <= 40 else nxt[(k-41)*30:(k-40)*30]
    print(f'update {k}: {len(set(b.tolist()) & set(fitted.tolist()))}/30 frames from the warm-start sample')
