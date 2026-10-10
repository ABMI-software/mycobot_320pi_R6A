#!/usr/bin/env python3
"""Read one episode with LeRobot's own LeRobotDataset and write its actions,
states and timestamps to an .npz for replay_episode.py (runs in venv_lerobot).

  export_lerobot_episode.py <root> <repo_id> <episode_index> <out.npz>
"""
import sys

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root, repo_id, episode, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
ds = LeRobotDataset(repo_id, root=root, episodes=[episode])
rows = [ds.hf_dataset[i] for i in range(len(ds.hf_dataset))]
np.savez(out,
         source=np.array('lerobot'),
         timestamp=np.array([float(r['timestamp']) for r in rows]),
         action=np.stack([np.asarray(r['action'], float) for r in rows]),
         state=np.stack([np.asarray(r['observation.state'], float) for r in rows]),
         names=np.array(ds.features['action']['names']),
         instruction=np.array(ds.meta.tasks.index[int(rows[0]['task_index'])]),
         fps=np.array(ds.fps))
print(f'{out}: {len(rows)} steps at {ds.fps} fps, '
      f'"{ds.meta.tasks.index[int(rows[0]["task_index"])]}"')
