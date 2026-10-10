#!/usr/bin/env python3
"""Read one episode of the built RLDS dataset with tfds and write its OpenVLA
fields to an .npz for replay_episode.py (runs in rlds_builder).

  export_rlds_episode.py <tfds version dir> <seed> <piece> <out.npz>

The episode is the one whose episode_metadata.file_path is
episode_seed<NNN>_<piece>_a<attempt>.npy (extract_tri_sort.py). Only what
OpenVLA sees is used by the replay: the end-effector state and the 7-value
action; joint_action is exported solely to measure the IK round trip.
"""
import sys

import numpy as np
import tensorflow_datasets as tfds

src, seed, piece, out = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
prefix = f'episode_seed{seed:03d}_{piece}_a'
builder = tfds.builder_from_directory(src)
episode = next(ep for ep in builder.as_dataset(split='train')
               if ep['episode_metadata']['file_path'].numpy().decode().startswith(prefix))
steps = list(episode['steps'].as_numpy_iterator())
np.savez(out,
         source=np.array('rlds'),
         timestamp=np.arange(len(steps)) / 10.0,
         state=np.stack([s['observation']['state'] for s in steps]).astype(float),
         action=np.stack([s['action'] for s in steps]).astype(float),
         joint_state=np.stack([s['observation']['joint_state'] for s in steps]).astype(float),
         joint_action=np.stack([s['joint_action'] for s in steps]).astype(float),
         instruction=np.array(steps[0]['language_instruction'].decode()),
         fps=np.array(10))
print(f'{out}: {len(steps)} steps, "{steps[0]["language_instruction"].decode()}", '
      f'from {episode["episode_metadata"]["file_path"].numpy().decode()}')
