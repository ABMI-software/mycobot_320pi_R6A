#!/usr/bin/env python3
"""Read one episode of the built RLDS dataset with tfds and write its OpenVLA
fields to an .npz for replay_episode.py (runs in rlds_builder).

  export_rlds_episode.py <tfds version dir> <episode_index> <out.npz>

Only what OpenVLA sees is exported for the replay: the end-effector state and
the 7-value action. joint_action is exported too, solely to measure the
inverse-kinematics round trip; the replay never sends it.
"""
import sys

import numpy as np
import tensorflow_datasets as tfds

builder = tfds.builder_from_directory(sys.argv[1])
episode = list(builder.as_dataset(split='train').skip(int(sys.argv[2])).take(1))[0]
steps = list(episode['steps'].as_numpy_iterator())
np.savez(sys.argv[3],
         source=np.array('rlds'),
         timestamp=np.arange(len(steps)) / 10.0,
         state=np.stack([s['observation']['state'] for s in steps]).astype(float),
         action=np.stack([s['action'] for s in steps]).astype(float),
         joint_state=np.stack([s['observation']['joint_state'] for s in steps]).astype(float),
         joint_action=np.stack([s['joint_action'] for s in steps]).astype(float),
         instruction=np.array(steps[0]['language_instruction'].decode()),
         fps=np.array(10))
print(f'{sys.argv[3]}: {len(steps)} steps, "{steps[0]["language_instruction"].decode()}", '
      f'from {episode["episode_metadata"]["file_path"].numpy().decode()}')
