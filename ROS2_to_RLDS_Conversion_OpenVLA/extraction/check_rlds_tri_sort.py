"""Reload the built RLDS dataset with tfds and report what a loader sees;
save a contact sheet (4 steps x 3 cameras). Runs in rlds_builder."""
import sys
import numpy as np
import tensorflow_datasets as tfds
from PIL import Image

builder = tfds.builder_from_directory(sys.argv[1])
ds = builder.as_dataset(split='train')
print('episodes:', builder.info.splits['train'].num_examples)
for ep in ds:
    steps = list(ep['steps'].as_numpy_iterator())
    print('file:', ep['episode_metadata']['file_path'].numpy().decode(), '| steps:', len(steps))
    print('instruction:', steps[0]['language_instruction'].decode())
    for k in ('image_top', 'image_right', 'image_left', 'state', 'joint_state'):
        v = steps[0]['observation'][k]; print(f'  observation.{k:12s} {v.shape} {v.dtype}')
    for k in ('action', 'joint_action'):
        v = steps[0][k]; print(f'  {k:24s} {v.shape} {v.dtype}')
    a = np.stack([s['action'] for s in steps]); st = np.stack([s['observation']['state'] for s in steps])
    print('is_first/is_last:', steps[0]['is_first'], steps[-1]['is_last'], '| reward on last:', steps[-1]['reward'])
    print('action |dpos| max mm:', round(float(np.abs(a[:, :3]).max()) * 1000, 1),
          '| gripper action range:', round(float(a[:, 6].min()), 2), '..', round(float(a[:, 6].max()), 2))
    print('state z range (m):', round(float(st[:, 2].min()), 3), '..', round(float(st[:, 2].max()), 3),
          '| NaN anywhere:', bool(np.isnan(a).any() or np.isnan(st).any()))
    picks = [0, len(steps) // 3, 2 * len(steps) // 3, len(steps) - 1]
    rows = [np.hstack([steps[i]['observation'][c] for c in ('image_top', 'image_right', 'image_left')])
            for i in picks]
    Image.fromarray(np.vstack(rows)).save(sys.argv[2])
