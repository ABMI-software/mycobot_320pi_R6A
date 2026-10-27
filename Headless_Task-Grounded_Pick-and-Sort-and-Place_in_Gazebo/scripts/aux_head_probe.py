"""The frozen auxiliary head on the probe layouts (probe_protocol.yaml run_c_gate auxiliary_head).

  aux_head_probe.py <render dir> <features dir> <aux_head.safetensors> <aux_labels.npz> <encoder> ...

Input as in training (smolvla_kaggle_driver.AuxHead.error_sum): the connector output of the three
cameras, each flattened, concatenated in the policy's camera order camera1, camera2, camera3 =
top, right, left (policy_preprocessor rename_map). Output de-normalised with the labels' mean and
std; error per piece = Euclidean x, y distance (mm) to Gazebo's settled pose, layouts drifting more
than 5 mm dropped as in probe_fit.py.
"""
import json, sys
from pathlib import Path

import numpy as np
import torch
from safetensors.torch import load_file

render, feats, head_path, labels_path = map(Path, sys.argv[1:5])
CAMERA_ORDER = ('top', 'right', 'left')
labels = np.load(labels_path)
pieces = [str(p) for p in labels['pieces']]
mean, std = labels['mean'].astype(np.float64), labels['std'].astype(np.float64)
head = {k: v.double().numpy() for k, v in load_file(str(head_path)).items()}
layouts = [json.loads(l) for l in (render / 'layouts.jsonl').read_text().splitlines()]
kept = [l for l in layouts if l['drift_max_mm'] <= 5.0]
y = np.array([np.concatenate([l['gazebo'][p][:2] for p in pieces]) for l in kept])
print(f'{len(kept)} of {len(layouts)} layouts, pieces {pieces}, head {head["weight"].shape}')
out = {}
for enc in sys.argv[5:]:
    f = np.load(feats / f'{enc}.npz')
    index = {int(s): i for i, s in enumerate(f['seeds'])}
    rows = [index[l['seed']] for l in kept]
    x = np.concatenate([f[f'{c}_connector'][rows].reshape(len(rows), -1).astype(np.float64) for c in CAMERA_ORDER], 1)
    pred = (x @ head['weight'].T + head['bias']) * std + mean
    err = np.linalg.norm((pred - y).reshape(len(y), 4, 2), axis=2) * 1000
    out[enc] = {'median_mm': round(float(np.median(err)), 1), 'p90_mm': round(float(np.percentile(err, 90)), 1),
                'per_piece_median_mm': {p: round(float(np.median(err[:, k])), 1) for k, p in enumerate(pieces)}}
    print(enc, json.dumps(out[enc]))
floor = np.linalg.norm((mean - y).reshape(len(y), 4, 2), axis=2) * 1000
print('labels mean (no image)', round(float(np.median(floor)), 1), round(float(np.percentile(floor, 90)), 1))
