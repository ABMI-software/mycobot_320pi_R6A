#!/usr/bin/env python3
"""Warm start of Run C's auxiliary head: a ridge fit on frozen smolvla_base connector features.

  aux_warm_start.py <features .npz> <dataset meta dir>

<features .npz> comes from the Kaggle driver's auxfeatures mode (index, x: the three cameras'
connector output of a sample of training frames, through the training data path). Labels and
their normalisation are <meta dir>/aux_labels.npz (build_aux_labels.py). Per piece, on the frames
where its label is valid, the probe's own ridge (probe_fit.ridge: features standardised, alpha by
inner 5-fold CV over 10^-2 .. 10^5) predicts the normalised x, y. Its error is estimated by an
outer 5-fold split by episode (one episode = one layout, the probe's split by layout), in mm, next
to the probe's floor_mean (the training folds' mean position, no image); the
fit on all sampled frames is then folded into the linear head (weight / feature spread, bias) and
written as <meta dir>/aux_head_init.safetensors, with aux_warm_start.json. The head is checked to
reproduce the ridge's predictions.

/workspace/venv_lerobot/bin/python with ROS sourced (probe_fit imports the scene geometry).
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
from safetensors.torch import save_file

sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_fit import FOLDS, ridge, standardise  # noqa: E402


def ridge_weights(x, y):
    """Weight (d, k) and bias (k) on raw features equal to probe_fit.ridge's fit on standardised ones."""
    _, alpha = ridge(x, y, x[:1])
    mu, sd = x.mean(0), x.std(0)
    sd[sd < 1e-8] = 1.0
    xs, _ = standardise(x, x[:1])
    a = torch.as_tensor(xs, dtype=torch.float64)
    ym = y.mean(0)
    c = torch.linalg.solve(a @ a.T + alpha * torch.eye(len(a), dtype=torch.float64),
                           torch.as_tensor(y - ym, dtype=torch.float64))
    w = (a.T @ c).numpy()
    weight = w / sd[:, None]
    bias = ym - (mu / sd) @ w
    return weight, bias, alpha


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('features', type=Path)
    ap.add_argument('meta', type=Path)
    args = ap.parse_args()
    f = np.load(args.features)
    labels = np.load(args.meta / 'aux_labels.npz')
    row = {int(i): k for k, i in enumerate(labels['index'])}
    k = np.array([row[int(i)] for i in f['index']])
    x = f['x'].astype(np.float32)
    y = (labels['xy'][k] - labels['mean']) / labels['std']
    mask, episode, std = labels['mask'][k], labels['episode'][k], labels['std']
    pieces = [str(p) for p in labels['pieces']]
    weight = np.zeros((x.shape[1], y.shape[1]), dtype=np.float64)
    bias = np.zeros(y.shape[1], dtype=np.float64)
    errors, floors, report = [], [], {'frames': int(len(x)), 'episodes': int(len(set(episode))), 'pieces': {}}
    for j, piece in enumerate(pieces):
        rows = np.flatnonzero(mask[:, j])
        cols = [2 * j, 2 * j + 1]
        eps = np.array(sorted(set(episode[rows])))
        folds = np.array_split(np.random.default_rng(0).permutation(eps), FOLDS)
        pred, floor = np.zeros((len(rows), 2)), np.zeros((len(rows), 2))
        for held in folds:
            te = np.isin(episode[rows], held)
            pred[te], _ = ridge(x[rows[~te]], y[rows[~te]][:, cols], x[rows[te]])
            floor[te] = y[rows[~te]][:, cols].mean(0)
        err = np.linalg.norm((pred - y[rows][:, cols]) * std[cols], axis=1) * 1000
        floor_err = np.linalg.norm((floor - y[rows][:, cols]) * std[cols], axis=1) * 1000
        errors.append(err)
        floors.append(floor_err)
        w, b, alpha = ridge_weights(x[rows], y[rows][:, cols])
        weight[:, cols], bias[cols] = w, b
        check = x[rows] @ w + b
        refit, _ = ridge(x[rows], y[rows][:, cols], x[rows])
        report['pieces'][piece] = {'frames': int(len(rows)), 'alpha': alpha,
                                   'cv_by_episode_median_mm': round(float(np.median(err)), 2),
                                   'cv_by_episode_p90_mm': round(float(np.percentile(err, 90)), 2),
                                   'floor_mean_median_mm': round(float(np.median(floor_err)), 2),
                                   'head_vs_ridge_max_abs': float(np.abs(check - refit).max())}
    allerr = np.concatenate(errors)
    report['cv_by_episode_median_mm'] = round(float(np.median(allerr)), 2)
    report['cv_by_episode_p90_mm'] = round(float(np.percentile(allerr, 90)), 2)
    report['floor_mean_median_mm'] = round(float(np.median(np.concatenate(floors))), 2)
    save_file({'weight': torch.from_numpy(weight.T.astype(np.float32)).contiguous(),
               'bias': torch.from_numpy(bias.astype(np.float32))}, str(args.meta / 'aux_head_init.safetensors'))
    (args.meta / 'aux_warm_start.json').write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
