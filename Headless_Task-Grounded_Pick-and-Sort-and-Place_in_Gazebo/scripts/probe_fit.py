#!/usr/bin/env python3
"""Fit and judge the encoder probes exactly as config/probe_protocol.yaml fixes them.

  probe_fit.py <render dir> <features dir> <out .json> [--encoders smolvla_base ...] [--crop '{...}']
               [--layers vision layer4 layer8 ...] [--layer-test]

<features dir> holds one probe_features.py output per encoder: smolvla_base.npz,
36k_008000.npz, 36k_036000.npz. Targets: the four pieces' x, y as Gazebo reported them after
settling (layouts.jsonl); layouts whose pieces drifted more than 5 mm are dropped and counted.
Same 5 folds by layout everywhere. Per encoder, camera, layer and probe: median and 90th
percentile of the per-piece error (mm); paired bootstrap of the median difference against
smolvla_base; the protocol's verdict on the linear probe of the connector layer. With --crop
(probe_protocol.yaml, crop_test), the raw-pixel floor is computed on the same crops as the
features, and the verdict is the crop test's pass rule instead. With --layer-test
(probe_protocol.yaml, layer_probe; smolvla_base features from probe_features.py --layers), each
intermediate layer is compared with the 'vision' layer and the verdict is that addendum's rule.

/workspace/venv_lerobot/bin/python with ROS sourced (tri_scene and the camera models give the
ceiling baseline), inside gazebo_to_lerobot.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from ament_index_python.packages import get_package_share_directory

from mycobot_gateway.vision import tri_scene
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras

PIECES = tri_scene.OBJECTS
CAMERAS = ('top', 'left', 'right')
ENCODERS = ('smolvla_base', '36k_008000', '36k_036000')
LAYERS = ('connector', 'vision')
LAYER_TEST_JUDGED = ('layer4', 'layer8')
DRIFT_MAX_MM = 5.0
FOLDS = 5
ALPHAS = np.logspace(-2, 5, 15)
BOOT = 2000


def folds_of(n):
    perm = np.random.default_rng(0).permutation(n)
    return [np.sort(f) for f in np.array_split(perm, FOLDS)]


def standardise(train, test):
    mu, sd = train.mean(0), train.std(0)
    sd[sd < 1e-8] = 1.0
    return (train - mu) / sd, (test - mu) / sd


def ridge_fit_predict(xtr, ytr, xte, alpha):
    """Dual ridge, n << d: (K + alpha I) c = y - mean."""
    xtr, xte = torch.as_tensor(xtr, dtype=torch.float64), torch.as_tensor(xte, dtype=torch.float64)
    ym = ytr.mean(0)
    k = xtr @ xtr.T
    c = torch.linalg.solve(k + alpha * torch.eye(len(k), dtype=torch.float64),
                           torch.as_tensor(ytr - ym, dtype=torch.float64))
    return (xte @ xtr.T @ c).numpy() + ym


def ridge(xtr, ytr, xte):
    """Alpha by inner 5-fold CV on the training folds (one eigendecomposition per inner fold)."""
    xtr, xte = standardise(xtr, xte)
    inner = folds_of(len(xtr))
    loss = np.zeros(len(ALPHAS))
    for f in inner:
        tr = np.setdiff1d(np.arange(len(xtr)), f)
        a = torch.as_tensor(xtr[tr], dtype=torch.float64)
        b = torch.as_tensor(xtr[f], dtype=torch.float64)
        ym = ytr[tr].mean(0)
        w, v = torch.linalg.eigh(a @ a.T)
        proj = v.T @ torch.as_tensor(ytr[tr] - ym, dtype=torch.float64)
        cross = b @ a.T @ v
        for i, alpha in enumerate(ALPHAS):
            pred = (cross @ (proj / (w + alpha)[:, None])).numpy() + ym
            loss[i] += ((pred - ytr[f]) ** 2).sum()
    alpha = float(ALPHAS[int(np.argmin(loss))])
    return ridge_fit_predict(xtr, ytr, xte, alpha), alpha


def mlp(xtr, ytr, xte):
    xtr, xte = standardise(xtr, xte)
    torch.manual_seed(0)
    a = torch.as_tensor(xtr, dtype=torch.float32)
    mean = a.mean(0)
    k = min(256, len(a) - 1, a.shape[1])
    _, _, v = torch.linalg.svd(a - mean, full_matrices=False)
    pca = lambda x: (torch.as_tensor(x, dtype=torch.float32) - mean) @ v[:k].T  # noqa: E731
    ptr, pte = pca(xtr), pca(xte)
    scale = ptr.std(0).clamp_min(1e-6)
    ptr, pte = ptr / scale, pte / scale
    ym, ys = ytr.mean(0), ytr.std(0)
    y = torch.as_tensor((ytr - ym) / ys, dtype=torch.float32)
    perm = np.random.default_rng(0).permutation(len(ptr))
    val, fit = perm[:len(perm) // 10], perm[len(perm) // 10:]
    net = torch.nn.Sequential(torch.nn.Linear(k, 512), torch.nn.ReLU(), torch.nn.Linear(512, y.shape[1]))
    opt = torch.optim.Adam(net.parameters(), lr=1e-3, weight_decay=1e-4)
    best, best_state, wait = np.inf, None, 0
    for _ in range(5000):
        opt.zero_grad()
        loss = ((net(ptr[fit]) - y[fit]) ** 2).mean()
        loss.backward()
        opt.step()
        with torch.no_grad():
            v_loss = float(((net(ptr[val]) - y[val]) ** 2).mean())
        if v_loss < best - 1e-6:
            best, best_state, wait = v_loss, {n: t.clone() for n, t in net.state_dict().items()}, 0
        else:
            wait += 1
            if wait >= 50:
                break
    net.load_state_dict(best_state)
    with torch.no_grad():
        return net(pte).numpy() * ys + ym


def cross_validate(x, y, probe, folds):
    pred, alphas = np.zeros_like(y), []
    for f in folds:
        tr = np.setdiff1d(np.arange(len(x)), f)
        if probe == 'linear':
            pred[f], alpha = ridge(x[tr], y[tr], x[f])
            alphas.append(alpha)
        elif probe == 'mlp':
            pred[f] = mlp(x[tr], y[tr], x[f])
        else:
            pred[f] = y[tr].mean(0)
    return pred, alphas


def piece_errors(pred, y):
    """(n layouts, 4 pieces) Euclidean errors in mm."""
    return np.linalg.norm((pred - y).reshape(len(y), len(PIECES), 2), axis=2) * 1000


def summary(err):
    return {'median_mm': round(float(np.median(err)), 2), 'p90_mm': round(float(np.percentile(err, 90)), 2)}


def paired_bootstrap(err, base):
    rng = np.random.default_rng(0)
    n = len(err)
    diffs = []
    for _ in range(BOOT):
        i = rng.integers(0, n, n)
        diffs.append(np.median(err[i]) - np.median(base[i]))
    return [round(float(np.percentile(diffs, 2.5)), 2), round(float(np.percentile(diffs, 97.5)), 2)]


def pixel_features(render, seeds, cam, box=None):
    """Raw image (or its crop) box-averaged to 32 x 24 RGB; uncropped, 480 x 640 / 20 exactly."""
    out = []
    for s in seeds:
        img = np.load(render / f'{s}.npz')[cam]
        if box is None:
            out.append(img.astype(np.float32).reshape(24, 20, 32, 20, 3).mean(axis=(1, 3)).ravel())
        else:
            u0, v0, u1, v1 = box
            small = Image.fromarray(img[v0:v1, u0:u1]).resize((32, 24), Image.Resampling.BOX)
            out.append(np.asarray(small, dtype=np.float32).ravel())
    return np.array(out)


def ceiling_features(layouts, cam, cameras, footprints):
    camera = cameras[f'synth_camera_{cam}']
    return np.array([np.concatenate([tri_scene.box_centre(camera, np.asarray(l['gazebo'][p][:2]), footprints[p])
                                     for p in PIECES]) for l in layouts])


def layer_test(result, err_of, layers):
    """probe_protocol.yaml, layer_probe: each smolvla_base layer against its 'vision' layer, linear probe."""
    cells, better = result['cells'], {}
    for layer in layers:
        if layer == 'vision':
            continue
        better[layer] = []
        for cam in CAMERAS:
            ref, name = f'smolvla_base/{cam}/vision/linear', f'smolvla_base/{cam}/{layer}/linear'
            lo, hi = paired_bootstrap(err_of[name], err_of[ref])
            cells[name]['diff_vs_vision_ci95_mm'] = [lo, hi]
            b, d = cells[ref]['median_mm'], cells[name]['median_mm'] - cells[ref]['median_mm']
            if -d > 5 and -d > 0.2 * b and hi < 0:
                better[layer].append(cam)
    clear = {layer: len(cams) >= 2 for layer, cams in better.items()}
    verdict = {'cameras_clearly_better': better, 'clearly_better': clear}
    verdict['proposal'] = ('last 4 layers' if clear['layer8'] else 'whole encoder')
    verdict['judged'] = list(LAYER_TEST_JUDGED)
    result['verdict'] = verdict
    print(json.dumps(verdict, indent=1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('render', type=Path)
    ap.add_argument('features', type=Path)
    ap.add_argument('out', type=Path)
    ap.add_argument('--encoders', nargs='+', default=list(ENCODERS))
    ap.add_argument('--crop', type=json.loads, default=None, help='{camera: [u0, v0, u1, v1]}')
    ap.add_argument('--layers', nargs='+', default=list(LAYERS))
    ap.add_argument('--layer-test', action='store_true')
    args = ap.parse_args()
    encoders = args.encoders
    layouts = [json.loads(line) for line in (args.render / 'layouts.jsonl').read_text().splitlines()]
    kept = [l for l in layouts if l['drift_max_mm'] <= DRIFT_MAX_MM]
    seeds = np.array([l['seed'] for l in kept])
    y = np.array([np.concatenate([l['gazebo'][p][:2] for p in PIECES]) for l in kept])
    folds = folds_of(len(kept))
    desc = Path(get_package_share_directory('mycobot_description'))
    cameras = load_cameras(str(desc / 'urdf' / '320_pi' / 'mycobot_pro_320_pi_gazebo.urdf'),
                           {'camera_layout': 'dream50k'})
    footprints = tri_scene.load_footprints(str(desc / 'models'))
    feats = {}
    for enc in encoders:
        f = np.load(args.features / f'{enc}.npz')
        index = {int(s): i for i, s in enumerate(f['seeds'])}
        rows = [index[int(s)] for s in seeds]
        if args.crop is not None and json.loads(str(f['crop'])) != args.crop:
            raise SystemExit(f'{enc}.npz was computed with crop {f["crop"]}, not {args.crop}')
        feats[enc] = {f'{c}_{layer}': f[f'{c}_{layer}'][rows].astype(np.float32)
                      for c in CAMERAS for layer in args.layers}
    result = {'crop': args.crop, 'layouts_rendered': len(layouts), 'layouts_dropped_drift': len(layouts) - len(kept),
              'layouts_used': len(kept), 'cells': {}}
    err_of = {}

    def run(name, x, probe):
        pred, alphas = cross_validate(x, y, probe, folds)
        err = piece_errors(pred, y)
        err_of[name] = err.ravel()
        result['cells'][name] = {**summary(err), **({'alphas': alphas} if alphas else {})}
        print(name, result['cells'][name], flush=True)

    run('floor_mean', np.zeros((len(y), 1)), 'mean')
    for cam in CAMERAS:
        px = pixel_features(args.render, seeds, cam, (args.crop or {}).get(cam))
        run(f'floor_pixels/{cam}/linear', px, 'linear')
        ceil = ceiling_features(kept, cam, cameras, footprints)
        for probe in ('linear', 'mlp'):
            run(f'ceiling_pixel_position/{cam}/{probe}', ceil, probe)
        for enc in encoders:
            for layer in args.layers:
                for probe in ('linear', 'mlp'):
                    run(f'{enc}/{cam}/{layer}/{probe}', feats[enc][f'{cam}_{layer}'], probe)
    for name in list(result['cells']):
        enc = name.split('/')[0]
        if enc in encoders and enc != 'smolvla_base':
            base = 'smolvla_base/' + name.split('/', 1)[1]
            result['cells'][name]['diff_vs_base_ci95_mm'] = paired_bootstrap(err_of[name], err_of[base])

    if args.layer_test:
        layer_test(result, err_of, args.layers)
        args.out.write_text(json.dumps(result, indent=1) + '\n')
        return
    if args.crop is not None:
        top = result['cells']['smolvla_base/top/connector/linear']['median_mm']
        floor = result['cells']['floor_pixels/top/linear']['median_mm']
        result['verdict'] = {'top_median_mm': top, 'top_cropped_pixel_floor_mm': floor,
                             'median_le_20mm': top <= 20.0, 'beats_floor_by_20pct': top <= 0.8 * floor,
                             'pass': top <= 20.0 and top <= 0.8 * floor}
        print(json.dumps(result['verdict'], indent=1))
        args.out.write_text(json.dumps(result, indent=1) + '\n')
        return
    verdict = {}
    base_ok = [cam for cam in CAMERAS
               if result['cells'][f'smolvla_base/{cam}/connector/linear']['median_mm']
               <= 0.8 * result['cells'][f'floor_pixels/{cam}/linear']['median_mm']]
    verdict['valid'] = len(base_ok) >= 2
    verdict['base_beats_pixels_by_20pct_on'] = base_ok
    for enc in [e for e in encoders if e != 'smolvla_base']:
        worse, better = [], []
        for cam in CAMERAS:
            b = result['cells'][f'smolvla_base/{cam}/connector/linear']['median_mm']
            c = result['cells'][f'{enc}/{cam}/connector/linear']
            lo, hi = c['diff_vs_base_ci95_mm']
            d = c['median_mm'] - b
            if d > 5 and d > 0.2 * b and lo > 0:
                worse.append(cam)
            if -d > 5 and -d > 0.2 * b and hi < 0:
                better.append(cam)
        verdict[enc] = ('not judged: probe invalid' if not verdict['valid'] else
                        'damaged' if len(worse) >= 2 else 'improved' if len(better) >= 2 else
                        'no measurable difference')
        verdict[f'{enc}_cameras_worse'] = worse
        verdict[f'{enc}_cameras_better'] = better
    result['verdict'] = verdict
    print(json.dumps(verdict, indent=1))
    args.out.write_text(json.dumps(result, indent=1) + '\n')


if __name__ == '__main__':
    main()
