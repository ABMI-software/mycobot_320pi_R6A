#!/usr/bin/env python3
"""Encoder features of the probe renders (config/probe_protocol.yaml), one encoder at a time.

  probe_features.py <render dir> <out .npz> --encoder <checkpoint dir | smolvla_base> \
                    [--policy <checkpoint dir>]

The images go through a SmolVLA checkpoint's own path (--policy, default the 13/10 one):
its preprocessor (camera renaming), prepare_images (resize with padding to 512x512, [-1, 1]),
then vision_model and connector, as embed_image does. Only the vision and connector weights
are swapped in from --encoder, so the three encoders see identical inputs. Per camera:
'connector' (64 x 960) and 'vision' (32 x 32 x 768 average-pooled to 8 x 8 x 768), float16,
in render-seed order, with the seeds.

Runs in /workspace/venv_lerobot (no ROS needed).
"""
import argparse
import glob
from pathlib import Path

import numpy as np
import torch
from safetensors.torch import load_file

from smolvla_policy import CAMERAS, SmolVLA

POLICY = '/workspace/checkpoints/run_1310_frozen_20k/020000'
BASE = '/root/.cache/huggingface/hub/models--lerobot--smolvla_base/snapshots/*/model.safetensors'
BATCH = 8


def encoder_tensors(encoder):
    path = glob.glob(BASE)[0] if encoder == 'smolvla_base' else str(Path(encoder) / 'model.safetensors')
    return {k: v for k, v in load_file(path).items() if 'vision_model' in k or 'connector' in k}


@torch.no_grad()
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('render', type=Path)
    ap.add_argument('out', type=Path)
    ap.add_argument('--encoder', required=True)
    ap.add_argument('--policy', default=POLICY)
    args = ap.parse_args()
    torch.set_num_threads(max(1, torch.get_num_threads()))
    model = SmolVLA(args.policy)
    swap = encoder_tensors(args.encoder)
    own = model.policy.state_dict()
    if set(swap) - set(own):
        raise SystemExit(f'encoder tensors not in the policy: {sorted(set(swap) - set(own))[:3]}')
    model.policy.load_state_dict({k: v.to(own[k].dtype) for k, v in swap.items()}, strict=False)
    vlm = model.policy.model.vlm_with_expert.get_vlm_model()
    seeds = sorted(int(p.stem) for p in args.render.glob('*.npz'))
    feats = {f'{c}_{layer}': [] for c in CAMERAS for layer in ('connector', 'vision')}
    for i in range(0, len(seeds), BATCH):
        renders = [np.load(args.render / f'{s}.npz') for s in seeds[i:i + BATCH]]
        obs = {f'observation.images.{c}': torch.stack([
            torch.from_numpy(np.ascontiguousarray(r[c])).permute(2, 0, 1).float() / 255.0 for r in renders])
            for c in CAMERAS}
        obs['observation.state'] = torch.zeros(len(renders), 7)
        obs['task'] = ['probe'] * len(renders)
        batch = model.pre(obs)
        images, _ = model.policy.prepare_images(batch)
        # prepare_images follows config.image_features: camera1/2/3 = top/right/left after renaming.
        for cam, img in zip(CAMERAS, images):
            hidden = vlm.vision_model(pixel_values=img.to(vlm.vision_model.dtype)).last_hidden_state
            tokens = vlm.connector(hidden)
            grid = hidden.reshape(len(renders), 32, 32, -1).permute(0, 3, 1, 2)
            pooled = torch.nn.functional.avg_pool2d(grid, 4).permute(0, 2, 3, 1)
            feats[f'{cam}_connector'].append(tokens.flatten(1).half().numpy())
            feats[f'{cam}_vision'].append(pooled.flatten(1).half().numpy())
        print(f'{min(i + BATCH, len(seeds))}/{len(seeds)}', flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, seeds=np.array(seeds), encoder=args.encoder, policy=args.policy,
             **{k: np.concatenate(v) for k, v in feats.items()})


if __name__ == '__main__':
    main()
