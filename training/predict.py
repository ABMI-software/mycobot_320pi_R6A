#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run inference on a single image to predict MyCobot joint angles.

Usage::

    python3 training/predict.py \\
        --image /tmp/mycobot_synth_dataset/images/000042.png \\
        --checkpoint training/checkpoints/best_model.pth

    # Or on multiple images:
    python3 training/predict.py \\
        --image /tmp/mycobot_synth_dataset/images/000*.png \\
        --checkpoint training/checkpoints/best_model.pth
"""

import argparse
import glob
import math
import os
import sys
from pathlib import Path

import numpy as np
import torch
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dataset import denormalize_angles, get_eval_transforms
from model import PoseResNet


def predict_single(model, image_path: str, transform, device) -> np.ndarray:
    """Predict joint angles (radians) from a single image."""
    img = Image.open(image_path).convert('RGB')
    tensor = transform(img).unsqueeze(0).to(device)

    with torch.no_grad():
        normed = model(tensor).cpu().numpy()[0]

    return denormalize_angles(normed)


def main():
    parser = argparse.ArgumentParser(
        description='Predict MyCobot joint angles from an image')
    parser.add_argument('--image', nargs='+', required=True,
                        help='Path(s) to image file(s)')
    parser.add_argument('--checkpoint', required=True,
                        help='Path to trained model checkpoint')
    parser.add_argument('--backbone', default='resnet18',
                        choices=['resnet18', 'resnet34', 'resnet50'])
    parser.add_argument('--image-size', type=int, default=224)
    args = parser.parse_args()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # Load model
    model = PoseResNet(backbone=args.backbone, pretrained=False).to(device)
    ckpt = torch.load(args.checkpoint, map_location=device, weights_only=True)
    model.load_state_dict(ckpt['model_state_dict'])
    model.eval()
    print(f'✅ Model loaded from {args.checkpoint}  (device={device})\n')

    transform = get_eval_transforms(args.image_size)

    # Expand globs
    image_paths = []
    for pattern in args.image:
        expanded = sorted(glob.glob(pattern))
        image_paths.extend(expanded if expanded else [pattern])

    # Predict
    for img_path in image_paths:
        if not os.path.isfile(img_path):
            print(f'⚠️  File not found: {img_path}')
            continue

        angles_rad = predict_single(model, img_path, transform, device)
        angles_deg = np.degrees(angles_rad)

        fname = os.path.basename(img_path)
        print(f'📸 {fname}')
        print(f'   Radians: [{", ".join(f"{a:.4f}" for a in angles_rad)}]')
        print(f'   Degrees: [{", ".join(f"{a:.1f}" for a in angles_deg)}]')
        print()


if __name__ == '__main__':
    main()
