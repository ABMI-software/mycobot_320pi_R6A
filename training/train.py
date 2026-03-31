#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Training script for MyCobot 320 Pi pose estimation.

Usage
-----
Train::

    python3 training/train.py --dataset /tmp/mycobot_synth_dataset --epochs 100

Evaluate only::

    python3 training/train.py --dataset /tmp/mycobot_synth_dataset \\
        --evaluate --checkpoint training/checkpoints/best_model.pth

See ``--help`` for all options.
"""

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader, Subset

# Local imports
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dataset import (
    MyCobotPoseDataset,
    denormalize_angles,
    normalize_angles,
    get_eval_transforms,
    get_train_transforms,
    JOINT_RANGE,
)
from model import PoseResNet
from PIL import Image


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def mae_per_joint_deg(pred: torch.Tensor, target: torch.Tensor,
                       normalized: bool = True) -> np.ndarray:
    """Compute mean-absolute-error per joint in **degrees**.

    If *normalized* is True, pred/target are in [-1, 1] and will be
    converted back to radians first.
    """
    with torch.no_grad():
        err = (pred - target).abs()
        if normalized:
            # Scale back: normed * (range/2) → radians
            half_range = torch.tensor(JOINT_RANGE / 2.0,
                                      device=pred.device)
            err = err * half_range
        err_deg = err * (180.0 / math.pi)
        return err_deg.mean(dim=0).cpu().numpy()


# ---------------------------------------------------------------------------
# Train one epoch
# ---------------------------------------------------------------------------
def train_one_epoch(model, loader, criterion, optimizer, device, scaler):
    model.train()
    running_loss = 0.0
    n_samples = 0
    all_preds, all_targets = [], []

    for images, targets in loader:
        images = images.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)

        optimizer.zero_grad(set_to_none=True)

        with torch.amp.autocast('cuda', enabled=scaler is not None):
            preds = model(images)
            loss = criterion(preds, targets)

        if scaler is not None:
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
        else:
            loss.backward()
            optimizer.step()

        bs = images.size(0)
        running_loss += loss.item() * bs
        n_samples += bs
        all_preds.append(preds.detach())
        all_targets.append(targets.detach())

    epoch_loss = running_loss / n_samples
    all_preds = torch.cat(all_preds)
    all_targets = torch.cat(all_targets)
    epoch_mae = mae_per_joint_deg(all_preds, all_targets)
    return epoch_loss, epoch_mae


# ---------------------------------------------------------------------------
# Validate
# ---------------------------------------------------------------------------
@torch.no_grad()
def validate(model, loader, criterion, device):
    model.eval()
    running_loss = 0.0
    n_samples = 0
    all_preds, all_targets = [], []

    for images, targets in loader:
        images = images.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)

        preds = model(images)
        loss = criterion(preds, targets)

        bs = images.size(0)
        running_loss += loss.item() * bs
        n_samples += bs
        all_preds.append(preds)
        all_targets.append(targets)

    epoch_loss = running_loss / n_samples
    all_preds = torch.cat(all_preds)
    all_targets = torch.cat(all_targets)
    epoch_mae = mae_per_joint_deg(all_preds, all_targets)
    return epoch_loss, epoch_mae


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_training_curves(log_path: str, out_path: str):
    """Read training_log.csv and produce loss + MAE plots."""
    epochs, train_loss, val_loss = [], [], []
    train_mae, val_mae = [], []

    with open(log_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            epochs.append(int(row['epoch']))
            train_loss.append(float(row['train_loss']))
            val_loss.append(float(row['val_loss']))
            train_mae.append(float(row['train_mae_mean_deg']))
            val_mae.append(float(row['val_mae_mean_deg']))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    ax1.plot(epochs, train_loss, label='Train Loss')
    ax1.plot(epochs, val_loss, label='Val Loss')
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('MSE Loss')
    ax1.set_title('Training & Validation Loss')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(epochs, train_mae, label='Train MAE')
    ax2.plot(epochs, val_mae, label='Val MAE')
    ax2.set_xlabel('Epoch')
    ax2.set_ylabel('Mean MAE (degrees)')
    ax2.set_title('Mean Absolute Error (degrees)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f'📈 Training curves saved to {out_path}')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Train MyCobot 320 Pi pose estimation model')
    parser.add_argument('--dataset', required=True,
                        help='Path to dataset directory')
    parser.add_argument('--checkpoint', default=None,
                        help='Path to checkpoint to resume / evaluate')
    parser.add_argument('--evaluate', action='store_true',
                        help='Evaluate only (requires --checkpoint)')
    # Model
    parser.add_argument('--backbone', default='resnet18',
                        choices=['resnet18', 'resnet34', 'resnet50'])
    parser.add_argument('--image-size', type=int, default=224)
    parser.add_argument('--dropout', type=float, default=0.3)
    # Training
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--lr', type=float, default=1e-4)
    parser.add_argument('--weight-decay', type=float, default=1e-4)
    parser.add_argument('--patience', type=int, default=15,
                        help='Early stopping patience (0 = disabled)')
    parser.add_argument('--val-split', type=float, default=0.2)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--workers', type=int, default=4)
    # Backbone fine-tuning
    parser.add_argument('--freeze-epochs', type=int, default=5,
                        help='Freeze backbone for first N epochs')
    # Output
    parser.add_argument('--output-dir', default='training/checkpoints',
                        help='Directory for model checkpoints and logs')
    parser.add_argument('--amp', action='store_true', default=True,
                        help='Use automatic mixed precision (default: on)')
    parser.add_argument('--no-amp', dest='amp', action='store_false')

    args = parser.parse_args()

    # ---------- Setup ----------
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'🖥️  Device: {device}'
          f'{" — " + torch.cuda.get_device_name(0) if device.type == "cuda" else ""}')

    os.makedirs(args.output_dir, exist_ok=True)

    # ---------- Dataset ----------
    full_dataset = MyCobotPoseDataset(
        args.dataset,
        transform=None,        # set per split below
        normalize_targets=True,
    )
    n = len(full_dataset)
    indices = list(range(n))
    train_idx, val_idx = train_test_split(
        indices, test_size=args.val_split,
        random_state=args.seed,
    )

    # Wrap subsets with appropriate transforms
    train_tf = get_train_transforms(args.image_size)
    eval_tf = get_eval_transforms(args.image_size)

    train_set = _TransformSubset(full_dataset, train_idx, train_tf)
    val_set   = _TransformSubset(full_dataset, val_idx, eval_tf)

    train_loader = DataLoader(
        train_set, batch_size=args.batch_size, shuffle=True,
        num_workers=args.workers, pin_memory=True, drop_last=True,
    )
    val_loader = DataLoader(
        val_set, batch_size=args.batch_size, shuffle=False,
        num_workers=args.workers, pin_memory=True,
    )
    print(f'📂 Dataset: {n} samples  (train={len(train_set)}, val={len(val_set)})')

    # ---------- Model ----------
    model = PoseResNet(
        backbone=args.backbone,
        pretrained=True,
        dropout=args.dropout,
    ).to(device)
    print(f'🧠 {model}')

    # ---------- Evaluate only ----------
    if args.evaluate:
        if not args.checkpoint:
            parser.error('--evaluate requires --checkpoint')
        state = torch.load(args.checkpoint, map_location=device, weights_only=True)
        model.load_state_dict(state['model_state_dict'])
        criterion = nn.MSELoss()
        val_loss, val_mae = validate(model, val_loader, criterion, device)
        print(f'\n📊 Evaluation on {len(val_set)} samples:')
        print(f'   Val Loss  : {val_loss:.6f}')
        print(f'   Val MAE   : {val_mae.mean():.2f}° (mean)')
        for i, m in enumerate(val_mae):
            print(f'     Joint {i+1} : {m:.2f}°')
        return

    # ---------- Optimiser & scheduler ----------
    criterion = nn.MSELoss()
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay,
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=1e-6,
    )
    scaler = torch.amp.GradScaler('cuda') if (args.amp and device.type == 'cuda') else None

    # ---------- Optional backbone freeze ----------
    if args.freeze_epochs > 0:
        model.freeze_backbone()
        print(f'🔒 Backbone frozen for first {args.freeze_epochs} epochs')

    # ---------- Resume from checkpoint ----------
    start_epoch = 0
    best_val_loss = float('inf')
    if args.checkpoint and os.path.isfile(args.checkpoint):
        ckpt = torch.load(args.checkpoint, map_location=device, weights_only=True)
        model.load_state_dict(ckpt['model_state_dict'])
        optimizer.load_state_dict(ckpt['optimizer_state_dict'])
        start_epoch = ckpt.get('epoch', 0) + 1
        best_val_loss = ckpt.get('best_val_loss', best_val_loss)
        print(f'♻️  Resumed from epoch {start_epoch}')

    # ---------- Logging ----------
    log_path = os.path.join(args.output_dir, 'training_log.csv')
    log_file = open(log_path, 'w', newline='')
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        'epoch', 'train_loss', 'val_loss',
        'train_mae_mean_deg', 'val_mae_mean_deg',
        'train_mae_j1', 'train_mae_j2', 'train_mae_j3',
        'train_mae_j4', 'train_mae_j5', 'train_mae_j6',
        'val_mae_j1', 'val_mae_j2', 'val_mae_j3',
        'val_mae_j4', 'val_mae_j5', 'val_mae_j6',
        'lr',
    ])

    # ---------- Training loop ----------
    patience_counter = 0
    t_start = time.time()

    print(f'\n🚀 Training for {args.epochs} epochs …\n')

    for epoch in range(start_epoch, args.epochs):
        # Unfreeze backbone after N epochs
        if epoch == args.freeze_epochs and args.freeze_epochs > 0:
            model.unfreeze_backbone()
            print(f'🔓 Backbone unfrozen at epoch {epoch}')

        t0 = time.time()
        train_loss, train_mae = train_one_epoch(
            model, train_loader, criterion, optimizer, device, scaler,
        )
        val_loss, val_mae = validate(model, val_loader, criterion, device)
        scheduler.step()

        lr = optimizer.param_groups[0]['lr']
        dt = time.time() - t0

        # Log
        log_writer.writerow([
            epoch, f'{train_loss:.6f}', f'{val_loss:.6f}',
            f'{train_mae.mean():.2f}', f'{val_mae.mean():.2f}',
            *[f'{m:.2f}' for m in train_mae],
            *[f'{m:.2f}' for m in val_mae],
            f'{lr:.2e}',
        ])
        log_file.flush()

        # Console output
        improved = ''
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
            improved = ' ⭐ best'
            # Save best
            _save_checkpoint(model, optimizer, epoch, best_val_loss,
                             os.path.join(args.output_dir, 'best_model.pth'))
        else:
            patience_counter += 1

        print(f'  Epoch {epoch:3d}/{args.epochs}  '
              f'train_loss={train_loss:.5f}  val_loss={val_loss:.5f}  '
              f'mae_train={train_mae.mean():.1f}°  mae_val={val_mae.mean():.1f}°  '
              f'lr={lr:.1e}  ({dt:.1f}s){improved}')

        # Early stopping
        if args.patience > 0 and patience_counter >= args.patience:
            print(f'\n⏹️  Early stopping at epoch {epoch} '
                  f'(no improvement for {args.patience} epochs)')
            break

    log_file.close()

    # Save last model
    _save_checkpoint(model, optimizer, epoch, best_val_loss,
                     os.path.join(args.output_dir, 'last_model.pth'))

    elapsed = time.time() - t_start
    print(f'\n✅ Training complete in {elapsed/60:.1f} min')
    print(f'   Best val loss : {best_val_loss:.6f}')
    print(f'   Checkpoints   : {args.output_dir}/')

    # Final evaluation with best model
    best_path = os.path.join(args.output_dir, 'best_model.pth')
    if os.path.exists(best_path):
        state = torch.load(best_path, map_location=device, weights_only=True)
        model.load_state_dict(state['model_state_dict'])
        val_loss, val_mae = validate(model, val_loader, criterion, device)
        print(f'\n📊 Best model evaluation on {len(val_set)} val samples:')
        print(f'   Val Loss : {val_loss:.6f}')
        print(f'   Mean MAE : {val_mae.mean():.2f}°')
        for i, m in enumerate(val_mae):
            print(f'     Joint {i+1}: {m:.2f}°')

    # Plot curves
    plot_training_curves(log_path,
                         os.path.join(args.output_dir, 'training_curves.png'))


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
class _TransformSubset:
    """A Subset that overrides the dataset's transform per sample."""

    def __init__(self, dataset: MyCobotPoseDataset, indices, transform):
        self.dataset = dataset
        self.indices = indices
        self.transform = transform

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        real_idx = self.indices[idx]
        img_rel, angles = self.dataset.samples[real_idx]
        img_path = os.path.join(self.dataset.dataset_dir, img_rel)

        image = Image.open(img_path).convert('RGB')
        image = self.transform(image)

        if self.dataset.normalize_targets:
            angles = normalize_angles(angles)

        target = torch.from_numpy(angles)
        return image, target


def _save_checkpoint(model, optimizer, epoch, best_val_loss, path):
    torch.save({
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'best_val_loss': best_val_loss,
    }, path)


if __name__ == '__main__':
    main()
