#!/usr/bin/env python3
"""Fine-tune a DREAM model on real images using FK-based auto-annotations.

Workflow:
  1. Load real images + joint angles from /tmp/dream_data/real_cam0/
  2. The NDDS annotations already contain FK-projected keypoints
  3. Mix real data with synthetic data (configurable ratio)
  4. Fine-tune from a pretrained checkpoint with lower LR
  5. Apply aggressive style-transfer augmentation to synthetic images

Usage:
  python finetune_real.py \
      --pretrained checkpoints_dream/vgg_weighted_50k_e50/best_network.pth \
      --real-data /tmp/dream_data/real_cam0 \
      --synth-data /tmp/dream_data/synthetic_50k \
      --epochs 30 \
      --output checkpoints_dream/vgg_finetuned_real
"""

import argparse
import glob
import json
import math
import os
import pickle
import random
import socket
import sys
import time
from collections import OrderedDict as odict

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset, ConcatDataset, WeightedRandomSampler
from tqdm import tqdm
from ruamel.yaml import YAML
import albumentations as albu
from PIL import Image as PILImage

DREAM_DIR = "/tmp/DREAM"
sys.path.insert(0, DREAM_DIR)
import dream

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

DEFAULT_KP_WEIGHTS = [1.0, 1.0, 1.0, 1.0, 1.5, 3.0, 5.0]


class WeightedBeliefMapLoss(torch.nn.Module):
    """MSE loss with per-keypoint channel weighting."""

    def __init__(self, kp_weights):
        super().__init__()
        self.register_buffer(
            "weights",
            torch.tensor(kp_weights, dtype=torch.float32).view(1, -1, 1, 1),
        )

    def forward(self, pred, target):
        diff_sq = (pred - target) ** 2
        weighted = diff_sq * self.weights
        return weighted.mean()


# =============================================================================
# Style-transfer augmentation pipeline
# =============================================================================

def _synth_augmentation():
    """Aggressive augmentation for synthetic images to close domain gap."""
    return albu.Compose([
        # --- Colour domain ---
        albu.RandomBrightnessContrast(
            brightness_limit=0.4, contrast_limit=0.4,
            brightness_by_max=False, p=0.8,
        ),
        albu.HueSaturationValue(
            hue_shift_limit=25, sat_shift_limit=40, val_shift_limit=30, p=0.7,
        ),
        albu.RandomGamma(gamma_limit=(60, 140), p=0.4),
        albu.CLAHE(clip_limit=4.0, tile_grid_size=(8, 8), p=0.3),
        albu.ColorJitter(
            brightness=0.3, contrast=0.3, saturation=0.3, hue=0.1, p=0.5,
        ),

        # --- Sensor noise simulation ---
        albu.GaussNoise(p=0.6),
        albu.OneOf([
            albu.GaussianBlur(blur_limit=(3, 7)),
            albu.MotionBlur(blur_limit=(3, 9)),
            albu.MedianBlur(blur_limit=(3, 5)),
        ], p=0.5),
        albu.ImageCompression(quality_range=(40, 90), p=0.4),

        # --- Occlusion simulation ---
        albu.CoarseDropout(
            num_holes_range=(1, 5),
            hole_height_range=(10, 60),
            hole_width_range=(10, 60),
            fill="random",
            p=0.4,
        ),

        # --- Geometric (mild — keypoints are on belief maps) ---
        albu.ShiftScaleRotate(
            shift_limit=0.05, scale_limit=0.1, rotate_limit=15, p=0.5,
        ),

        # --- Channel operations ---
        albu.ChannelShuffle(p=0.05),
        albu.ToGray(p=0.05),
    ], p=1.0)


def _real_augmentation():
    """Mild augmentation for real images (already realistic)."""
    return albu.Compose([
        albu.RandomBrightnessContrast(
            brightness_limit=0.15, contrast_limit=0.15, p=0.5,
        ),
        albu.GaussNoise(p=0.3),
        albu.OneOf([
            albu.GaussianBlur(blur_limit=(3, 5)),
            albu.MedianBlur(blur_limit=3),
        ], p=0.2),
        albu.ShiftScaleRotate(
            shift_limit=0.03, scale_limit=0.05, rotate_limit=10, p=0.3,
        ),
    ], p=1.0)


# =============================================================================
# Mixed dataset
# =============================================================================

class NDDSBeliefMapDataset(Dataset):
    """Load NDDS-format data and produce belief maps for training."""

    def __init__(self, data_dir, keypoint_names, net_input_res, net_output_res,
                 image_norm, image_preproc, augmentation=None, is_real=False,
                 frame_ids=None):
        self.data_dir = data_dir
        self.keypoint_names = keypoint_names
        self.net_input_res = net_input_res   # (W, H)
        self.net_output_res = net_output_res  # (W, H)
        self.image_norm = image_norm
        self.image_preproc = image_preproc
        self.augmentation = augmentation
        self.is_real = is_real

        # Find frames
        if frame_ids is not None:
            self.frame_ids = frame_ids
        else:
            json_files = sorted(glob.glob(os.path.join(data_dir, "??????.json")))
            self.frame_ids = [os.path.basename(f).replace(".json", "") for f in json_files]

        self.n_keypoints = len(keypoint_names)
        self.sigma = 2.0  # Must match DREAM's default sigma=2 for belief maps

    def __len__(self):
        return len(self.frame_ids)

    def __getitem__(self, idx):
        fid = self.frame_ids[idx]
        img_path = os.path.join(self.data_dir, f"{fid}.rgb.png")
        json_path = os.path.join(self.data_dir, f"{fid}.json")

        # Load image
        img_pil = PILImage.open(img_path).convert("RGB")
        raw_w, raw_h = img_pil.size

        # Preprocess (resize/shrink-and-crop)
        img_preproc = dream.image_proc.preprocess_image(
            img_pil, self.net_input_res, self.image_preproc
        )
        netin_w, netin_h = img_preproc.size

        # Load GT keypoints (in raw image coords)
        with open(json_path) as f:
            data = json.load(f)

        gt_projs_raw = []
        for kp in data["objects"][0]["keypoints"]:
            gt_projs_raw.append(kp["projected_location"])

        # Convert raw → net_input coords
        gt_projs_netin = dream.image_proc.convert_keypoints_to_netin_from_raw(
            gt_projs_raw,
            (netin_w, netin_h),
            (raw_w, raw_h),
            self.image_preproc,
        )

        # Convert net_input → net_output coords
        gt_projs_netout = dream.image_proc.convert_keypoints_to_netout_from_netin(
            gt_projs_netin,
            (netin_w, netin_h),
            self.net_output_res,
        )

        # Convert to numpy array
        img_np = np.array(img_preproc)  # (H, W, 3) uint8

        # Apply augmentation
        if self.augmentation is not None:
            augmented = self.augmentation(image=img_np)
            img_np = augmented["image"]

        # Normalize
        img_f = img_np.astype(np.float32) / 255.0
        mean = self.image_norm["mean"]
        stdev = self.image_norm["stdev"]
        for c in range(3):
            img_f[:, :, c] = (img_f[:, :, c] - mean[c]) / stdev[c]

        img_tensor = torch.from_numpy(img_f.transpose(2, 0, 1)).float()

        # Generate belief maps (vectorized)
        out_w, out_h = self.net_output_res
        belief_maps = np.zeros((self.n_keypoints, out_h, out_w), dtype=np.float32)
        iy_grid, ix_grid = np.mgrid[0:out_h, 0:out_w].astype(np.float32)
        for ki in range(self.n_keypoints):
            kp = gt_projs_netout[ki]
            if kp is None or (hasattr(kp, '__len__') and (kp[0] is None or np.isnan(kp[0]))):
                continue
            u, v = float(kp[0]), float(kp[1])
            dist_sq = (ix_grid - u) ** 2 + (iy_grid - v) ** 2
            belief_maps[ki] = np.exp(-dist_sq / (2 * self.sigma ** 2))

        belief_tensor = torch.from_numpy(belief_maps)

        return {
            "image_rgb_input": img_tensor,
            "belief_maps": belief_tensor,
            "is_real": self.is_real,
        }


# =============================================================================
# Fine-tuning
# =============================================================================

def finetune(args):
    """Fine-tune DREAM model on mixed real+synthetic data."""

    print("=" * 70)
    print("DREAM Fine-Tuning: Synthetic → Real Transfer")
    print("=" * 70)

    # Load pretrained model
    config_path = os.path.splitext(args.pretrained)[0] + ".yaml"
    print(f"\nLoading pretrained: {args.pretrained}")
    dream_net = dream.create_network_from_config_file(config_path, args.pretrained)

    # Extract network properties
    net_input_res = dream_net.trained_net_input_resolution()
    net_output_res = dream_net.net_output_resolution_from_input_resolution(net_input_res)
    image_preproc = dream_net.image_preprocessing()
    image_norm = dream_net.image_normalization
    kp_names = dream_net.keypoint_names
    n_kp = dream_net.n_keypoints

    print(f"  Architecture: {dream_net.architecture_type}")
    print(f"  Input/Output: {net_input_res} → {net_output_res}")
    print(f"  Keypoints: {n_kp}")

    # Parse kp weights
    kp_weights = [float(w) for w in args.kp_weights.split(",")]

    # Set up criterion
    weighted_criterion = WeightedBeliefMapLoss(kp_weights).cuda()

    # ---- Build datasets ----
    # Real data
    real_jsons = sorted(glob.glob(os.path.join(args.real_data, "??????.json")))
    real_ids = [os.path.basename(f).replace(".json", "") for f in real_jsons]
    n_real = len(real_ids)

    # Split real: 80% train, 20% val
    n_real_train = int(0.8 * n_real)
    real_train_ids = real_ids[:n_real_train]
    real_val_ids = real_ids[n_real_train:]

    real_train_ds = NDDSBeliefMapDataset(
        args.real_data, kp_names, net_input_res, net_output_res,
        image_norm, image_preproc,
        augmentation=_real_augmentation(), is_real=True,
        frame_ids=real_train_ids,
    )

    real_val_ds = NDDSBeliefMapDataset(
        args.real_data, kp_names, net_input_res, net_output_res,
        image_norm, image_preproc,
        augmentation=None, is_real=True,
        frame_ids=real_val_ids,
    )

    print(f"\n📷 Real data: {n_real} total ({n_real_train} train, {len(real_val_ids)} val)")

    # Synthetic data (optional — for mixed training)
    synth_train_ds = None
    if args.synth_data:
        synth_jsons = sorted(glob.glob(os.path.join(args.synth_data, "??????.json")))
        synth_ids = [os.path.basename(f).replace(".json", "") for f in synth_jsons]
        n_synth = len(synth_ids)
        n_synth_train = int(0.8 * n_synth)
        synth_train_ids = synth_ids[:n_synth_train]

        # Subsample synthetic to match ratio
        if args.synth_ratio < 1.0:
            n_use = int(len(synth_train_ids) * args.synth_ratio)
            synth_train_ids = random.sample(synth_train_ids, min(n_use, len(synth_train_ids)))

        synth_train_ds = NDDSBeliefMapDataset(
            args.synth_data, kp_names, net_input_res, net_output_res,
            image_norm, image_preproc,
            augmentation=_synth_augmentation(), is_real=False,
            frame_ids=synth_train_ids,
        )
        print(f"🖥️  Synth data: {len(synth_train_ids)} train frames (ratio={args.synth_ratio})")

    # Combine datasets with oversampling of real data
    if synth_train_ds is not None:
        combined_ds = ConcatDataset([real_train_ds, synth_train_ds])
        # Oversample real data so each epoch sees ~equal real and synth
        n_r = len(real_train_ds)
        n_s = len(synth_train_ds)
        # Weight real samples higher so they're sampled more often
        real_weight = args.real_weight
        weights = [real_weight] * n_r + [1.0] * n_s
        sampler = WeightedRandomSampler(weights, num_samples=n_r + n_s, replacement=True)
        train_loader = DataLoader(
            combined_ds, batch_size=args.batch_size,
            sampler=sampler, num_workers=args.workers, pin_memory=True,
        )
        print(f"⚖️  Real weight: {real_weight}x (real={n_r}, synth={n_s})")
    else:
        train_loader = DataLoader(
            real_train_ds, batch_size=args.batch_size,
            shuffle=True, num_workers=args.workers, pin_memory=True,
        )

    val_loader = DataLoader(
        real_val_ds, batch_size=args.batch_size,
        num_workers=args.workers, pin_memory=True,
    )

    # ---- Optimizer ----
    # Use lower LR for fine-tuning
    dream_net.enable_training()

    # Differential LR: backbone at lower LR, head at higher LR
    backbone_params = []
    head_params = []
    for name, param in dream_net.model.named_parameters():
        if 'features' in name or 'vgg' in name.lower():
            backbone_params.append(param)
        else:
            head_params.append(param)

    if not head_params:
        # Fallback: all params at same LR
        optimizer = torch.optim.Adam(
            dream_net.model.parameters(), lr=args.lr, weight_decay=1e-5,
        )
        print(f"📈 LR: {args.lr} (all params)")
    else:
        optimizer = torch.optim.Adam([
            {"params": backbone_params, "lr": args.lr * 0.1},
            {"params": head_params, "lr": args.lr},
        ], weight_decay=1e-5)
        print(f"📈 Differential LR: backbone={args.lr * 0.1:.6f}, head={args.lr:.6f}")

    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=args.lr * 0.01,
    )

    os.makedirs(args.output, exist_ok=True)

    # ---- Training loop ----
    print(f"\n{'='*70}")
    print(f"FINE-TUNING: {args.epochs} epochs | BS={args.batch_size}")
    print(f"{'='*70}\n")

    train_log = {
        "epochs": [], "losses": [], "validation_losses": [],
        "lr_history": [], "start_time": time.time(), "timestamps": [],
    }
    best_val_loss = float("inf")

    for epoch in range(args.epochs):
        epoch_num = epoch + 1
        current_lr = optimizer.param_groups[-1]["lr"]

        # Train
        dream_net.model.train()
        train_losses = []
        for batch in tqdm(train_loader, desc=f"Epoch {epoch_num}/{args.epochs} [train]",
                         leave=False):
            imgs = batch["image_rgb_input"].cuda()
            targets = batch["belief_maps"].cuda()

            optimizer.zero_grad()
            output_heads = dream_net.model(imgs)
            # Multi-stage loss (like DREAM's native training)
            if isinstance(output_heads, list) and len(output_heads) > 1:
                n_stages = len(output_heads)
                expanded = targets.unsqueeze(0).expand([n_stages] + [-1] * targets.dim())
                loss = torch.nn.functional.mse_loss(torch.stack(output_heads), expanded)
            else:
                out = output_heads[-1] if isinstance(output_heads, list) else output_heads
                loss = torch.nn.functional.mse_loss(out, targets)
            loss.backward()

            # Gradient clipping
            torch.nn.utils.clip_grad_norm_(dream_net.model.parameters(), max_norm=5.0)
            optimizer.step()
            train_losses.append(loss.item())

        mean_train = np.mean(train_losses)

        # Validate on real data only
        dream_net.model.eval()
        val_losses = []
        with torch.no_grad():
            for batch in val_loader:
                imgs = batch["image_rgb_input"].cuda()
                targets = batch["belief_maps"].cuda()
                output_heads = dream_net.model(imgs)
                if isinstance(output_heads, list) and len(output_heads) > 1:
                    n_stages = len(output_heads)
                    expanded = targets.unsqueeze(0).expand([n_stages] + [-1] * targets.dim())
                    loss = torch.nn.functional.mse_loss(torch.stack(output_heads), expanded)
                else:
                    out = output_heads[-1] if isinstance(output_heads, list) else output_heads
                    loss = torch.nn.functional.mse_loss(out, targets)
                val_losses.append(loss.item())

        mean_val = np.mean(val_losses)
        scheduler.step()

        # Log
        train_log["epochs"].append(epoch_num)
        train_log["losses"].append(float(mean_train))
        train_log["validation_losses"].append(float(mean_val))
        train_log["lr_history"].append(current_lr)
        train_log["timestamps"].append(time.time())

        is_best = mean_val < best_val_loss
        marker = " ⭐ BEST" if is_best else ""
        print(f"Epoch {epoch_num:3d}/{args.epochs} | "
              f"train={mean_train:.6f} | val_real={mean_val:.6f} | "
              f"lr={current_lr:.6f}{marker}")

        if is_best:
            best_val_loss = mean_val
            dream_net.save_network(args.output, "best_network", overwrite=True)
            print(f"  🏆 Best model saved (val={best_val_loss:.6f})")

        if epoch_num % 10 == 0:
            dream_net.save_network(args.output, f"epoch_{epoch_num}", overwrite=True)

    # Save final
    dream_net.save_network(args.output, f"epoch_{args.epochs}", overwrite=True)

    log_path = os.path.join(args.output, "training_log.pkl")
    with open(log_path, "wb") as f:
        pickle.dump(train_log, f)

    elapsed = time.time() - train_log["start_time"]
    print(f"\n{'='*70}")
    print(f"🎉 FINE-TUNING COMPLETE")
    print(f"  Time: {elapsed/60:.1f} min")
    print(f"  Best val loss (real): {best_val_loss:.6f}")
    print(f"  Output: {args.output}")
    print(f"{'='*70}")


def main():
    parser = argparse.ArgumentParser(description="Fine-tune DREAM on real data")
    parser.add_argument("--pretrained", "-p", required=True,
                        help="Pretrained model .pth")
    parser.add_argument("--real-data", "-r", required=True,
                        help="Real NDDS data directory")
    parser.add_argument("--synth-data", "-s", default=None,
                        help="Synthetic NDDS data (optional, for mixed training)")
    parser.add_argument("--epochs", "-e", type=int, default=30)
    parser.add_argument("--batch-size", "-b", type=int, default=16)
    parser.add_argument("--lr", type=float, default=0.00002,
                        help="Fine-tuning LR (default: 2e-5, lower than pretraining)")
    parser.add_argument("--workers", "-w", type=int, default=8)
    parser.add_argument("--output", "-o", required=True)
    parser.add_argument("--kp-weights", default=",".join(str(w) for w in DEFAULT_KP_WEIGHTS))
    parser.add_argument("--synth-ratio", type=float, default=0.2,
                        help="Fraction of synth data to use (default: 0.2)")
    parser.add_argument("--real-weight", type=float, default=5.0,
                        help="Oversampling weight for real data (default: 5.0)")
    args = parser.parse_args()
    finetune(args)


if __name__ == "__main__":
    main()
