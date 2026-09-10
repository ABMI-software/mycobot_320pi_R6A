#!/usr/bin/env python3
"""DREAM training with per-keypoint weighted loss (Step B3) — v3.

v3 changes vs v2:
  - Added native --seed argument (fixes random/numpy/torch seeds for
    reproducibility). No environment variable needed.

Applies higher loss weights to distal keypoints (link5=Joint5, link6=EndEffector)
which are the hardest to detect and most important for manipulation accuracy.

Combines:
  B3: Weighted loss for distal keypoints
  B1: Longer training (50-100 epochs) with cosine annealing LR
  Strong data augmentation for sim-to-real transfer

The belief map tensor has shape (N, K, H, W) where K=7 keypoints.
We multiply the per-keypoint MSE by a weight vector before averaging.

Default weights:
  base=1.0, link1=1.0, link2=1.0, link3=1.0, link4=1.5, link5=1.5, link6=6.0

Usage:
  python train_dream_ultimate_v3.py \
      --data /tmp/dream_data/synthetic \
      --arch vgg \
      --epochs 50 \
      --batch-size 32 \
      --lr 0.0001 \
      --seed 42 \
      --output checkpoints_dream/vgg_weighted_e50

  # Custom weights: base,link1,link2,link3,link4,link5,link6
  python train_dream_ultimate_v3.py \
      --data /tmp/dream_data/synthetic \
      --kp-weights 1,1,1,1,2,4,8 \
      --epochs 75 \
      --seed 42 \
      --output checkpoints_dream/vgg_weighted_custom_e75
"""

import sys
import os
import time
import pickle
import random
import socket
from collections import OrderedDict as odict

import numpy as np
import torch
from torch.utils.data import DataLoader as TorchDataLoader
from tqdm import tqdm
from ruamel.yaml import YAML
import albumentations as albu

DREAM_DIR = "/tmp/DREAM"
sys.path.insert(0, DREAM_DIR)
import dream

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Default per-keypoint loss weights
# Index: 0=base, 1=link1, 2=link2, 3=link3, 4=link4, 5=link5, 6=link6
DEFAULT_KP_WEIGHTS = [1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 6.0]


def set_global_seed(seed):
    """Fixe les seeds random/numpy/torch pour reproductibilité."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    print(f"🌱 Seed fixé: {seed}")


class EarlyStopping:
    def __init__(self, patience=5, min_delta=1e-6):
        self.patience = patience
        self.min_delta = min_delta
        self.counter = 0
        self.best_loss = float("inf")
        self.best_epoch = 0
        self.should_stop = False

    def __call__(self, val_loss, epoch):
        if val_loss < self.best_loss - self.min_delta:
            self.best_loss = val_loss
            self.best_epoch = epoch
            self.counter = 0
            return True
        else:
            self.counter += 1
            print(f"  ⏳ Pas d'amélioration ({self.counter}/{self.patience})")
            if self.counter >= self.patience:
                self.should_stop = True
                print(f"  🛑 Early stop → meilleur epoch : {self.best_epoch}")
            return False


class WeightedBeliefMapLoss(torch.nn.Module):
    """MSE loss with per-keypoint channel weighting.

    For belief maps of shape (N, K, H, W), applies a weight per channel K
    so that the loss for harder-to-detect distal keypoints is amplified.
    """

    def __init__(self, kp_weights):
        super().__init__()
        # weights shape: (K,) → will be broadcast to (1, K, 1, 1)
        self.register_buffer(
            "weights",
            torch.tensor(kp_weights, dtype=torch.float32).view(1, -1, 1, 1),
        )

    def forward(self, pred, target):
        # pred, target: (N, K, H, W)
        diff_sq = (pred - target) ** 2  # (N, K, H, W)
        weighted = diff_sq * self.weights  # broadcast (1,K,1,1)
        return weighted.mean()


def _patch_augmentation():
    """Patch ManipulatorNDDSDataset to use stronger augmentation."""
    import dream.datasets as dream_datasets
    import numpy as np

    original_getitem = dream_datasets.ManipulatorNDDSDataset.__getitem__

    def patched_getitem(self, idx):
        if self.augment_data:
            self.augment_data = False
            sample = original_getitem(self, idx)
            self.augment_data = True

            img_tensor = sample["image_rgb_input"]
            img_np = img_tensor.permute(1, 2, 0).numpy() * 0.5 + 0.5
            img_np = (img_np * 255).clip(0, 255).astype(np.uint8)

            strong_aug = albu.Compose([
                albu.GaussNoise(p=0.5),
                albu.RandomBrightnessContrast(
                    brightness_limit=0.3,
                    contrast_limit=0.3,
                    brightness_by_max=False,
                    p=0.7,
                ),
                albu.HueSaturationValue(
                    hue_shift_limit=20,
                    sat_shift_limit=30,
                    val_shift_limit=20,
                    p=0.5,
                ),
                albu.OneOf([
                    albu.GaussianBlur(blur_limit=(3, 7)),
                    albu.MotionBlur(blur_limit=(3, 7)),
                    albu.MedianBlur(blur_limit=(3, 5)),
                ], p=0.3),
                albu.CLAHE(clip_limit=4.0, p=0.2),
                albu.RandomGamma(gamma_limit=(70, 130), p=0.3),
                albu.ImageCompression(quality_range=(50, 95), p=0.3),
                albu.CoarseDropout(
                    num_holes_range=(2, 6),
                    hole_height_range=(15, 60),
                    hole_width_range=(15, 60),
                    fill="random",
                    p=0.4,
                ),
                # NB: geometric transforms (Perspective, ShiftScaleRotate) were removed.
                # They warp the image but the belief-map labels are NOT re-transformed
                # here, so every geometrically-augmented sample had MISALIGNED labels —
                # worst for distal keypoints (a 15° rotation moves the end effector ~40px
                # while the base barely moves). Only label-preserving (photometric +
                # occlusion) transforms remain.
            ], p=1.0)

            augmented = strong_aug(image=img_np)
            img_aug = augmented["image"]

            img_aug_f = img_aug.astype(np.float32) / 255.0
            img_aug_f = (img_aug_f - 0.5) / 0.5
            sample["image_rgb_input"] = torch.from_numpy(
                img_aug_f.transpose(2, 0, 1)
            ).float()

            return sample
        else:
            return original_getitem(self, idx)

    dream_datasets.ManipulatorNDDSDataset.__getitem__ = patched_getitem
    print("[PATCH] Augmentation pipeline → strong sim-to-real transforms")


def _patch_belief_map_sigma(sigmas):
    """Per-keypoint belief-map sigma.

    Distal keypoints (link4/5/6) collapse: the net is too uncertain to commit a
    sharp peak. A wider target gaussian for those channels rewards being roughly
    right over a larger area, so the net forms a (broader) peak instead of
    hedging flat. Proximal keypoints keep sigma=2 (precise). Trades a little
    distal precision for detectability — exactly the failing axis."""
    import dream.image_proc as ip
    import numpy as np

    def create_belief_map_per_kp(image_resolution, pointsBelief, sigma=2):
        image_width, image_height = image_resolution
        out = np.zeros((len(pointsBelief), image_height, image_width))
        for i_point, point in enumerate(pointsBelief):
            s = sigmas[i_point] if i_point < len(sigmas) else sigma
            w = int(s * 2)
            pixel_u, pixel_v = int(point[0]), int(point[1])
            array = np.zeros((image_height, image_width))
            if (pixel_u - w >= 0 and pixel_u + w + 1 < image_width
                    and pixel_v - w >= 0 and pixel_v + w + 1 < image_height):
                for i in range(pixel_u - w, pixel_u + w + 1):
                    for j in range(pixel_v - w, pixel_v + w + 1):
                        array[j, i] = np.exp(
                            -(((i - pixel_u) ** 2 + (j - pixel_v) ** 2)
                              / (2 * (s ** 2))))
            out[i_point] = array
        return out

    ip.create_belief_map = create_belief_map_per_kp
    print(f"[PATCH] Per-keypoint belief-map sigma → {sigmas}")


def train_weighted(args):
    """Main training loop with weighted loss."""

    # Fixe le seed en tout premier, avant tout chargement de données/modèle
    set_global_seed(args.seed)

    # Apply augmentation patch
    _patch_augmentation()

    # Apply per-keypoint belief-map sigma (wider targets for distal keypoints)
    kp_sigmas = [float(s) for s in args.kp_sigmas.split(",")]
    assert len(kp_sigmas) == 7, f"Expected 7 keypoint sigmas, got {len(kp_sigmas)}"
    if any(s != 2.0 for s in kp_sigmas):
        _patch_belief_map_sigma(kp_sigmas)

    # Configs
    MANIP_CONFIG = os.path.join(SCRIPT_DIR, "manip_configs", "mycobot320.yaml")
    arch_map = {
        "vgg": os.path.join(DREAM_DIR, "arch_configs", "dream_vgg_q.yaml"),
        "resnet": os.path.join(DREAM_DIR, "arch_configs", "dream_resnet_h.yaml"),
    }
    arch_config_path = arch_map[args.arch]

    os.makedirs(args.output, exist_ok=True)

    # Parse keypoint weights
    kp_weights = [float(w) for w in args.kp_weights.split(",")]
    assert len(kp_weights) == 7, f"Expected 7 keypoint weights, got {len(kp_weights)}"
    print(f"\n🎯 Per-keypoint loss weights:")
    kp_names = ["base", "link1", "link2", "link3", "link4", "link5", "link6"]
    for name, w in zip(kp_names, kp_weights):
        marker = "⬆️" if w > 1.0 else "  "
        print(f"  {marker} {name:<8s}: {w:.1f}x")
    print()

    # Parse configs
    yaml_parser = YAML(typ="safe")
    with open(MANIP_CONFIG) as f:
        manipulator_config_file = yaml_parser.load(f)
    manipulator_config = manipulator_config_file["manipulator"]

    with open(arch_config_path) as f:
        architecture_config_file = yaml_parser.load(f)
    architecture_config = architecture_config_file["architecture"]
    training_config = architecture_config_file["training"]["config"]

    training_image_preprocessing = training_config["image_preprocessing"]
    training_net_input_resolution = training_config["net_input_resolution"]

    architecture_config["image_preprocessing"] = training_image_preprocessing

    # Data augmentation config
    data_augment_config = odict([("image_rgb", True)])

    # Find data
    found_data = dream.utilities.find_ndds_data_in_dir(args.data)
    found_data_config = found_data[1]
    image_raw_resolution = dream.utilities.load_image_resolution(
        found_data_config["camera"]
    )

    try:
        user = os.getlogin()
    except:
        user = "unknown"

    # Build network config
    network_config = odict([
        ("data_path", args.data),
        ("manipulator", manipulator_config),
        ("architecture", architecture_config),
        ("training", odict([
            ("config", odict([
                ("epochs", args.epochs),
                ("training_data_fraction", args.training_fraction),
                ("validation_data_fraction", 1.0 - args.training_fraction),
                ("batch_size", args.batch_size),
                ("data_augmentation", data_augment_config),
                ("worker_size", args.workers),
                ("optimizer", odict([
                    ("type", "adam"),
                    ("learning_rate", args.lr),
                ])),
                ("image_preprocessing", training_image_preprocessing),
                ("image_raw_resolution", list(image_raw_resolution)),
                ("net_input_resolution", training_net_input_resolution),
                ("keypoint_weights", kp_weights),
                ("seed", args.seed),
            ])),
            ("platform", odict([
                ("user", user),
                ("hostname", socket.gethostname()),
                ("gpu_ids", [args.gpu]),
            ])),
            ("results", odict([("epochs_trained", 0)])),
        ])),
    ])

    # Create network
    print(f"Creating {args.arch} network...")
    dream_network = dream.create_network_from_config_data(network_config)

    # Replace the criterion with our weighted loss
    weighted_criterion = WeightedBeliefMapLoss(kp_weights).cuda()
    dream_network.criterion = weighted_criterion
    print(f"✅ Weighted loss installed (weights sum={sum(kp_weights):.1f})")

    dream_network.enable_training()

    # Set up cosine annealing scheduler
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        dream_network.optimizer,
        T_max=args.epochs,
        eta_min=args.lr * 0.01,  # min LR = 1% of initial
    )
    print(f"📈 Cosine annealing LR: {args.lr} → {args.lr * 0.01:.6f}")

    # Compute resolutions
    trained_net_input_res, trained_net_output_res = \
        dream_network.net_resolutions_from_image_raw_resolution(image_raw_resolution)
    dream_network.network_config["training"]["config"]["net_output_resolution"] = \
        trained_net_output_res

    # Create dataset
    found_dataset = dream.datasets.ManipulatorNDDSDataset(
        found_data,
        manipulator_config["name"],
        dream_network.keypoint_names,
        trained_net_input_res,
        trained_net_output_res,
        dream_network.image_normalization,
        dream_network.image_preprocessing(),
        augment_data=True,
        include_ground_truth=True,
        include_belief_maps=True,
    )

    # Split train/val — uses a seeded generator so the split is reproducible
    n_data = len(found_dataset)
    n_train = int(round(n_data * args.training_fraction))
    n_valid = n_data - n_train
    split_generator = torch.Generator().manual_seed(args.seed)
    train_dataset, valid_dataset = torch.utils.data.random_split(
        found_dataset, [n_train, n_valid], generator=split_generator
    )

    train_loader = TorchDataLoader(
        train_dataset, batch_size=args.batch_size, num_workers=args.workers,
        shuffle=True, pin_memory=True, persistent_workers=args.workers > 0,
    )
    valid_loader = TorchDataLoader(
        valid_dataset, batch_size=args.batch_size, num_workers=args.workers,
        pin_memory=True, persistent_workers=args.workers > 0,
    )

    print(f"\n📊 Dataset: {n_data} total ({n_train} train, {n_valid} valid)")
    print(f"   Input res:  {trained_net_input_res}")
    print(f"   Output res: {trained_net_output_res}")
    print(f"   Batches:    {len(train_loader)} train, {len(valid_loader)} valid")
    print(f"\n{'='*70}")
    print(f"TRAINING: {args.arch} | {args.epochs} epochs | BS={args.batch_size} | LR={args.lr} | seed={args.seed}")
    print(f"  Weighted loss: {kp_weights}")
    print(f"  Cosine annealing LR schedule")
    print(f"  Strong sim-to-real augmentation")
    print(f"{'='*70}\n")

    # Training log
    train_log = {
        "epochs": [],
        "losses": [],
        "validation_losses": [],
        "per_kp_val_losses": [],
        "lr_history": [],
        "start_time": time.time(),
        "timestamps": [],
        "keypoint_weights": kp_weights,
        "seed": args.seed,
    }

    best_valid_loss = float("inf")
    early_stopping = EarlyStopping(patience=args.patience)

    # Mixed precision (AMP) + TF32: ~1.5-2x faster on RTX 4000 Ada, ~half the VRAM.
    # Weights stay fp32; only the forward/loss compute runs in fp16. Eval is separate
    # and full-precision, so detection % / PnP are unaffected.
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.backends.cudnn.benchmark = True
    scaler = torch.amp.GradScaler("cuda")

    for epoch in range(args.epochs):
        epoch_num = epoch + 1
        current_lr = dream_network.optimizer.param_groups[0]["lr"]

        # ---- Training ----
        dream_network.enable_training()
        train_losses = []

        for batch in tqdm(train_loader, desc=f"Epoch {epoch_num}/{args.epochs} [train]",
                         leave=False):
            network_input = [batch["image_rgb_input"].cuda()]
            target = batch["belief_maps"].cuda()

            dream_network.optimizer.zero_grad()

            # Forward pass + weighted loss under autocast (fp16)
            with torch.amp.autocast("cuda"):
                output = dream_network.model(network_input[0])
                pred = output[0] if isinstance(output, list) else output
                loss = weighted_criterion(pred, target)

            scaler.scale(loss).backward()
            scaler.step(dream_network.optimizer)
            scaler.update()

            train_losses.append(loss.item())

        mean_train_loss = np.mean(train_losses)

        # ---- Validation ----
        dream_network.enable_evaluation()
        valid_losses = []
        per_kp_losses_accum = np.zeros(7)
        per_kp_counts = 0

        with torch.no_grad():
            for batch in tqdm(valid_loader, desc=f"Epoch {epoch_num}/{args.epochs} [valid]",
                             leave=False):
                network_input = [batch["image_rgb_input"].cuda()]
                target = batch["belief_maps"].cuda()

                with torch.amp.autocast("cuda"):
                    output = dream_network.model(network_input[0])
                    pred = output[0] if isinstance(output, list) else output
                    loss = weighted_criterion(pred, target)
                pred = pred.float()  # per-KP MSE below in fp32 for stable logging
                valid_losses.append(loss.item())

                # Per-keypoint unweighted MSE for monitoring
                per_kp_mse = ((pred - target) ** 2).mean(dim=(0, 2, 3))  # (K,)
                per_kp_losses_accum += per_kp_mse.cpu().numpy()
                per_kp_counts += 1

        mean_valid_loss = np.mean(valid_losses)
        per_kp_avg = per_kp_losses_accum / max(per_kp_counts, 1)

        # LR scheduler step
        scheduler.step()

        # Logging
        train_log["epochs"].append(epoch_num)
        train_log["losses"].append(float(mean_train_loss))
        train_log["validation_losses"].append(float(mean_valid_loss))
        train_log["per_kp_val_losses"].append(per_kp_avg.tolist())
        train_log["lr_history"].append(current_lr)
        train_log["timestamps"].append(time.time())

        # Print epoch summary
        is_best = mean_valid_loss < best_valid_loss
        marker = " ⭐ BEST" if is_best else ""
        print(f"Epoch {epoch_num:3d}/{args.epochs} | "
              f"train={mean_train_loss:.6f} | val={mean_valid_loss:.6f} | "
              f"lr={current_lr:.6f}{marker}")

        # Per-keypoint detail every 5 epochs
        if epoch_num % 5 == 0 or epoch_num == 1 or is_best:
            print(f"  Per-KP val MSE (unweighted): ", end="")
            for ki, name in enumerate(kp_names):
                print(f"{name}={per_kp_avg[ki]:.6f}", end="  ")
            print()

        # Save checkpoint
        dream_network.network_config["training"]["results"]["epochs_trained"] = epoch_num
        dream_network.network_config["training"]["results"]["training_loss"] = odict([
            ("mean", float(mean_train_loss)),
            ("stdev", float(np.std(train_losses))),
        ])
        dream_network.network_config["training"]["results"]["validation_loss"] = odict([
            ("mean", float(mean_valid_loss)),
            ("stdev", float(np.std(valid_losses))),
        ])

        # Save epoch checkpoint every 10 epochs
        if epoch_num % 10 == 0:
            dream_network.save_network(args.output, f"epoch_{epoch_num}", overwrite=True)
            print(f"  💾 Saved checkpoint: epoch_{epoch_num}")

        # Save best model
        if early_stopping(mean_valid_loss, epoch_num):
            best_valid_loss = mean_valid_loss
            dream_network.save_network(args.output, "best_network", overwrite=True)
            print(f"  🏆 New best model saved (val_loss={best_valid_loss:.6f})")
        if early_stopping.should_stop:
            print(f"\n🛑🏆 Early stopping → meilleur epoch : {early_stopping.best_epoch}")
            break    
            

    # Save final model
    dream_network.save_network(args.output, f"epoch_{args.epochs}", overwrite=True)

    # Save training log
    log_path = os.path.join(args.output, "training_log.pkl")
    with open(log_path, "wb") as f:
        pickle.dump(train_log, f)

    # Summary
    elapsed = time.time() - train_log["start_time"]
    print(f"\n{'='*70}")
    print(f"🎉 TRAINING COMPLETE")
    print(f"  Total time: {elapsed/60:.1f} min ({elapsed/3600:.1f} hours)")
    print(f"  Best validation loss: {best_valid_loss:.6f}")
    print(f"  Seed: {args.seed}")
    print(f"  Output: {args.output}")
    print(f"{'='*70}")

    # Print per-keypoint improvement analysis
    if len(train_log["per_kp_val_losses"]) > 1:
        first_kp = np.array(train_log["per_kp_val_losses"][0])
        best_epoch_idx = np.argmin(train_log["validation_losses"])
        best_kp = np.array(train_log["per_kp_val_losses"][best_epoch_idx])
        print(f"\n📈 Per-keypoint improvement (epoch 1 → best epoch {best_epoch_idx+1}):")
        for ki, name in enumerate(kp_names):
            improvement = (1.0 - best_kp[ki] / max(first_kp[ki], 1e-10)) * 100
            print(f"  {name:<8s}: {first_kp[ki]:.6f} → {best_kp[ki]:.6f} ({improvement:+.1f}%)")


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="DREAM training with weighted keypoint loss + fixed seed (v3)"
    )
    parser.add_argument("--data", "-d", required=True,
                        help="NDDS data directory")
    parser.add_argument("--arch", default="vgg", choices=["vgg", "resnet"],
                        help="Architecture (default: vgg)")
    parser.add_argument("--epochs", "-e", type=int, default=50,
                        help="Training epochs (default: 50)")
    parser.add_argument("--batch-size", "-b", type=int, default=16,
                        help="Batch size (default: 16 — safe on RTX 4000 Ada 20GB, ~2x faster than 8)")
    parser.add_argument("--lr", type=float, default=0.0001,
                        help="Initial learning rate (default: 0.0001, calibrated for this batch)")
    parser.add_argument("--workers", "-w", type=int, default=16,
                        help="Data loader workers (default: 16 — machine has 28 cores)")
    parser.add_argument("--gpu", "-g", type=int, default=0,
                        help="GPU ID (default: 0)")
    parser.add_argument("--output", "-o", default=None,
                        help="Output directory for checkpoints")
    parser.add_argument("--training-fraction", "-t", type=float, default=0.8,
                        help="Training data fraction (default: 0.8)")
    parser.add_argument("--kp-weights", default=",".join(str(w) for w in DEFAULT_KP_WEIGHTS),
                        help="Comma-separated per-keypoint loss weights "
                             "(base,link1,link2,link3,link4,link5,link6). "
                             f"Default: {','.join(str(w) for w in DEFAULT_KP_WEIGHTS)}")
    parser.add_argument("--kp-sigmas", default="2,2,2,2,2,2,2",
                        help="Comma-separated per-keypoint belief-map sigma "
                             "(base,link1,link2,link3,link4,link5,link6). Wider on distal "
                             "keypoints fights belief-map collapse. Default: 2,2,2,2,2,2,2 "
                             "(unchanged). Try 2,2,2,2,3,4,4.")
    parser.add_argument("--patience", type=int, default=5,
                        help="Early stopping patience (default: 5)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Seed fixe pour reproductibilité (random/numpy/torch). Default: 42")

    args = parser.parse_args()

    if args.output is None:
        weights_tag = "_".join(f"{w:.0f}" for w in
                               [float(w) for w in args.kp_weights.split(",")])
        args.output = os.path.join(
            SCRIPT_DIR, "checkpoints_dream",
            f"{args.arch}_ultimate_v3_e{args.epochs}"
        )
    train_weighted(args)


if __name__ == "__main__":
    main()