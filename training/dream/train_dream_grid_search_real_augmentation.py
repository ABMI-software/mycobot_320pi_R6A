#!/usr/bin/env python3
"""Grid search sur les augmentations CoarseDropout + Perspective transform
pour améliorer la détection sur données RÉELLES.

Stratégie :
  - Entraîner 5 epochs sur dataset MIXTE (réel + synth)
  - Sauvegarder les checkpoints dans checkpoints_dream/aug_grid_*
  - Utiliser evaluate_grid_real.py pour évaluer sur réel

Combinaisons testées :
  CD = CoarseDropout (holes, size, proba)
  PT = Perspective transform (scale, proba)

Usage:
  python dream/train_dream_grid_search_real_augmentation.py \
      --data-mixed dream/dream_data/mixed_real_synth \
      --epochs 5 \
      --batch-size 8 \
      --workers 8
"""

import sys
import os
import time
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

# Weights fixes (meilleur modèle synthétique)
KP_WEIGHTS = [1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 6.0]

# ============================================================
# Grid Search : combinaisons CoarseDropout + Perspective
# ============================================================
AUG_GRID = [
    {
        "name": "baseline",
        "cd_holes": (2, 6),
        "cd_size":  (15, 60),
        "cd_p":     0.4,
        "pt_scale": (0.03, 0.07),
        "pt_p":     0.3,
    },
    {
        "name": "cd_medium_pt_medium",
        "cd_holes": (4, 10),
        "cd_size":  (30, 80),
        "cd_p":     0.6,
        "pt_scale": (0.05, 0.12),
        "pt_p":     0.5,
    },
    {
        "name": "cd_high_pt_medium",
        "cd_holes": (6, 12),
        "cd_size":  (40, 100),
        "cd_p":     0.8,
        "pt_scale": (0.05, 0.12),
        "pt_p":     0.5,
    },
    {
        "name": "cd_medium_pt_high",
        "cd_holes": (4, 10),
        "cd_size":  (30, 80),
        "cd_p":     0.6,
        "pt_scale": (0.08, 0.15),
        "pt_p":     0.7,
    },
    {
        "name": "cd_high_pt_high",
        "cd_holes": (6, 12),
        "cd_size":  (40, 100),
        "cd_p":     0.8,
        "pt_scale": (0.08, 0.15),
        "pt_p":     0.7,
    },
]


# ============================================================
# Classes
# ============================================================

class WeightedBeliefMapLoss(torch.nn.Module):
    def __init__(self, kp_weights):
        super().__init__()
        self.register_buffer(
            "weights",
            torch.tensor(kp_weights, dtype=torch.float32).view(1, -1, 1, 1),
        )

    def forward(self, pred, target):
        diff_sq = (pred - target) ** 2
        return (diff_sq * self.weights).mean()


class EarlyStopping:
    def __init__(self, patience=3, min_delta=1e-6):
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
            if self.counter >= self.patience:
                self.should_stop = True
            return False


# ============================================================
# Patch augmentation dynamique
# ============================================================

def patch_augmentation(aug_config):
    """Patch le dataset avec la config d'augmentation donnée."""
    import dream.datasets as dream_datasets

    original_getitem = dream_datasets.ManipulatorNDDSDataset.__getitem__

    def patched_getitem(self, idx):
        if self.augment_data:
            self.augment_data = False
            sample = original_getitem(self, idx)
            self.augment_data = True

            img_tensor = sample["image_rgb_input"]
            img_np = img_tensor.permute(1, 2, 0).numpy() * 0.5 + 0.5
            img_np = (img_np * 255).clip(0, 255).astype(np.uint8)

            aug = albu.Compose([
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
                # ---- CoarseDropout paramétrable ----
                albu.CoarseDropout(
                    num_holes_range=aug_config["cd_holes"],
                    hole_height_range=aug_config["cd_size"],
                    hole_width_range=aug_config["cd_size"],
                    fill="random",
                    p=aug_config["cd_p"],
                ),
                # ---- Perspective paramétrable ----
                albu.Perspective(
                    scale=aug_config["pt_scale"],
                    p=aug_config["pt_p"],
                ),
                albu.ShiftScaleRotate(
                    shift_limit=0.05,
                    scale_limit=0.1,
                    rotate_limit=15,
                    p=0.5,
                ),
            ], p=1.0)

            augmented = aug(image=img_np)
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
    print(f"  [PATCH] CoarseDropout: holes={aug_config['cd_holes']}, "
          f"size={aug_config['cd_size']}, p={aug_config['cd_p']}")
    print(f"  [PATCH] Perspective: scale={aug_config['pt_scale']}, p={aug_config['pt_p']}")


# ============================================================
# Training
# ============================================================

def train_one_run(data_mixed, output_dir, epochs, batch_size, workers, aug_config):
    """Entraîne un run avec la config d'augmentation donnée."""

    patch_augmentation(aug_config)

    MANIP_CONFIG = os.path.join(SCRIPT_DIR, "manip_configs", "mycobot320.yaml")
    arch_config_path = os.path.join(DREAM_DIR, "arch_configs", "dream_vgg_q.yaml")

    os.makedirs(output_dir, exist_ok=True)

    yaml_parser = YAML(typ="safe")
    with open(MANIP_CONFIG) as f:
        manipulator_config = yaml_parser.load(f)["manipulator"]
    with open(arch_config_path) as f:
        arch_file = yaml_parser.load(f)
    architecture_config = arch_file["architecture"]
    training_config = arch_file["training"]["config"]
    training_image_preprocessing = training_config["image_preprocessing"]
    training_net_input_resolution = training_config["net_input_resolution"]
    architecture_config["image_preprocessing"] = training_image_preprocessing

    found_data = dream.utilities.find_ndds_data_in_dir(data_mixed)
    image_raw_resolution = dream.utilities.load_image_resolution(found_data[1]["camera"])

    try:
        user = os.getlogin()
    except:
        user = "unknown"

    network_config = odict([
        ("data_path", data_mixed),
        ("manipulator", manipulator_config),
        ("architecture", architecture_config),
        ("training", odict([
            ("config", odict([
                ("epochs", epochs),
                ("training_data_fraction", 0.8),
                ("validation_data_fraction", 0.2),
                ("batch_size", batch_size),
                ("data_augmentation", odict([("image_rgb", True)])),
                ("worker_size", workers),
                ("optimizer", odict([("type", "adam"), ("learning_rate", 0.0001)])),
                ("image_preprocessing", training_image_preprocessing),
                ("image_raw_resolution", list(image_raw_resolution)),
                ("net_input_resolution", training_net_input_resolution),
                ("keypoint_weights", KP_WEIGHTS),
            ])),
            ("platform", odict([
                ("user", user),
                ("hostname", socket.gethostname()),
                ("gpu_ids", [0])
            ])),
            ("results", odict([("epochs_trained", 0)])),
        ])),
    ])

    dream_network = dream.create_network_from_config_data(network_config)
    weighted_criterion = WeightedBeliefMapLoss(KP_WEIGHTS).cuda()
    dream_network.criterion = weighted_criterion
    dream_network.enable_training()

    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        dream_network.optimizer, T_max=epochs, eta_min=0.0001 * 0.01
    )

    trained_net_input_res, trained_net_output_res = \
        dream_network.net_resolutions_from_image_raw_resolution(image_raw_resolution)
    dream_network.network_config["training"]["config"]["net_output_resolution"] = trained_net_output_res

    found_dataset = dream.datasets.ManipulatorNDDSDataset(
        found_data, manipulator_config["name"], dream_network.keypoint_names,
        trained_net_input_res, trained_net_output_res,
        dream_network.image_normalization, dream_network.image_preprocessing(),
        augment_data=True, include_ground_truth=True, include_belief_maps=True,
    )

    n_data = len(found_dataset)
    n_train = int(round(n_data * 0.8))
    n_valid = n_data - n_train
    train_dataset, valid_dataset = torch.utils.data.random_split(
        found_dataset, [n_train, n_valid]
    )

    train_loader = TorchDataLoader(train_dataset, batch_size=batch_size,
                                   num_workers=workers, shuffle=True, pin_memory=True)
    valid_loader = TorchDataLoader(valid_dataset, batch_size=batch_size,
                                   num_workers=workers, pin_memory=True)

    best_valid_loss = float("inf")
    early_stopping = EarlyStopping(patience=3)

    for epoch in range(epochs):
        epoch_num = epoch + 1

        # Train
        dream_network.enable_training()
        train_losses = []
        for batch in tqdm(train_loader, desc=f"  Epoch {epoch_num}/{epochs} [train]", leave=False):
            network_input = batch["image_rgb_input"].cuda()
            target = batch["belief_maps"].cuda()
            dream_network.optimizer.zero_grad()
            output = dream_network.model(network_input)
            pred = output[0] if isinstance(output, list) else output
            loss = weighted_criterion(pred, target)
            loss.backward()
            dream_network.optimizer.step()
            train_losses.append(loss.item())

        # Validation
        dream_network.enable_evaluation()
        valid_losses = []
        with torch.no_grad():
            for batch in tqdm(valid_loader, desc=f"  Epoch {epoch_num}/{epochs} [valid]", leave=False):
                network_input = batch["image_rgb_input"].cuda()
                target = batch["belief_maps"].cuda()
                output = dream_network.model(network_input)
                pred = output[0] if isinstance(output, list) else output
                loss = weighted_criterion(pred, target)
                valid_losses.append(loss.item())

        mean_train = np.mean(train_losses)
        mean_valid = np.mean(valid_losses)
        scheduler.step()

        is_best = mean_valid < best_valid_loss
        print(f"  Epoch {epoch_num:2d}/{epochs} | train={mean_train:.6f} | "
              f"val={mean_valid:.6f}" + (" ⭐" if is_best else ""))

        if early_stopping(mean_valid, epoch_num):
            best_valid_loss = mean_valid
            dream_network.network_config["training"]["results"]["epochs_trained"] = epoch_num
            dream_network.save_network(output_dir, "best_network", overwrite=True)

        if early_stopping.should_stop:
            print(f"  🛑 Early stop → best epoch: {early_stopping.best_epoch}")
            break

    print(f"  ✅ Checkpoint sauvegardé → {output_dir}/best_network.pth")
    return best_valid_loss


# ============================================================
# Main
# ============================================================

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Grid search augmentations — training sur mixed"
    )
    parser.add_argument("--data-mixed", required=True, help="Dataset mixte NDDS")
    parser.add_argument("--epochs", type=int, default=5, help="Epochs par run (default: 5)")
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    print(f"\n{'='*70}")
    print(f"GRID SEARCH AUGMENTATIONS — Training sur mixed")
    print(f"  Mixed data : {args.data_mixed}")
    print(f"  Epochs/run : {args.epochs}")
    print(f"  Combos     : {len(AUG_GRID)}")
    print(f"  Durée est. : ~{len(AUG_GRID) * args.epochs * 8} min")
    print(f"  Checkpoints: checkpoints_dream/aug_grid_*")
    print(f"{'='*70}\n")

    for i, aug_config in enumerate(AUG_GRID):
        name = aug_config["name"]
        output_dir = os.path.join(SCRIPT_DIR, "checkpoints_dream", f"aug_grid_{name}")
        weights_path = os.path.join(output_dir, "best_network.pth")

        print(f"\n{'='*60}")
        print(f"▶ Run {i+1}/{len(AUG_GRID)} : {name}")
        print(f"  CoarseDropout: holes={aug_config['cd_holes']}, "
              f"size={aug_config['cd_size']}, p={aug_config['cd_p']}")
        print(f"  Perspective  : scale={aug_config['pt_scale']}, p={aug_config['pt_p']}")
        print(f"{'='*60}")

        if os.path.exists(weights_path):
            print(f"  ⏭️  Déjà entraîné → {output_dir}")
            continue

        t0 = time.time()
        best_val_loss = train_one_run(
            args.data_mixed, output_dir,
            args.epochs, args.batch_size, args.workers,
            aug_config
        )
        elapsed = time.time() - t0
        print(f"  ✅ Terminé en {elapsed/60:.1f} min | best_val_loss={best_val_loss:.6f}")

    print(f"\n\n{'='*70}")
    print(f"🎉 TRAINING TERMINÉ — {len(AUG_GRID)} runs")
    print(f"   Lancer maintenant : python dream/evaluate_grid_real.py")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()