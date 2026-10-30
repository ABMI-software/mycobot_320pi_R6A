#!/usr/bin/env python3
"""Fine-tuning d'un modèle DREAM pré-entraîné sur synthétique vers données réelles.

Stratégie :
  1. Charger un modèle pré-entraîné sur synthétique (ex: vgg_ultimate_v2_e50)
  2. Fine-tuner sur données réelles avec LR très faible (0.00001)
  3. Éviter le catastrophic forgetting grâce au LR bas

Usage:
  python dream/train_dream_finetune_real.py \
      --data dream/dream_data/real \
      --resume dream/checkpoints_dream/vgg_ultimate_v2_e50/best_network.pth \
      --epochs 20 \
      --batch-size 8 \
      --workers 8 \
      --lr 0.00001 \
      --output checkpoints_dream/vgg_finetune_real_e20

Évaluation après fine-tuning :
  python dream/evaluate_dream.py \
      --weights checkpoints_dream/vgg_finetune_real_e20/best_network.pth \
      --data dream/dream_data/real_test \
      --split all
"""

import sys
import os
import time
import pickle
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

DEFAULT_KP_WEIGHTS = [1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 6.0]


class EarlyStopping:
    def __init__(self, patience=5, min_delta=1e-6):
        self.patience    = patience
        self.min_delta   = min_delta
        self.counter     = 0
        self.best_loss   = float("inf")
        self.best_epoch  = 0
        self.should_stop = False

    def __call__(self, val_loss, epoch):
        if val_loss < self.best_loss - self.min_delta:
            self.best_loss  = val_loss
            self.best_epoch = epoch
            self.counter    = 0
            return True
        else:
            self.counter += 1
            print(f"  ⏳ Pas d'amélioration ({self.counter}/{self.patience})")
            if self.counter >= self.patience:
                self.should_stop = True
                print(f"  🛑 Early stop → meilleur epoch : {self.best_epoch}")
            return False


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


def _patch_augmentation():
    """Augmentations sim-to-real pour fine-tuning sur réel."""
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
                albu.ISONoise(color_shift=(0.01, 0.05), intensity=(0.1, 0.5), p=0.4),
                albu.RandomBrightnessContrast(
                    brightness_limit=0.3, contrast_limit=0.3,
                    brightness_by_max=False, p=0.7,
                ),
                albu.HueSaturationValue(
                    hue_shift_limit=20, sat_shift_limit=30, val_shift_limit=20, p=0.5,
                ),
                albu.RandomShadow(
                    shadow_roi=(0, 0, 1, 1), num_shadows_limit=(1, 2),
                    shadow_dimension=5, p=0.3,
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
                    num_holes_range=(4, 10), hole_height_range=(30, 80),
                    hole_width_range=(30, 80), fill="random", p=0.6,
                ),
                albu.Perspective(scale=(0.05, 0.12), p=0.5),
                albu.ShiftScaleRotate(
                    shift_limit=0.05, scale_limit=0.1, rotate_limit=15, p=0.5,
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
    print("[PATCH] Fine-tuning augmentation pipeline")


def finetune(args):
    _patch_augmentation()

    MANIP_CONFIG     = os.path.join(SCRIPT_DIR, "manip_configs", "mycobot320.yaml")
    arch_config_path = os.path.join(DREAM_DIR, "arch_configs", "dream_vgg_q.yaml")

    os.makedirs(args.output, exist_ok=True)

    kp_weights = [float(w) for w in args.kp_weights.split(",")]
    kp_names   = ["base", "link1", "link2", "link3", "link4", "link5", "link6"]

    print(f"\n🎯 Per-keypoint loss weights:")
    for name, w in zip(kp_names, kp_weights):
        marker = "⬆️" if w > 1.0 else "  "
        print(f"  {marker} {name:<8s}: {w:.1f}x")

    yaml_parser = YAML(typ="safe")
    with open(MANIP_CONFIG) as f:
        manipulator_config = yaml_parser.load(f)["manipulator"]
    with open(arch_config_path) as f:
        arch_file = yaml_parser.load(f)
    architecture_config = arch_file["architecture"]
    training_config     = arch_file["training"]["config"]
    training_image_preprocessing  = training_config["image_preprocessing"]
    training_net_input_resolution = training_config["net_input_resolution"]
    architecture_config["image_preprocessing"] = training_image_preprocessing

    found_data = dream.utilities.find_ndds_data_in_dir(args.data)
    image_raw_resolution = dream.utilities.load_image_resolution(found_data[1]["camera"])

    try:
        user = os.getlogin()
    except:
        user = "unknown"

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
                ("data_augmentation", odict([("image_rgb", True)])),
                ("worker_size", args.workers),
                ("optimizer", odict([("type", "adam"), ("learning_rate", args.lr)])),
                ("image_preprocessing", training_image_preprocessing),
                ("image_raw_resolution", list(image_raw_resolution)),
                ("net_input_resolution", training_net_input_resolution),
                ("keypoint_weights", kp_weights),
            ])),
            ("platform", odict([
                ("user", user),
                ("hostname", socket.gethostname()),
                ("gpu_ids", [args.gpu]),
            ])),
            ("results", odict([("epochs_trained", 0)])),
        ])),
    ])

    # Créer le réseau
    print(f"\nCréation du réseau VGG...")
    dream_network = dream.create_network_from_config_data(network_config)

    # ---- RESUME : charger les poids pré-entraînés ----
    if args.resume:
        print(f"\n🔄 Chargement des poids pré-entraînés : {args.resume}")
        state_dict = torch.load(args.resume, map_location="cpu")
        # Gérer les différents formats de checkpoint
        if isinstance(state_dict, dict) and "model_state_dict" in state_dict:
            state_dict = state_dict["model_state_dict"]
        try:
            dream_network.model.load_state_dict(state_dict, strict=True)
            print(f"  ✅ Poids chargés (strict=True)")
        except RuntimeError:
            dream_network.model.load_state_dict(state_dict, strict=False)
            print(f"  ⚠️  Poids chargés (strict=False — certaines couches ignorées)")
    else:
        print(f"  ⚠️  Pas de --resume fourni — training from scratch")

    # Weighted loss
    weighted_criterion = WeightedBeliefMapLoss(kp_weights).cuda()
    dream_network.criterion = weighted_criterion
    print(f"✅ Weighted loss (weights sum={sum(kp_weights):.1f})")
    dream_network.enable_training()

    # Cosine annealing LR — très bas pour fine-tuning
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        dream_network.optimizer, T_max=args.epochs, eta_min=args.lr * 0.1
    )
    print(f"📈 Fine-tuning LR: {args.lr} → {args.lr * 0.1:.8f} (très bas pour éviter catastrophic forgetting)")

    trained_net_input_res, trained_net_output_res = \
        dream_network.net_resolutions_from_image_raw_resolution(image_raw_resolution)
    dream_network.network_config["training"]["config"]["net_output_resolution"] = trained_net_output_res

    found_dataset = dream.datasets.ManipulatorNDDSDataset(
        found_data, manipulator_config["name"], dream_network.keypoint_names,
        trained_net_input_res, trained_net_output_res,
        dream_network.image_normalization, dream_network.image_preprocessing(),
        augment_data=True, include_ground_truth=True, include_belief_maps=True,
    )

    n_data  = len(found_dataset)
    n_train = int(round(n_data * args.training_fraction))
    n_valid = n_data - n_train
    train_dataset, valid_dataset = torch.utils.data.random_split(
        found_dataset, [n_train, n_valid]
    )

    train_loader = TorchDataLoader(train_dataset, batch_size=args.batch_size,
                                   num_workers=args.workers, shuffle=True, pin_memory=True)
    valid_loader = TorchDataLoader(valid_dataset, batch_size=args.batch_size,
                                   num_workers=args.workers, pin_memory=True)

    print(f"\n📊 Dataset réel: {n_data} total ({n_train} train, {n_valid} valid)")
    print(f"\n{'='*70}")
    print(f"FINE-TUNING | {args.epochs} epochs | BS={args.batch_size} | LR={args.lr}")
    print(f"  Resume  : {args.resume}")
    print(f"  Data    : {args.data}")
    print(f"  Weights : {kp_weights}")
    print(f"{'='*70}\n")

    train_log = {
        "epochs": [], "losses": [], "validation_losses": [],
        "per_kp_val_losses": [], "lr_history": [],
        "start_time": time.time(), "timestamps": [],
        "keypoint_weights": kp_weights,
        "resume": args.resume,
    }

    best_valid_loss = float("inf")
    early_stopping  = EarlyStopping(patience=args.patience)

    for epoch in range(args.epochs):
        epoch_num  = epoch + 1
        current_lr = dream_network.optimizer.param_groups[0]["lr"]

        # ---- Training ----
        dream_network.enable_training()
        train_losses = []
        for batch in tqdm(train_loader, desc=f"Epoch {epoch_num}/{args.epochs} [train]", leave=False):
            network_input = batch["image_rgb_input"].cuda()
            target        = batch["belief_maps"].cuda()
            dream_network.optimizer.zero_grad()
            output = dream_network.model(network_input)
            pred   = output[0] if isinstance(output, list) else output
            loss   = weighted_criterion(pred, target)
            loss.backward()
            dream_network.optimizer.step()
            train_losses.append(loss.item())

        # ---- Validation ----
        dream_network.enable_evaluation()
        valid_losses = []
        per_kp_accum = np.zeros(7)
        per_kp_count = 0
        with torch.no_grad():
            for batch in tqdm(valid_loader, desc=f"Epoch {epoch_num}/{args.epochs} [valid]", leave=False):
                network_input = batch["image_rgb_input"].cuda()
                target        = batch["belief_maps"].cuda()
                output = dream_network.model(network_input)
                pred   = output[0] if isinstance(output, list) else output
                loss   = weighted_criterion(pred, target)
                valid_losses.append(loss.item())
                per_kp_accum += ((pred - target) ** 2).mean(dim=(0, 2, 3)).cpu().numpy()
                per_kp_count += 1

        mean_train = np.mean(train_losses)
        mean_valid = np.mean(valid_losses)
        per_kp_avg = per_kp_accum / max(per_kp_count, 1)
        scheduler.step()

        train_log["epochs"].append(epoch_num)
        train_log["losses"].append(float(mean_train))
        train_log["validation_losses"].append(float(mean_valid))
        train_log["per_kp_val_losses"].append(per_kp_avg.tolist())
        train_log["lr_history"].append(current_lr)
        train_log["timestamps"].append(time.time())

        is_best = mean_valid < best_valid_loss
        marker  = " ⭐ BEST" if is_best else ""
        print(f"Epoch {epoch_num:3d}/{args.epochs} | train={mean_train:.6f} | val={mean_valid:.6f} | lr={current_lr:.8f}{marker}")

        if epoch_num % 5 == 0 or epoch_num == 1 or is_best:
            print(f"  Per-KP val MSE: ", end="")
            for ki, name in enumerate(kp_names):
                print(f"{name}={per_kp_avg[ki]:.6f}", end="  ")
            print()

        dream_network.network_config["training"]["results"]["epochs_trained"] = epoch_num

        if epoch_num % 5 == 0:
            dream_network.save_network(args.output, f"epoch_{epoch_num}", overwrite=True)
            print(f"  💾 Checkpoint: epoch_{epoch_num}")

        if early_stopping(mean_valid, epoch_num):
            best_valid_loss = mean_valid
            dream_network.save_network(args.output, "best_network", overwrite=True)
            print(f"  🏆 New best (val_loss={best_valid_loss:.6f})")

        if early_stopping.should_stop:
            break

    dream_network.save_network(args.output, f"epoch_{args.epochs}", overwrite=True)

    with open(os.path.join(args.output, "training_log.pkl"), "wb") as f:
        pickle.dump(train_log, f)

    elapsed = time.time() - train_log["start_time"]
    print(f"\n{'='*70}")
    print(f"🎉 FINE-TUNING COMPLETE")
    print(f"  Durée : {elapsed/60:.1f} min")
    print(f"  Best val_loss : {best_valid_loss:.6f}")
    print(f"  Output : {args.output}")
    print(f"{'='*70}")

    if len(train_log["per_kp_val_losses"]) > 1:
        first_kp = np.array(train_log["per_kp_val_losses"][0])
        best_idx = np.argmin(train_log["validation_losses"])
        best_kp  = np.array(train_log["per_kp_val_losses"][best_idx])
        print(f"\n📈 Per-keypoint improvement (epoch 1 → best epoch {best_idx+1}):")
        for ki, name in enumerate(kp_names):
            imp = (1.0 - best_kp[ki] / max(first_kp[ki], 1e-10)) * 100
            print(f"  {name:<8s}: {first_kp[ki]:.6f} → {best_kp[ki]:.6f} ({imp:+.1f}%)")

    print(f"\nÉvaluation sur real_test :")
    print(f"  python dream/evaluate_dream.py \\")
    print(f"    --weights {args.output}/best_network.pth \\")
    print(f"    --data dream/dream_data/real_test --split all")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Fine-tuning DREAM sur données réelles")
    parser.add_argument("--data",     "-d", required=True, help="Dataset réel NDDS")
    parser.add_argument("--resume",   "-r", required=True, help="Checkpoint pré-entraîné (.pth)")
    parser.add_argument("--epochs",   "-e", type=int, default=20)
    parser.add_argument("--batch-size", "-b", type=int, default=8)
    parser.add_argument("--lr",       type=float, default=0.00001,
                        help="LR très faible pour fine-tuning (default: 0.00001)")
    parser.add_argument("--workers",  "-w", type=int, default=8)
    parser.add_argument("--gpu",      "-g", type=int, default=0)
    parser.add_argument("--output",   "-o", default=None)
    parser.add_argument("--training-fraction", "-t", type=float, default=0.8)
    parser.add_argument("--kp-weights", default=",".join(str(w) for w in DEFAULT_KP_WEIGHTS))
    parser.add_argument("--patience", type=int, default=5)
    args = parser.parse_args()

    if args.output is None:
        args.output = os.path.join(
            SCRIPT_DIR, "checkpoints_dream", f"vgg_finetune_real_e{args.epochs}"
        )
    finetune(args)


if __name__ == "__main__":
    main()