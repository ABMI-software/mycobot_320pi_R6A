#!/usr/bin/env python3
"""DREAM training v6 — comme v5 mais avec augmentation paramétrable via CLI.

Différence vs v5 : _patch_augmentation_v2() hardcodée remplacée par des args
--aug-brightness, --aug-contrast, --aug-hue, --aug-sat, --aug-noise. Permet
au grid_search_aug.py de tester plusieurs configs sans dupliquer de code.

Usage:
  python dream/train_dream_mixed_aug_optimized_v6.py \\
      --data dream/dream_data/mixed_cyclegan \\
      --epochs 50 \\
      --batch-size 8 \\
      --workers 8 \\
      --output checkpoints_dream/vgg_cyclegan_e50 \\
      --aug-brightness 0.3 --aug-contrast 0.3 --aug-hue 5 --aug-sat 10
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
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

DREAM_DIR = "/tmp/DREAM"
sys.path.insert(0, DREAM_DIR)
import dream

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_KP_WEIGHTS = [1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 6.0]

TRAIN_FRAC = 0.80
VAL_FRAC   = 0.10
TEST_FRAC  = 0.10


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
        return ((pred - target) ** 2 * self.weights).mean()


def _patch_augmentation(brightness=0.3, contrast=0.3, hue=5, sat=10, noise=0.0):
    """Augmentation paramétrable — tous à 0 = aucune transformation."""
    import dream.datasets as dream_datasets

    transforms = []
    if brightness > 0 or contrast > 0:
        transforms.append(albu.RandomBrightnessContrast(
            brightness_limit=brightness, contrast_limit=contrast,
            brightness_by_max=False, p=0.7))
    if hue > 0 or sat > 0:
        transforms.append(albu.HueSaturationValue(
            hue_shift_limit=hue, sat_shift_limit=sat, val_shift_limit=20, p=0.5))
    if noise > 0:
        transforms.append(albu.GaussNoise(
            var_limit=(noise, noise * 2), p=0.4))

    if not transforms:
        print("[AUG] Désactivée (tous params à 0)")
        return

    aug_pipeline = albu.Compose(transforms, p=1.0)
    original_getitem = dream_datasets.ManipulatorNDDSDataset.__getitem__

    def patched_getitem(self, idx):
        if self.augment_data:
            self.augment_data = False
            sample = original_getitem(self, idx)
            self.augment_data = True

            img_tensor = sample["image_rgb_input"]
            img_np = img_tensor.permute(1, 2, 0).numpy() * 0.5 + 0.5
            img_np = (img_np * 255).clip(0, 255).astype(np.uint8)

            img_aug = aug_pipeline(image=img_np)["image"]
            img_aug_f = img_aug.astype(np.float32) / 255.0
            img_aug_f = (img_aug_f - 0.5) / 0.5
            sample["image_rgb_input"] = torch.from_numpy(
                img_aug_f.transpose(2, 0, 1)).float()
            return sample
        else:
            return original_getitem(self, idx)

    dream_datasets.ManipulatorNDDSDataset.__getitem__ = patched_getitem
    desc = []
    if brightness > 0:
        desc.append(f"brightness={brightness}/contrast={contrast}")
    if hue > 0:
        desc.append(f"hue={hue}/sat={sat}")
    if noise > 0:
        desc.append(f"noise={noise}")
    print(f"[AUG] {', '.join(desc)}")


def setup_live_plot():
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title("Courbe d'apprentissage — Val vs Test Loss")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("Loss")
    ax.grid(True, alpha=0.3)
    line_val,  = ax.plot([], [], "o-", color="orange", label="Val loss",  linewidth=2)
    line_test, = ax.plot([], [], "s-", color="green",  label="Test loss", linewidth=2)
    ax.legend()
    fig.tight_layout()
    plt.show(block=False)
    return fig, ax, line_val, line_test


def update_live_plot(fig, ax, line_val, line_test, epochs, val_losses, test_losses, output_dir):
    line_val.set_data(epochs, val_losses)
    line_test.set_data(epochs, test_losses)
    ax.relim()
    ax.autoscale_view()

    if len(val_losses) > 3:
        gap = test_losses[-1] - val_losses[-1]
        if gap > 0.0005:
            ax.set_title(f"⚠️ Possible overfitting — Gap: {gap:.6f}", color="red")
        else:
            ax.set_title(f"✅ Bonne généralisation — Gap: {gap:.6f}", color="green")

    fig.canvas.draw()
    fig.canvas.flush_events()
    fig.savefig(os.path.join(output_dir, "learning_curve.png"), dpi=100, bbox_inches="tight")


def train(args):
    _patch_augmentation(
        brightness=args.aug_brightness,
        contrast=args.aug_contrast,
        hue=args.aug_hue,
        sat=args.aug_sat,
        noise=args.aug_noise,
    )

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
    except Exception:
        user = "unknown"

    network_config = odict([
        ("data_path", args.data),
        ("manipulator", manipulator_config),
        ("architecture", architecture_config),
        ("training", odict([
            ("config", odict([
                ("epochs", args.epochs),
                ("training_data_fraction", TRAIN_FRAC),
                ("validation_data_fraction", VAL_FRAC),
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

    dream_network = dream.create_network_from_config_data(network_config)
    weighted_criterion = WeightedBeliefMapLoss(kp_weights).cuda()
    dream_network.criterion = weighted_criterion
    print(f"✅ Weighted loss (weights sum={sum(kp_weights):.1f})")
    dream_network.enable_training()

    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        dream_network.optimizer, T_max=args.epochs, eta_min=args.lr * 0.01
    )
    print(f"📈 Cosine annealing LR: {args.lr} → {args.lr * 0.01:.6f}")

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
    n_train = int(round(n_data * TRAIN_FRAC))
    n_val   = int(round(n_data * VAL_FRAC))
    n_test  = n_data - n_train - n_val

    indices   = list(range(n_data))
    train_idx = indices[:n_train]
    val_idx   = indices[n_train:n_train + n_val]
    test_idx  = indices[n_train + n_val:]

    train_dataset = torch.utils.data.Subset(found_dataset, train_idx)
    valid_dataset = torch.utils.data.Subset(found_dataset, val_idx)
    test_dataset  = torch.utils.data.Subset(found_dataset, test_idx)

    train_loader = TorchDataLoader(train_dataset, batch_size=args.batch_size,
                                   num_workers=args.workers, shuffle=True, pin_memory=True)
    valid_loader = TorchDataLoader(valid_dataset, batch_size=args.batch_size,
                                   num_workers=args.workers, pin_memory=True)
    test_loader  = TorchDataLoader(test_dataset,  batch_size=args.batch_size,
                                   num_workers=args.workers, pin_memory=True)

    print(f"\n📊 Dataset: {n_data} total")
    print(f"   Train : {n_train} ({TRAIN_FRAC*100:.0f}%)")
    print(f"   Val   : {n_val} ({VAL_FRAC*100:.0f}%) → early stopping")
    print(f"   Test  : {n_test} ({TEST_FRAC*100:.0f}%) → courbe overfitting")
    print(f"\n{'='*70}")
    print(f"TRAINING v6 | {args.epochs} epochs | BS={args.batch_size} | LR={args.lr}")
    print(f"  Weights : {kp_weights}")
    print(f"  Split   : 80/10/10")
    print(f"  Aug     : brightness={args.aug_brightness} contrast={args.aug_contrast} "
          f"hue={args.aug_hue} sat={args.aug_sat} noise={args.aug_noise}")
    print(f"{'='*70}\n")

    fig, ax, line_val, line_test = setup_live_plot()

    train_log = {
        "epochs": [], "losses": [], "validation_losses": [], "test_losses": [],
        "per_kp_val_losses": [], "lr_history": [],
        "start_time": time.time(), "timestamps": [],
        "keypoint_weights": kp_weights,
        "split": {"train": n_train, "val": n_val, "test": n_test},
        "aug": {
            "brightness": args.aug_brightness, "contrast": args.aug_contrast,
            "hue": args.aug_hue, "sat": args.aug_sat, "noise": args.aug_noise,
        },
    }

    best_valid_loss = float("inf")
    early_stopping  = EarlyStopping(patience=args.patience)
    epochs_list      = []
    val_losses_list  = []
    test_losses_list = []

    for epoch in range(args.epochs):
        epoch_num  = epoch + 1
        current_lr = dream_network.optimizer.param_groups[0]["lr"]

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

        test_losses = []
        with torch.no_grad():
            for batch in tqdm(test_loader, desc=f"Epoch {epoch_num}/{args.epochs} [test] ", leave=False):
                network_input = batch["image_rgb_input"].cuda()
                target        = batch["belief_maps"].cuda()
                output = dream_network.model(network_input)
                pred   = output[0] if isinstance(output, list) else output
                loss   = weighted_criterion(pred, target)
                test_losses.append(loss.item())

        mean_train = np.mean(train_losses)
        mean_valid = np.mean(valid_losses)
        mean_test  = np.mean(test_losses)
        per_kp_avg = per_kp_accum / max(per_kp_count, 1)
        scheduler.step()

        train_log["epochs"].append(epoch_num)
        train_log["losses"].append(float(mean_train))
        train_log["validation_losses"].append(float(mean_valid))
        train_log["test_losses"].append(float(mean_test))
        train_log["per_kp_val_losses"].append(per_kp_avg.tolist())
        train_log["lr_history"].append(current_lr)
        train_log["timestamps"].append(time.time())

        gap = mean_test - mean_valid
        overfit_marker = f" ⚠️ gap={gap:.6f}" if gap > 0.0005 else ""

        is_best = mean_valid < best_valid_loss
        marker  = " ⭐ BEST" if is_best else ""
        print(f"Epoch {epoch_num:3d}/{args.epochs} | train={mean_train:.6f} | val={mean_valid:.6f} | test={mean_test:.6f} | lr={current_lr:.6f}{marker}{overfit_marker}")

        if epoch_num % 5 == 0 or epoch_num == 1 or is_best:
            print(f"  Per-KP: ", end="")
            for ki, name in enumerate(kp_names):
                print(f"{name}={per_kp_avg[ki]:.6f}", end="  ")
            print()

        epochs_list.append(epoch_num)
        val_losses_list.append(mean_valid)
        test_losses_list.append(mean_test)
        update_live_plot(fig, ax, line_val, line_test,
                         epochs_list, val_losses_list, test_losses_list, args.output)

        dream_network.network_config["training"]["results"]["epochs_trained"] = epoch_num

        if epoch_num % 10 == 0:
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

    curve_path = os.path.join(args.output, "learning_curve_final.png")
    fig.savefig(curve_path, dpi=150, bbox_inches="tight")
    print(f"  📊 Courbe sauvegardée : {curve_path}")

    elapsed = time.time() - train_log["start_time"]
    print(f"\n{'='*70}")
    print(f"🎉 TRAINING v6 COMPLETE")
    print(f"  Durée         : {elapsed/60:.1f} min ({elapsed/3600:.1f} h)")
    print(f"  Best val_loss : {best_valid_loss:.6f}")
    print(f"  Output        : {args.output}")
    print(f"{'='*70}")

    if len(train_log["per_kp_val_losses"]) > 1:
        first_kp = np.array(train_log["per_kp_val_losses"][0])
        best_idx = np.argmin(train_log["validation_losses"])
        best_kp  = np.array(train_log["per_kp_val_losses"][best_idx])
        print(f"\n📈 Per-keypoint improvement (epoch 1 → best epoch {best_idx+1}):")
        for ki, name in enumerate(kp_names):
            imp = (1.0 - best_kp[ki] / max(first_kp[ki], 1e-10)) * 100
            print(f"  {name:<8s}: {first_kp[ki]:.6f} → {best_kp[ki]:.6f} ({imp:+.1f}%)")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="DREAM training v6 — augmentation paramétrable via CLI")
    parser.add_argument("--data",           "-d", required=True)
    parser.add_argument("--epochs",         "-e", type=int,   default=50)
    parser.add_argument("--batch-size",     "-b", type=int,   default=8)
    parser.add_argument("--lr",                   type=float, default=0.0001)
    parser.add_argument("--workers",        "-w", type=int,   default=8)
    parser.add_argument("--gpu",            "-g", type=int,   default=0)
    parser.add_argument("--output",         "-o", default=None)
    parser.add_argument("--kp-weights",           default=",".join(str(w) for w in DEFAULT_KP_WEIGHTS))
    parser.add_argument("--patience",             type=int,   default=5)
    parser.add_argument("--aug-brightness",       type=float, default=0.3)
    parser.add_argument("--aug-contrast",         type=float, default=0.3)
    parser.add_argument("--aug-hue",              type=float, default=5.0)
    parser.add_argument("--aug-sat",              type=float, default=10.0)
    parser.add_argument("--aug-noise",            type=float, default=0.0)
    args = parser.parse_args()

    if args.output is None:
        args.output = os.path.join(
            SCRIPT_DIR, "checkpoints_dream", f"vgg_cyclegan_e{args.epochs}"
        )
    train(args)


if __name__ == "__main__":
    main()
