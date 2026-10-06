#!/usr/bin/env python3
"""DREAM training — v5, augmentation geometrique reparee.

Identique a `train_dream_ultimate_v4_mix.py` (conserve intact) sur tout le
reste ; SEULE l'augmentation change.

CE QUI N'ALLAIT PAS. Le patch du v4 faisait ceci :

    self.augment_data = False          # coupe l'augmentation de DREAM
    sample = original_getitem(self, idx)
    ...
    augmented = strong_aug(image=img_np)          # image SEULE
    sample["image_rgb_input"] = <image augmentee>  # cible NON mise a jour

Or `strong_aug` contenait `ShiftScaleRotate(shift_limit=0.05, scale_limit=0.3,
rotate_limit=15, p=0.5)` et `Perspective(p=0.3)`. Sur ~65 % des echantillons,
l'image etait donc translatee, redimensionnee et tournee pendant que
`belief_maps` et `keypoint_projections_output`, produits par `original_getitem`,
restaient ceux de l'image NON transformee.

On apprenait donc explicitement au reseau a rendre la meme position quelle que
soit la geometrie de l'image. Mesure du 03/09 sur `vgg_montage0901_ft_e30` :
`base` est a 0,8 px de la verite et suit 0 % d'une translation de 30 px. Le
second symptome connu s'explique pareil — l'optimum d'une MSE sur cible fixe
avec image bougee est une bosse etalee, d'ou les belief maps distales aplaties.

CE QUE FAIT LE v5. DREAM sait deja faire : `datasets.py` compose sa propre
augmentation avec `keypoint_params={"format": "xy"}`, transforme les keypoints
AVEC l'image, puis recalcule `kp_projs_net_output` et les belief maps. On lui
rend donc la main sur la geometrie (`augment_data` reste True) et on ne
superpose plus que du PHOTOMETRIQUE, qui ne deplace rien et peut legitimement
s'appliquer a l'image seule.

`scale_limit` REVIENT au 0,1 de DREAM, l'elargissement a 0,3 est abandonne.
Deux raisons, mesurees le 03/09.

D'abord le risque. `create_belief_map` ne dessine sa gaussienne QUE si le point
tient a plus de 4 px du bord ; sinon la cible du keypoint est une carte
ENTIEREMENT NULLE, et `keypoint_params` de DREAM porte `remove_invisible:
False`, donc un point chasse hors cadre par la transformation arrive quand meme
la. Trop d'amplitude geometrique n'ecrase pas le pic : elle apprend au reseau a
ne RIEN detecter. Mesure : 0,2-0,3 % de cibles nulles en moyenne, 1,3 % au pire
sur link6, sans croissance de 0,0 a 0,3 — le moteur des sorties de cadre est
`shift_limit`, inchange, et le robot n'occupe que 17-26 % de l'image.

Ensuite l'inutilite. L'elargissement visait l'ecart de focale des cameras
reelles. Le jeu a camera randomisee tire desormais la distance entre 0,70 et
1,45 m, soit un rapport 2:1 de taille apparente — la diversite d'echelle vient
de la donnee, pas de l'augmentation. Ajouter 30 % par-dessus n'apporte rien et
ne fait que rapprocher des bords.

`Perspective` est retiree : elle est geometrique, elle n'etait pas dans le
pipeline de DREAM, et son gain (p=0,3) ne vaut pas le risque d'un second
decalage image/cible.

Variante de `train_dream_ultimate_v4_mix.py` (synthetique + reel suréchantillonné).

Same recipe as train_dream_ultimate_v4.py (weighted per-keypoint loss, strong
sim-to-real augmentation, cosine annealing LR, --pretrained warm-start),
plus two things carried over from train_dream_mixed_aug_optimized_v10.py:

  1. Fixed SEED=42 and a deterministic 80/10/10 train/val/test split
     (v4 used torch.utils.data.random_split with NO seed -> unreproducible
     val split across runs; this version shuffles indices with a fixed
     seed so the split -- and therefore any "% val detection" figure --
     is reproducible).
  2. A val-vs-test learning curve saved every epoch, with an overfitting
     warning when the test/val gap grows (same logic as v10's live plot,
     minus the interactive TkAgg window since this run is meant to churn
     unattended in the background).


Usage:
  python train_dream_ultimate_v4_mix.py \\
      --data dream_data/mix_synth50k_real3camx5_ndds \\
      --pretrained output/checkpoints_dream/vgg_ultimate_v4_e50/best_network.pth \\
      --epochs 30
"""

import sys
import os
import glob
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
import matplotlib
matplotlib.use("Agg")  # headless: this run is meant to churn unattended
import matplotlib.pyplot as plt

# /tmp/DREAM is wiped at reboot; the library now lives in ~/DREAM.
DREAM_DIR = os.environ.get("DREAM_DIR", os.path.expanduser("~/DREAM"))
sys.path.insert(0, DREAM_DIR)
import dream

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# INCHANGES par rapport au v4, deliberement : le v5 ne doit differer que par
# l'augmentation, sinon un resultat different ne serait attribuable a rien.
#
# Les baisser avait ete envisage puis abandonne le 03/09, les logs disant le
# contraire de l'intuition : sur `vgg_ultimate_v4_mix_ft_e30`, MSE brute non
# ponderee a l'epoque 30, link6 (poids 6,0) finit a 0,000513 — MIEUX que link1,
# link2 (0,000540) et link5 (0,000557). Et les keypoints ponderes progressent le
# plus sur 30 epoques : 30,7 % pour link4 et 31,8 % pour link6, contre 9,9 %
# pour les non ponderes. Aucune trace de l'aplatissement redoute. Les distaux
# sont plus loin, plus petits, occultes par la pince et balaient une surface
# d'image bien plus large : les compenser est legitime.
DEFAULT_KP_WEIGHTS = [1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 6.0]

TRAIN_FRAC = 0.80
VAL_FRAC = 0.10
TEST_FRAC = 0.10

SEED = 42


def _load_pretrained(dream_network, path):
    """Warm-start the model from a DREAM .pth state_dict (handles 'module.' prefix)."""
    sd = torch.load(path, map_location="cpu")
    if isinstance(sd, dict) and "state_dict" in sd and not any(
        hasattr(v, "shape") for v in list(sd.values())[:1]
    ):
        sd = sd["state_dict"]
    model = dream_network.model
    try:
        model.load_state_dict(sd)
        print(f"✅ Pretrained weights loaded (strict): {path}")
        return
    except Exception as e:
        print(f"  strict load failed ({type(e).__name__}); adjusting 'module.' prefix")
    model_keys = list(model.state_dict().keys())
    want_module = model_keys[0].startswith("module.")
    fixed = {}
    for k, v in sd.items():
        if want_module and not k.startswith("module."):
            k = "module." + k
        elif not want_module and k.startswith("module."):
            k = k[len("module."):]
        fixed[k] = v
    missing, unexpected = model.load_state_dict(fixed, strict=False)
    print(f"✅ Pretrained weights loaded (non-strict) from {path}: "
          f"{len(fixed)} tensors | missing={len(missing)} unexpected={len(unexpected)}")


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
    """MSE loss with per-keypoint channel weighting (identical to v4)."""

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


def _patch_augmentation():
    """Geometrie a DREAM (keypoints suivis), photometrie par-dessus.

    Une seule greffe : `__getitem__` laisse `augment_data` a True — donc DREAM
    fait sa geometrie et recalcule les belief maps — et n'ajoute que des
    transformees photometriques, qui ne deplacent aucun pixel de place.

    Aucun reglage geometrique n'est injecte : ceux de DREAM (rotate 15,
    shift 0,0625, scale 0,1) restent tels quels. Voir l'en-tete du module.
    """
    import dream.datasets as dream_datasets

    original_getitem = dream_datasets.ManipulatorNDDSDataset.__getitem__

    def patched_getitem(self, idx):
        if self.augment_data:
            sample = original_getitem(self, idx)

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
                albu.RandomGamma(gamma_limit=(70, 130), p=0.3),
                albu.ImageCompression(quality_range=(50, 95), p=0.3),
                albu.CoarseDropout(
                    num_holes_range=(2, 6),
                    hole_height_range=(15, 60),
                    hole_width_range=(15, 60),
                    fill="random",
                    p=0.4,
                ),
                # NI `Perspective` NI `ShiftScaleRotate` ici : elles deplacent
                # les pixels et ce pipeline ne voit pas les keypoints. La
                # geometrie est faite au-dessus, par DREAM, qui les transforme.
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
    print("[PATCH] Geometrie a DREAM avec keypoints (ses reglages inchanges), "
          "photometrie image-seule par-dessus")


def _save_learning_curve(output_dir, epochs, val_losses, test_losses):
    """Val-vs-test curve with overfitting-gap warning, from v10 — saved to
    disk every epoch instead of shown interactively (headless run)."""
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(epochs, val_losses, "o-", color="orange", label="Val loss", linewidth=2)
    ax.plot(epochs, test_losses, "s-", color="green", label="Test loss", linewidth=2)
    ax.set_xlabel("Epoch")
    ax.set_ylabel("Loss")
    ax.grid(True, alpha=0.3)
    ax.legend()
    if len(val_losses) > 3:
        gap = test_losses[-1] - val_losses[-1]
        if gap > 0.0005:
            ax.set_title(f"⚠️ Possible overfitting — gap val/test: {gap:.6f}", color="red")
        else:
            ax.set_title(f"✅ Bonne généralisation — gap val/test: {gap:.6f}", color="green")
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "learning_curve.png"), dpi=100, bbox_inches="tight")
    plt.close(fig)


def train_weighted(args):
    """Main training loop with weighted loss (identical to v4 except for the
    seeded shuffle-split and the val/test learning curve)."""

    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    torch.cuda.manual_seed_all(SEED)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    print(f"🌱 Seed fixe : {SEED}")

    _patch_augmentation()

    MANIP_CONFIG = os.path.join(SCRIPT_DIR, "manip_configs", "mycobot320.yaml")
    arch_map = {
        "vgg": os.path.join(DREAM_DIR, "arch_configs", "dream_vgg_q.yaml"),
        "resnet": os.path.join(DREAM_DIR, "arch_configs", "dream_resnet_h.yaml"),
    }
    arch_config_path = arch_map[args.arch]

    # BARRIERE : jamais ecrire dans un dossier qui contient deja des poids.
    # Le defaut herite du v4 pointait sur `vgg_ultimate_v4_mix_ft_e30`, donc sur
    # un checkpoint existant ; il a ete corrige, mais un defaut peut se retromper
    # et un `--output` peut se taper de travers. On refuse plutot que d'ecraser :
    # un entrainement se relance, des poids perdus ne se retrouvent pas.
    if glob.glob(os.path.join(args.output, "*.pth")):
        raise SystemExit(
            f"REFUS : {args.output} contient deja des .pth.\n"
            f"Ce script n'ecrase jamais un checkpoint existant. "
            f"Donnez un --output neuf.")
    os.makedirs(args.output, exist_ok=True)

    kp_weights = [float(w) for w in args.kp_weights.split(",")]
    assert len(kp_weights) == 7, f"Expected 7 keypoint weights, got {len(kp_weights)}"
    print(f"\n🎯 Per-keypoint loss weights:")
    kp_names = ["base", "link1", "link2", "link3", "link4", "link5", "link6"]
    for name, w in zip(kp_names, kp_weights):
        marker = "⬆️" if w > 1.0 else "  "
        print(f"  {marker} {name:<8s}: {w:.1f}x")
    print()

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

    data_augment_config = odict([("image_rgb", True)])

    found_data = dream.utilities.find_ndds_data_in_dir(args.data)
    found_data_config = found_data[1]
    image_raw_resolution = dream.utilities.load_image_resolution(
        found_data_config["camera"]
    )

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
            ])),
            ("platform", odict([
                ("user", user),
                ("hostname", socket.gethostname()),
                ("gpu_ids", [args.gpu]),
            ])),
            ("results", odict([("epochs_trained", 0)])),
        ])),
    ])

    print(f"Creating {args.arch} network...")
    dream_network = dream.create_network_from_config_data(network_config)

    if args.pretrained:
        _load_pretrained(dream_network, args.pretrained)

    weighted_criterion = WeightedBeliefMapLoss(kp_weights).cuda()
    dream_network.criterion = weighted_criterion
    print(f"✅ Weighted loss installed (weights sum={sum(kp_weights):.1f})")

    dream_network.enable_training()

    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        dream_network.optimizer,
        T_max=args.epochs,
        eta_min=args.lr * 0.01,
    )
    print(f"📈 Cosine annealing LR: {args.lr} → {args.lr * 0.01:.6f}")

    trained_net_input_res, trained_net_output_res = \
        dream_network.net_resolutions_from_image_raw_resolution(image_raw_resolution)
    dream_network.network_config["training"]["config"]["net_output_resolution"] = \
        trained_net_output_res

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

    # 80/10/10 split, seeded (v4 used torch.utils.data.random_split with NO
    # seed -> a different, unreproducible val set every run). Indices are
    # shuffled once with the fixed seed so both synth and oversampled-real
    # frames (concatenated, not interleaved, by merge_ndds.py) land in all
    # three splits in a representative mix.
    n_data = len(found_dataset)
    indices = list(range(n_data))
    rng = random.Random(SEED)
    rng.shuffle(indices)

    n_train = int(round(n_data * TRAIN_FRAC))
    n_val = int(round(n_data * VAL_FRAC))
    train_idx = indices[:n_train]
    val_idx = indices[n_train:n_train + n_val]
    test_idx = indices[n_train + n_val:]
    n_test = len(test_idx)

    train_dataset = torch.utils.data.Subset(found_dataset, train_idx)
    valid_dataset = torch.utils.data.Subset(found_dataset, val_idx)
    test_dataset = torch.utils.data.Subset(found_dataset, test_idx)

    train_loader = TorchDataLoader(
        train_dataset, batch_size=args.batch_size, num_workers=args.workers,
        shuffle=True, pin_memory=True,
    )
    valid_loader = TorchDataLoader(
        valid_dataset, batch_size=args.batch_size, num_workers=args.workers,
        pin_memory=True,
    )
    test_loader = TorchDataLoader(
        test_dataset, batch_size=args.batch_size, num_workers=args.workers,
        pin_memory=True,
    )

    print(f"\n📊 Dataset: {n_data} total (seed={SEED}, shuffled)")
    print(f"   Train : {n_train} ({TRAIN_FRAC*100:.0f}%)")
    print(f"   Val   : {n_val} ({VAL_FRAC*100:.0f}%) → early stopping")
    print(f"   Test  : {n_test} ({TEST_FRAC*100:.0f}%) → courbe overfitting")
    print(f"   Input res:  {trained_net_input_res}")
    print(f"   Output res: {trained_net_output_res}")
    print(f"\n{'='*70}")
    print(f"TRAINING: {args.arch} MIX | {args.epochs} epochs | BS={args.batch_size} | LR={args.lr}")
    print(f"  Pretrained     : {args.pretrained}")
    print(f"  Weighted loss  : {kp_weights}")
    print(f"  Split          : 80/10/10 (seed={SEED})")
    print(f"  Cosine annealing LR schedule")
    print(f"  Augmentation : geometrie par DREAM (keypoints suivis), "
          f"photometrie par-dessus")
    print(f"{'='*70}\n")

    train_log = {
        "epochs": [],
        "losses": [],
        "validation_losses": [],
        "test_losses": [],
        "per_kp_val_losses": [],
        "lr_history": [],
        "start_time": time.time(),
        "timestamps": [],
        "keypoint_weights": kp_weights,
        "seed": SEED,
        "split": {"train": n_train, "val": n_val, "test": n_test},
        "pretrained": args.pretrained,
    }

    best_valid_loss = float("inf")
    early_stopping = EarlyStopping(patience=args.patience)
    epochs_list, val_losses_list, test_losses_list = [], [], []

    for epoch in range(args.epochs):
        epoch_num = epoch + 1
        current_lr = dream_network.optimizer.param_groups[0]["lr"]

        # ---- Training ----
        dream_network.enable_training()
        train_losses = []

        for batch in tqdm(train_loader, desc=f"Epoch {epoch_num}/{args.epochs} [train]",
                           leave=False):
            network_input = batch["image_rgb_input"].cuda()
            target = batch["belief_maps"].cuda()

            dream_network.optimizer.zero_grad()
            output = dream_network.model(network_input)
            pred = output[0] if isinstance(output, list) else output

            loss = weighted_criterion(pred, target)
            loss.backward()
            dream_network.optimizer.step()

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
                network_input = batch["image_rgb_input"].cuda()
                target = batch["belief_maps"].cuda()

                output = dream_network.model(network_input)
                pred = output[0] if isinstance(output, list) else output

                loss = weighted_criterion(pred, target)
                valid_losses.append(loss.item())

                per_kp_mse = ((pred - target) ** 2).mean(dim=(0, 2, 3))
                per_kp_losses_accum += per_kp_mse.cpu().numpy()
                per_kp_counts += 1

        mean_valid_loss = np.mean(valid_losses)
        per_kp_avg = per_kp_losses_accum / max(per_kp_counts, 1)

        # ---- Test (monitoring only, for the overfitting-gap curve) ----
        test_losses = []
        with torch.no_grad():
            for batch in tqdm(test_loader, desc=f"Epoch {epoch_num}/{args.epochs} [test] ",
                               leave=False):
                network_input = batch["image_rgb_input"].cuda()
                target = batch["belief_maps"].cuda()
                output = dream_network.model(network_input)
                pred = output[0] if isinstance(output, list) else output
                loss = weighted_criterion(pred, target)
                test_losses.append(loss.item())
        mean_test_loss = np.mean(test_losses)

        scheduler.step()

        train_log["epochs"].append(epoch_num)
        train_log["losses"].append(float(mean_train_loss))
        train_log["validation_losses"].append(float(mean_valid_loss))
        train_log["test_losses"].append(float(mean_test_loss))
        train_log["per_kp_val_losses"].append(per_kp_avg.tolist())
        train_log["lr_history"].append(current_lr)
        train_log["timestamps"].append(time.time())

        gap = mean_test_loss - mean_valid_loss
        overfit_marker = f" ⚠️ gap={gap:.6f}" if gap > 0.0005 else ""

        is_best = mean_valid_loss < best_valid_loss
        marker = " ⭐ BEST" if is_best else ""
        print(f"Epoch {epoch_num:3d}/{args.epochs} | "
              f"train={mean_train_loss:.6f} | val={mean_valid_loss:.6f} | "
              f"test={mean_test_loss:.6f} | lr={current_lr:.6f}{marker}{overfit_marker}")

        if epoch_num % 5 == 0 or epoch_num == 1 or is_best:
            print(f"  Per-KP val MSE (unweighted): ", end="")
            for ki, name in enumerate(kp_names):
                print(f"{name}={per_kp_avg[ki]:.6f}", end="  ")
            print()

        epochs_list.append(epoch_num)
        val_losses_list.append(mean_valid_loss)
        test_losses_list.append(mean_test_loss)
        _save_learning_curve(args.output, epochs_list, val_losses_list, test_losses_list)

        dream_network.network_config["training"]["results"]["epochs_trained"] = epoch_num
        dream_network.network_config["training"]["results"]["training_loss"] = odict([
            ("mean", float(mean_train_loss)),
            ("stdev", float(np.std(train_losses))),
        ])
        dream_network.network_config["training"]["results"]["validation_loss"] = odict([
            ("mean", float(mean_valid_loss)),
            ("stdev", float(np.std(valid_losses))),
        ])

        if epoch_num % 10 == 0:
            dream_network.save_network(args.output, f"epoch_{epoch_num}", overwrite=True)
            print(f"  💾 Saved checkpoint: epoch_{epoch_num}")

        if early_stopping(mean_valid_loss, epoch_num):
            best_valid_loss = mean_valid_loss
            dream_network.save_network(args.output, "best_network", overwrite=True)
            print(f"  🏆 New best model saved (val_loss={best_valid_loss:.6f})")
        if early_stopping.should_stop:
            print(f"\n🛑🏆 Early stopping → meilleur epoch : {early_stopping.best_epoch}")
            break

    dream_network.save_network(args.output, f"epoch_{args.epochs}", overwrite=True)

    log_path = os.path.join(args.output, "training_log.pkl")
    with open(log_path, "wb") as f:
        pickle.dump(train_log, f)

    curve_path = os.path.join(args.output, "learning_curve_final.png")
    if epochs_list:
        _save_learning_curve(args.output, epochs_list, val_losses_list, test_losses_list)
        os.replace(os.path.join(args.output, "learning_curve.png"), curve_path)
    print(f"  📊 Courbe sauvegardée : {curve_path}")

    elapsed = time.time() - train_log["start_time"]
    print(f"\n{'='*70}")
    print(f"🎉 TRAINING COMPLETE")
    print(f"  Total time: {elapsed/60:.1f} min ({elapsed/3600:.1f} hours)")
    print(f"  Best validation loss: {best_valid_loss:.6f}")
    print(f"  Output: {args.output}")
    print(f"{'='*70}")

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
        description="DREAM training v5 — identique au v4 MIX, augmentation geometrique reparee (keypoints suivis). Poids et tout le reste inchanges."
    )
    parser.add_argument("--data", "-d", required=True,
                         help="NDDS data directory (merged synth+real, see merge_ndds.py)")
    parser.add_argument("--pretrained", "-p", default=None,
                         help="Optional .pth to warm-start from "
                              "(e.g. output/checkpoints_dream/vgg_ultimate_v4_e50/best_network.pth)")
    parser.add_argument("--arch", default="vgg", choices=["vgg", "resnet"],
                         help="Architecture (default: vgg)")
    parser.add_argument("--epochs", "-e", type=int, default=30,
                         help="Training epochs (default: 30 — fine-tune, not from scratch)")
    parser.add_argument("--batch-size", "-b", type=int, default=8,
                         help="Batch size (default: 8)")
    parser.add_argument("--lr", type=float, default=0.0001,
                         help="Initial learning rate (default: 0.0001)")
    parser.add_argument("--workers", "-w", type=int, default=8,
                         help="Data loader workers (default: 8)")
    parser.add_argument("--gpu", "-g", type=int, default=0,
                         help="GPU ID (default: 0)")
    parser.add_argument("--output", "-o", default=None,
                         help="Output directory for checkpoints")
    parser.add_argument("--kp-weights", default=",".join(str(w) for w in DEFAULT_KP_WEIGHTS),
                         help="Comma-separated per-keypoint loss weights "
                              "(base,link1,link2,link3,link4,link5,link6). "
                              f"Default: {','.join(str(w) for w in DEFAULT_KP_WEIGHTS)}")
    parser.add_argument("--patience", type=int, default=5,
                         help="Early stopping patience (default: 5)")

    args = parser.parse_args()

    if args.output is None:
        weights_tag = "_".join(f"{w:.0f}" for w in
                                [float(w) for w in args.kp_weights.split(",")])
        tag = "_ft" if args.pretrained else ""
        args.output = os.path.join(
            SCRIPT_DIR, "checkpoints_dream",
            f"vgg_v5_geo{tag}_e{args.epochs}"
        )
    train_weighted(args)


if __name__ == "__main__":
    main()
