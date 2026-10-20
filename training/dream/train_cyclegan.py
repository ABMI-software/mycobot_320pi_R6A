#!/usr/bin/env python3
"""CycleGAN pour Domain Adaptation Synth → Réel.

Entraîne un CycleGAN pour traduire les images synthétiques
en style réel, réduisant le sim-to-real gap.

Architecture :
  G_S2R : Générateur Synth → Réel
  G_R2S : Générateur Réel → Synth
  D_R   : Discriminateur Réel
  D_S   : Discriminateur Synth

Usage:
  python dream/train_cyclegan.py \
      --data-synth dream/dream_data/synthetic \
      --data-real  dream/dream_data/real \
      --output     dream/cyclegan_checkpoints \
      --epochs     50 \
      --batch-size 4
"""

import os
import sys
import time
import itertools
import random
from tqdm import tqdm

import numpy as np
from PIL import Image
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as transforms

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ============================================================
# Réseau — Blocs Résiduels + Générateur + Discriminateur
# ============================================================

class ResidualBlock(nn.Module):
    def __init__(self, channels):
        super().__init__()
        self.block = nn.Sequential(
            nn.ReflectionPad2d(1),
            nn.Conv2d(channels, channels, 3),
            nn.InstanceNorm2d(channels),
            nn.ReLU(inplace=True),
            nn.ReflectionPad2d(1),
            nn.Conv2d(channels, channels, 3),
            nn.InstanceNorm2d(channels),
        )

    def forward(self, x):
        return x + self.block(x)


class Generator(nn.Module):
    """Générateur CycleGAN avec 9 blocs résiduels."""
    def __init__(self, n_residual=9):
        super().__init__()
        model = [
            nn.ReflectionPad2d(3),
            nn.Conv2d(3, 64, 7),
            nn.InstanceNorm2d(64),
            nn.ReLU(inplace=True),
            # Downsampling
            nn.Conv2d(64, 128, 3, stride=2, padding=1),
            nn.InstanceNorm2d(128),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 256, 3, stride=2, padding=1),
            nn.InstanceNorm2d(256),
            nn.ReLU(inplace=True),
        ]
        # Blocs résiduels
        for _ in range(n_residual):
            model.append(ResidualBlock(256))
        # Upsampling
        model += [
            nn.ConvTranspose2d(256, 128, 3, stride=2, padding=1, output_padding=1),
            nn.InstanceNorm2d(128),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(128, 64, 3, stride=2, padding=1, output_padding=1),
            nn.InstanceNorm2d(64),
            nn.ReLU(inplace=True),
            nn.ReflectionPad2d(3),
            nn.Conv2d(64, 3, 7),
            nn.Tanh(),
        ]
        self.model = nn.Sequential(*model)

    def forward(self, x):
        return self.model(x)


class Discriminator(nn.Module):
    """PatchGAN Discriminateur."""
    def __init__(self):
        super().__init__()
        self.model = nn.Sequential(
            nn.Conv2d(3, 64, 4, stride=2, padding=1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(64, 128, 4, stride=2, padding=1),
            nn.InstanceNorm2d(128),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(128, 256, 4, stride=2, padding=1),
            nn.InstanceNorm2d(256),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(256, 512, 4, padding=1),
            nn.InstanceNorm2d(512),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(512, 1, 4, padding=1),
        )

    def forward(self, x):
        return self.model(x)


# ============================================================
# Dataset
# ============================================================

class ImageDataset(Dataset):
    def __init__(self, data_dir, transform, max_samples=None):
        self.files = sorted([
            os.path.join(data_dir, f)
            for f in os.listdir(data_dir)
            if f.endswith(".rgb.png") and not f.startswith("_")
        ])
        if max_samples:
            self.files = self.files[:max_samples]
        self.transform = transform

    def __len__(self):
        return len(self.files)

    def __getitem__(self, idx):
        img = Image.open(self.files[idx % len(self.files)]).convert("RGB")
        return self.transform(img)


class ReplayBuffer:
    """Buffer pour stabiliser l'entraînement du discriminateur."""
    def __init__(self, max_size=50):
        self.max_size = max_size
        self.data = []

    def push_and_pop(self, data):
        result = []
        for element in data:
            if len(self.data) < self.max_size:
                self.data.append(element)
                result.append(element)
            else:
                if random.random() > 0.5:
                    i = random.randint(0, self.max_size - 1)
                    result.append(self.data[i].clone())
                    self.data[i] = element
                else:
                    result.append(element)
        return torch.stack(result)


def weights_init(m):
    if isinstance(m, (nn.Conv2d, nn.ConvTranspose2d)):
        nn.init.normal_(m.weight.data, 0.0, 0.02)
    elif isinstance(m, nn.InstanceNorm2d):
        if m.weight is not None:
            nn.init.normal_(m.weight.data, 1.0, 0.02)
        if m.bias is not None:
            nn.init.constant_(m.bias.data, 0)


# ============================================================
# Training
# ============================================================

def train(args):
    os.makedirs(args.output, exist_ok=True)

    # Transforms
    transform = transforms.Compose([
        transforms.Resize((args.img_size, args.img_size)),
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)),
    ])

    # Datasets
    synth_dataset = ImageDataset(args.data_synth, transform, max_samples=args.max_synth)
    real_dataset  = ImageDataset(args.data_real,  transform, max_samples=args.max_real)

    synth_loader = DataLoader(synth_dataset, batch_size=args.batch_size,
                              shuffle=True, num_workers=4, pin_memory=True)
    real_loader  = DataLoader(real_dataset,  batch_size=args.batch_size,
                              shuffle=True, num_workers=4, pin_memory=True)

    print(f"\n📊 Dataset :")
    print(f"  Synth : {len(synth_dataset)} images")
    print(f"  Réel  : {len(real_dataset)} images")

    # Modèles
    G_S2R = Generator().cuda()  # Synth → Réel
    G_R2S = Generator().cuda()  # Réel → Synth
    D_R   = Discriminator().cuda()
    D_S   = Discriminator().cuda()

    G_S2R.apply(weights_init)
    G_R2S.apply(weights_init)
    D_R.apply(weights_init)
    D_S.apply(weights_init)

    # Pertes
    criterion_GAN    = nn.MSELoss()
    criterion_cycle  = nn.L1Loss()
    criterion_ident  = nn.L1Loss()

    # Optimiseurs
    opt_G = torch.optim.Adam(
        itertools.chain(G_S2R.parameters(), G_R2S.parameters()),
        lr=args.lr, betas=(0.5, 0.999)
    )
    opt_D_R = torch.optim.Adam(D_R.parameters(), lr=args.lr, betas=(0.5, 0.999))
    opt_D_S = torch.optim.Adam(D_S.parameters(), lr=args.lr, betas=(0.5, 0.999))

    # LR Scheduler — decay après moitié des epochs
    def lr_lambda(epoch):
        return 1.0 - max(0, epoch - args.epochs // 2) / float(args.epochs // 2 + 1)

    scheduler_G   = torch.optim.lr_scheduler.LambdaLR(opt_G,   lr_lambda)
    scheduler_D_R = torch.optim.lr_scheduler.LambdaLR(opt_D_R, lr_lambda)
    scheduler_D_S = torch.optim.lr_scheduler.LambdaLR(opt_D_S, lr_lambda)

    fake_R_buffer = ReplayBuffer()
    fake_S_buffer = ReplayBuffer()

    print(f"\n{'='*60}")
    print(f"CYCLEGAN TRAINING")
    print(f"  Epochs     : {args.epochs}")
    print(f"  Batch size : {args.batch_size}")
    print(f"  LR         : {args.lr}")
    print(f"  Output     : {args.output}")
    print(f"{'='*60}\n")

    t0 = time.time()

    for epoch in range(args.epochs):
        epoch_num = epoch + 1
        losses_G, losses_D = [], []

        real_iter  = iter(real_loader)
        synth_iter = iter(synth_loader)
        n_batches  = min(len(real_loader), len(synth_loader))

        for i in tqdm(range(n_batches), desc=f"Epoch {epoch_num}/{args.epochs}", unit="batch"):
            try:
                real_imgs  = next(real_iter).cuda()
                synth_imgs = next(synth_iter).cuda()
            except StopIteration:
                break

            valid = torch.ones_like(D_R(real_imgs), requires_grad=False)
            fake_label = torch.zeros_like(valid)

            # ---- Générateurs ----
            opt_G.zero_grad()

            # Identity loss
            loss_ident_R = criterion_ident(G_S2R(real_imgs),  real_imgs)  * args.lambda_ident
            loss_ident_S = criterion_ident(G_R2S(synth_imgs), synth_imgs) * args.lambda_ident

            # GAN loss
            fake_R = G_S2R(synth_imgs)
            fake_S = G_R2S(real_imgs)
            loss_GAN_S2R = criterion_GAN(D_R(fake_R), valid)
            loss_GAN_R2S = criterion_GAN(D_S(fake_S), valid)

            # Cycle loss
            recov_S = G_R2S(fake_R)
            recov_R = G_S2R(fake_S)
            loss_cycle_S = criterion_cycle(recov_S, synth_imgs) * args.lambda_cycle
            loss_cycle_R = criterion_cycle(recov_R, real_imgs)  * args.lambda_cycle

            loss_G = (loss_GAN_S2R + loss_GAN_R2S +
                      loss_cycle_S + loss_cycle_R +
                      loss_ident_R + loss_ident_S)
            loss_G.backward()
            opt_G.step()
            losses_G.append(loss_G.item())

            # ---- Discriminateur Réel ----
            opt_D_R.zero_grad()
            fake_R_buf = fake_R_buffer.push_and_pop(fake_R.detach())
            loss_D_R = (criterion_GAN(D_R(real_imgs), valid) +
                        criterion_GAN(D_R(fake_R_buf), fake_label)) * 0.5
            loss_D_R.backward()
            opt_D_R.step()

            # ---- Discriminateur Synth ----
            opt_D_S.zero_grad()
            fake_S_buf = fake_S_buffer.push_and_pop(fake_S.detach())
            loss_D_S = (criterion_GAN(D_S(synth_imgs), valid) +
                        criterion_GAN(D_S(fake_S_buf), fake_label)) * 0.5
            loss_D_S.backward()
            opt_D_S.step()

            losses_D.append((loss_D_R + loss_D_S).item())

           

        scheduler_G.step()
        scheduler_D_R.step()
        scheduler_D_S.step()

        mean_G = np.mean(losses_G)
        mean_D = np.mean(losses_D)
        elapsed = (time.time() - t0) / 60

        print(f"Epoch {epoch_num:3d}/{args.epochs} | "
              f"loss_G={mean_G:.4f} | loss_D={mean_D:.4f} | "
              f"{elapsed:.1f} min")

        # Sauvegarder checkpoints selon --save-interval
        if epoch_num % args.save_interval == 0 or epoch_num == args.epochs:
            torch.save(G_S2R.state_dict(), os.path.join(args.output, f"G_S2R_epoch{epoch_num}.pth"))
            torch.save(G_R2S.state_dict(), os.path.join(args.output, f"G_R2S_epoch{epoch_num}.pth"))
            print(f"  💾 Checkpoints sauvegardés → epoch {epoch_num}")

        # Sauvegarder quelques images exemples tous les 5 epochs
        if epoch_num % 5 == 0:
            sample_dir = os.path.join(args.output, "samples")
            os.makedirs(sample_dir, exist_ok=True)
            with torch.no_grad():
                fake_real_sample = G_S2R(synth_imgs[:1])
                # Dénormaliser
                img_np = fake_real_sample[0].cpu().numpy().transpose(1, 2, 0)
                img_np = ((img_np * 0.5 + 0.5) * 255).clip(0, 255).astype(np.uint8)
                Image.fromarray(img_np).save(
                    os.path.join(sample_dir, f"fake_real_epoch{epoch_num}.png")
                )

    elapsed_total = (time.time() - t0) / 60
    print(f"\n{'='*60}")
    print(f"✅ CYCLEGAN TERMINÉ !")
    print(f"  Durée totale : {elapsed_total:.1f} min")
    print(f"  Output       : {args.output}")
    print(f"{'='*60}")
    print(f"\nProchaine étape :")
    print(f"  python dream/apply_cyclegan.py \\")
    print(f"    --weights {args.output}/G_S2R_epoch{args.epochs}.pth \\")
    print(f"    --data dream/dream_data/synthetic \\")
    print(f"    --output dream/dream_data/synth_real_style")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="CycleGAN Synth → Réel")
    parser.add_argument("--data-synth",    required=True, help="Dossier images synthétiques")
    parser.add_argument("--data-real",     required=True, help="Dossier images réelles")
    parser.add_argument("--output",        required=True, help="Dossier output checkpoints")
    parser.add_argument("--epochs",        type=int, default=50)
    parser.add_argument("--batch-size",    type=int, default=4)
    parser.add_argument("--lr",            type=float, default=0.0002)
    parser.add_argument("--lambda-cycle",  type=float, default=10.0)
    parser.add_argument("--lambda-ident",  type=float, default=5.0)
    parser.add_argument("--max-synth",     type=int, default=5000,
                        help="Max images synth utilisées (default: 5000)")
    parser.add_argument("--max-real",      type=int, default=3000,
                        help="Max images réelles utilisées (default: 3000)")
    parser.add_argument("--save-interval", type=int, default=10,
                        help="Sauvegarder checkpoints tous les N epochs (default: 10)")
    parser.add_argument("--img-size",     type=int, default=256,
                        help="Taille des images (256, 512, 640)")
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()