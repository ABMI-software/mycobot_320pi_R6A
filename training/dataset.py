#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PyTorch Dataset for MyCobot synthetic pose estimation data (v2).

Supports both single-view and multi-view datasets.

Single-view layout (v1 compatible)::

    dataset_dir/
    ├── images/
    │   ├── 000000.png
    │   └── ...
    └── labels.csv          # index, j*_rad, j*_deg, image_path

Multi-view layout (v2)::

    dataset_dir/
    ├── images/
    │   ├── front/000000.png ...
    │   ├── right/000000.png ...
    │   ├── left/000000.png  ...
    │   └── top/000000.png   ...
    └── labels.csv          # index, j*_rad, j*_deg, camera, image_path

The dataset auto-detects the format from the CSV columns.
"""

import csv
import os
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms


# ---------- Normalisation constants ----------
JOINT_LIMITS = np.array([
    [-2.96, 2.96],   # joint1
    [-2.79, 2.79],   # joint2
    [-2.79, 2.79],   # joint3
    [-2.79, 2.79],   # joint4
    [-2.96, 2.96],   # joint5
    [-3.05, 3.05],   # joint6
], dtype=np.float32)

JOINT_RANGE = JOINT_LIMITS[:, 1] - JOINT_LIMITS[:, 0]
JOINT_MID   = (JOINT_LIMITS[:, 1] + JOINT_LIMITS[:, 0]) / 2.0

CAMERA_VIEWS = ['front', 'right', 'left', 'top']


def normalize_angles(angles: np.ndarray) -> np.ndarray:
    """Map raw radians → [-1, 1] using known joint limits."""
    return (angles - JOINT_MID) / (JOINT_RANGE / 2.0)


def denormalize_angles(normed: np.ndarray) -> np.ndarray:
    """Inverse of :func:`normalize_angles`."""
    return normed * (JOINT_RANGE / 2.0) + JOINT_MID


# ---------- Image transforms ----------
def get_train_transforms(image_size: int = 224) -> transforms.Compose:
    """Augmented transforms for training (heavier than v1)."""
    return transforms.Compose([
        transforms.Resize((image_size, image_size)),
        transforms.ColorJitter(brightness=0.3, contrast=0.3,
                               saturation=0.3, hue=0.08),
        transforms.RandomGrayscale(p=0.05),
        transforms.GaussianBlur(kernel_size=5, sigma=(0.1, 2.0)),
        transforms.RandomAffine(degrees=3, translate=(0.02, 0.02),
                                scale=(0.95, 1.05)),
        transforms.ToTensor(),
        transforms.RandomErasing(p=0.1, scale=(0.02, 0.08)),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])


def get_eval_transforms(image_size: int = 224) -> transforms.Compose:
    """Deterministic transforms for validation / inference."""
    return transforms.Compose([
        transforms.Resize((image_size, image_size)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])


# ---------- Single-view Dataset ----------
class MyCobotPoseDataset(Dataset):
    """Dataset of (image, joint_angles) pairs — one image per sample.

    Supports both v1 (no camera column) and v2 (with camera column) CSV.
    When a v2 CSV is loaded, only images from the specified ``camera_filter``
    are included (default: all cameras → each view = separate sample).
    """

    RAD_COLS = ['j1_rad', 'j2_rad', 'j3_rad', 'j4_rad', 'j5_rad', 'j6_rad']

    def __init__(
        self,
        dataset_dir: str,
        transform: Optional[Callable] = None,
        normalize_targets: bool = True,
        camera_filter: Optional[str] = None,
    ):
        self.dataset_dir = dataset_dir
        self.transform = transform or get_eval_transforms()
        self.normalize_targets = normalize_targets

        csv_path = os.path.join(dataset_dir, 'labels.csv')
        self.samples: List[Tuple[str, np.ndarray]] = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            has_camera_col = 'camera' in (reader.fieldnames or [])
            for row in reader:
                if has_camera_col and camera_filter is not None:
                    if row['camera'] != camera_filter:
                        continue
                img_rel = row['image_path']
                angles = np.array([float(row[c]) for c in self.RAD_COLS],
                                  dtype=np.float32)
                self.samples.append((img_rel, angles))

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        img_rel, angles = self.samples[idx]
        img_path = os.path.join(self.dataset_dir, img_rel)

        image = Image.open(img_path).convert('RGB')
        image = self.transform(image)

        if self.normalize_targets:
            angles = normalize_angles(angles)

        target = torch.from_numpy(angles)
        return image, target


# ---------- Multi-view Dataset ----------
class MyCobotMultiViewDataset(Dataset):
    """Dataset that returns multiple camera views per pose sample.

    Groups all camera views sharing the same ``index`` value.
    Returns stacked view tensors (N_views, 3, H, W) + joint angles.
    Used with :class:`MultiViewPoseResNet`.
    """

    RAD_COLS = ['j1_rad', 'j2_rad', 'j3_rad', 'j4_rad', 'j5_rad', 'j6_rad']

    def __init__(
        self,
        dataset_dir: str,
        views: Optional[List[str]] = None,
        transform: Optional[Callable] = None,
        normalize_targets: bool = True,
    ):
        self.dataset_dir = dataset_dir
        self.views = views or CAMERA_VIEWS
        self.transform = transform or get_eval_transforms()
        self.normalize_targets = normalize_targets

        csv_path = os.path.join(dataset_dir, 'labels.csv')
        grouped: Dict[int, Dict[str, Tuple[str, np.ndarray]]] = {}
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                idx = int(row['index'])
                cam = row.get('camera', 'front')
                if cam not in self.views:
                    continue
                img_rel = row['image_path']
                angles = np.array([float(row[c]) for c in self.RAD_COLS],
                                  dtype=np.float32)
                if idx not in grouped:
                    grouped[idx] = {}
                grouped[idx][cam] = (img_rel, angles)

        # Only keep samples that have ALL required views
        self.samples: List[Dict[str, Tuple[str, np.ndarray]]] = []
        for idx_key in sorted(grouped.keys()):
            if all(v in grouped[idx_key] for v in self.views):
                self.samples.append(grouped[idx_key])

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int):
        sample = self.samples[idx]
        images = {}
        angles = None
        for view_name in self.views:
            img_rel, ang = sample[view_name]
            img_path = os.path.join(self.dataset_dir, img_rel)
            img = Image.open(img_path).convert('RGB')
            images[view_name] = self.transform(img)
            angles = ang

        if self.normalize_targets:
            angles = normalize_angles(angles)

        stacked = torch.stack([images[v] for v in self.views], dim=0)
        target = torch.from_numpy(angles)
        return stacked, target


# ---------- Merged Dataset (synthetic + real) ----------
class MergedPoseDataset(Dataset):
    """Combine multiple :class:`MyCobotPoseDataset` instances into one.

    Useful for merging synthetic + real-world data for fine-tuning.
    """

    def __init__(self, datasets: List[Dataset]):
        self.datasets = datasets
        self.cumulative = []
        total = 0
        for ds in datasets:
            total += len(ds)
            self.cumulative.append(total)
        self._total = total

    def __len__(self) -> int:
        return self._total

    def __getitem__(self, idx: int):
        for i, cum in enumerate(self.cumulative):
            if idx < cum:
                offset = cum - len(self.datasets[i])
                return self.datasets[i][idx - offset]
        raise IndexError(f'Index {idx} out of range')
