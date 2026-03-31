#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ResNet-based regression models for MyCobot pose estimation.

Single-view:  PoseResNet           — one image  → 6 joint angles
Multi-view:   MultiViewPoseResNet  — N images   → 6 joint angles (fused)
"""

from typing import Optional

import torch
import torch.nn as nn
from torchvision import models


class PoseResNet(nn.Module):
    """ResNet backbone + regression head → 6 joint angles.

    Parameters
    ----------
    backbone : str
        One of ``'resnet18'``, ``'resnet34'``, ``'resnet50'``.
    pretrained : bool
        Use ImageNet-pretrained weights.
    num_joints : int
        Number of output values (6 for MyCobot 320).
    dropout : float
        Dropout probability in the regression head.
    """

    BACKBONES = {
        'resnet18': (models.resnet18, models.ResNet18_Weights.DEFAULT, 512),
        'resnet34': (models.resnet34, models.ResNet34_Weights.DEFAULT, 512),
        'resnet50': (models.resnet50, models.ResNet50_Weights.DEFAULT, 2048),
    }

    def __init__(
        self,
        backbone: str = 'resnet18',
        pretrained: bool = True,
        num_joints: int = 6,
        dropout: float = 0.3,
    ):
        super().__init__()
        if backbone not in self.BACKBONES:
            raise ValueError(f'Unknown backbone {backbone!r}. '
                             f'Choose from {list(self.BACKBONES)}')

        factory, weights, feat_dim = self.BACKBONES[backbone]
        base = factory(weights=weights if pretrained else None)

        # Keep everything except the final FC layer
        self.features = nn.Sequential(*list(base.children())[:-1])  # → (B, feat_dim, 1, 1)

        # Regression head
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(feat_dim, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(256, num_joints),
        )

        self._num_joints = num_joints
        self._backbone_name = backbone
        self._feat_dim = feat_dim

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass.  ``x`` shape: (B, 3, H, W) → (B, num_joints)."""
        feat = self.features(x)
        return self.head(feat)

    def extract_features(self, x: torch.Tensor) -> torch.Tensor:
        """Extract feature vector (B, feat_dim) without regression head."""
        feat = self.features(x)
        return feat.flatten(1)

    def freeze_backbone(self):
        """Freeze backbone weights (useful for fine-tuning head only)."""
        for param in self.features.parameters():
            param.requires_grad = False

    def unfreeze_backbone(self):
        """Unfreeze all backbone weights."""
        for param in self.features.parameters():
            param.requires_grad = True

    def __repr__(self) -> str:
        n_params = sum(p.numel() for p in self.parameters())
        n_trainable = sum(p.numel() for p in self.parameters() if p.requires_grad)
        return (f'PoseResNet(backbone={self._backbone_name!r}, '
                f'joints={self._num_joints}, '
                f'params={n_params:,}, trainable={n_trainable:,})')


class MultiViewPoseResNet(nn.Module):
    """Multi-view fusion model for pose estimation.

    Each view is processed by a shared ResNet backbone.
    Features from all views are concatenated and passed through
    a fusion MLP to predict joint angles.

    Parameters
    ----------
    backbone : str
        ResNet variant (``'resnet18'``, ``'resnet34'``, ``'resnet50'``).
    num_views : int
        Number of camera viewpoints (default 4).
    pretrained : bool
        Use ImageNet-pretrained backbone.
    num_joints : int
        Number of output joint angles.
    dropout : float
        Dropout in fusion head.
    """

    BACKBONES = PoseResNet.BACKBONES

    def __init__(
        self,
        backbone: str = 'resnet18',
        num_views: int = 4,
        pretrained: bool = True,
        num_joints: int = 6,
        dropout: float = 0.3,
    ):
        super().__init__()
        if backbone not in self.BACKBONES:
            raise ValueError(f'Unknown backbone {backbone!r}')

        factory, weights, feat_dim = self.BACKBONES[backbone]
        base = factory(weights=weights if pretrained else None)

        # Shared feature extractor (no final FC)
        self.features = nn.Sequential(*list(base.children())[:-1])

        fused_dim = feat_dim * num_views

        # Fusion head: concat all views → MLP
        self.fusion_head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(fused_dim, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(512, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout * 0.5),
            nn.Linear(256, num_joints),
        )

        self._num_views = num_views
        self._num_joints = num_joints
        self._backbone_name = backbone
        self._feat_dim = feat_dim

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass.

        ``x`` shape: (B, N_views, 3, H, W) → (B, num_joints).
        """
        B, N, C, H, W = x.shape
        assert N == self._num_views, (
            f'Expected {self._num_views} views, got {N}')

        # Reshape to process all views in one batch
        x_flat = x.view(B * N, C, H, W)           # (B*N, 3, H, W)
        feats = self.features(x_flat)              # (B*N, feat_dim, 1, 1)
        feats = feats.view(B, N, self._feat_dim)   # (B, N, feat_dim)

        # Concatenate all views
        fused = feats.view(B, -1)                  # (B, N*feat_dim)
        return self.fusion_head(fused)

    def freeze_backbone(self):
        for param in self.features.parameters():
            param.requires_grad = False

    def unfreeze_backbone(self):
        for param in self.features.parameters():
            param.requires_grad = True

    def __repr__(self) -> str:
        n_params = sum(p.numel() for p in self.parameters())
        n_trainable = sum(p.numel() for p in self.parameters() if p.requires_grad)
        return (f'MultiViewPoseResNet(backbone={self._backbone_name!r}, '
                f'views={self._num_views}, joints={self._num_joints}, '
                f'params={n_params:,}, trainable={n_trainable:,})')
