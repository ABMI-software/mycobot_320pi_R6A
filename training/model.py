#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ResNet-based regression model for MyCobot pose estimation.

Predicts 6 joint angles (radians, optionally normalised to [-1, 1])
from a single RGB image.
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

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass.  ``x`` shape: (B, 3, H, W) → (B, num_joints)."""
        feat = self.features(x)
        return self.head(feat)

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
