#!/usr/bin/env bash
# Active les hooks versionnés du dépôt (.githooks/) pour cette copie de travail.
# À lancer une fois par clone : bash scripts/installer_hooks.sh
set -euo pipefail
racine="$(git rev-parse --show-toplevel)"
chmod +x "$racine"/.githooks/*
git -C "$racine" config core.hooksPath .githooks
echo "Hooks actifs : $(ls "$racine/.githooks" | tr '\n' ' ')"
echo "Contournement exceptionnel, à justifier dans le message : git commit --no-verify"
