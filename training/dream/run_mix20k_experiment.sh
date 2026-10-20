#!/usr/bin/env bash
# Experience : mix reel + 20K synthetique PROPRE (link6 90.8%) au lieu du 50K_clahe.
# Hypothese : le signal distal propre du 20K transfere au reel (link4/5/6 effondres
# a 11/2/3% sur real_test_1k avec le 50K_clahe). On change UNE variable : la source synth.
# Held-out identique (seed 42, test-poses 333) -> comparable au 47% du run 50K_clahe.
#
# Usage : source ~/ros_jazzy/venv_dream/bin/activate && cd training && bash dream/run_mix20k_experiment.sh
set -euo pipefail
cd "$(dirname "$0")/.."   # -> training/

# 1) Rebuild le mix avec le 20K propre
python dream/build_real_synth_mix.py \
  --new-real dream_data/real_arducam_full_ndds \
             dream_data/real_svpro_full_ndds \
             dream_data/real_astra_full_ndds \
  --old-real dream_data/real_3k \
  --synth dream_data/synthetic \
  --n-synth 10000 \
  --test-poses 333 \
  --train-out dream_data/mix_realsynth20k_train \
  --test-out  dream_data/real_test_1k_check \
  --seed 42

# 2) Train (poids natifs 20K : distal 1,1,1,1,1.5,1.5,6 ; aug = config 52.9%)
python dream/train_dream_mixed_aug_optimized_v10.py \
  --data dream_data/mix_realsynth20k_train \
  --epochs 50 --batch-size 8 --workers 8 \
  --output dream/vgg_realsynth20k_v10_e50 \
  --kp-weights 1.0,1.0,1.0,1.0,1.5,1.5,6.0 \
  --patience 5 \
  --aug-brightness 0.4 --aug-contrast 0.4 --aug-hue 5 --aug-sat 10 --aug-noise 30

# 3) Verdict sur le reel pur tenu a l'ecart (a comparer au 47%)
python dream/evaluate_dream_v2.py \
  --weights dream/vgg_realsynth20k_v10_e50/best_network.pth \
  --data dream_data/real_test_1k \
  --split all --max-samples 999
