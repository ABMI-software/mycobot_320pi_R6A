# Entraînement VGG ultimate v4 — dataset synthétique 50k (v3)

Documente le run d'entraînement DREAM lancé sur le nouveau dataset synthétique
50k (intrinsèques corrigées, filtre capsule). Génération du dataset détaillée
dans [`../SYNTHETIC_50K_V3.md`](../SYNTHETIC_50K_V3.md).

Date : 2026-07-02 · Robot : MyCobot 320 Pi (variante sans grippeur)

---

## Contexte

- Objectif : dépasser le run v2 (**97.7 %** en synthétique) sur le nouveau 50k,
  qui a des intrinsèques caméra corrigées (`fov=1.15`, `fx≈494`, top-cam 0.95 m)
  — géométrie différente de v2.
- Décision : entraînement **from scratch** (pas de `--pretrained`) plutôt que
  warm-start depuis v2, pour ne pas ancrer le modèle sur l'optimum de l'ancienne
  géométrie de caméra.
- Recette identique à v2 : perte pondérée par keypoint `[1,1,1,1,1.5,1.5,6.0]`
  (link6 = 6.0, keypoint distal le plus dur), augmentation forte sim-to-real,
  cosine annealing LR (1e-4 → 1e-6), early stopping.

## Commande lancée

```bash
conda deactivate
source ~/ros_jazzy/venv_dream/bin/activate
cd /home/genji/Osama_ws/src/mycobot_R6A/training/dream

python train_dream_ultimate_v4.py \
    --data /home/genji/Osama_ws/src/mycobot_R6A/training/dream/dream_data/synthetic_50k_ndds \
    --output /home/genji/Osama_ws/src/mycobot_R6A/training/dream/output/checkpoints_dream/vgg_ultimate_v4_e50 \
    --epochs 50 --batch-size 8 --workers 8 --patience 5
```

| Paramètre | Valeur | Note |
|---|---|---|
| `--data` | `dream_data/synthetic_50k_ndds` | 50 000 images NDDS, split 40 000/10 000 (80/20) |
| `--output` | `output/checkpoints_dream/vgg_ultimate_v4_e50` | |
| `--epochs` | 50 | early stopping patience 5 → coupera avant si plateau |
| `--batch-size` | 8 | |
| `--workers` | 8 | 28 cœurs logiques dispo, load ~2.9 → large marge |
| `--pretrained` | *(absent)* | from scratch, voir Contexte |
| AMP | non | script en FP32 pur, pas d'`autocast`/`GradScaler` implémenté |
| Poids keypoints | `[1,1,1,1,1.5,1.5,6.0]` (défaut) | |

## Smoke test préalable

Passe de contrôle 1 epoch effectuée avant le lancement réel pour valider le
pipeline (chargement dataset 50 000 → split 40 000/10 000, création réseau,
perte pondérée, résolutions 400×400 → 100×100, ~4.9 it/s). Résultat concluant,
run réel lancé ensuite. Smoke dir nettoyé après vérification GPU libre.

## Statut

- **Lancé** : 2026-07-02 11:09 (PID 1293122 + 8 workers)
- **Terminé** : val_loss 0.000750 — checkpoint `output/checkpoints_dream/vgg_ultimate_v4_e50/best_network.pth`
- Objectif 98 %+ dépassé (voir Résultat final ci-dessous), devient le nouveau
  record synthétique et le `--pretrained` de départ du fine-tune mixte
  (voir [`FINETUNE_MIX_REAL3CAM_PLAN.md`](FINETUNE_MIX_REAL3CAM_PLAN.md)).

## Résultat final — évaluation `evaluate_dream.py`

Commande (2026-07-06, `venv_dream`) :

```bash
python evaluate_dream.py \
  --weights output/checkpoints_dream/vgg_ultimate_v4_e50/best_network.pth \
  --data dream_data/synthetic_50k_ndds --split val --max-samples 2000
```

2000 frames du split val (40000–50000), résultats :

| Keypoint | Mean (px) | Median (px) | Std | Max | Det% |
|----------|-----------|-------------|-----|-----|------|
| base | 3.47 | 3.38 | 0.17 | 3.89 | 100.0% |
| link1 | 3.20 | 3.17 | 0.21 | 3.64 | 100.0% |
| link2 | 3.20 | 3.18 | 0.21 | 3.65 | 100.0% |
| link3 | 1.88 | 1.61 | 2.40 | 51.88 | 99.8% |
| link4 | 2.11 | 1.69 | 4.55 | 97.69 | 100.0% |
| link5 | 2.11 | 1.59 | 5.15 | 127.65 | 99.2% |
| link6 | 2.30 | 1.77 | 5.35 | 112.54 | 97.0% |
| **OVERALL** | **2.61** | **2.78** | **3.46** | 127.65 | **99.4%** (13920/14000) |

Précision : 37.1% <2px, 98.8% <5px, 99.5% <10px, 99.7% <20px, 99.9% <50px.
Erreur moyenne par frame : 2.62 ± 2.33 px.

**99.4 % de détection, dépasse le record v2 (97.7 %)** — voir aussi le
tableau comparatif dans [`SYNTHETIC_TRAINING_REPORT.md`](SYNTHETIC_TRAINING_REPORT.md#5-tableau-des-expériences--détection-val-synthétique).

## Prochaines actions

1. [FAIT] ~~Laisser tourner, surveiller `nvidia-smi`~~
2. [FAIT] ~~Comparer `best_network.pth` au v2 (97.7 %)~~ → **99.4 %**, objectif dépassé
3. [ROUGE] Transfert sim→réel toujours bloqué à ≈27 % (`real_3cam_ndds`) malgré
   le gain synthétique — voir [`FINETUNE_MIX_REAL3CAM_PLAN.md`](FINETUNE_MIX_REAL3CAM_PLAN.md).
   Fine-tune mixte synthétique+réel (`train_dream_ultimate_v4_mix.py`, 30 epochs,
   `scale_limit=0.3`) en cours depuis ce checkpoint pour combler l'écart.
