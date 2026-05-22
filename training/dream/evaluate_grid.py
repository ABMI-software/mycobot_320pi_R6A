#!/usr/bin/env python3
import os
import glob
import subprocess
import csv

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

grid_dirs = glob.glob(os.path.join(SCRIPT_DIR, "checkpoints_dream", "vgg_grid_*"))

results = []
for d in sorted(grid_dirs):
    weights_path = os.path.join(d, "best_network.pth")
    if not os.path.exists(weights_path):
        continue
    print(f"\n▶ Évaluation : {os.path.basename(d)}")
    result = subprocess.run([
        "python3", "dream/evaluate_dream.py",
        "--weights", weights_path,
        "--data", "dream/dream_data/synthetic",
        "--split", "val"
    ], capture_output=True, text=True)
    print(result.stdout)
    results.append({
        "model": os.path.basename(d),
        "output": result.stdout
    })

# Sauvegarder dans un fichier texte
with open("grid_search_results.txt", "w") as f:
    for r in results:
        f.write(f"\n{'='*50}\n")
        f.write(f"{r['model']}\n")
        f.write(r['output'])

print("\n✅ Résultats sauvegardés → grid_search_results.txt")