#!/usr/bin/env python3
"""DREAM (keypoints du bras) pour le dashboard du tri reel, sous `venv_dream`.

Meme protocole que `yolo26_service.py`, meme raison d'etre : le dashboard tourne
en Python systeme sans torch, le modele tourne ici, les deux se parlent par un
tube.

  entree : une ligne JSON {"h": ..., "w": ..., "camera": "arducam"|"svpro"}
           puis h*w*3 octets BGR bruts
  sortie : une ligne JSON {"kp": [[u, v] | null] x 7, "ms": inference}
Une entree vide termine le service.

Un keypoint non detecte sort `null` : DREAM le signale par un pic hors cadre,
jamais par NaN, d'ou le test de cadre et pas seulement de finitude.
"""
import json
import sys
import time
from pathlib import Path

# Le protocole passe par stdout : tout ce que les bibliotheques ecrivent part sur stderr.
SORTIE = sys.stdout
sys.stdout = sys.stderr

import numpy as np  # noqa: E402

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path.home() / 'DREAM'))
MODELE_DEFAUT = 'vgg_tri_reel_ft_e10'


def main():
    nom = sys.argv[1] if len(sys.argv) > 1 else MODELE_DEFAUT
    poids = RACINE / 'training' / 'dream' / 'checkpoints_dream' / nom / 'best_network.pth'
    if not poids.exists():
        print(json.dumps({'erreur': f'poids introuvables : {poids}'}), file=SORTIE, flush=True)
        return
    import dream
    from PIL import Image
    net = dream.create_network_from_config_file(str(poids.with_suffix('.yaml')), str(poids))
    net.keypoints_from_image(Image.fromarray(np.zeros((480, 640, 3), np.uint8)))
    print(json.dumps({'pret': True, 'modele': nom}), file=SORTIE, flush=True)

    entree = sys.stdin.buffer
    while True:
        ligne = entree.readline()
        if not ligne:
            return
        forme = json.loads(ligne)
        h, w = forme['h'], forme['w']
        octets = entree.read(h * w * 3)
        if len(octets) < h * w * 3:
            return
        image = np.frombuffer(octets, np.uint8).reshape(h, w, 3)
        t0 = time.monotonic()
        kp = np.array(net.keypoints_from_image(Image.fromarray(image[:, :, ::-1]))['detected_keypoints'], float)
        ms = (time.monotonic() - t0) * 1000.0
        print(json.dumps({'kp': [[float(u), float(v)] if np.all(np.isfinite([u, v])) and 0 <= u < w
                                 and 0 <= v < h else None for u, v in kp], 'ms': ms}),
              file=SORTIE, flush=True)


if __name__ == '__main__':
    main()
