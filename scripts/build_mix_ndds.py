#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Assemble un train NDDS mixte et un test tenu a l'ecart, par SYMLINKS.

Ecrit a cote de `training/dream/build_real_synth_mix.py` plutot que de le
modifier, pour deux raisons : le suroversamplage y etait fait en amont par un
autre outil (`merge_and_convert.py`), et le jeu du montage 0901 demande de
suroversampler la PARTIE TRAIN seulement — dupliquer le bloc de test le
rendrait sans valeur.

Le decoupage du test est un BLOC CONTIGU pris en fin de trajectoire, jamais un
tirage aleatoire : deux images consecutives de cette capture sont a 2,46 deg
l'une de l'autre, donc quasi jumelles. Un tirage au hasard poserait des
quasi-doublons des deux cotes et rendrait le score du test flatteur et faux.
Les dossiers d'une meme capture partagent leur numerotation (trame N = pose N)
: couper au meme rang dans chacun sort la meme pose de toutes les vues, sans
quoi une camera verrait au train la pose que l'autre garde au test.

Les intrinseques ne sont pas melangees ici : `_camera_settings.json` est copie
de la premiere source. C'est sans effet sur l'entrainement, qui n'apprend que
des cartes de croyance 2D depuis `projected_location` — les intrinseques ne
servent qu'au PnP, a l'inference. (fx : arducam 496,3 / svpro 447,8 /
synthetique 493,8.)

Usage :
    python3 scripts/build_mix_ndds.py \
        --reel-neuf training/dream/captures/real_montage_0901_{arducam,svpro}_ndds \
        --test-poses 150 --oversample 5 \
        --reel-ancien training/dream/dream_data/real_3cam_{arducam,svpro}_ndds \
        --synth training/dream/dream_data/synthetic_50k_ndds --n-synth 30000 \
        --train-out training/dream/dream_data/mix_montage0901_train \
        --test-out  training/dream/dream_data/mix_montage0901_test
"""
import argparse
import random
import shutil
from pathlib import Path


def trames(d):
    return sorted(f.stem for f in Path(d).glob('??????.json'))


def purge(d):
    """Efface les liens d'un assemblage precedent, jamais un fichier reel.

    Sans ca, un second tirage plus court laisse en place la queue du premier :
    mesure faite, 2500 liens perimes survivaient, dont 129 pointaient sur des
    trames passees du train au test — une fuite creee par la reconstruction
    elle-meme, invisible dans le compte affiche.
    """
    n = 0
    for f in list(d.glob('??????.json')) + list(d.glob('??????.rgb.png')):
        if f.is_symlink():
            f.unlink()
            n += 1
        else:
            raise SystemExit(f'{f} n est pas un lien — dossier refuse')
    return n


def lie(src, fid, dest, i):
    complet = True
    for ext in ('.json', '.rgb.png'):
        s = (Path(src) / (fid + ext)).resolve()
        t = Path(dest) / f'{i:06d}{ext}'
        if t.is_symlink() or t.exists():
            t.unlink()
        if s.exists():
            t.symlink_to(s)
        else:
            complet = False
    return complet


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reel-neuf', nargs='+', required=True,
                    help='dossiers NDDS de la capture neuve (trame N = pose N)')
    ap.add_argument('--test-poses', type=int, default=150,
                    help='dernieres poses de chaque dossier neuf, mises de cote')
    ap.add_argument('--oversample', type=int, default=5,
                    help='repetitions de la part TRAIN du reel neuf')
    ap.add_argument('--reel-ancien', nargs='*', default=[])
    ap.add_argument('--synth', required=True)
    ap.add_argument('--n-synth', type=int, default=30000)
    ap.add_argument('--train-out', required=True)
    ap.add_argument('--test-out', required=True)
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    train_out, test_out = Path(args.train_out), Path(args.test_out)
    for d in (train_out, test_out):
        d.mkdir(parents=True, exist_ok=True)
        vides = purge(d)
        if vides:
            print(f'purge {vides} liens perimes dans {d}')
        for f in ('_camera_settings.json', '_object_settings.json'):
            s = Path(args.reel_neuf[0]) / f
            if s.exists():
                shutil.copy2(s, d / f)

    test, neuf_train = [], []
    for d in args.reel_neuf:
        fl = trames(d)
        coupe = max(0, len(fl) - args.test_poses)
        test += [(d, f) for f in fl[coupe:]]
        neuf_train += [(d, f) for f in fl[:coupe]]

    ancien = [(d, f) for d in args.reel_ancien for f in trames(d)]

    sfl = trames(args.synth)
    if len(sfl) > args.n_synth:
        pas = len(sfl) / args.n_synth
        sfl = [sfl[int(i * pas)] for i in range(args.n_synth)]
    synth = [(args.synth, f) for f in sfl]

    train = neuf_train * args.oversample + ancien + synth
    random.seed(args.seed)
    random.shuffle(train)

    n = sum(lie(d, f, test_out, i) for i, (d, f) in enumerate(test))
    print(f'TEST  : {n}/{len(test)} trames -> {test_out}')

    n = 0
    for i, (d, f) in enumerate(train):
        n += lie(d, f, train_out, i)
        if i and i % 10000 == 0:
            print(f'  train ... {i}/{len(train)}')
    print(f'TRAIN : {n}/{len(train)} trames -> {train_out}')
    print(f'  neuf {len(neuf_train)} x{args.oversample} = '
          f'{len(neuf_train) * args.oversample}   ancien {len(ancien)}   '
          f'synth {len(synth)}')


if __name__ == '__main__':
    main()
