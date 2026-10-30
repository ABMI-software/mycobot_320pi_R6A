#!/usr/bin/env python3
"""Construit un train mix reel+synthetique + un test reel tenu a l'ecart.

- Test : les dernieres poses du reel 3-cam (aucune fuite dans le train).
- Train : reste du reel 3-cam + reel ancien + synthetique, shuffle, renumerote.
Symlinks (pas de copie). camera_settings copie depuis la 1re source.
"""
import os, glob, random, shutil, argparse


def frames(d):
    return sorted(os.path.basename(f)[:-5] for f in glob.glob(os.path.join(d, "??????.json")))


def link(src_dir, fid, out_dir, idx):
    new = f"{idx:06d}"
    ok = True
    for ext in (".json", ".rgb.png"):
        s = os.path.realpath(os.path.join(src_dir, fid + ext))
        t = os.path.join(out_dir, new + ext)
        if os.path.lexists(t):
            os.remove(t)
        if os.path.exists(s):
            os.symlink(s, t)
        else:
            ok = False
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--new-real", nargs="+", required=True,
                    help="dossiers NDDS reel 3-cam (frame N = pose N)")
    ap.add_argument("--old-real", nargs="*", default=[],
                    help="dossiers NDDS reel ancien (tout pris pour le train)")
    ap.add_argument("--synth", required=True, help="dossier NDDS synthetique")
    ap.add_argument("--n-synth", type=int, default=10000)
    ap.add_argument("--test-poses", type=int, default=333,
                    help="nb de dernieres poses tenues a l'ecart (x3 cams ~ test)")
    ap.add_argument("--train-out", required=True)
    ap.add_argument("--test-out", required=True)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    os.makedirs(args.train_out, exist_ok=True)
    os.makedirs(args.test_out, exist_ok=True)
    for f in ["_camera_settings.json", "_object_settings.json"]:
        s = os.path.join(args.new_real[0], f)
        if os.path.exists(s):
            shutil.copy2(s, os.path.join(args.train_out, f))
            shutil.copy2(s, os.path.join(args.test_out, f))

    # --- TEST : dernieres poses du reel 3-cam ---
    test_pool, train_real = [], []
    for d in args.new_real:
        fl = frames(d)
        cut = max(0, len(fl) - args.test_poses)
        for fid in fl[cut:]:
            test_pool.append((d, fid))
        for fid in fl[:cut]:
            train_real.append((d, fid))
    # --- reel ancien : tout au train ---
    for d in args.old_real:
        for fid in frames(d):
            train_real.append((d, fid))
    # --- synthetique : echantillon ---
    sfl = frames(args.synth)
    if len(sfl) > args.n_synth:
        step = len(sfl) / args.n_synth
        sfl = [sfl[int(i * step)] for i in range(args.n_synth)]
    train_synth = [(args.synth, fid) for fid in sfl]

    train = train_real + train_synth
    random.seed(args.seed)
    random.shuffle(train)

    n = 0
    for i, (d, fid) in enumerate(test_pool):
        n += link(d, fid, args.test_out, i)
    print(f"TEST : {n}/{len(test_pool)} frames -> {args.test_out}")

    n = 0
    for i, (d, fid) in enumerate(train):
        n += link(d, fid, args.train_out, i)
        if i % 5000 == 0:
            print(f"  train ... {i}/{len(train)}")
    print(f"TRAIN: {n}/{len(train)} frames -> {args.train_out}")
    print(f"  (reel 3-cam train={len(train_real)-sum(len(frames(d)) for d in args.old_real)}, "
          f"reel ancien={sum(len(frames(d)) for d in args.old_real)}, synth={len(train_synth)})")


if __name__ == "__main__":
    main()
