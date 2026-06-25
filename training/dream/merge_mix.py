#!/usr/bin/env python3
"""Merge several NDDS datasets into one, with per-source oversampling.

Unlike merge_ndds.py this:
  - uses SYMLINKS (no copy — 90K frames would be GBs)
  - SHUFFLES the merged pool with a fixed seed, so the downstream 80/10/10
    index split puts both domains in train AND val (no domain leakage)
  - lets you OVERSAMPLE a source with dir:repeat, to keep a clean teacher
    (e.g. 20K) from being drowned by a larger/harder set (e.g. 50K)

Example — 20K clean (x2) + 50K CLAHE (x1):
  python dream/merge_mix.py -o dream_data/mix_20k_50k \\
      -s dream_data/synthetic:2 dream_data/synth_50k_clahe:1
"""
import os, sys, glob, random, shutil, argparse


def frames(d):
    return sorted(os.path.basename(f)[:-5]
                  for f in glob.glob(os.path.join(d, "??????.json")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", "-o", required=True)
    ap.add_argument("--src", "-s", nargs="+", required=True,
                    help="dossiers NDDS, suffixe :N pour répéter N fois")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    pool = []
    for spec in args.src:
        d, _, r = spec.partition(":")
        r = int(r) if r else 1
        fl = frames(d)
        if not fl:
            print(f"  ⚠️  aucun frame dans {d}"); continue
        pool += [(d, fid) for _ in range(r) for fid in fl]
        print(f"  {d}  x{r}  = {len(fl)*r} frames")

    random.seed(args.seed)
    random.shuffle(pool)

    first = args.src[0].partition(":")[0]
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(first, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(args.out, f))

    n_ok = 0
    for i, (d, fid) in enumerate(pool):
        new = f"{i:06d}"
        ok = True
        for ext in (".json", ".rgb.png"):
            s = os.path.realpath(os.path.join(d, fid + ext))
            t = os.path.join(args.out, new + ext)
            if os.path.lexists(t):
                os.remove(t)
            if os.path.exists(s):
                os.symlink(s, t)
            else:
                ok = False
        n_ok += ok
        if i % 5000 == 0:
            print(f"  ... {i}/{len(pool)}")

    print(f"✅ {n_ok}/{len(pool)} frames mergés (symlinks) → {args.out}")


if __name__ == "__main__":
    main()
