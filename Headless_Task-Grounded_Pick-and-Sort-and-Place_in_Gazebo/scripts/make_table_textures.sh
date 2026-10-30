#!/usr/bin/env bash
# Table-texture variants for per-episode scene variation, derived from the
# team's own wood texture with ffmpeg colour filters (no new assets in git).
# Run once per container, from the project root:
#   bash scripts/make_table_textures.sh [path/to/wood_albedo.png]
set -eu
SRC=${1:-/workspace/install/mycobot_description/share/mycobot_description/models/wood_table/materials/textures/wood_albedo.png}
OUT=textures
mkdir -p "$OUT"
make() { ffmpeg -loglevel error -y -i "$SRC" -vf "scale=1024:-1,$2" "$OUT/$1.png"; }
make wood_original   "null"
make wood_dark       "eq=brightness=-0.12:saturation=1.1"
make wood_light      "eq=brightness=0.10:saturation=0.8"
make wood_walnut     "hue=h=-12:s=1.2,eq=brightness=-0.18"
make wood_ash        "hue=s=0.35,eq=brightness=0.12"
make laminate_grey   "hue=s=0,eq=brightness=0.05:contrast=0.6"
make laminate_warm   "hue=s=0.25,eq=brightness=-0.02:contrast=0.5,colorbalance=rm=0.08:gm=0.03"
make wood_reddish    "hue=h=-20:s=1.3"
ls "$OUT"
