#!/usr/bin/env bash
# Enregistre les 3 sources DANS UNE SEULE vidéo mosaïque, AFFICHÉE EN DIRECT.
#   +-----------------+-----------------+
#   |     Arducam     |      SVPRO       |   (haut : les 2 caméras)
#   +-----------------------------------+
#   |        écran / GUI (capture)       |   (bas : l'écran)
#   +-----------------------------------+
#
# À LANCER SUR LE PC TOUR, terminal à part :
#   bash scripts/record_demo.sh
#   bash scripts/record_demo.sh /dev/video2 /dev/video0   # arducam svpro
#
# Une fenêtre "LIVE" s'ouvre : tu vois les 3 en même temps ET c'est enregistré.
# Fais ton RUN dans le GUI, puis Ctrl+C ici pour finaliser la vidéo.
#
# Sortie : ~/Videos/pick_place_mosaic_<ts>.mp4

set -u
ARDUCAM="${1:-/dev/video2}"
SVPRO="${2:-/dev/video0}"
OUTDIR="${HOME}/Videos"; mkdir -p "$OUTDIR"
# .mkv : reste lisible même si l'enregistrement est coupé brutalement
# (fermeture de la fenêtre LIVE) — le .mp4, lui, se corrompt s'il n'est pas finalisé.
OUT="${OUTDIR}/pick_place_mosaic_$(date +%Y%m%d_%H%M%S).mkv"

RES="$(xrandr 2>/dev/null | grep -oP '(?<=current )\d+ x \d+' | tr -d ' ' | head -1)"
RES="${RES:-1920x1080}"

echo "🔴 Enregistrement mosaïque -> $OUT"
echo "   Fenêtre LIVE = les 2 CAMÉRAS (l'écran, tu le vois déjà en vrai)."
echo "   Le FICHIER, lui, contient les 3 vues (caméras + écran)."
echo "   → fais ton RUN dans le GUI, puis Ctrl+C ici pour arrêter."

# 0 = écran (x11grab) | 1 = arducam | 2 = svpro
# FICHIER  : mosaïque complète (caméras en haut, écran en bas).
# LIVE SDL : seulement les 2 caméras (pas d'écran -> pas de miroir récursif).
# ffmpeg : écrit le FICHIER mosaïque complet + PIPE le flux caméras vers ffplay
# (affichage via ffplay = bien plus fiable que la sortie sdl2 de ffmpeg).
ffmpeg -hide_banner -loglevel error -y \
  -f x11grab -framerate 25 -video_size "$RES" -i "${DISPLAY:-:0.0}" \
  -f v4l2 -framerate 30 -video_size 640x480 -i "$ARDUCAM" \
  -f v4l2 -framerate 30 -video_size 640x480 -i "$SVPRO" \
  -filter_complex "\
    [1:v]scale=640:360,setsar=1,drawtext=text='Arducam':x=8:y=8:fontcolor=white:fontsize=20:box=1:boxcolor=black@0.5[ard]; \
    [2:v]scale=640:360,setsar=1,drawtext=text='SVPRO':x=8:y=8:fontcolor=white:fontsize=20:box=1:boxcolor=black@0.5[sv]; \
    [ard][sv]hstack=inputs=2[cams]; \
    [cams]split=2[cams_file][cams_live]; \
    [0:v]scale=1280:540,setsar=1[scr]; \
    [cams_file][scr]vstack=inputs=2[m]; \
    [cams_live]format=yuv420p[live]" \
  -map "[m]" -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$OUT" \
  -map "[live]" -c:v rawvideo -f nut pipe:1 \
  | ffplay -hide_banner -loglevel error -window_title "LIVE cameras (Ctrl+C dans le terminal)" -i -

echo -e "\n✅ vidéo : $OUT"
