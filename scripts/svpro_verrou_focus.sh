#!/usr/bin/env bash
# Verrouille la mise au point de la SVPRO.
#
# Pourquoi ce script existe : rien dans le depot ne fixe le focus de la SVPRO.
# `pick_dashboard.py` ne mentionne jamais le mot, et `camera_registry.py` la
# declare `manual_focus=-1` sans que personne ne lise ce champ. Or le defaut de
# la camera est `focus_automatic_continuous=1` : a chaque ouverture du flux
# l'autofocus se rallume et pompe entre les poses.
#
# focus_absolute=40, et non 90 : mesure du 01/09 apres l'inclinaison de la
# camera (faite pour faire entrer le marqueur 25 dans le champ). La planche
# s'est rapprochee, le point de nettete a bouge. Balayage 0-250 :
#   focus  40 -> nettete 418, 4/4 marqueurs, 48 detections sur 48
#   focus  90 -> nettete 211, 2/4 marqueurs, 14 detections     <- l'ancien
#   focus 140 -> nettete  51, 0/4
# A rebalayer si la camera est rebougee (voir svpro_regler_4_marqueurs.py).
#
# La camera est trouvee par son NOM V4L2 : son /dev/videoN change selon le port
# USB (mesure : -4 -> video0, -5 -> video3).
#
# Usage :  bash scripts/svpro_verrou_focus.sh [focus]
set -euo pipefail

FOCUS="${1:-40}"
DEV=$(v4l2-ctl --list-devices 2>/dev/null \
      | awk '/5MP/{f=1; next} f && /\/dev\/video/{print $1; exit}')

if [ -z "${DEV:-}" ]; then
    echo "SVPRO introuvable — est-elle branchee ?" >&2
    exit 1
fi

v4l2-ctl -d "$DEV" --set-ctrl focus_automatic_continuous=0
v4l2-ctl -d "$DEV" --set-ctrl "focus_absolute=${FOCUS},sharpness=0,contrast=1"

echo "SVPRO sur $DEV :"
v4l2-ctl -d "$DEV" --get-ctrl focus_automatic_continuous,focus_absolute,sharpness,contrast
echo
echo "A rejouer apres chaque rebranchement ou redemarrage, et APRES avoir lance"
echo "ce qui ouvre la camera : un controle pose avant le flux peut etre efface."
