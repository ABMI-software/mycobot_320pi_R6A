#!/usr/bin/env bash
# Collecte caméra libre par tranches (E3.7 de docs/ARTICLE_DREAM.md).
#   bash scripts/collecte_tranches.sh <dossier> <nb_tranches> [images_par_tranche]
# Tranches de 1 000 : à 5 000, la cadence passe de 3,7 à 23 s/image et les
# masques à jour de 95 à 25 % (E3.7, 11/11) ; à 1 000, 3,9 s/image et 96 %.
# Chaque tranche relance Gazebo : sur le pilote du 03/11, la cadence dérivait
# de 7,9 à 10,1 s/image en 1 000 images, et une relance la remet à zéro. Une
# tranche finie porte un fichier `fait` et n'est pas refaite : relancer le
# script reprend où il s'était arrêté. À lancer détaché :
#   setsid nohup bash scripts/collecte_tranches.sh ~/dream_article/E3/E3_7 10 > /dev/null 2>&1 &
dossier=$1; nb=$2; par_tranche=${3:-1000}
journal="$dossier/journal.txt"
mkdir -p "$dossier"
# Avant `set -u` : les scripts d'environnement de ROS lisent des variables non définies.
source /opt/ros/jazzy/setup.bash
source "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/../../install/setup.bash"
set -u

arreter_gazebo() {
  kill -INT -"$1" 2>/dev/null
  sleep 15
  # Le launch arrêté laisse Gazebo orphelin : on le cherche par son nom.
  for p in $(pgrep -f '^(gz sim|ruby .*gz sim)'); do kill -INT "$p" 2>/dev/null; done
  sleep 10
  for p in $(pgrep -f '^(gz sim|ruby .*gz sim)'); do kill -KILL "$p" 2>/dev/null; done
}

for i in $(seq -w 1 "$nb"); do
  t="$dossier/tranche_$i"
  [ -f "$t/fait" ] && continue
  mkdir -p "$t"
  echo "$(date '+%F %T') tranche $i : début" >> "$journal"
  # Variables GTK du snap ôtées (l'interface plante sinon) ; GPU NVIDIA forcé
  # (sinon rendu logiciel possible sur cette machine hybride).
  setsid env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR \
    -u GTK_IM_MODULE_FILE \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    __GLX_VENDOR_LIBRARY_NAME=nvidia __NV_PRIME_RENDER_OFFLOAD=1 \
    taskset -c 16-19 ros2 launch mycobot_gateway synthetic_data_camera_libre.launch.py \
      num_samples:="$par_tranche" max_settle_skips:=1 output_dir:="$t/collecte" \
      > "$t/launch.log" 2>&1 < /dev/null &
  groupe=$!
  limite=$((SECONDS + par_tranche * 12))
  until grep -qE "Collection complete|terminate called|was required|AUCUN|Traceback" "$t/launch.log" \
        || [ $SECONDS -ge $limite ]; do
    sleep 30
  done
  n=$(( $(wc -l < "$t/collecte/labels.csv" 2>/dev/null || echo 1) - 1 ))
  if grep -q "Collection complete" "$t/launch.log"; then
    touch "$t/fait"
    echo "$(date '+%F %T') tranche $i : finie, $n images" >> "$journal"
  else
    echo "$(date '+%F %T') tranche $i : INTERROMPUE à $n images ($(grep -m1 -oE 'terminate called|was required|AUCUN|Traceback' "$t/launch.log" || echo 'délai dépassé'))" >> "$journal"
  fi
  arreter_gazebo "$groupe"
done
echo "$(date '+%F %T') toutes les tranches traitées" >> "$journal"
