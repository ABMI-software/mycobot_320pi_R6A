#!/usr/bin/env bash
# Met en pause (SIGSTOP) le groupe de processus d'un entraînement au-delà de
# 90 °C sur « Package id 0 », le reprend sous 80 °C, et consigne chaque bascule.
#   bash scripts/garde_thermique.sh <pid> <journal>
# L'entraînement doit avoir été lancé détaché (`setsid nohup … &`) : lancé en
# tâche de fond par Claude Code, une pause le tue (docs/ARTICLE_DREAM.md,
# « Ce qu'on a appris »).
set -u
pid=$1; journal=$2; pgid=$(ps -o pgid= -p "$pid" | tr -d ' '); stoppe=0
while kill -0 "$pid" 2>/dev/null; do
  t=$(sensors | awk '/Package id 0/ {gsub(/[+°C]/,"",$4); print int($4)}')
  if [ "$stoppe" = 0 ] && [ "$t" -ge 90 ]; then kill -STOP -"$pgid"; stoppe=1; echo "$(date +%T) pause à ${t} °C" >> "$journal"; fi
  if [ "$stoppe" = 1 ] && [ "$t" -lt 80 ]; then kill -CONT -"$pgid"; stoppe=0; echo "$(date +%T) reprise à ${t} °C" >> "$journal"; fi
  sleep 10
done
