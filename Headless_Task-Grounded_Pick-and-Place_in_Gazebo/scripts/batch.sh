#!/bin/bash
# Usage: batch.sh [first_idx last_idx]   -- records episodes, up to 3 attempts each
FIRST=${1:-1}; LAST=${2:-20}
H=/workspace/htgpp; S=$H/batch_status.txt
declare -A X Y C
for i in 1 2 3 4 5;        do X[$i]=0.250; Y[$i]=0.000; C[$i]=front; done
XS=(0.250 0.265 0.280 0.295 0.310); for k in 0 1 2 3 4; do i=$((6+k));  X[$i]=${XS[$k]}; Y[$i]=0.000; C[$i]=front; done
YS=(-0.080 -0.040 0.000 0.040 0.080); for k in 0 1 2 3 4; do i=$((11+k)); X[$i]=0.250; Y[$i]=${YS[$k]}; C[$i]=front; done
for i in 16 17 18 19 20;   do X[$i]=0.250; Y[$i]=0.000; C[$i]=heldout; done
for i in $(seq $FIRST $LAST); do
  for attempt in 1 2 3; do
    echo "$(date +%T) episode $i attempt $attempt start (${X[$i]},${Y[$i]} ${C[$i]})" >> $S
    bash $H/episode.sh $i ${X[$i]} ${Y[$i]} ${C[$i]} > $H/episode_$i.attempt$attempt.txt 2>&1
    RC=$?
    echo "$(date +%T) episode $i attempt $attempt rc=$RC" >> $S
    [ $RC = 0 ] && break
  done
done
echo "BATCH DONE" >> $S
