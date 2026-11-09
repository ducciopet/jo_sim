#!/bin/bash
# At the end of a lab PC reservation (bags/lab_blackout), before our chains resume: measure the load from OTHER users
# (ours is paused by lab_gate): CPU utilisation over 2 min from /proc/stat, 1-min loadavg, GPU compute processes.
# Busy (CPU > 30%, or loadavg > cores/2, or any GPU compute process) -> postpone the end of the blackout by 15 min
# and check again. Every check is logged in bags/lab_watchdog.txt. Never touches other processes.
cd $(dirname $0)/../../..; F=bags/lab_blackout; W=bags/lab_watchdog.txt
read s e < $F; while [ $(date +%s) -lt $((e - 150)) ]; do sleep 30; done
half=$(( $(nproc) / 2 ))
while :; do
  a=($(head -1 /proc/stat)); sleep 120; b=($(head -1 /proc/stat))
  idle=$(( (b[4]+b[5]) - (a[4]+a[5]) )); tot=0; for i in 1 2 3 4 5 6 7 8; do tot=$(( tot + b[i] - a[i] )); done
  cpu=$(( 100 * (tot - idle) / tot )); la=$(cut -d' ' -f1 /proc/loadavg)
  gpu=$(nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader 2>/dev/null | tr '\n' ';')
  busy=0; [ $cpu -gt 30 ] && busy=1; awk -v l=$la -v h=$half 'BEGIN{exit !(l>h)}' && busy=1; [ -n "$gpu" ] && busy=1
  echo "$(date '+%F %T') check: cpu ${cpu}% (2 min), loadavg ${la} (half cores ${half}), gpu [${gpu:-free}] -> $([ $busy = 1 ] && echo BUSY || echo FREE)" >> $W
  [ $busy = 0 ] && { echo "$(date '+%F %T') resume" >> $W; break; }
  read s e < $F; ne=$(( $(date +%s) + 900 )); echo "$s $ne" > $F
  echo "$(date '+%F %T') blackout postponed to $(date -d @$ne +%T)" >> $W
  while [ $(date +%s) -lt $((ne - 150)) ]; do sleep 30; done
done
