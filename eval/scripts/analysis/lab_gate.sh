# Lab PC blackout gate (sourced by the lab chains; also inlined in eval/scripts/run_street.sh for the container).
# File bags/lab_blackout (not committed): one line "<epoch the PC must be free> <epoch we may restart>".
# lab_gate <expected job seconds>: before starting a job, wait while it would still be running at the first epoch
# and we are before the second one; so a job starts in the hour before only if it ends in time.
lab_gate(){ local f=${LAB_BLACKOUT:-$(dirname ${BASH_SOURCE[0]})/../../../bags/lab_blackout} s e n w=0
  [ -f "$f" ] || return 0; read s e < "$f"
  while n=$(date +%s); [ $((n + $1)) -gt $s ] && [ $n -lt $e ]; do
    [ $w = 0 ] && echo "$(date +%T) lab_gate: PC reserved, waiting until $(date -d @$e +%T)"; w=1; sleep 60; done; }
