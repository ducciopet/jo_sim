#!/bin/bash
# CROSSLAB 2026-10-29 (Qualisys benchmark + fairy zips) from the shared OneDrive to the kloster.
# Zips one at a time, smallest first, each only when the share has room for it plus RESERVE GB (the KITTI colour
# images still downloading); otherwise wait. Each zip is checked against its .sha256 after copy.
SRC="onedrive_sanrossore,root_folder_id='':JO MISSIONS/FAIRY MISSIONS/CROSSLAB/20261029_crosslab"
DST=/mnt/kloster-reichenau/datasets_SINS/CROSSLAB/20261029_crosslab
LOG=$HOME/rclone-crosslab.log
RESERVE=${RESERVE:-25}
OPTS=(--transfers 4 --checkers 4 --retries 10 --low-level-retries 20 --stats 5m --stats-one-line --log-file "$LOG" --log-level INFO)
echo "=== $(date '+%F %T') START motion capture + checksums" >> $LOG
rclone copy "$SRC" "$DST" --exclude '*.zip' "${OPTS[@]}"; echo "=== $(date '+%F %T') END small exit=$?" >> $LOG
rclone lsf "$SRC/fairy" --include '*.zip' --format sp --separator ' ' | sort -n | while read size name; do
  [ -f "$DST/fairy/$name" ] && [ $(stat -c %s "$DST/fairy/$name") = $size ] && continue
  while [ $(( $(df -B1 --output=avail $DST | tail -1) / 1000000000 )) -lt $(( size / 1000000000 + RESERVE )) ]; do
    echo "$(date '+%F %T') waiting for space: $name needs $((size/1000000000))+$RESERVE GB, free $(df -h --output=avail $DST | tail -1)" >> $LOG; sleep 1800
  done
  echo "=== $(date '+%F %T') START $name" >> $LOG
  rclone copy "$SRC/fairy" "$DST/fairy" --include "$name" "${OPTS[@]}"
  echo "=== $(date '+%F %T') END $name exit=$? sha: $(cd $DST/fairy && sha256sum -c $name.sha256 2>&1 | tail -1)" >> $LOG
done
echo "=== $(date '+%F %T') ALL DONE" >> $LOG
