#!/bin/bash
# Extract every zip left in <dataset>/_zips/ into <dataset>/ (one at a time), deleting each zip once it is extracted.
# A zip still there is not yet extracted (download finished: pget renames .part only when complete). Resumable.
# usage: reunzip.sh <dataset dir> [...]
for d in "$@"; do
  for z in "$d"/_zips/*.zip; do
    [ -e "$z" ] || continue
    echo "$(date +%T) unzip $z"
    if unzip -q -o "$z" -d "$d"; then rm -f "$z"; touch "${z%.zip}.extracted"; echo "$(date +%T) ok"
    else echo "$(date +%T) FAILED $z"; fi
  done
done
