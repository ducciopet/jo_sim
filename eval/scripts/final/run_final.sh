#!/bin/bash
# SINS FINAL RUNS (FINAL_RUN_SPEC 2 and 5). SKELETON prepared 25/11 on the lab PC; NOT to be run before the freeze (05/12).
# Default DRY=1 prints the plan only. At the freeze: apply eval/scripts/final/run_method_dur15.patch to bags/replay/run_method.sh
# (DUR +15 s, all final runs; every development run used +3 s), copy run_*.sh into bags/replay, then DRY=0.
# Streaming (disk): one item at a time into bags/final_scratch (user pre-authorised 24/11: delete there only after the NAS copy is
# verified). Inputs come from NAS derived/final_inputs/<ds>/ (converted once; converted here and uploaded if missing).
# Outputs: NAS derived/sins_final/<ds>/<item>_r<k>/ (run bag + logs + manifest.json); JSONs eval/results/final/<ds>/<metric>/.
# Tags: SF, SFr2, SFr3, ... (TAG=SFA_<fix> for the leave-one-out ablation). Rerun once if dropped > 2% or crash (manifest).
# Street (Marina di Pisa) is NOT a reported dataset any more (user decision 25/11; a CROSSLAB mocap experiment replaces it, not before the freeze).
# usage: DRY=0 REPS="1 2 3" bash eval/scripts/final/run_final.sh <coda|coda_new|kitti|doals|jrdb|lab> <item ...>   (CODa tuning: REPS="1 2 3 4 5")
#   items: coda: w1 10_w1 ... (window names); kitti: 0000..0020; doals: hauptgebaeude_1 ...; jrdb: forbes huangb ...
cd $(dirname $0)/../../..
DS=$1; shift; ITEMS="$@"; DRY=${DRY:-1}; TAG=${TAG:-SF}; REPS=${REPS:-1 2 3}
N=/mnt/nas-imperatore/datasets_SINS; NI=$N/derived/final_inputs/$DS; NO=$N/derived/sins_final/$DS
S=bags/final_scratch/$DS; RES=eval/results/final/$DS
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
case $DS in   # profile per dataset (FINAL_RUN_SPEC 2) [TBD at the freeze: final profile names]
  coda|coda_new) PROF=${PROF:-coda_FINAL}; LOCAL=bags/coda ;;
  kitti) PROF=${PROF:-kitti_tracking}; LOCAL=bags/kitti ;;
  doals) PROF=${PROF:-doals}; LOCAL=bags/doals ;;
  jrdb)  PROF=${PROF:-jrdb}; LOCAL=bags/jrdb ;;
  lab)   PROF=${PROF:-lab_ucar1lccbma}; LOCAL=bags/lab_qualisys; NI=$N/derived/lab_qualisys/bags_benchmark ;;   # items: 20260519_1942 ...
  *) echo "unknown dataset $DS"; exit 1 ;;
esac
# LOAD GATE (26/11, same-profile drift under another user's load, lab_queue 41): a run starts only when load5 < 1.5 and
# load1 < 2, waiting up to GATE_MAX s (2 h); past the deadline it runs anyway and is flagged "loaded" in the manifest and
# requeued once at the end of the dataset. During each run load1/load5 are sampled every 30 s (median/max in the manifest);
# latency p90 > 1.5 x QUIET_P90 (ms, the dataset's quiet value) also flags a rerun. REQUEUE: at the end of the dataset each flagged
# item/rep is rerun once through the same gate into <item>_r<k>_rerun/; the rerun replaces the flagged run in the tables, both
# manifests kept; if the rerun is flagged too, the better-gated one is kept (quiet over loaded, then lower load5 median) and is
# marked loaded in the tables. The choice is written to <item>_r<k>/selected.json on the NAS.
GATE_MAX=${GATE_MAX:-7200}
case $DS in   # quiet latency p90 [ms] per dataset, fixed 26/11 from quiet-hours dev medians (FINAL_RUN_SPEC 5), never from a final run
  coda|coda_new) Q=125 ;; kitti) Q=175 ;; doals) Q=104 ;; jrdb) Q=42 ;; lab) Q=115 ;;
esac; QUIET_P90=${QUIET_P90:-$Q}
gate() { local t0=$(date +%s); while awk '{exit !($2 >= 1.5 || $1 >= 2.0)}' /proc/loadavg; do
  [ $(( $(date +%s) - t0 )) -ge $GATE_MAX ] && { echo loaded; return; }; sleep 60; done; echo quiet; }
do_() { echo "+ $*"; [ "$DRY" = 1 ] || eval "$@" || { echo "FAILED: $*"; exit 1; }; }
guard() { [ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "DISK < 3 GB, stop"; exit 1; }; }
inputs() {   # names of the input folders/files of one item (bag, odom, gt...) as the dataset runner expects them
  case $DS in
    coda|coda_new) c=$1; [[ $c == *_* ]] || c=0_$c; echo "coda$c coda${c}_stereo coda${c}_odom coda${c}_gt.csv coda${c}_gt.json" ;;
    kitti) echo "$1 ${1}_odom ${1}_gt.csv ${1}_gt.json ${1}_cam.json" ;;
    doals) echo "$1 ${1}_odom ${1}_doals_gt.json" ;;
    jrdb)  echo "$1 ${1}_odom ${1}_gt.csv ${1}_gt.json ${1}_pano.csv" ;;
    lab)   echo "duccio_test_${1}_0" ;;   # odom_<seq> and the mocap GT (qualysis/) are local in bags/lab_qualisys
  esac; }
runname() { case $DS in coda|coda_new) c=$2; [[ $c == *_* ]] || c=0_$c; echo "$1_v2_coda$c" ;; lab) echo "$1_$2" ;; *) echo "$1_v2_${DS}_$2" ;; esac; }
# SCORING (FINAL_RUN_SPEC 3.1/3.1b/3.2), per run while it is in scratch: <RES>/<view>[/test]/<tag>_<item><sfx>.json
# (the tables take the run named by <item>_r<k>/selected.json when a rerun exists). Lab: scored once per dataset (score_lab).
CE() { do_ "python3 eval/coda_eval.py $* > /dev/null 2>&1 || echo 'SCORE FAILED: $*'"; }
score() { local it=$1 t=$2 run=$3 sfx=$4 c=$1; [[ $c == *_* ]] || c=0_$c; local o
  case $DS in
    coda|coda_new) local B="--run $S/$run --gt $S/coda${c}_gt.csv --odom $S/coda${c}_odom"
      for v in "e3|--ignore-undefined" "e3ib|--ignore-undefined --ignore-box" "state|--ignore-undefined --state-buffer 0.5" \
               "pdprev|--ignore-undefined --pd-prev-observable" "dyn0.2|--ignore-undefined --state-buffer 0.5 --dyn-speed 0.2" \
               "dyn0.4|--ignore-undefined --state-buffer 0.5 --dyn-speed 0.4"; do
        for F in all testing; do o=$RES/${v%%|*}; [ $F = testing ] && o=$o/test; do_ mkdir -p $o
          CE ${v#*|} --frames $F $B --out $o/${t}_$it$sfx.json; done; done ;;
    kitti) local B="--run $S/$run --gt $S/${it}_gt.csv --odom $S/${it}_odom --camera-only --cam-json $S/${it}_cam.json --range 40"
      for v in "e3|--ignore-undefined" "state|--ignore-undefined --state-buffer 0.5" "pdprev|--ignore-undefined --pd-prev-observable"; do
        o=$RES/${v%%|*}; do_ mkdir -p $o; CE ${v#*|} $B --out $o/${t}_$it$sfx.json; done ;;
    jrdb)  local B="--run $S/$run --gt $S/${it}_gt.csv --odom $S/${it}_odom --range 15 --extra-ignore $S/${it}_pano.csv"
      for v in "e3|--ignore-undefined" "state|--ignore-undefined --state-buffer 0.5"; do
        o=$RES/${v%%|*}; do_ mkdir -p $o; CE ${v#*|} $B --out $o/${t}_$it$sfx.json; done ;;
    doals) for v in "d|dynamic" "dpd|dynamic potentially_dynamic"; do o=$RES/doals_${v%%|*}; do_ mkdir -p $o
        do_ "python3 eval/doals_eval.py --run $S/$run --bag $S/$it --odom $S/${it}_odom --gt $S/${it}_doals_gt.json --states ${v#*|} --out $o/${t}_$it$sfx.json > /dev/null 2>&1 || echo 'SCORE FAILED doals $run'"; done ;;
  esac; }
score_lab() {   # mocap evaluator over the selected run of every seq x rep, dyn 0.3 / 0.2 / 0.4 (as lab_dev_views)
  local E=$S/eval_runs M=""; do_ rm -rf $E; do_ mkdir -p $E
  for k in $REPS; do local t=$TAG; [ $k = 1 ] || t=${TAG}r$k; M="$M $t"
    for it in $ITEMS; do local d=$NO/${it}_r$k sub=""
      do_ "sub=; [ -e $d/selected.json ] && python3 -c \"import json;exit(json.load(open('$d/selected.json'))['use']!='rerun')\" && sub=_rerun; ln -sfn ${d}\$sub/${t}_$it $E/${t}_$it"; done; done
  do_ mkdir -p $RES/mocap
  do_ "python3 eval/run_own_eval.py --runs $E --mocap $LOCAL/qualysis --methods $M --annotated-classes person --dyn-speed 0.3 0.2 0.4 --out $RES/mocap > $RES/mocap/eval.log 2>&1 || echo 'SCORE FAILED lab'"; }
stage() { local it=$1   # 1. stage the item into the scratch folder and link it where the runner reads
  do_ mkdir -p $S $LOCAL/runs $RES
  for f in $(inputs $it); do
    do_ "[ -e $S/$f ] || [ -e $NI/$f ] && { [ -e $S/$f ] || rsync -a $NI/$f $S/; } || echo '[TBD] convert $DS $it into $S, then rsync -a $S/$f $NI/'"
    do_ "[ -e $LOCAL/$f ] && [ ! -L $LOCAL/$f ] && { echo 'real file in the way: $LOCAL/$f'; exit 1; } || ln -sfn ../final_scratch/$DS/$f $LOCAL/$f"
  done; }
unstage() { local it=$1   # inputs out of scratch (scratch only) and the links
  for f in $(inputs $it); do do_ rm -rf $S/$f; do_ rm -f $LOCAL/$f; done; }
run_one() { local it=$1 k=$2 sfx=$3   # one rep; sfx "" or "_rerun"
  local t=$TAG; [ $k = 1 ] || t=${TAG}r$k; local run=$(runname $t $it); local dst=$NO/${it}_r$k$sfx
  [ -e $dst/manifest.json ] && { echo "done: $DS $it r$k$sfx"; return; }
  guard; local G=quiet; [ "$DRY" = 1 ] || G=$(gate); local L0=$(cut -d' ' -f1 /proc/loadavg)
  local LL=$S/${run}_load.txt SAMP=""; [ "$DRY" = 1 ] || { : > $LL; ( while true; do echo "$(date +%s) $(cut -d' ' -f1-2 /proc/loadavg)" >> $LL; sleep 30; done ) & SAMP=$!; }
  # 2. run (container), same runners as development
  case $DS in
    coda|coda_new) do_ "docker exec -u ros jo_sim_sins bash -c \"TAG=$t ENV=$PROF STEREO=1 bash /home/ros/bags/replay/run_coda.sh $it\" > /dev/null 2>&1" ;;
    kitti) do_ "docker exec -u ros jo_sim_sins bash -c \"TAG=$t KITTI_ENV=$PROF bash /home/ros/bags/replay/run_kitti.sh $it\" > /dev/null 2>&1" ;;
    doals) do_ "docker exec -u ros jo_sim_sins bash -c \"TAG=$t DOALS_ENV=$PROF bash /home/ros/bags/replay/run_doals.sh $it\" > /dev/null 2>&1" ;;
    jrdb)  do_ "docker exec -u ros jo_sim_sins bash -c \"JRDB_ENV=$PROF TAG=$t bash /home/ros/bags/replay/run_jrdb.sh $it\" > /dev/null 2>&1" ;;
    lab)   do_ "docker exec -u ros jo_sim_sins bash -c \"EXTRA_TOPICS='/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_seg_track/front_camera/detections /tf /tf_static' ADAPTER=1 V2_ARGS='env:=$PROF camera_names:=front_camera raw_depth_transport:=raw' bash /home/ros/bags/replay/run_method.sh v2 /home/ros/bags/lab_qualisys/duccio_test_${it}_0 /home/ros/bags/lab_qualisys/odom_$it /home/ros/bags/lab_qualisys/runs/$run\" > /dev/null 2>&1" ;;
  esac
  [ -n "$SAMP" ] && kill $SAMP 2>/dev/null
  # 3. manifest (dropped scans, load gate + load during the run, latency p90, commits, md5s) while the run is where the container sees it
  do_ python3 eval/scripts/final/make_manifest.py --run $LOCAL/runs/$run --input $S/$(inputs $it | cut -d' ' -f1) --profile $PROF \
      --load-start $L0 --gate $G --load-log $LL --quiet-p90 $QUIET_P90 ${sfx:+--rerun-of $NO/${it}_r$k}
  do_ mv $LOCAL/runs/$run $LOCAL/runs/${run}_logs $S/
  do_ "mv $LL $S/$run/load.txt 2>/dev/null || true"
  do_ rm -f $LOCAL/runs/$run.done
  # 4. score (all views of FINAL_RUN_SPEC 3 for this dataset; lab at the end of the dataset)
  [ $DS = lab ] || score $it $t $run "$sfx"
  # 5. copy to the NAS and verify (checksum dry run, -r only: SMB time/perm differences are not listed, must list nothing)
  do_ mkdir -p $dst
  do_ rsync -a $S/$run $S/${run}_logs $dst/
  do_ "[ -z \"\$(rsync -rnc --out-format=%n $S/$run $S/${run}_logs $dst/)\" ] || { echo 'NAS verify FAILED $run'; exit 1; }"
  do_ cp $S/$run/manifest.json $dst/manifest.json
  do_ rm -rf $S/$run $S/${run}_logs
  if [ -z "$sfx" ]; then   # flagged -> requeue at the end of the dataset
    do_ "python3 -c \"import json,sys; sys.exit(0 if json.load(open('$dst/manifest.json'))['rerun_needed'] else 1)\" && echo '$it $k' >> $S/requeue.txt || true"
  else                     # choose between the flagged run and its rerun
    do_ python3 eval/scripts/final/select_run.py $NO/${it}_r$k $dst
  fi; }
for it in $ITEMS; do
  stage $it
  for k in $REPS; do run_one $it $k ""; done
  unstage $it
done
# REQUEUE: each flagged item/rep once, same gate
if [ -s $S/requeue.txt ]; then
  echo "$(date +%T) requeue: $(wc -l < $S/requeue.txt) runs"
  while read it k; do stage $it; run_one $it $k _rerun; unstage $it; done < $S/requeue.txt
  do_ mv $S/requeue.txt $S/requeue_done_$(date +%Y%m%d_%H%M).txt
fi
[ $DS = lab ] && score_lab
echo "$(date +%T) $DS done"
