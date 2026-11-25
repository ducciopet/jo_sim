#!/bin/bash
# SINS FINAL RUNS (FINAL_RUN_SPEC 2 and 5). SKELETON prepared 25/11 on the lab PC; NOT to be run before the freeze (05/12).
# Default DRY=1 prints the plan only. At the freeze: apply eval/scripts/final/run_method_dur15.patch to bags/replay/run_method.sh
# (DUR +15 s, all final runs; every development run used +3 s), copy run_*.sh into bags/replay, then DRY=0.
# Streaming (disk): one item at a time into bags/final_scratch (user pre-authorised 24/11: delete there only after the NAS copy is
# verified). Inputs come from NAS derived/final_inputs/<ds>/ (converted once; converted here and uploaded if missing).
# Outputs: NAS derived/sins_final/<ds>/<item>_r<k>/ (run bag + logs + manifest.json); JSONs eval/results/final/<ds>/<metric>/.
# Tags: SF, SFr2, SFr3, ... (TAG=SFA_<fix> for the leave-one-out ablation). Rerun once if dropped > 2% or crash (manifest).
# usage: DRY=0 REPS="1 2 3" bash eval/scripts/final/run_final.sh <coda|coda_new|kitti|doals|jrdb> <item ...>
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
  *) echo "unknown dataset $DS"; exit 1 ;;
esac
do_() { echo "+ $*"; [ "$DRY" = 1 ] || eval "$@" || { echo "FAILED: $*"; exit 1; }; }
guard() { [ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "DISK < 3 GB, stop"; exit 1; }; }
inputs() {   # names of the input folders/files of one item (bag, odom, gt...) as the dataset runner expects them
  case $DS in
    coda|coda_new) c=$1; [[ $c == *_* ]] || c=0_$c; echo "coda$c coda${c}_stereo coda${c}_odom coda${c}_gt.csv coda${c}_gt.json" ;;
    kitti) echo "$1 ${1}_odom ${1}_gt.csv ${1}_gt.json ${1}_cam.json" ;;
    doals) echo "$1 ${1}_odom ${1}_doals_gt.json" ;;
    jrdb)  echo "$1 ${1}_odom ${1}_gt.csv ${1}_gt.json" ;;
  esac; }
runname() { case $DS in coda|coda_new) c=$2; [[ $c == *_* ]] || c=0_$c; echo "$1_v2_coda$c" ;; *) echo "$1_v2_${DS}_$2" ;; esac; }
for it in $ITEMS; do
  guard
  # 1. stage the item into the scratch folder (convert + upload to final_inputs once if missing) and link it where the runner reads
  do_ mkdir -p $S $LOCAL/runs $RES
  for f in $(inputs $it); do
    do_ "[ -e $NI/$f ] && rsync -a $NI/$f $S/ || echo '[TBD] convert $DS $it into $S, then rsync -a $S/$f $NI/'"
    do_ "[ -e $LOCAL/$f ] && [ ! -L $LOCAL/$f ] && { echo 'real file in the way: $LOCAL/$f'; exit 1; } || ln -sfn ../final_scratch/$DS/$f $LOCAL/$f"
  done
  for k in $REPS; do
    t=$TAG; [ $k = 1 ] || t=${TAG}r$k; run=$(runname $t $it)
    [ -e $NO/${it}_r$k/manifest.json ] && { echo "done: $DS $it r$k"; continue; }
    guard; L0=$(cut -d' ' -f1 /proc/loadavg)
    # 2. run (container), same runners as development
    case $DS in
      coda|coda_new) do_ "docker exec -u ros jo_sim_sins bash -c \"TAG=$t ENV=$PROF STEREO=1 bash /home/ros/bags/replay/run_coda.sh $it\" > /dev/null 2>&1" ;;
      kitti) do_ "docker exec -u ros jo_sim_sins bash -c \"TAG=$t KITTI_ENV=$PROF bash /home/ros/bags/replay/run_kitti.sh $it\" > /dev/null 2>&1" ;;
      doals) do_ "docker exec -u ros jo_sim_sins bash -c \"TAG=$t DOALS_ENV=$PROF bash /home/ros/bags/replay/run_doals.sh $it\" > /dev/null 2>&1" ;;
      jrdb)  do_ "docker exec -u ros jo_sim_sins bash -c \"JRDB_ENV=$PROF TAG=$t bash /home/ros/bags/replay/run_jrdb.sh $it\" > /dev/null 2>&1" ;;
    esac
    # 3. move the run into scratch, manifest (dropped scans, load, commits, md5s), latency log
    do_ mv $LOCAL/runs/$run $LOCAL/runs/${run}_logs $S/
    do_ rm -f $LOCAL/runs/$run.done
    do_ python3 eval/scripts/final/make_manifest.py --run $S/$run --input $S/$(inputs $it | cut -d' ' -f1) --profile $PROF --load-start $L0
    # [TBD] rerun once if manifest says rerun_needed (keep both, the rerun counts)
    # 4. score into eval/results/final/$DS/<metric>/ [TBD: evaluator + metrics per dataset, FINAL_RUN_SPEC 3]
    # 5. copy to the NAS and verify (checksum dry run, -r only: SMB time/perm differences are not listed, must list nothing), then remove the run from scratch
    do_ mkdir -p $NO/${it}_r$k
    do_ rsync -a $S/$run $S/${run}_logs $NO/${it}_r$k/
    do_ "[ -z \"\$(rsync -rnc --out-format=%n $S/$run $S/${run}_logs $NO/${it}_r$k/)\" ] || { echo 'NAS verify FAILED $run'; exit 1; }"
    do_ cp $S/$run/manifest.json $NO/${it}_r$k/manifest.json
    do_ rm -rf $S/$run $S/${run}_logs
  done
  # 6. all reps of the item done and on the NAS: drop the staged inputs (scratch only) and the links
  for f in $(inputs $it); do do_ rm -rf $S/$f; do_ rm -f $LOCAL/$f; done
done
echo "$(date +%T) $DS done"
