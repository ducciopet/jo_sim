#!/bin/bash
# B1 (19/11): Dynablox and M-detector on the Marina di Pisa street (VLP-16, robot still on the kerb), same protocol as
# the lab runs (eval/baselines/dynablox/run_dynablox.sh, mdetector/run_mdetector.sh): upstream containers, lab VLP-16
# parameter files (motion_detector_jo.yaml max_range 20; M-detector jo.yaml, playback 1x), shared street odometry
# (bags/mdp/mdp_street_odom) as TF odom -> base_link, base_link -> velodyne from the street bag's /tf_static
# (0.03 0.029 0.652, the lab URDF has z 0.6), detector started 12 s before the bag. Only the street sensor bag's
# /velodyne_points is used (its own /tf odom -> base_link is the robot's, not the shared odometry).
# Scored like the SINS street runs (rescore_e2.sh E=e3): coda_eval --ignore-undefined --range 15, all / vehicle / person;
# M-detector points clustered with the lab minimum cluster size (5, VLP-16).
# usage: bash eval/baselines/street_baselines.sh   (env METHODS="dynablox mdetector")
cd $(dirname $0)/../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
R=$(pwd); B=$R/bags; S=/mnt/nas-imperatore/datasets_SINS/derived/mdp_street_work/mdp_street
OUTNAS=/mnt/nas-imperatore/datasets_SINS/derived/baselines_out
DB=$R/eval/baselines/dynablox; MD=$R/eval/baselines/mdetector
mkdir -p $B/dynablox_tmp $B/dynablox_out $B/mdetector_tmp $B/mdetector_out eval/results/mdp_e3
score() {  # method bag name [extra]
  G=bags/mdp/mdp_street; A="--method $1 --run $2 --odom ${G}_odom --range 15 --ignore-undefined $4"
  python3 eval/coda_eval.py $A --gt ${G}_gt.csv --out eval/results/mdp_e3/$3.json | cut -c1-200
  python3 eval/coda_eval.py $A --gt ${G}_vehicle_gt.csv --ignore-gt ${G}_person_gt.csv --out eval/results/mdp_e3/$3_vehicle.json > /dev/null
  python3 eval/coda_eval.py $A --gt ${G}_person_gt.csv --ignore-gt ${G}_vehicle_gt.csv --out eval/results/mdp_e3/$3_person.json > /dev/null
}
for M in ${METHODS:-dynablox mdetector}; do
  if [ $M = dynablox ]; then
    OUT=dynabloxF_street; L=$B/dynablox_out/${OUT}_logs; mkdir -p $L
    [ -e $B/dynablox_tmp/in_street_pc.bag ] || rosbags-convert --src $S --dst $B/dynablox_tmp/in_street_pc.bag \
      --dst-typestore ros1_noetic --include-topic /velodyne_points || { echo "CONVERT FAIL"; continue; }
    [ -e $B/dynablox_tmp/in_street_odom.bag ] || rosbags-convert --src $B/mdp/mdp_street_odom --dst $B/dynablox_tmp/in_street_odom.bag \
      --dst-typestore ros1_noetic --include-topic /odometry/filtered /tf || { echo "CONVERT FAIL"; continue; }
    sed 's|args="0.03 0.029 0.6 0 0 0 base_link velodyne"|args="0.03 0.029 0.652 0 0 0 base_link velodyne"|' $DB/run_dynablox_jo.launch > $B/dynablox_tmp/launch_street.launch
    grep -q 0.652 $B/dynablox_tmp/launch_street.launch || { echo "TF SED FAIL"; continue; }
    docker rm -f dynablox_run > /dev/null 2>&1
    echo "$(date +%T) dynablox start"
    timeout -k 30 2400 docker run --rm --name dynablox_run -v $B:/bags -v $DB:/jo dynablox-noetic:upstream bash -c "
      source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash
      roscore > /bags/dynablox_out/${OUT}_logs/roscore.log 2>&1 &
      for i in \$(seq 60); do rosparam list > /dev/null 2>&1 && break; sleep 1; done
      rosparam set use_sim_time true || exit 3
      roslaunch /bags/dynablox_tmp/launch_street.launch > /bags/dynablox_out/${OUT}_logs/detector.log 2>&1 &
      sleep 12
      rosbag record -O /bags/dynablox_out/$OUT.bag /motion_detector/visualization/clusters /odometry/filtered __name:=rec > /bags/dynablox_out/${OUT}_logs/rec.log 2>&1 &
      sleep 2
      rosbag play --clock -q /bags/dynablox_tmp/in_street_pc.bag /bags/dynablox_tmp/in_street_odom.bag > /bags/dynablox_out/${OUT}_logs/play.log 2>&1
      sleep 3; rosnode kill /rec; sleep 2" || echo "$(date +%T) RUN FAIL dynablox"
    docker rm -f dynablox_run > /dev/null 2>&1
    echo "$(date +%T) dynablox done"
    score dynablox $B/dynablox_out/$OUT.bag dynablox_street
    rm -f $B/dynablox_tmp/in_street_pc.bag $B/dynablox_tmp/in_street_odom.bag
  else
    OUT=mdetectorF_street; L=$B/mdetector_out/${OUT}_logs; mkdir -p $L
    [ -e $B/mdetector_tmp/in_street.bag ] || python3 $MD/make_input_street.py $B/mdetector_tmp/in_street.bag $S || { echo "INPUT FAIL"; continue; }
    docker rm -f mdetector_run > /dev/null 2>&1
    echo "$(date +%T) mdetector start"
    timeout -k 30 2400 docker run --rm --name mdetector_run -v $B:/bags -v $MD:/jo mdetector-noetic:upstream bash -c "
      source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash
      roscore > /bags/mdetector_out/${OUT}_logs/roscore.log 2>&1 &
      for i in \$(seq 60); do rosparam list > /dev/null 2>&1 && break; sleep 1; done
      rosparam set use_sim_time true || exit 3
      roslaunch /jo/run_mdetector_jo.launch > /bags/mdetector_out/${OUT}_logs/detector.log 2>&1 &
      sleep 12
      rosbag record -O /bags/mdetector_out/$OUT.bag /m_detector/point_out /m_detector/frame_out /odom_velodyne __name:=rec > /bags/mdetector_out/${OUT}_logs/rec.log 2>&1 &
      sleep 2
      rosbag play --clock -q /bags/mdetector_tmp/in_street.bag > /bags/mdetector_out/${OUT}_logs/play.log 2>&1
      sleep 3; rosnode kill /rec; sleep 2" || echo "$(date +%T) RUN FAIL mdetector"
    docker rm -f mdetector_run > /dev/null 2>&1
    echo "$(date +%T) mdetector done"
    score mdetector $B/mdetector_out/$OUT.bag mdetector_street "--points-min-cluster 5"
    rm -f $B/mdetector_tmp/in_street.bag
  fi
done
for f in $B/dynablox_out/dynabloxF_street.bag $B/mdetector_out/mdetectorF_street.bag; do [ -e $f ] && cp $f $OUTNAS/; done
echo "$(date +%T) STREET BASELINES DONE"
