#!/bin/bash
# Laptop: orphan-piece absorption (track_absorb_pieces) — CODa tuning x1 (tag VA, coda_vjsyea), street x1 with lifecycle
cd /home/duccio/Desktop/jo_sim_sins
ENV=coda_vjsyea STEREO=1 bash eval/scripts/coda_ab.sh VA w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) coda done"
touch bags/mdp/runs/mdp_street_vjsyeadbg_r2.done bags/mdp/runs/mdp_street_vjsyeadbg_r3.done
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_vjsyeadbg" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_abs.log 2>&1
echo "$(date +%T) ABS DONE"
