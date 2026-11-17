# Lab PC queue, 17/11 (afternoon/evening) — summary for the laptop session

Code: sins branch `floor-carrier`, local on the lab PC (not pushed yet): 6cd67e5 floor_source + track_class_anchor_carrier,
cec8e5e track_point_shift_veto_unclassified_only. All default off.

## 1. A/B carrier + A (lab_carrier.sh), x3 alternated in one chain

### Lab (6 Qualisys seqs, GT person only, --annotated-classes person) — eval/results/lab_carrier

| | indoor LIN2 | lab_uc LUC (lab_u + carrier) | lab_uca LUCA (+ A) |
|---|---|---|---|
| MOTA full a/b/c | 97.8 / 97.9 / 98.0 | 93.9 / 94.0 / 93.1 | 94.9 / 88.2 / 96.4 |
| IDF1 | 98.6 / 99.0 / 99.0 | 87.1 / 88.3 / 88.0 | 87.7 / 86.1 / 98.2 |
| FP | 9 / 7 / 2 | 19 / 30 / 51 | 15 / 39 / 15 |
| FN | 55 / 53 / 55 | 154 / 139 / 147 | 130 / 297 / 87 |
| IDSW | 1 / 0 / 0 | 4 / 4 / 2 | 3 / 5 / 2 |
| MOTA dynamic_only | 90.1 / 91.1 / 90.7 | 86.4 / 86.8 / 86.4 | 88.4 / 86.1 / 88.0 |
| recall D | 94.7 / 95.5 / 94.5 | 91.7 / 92.8 / 92.6 | 93.6 / 92.6 / 93.0 |
| latency med / p90 (ms) | 93.5 / 122 | 78 / 105 | 79.5 / 105 |

Reading: carrier closes the lab_u FP (211-334 -> 19-51; lab_u bare from lab_unified: MOTA 83.5-85.4). A: dynamic_only +1
on average. Residual gap = FN on the STANDING person: 1950 ~70 FN in 5 of 6 lab_u runs (indoor 2), 1947 intermittent
(LUCAb 180), 1952 27-28 for all profiles. The extra lifecycle replay (lab_uca 1947) did not catch a class clear.

## State at ~18:00 (usage limit reached on the lab session)
- sins floor-carrier pushed: ducciopet/sins main = cec8e5e; submodule pointer 0a8f097.
- CODa split metadata in eval/datasets/coda_metadata/ (f967db0).
- A/B chain (lab_carrier.sh, background) still running: CODa runs DONE (LXI/LXC/LXA x3, 9 windows), street in progress
  (~19 min/run, end ~20:00). At its end the chain itself scores CODa e3 (coda_e3, all frames) + street mdp_e3 + vehicles e3.
- NOT done yet (pending, in this order):
  1. FRAMES=testing for the SINS runs: TAGS="LXI LXIr2 LXIr3 LXC LXCr2 LXCr3 LXA LXAr2 LXAr3" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
  2. DBX / MDET e3 all + testing on the 15 windows: E=e3 FRAMES=testing WINDOWS="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 11_w1 11_w2 11_w3 12_w1 12_w2 12_w3"
     BASE="dynablox:/mnt/nas-imperatore/datasets_SINS/derived/baselines_out/dynabloxF_coda{c}.bag:DBX mdetector:/mnt/nas-imperatore/datasets_SINS/derived/baselines_out/mdetectorF_coda{c}.bag:MDET" bash eval/scripts/analysis/rescore_e2.sh
     + E=e3 BASE="dynablox mdetector" bash eval/scripts/analysis/coda_vehicle_e2.sh
  3. CODa/street A/B tables (agg_e2.py on coda_e3), commit results.
  4. Input audit (eval/diagnostics/input_audit: coverage_logs.py, param_dump.sh + param_compare.py, cam_chain.py, lab_cam.py).
     First lab_cam result (LUCAa 1950): mocap person inside a YOLO person box 98%, lag median 0 / p90 46 ms; projected
     1.0 m point ~0.4 box heights above the box centre (height reference to check: odom z = base_link, ground_height -0.3;
     online calibration puts the camera 0.19 m lower than the URDF).
  5. YOLO-3D association diagnosis (CODa first: still vehicles, two cameras, per-camera ByteTrack ids; then lab 1950).
