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

## 2. A/B carrier + A on CODa (9 tuning windows) and street, x3, e3 all frames (20:27)
agg: `python3 eval/scripts/analysis/agg_e3.py LXI,LXC,LXA "" mdp_street_vjsye2i2pl,mdp_street_vjsye2i2pc,mdp_street_vjsye2i2pca`
(columns per seq 0 / 10 / 16; PD found / precision; vehicle recall = coda_vehicle_e3 dynamic_only)

| | MOTA moving 0 / 10 / 16 | MOTA all 0 / 10 / 16 | PD found / prec | vehicle rec | IDSW | lat med / p90 |
|---|---|---|---|---|---|---|
| LXI = I2P | 80.1 / 64.2 / 80.1 | 59.4 / 51.4 / 63.3 | 35.9 / 61.4 | 24.2 [22.7-26.2] | 200 | 98 / 151 |
| LXC = + carrier | 80.2 / 65.2 / 81.2 | 56.6 / 51.2 / 61.5 | 29.3 / 56.3 | 29.0 [24.1-34.3] | 162 | 93 / 154 |
| LXA = + carrier + A | 79.2 / 66.2 / 81.3 | 58.5 / 55.3 / 62.7 | 31.1 / 61.2 | 32.2 [23.0-41.0] | 150 | 96 / 169 |

Street (e3): MOTA | vehicle rec / prec | person rec / prec / MOTA
- I2P (i2pl): 50.1 [49.6-50.7] | 44.0 / 43.5 | 64.8 / 84.0 / 51.0
- + carrier: 50.4 [49.4-51.9] | 44.5 / 44.1 | 64.4 / 84.7 / 51.2
- + carrier + A: 51.0 [49.6-51.7] | 47.7 / 42.7 | 65.4 / 84.0 / 51.3
- street latency from lat_tracks reads -2 / 10 ms: the street measure is broken (not comparable), to check.

Reading:
- A (veto only for unclassified tracks) is the useful fix: seq 10 MOTA all +3.9 (51.4 -> 55.3), moving +2.0, vehicles
  +8 (24 -> 32), IDSW -50 (200 -> 150), street +0.9 MOTA / +3.7 vehicle recall.
- carrier ALONE costs on CODa: PD found 35.9 -> 29.3 and MOTA all seq 0 -2.8 / seq 16 -1.8 (fragments of a still object
  with a live carrier stay unclassified, as decided; on CODa the anchor was ALSO giving classes to legitimate pieces).
  With A the loss shrinks (seq 0 -0.9, seq 16 -0.6) but PD found stays -4.8.
- Net LXA vs I2P on the primary metric (MOTA all): -0.9 / +3.9 / -0.6 (seq 0 is noisy: 50-60 range across reps before).
- Lab: carrier is what fixes lab_u (see section 1). A trade-off to decide: carrier helps the lab, A helps CODa and street.
