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

## 3. Input audit (rep 1 runs: LXA CODa 9 windows, street i2pca r1, lab LUCAa), eval/diagnostics/input_audit/
**Effective parameters** (param_dump.sh: launch started as in the runs, `ros2 param dump` of every node; param_compare.py):
- CODa coda_vjsye2i2pca: YOLO26s-seg, 9 classes (person, bicycle, car, motorbike, bus, truck, dog, cat, horse),
  bicycle:0.3, inference 1280, one YOLO per camera (front_camera + stereo_camera), semantic-seg node NOT running
  (single_yolo + mask), ground_model grid, floor_source camera, carrier on, veto-unclassified-only on, anchor ttl 5,
  corroboration_all on, inherit probation 0.5, range 25 m, calibration_icp NOT running (enable_calibration false).
- lab lab_uca: same YOLO model / classes / thresholds, inference 640, front_camera only, ground planes, range 15 m,
  calibration_icp running.
- street mdp_street_vjsye2i2pca: as CODa, front_camera only, inference 1280, grid, calibration_icp running.
- The only differences from the YAML are launch-set by design: camera_names, tracker yolo2d_topic_template ->
  /yolo_seg_track (mask + single_yolo), calibration_icp topics/frames/intrinsics from the camera's own topics.
  (param_compare also lists detection_yolo26.yaml values: that second model file is not launched; ignore.)

**Coverage** (coverage_logs.py, coverage_logs_17nov.txt): CODa 9/9 windows 0 annotated frames before the first / after
the last tracker output, tracker msgs = scans (0_w1: 588/600, max gap 0.30 s), no gap > 0.3 s. Street: 10929/10929, 16
gaps of 0.3-0.4 s. Lab: tracker = scans; 6-12 mocap frames per sequence lie after the last scan of the sensor bag (the
mocap recording is longer): they count as FN for EVERY method (small, common to all).

**Logs**: most WARN lines are yolo_refine diagnostics (ACCEPTED / no candidate / too low for its class) and class clears.
`Invalid frame ID "<camera>_refined"` on CODa (16-36 per run) is benign: with calibration off the tracker falls back to
the camera's optical frame ("FOV pose from frame '..._color_optical_frame'" logged). calibration_icp tf2 "Do not call
canTransform with a timeout" ERROR on lab/street: cosmetic (calibration converges, VALID). No TF failures, no dropped
scans, no queue warnings.

**Camera chain CODa** (cam_chain.py, + fallback to the run's /tf_static since the NAS window bags carry an empty one;
cam_chain_LXA_17nov.txt): YOLO-scan lag median -7..+6 ms, p90 42-47 ms (16_w1 stereo 0/0); YOLO outputs on 70-97% of
scans (ZED ~7.5 Hz); annotated pedestrians in the image inside a YOLO person box: 10_w1 100%, 10_w2 38%, 10_w3 53%,
10_w4 57%, 10_w5 40%, 16_w1 89% (seq 0 windows: no annotated pedestrian with >= 20 points in the image); median
offset from the box centre |du| <= 13 px, |dv| <= 17 px -> extrinsics and timing are right; the 38-57% on 10_w2-w5 is
YOLO missing people (crowds, small/far), not geometry.

**Camera chain lab** (lab_cam.py, LUCAa; lab_cam_17nov.txt; the indoor runs do not record /yolo_seg_track, not measured):
YOLO-scan lag median 0 ms, p90 45-46 ms on all 6. Mocap person in the image -> inside a YOLO person box: 1942 100%,
1947 100%, 1949 100%, 1950 98%. In 1945 and 1952 the person NEVER projects into the camera (median camera z -2.8 m:
behind the robot, which stands still) -> there SINS can only use LiDAR motion; it explains the ~27 FN every profile has
on 1952. Horizontal offset small (du -0.02..-0.21 box widths); the 1.0 m reference point lands 0.36-0.47 box heights
above the box centre: odom z = base_link (ground_height -0.3), so the point is ~1.3 m above the floor plus a camera
height difference (the online calibration puts the camera 0.19 m below the URDF) -> a height-reference effect in this
check, not a misalignment (98-100% inside).

## 4. YOLO 2D <-> 3D association diagnosis, CODa (eval/diagnostics/yolo_assoc/, LXA rep 1, 9 tuning windows; measure only)
yolo_assoc.py: class-loss events (published class at tick i-1, none at i, track alive) and the 3-tick decay streak
checked per camera (front ZED + stereo cam0; 16_w1 stereo only) with the track box projected; ByteTrack ids per camera.
Raw: coda_LXA_17nov.txt, yolo_assoc_rows.csv.

| class | a: YOLO box of the same class covers it, not associated | b: occluded by a nearer track / truncated by the border | c: visible, no YOLO | total |
|---|---|---|---|---|
| person | 45 | 31 | 39 | 115 |
| bicycle | 13 | 0 | 4 | 17 |
| truck | 16 | 0 | 1 | 17 |
| bus | 2 | 0 | 1 | 3 |
| car | 0 | 2 | 0 | 2 |
| all | **76 (49%)** | **33 (21%)** | 45 (29%) | 154 |

- Losses with a known previous covering 2D instance (camera, id): 128; that same (camera, id) still published by its
  camera during the streak: 76 (59%) -> the class was dropped while YOLO was still tracking the same instance.
- Vehicles: 16 of the 17 truck losses and 13 of 17 bicycle losses are type a (YOLO sees them, association fails).
- What-if over the 1932 unclassified ticks that follow the losses (until the class returns / track ends, max 50):
  kept by (3) keep-while-the-same-(camera,id)-is-alive 192 (10%), by (4) evidence-only decay (occluded/truncated/not
  in view) 701 (36%), by either 893 (46%). (3) as measured is per single (camera, id); a per-camera version that
  accepts any camera keeping its own id would keep more.
- Multi-camera (windows 0/10, ZED + stereo): ticks of classified tracks covered by a same-class YOLO box in 2 cameras
  2685, in 1 camera 2256 -> ~54% of the covered ticks the object is seen by both cameras. Along the tracks the covering
  (camera, id) changed 392 times by a camera switch vs 190 times by a new id in the same camera: two thirds of the
  2D-identity changes are only "the other camera now sees it" -> with the ByteTrack id compared without its camera
  (detectionYoloTrackId / last_yolo_track_id) these read as an identity change. Confirms the peer's concern (a).
- Not yet measured: class conflicts between cameras on the same tick, double counting of class votes, lab 1950 (one
  camera only there).

## 5. e3 SINS testing frames + Dynablox / M-detector e3 (all + testing, 15 windows) + vehicles e3 (done 21:xx)
`python3 eval/scripts/analysis/cmp_e3.py <coda_e3 | coda_e3/test> LXI,LXIr2,LXIr3 LXC,... LXA,... DBX MDET`
(MOTA all annotated agents / moving, pooled per sequence; SINS = mean of 3 reps; held-out 11/12 not run for LX*)

| | seq 0 all / moving | seq 10 | seq 16 | seq 11 | seq 12 |
|---|---|---|---|---|---|
| LXI (I2P), all frames | 59.4 / 80.1 | 51.4 / 64.2 | 63.3 / 80.1 | - | - |
| LXA (+carrier +A), all frames | 58.5 / 79.2 | 55.3 / 66.2 | 62.7 / 81.3 | - | - |
| Dynablox, all frames | -284.2 / -527.8 | -74.9 / -88.6 | 38.8 / 50.9 | 10.7 / 4.3 | -181.0 / -198.5 |
| M-detector, all frames | -388.6 / -739.9 | -59.5 / -69.1 | -21.7 / -35.5 | -61.1 / -89.2 | -90.5 / -98.2 |
| LXI, testing frames | 58.0 / 79.9 | 51.2 / 62.5 | 60.0 / 79.9 | - | - |
| LXA, testing frames | 56.5 / 80.7 | 55.7 / 65.2 | 60.1 / 81.5 | - | - |
| Dynablox, testing | -286.6 / -519.5 | -92.2 / -104.5 | 30.1 / 38.4 | 17.0 / 15.0 | -181.3 / -196.4 |
| M-detector, testing | -435.3 / -807.8 | -63.3 / -70.9 | -16.9 / -32.2 | -63.0 / -88.4 | -72.3 / -78.9 |

Files: coda_e3/{DBX,MDET}_<w>.json, coda_e3/test/..., coda_vehicle_e3/{dynablox,mdetector}_<w>.json (tuning vehicle
windows). On testing frames LXA vs I2P: seq 10 +4.5, seq 0 -1.5, seq 16 +0.1 (MOTA all).
Queue from the laptop session: done, except the lab part of the YOLO-3D diagnosis and the multi-camera class-conflict
count (section 4). Nothing else launched.
