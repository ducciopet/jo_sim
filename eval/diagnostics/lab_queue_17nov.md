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

## 6. S1+S2 (eval/PIPELINE.md) launched 18/11 ~21:40 as one chain (s12_chain.sh), sins branch yolo-assoc 43768e1
New switches (all default off, counters + cost in the 50-tick log): track_yolo_id_per_camera (M; dbscan passes the id's
camera), track_class_keep_while_2d_alive (P3), track_class_decay_visible_only (P4), track_vehicle_extent_prior (P5);
P2 = the existing yolo_refine_merge_fragments. Smoke test 10_w4 (all on): no crash, check cost 0.012 ms median.
Arms, CODa 9 windows + street, x3 alternated: S0 = R0 (I2P+carrier+A) / SA = I2P+A without carrier / SM = R0+M /
SMP = R0+M+P3+P4 / SP2 = +P2 / SV = +P2+P5; scored e3 all + testing + vehicles; then lab regression lab_uca vs
lab_ucamv x3. Expected end ~09:00 on 19/11. Then S4 (ground), S5 (640 vs 1280).

## 7. S1+S2 results (s12_chain.sh, ended 19/11 08:30), e3, x3 in one chain
CODa, MOTA all / moving per seq (pooled windows, mean of 3 reps; `cmp_e3.py`), + agg_e3.py columns:

| arm | all frames 0 / 10 / 16 | testing frames 0 / 10 / 16 | PD found / prec | vehicle rec | IDSW | lat med / p90 |
|---|---|---|---|---|---|---|
| S0 = R0 (I2P+carrier+A) | 55.2/73.3, 51.3/62.5, 62.2/78.4 | 53.6/74.2, 50.6/60.1, 59.3/78.9 | 35.3 / 58.9 | 28.8 | 211 | 79 / 154 |
| SA = I2P+A, no carrier | 53.1/73.3, 47.6/63.2, 60.7/79.3 | 52.3/74.2, 48.0/61.9, 57.4/78.8 | 32.8 / 52.2 | 28.2 | 238 | 94 / 144 |
| SM = R0+M | **60.9/76.7**, 49.4/63.2, 61.5/79.1 | **58.6/76.3**, 49.6/61.0, 58.1/78.1 | 32.3 / 58.6 | 25.5 | **190** | 100 / 144 |
| SMP = +P3+P4 | 52.1/70.9, 46.8/62.4, 62.6/80.3 | 48.6/69.5, 46.2/60.9, 58.2/78.8 | 34.9 / 52.1 | **35.4** | 209 | 82 / 150 |
| SP2 = +P2 | 51.7/73.7, 47.8/62.6, 62.1/78.3 | 47.8/74.7, 47.0/61.1, 59.4/78.0 | 38.4 / 54.0 | 30.5 | 241 | 79 / 151 |
| SV = +P2+P5 | 45.4/73.7, 50.3/62.4, 59.8/77.7 | 41.2/76.8, 46.7/58.9, 55.7/76.5 | 35.2 / 51.8 | 31.1 | 287 | 80 / 143 |

Street (e3): MOTA | vehicle rec / prec | person rec / prec / MOTA
- S0 52.4 [52.2-52.5] | 49.2 / 44.6 | 65.1 / 85.6 / 52.8
- SA 51.4 | 50.2 / 43.7 | 65.2 / 84.5 / 51.5
- SM 51.1 | 49.2 / 43.7 | 64.9 / 84.4 / 51.2
- SMP 52.0 | 51.6 / 45.1 | 64.9 / 84.9 / 51.9
- SP2 51.1 | 49.9 / 44.3 | 64.6 / 84.4 / 51.1
- SV 46.1 [43.6-47.5] | 34.3 / 29.8 | 64.9 / 81.2 / 48.3

Lab regression (lab_s12, MOTA full a/b/c): lab_uca 95.3 / 95.0 / 96.0 (FN 100/137/88); lab_ucamv 96.3 / 90.6 / 94.3
(FN 78/240/137) -> no clear regression beyond the known intermittent standing-person loss (b run).

Counters (10_w4, last 50-tick log, totals): SM per-camera id new-camera 106 / same 866 / changed 33; SMP kept by a live
2D instance 168 ticks, decay skipped as hidden 285; SV vehicle boxes extended 206.

Reading:
- Chain-to-chain drift is large: R0 in this chain scores seq 10 51.3 (all) vs 55.3 in the 17/11 carrier chain (same
  profile, same build behaviour). Only within-chain comparisons are valid.
- No new switch beats R0 consistently. SM (per-camera id) is the only candidate: seq 0 +5.7 all / +3.4 moving (testing
  +5.0 / +2.1), IDSW -21, but seq 10 -1.9 and seq 16 -0.7 (testing -1.0 / -1.2), street -1.3. Mixed, inside the noise
  of seq 10/16; seq 0 is the noisiest sequence.
- P3+P4 (SMP): vehicles +6.6 recall and street vehicles +2.4, but MOTA all -3/-4.5 on seq 0/10: keeping the class
  longer keeps more PD boxes that do not match (PD precision 58.9 -> 52.1).
- P2: neutral to slightly worse. P5 (vehicle extent prior): clearly harmful (street vehicle recall 49 -> 34, precision
  45 -> 30; CODa seq 0 -6): the extended centre moves past the 1 m match gate. Drop P5.
- SA (no carrier) is below R0 here on all three seqs and the street: R0 stays the reference.
- Proposal: keep R0; optional confirm-run R0 vs R0+M with more reps (x5) if M is wanted; P3/P4/P2/P5 stay off.

## 8. S4 ground test results (s4_ground.sh, 08:33 -> 13:35), e3, x3 in one chain
Arms on R0: G0 = grid (pcag), GL = planes with floor from the raw LiDAR (pcapl, floor_source lidar), GC = planes with floor
from cameras (pcapc). Note: G0 rep1 of 0_w1 was not run (an older G0_w1.json had the same name, so lab_local skipped it):
G0 on 0_w1 is x2.

| MOTA all / moving | 0 | 10 | 16 | 0 testing | 10 testing | 16 testing | PD prec | CODa veh rec | IDSW |
|---|---|---|---|---|---|---|---|---|---|
| G0 grid       | 58.1 / 75.4 | 48.4 / 62.3 | 61.4 / 79.9 | 58.2 / 77.0 | 48.8 / 60.1 | 57.3 / 78.0 | 30.8 / 57.3 | 35.0 | 207 |
| GL lidar floor| 58.6 / 68.1 | 50.7 / 63.9 | 61.4 / 78.6 | 56.8 / 69.0 | 50.3 / 63.2 | 57.9 / 77.2 | 32.2 / 58.5 | 35.6 | 218 |
| GC camera floor| 51.0 / 71.4 | 47.4 / 57.7 | 39.1 / 50.2 | 50.7 / 72.9 | 47.1 / 55.6 | 37.4 / 50.7 | 20.3 / 51.5 | 24.9 | 121 |

Street (x3): MOTA G0 50.6 [50.1-51.6], GL 52.0 [50.7-53.0], GC 54.5 [51.7-56.3]; persons MOTA 50.9 / 53.7 / 55.2;
vehicles recall 47.2 / 42.6 / 49.2, precision 42.6 / 32.5 / 39.5.
Floor-from-LiDAR cost: CODa 4-8 ms median, p90 6-12 ms per scan (plane found 47-50/50); street 2.3 ms median.
Latency CODa median/p90: G0 79/126, GL 78/105, GC 94/127 ms. Street latency still reads -1..-3 ms (broken measure, to fix before S6).

Reading:
- GC (camera floor) is clearly worse on CODa (seq 16 -22 MOTA all, vehicles -10): out. Its street gain (+3.9) does not transfer.
- GL: +2.3 on seq 10 all (+1.5 testing), flat on 16, street +1.4 (persons +2.8, but vehicle precision -10). On seq 0
  moving it loses 7 points: on 0_w1 the FP on moving people doubles (~80 -> 170-200/min) and IDSW rises (45 -> 70-100);
  0_w2/0_w3 are within noise. Not a clean win: GL is not adopted as the default; worth a look at 0_w1 (what the
  LiDAR plane keeps that the grid removes) only if the ground becomes the gap.
- Grid (R0) stays the reference.

## 9. P5 (vehicle extent prior) bug check (eval/diagnostics/yolo_assoc/p5_check.py), 18/11 afternoon
The S1+S2 arms were cumulative (SV = M+P3+P4+P2+P5), so P5 alone = SP2 (pcamp) vs SV (pcamv), same chain, rep1.
GT vehicles matched to a published obstacle within 3 m; radial error = (published - GT centre) on the ray robot -> GT,
negative = short of the sensor.

| | paired | radial shift (P5 - ref) | moved away / towards sensor | centre < 1 m ref -> P5 | end-on radial ref -> P5 | end-on ray extent ref / P5 / GT |
|---|---|---|---|---|---|---|
| CODa 10_w1 | 329 | +1.56 m | 84% / 6% | 47% -> 60% | -0.84 -> +0.46 | 1.06 / 6.90 / 5.85 |
| CODa 10_w4 | 160 | +0.32 m | 51% / 9% | 48% -> 62% | -0.70 -> -0.12 | 0.74 / 1.98 / 5.54 |
| street r1 | 772 | +0.61 m | 81% / 6% | 90% -> 58% | +0.25 -> +0.77 | 1.16 / 1.90 / 3.14 |

- No sign or axis error: the centre moves away from the sensor along the ray, and the extended axis is the one on the ray
  (length end-on, width side-on). On CODa end-on vehicles (OS1-128) it does what it should: centres ~0.8 m short come
  to the GT (end-on < 1 m 50 -> 72-83%); on 10_w1 trucks get the 6.9 m prior (GT 5.85 m) and overshoot a bit.
- On the street the reference centres are NOT short of the sensor (radial +0.2 m median, 90% already within 1 m): the
  vehicles are mostly side-on (711/772) and already centred on the GT, so the +0.6 m push overshoots past the 1 m gate
  (90 -> 58%; street vehicle recall 49.9 -> 34.3, MOTA 51.1 -> 46.1). The prior's premise (only the near face is seen,
  centre short by half the hidden depth) holds on CODa end-on but not on the street sequence.
- Net on CODa (SP2 -> SV): vehicle recall 30.5 -> 31.1 (noise), seq 0 MOTA all 51.7 -> 45.4, IDSW 241 -> 287.
- Verdict: not a bug, P5 stays off (no extra R0+P5 arm in S5).

## 10. S5 (YOLO 640 vs 1280) + M confirmation (s5m_chain.sh, ended 18/11 21:18), e3, one chain
Y0 = R0 (coda_vjsye2i2pca / street pcay), YM = R0+M (per-camera YOLO identity, pcm / pcmy), Y6 = R0 with YOLO
inference_size 640 (pca6). x3 alternated on the 9 tuning windows + street; YM and Y0 reps 4-5 on seq 10, 16 and street (x5).

CODa MOTA all / moving (cmp_e3.py), x3:

| arm | all frames 0 / 10 / 16 | testing frames 0 / 10 / 16 | PD found / prec | vehicle rec | IDSW | lat med / p90 |
|---|---|---|---|---|---|---|
| Y0 = R0 | 59.6/77.0, 49.3/61.5, 58.5/73.6 | 58.5/78.6, 48.4/59.2, 53.7/71.3 | 33.1 / 58.3 | 23.2 | 329 | 86 / 126 |
| YM = R0+M | 57.4/77.3, 49.5/61.5, 59.4/75.9 | 55.9/79.9, 48.6/58.7, 54.5/73.7 | 32.2 / 55.2 | 28.4 | 258 | 92 / 130 |
| Y6 = 640 | 56.4/75.9, 50.6/62.8, 57.5/77.7 | 55.3/77.6, 50.0/61.1, 52.6/75.5 | 27.5 / 52.1 | 30.9 | 265 | 86 / 130 |

x5 on seq 10 / 16 (all, testing): Y0 50.5 / 59.6 (49.6 / 55.4), YM 50.3 / 60.7 (49.6 / 56.5).
Street e3 x3 (moving MOTA | veh rec / prec | person rec / prec / MOTA): Y0 52.7 | 50.7 / 45.6 | 65.1 / 85.6 / 52.8;
YM 52.2 | 50.5 / 44.7 | 65.4 / 85.0 / 52.3; Y6 51.8 | 50.8 / 45.7 | 64.5 / 85.1 / 51.7.
Street x5 (merged MOTA all / moving): Y0 43.9±0.9 / 52.4, YM 43.8±0.8 / 52.1.

Reading:
- M: neutral. x5 on seq 10 identical (testing 49.6 = 49.6), seq 16 +1.1 (all and testing), street -0.1; seq 0 (x3) now
  -2.2 all / -2.6 testing, opposite sign to the S1 chain (+5.7): seq 0 swings are noise. Fewer IDSW (329 -> 258) and
  vehicle recall +5, but no MOTA gain on the native metric -> M stays off.
- YOLO 640: no latency gain on this PC (median 86 ms both), PD found -5.6 / precision -6, seq 0 -3.2 and street -0.9;
  seq 10 +1.3 only. 1280 stays.
- Street latency still reads ~0 ms (sim-clock issue, open).

## 11. Night 18-19/11: P5b, lab grid ground, motion-threshold sensitivity (after_s5m.sh, ended 03:48)
sins p5b-heading (P5b/P6/P7 code, all default off) built OK at 21:19.

P5b (heading-gated vehicle extent prior), x3 alternated, Z0 = R0 / ZH = R0+P5b:

| arm | all frames 0 / 10 / 16 | testing frames 0 / 10 / 16 | PD found / prec | vehicle rec | IDSW | lat med / p90 |
|---|---|---|---|---|---|---|
| Z0 = R0 | 59.8/77.7, 49.5/61.8, 57.7/73.3 | 56.9/79.4, 48.9/59.8, 52.3/70.2 | 32.5 / 56.6 | 29.7 | 310 | 93 / 151 |
| ZH = +P5b | 59.6/78.4, 49.7/63.6, 57.1/73.0 | 58.3/80.5, 47.8/61.4, 51.5/68.9 | 33.0 / 55.7 | 32.6 | 316 | 88 / 150 |

Street e3 (moving MOTA | veh rec / prec | person rec / prec / MOTA): Z0 51.0 [47.7-52.8] | 48.3 / 43.8 | 65.2 / 84.1 / 51.3;
ZH 49.6 [48.2-51.0] | 44.5 / 42.8 | 64.1 / 84.3 / 50.5. (Z0 r3 is a low outlier: 630 vehicle TP vs 780 in r1/r2.)
-> P5b: CODa all-frames flat (-0.2/+0.2/-0.6), testing +1.4/-1.1/-0.8, vehicle recall +2.9; street -1.4 and vehicle
recall -3.8 on all three reps. Not a winner: no extent prior in the P6/P7 chain (arms R0 / +P6 / +P6+P7).

Lab ground (lab_ab x3 on the 6 Qualisys sequences, pooled), LZ0 = lab_uca (floor planes) / LZG = lab_ucag (grid):
- MOTA full: LZ0 94.3 / 95.0 / 94.5, LZG 96.9 / 96.7 / 92.5 (c: FN 212, the known intermittent standing-person loss).
- MOTA dynamic only: LZ0 89.4 / 91.3 / 90.8, LZG 88.3 / 89.0 / 88.9 (FN 63-76 -> 93-100, dyn recall 94.6 -> 92.7).
-> The grid has no clear lab regression on full MOTA (fewer standing-person FN) but loses ~2 points on the moving
people. Lab keeps its planes; CODa and street keep the grid (S4).

Motion-threshold sensitivity (rescore_dyn.sh, CODa e3, R0 = Y0 x3 of the S5 chain; Dynablox / M-detector from baselines_out),
MOTA all / moving, all frames (testing frames in brackets for R0):
- 0.2 m/s: R0 58.5/71.1, 49.1/60.7, 57.9/71.2 (56.6/74.6, 48.4/58.9, 52.8/67.8); Dynablox -279 / -75 / 38.3, held-out 11 10.8,
  12 -177; M-detector -382 / -59 / -21, 11 -57, 12 -89.
- 0.3 m/s (default): R0 59.6/77.0, 49.3/61.5, 58.5/73.6 (58.5/78.6, 48.4/59.2, 53.7/71.3).
- 0.5 m/s: R0 59.2/79.2, 49.4/62.0, 58.8/74.6 (57.2/77.9, 48.4/59.3, 53.9/72.6); Dynablox -293 / -75 / 38.8, 11 9.7, 12 -188;
  M-detector -398 / -60 / -22, 11 -65, 12 -94.
-> R0 MOTA all moves at most 1.9 points across 0.2-0.5 m/s; the ranking vs Dynablox / M-detector does not change at any
threshold (they stay negative except Dynablox on seq 16 and 11, still 20+ points below R0).

Next: P6/P7 chain (p67_chain.sh) started 03:55: X0 = R0 / XL = +P6 / XS = +P6+P7, CODa 9 windows + street x3, lab
LX0 lab_uca / LXS lab_ucals x3, then e3 all + testing + vehicles.

## 12. P6 / P7 (p67_chain.sh, 19/11 03:55-08:44), sins p5b-heading, x3 alternated in one chain
X0 = R0 / XL = R0+P6 (track_vehicle_lshape) / XS = R0+P6+P7 (track_shape_memory). P5 and P5b both lost -> no extent prior arm.

| arm | all frames 0 / 10 / 16 | testing frames 0 / 10 / 16 | PD found / prec | moving veh rec | IDSW | lat med / p90 |
|---|---|---|---|---|---|---|
| X0 = R0 | **60.6**/75.7, **50.9**/61.6, 59.2/76.6 | **60.3**/79.2, **50.9**/59.1, 54.5/74.9 | 31.0 / 58.5 | 25.8 | 289 | 79 / 117 |
| XL = +P6 | 57.7/73.4, 48.9/60.6, 59.4/76.4 | 56.9/76.8, 48.7/57.6, 54.8/74.6 | 31.7 / 56.3 | **39.1** | 287 | 79 / 111 |
| XS = +P6+P7 | 58.2/75.0, 48.6/60.1, 57.8/74.0 | 58.3/76.8, 47.1/57.5, 52.2/70.8 | 33.1 / 58.2 | 34.8 | 372 | 79 / 106 |

Pooled counts, all frames (3 reps): XL vs X0 TP +51 / +5 / +33 on seq 0 / 10 / 16, but FP +250 / +203 / +8 -> the loss is
extra false positives, not missed objects. Vehicle-only (full) seq 10: TP 736 -> 751, FP 3035 -> 3237, IDSW 49 -> 109.
Street e3 (moving MOTA | veh rec / prec | person MOTA): X0 51.9 | 49.7 / 44.1 | 52.1; XL INVALID (see 13) [52.4 | 49.2 / 44.6 | 52.8];
XS 50.7 | 48.6 / 42.9 | 51.0.
Lab (lab_p67, MOTA full / dynamic only, a b c): LX0 94.5 93.7 92.8 / 87.0 86.4 88.4; LXS 96.5 92.9 89.4 / 88.4 86.4 87.7
(LXS c: FN 289, the known intermittent standing-person loss) -> no evident lab regression.

Reading:
- P6 raises moving-vehicle recall a lot (+13) but costs 2-3 MOTA all points on seq 0 and 10 (testing -3.4 / -2.2)
  through extra FP; flat on seq 16. Street not measured (see 13, re-run p6street_chain.sh). Not adopted on CODa.
- P7 on top of P6: worse on 10 and 16 (testing -3.8 / -2.3 vs R0), IDSW +83, street -1.2. Not adopted.
- Vehicle box work (P5, P5b, P6, P7) is closed with no winner: R0 stays the reference for S6.
- Open question for P6 only: where the extra FP come from (re-centred/re-oriented boxes double-matching, or class
  carried to non-vehicle clusters). A per-cluster FP diff on seq 0/10 would tell; not run.

## 13. Corrections 19/11: P6 street arm, street latency
- P6 street arm invalid: the street profile name `mdp_street_vjsye2i2pcal` already existed (an R0 re-run of the 18/11
  S1+S2 chain, commit 1547030); its run bags had been archived to the NAS but the `.done` files stayed, so run_street.sh
  skipped the P6 runs and the old R0 runs were scored as "XL". The CODa XL runs are valid (fresh tags, run with the
  P6 profile then named coda_vjsye2i2pcal, now kept as coda_vjsye2i2pcp6). The pcal profiles are restored to their
  original content (R0 re-run). Other street arms checked: pcax, pcals, pcaz, pcah all run on 18-19/11.
  Re-run started 19/11 ~09:15: p6street_chain.sh, street x3 alternated pcaw (R0) / pcp6 (R0+P6).
- Street latency (open since S4): not a pipeline bug. On the street the sim clock of `ros2 bag play --clock` runs
  ~8.5 ms BEHIND the scan stamps (preprocessing "scan age at processing start" median -8.5 ms; CODa -1 ms), and the
  street scans are small (VLP-16: 29k raw -> 2.3k points after crop, preprocessing 2.7 ms vs 21.5 ms on CODa's 25k), so
  recorded - stamp read ~ -2 ms. lat_tracks.py now adds back a negative median scan age (not physical: a scan cannot be
  processed before it was taken) from <run>_logs/det.log, and prints the raw value too. Street track latency corrected:
  median 6-7 ms, p90 17-19 ms (pcax, pcals, pcay x3). CODa unchanged (offset +0.5 ms; X0 0_w1 78 / 176 ms).
  The YOLO path on the street is 171 ms behind the scan stamp (recorded - stamp of the detections), but the tracker does
  not wait for it (late YOLO leaves are class-only, 1631 in r1 log), so it does not add to the track latency.

## 14. 19/11 13:17: order changed by the user (via the laptop): leaf/size/speed chain now
- p6street_chain.sh stopped at 13:15 (container processes killed by PID + replay cleanup.sh): done pcaw r1-r2, pcp6 r1;
  pcp6_r2 interrupted (no .done; bag left on disk, not deleted: deletion needs the user's own OK). Not rerun: P6 is
  rejected on CODa and the connected leaf changes the vehicle boxes upstream. P6 FP diagnosis (p6_fp.py) skipped.
- leafc_then_b1.sh started 13:17: sins leaf-connected aebd06c built OK, first run Q0 w1 13:19. Q0/QC/QB/QS x3 on CODa
  9 windows + street (pcaq/pcalc/pcalcb/pcalcbs), lab LQ0/LQC/LQS x3 (lab_leafc), e3 rescore; then B1 street
  (eval/baselines/street_baselines.sh). Expected end: chain ~01:30 on 20/11, B1 ~03:00.

## 15. LEAFC (leaf/size/speed, 19/11 13:17-21:24) + B1 street baselines (21:25-22:09)
sins leaf-connected aebd06c. Cumulative arms x3 alternated: Q0 = R0 / QC = + connected mask leaf / QB = + cluster size
limit = bus / QS = + association speed gate 15 m/s. CODa MOTA all / moving (cmp_e3.py), 9 tuning windows:

| arm | all frames 0 / 10 / 16 | testing frames 0 / 10 / 16 | PD found / prec | moving veh rec | IDSW |
|---|---|---|---|---|---|
| Q0 = R0 | 61.5/74.9, 47.6/60.7, 59.6/76.3 | **60.8**/76.6, **47.5**/57.0, 54.6/73.3 | 30.5 / 56.8 | 32.8 | 262 |
| QC = +LC | 54.9/76.1, 48.3/60.9, 59.9/74.2 | 51.0/76.6, 46.6/58.3, 55.0/70.7 | 34.3 / 58.0 | 37.4 | 298 |
| QB = +LC+size | 61.3/76.8, 47.5/60.1, 60.2/74.6 | 58.8/77.6, 45.9/56.2, 54.8/72.0 | 34.7 / 56.6 | 41.0 | 277 |
| QS = +LC+size+speed | 61.8/78.2, 46.2/60.2, 59.1/73.3 | 58.0/79.2, 44.6/57.5, 54.3/70.4 | 34.7 / 56.4 | 42.4 | 299 |

Testing MOTA all per rep (r1/r2/r3), seq 0 | 10 | 16:
Q0 59.5/58.2/64.7 | 48.7/46.5/47.3 | 57.3/53.6/52.9; QC 55.2/51.7/46.1 | 49.5/47.7/42.6 | 53.4/55.8/55.7;
QB 52.2/62.1/62.1 | 46.0/43.9/47.7 | 54.5/54.1/55.9; QS 50.9/61.2/62.1 | 44.5/44.3/45.1 | 54.0/54.7/54.1.
Street e3 (moving MOTA | veh rec / prec | person MOTA): Q0 (pcaq) 52.5 [50.8-53.4] | 49.9 / 44.9 | 52.7; QC 52.2 | 50.7 / 44.9 |
52.3; QB 51.6 | 48.2 / 44.4 | 52.0; QS 49.9 | 44.6 / 41.3 | 50.6.
Lab (lab_leafc, MOTA full / dynamic only, a b c): LQ0 94.0 96.4 86.6 / 86.1 88.7 89.7; LQC 91.1 94.9 90.5 / 89.7 88.5 86.7;
LQS 96.6 92.8 93.7 / 89.3 86.7 86.1 -> means full 92.3 / 92.2 / 94.4, dynamic 88.2 / 88.3 / 87.4: no clear regression.
Leaf counters (QC): points of the mask kept 96-99% (was the 0.3 m depth slice), 0.09-0.12 ms per leaf; CODa track
latency median 100 / p90 125 ms in all four arms, street 7 / 19 ms (corrected clock).
People next to walls: the lab (persons beside walls and furniture) shows no regression (LQC dynamic 88.3 vs 88.2); no
per-person size log exists, so this is the only check made.

Pre-registered rule (testing MOTA all better on >= 2 of 3 seqs by more than the rep spread, no seq -1, street not worse):
- QC: 0 -9.8, 10 -0.9, 16 +0.4 -> fails. QB: -2.0, -1.6, +0.2 -> fails. QS: -2.8, -2.9, -0.3 -> fails. Street: none better.
- No LEAFC winner: R0 (Q0) stays the reference. The connected leaf does what it should for vehicles (moving vehicle recall
  32.8 -> 37.4, +size cap 41.0, +speed 42.4) but MOTA all does not follow; QC alone loses 10 points on seq 0 testing
  (r3 46.1), recovered when the size cap is added (QB): a connected leaf larger than 6 m was dropped whole by the old cap.

B1, street (eval/baselines/street_baselines.sh; lab VLP-16 configs, street extrinsic z 0.652, e3, range 15 m):

| method | MOTA all | moving MOTA | recall all | FP | IDSW | person MOTA all | vehicle MOTA all |
|---|---|---|---|---|---|---|---|
| SINS R0 (pcaq x3) | 43.7 | 52.5 | 65.5 | - | - | 42.3* | -92* |
| Dynablox | 20.5 | 17.8 | 65.3 | 3866 | 982 | 21.1 | -131.5 |
| M-detector | -195.5 | -203.3 | 95.0 | 21435 | 10037 | -229.5 | -1430 |
(* person / vehicle views of SINS from section 10, x5 Y0.) The robot is still on the kerb: M-detector flags most points
of moving and nearby still objects, Dynablox has no identity (IDF1 9%).

## 16. Error decomposition on LEAFC (eval/diagnostics/err_decomp.py, laptop script; run 19/11 22:09-~23:30)
Q0/QC/QB/QS, r1-r3 pooled, 9 tuning windows, all frames then testing frames. EMMC call skipped (CSVs not on the lab PC).
Full output: eval/diagnostics/err_decomp_leafc.txt
```
===== Q0 r1-r3, frames all
runs: ['bags/coda/runs/Q0_v2_coda{c}', 'bags/coda/runs/Q0r2_v2_coda{c}', 'bags/coda/runs/Q0r3_v2_coda{c}']  windows: ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']  frames: all

GT rows by class group (TP at the 1 m gate / found only within 2 m / missed):
  persons    26772  TP@1m  81.4%  only@2m   0.4%  missed  18.2%
  vehicles    2097  TP@1m  39.5%  only@2m  12.2%  missed  48.3%
  bikes       8940  TP@1m  33.9%  only@2m   1.1%  missed  65.0%
  other        429  TP@1m   0.0%  only@2m   0.0%  missed 100.0%

unmatched predictions at 1 m: 3914
  near persons      884: offset 178, piece 345, duplicate 9, near 238, ghost 114
  near vehicles     904: offset 284, piece 329, duplicate 2, near 289, ghost 0
  near bikes        175: offset 33, piece 74, duplicate 0, near 3, ghost 65
  near other          3: offset 0, piece 0, duplicate 0, near 3, ghost 0
  near none        1948: offset 0, piece 3, duplicate 2, near 50, ghost 1893
===== QC r1-r3, frames all
runs: ['bags/coda/runs/QC_v2_coda{c}', 'bags/coda/runs/QCr2_v2_coda{c}', 'bags/coda/runs/QCr3_v2_coda{c}']  windows: ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']  frames: all

GT rows by class group (TP at the 1 m gate / found only within 2 m / missed):
  persons    26772  TP@1m  81.7%  only@2m   0.4%  missed  17.9%
  vehicles    2097  TP@1m  50.0%  only@2m  13.1%  missed  36.9%
  bikes       8940  TP@1m  36.5%  only@2m   1.6%  missed  61.9%
  other        429  TP@1m   0.0%  only@2m   0.0%  missed 100.0%

unmatched predictions at 1 m: 4435
  near persons     1165: offset 233, piece 495, duplicate 1, near 236, ghost 200
  near vehicles     967: offset 319, piece 389, duplicate 2, near 257, ghost 0
  near bikes        127: offset 30, piece 15, duplicate 0, near 2, ghost 80
  near other          3: offset 0, piece 0, duplicate 0, near 3, ghost 0
  near none        2173: offset 0, piece 0, duplicate 18, near 33, ghost 2122
===== QB r1-r3, frames all
runs: ['bags/coda/runs/QB_v2_coda{c}', 'bags/coda/runs/QBr2_v2_coda{c}', 'bags/coda/runs/QBr3_v2_coda{c}']  windows: ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']  frames: all

GT rows by class group (TP at the 1 m gate / found only within 2 m / missed):
  persons    26772  TP@1m  82.1%  only@2m   0.4%  missed  17.4%
  vehicles    2097  TP@1m  52.2%  only@2m  14.0%  missed  33.8%
  bikes       8940  TP@1m  37.6%  only@2m   1.5%  missed  60.9%
  other        429  TP@1m   0.0%  only@2m   0.0%  missed 100.0%

unmatched predictions at 1 m: 4530
  near persons     1154: offset 222, piece 465, duplicate 1, near 243, ghost 223
  near vehicles    1102: offset 348, piece 484, duplicate 0, near 270, ghost 0
  near bikes        151: offset 34, piece 37, duplicate 0, near 3, ghost 77
  near none        2123: offset 0, piece 0, duplicate 14, near 44, ghost 2065
===== QS r1-r3, frames all
runs: ['bags/coda/runs/QS_v2_coda{c}', 'bags/coda/runs/QSr2_v2_coda{c}', 'bags/coda/runs/QSr3_v2_coda{c}']  windows: ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']  frames: all

GT rows by class group (TP at the 1 m gate / found only within 2 m / missed):
  persons    26772  TP@1m  81.6%  only@2m   0.3%  missed  18.1%
  vehicles    2097  TP@1m  54.0%  only@2m  13.9%  missed  32.1%
  bikes       8940  TP@1m  37.3%  only@2m   1.5%  missed  61.3%
  other        429  TP@1m   0.0%  only@2m   0.0%  missed 100.0%

unmatched predictions at 1 m: 4713
  near persons     1204: offset 194, piece 505, duplicate 4, near 283, ghost 218
  near vehicles    1171: offset 363, piece 580, duplicate 1, near 227, ghost 0
  near bikes        133: offset 21, piece 40, duplicate 2, near 5, ghost 65
  near other          3: offset 0, piece 0, duplicate 0, near 3, ghost 0
  near none        2202: offset 0, piece 0, duplicate 22, near 38, ghost 2142
===== Q0 r1-r3, frames testing
runs: ['bags/coda/runs/Q0_v2_coda{c}', 'bags/coda/runs/Q0r2_v2_coda{c}', 'bags/coda/runs/Q0r3_v2_coda{c}']  windows: ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']  frames: testing

GT rows by class group (TP at the 1 m gate / found only within 2 m / missed):
  persons     3612  TP@1m  81.7%  only@2m   0.4%  missed  17.9%
  vehicles     306  TP@1m  44.8%  only@2m  11.8%  missed  43.5%
  bikes       1239  TP@1m  29.0%  only@2m   1.0%  missed  70.1%
  other         69  TP@1m   0.0%  only@2m   0.0%  missed 100.0%

unmatched predictions at 1 m: 599
  near persons      142: offset 21, piece 75, duplicate 0, near 28, ghost 18
  near vehicles     124: offset 42, piece 46, duplicate 0, near 36, ghost 0
  near bikes         38: offset 5, piece 10, duplicate 0, near 1, ghost 22
  near none         295: offset 0, piece 0, duplicate 0, near 5, ghost 290
===== QC r1-r3, frames testing
runs: ['bags/coda/runs/QC_v2_coda{c}', 'bags/coda/runs/QCr2_v2_coda{c}', 'bags/coda/runs/QCr3_v2_coda{c}']  windows: ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']  frames: testing

GT rows by class group (TP at the 1 m gate / found only within 2 m / missed):
  persons     3612  TP@1m  82.4%  only@2m   0.5%  missed  17.1%
  vehicles     306  TP@1m  54.6%  only@2m  12.7%  missed  32.7%
  bikes       1239  TP@1m  31.0%  only@2m   1.5%  missed  67.6%
  other         69  TP@1m   0.0%  only@2m   0.0%  missed 100.0%

unmatched predictions at 1 m: 673
  near persons      191: offset 32, piece 102, duplicate 0, near 29, ghost 28
  near vehicles     128: offset 44, piece 49, duplicate 0, near 35, ghost 0
  near bikes         26: offset 6, piece 1, duplicate 0, near 0, ghost 19
  near none         328: offset 0, piece 0, duplicate 2, near 3, ghost 323
===== QB r1-r3, frames testing
runs: ['bags/coda/runs/QB_v2_coda{c}', 'bags/coda/runs/QBr2_v2_coda{c}', 'bags/coda/runs/QBr3_v2_coda{c}']  windows: ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']  frames: testing

GT rows by class group (TP at the 1 m gate / found only within 2 m / missed):
  persons     3612  TP@1m  82.8%  only@2m   0.6%  missed  16.7%
  vehicles     306  TP@1m  58.2%  only@2m  11.4%  missed  30.4%
  bikes       1239  TP@1m  33.1%  only@2m   1.3%  missed  65.6%
  other         69  TP@1m   0.0%  only@2m   0.0%  missed 100.0%

unmatched predictions at 1 m: 659
  near persons      182: offset 33, piece 84, duplicate 0, near 30, ghost 35
  near vehicles     148: offset 43, piece 65, duplicate 0, near 40, ghost 0
  near bikes         32: offset 6, piece 7, duplicate 0, near 0, ghost 19
  near none         297: offset 0, piece 0, duplicate 0, near 5, ghost 292
===== QS r1-r3, frames testing
runs: ['bags/coda/runs/QS_v2_coda{c}', 'bags/coda/runs/QSr2_v2_coda{c}', 'bags/coda/runs/QSr3_v2_coda{c}']  windows: ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']  frames: testing

GT rows by class group (TP at the 1 m gate / found only within 2 m / missed):
  persons     3612  TP@1m  82.3%  only@2m   0.4%  missed  17.3%
  vehicles     306  TP@1m  57.2%  only@2m  12.4%  missed  30.4%
  bikes       1239  TP@1m  33.1%  only@2m   1.3%  missed  65.6%
  other         69  TP@1m   0.0%  only@2m   0.0%  missed 100.0%

unmatched predictions at 1 m: 700
  near persons      181: offset 27, piece 95, duplicate 0, near 32, ghost 27
  near vehicles     155: offset 51, piece 72, duplicate 0, near 32, ghost 0
  near bikes         24: offset 4, piece 5, duplicate 0, near 0, ghost 15
  near none         340: offset 0, piece 0, duplicate 2, near 3, ghost 335
```

## 17. R1 check (r1check.sh, 19/11 22:14)
sins c2-shape-assoc d38adae built OK. R1 = coda_vjsye2i2pcar1 (R0 + track_person_ray_offset 0 + random_seed 1 in
preprocessing and static_structures; installed cfg checked), 1 run on CODa w1 (tag R1C): e3 MOTA all 62.3 (moving 75.3),
latency 77 / 176 ms. Q0 r1-r3 on w1: 61.6 / 61.6 / 61.0 -> pass range 59.0-63.6: PASS. Chain B can start from this branch.

## 18. Chain B on R1 (20/11, 22:18 -> 06:23)

Arms on R1 (= R0 + ray_offset 0 + random_seed 1, sins c2-shape-assoc): CB0 = R1, CBW = walls off + bus size cap (r1wb),
CBN = splitter off (r1n), CBT = outdoor roof 4.5 (r1t). x3 CODa + street, lab LCB0/LCBN/LCBB x3.

CODa testing MOTA all, seq 0 / 10 / 16 (mean of 3 reps; per-rep range):
- CB0 63.5 [62.5-65.5] / 49.9 [49.8-50.2] / 56.3 [53.4-59.4]
- CBW 58.8 / 51.8 / 54.0  -> seq 0 -4.7: FAIL
- CBN 50.3 / 56.4 / 60.2  -> seq 0 -13.2 (+6.5 on 10, +3.9 on 16): FAIL
- CBT 62.4 / 51.2 / 52.3  -> seq 0 -1.1, seq 16 -4.0: FAIL
- ref Q0 (R0, chain 19/11) 60.8 / 47.5 / 54.6: R1 above R0 on all 3 seqs (different chain, same machine)
All frames: CB0 63.5/50.3/60.3, CBW 60.5/52.5/59.0, CBN 52.9/54.7/62.6, CBT 61.5/51.2/57.2.
Moving vehicles (all frames): CB0 19.2, CBW 33.9, CBN 38.4, CBT 33.9. Latency CODa med/p90: CB0 96/125, CBW 102/175, CBN 75/101, CBT 123/174 ms.

Street MOTA (15 m): R1 52.7 [52.2-53.1], wb 52.2, n 49.2 (vehicle recall 44.7 vs 50.7), t 53.0, R0 pcaq 52.5. Latency 6-7 ms med.
Lab (summary / dynamic only MOTA, a/b/c): LCB0 95.5/93.8/94.1 | 86.4/87.4/88.1; LCBN 96.2/94.0/92.9 | 87.7/86.0/86.1;
LCBB 94.8/94.1/93.1 | 89.1/87.7/89.4. No evident regression.

err_decomp (eval/diagnostics/err_decomp_chainb.txt, all frames): CBN raises vehicle TP@1m 39.1 -> 53.8% and cuts unmatched
3638 -> 3167, but persons TP 82.0 -> 79.5% and seq 0 collapses (likely: splitter off merges people in the crowded seq 0; not verified).
Ghosts with no GT near: CB0 1702, CBW 1466, CBN 1801, CBT ~similar.

Decision (pre-registered rule): no arm adopted. Reference becomes R1 (CB0). CBN is the interesting signal for vehicles
(seq 10/16 and moving vehicles) but must not apply to people: candidate for chain A (splitter off only for vehicle-class
clusters), to be named by the peer. e3ib column follows when after_chainb_ib.sh finishes.

### 18b. e3ib column (e3 + --ignore-box), 07:55

CODa testing MOTA all seq 0/10/16, e3 -> e3ib: CB0 63.5/49.9/56.3 -> 63.5/52.6/56.3; CBW 58.8/51.8/54.0 -> 58.8/54.0/54.0;
CBN 50.3/56.4/60.2 -> 50.4/58.6/60.2; CBT 62.4/51.2/52.3 -> 63.4/54.2/52.3; Q0 60.8/47.5/54.6 -> 61.5/50.4/54.6.
Dynablox -286.6/-92.2/30.1 -> -271.1/-91.6/30.1 (11: 17.0 -> 21.4, 12: -181.3 -> -169.1); M-detector -435.3/-63.3/-16.9 -> -434.9/-62.1/-16.9.
--ignore-box helps mostly seq 10 (+2-3 for all SINS arms); ranking and rule outcome unchanged (no arm adopted).
Street MOTA all (full, not the 15 m moving column of agg_e3): e3ib = e3 within 0.2 for every run.
Street MOTA all per rep: R1 43.0/42.3/41.8 (mean 42.4), R0 pcaq 42.0/44.4/44.6 (43.7), wb 44.1/45.9/44.8 (45.0),
n 38.4/39.7/38.9, t 40.8/40.7/43.6. Note: on this column R1 is 1.3 below R0 (ranges overlap); CBW is the best street arm.
Baseline files in coda_e3ib are named dynablox_*/mdetector_*; DBX_*/MDET_* symlinks added for cmp_e3.py.

## 19. User requirement: no distance window on YOLO mask points (20/11, 09:10)

The user wants no distance filtering of the points inside YOLO masks. R1 still used the mode + valley depth window capped at +-0.3 m
(yolo_leaf_depth_tolerance), so yolo_leaf_connected becomes mandatory. The user keeps yolo_leaf_nearest_cluster (with masks,
raycasting leaves no ambiguity between two masks in the same box). On 19/11, connected on R0 gave testing 51.0/46.6/55.0 vs
60.8/47.5/54.6, so seq 0 is the problem to understand.
Running r1lc_chain.sh (d708aff): RB0 (R1) vs RC0 (R1 + connected) x3 alternated on CODa + street (pcar1k vs pcar1lc) + lab
(LRB0/LRC0), then e3, e3ib, vehicles, and err_decomp incl. seq 0 only -> eval/diagnostics/err_decomp_r1lc.txt.

## 20. Disk full, cleanup, R1 vs r1lc on CODa, pooled columns, chain A started (20/11)

- ~10:30 the lab PC disk filled (another user's GLIM/SIT job in docker, ~2 GB / 30 s; not touched). r1lc_chain and the queued
  r1lc_nf_chain were stopped. Its CODa part (RB0/RC0 x3, 09:10-10:22) had finished and is valid (same bag sizes/durations as
  chain B); the street runs after 10:42 failed and were marked .done without bags.
- Cleanup with the user's OK (in this session): output bags of Q0/QC/QS x3 and CBT/CBW x3 (CODa) and street pcar1t/pcar1wb x3,
  after checking all their e3/e3 test/e3ib/e3ib test JSONs exist; the failed street pcar1k/pcar1lc markers. Kept: QB, CB0, CBN, logs.
- JSONs pushed (aad7574): coda, coda_e3[/test], coda_e3ib[/test], coda_vehicle_e3, mdp_e3, mdp_e3ib, lab_chainb for Q*, CB*, RB0/RC0.
- pooled9.py: pooled MOTA-all over the 9 windows per rep, paired gain vs reference rep range.

Pooled MOTA all (9 windows), mean [range], paired gain vs ref; e3 all | e3 testing:
- chain B, ref CB0 57.1 [56.5-58.1] | 54.8 [53.7-56.1]: CBW -0.3 | -1.0; CBN +1.3 | +2.6 (> spread 2.5 on testing only); CBT -1.5 | -1.6
- LEAFC, ref Q0 55.5 [54.8-56.4] | 52.7 [51.5-54.3]: QC -0.5 | -1.5; QB +0.2 | -0.8; QS -0.8 | -1.6
- R1 vs r1lc (same chain today), ref RB0 55.8 [54.8-57.3] | 53.4 [52.5-55.1]: RC0 +0.8 | +0.9 (e3ib +1.8 | +2.1), all <= spread
Old verdicts in the pooled form: no arm passes on both testing and all (CBN passes testing only); unchanged.
Per seq testing (e3): RB0 57.3/47.4/57.0, RC0 60.2/45.8/59.4. On R1, connected does NOT lose seq 0 (+2.9); seq 10 -1.6.
RB0 (R1, today) is below CB0 (R1, last night) on seq 0 (57.3 vs 63.5, same profile/build): between-chain variance is large.

Chain A started 11:10 (a30dd66, chainA.sh): AL0 r1lc / AB0 r1lcb / AN0 r1lcbn / AM0 r1lcbm / AK0 r1lcbk, x3 alternated CODa +
street + lab (LAL0..LAK0, lab_chaina), then e3, e3ib, vehicles, err_decomp -> eval/diagnostics/err_decomp_chainA.txt. Disk guard
kills the chain below 3 GB free. Afterwards: r1lc_nf (lcn, lcf), then the ks switches.

## 21. Chain A (20/11, 11:10 -> 22:30), all on R1 + connected

Pooled MOTA all over 9 windows, mean [range], paired gain vs AL0 (r1lc); e3 all | e3 testing (AL0 spread 4.7 | 4.9: rep r1 low):
- AL0 r1lc 56.7 [54.2-58.9] | 54.9 [52.3-57.2]
- AB0 + bus cap: -0.2 | -1.1
- AN0 + bus cap + splitter off: +2.0 | +0.8
- AM0 + bus cap + refine merge fragments: +1.8 | +1.9 (the most consistent: testing 57.0/56.9/56.4, range 0.6)
- AK0 + bus cap + shape association: +1.4 | +0.6
e3ib: AN0 +1.3 | -0.2, AM0 +1.0 | +1.1, AK0 +1.0 | +0.2, AB0 -0.5 | -1.4. No arm passes the pooled rule (gain > ref spread).
Per seq testing MOTA all (e3): AL0 59.5/50.2/57.4, AB0 62.2/49.2/55.2, AN0 48.4/54.6/58.7 (seq 0 -11 again: veto),
AM0 64.8/54.5/56.3, AK0 58.5/49.0/59.8. Moving vehicles (all): AL0 32.1, AB0 30.4, AN0 44.8, AM0 33.8, AK0 31.4.
Latency CODa med/p90: 100/152, AN0 78/103, others ~100/140-154.
Street, 15 m moving MOTA: lc 52.4, lcb 51.7, lcbn 49.7, lcbm 52.0, lcbk 50.5; full MOTA all lc 41.6, lcb 40.9, lcbn 41.9,
lcbm 41.5, lcbk 37.2 (rep 31.7). Lab MOTA a/b/c: LAL0 94.4/89.6/93.5, LAB0 91.4/95.6/94.3, LAN0 92.5/96.3/94.4,
LAM0 96.4/94.5/94.9, LAK0 94.3/92.2/92.4: no evident regression.
Reading: nothing passes; AM0 (merge fragments) is the best candidate (gain on all/testing, low spread, seq 0 and 10 up,
street and lab fine); AN0 helps vehicles and seq 10/16 but breaks seq 0 as before, also with the bus cap.
Next: r1lc_nf2 (RC1/RN0/RF0/RCC) started after A on c5-semantic-birth.

## 22. nf2: removing the depth window from the mask->box fallback (20/11 22:30 -> 21/11 00:54, CODa only)

sins c5-semantic-birth 2aff69c, all on R1 + connected. RC1 = lc (fallback with depth window), RN0 = nearest_cluster off,
RF0 = fallback off, RCC = fallback by 3D connectivity (no depth window anywhere).
Pooled MOTA all (9 windows), paired gain vs RC1; e3 all | testing (RC1 55.7 [53.4-57.1] | 53.5 [51.7-55.0], spread 3.8 | 3.3):
- RN0 +1.1 | +0.7;  RF0 +1.9 | +2.2;  RCC +2.4 (+3.9 +1.1 +2.4) | +2.4 (+2.7 -0.2 +4.6). e3ib: RCC +2.6 | +2.5, RF0 +1.8 | +2.2.
Per seq testing (e3): RC1 55.6/45.8/58.9, RN0 56.8/47.1/58.9, RF0 60.1/50.0/59.0, RCC 60.9/50.4/58.7.
Moving vehicles: RC1 42.0, RN0 46.4, RF0 41.6, RCC 45.6. Latency 100/125 ms for all.
No arm passes the pooled rule (gain <= spread), but per the 20/11 decision the base must be a no-window variant: RCC (lcc) is
the best on every column and becomes the base for the next chains.
fallback_connected counter (det.log, last periodic line; destructor "final" lines not in det.log): about 200-350 box fallbacks
per window on seq 10 / 0_w1 with ~20-40% getting a leaf, < 50 on 0_w2/0_w3/16_w1; 0.01-0.04 ms per fallback.
-> eval/diagnostics/fallback_connected_counts.txt.
DISK_ABORT at 00:54 when the street copy left 2 GB free: street (lcr/lcn/lcf/lcc) and lab not run. Partial dir
mdp_street_vjsye2i2pcar1lcr_r1 left behind (cleanup needs the user's OK).

## 23. Chain D on lcc (23/11, 12:26 -> 20:55) + videos

sins c5-semantic-birth cd01b90. DC0 = r1lcc (no depth window anywhere) / DM0 = + bus cap + merge fragments / DS0 = DM0 +
split_masked / DQ0 = DM0 + person-size confirm 3. Other users' jobs (glim, FIDO) loaded the PC during the chain: latency as
arm - DC0 only.
Pooled MOTA all (9 windows), paired gain vs DC0; e3 all | testing (DC0 57.5 [56.3-58.1] | 55.5 [54.1-56.5], spread 1.82 | 2.42):
- DM0 +1.75 (+2.6 +0.9 +1.7) | +2.50 (+3.4 +1.8 +2.4): passes on testing (> 2.42), just short on all (1.75 vs 1.82).
  e3ib: +1.7 > 1.3 | +2.5 > 2.0, passes both. Per seq testing DM0 59.6/53.4/61.1 vs DC0 54.7/52.5/58.0: all 3 up, no veto.
- DS0 -0.1 | -0.3;  DQ0 -0.4 | -0.7 (seq 0 testing 47.8 vs 54.7: veto).
Street 15 m moving MOTA: lcc 51.5 [49.6-52.6], lccbm 53.1 [51.6-54.0], lccbms 51.7, lccbmq 51.2; full MOTA all lcc 40.9,
lccbm 42.5, lccbms 42.5, lccbmq 39.7. Latency vs DC0: CODa med +0 for all (p90 +0/+2/+0), street lccbm +2 ms med.
Lab MOTA a/b/c: LDC0 95.8/93.7/87.9, LDM0 92.4/84.1/94.4 (b: pot recall 70.2), LDS0 94.7/94.4/95.4, LDQ0 95.6/93.4/93.9.
Lab: one low rep each for LDC0 and LDM0; DM0 mean 90.3 vs DC0 92.5, not a clear regression but to recheck.
ID switches per class (idsw_class.py, mean per rep, all/moving, all frames): persons DC0 73/62, DM0 85/72, DS0 80/68,
DQ0 105/87; vehicles 31/17, 34/15, 24/10, 24/16; bikes 67/1, 58/2, 67/1, 54/1. Testing: persons 35/31, 36/33, 33/30, 39/35.
DS0's 0_w1 IDSW drop was parked bikes and does not hold over 9 windows; DQ0 adds person ID switches.
err_decomp_chainD.txt: persons TP@1m DC0 81.2, DM0 82.0; vehicles 55.8 / 54.2; unmatched near vehicles "piece" 549 -> 302
with DM0 (merge fragments works on vehicle fragments).
split_masked N/M: not available (log every 50 rejoins, fewer per window); the counter is every 100 fusions from 70002e2 on.
Reading: DM0 is the chain D winner (passes testing and e3ib, just short on e3 all; street better; lab to recheck).
Videos (DM0 rep 1, EagerMOT on the right): NAS datasets_SINS/derived/videos/chainD/chainD_DM0_coda{0_w1,10_w4,16_w1}.mp4.
make_coda_video.py: CSV baselines (coda_eval.CSV_METHODS) and stereo camera (seq 16 has no front camera); bag path tested.

## 24. Lab recheck DC0 vs DM0 (23/11 22:00-22:46, same build cd01b90, x3 more: LDC1/LDM1, eval/results/lab_chaind2)

6 reps per arm, summary MOTA: LDC 95.8/93.7/87.9/96.6/90.6/93.9 (mean 93.1); LDM 92.4/84.1/94.4/94.4/92.5/92.9 (mean 91.8,
93.3 without the one failed sequence). Dynamic-only MOTA mean: DC 88.3, DM 89.2. PD recall mean: DC 87.7, DM 85.2.
Per sequence (6 reps): 1942 all 97-98.5 except LDM0b 10.1 (not reproduced in LDM1). 1947 DC 85.1 (bimodal 68.7-95.7) vs DM 91.9;
1950 DC 91.4 (4 reps ~95, 2 reps ~84) vs DM 84.2 (all 6 reps 79-86, FN ~75): a consistent DM loss on 1950, compensated on 1947.
LDM0b 1942 cause: NOT a merge. During the 34.8-54.6 s gap the person (standing still, LiDAR-only "L" detections, no YOLO
class) is detected every frame as a 0.55 x 0.23 x 1.22 m box at the right place; the tracker demoted track_30 from
potentially_dynamic to static at ~37 s and the object stopped being published, until a new track (345) at 54.6 s. A
still, unclassified person decaying to static: a one-off (not in the other 5 DM runs on 1942).
Reading: no clear lab regression for DM0 overall (mean -1.3 incl. the one-off, dynamic-only +0.9); seq 1950 is worth a look
(DM misses ~75 GT rows in every rep).

## 25. Offline analysis on the new base DM0 (user decision 23/11: r1lccbm is the base)

1) Lab seq 1950, DC vs DM, 6 reps each (eval/diagnostics/lab_fn_seq.py): person FN by cause
   (merged into a bigger box / box that exists only thanks to the bus cap / held static / not published / no detection).
   - merged 0 and bus-cap 0 in all 12 runs (1 merged row in LDC0a). Every FN has a person-size detection at the right place;
     the losses are all in the tracker (held static + not published).
   - FN by time: about 15 rows at 0-2 s (start-up) in every run, plus one event at 50-58 s worth about 55 rows. That event
     happens in 6 of 6 DM runs and in 2 of 6 DC runs (LDC0c, LDC1c).
   - Mechanism (LDM0a vs LDC0a, the run bags): at about 52.7 s the person crouches. The detected height drops from 1.7 to
     0.7-0.9 m (then back to 1.3 m), at the same position (< 0.1 m off). In the good runs the existing track (D) continues as
     PD. In the bad runs it is replaced by a new track (track_834 / track_865), which is born or quickly held static and is
     not published until about 58 s.
   - Examples (s from seq start): 52.7, 54.5, 56.5. Not a merge_fragments or bus-cap effect on the boxes; DM makes the
     re-birth on a height change systematic.
2) CODa person-loss funnel + person IDSW causes (eval/diagnostics/coda_person_funnel.py; person_funnel_chainD.txt).
   - Method: an approximation of the laptop funnel from the bags (no track_debug_csv); coasting is inferred from cluster absence.
   - Moving persons, all frames, 8637 rows, mean of 3 reps, DC0 -> DM0: lost 16.2 -> 15.4%.
     - held static by the motion rules 6.1 -> 5.3
     - no cluster (detection) 3.6 -> 3.6
     - published but given to another GT (crowd) 2.7 -> 2.8
     - eval range boundary 1.7 -> 1.6: the GT just inside 1.2 / 20 m, the prediction just outside; a metric effect, for all methods
     - coasting next to an unassociated cluster 0.8 -> 0.9; cluster with no track 0.7 -> 0.7
     - unconfirmed 0.2 -> 0.2; eval ignore zone 0.3 -> 0.3
   - Testing, moving: 16.7 -> 15.6% (held static 6.6 -> 5.2).
   - Person ID switches, per rep, all frames: re-birth DC0 53 -> DM0 50. A pre-existing track taking over the person:
     DC0 14 -> DM0 27 (doubled), plus 6 -> 6 taken over from another GT object. Testing: takeover 12 -> 15, re-birth 19 -> 17.
   Reading: merge_fragments slightly reduces "held static" (5.3 vs 6.1) and doubles the takeovers of a person by a
   pre-existing track (a merged fragment's old track inherits the person).

## 26. Chain E: static anchor (23/11 23:16 -> 24/11 04:55)

Base E0 = r1lccbm vs EB = r1lccbma (track_static_anchor_radius 0.5, sins c5-semantic-birth c15a337, v2: rejects only candidates
claimed by a moving track within 1 m). A first attempt with v1 (0736dee, tag EA) was clearly worse on 0_w1 (MOTA 48.0 vs 57.4,
FP/min 382 vs 248, counter 75k) and was stopped at the start of an EA run; EA runs kept on disk but discarded. E0 w1/r2 w1 reused.
CODa pooled MOTA all (9 windows), paired gain vs E0; e3 all | testing (E0 58.3 [56.8-60.0] | 55.8 [54.1-57.6], spread 3.3 | 3.5):
- EB +1.2 (+1.6 +1.0 +1.0) | +1.2 (+1.8 +0.8 +1.1): positive in every rep, below the spread (E0's reps vary 3.3). e3ib +1.1 | +1.0.
Per seq testing: E0 59.5/48.0/60.9, EB 60.3/51.9/60.1 (seq 16 -0.8, within E0's range: no veto). Moving vehicles 35.4 -> 42.1.
Street 15 m moving MOTA: lccbmr (E0 copy) 53.0, lccbma 52.9; full MOTA all 42.9 vs 42.4 (ranges overlap). Latency +0 / -1 ms.
Lab, 6 reps per arm (lab_chaine + lab_chaine2), pooled MOTA per rep: E0 94.4/95.6/91.9/87.2/92.1/96.0 (mean 92.9);
EA arm (= EB code) 94.7/96.2/94.0/96.1/96.3/96.1 (mean 95.6). Seq 1950: E0 87.6 (2 of 6 good) vs 91.4 (4 of 6 good);
seq 1947: E0 87.9 (one 66.2) vs 95.8 (all 95-96.5): the anchor removes the lab's bimodal losses.
Parked-then-moving risk: moving-vehicle TP per rep 122 -> 145, moving-vehicle IDSW 16 -> 15: no sign of it.
Person IDSW per rep (all frames, all/moving): E0 87/72, EB 88/68; testing 45/38 vs 39/34. Takeovers by a pre-existing track:
E0 18/35/30, EB 47/19/36 (mostly D person tracks, as in chain D): the anchor does not change the CODa crowd swaps.
Person funnel, moving: lost E0 15.4% vs EB 15.4%. Anchor counter: 2-18k candidate pairs rejected per window (not tens).
Reading: EB is consistently better (+1.2 in all 3 CODa reps, lab +2.7 and much more stable) but does not pass the pooled rule
because E0's own spread is 3.3. Decision for the user/laptop.

## 27. Crowd-swap diagnosis (24/11, offline; user adopted EB = r1lccbma as the base)

eval/diagnostics/crowd_absorb.py, crowd_absorb_chainE.txt. Per D/PD takeover A -> B (B a pre-existing D/PD track, B on the
tracker topic at the tick before), B's position in the 5 ticks before vs fused detections in the same run and in DC0
(r1lcc, no merge_fragments) at the same stamps; the person's own cluster excluded.
- 53 takeovers over E0 + EB (6 runs): B had its own separate cluster within 0.5 m in the last 2 ticks in 42 (79%).
- "absorbed" (B's position inside a neighbour's YOLO-refined F box, no own cluster): 0.
- B already on the person's cluster (no separate one) in 11; in 10 of those DC0 had a separate cluster at B's position.
- In all 10 "yes" cases B never had its own camera leaf (an F detection) in the 2 s before: a YOLO 2D-instance-id rule
  cannot separate them.
- The YOLO 2D instance id is not carried downstream: it is Detection2D.id on /yolo_seg_track/*/detections (e.g. '1'). Fused
  detections and /sins/tracker/tracks carry only id (fused_N / track_N) and results (label or state, class).
Reading: the merge_fragments absorption hypothesis does not hold for most crowd swaps; about 19% look like two persons in one
cluster that DC0 kept apart, and those have no camera instance. Crowd swaps go to after the freeze (association).
Held-static gate per track: not observable from the bags (needs track_debug_csv: a diagnostic run).

## 28. Held-static moving persons on EB: which motion gate (24/11, diagnostic run HDBG = r1lccbmadbg, 9 windows x1)

hdbg.sh (track_debug_csv per window, bags/coda/runs/HDBG_trk_<c>.csv), held_static_gates.py -> held_static_gates_EB.txt.
Moving person GT rows 8617; paired with a confirmed track within 1 m: 8491. Paired with a dynamic track: 7282.
Held (non-dynamic for >= 3 ticks): 719 rows, 8.3% of the moving rows.
- Status of the held track: static 47%, missed (coasting, not published) 41%, PD 13%.
- Out of camera FOV 75%; no class 83% (person class 17%). The persons really walk: GT speed median 1.29 m/s (p25 1.02, p75 1.55).
- Failing gates (share of held rows): point vote 50%, disocclusion 20%, consistency window 13%, size 7%, KF confidence 4%,
  speed 3%, net footprint from birth 3%.
- No gate failing: 24%. These are in the debounce: confirm_ticks 5, dyn_ticks median 2.
- Top combinations: vote alone 221, none 173, disocc 76, vote+disocc 57, cons 54, size 34 (all PD).
Reading: the point-vote evidence is the main gate holding walking, unclassified, out-of-FOV persons. Then debounce, then
disocclusion. The KF speed itself is fine.

## 29. Held-static follow-up: motion evidence and the point-to-surface veto (HDBG, held_static_veto.py -> held_static_veto_EB.txt)

The veto hypothesis (vetoed = pm_frac in [0, 0.5) on an unclassified track) does NOT hold.
Held walking persons (719 rows):
- motion_ev = 0 only 19%; vetoed 3% overall (18% of the motion_ev = 0 rows).
- pm_frac p10/50/90: 0.88 / 1.00 / 1.00; no baseline (pm_frac < 0) 31%.
- pm_n median 31, vs 94 for correctly dynamic walking persons. pm_dist about 0.6 m and pm_dt 0.6 s in both sets.
Correctly dynamic walking persons (7075 rows): motion_ev = 0 1%, pm_frac p10 0.97.
By range (held / dynamic):
- motion_ev = 0: < 10 m 27% / 1%, 10-15 m 12% / 1%, 15-20 m 14% / 1%.
- vetoed: 7%, 1%, 0% in the three ranges.
- pm_n held: 145, 25, 16.
Held rows WITH motion evidence: 579 (81%).
- Status: static 262, missed (coasting, unpublished) 250, PD 67.
- dyn_ticks below the 5-tick confirmation: 64%; dyn_ticks p10/50/90 0/3/82. Disocclusion in 23%.
- "No gate failing" rows (170): dyn_ticks p10/50/90 1/4/52.
Held episodes: 132. The dyn_ticks streak resets in 14% of them (0.16 resets per episode); at the reset ticks cons_ok 16,
vote 16, kf_conf 5, disocc 4.
Reading: most held walking persons already have motion evidence. They are lost to:
- the 5-tick confirmation not yet reached (short tracks, re-births);
- coasting tracks that are not published (missed, with dyn_ticks high in the p90 tail);
- disocclusion (about 1/4).
The veto and the point vote are minor.

## 30. Held-static: re-birth donors vs long tracks blocked by disocclusion (HDBG, held_donors.py -> held_donors_EB.txt)

Held rows with motion_ev = 1 and dyn_ticks < 5: 369.
- On tracks younger than 1 s: 24%; younger than 2 s: 37%. So most are NOT re-births.
- Young heir tracks: 31.
  - Donor within 0.6 m / 0.5 s: 3. All 3 donors never published dynamic, so inherit_motion correctly does nothing.
  - Donor within 1.5 m / 1.0 s: 8, all never published dynamic.
  - Farther: 15 (5 ever published dynamic).
  - No donor: 5.
  - Donor age at birth p50 0.60 / p90 1.65 s. Distance to the donor's KF position p50 1.00 / p90 2.17 m; to its last
    observation 1.48 / 3.51 m. Donor max dyn_ticks p50 0 / p90 13.
  - So these are chains of short tracks that were never dynamic. Widening inheritance would not help.
- Older tracks (>= 2 s, 231 rows on 18 tracks, age p50 4.9 s): status static 149, missed 74; dyn_ticks 0 in 120 rows.
  - Matched rows are mostly disocc = 1, often with large_travel = 1 (the track has already travelled far), out of FOV and
    unclassified (43 + 30 + 25 rows).
  - So these are long-lived walking tracks held static by the disocclusion block despite a large travel.
Reading: the clearest physical rule is that disocclusion must not block a track whose own net travel is already large (it
is the object moving, not a newly revealed background). Re-birth inheritance is not the lever.

## 31a. Seq 10 diagnosis on EB (24/11, offline, partial; chain F still running)
- Gap to EMMC on seq 10 (errors per rep, e3 all): +712 errors in total (15.5 points). By window:
  - 10_w1: +416 (vehicles ~285)
  - 10_w4: ~+540 (persons ~330, vehicles ~190)
  - 10_w5: +48
  - 10_w3: about 0
  - 10_w2: **-272**. SINS wins here: EMMC has 879 FP from pieces and duplicates. But SINS misses 65% of persons, mostly "held static" (39% of rows).
- Vehicles (new eval/diagnostics/coda_vehicle_funnel.py, output vehicle_funnel_seq10_EB.txt; 541 rows/rep): TP 55-60%.
  - Vehicles >= 6 m (Delivery Truck): 52-56% "unconfirmed". There is a fused cluster with about 3.5 fragments, and a non-static track, but nothing is published.
  - Side/back vehicles out of the camera FOV: 72-80% "held static" in r1/r3.
  - Offset 1-2 m: 13-18%, mostly still vehicles under 6 m. The centre sits short, towards the sensor, by -0.3 m median.
  - Vehicle FP: 226-303 per rep, made of pieces and offsets.
- Next: why the large trucks are not published (confirmation/size gate?). Persons in 10_w4: "taken by other GT" 7% (crowd).
