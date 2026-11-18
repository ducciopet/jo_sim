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
