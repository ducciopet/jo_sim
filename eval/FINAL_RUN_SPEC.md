# FINAL_RUN_SPEC: what the final runs are, exactly (v1, 24/11/2026)

This is the companion of PIPELINE_FINAL.md (approved 24/11). It is written so that the lab PC and the external PC never
have to guess. The `[TBD]` items are filled before the freeze (05/12); after the freeze this file only changes by a
dated, reasoned erratum.

## 1. Frozen system
- **SINS** = tag `paper-v1` on sins (from branch `c5-semantic-birth`) and jo_sim, set at the freeze 05/12.
- **Method profile** = the CODa final profile (the chain winner on 04/12, e.g. `coda_vjsye2i2pcar1lccbma[+t][+e]`).
- **Every other dataset profile** = `base:` that CODa profile + SENSOR overrides only. Allowed keys:
  - camera lists: `camera_names` in every node, per-camera depth topic/transport/`depth_max_value`, `eps_scale`;
  - LiDAR geometry:
    - `lidar_range_x/y`;
    - `lidar_eps_vertical_angle_deg` = ring spacing;
    - `yolo_leaf_connect_radius_per_m` = 1.2 x ring spacing [rad];
    - the band voxel sizes only if the beam density differs (stated);
  - floor source options for cameras without a depth sensor (`floor_lowest_below_sensor`, as in JRDB);
  - frame and topic names.

  Any other change is a method change and is forbidden. Each profile file lists every override with its physical reason. Its md5 is recorded in `eval/results/final/PROFILES.md` BEFORE its first run.
- **One YOLO per camera:** yolo26s-seg (`weights/yolo26s-seg.pt`, md5 91df624c...), the same everywhere.

## 2. Datasets, sequences and roles

| Dataset | Role | Sequences / windows | Sensor profile (to create) | Notes |
|---|---|---|---|---|
| CODa new (2, 3, 18) | **UNSEEN, headline** | annotated spans of seq 2, 3, 18, cut into windows like the others (`[TBD]` after download: window list fixed from frame counts only, before any run) | = CODa profile (same sensors) | converted only after the freeze |
| KITTI tracking training | **UNSEEN, headline** | 21 sequences 0000-0020 (0001: 443 images vs 447 scans, kept, missing images = no camera input) | `kitti_tracking`: HDL-64 (ring spacing 0.4 deg), cam2 only, depth = scan projected online (`lidar_depth_node`) | converter `eval/datasets/kitti/kitti_to_bag.py`; evaluated in the camera view only (KITTI labels only image_02 objects) |
| DOALS | **UNSEEN, headline (LiDAR-only)** | 8 = 4 places x 2 sequences | `doals`: OS1-64, NO camera (LiDAR-only mode: no semantics, no PD by design, stated) | converter + evaluator `[TBD, waiting for the bag topics]` |
| CODa tuning | development | seq 0 w1-3, 10 w1-5, 16 w1 | CODa profile | x5 reps |
| CODa 11/12 | exposed (used 10-17/11) | 11 w1-3, 12 w1-3 | CODa profile | reported as "exposed held-out" |
| JRDB | exposed | held-out 6: forbes-cafe_0, huang-basement_0, packard-poster-session_2, tressider-2019-04-26_2, clark-center_0, clark-center_1; bytes-cafe_0 = debug, never scored | `jrdb`: = CODa profile + 5 cameras, upper VLP-16 (2 deg), 15 m crop, `floor_lowest_below_sensor` (replaces jrdb_indoor/outdoor, which derived from old bases) | 15 Hz |
| Street (Marina di Pisa) | NOT reported (user 25/11: no solid GT); used during development as a veto only | — | — | — |
| CROSSLAB multi-person (new) | `[TBD: unseen if recorded/processed after the freeze]` | several people walking/stopping around the robot; people AND robot with mocap helmets | lab profile (same robot/sensors) | mocap evaluator with N persons, in_fov from the camera pose |
| Lab Qualisys | development | the 6 sequences | lab profile (`lab_uca...` final) | x6 reps |

Repetitions: SINS x3 everywhere (CODa tuning x5, lab x6). Baselines: x3 if nondeterministic, x1 if deterministic.

## 3. Evaluation (same scripts for every method, run on the lab PC)

### 3.1 Box tracking: `eval/coda_eval.py` (CLEAR-MOT, 1 m centre gate)

| Dataset | Flags |
|---|---|
| CODa (all) | `--ignore-undefined` (e3, primary). Second column `--ignore-undefined --ignore-box` (e3ib, declared post hoc). `--frames all` and `--frames testing`; testing frames lie 100 ms from training frames (declared). |
| KITTI | `--ignore-undefined --camera-only --cam-json <seq>_cam.json --range 40` [TBD: range 40 m, fixed before running; KITTI labels reach ~70 m, and 40 m is the HDL-64 density limit used by most trackers] |
| JRDB | `--ignore-undefined --range 15` (its own GT, persons only) |
| Lab | the mocap evaluator (`eval/harness`, as for every lab run so far) |

Columns per table:
- MOTA (primary), IDF1, recall, precision, FP/min, IDSW;
- HOTA / DetA / AssA: computed inside `eval/harness/metrics.py` (TrackEval's HOTA algorithm on the same frames and objects as CLEAR-MOT, centre-distance similarity S = max(0, 1 - d / 1 m) instead of box IoU, alpha 0.05-0.95; pooled over windows from the per-alpha sums `hota_alpha` with `hota_from_sums`). Stated as "HOTA with a centre-distance similarity". KITTI additionally gets the official TrackEval HOTA (2D IoU) where the method outputs image boxes;
- moving-only and PD columns.
- Vehicles: also with a 2 m gate (sensitivity).

### 3.1b State metrics and PD views (review 25/11)
- State metrics (F1_D, F1_PD, PD P/R) are computed in a SEPARATE coda_eval run with `--state-buffer 0.5` (GT rows within
  0.5 s of a D<->PD transition ignored); sensitivity of the GT threshold `--dyn-speed 0.2 / 0.3 / 0.4`.
- Baselines without a motion state: `--baseline-state raw` and `--baseline-state debounced` (0.5 s speed + the SINS debounce),
  both reported; `none` (all dynamic) for MOTA-all.
- PD views: (A) in FOV (default); (B) `--pd-prev-observable` (still agents outside the FOV whose track was in a FOV or moving
  earlier); (C) conditional persistence survival curve (script `[TBD: eval/scripts/analysis/persistence.py]`, from the run's
  /sins/tracker/tracks classes, also for baselines with their own classes); (D) never observable: diagnostic only.

### 3.2 DOALS point IoU (the authors' metric)
- Per annotated scan (indices.csv: header stamp + indices of the dynamic points of the raw cloud), points with 0.5 <= range <= 20 m (Dynablox default.yaml `[TBD: confirm min_range]`):
  - TP/FP/FN of "dynamic";
  - IoU = TP / (TP+FP+FN);
  - mean over the 10 scans per sequence, then the mean ± std over all 80 (as `evaluate_data.py`).
- SINS and the other box methods: a point is "dynamic" if it lies inside an output box whose state is `dynamic`, at the output nearest in time (≤ 50 ms) to the scan.
  - Second column: `dynamic` + `potentially_dynamic`.
  - The GT labels persons by appearance (moving or not): stated.
- Dynablox / M-detector: their own point labels.

### 3.3 Statistics (PIPELINE_FINAL section 2)
- Adoption of a fix (development only): sign test on 27 window x rep pairs, ≥ 19/27 on testing and all frames, with vetoes.
- Claims vs a baseline:
  - paired bootstrap over windows (CODa, KITTI sequences, JRDB sequences, DOALS scans), 10,000 resamples, 95% CI of the pooled-MOTA difference (SINS mean of reps - baseline);
  - "better" only if the CI excludes 0;
  - sign test over windows x reps next to it;
  - per-sequence numbers always shown.
  - Script: `eval/scripts/analysis/bootstrap_claims.py` `[TBD, laptop, before the freeze]`.

## 4. Runtime
- Paper latency: on the LAPTOP after the freeze (user 24/11, the only exception to "no tests on the laptop"; the lab PC is shared and its load inflates latency): nothing else running, load average and CPU/GPU logged per run; hardware (CPU, GPU, RAM) stated in the paper. Frozen SINS on 1 CODa window, 1 KITTI sequence, 1 DOALS sequence, 1 JRDB sequence and 1 lab sequence, x3.
- Report per-scan latency med/p90 per node and end to end; dropped scans.
- Baselines: their own reported per-frame times plus ours (external PC: RTX 3060 laptop GPU), stated per hardware.

## 5. Storage, names, manifest
- **SINS outputs:** `NAS/datasets_SINS/derived/sins_final/<dataset>/<window_or_seq>_r<k>/` (run bag + logs). JSONs in `eval/results/final/<dataset>/<metric>/<TAG>_<window>.json`, committed to the repo.
  - Tags: `SF` (SINS final) + rep suffix (`SF`, `SFr2`, `SFr3`, ...); ablation `SFA_<fix>`.
- **Baselines:** `NAS/datasets_SINS/derived/baselines_ext/<method>/<dataset>/` (README format) plus `manifest.json`; scored by the lab PC into `eval/results/final/<dataset>/<metric>/<METHOD>[r<k>]_<window>.json`.
- **Per-run manifest** (`manifest.json` next to every SINS run):
  - sins + jo_sim commit, profile name + md5;
  - YOLO weights md5;
  - host, load average (1 min) at start and end, GPU;
  - dropped scans per node, replay rate;
  - date.
- **Rerun rule:** a run is rerun (once, identical settings) if dropped scans > 2% or the process crashed. Both runs are logged; the rerun counts.
- **Disk on the lab PC:**
  - stream one sequence at a time from the NAS into a pre-authorised scratch folder (named by the user in the lab session);
  - delete it after the outputs are on the NAS and verified;
  - 3 GB guard.

## 6. Leave-one-out ablation (after the headline runs; CODa tuning AND unseen CODa 2/3/18; x3)
Run on both data sets after the headline runs, so the unseen CODa ablation involves no selection (review 25/11). Each arm
flips ONE switch from the frozen profile (sins >= 4d7d5c3 for the ablation switches):

| Arm | Switch(es) |
|---|---|
| connected leaf off (old depth valley) | `yolo_leaf_connected: false`, `yolo_leaf_fallback_connected: false` |
| merge_fragments + bus cap off | `yolo_refine_merge_fragments: false`, `max_box_extent/volume` = base |
| static anchor off | `track_static_anchor_radius: -1` |
| leaf cut off (class only) | `yolo_refine_class_only: true` |
| semantics off (LiDAR-only) | run_*_lidaronly switches (camera_names lidar_only_none, YOLO off) |
| disocclusion check off | `track_disocclusion_min_cos_angle: 1.01` (never satisfied) |
| occlusion-explained misses off | `track_ablate_occlusion_explained: true` |
| out-of-FOV persistence off | `track_ablate_persistence: true` (plain missed budget; class decays everywhere) |
| semantic decay off | `track_max_non_yolo_in_fov_frames: 100000`, `track_max_non_yolo_fov_border_frames: 100000` |
| debounce/corroboration off | all `track_dynamic_confirm_ticks*: 1`, `track_motion_min_net_footprints: -1`, `track_motion_consistency_window: -1`, `track_point_shift_veto: false` |

Reported: paired differences vs SF (statistics of 3.3) on MOTA-all, F1_D, F1_PD, false/min, plus the PD views of 3.4.

## 7. Smoke tests before the freeze (lab PC, never scored)
One debug input each:
- JRDB bytes-cafe_0;
- KITTI tracking **testing** 0000 (unlabelled split: nothing to leak);
- DOALS hauptgebaeude/sequence_1. DOALS has no unlabelled sequence, so this one is checked for TF/frames/overlay only. Its score is NOT computed before the final run (stated).

Pass = the run finishes, TF resolves, outputs are in the odom frame and overlay the scan, latency is sane, no node crashes. Found problems are fixed in the converter/profile (sensor side) only.
