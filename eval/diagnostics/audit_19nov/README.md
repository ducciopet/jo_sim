# Audit 19/11: which parameters/functions keep SINS from tracking D and PD (CODa, reference R0)
Laptop diagnostic runs (not timed results): R0 + per-track CSV (coda_vjsye2i2pcadbg, tag ADT, 9 tuning windows) and
R0 + lifecycle log + wall markers recorded (coda_vjsye2i2pcadbl, tag ADL, w1 10_w1 10_w4 16_w1).
Funnel (eval/diagnostics/dpd_study/funnel.py, rows_coda_ADT.csv), grouped by class with funnel_groups.py.

Share of annotated rows handled correctly (OK = output within 1 m with the right state):
moving: persons 82%, vehicles 39%, bikes/scooters 12%; still: vehicles 17%, bikes/scooters 23%, persons 2% (3 people).

Findings (impact order):
1. Leaf depth slice +-0.3 m (yolo_leaf_depth_tolerance): vehicles cut to ~0.6 m; leaf/cluster IoU < 0.12 -> "no candidate"
   (55% of car/truck leaves); "too low for its class" 26% of car leaves. Fix in test (leafc chain QC).
2. Cluster size cap 6 m / 40 m3 drops 9% of vehicle rows. In test (QB).
3. Walls: 33% of vehicle rows have a static-structures wall centred on the vehicle (pickup 35%, service vehicle 74%,
   delivery truck 22%, cars 0%: wall2d_min_vertical_extent 1.8 m); points within 0.15 m of a wall are stripped.
4. Fragmentation: 64% of vehicle rows reach the tracker as >= 2 detections, 48% with a piece < 0.7 m tall
   (plateau splitter split_* + refine keep_rest).
5. Still objects re-born: per window 13-33 new tracks per vehicle, 9-12 per bike, 2-5 per person. Refusal of the
   nearest track: another track took it 22-35%, velocity-direction gate 23-26%, top-height diff 15-19%, speed gate 11-15%
   (centre jumps of fragments read as motion). track_large_object_footprint (relaxed gates for large objects) is off (-1).
6. Out of camera after being seen: 73-82% of PD losses are a NEW track without class (not the same track losing it):
   still vehicles OK 16% out of FOV vs 39% in FOV; bikes 23% vs 34%.
7. Class cleared after 3 in-FOV ticks without YOLO (track_max_non_yolo_in_fov_frames) + growth trigger (5 ticks).
8. Unclassified movers need person size to become D (size gate: 6% of moving-vehicle rows); moving vehicles held PD 13%
   (corroboration). Bikes beyond 15 m: 53% not detected at all (thin, sensor limit).

## Code check 19/11 (R0 = coda_vjsye2i2pca on sins main cec8e5e)
Switched OFF in R0 (dead code for the reference; keep only what a chain will test, declare or remove the rest before
paper-v1): tracker 25 (disocclusion variants x5, travel_proof x6, motion_group, fragment_merge, absorb_pieces,
inherit_motion_size_check, large_object_footprint, publish_coasting_ticks, decay_fresh_frames_only, ...), dbscan 9
(yolo_refine_nested, merge_fragments, fragment_merge_*, occlusion_edge_check, late_leaves_motion_comp, ...).
ON in R0 and against what SINS must do (track D and PD of every movable class, one behaviour indoor/outdoor):
- track_dynamic_constrain_size + target 0.5x0.5x1.7: unclassified movers must be person-sized to become D.
- track_classified_corroboration_all: classified movers need KF confidence / point vote -> 13% of moving vehicles held PD.
- track_max_non_yolo_in_fov_frames 3 (0.3 s) and the growth trigger (5 ticks): class cleared on short YOLO gaps.
- track_publish_coasting_ticks 0 + coasting only for tracks published DYNAMIC: a still classified object (PD) is never
  published on a missed tick (coasting = 8-20% of PD losses) although it cannot have gone anywhere.
- association hard gates on the box centre (speed 8 / 4 m/s, veldir, top height 1.0 m, size): fragments of a still object
  read as motion -> re-births.
- split_enabled (plateau splitter) and yolo_refine_keep_rest: vehicles in pieces, low pieces become extra objects.
- yolo_class_min_height car 1.1 m: rejects 26% of car leaves (the matched object is a low piece / slice).
- static_structures walls (vertical extent >= 1.8 m, length >= 2 m) on tall parked vehicles, points stripped.
- track_person_box_completion with track_person_ray_offset 0.07 m measured on the lab mocap bags: a value fitted on the
  Qualisys sequences; declare it in the paper or set it to 0.
Kept on purpose (data says they help): lidar-fallback classification (majority of first classifications: persons 64 vs 29,
trucks 17 vs 6), disocclusion suppression (unclassified only), class anchors, inherited motion.
